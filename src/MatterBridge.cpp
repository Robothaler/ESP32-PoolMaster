// =============================================================================
//  MatterBridge.cpp — PoolMaster Matter Bridge Implementation
// =============================================================================
//  Architecture:
//    • Matter Bridge (Aggregator, device type 0x000E) on Endpoint 1
//    • 5 child On/Off Plugin-Unit endpoints (device type 0x010A)
//    • Endpoints for Filtration, pH, Heat pumps are always reachable
//    • Endpoints for Salt electrolysis and Chlorine pump are conditionally
//      reachable (hidden when storage.Salt_Chlor == false)
//    • Power metering via ElectricalMeasurement cluster on Filt/Heat/Salt EPs
//
//  Thread safety:
//    • Matter attribute callbacks run on the CHIP task (Core 0)
//    • Pool control tasks run on Core 1
//    • Commands from Matter → pool: via existing queueIn (FreeRTOS-safe)
//    • Pool state → Matter attributes: via esp_matter::lock in MatterSyncTask
//
//  Race condition mitigations:
//    • xQueueSend() in attribute callback: ISR/cross-core safe
//    • Lock/unlock around all attribute::update() calls
//    • MatterSyncTask uses startTasks guard before any sync attempt
//    • Timeout on lock acquisition (100 ms) prevents indefinite blocking
// =============================================================================

#ifdef MATTER_ENABLED

#include "MatterBridge.h"
#include "Config.h"
#include "PoolMaster.h"

// PCF8574.h defines P0-P7 as integer pin-number macros (0-7), which conflict
// with function parameter names in CHIP crypto headers (CHIPCryptoPAL.h uses
// "P1", "P2" as parameter names). Undefine them before pulling in CHIP headers.
// MatterBridge.cpp does not use PCF pin macros directly.
#undef P0
#undef P1
#undef P2
#undef P3
#undef P4
#undef P5
#undef P6
#undef P7

// ── esp_matter headers ────────────────────────────────────────────────────────
#include <esp_matter.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_attribute.h>
#include <esp_matter_core.h>
#include <esp_matter_cluster.h>
#include <esp_matter_identify.h>

// ── CHIP / Matter stack headers ───────────────────────────────────────────────
#include <platform/CHIPDeviceLayer.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>

// ── CHIP cluster IDs (from Matter specification) ──────────────────────────────
#include <app/clusters/on-off-server/on-off-server.h>
// Use raw IDs to avoid include-path fragility across esp_matter versions.
static constexpr uint32_t kElecMeasClusterId  = 0x0B04; // ElectricalMeasurement
static constexpr uint32_t kActivePowerAttrId  = 0x050B; // ActivePower
static constexpr uint32_t kTempMeasClusterId  = 0x0402; // TemperatureMeasurement
static constexpr uint32_t kTempMeasAttrId     = 0x0000; // MeasuredValue (int16, 0.01°C)
static constexpr uint32_t kOnOffClusterId     = 0x0006; // OnOff
static constexpr uint32_t kOnOffAttrId        = 0x0000; // OnOff attribute
static constexpr uint32_t kBoolStateClusterId = 0x0045; // BooleanState
static constexpr uint32_t kBoolStateAttrId    = 0x0000; // StateValue

// ── Matter Controller headers (optional — requires CONFIG_ESP_MATTER_CONTROLLER_ENABLE) ──
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
#include <esp_matter_controller_cluster_command.h>
#include <esp_matter_controller_subscribe_command.h>
#include <Preferences.h>
#endif

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "MatterBridge";

// =============================================================================
//  Configuration constants (override in Config.h via -D flag if needed)
// =============================================================================
#ifndef MATTER_VENDOR_ID
  #define MATTER_VENDOR_ID      0xFFF1   // Test vendor (replace with real VID for production)
#endif
#ifndef MATTER_PRODUCT_ID
  #define MATTER_PRODUCT_ID     0xCA01   // PoolMaster Bridge product ID
#endif
#ifndef MATTER_DISCRIMINATOR
  #define MATTER_DISCRIMINATOR  3840     // BLE commissioning discriminator (0–4095)
#endif
#ifndef MATTER_PASSCODE
  #define MATTER_PASSCODE       20202021 // Setup passcode (replace in production!)
#endif
#ifndef MATTER_DEVICE_NAME
  #define MATTER_DEVICE_NAME    "PoolMaster Bridge"
#endif

// Sync task period: how often pool state is pushed to Matter attributes
#ifndef MATTER_SYNC_PERIOD_MS
  #define MATTER_SYNC_PERIOD_MS 5000
#endif

// Timeout for acquiring the CHIP stack lock in the sync task
#define MATTER_LOCK_TIMEOUT_MS  100

// =============================================================================
//  Module-private state
// =============================================================================
static node_t   *s_node     = nullptr;
static bool      s_started  = false;

// Pump/electrolysis endpoint IDs (bridged, always/conditionally reachable)
static uint16_t s_ep_filt = chip::kInvalidEndpointId;
static uint16_t s_ep_ph   = chip::kInvalidEndpointId;
static uint16_t s_ep_heat = chip::kInvalidEndpointId;
static uint16_t s_ep_salt = chip::kInvalidEndpointId;
static uint16_t s_ep_chl  = chip::kInvalidEndpointId;

// Last-known reachability for conditional endpoints (Salt/Chl)
static bool s_salt_chl_was_active = false;

// Cached commissioning info — written once in matterBridgeStart(), read-only afterwards
static char s_qr_code[96]      = {};
static char s_pairing_code[32] = {};

// Native (non-bridged) endpoints exposed for SolarControl to subscribe to
static uint16_t s_ep_pool_temp  = chip::kInvalidEndpointId; // Pool water temperature
static uint16_t s_ep_pool_soll  = chip::kInvalidEndpointId; // Pool target temperature
static uint16_t s_ep_solar_mode = chip::kInvalidEndpointId; // Solar-mode request (OnOff)

// Last synced values — used to avoid redundant attribute writes
static float s_last_pool_temp  = -999.0f;
static float s_last_pool_soll  = -999.0f;
static bool  s_last_solar_mode = false;

#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
// SolarControl Matter configuration — loaded from NVS, set via SET_SOLAR_NODE
static uint64_t s_solar_node_id    = 0;       // SolarControl NodeId
static uint16_t s_solar_ep_pump    = 5;       // SolarControl pump status EP
static uint16_t s_solar_ep_valve   = 6;       // SolarControl valve position EP
static uint16_t s_solar_ep_circ    = 7;       // SolarControl circulation command EP
static uint16_t s_solar_ep_illum   = 8;       // SolarControl illumination command EP
static bool     s_solar_subscribed = false;   // Subscription active
#endif

// =============================================================================
//  Internal helpers
// =============================================================================

/**
 * @brief Send a JSON command string to the existing MQTT/Matter command queue.
 *        Called from the CHIP task (Core 0) — xQueueSend is cross-core safe.
 */
static void queueCommand(const char *json)
{
    // Guard: if pool tasks haven't started yet, drop the command
    if (!startTasks) {
        ESP_LOGW(TAG, "Pool tasks not started; dropping Matter command: %s", json);
        return;
    }
    if (xQueueSend(queueIn, json, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queueIn full — dropped Matter command: %s", json);
    } else {
        ESP_LOGD(TAG, "Queued Matter command: %s", json);
    }
}

/**
 * @brief Update an on/off attribute while holding the CHIP stack lock.
 *        Safe to call from any FreeRTOS task.
 */
static void updateOnOff(uint16_t ep_id, bool state)
{
    if (ep_id == chip::kInvalidEndpointId) return;
    if (esp_matter::lock::chip_stack_lock(pdMS_TO_TICKS(MATTER_LOCK_TIMEOUT_MS)) != ESP_OK) return;

    esp_matter_attr_val_t val = esp_matter_bool(state);
    attribute::update(ep_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val);

    esp_matter::lock::chip_stack_unlock();
}

/**
 * @brief Update the ActivePower attribute (int16, unit: Watt) while holding
 *        the CHIP stack lock.  Set to 0 when pump is stopped.
 */
static void updateActivePower(uint16_t ep_id, int16_t power_w)
{
    if (ep_id == chip::kInvalidEndpointId) return;
    if (esp_matter::lock::chip_stack_lock(pdMS_TO_TICKS(MATTER_LOCK_TIMEOUT_MS)) != ESP_OK) return;

    // ElectricalMeasurement cluster (0x0B04), ActivePower attribute (0x050B), unit: 1 W
    esp_matter_attr_val_t val = esp_matter_int16(power_w);
    attribute::update(ep_id, kElecMeasClusterId, kActivePowerAttrId, &val);

    esp_matter::lock::chip_stack_unlock();
}

/**
 * @brief Set the "reachable" flag on a bridged endpoint's
 *        BridgedDeviceBasicInformation cluster.
 */
static void setReachable(uint16_t ep_id, bool reachable)
{
    if (ep_id == chip::kInvalidEndpointId) return;
    if (esp_matter::lock::chip_stack_lock(pdMS_TO_TICKS(MATTER_LOCK_TIMEOUT_MS)) != ESP_OK) return;

    esp_matter_attr_val_t val = esp_matter_bool(reachable);
    attribute::update(ep_id,
                      BridgedDeviceBasicInformation::Id,
                      BridgedDeviceBasicInformation::Attributes::Reachable::Id,
                      &val);

    esp_matter::lock::chip_stack_unlock();
}

/**
 * @brief Update a TemperatureMeasurement::MeasuredValue attribute.
 *        Value is in °C; stored as int16 * 100 per Matter spec (0.01 °C units).
 */
static void updateTemperature(uint16_t ep_id, float temp_c)
{
    if (ep_id == chip::kInvalidEndpointId) return;
    if (esp_matter::lock::chip_stack_lock(pdMS_TO_TICKS(MATTER_LOCK_TIMEOUT_MS)) != ESP_OK) return;

    const int16_t val_i16 = static_cast<int16_t>(temp_c * 100.0f);
    esp_matter_attr_val_t val = esp_matter_int16(val_i16);
    attribute::update(ep_id, kTempMeasClusterId, kTempMeasAttrId, &val);

    esp_matter::lock::chip_stack_unlock();
}

// =============================================================================
//  SolarControl Matter Controller — subscription callback + helpers
// =============================================================================
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE

/**
 * @brief Attribute report callback for SolarControl subscriptions.
 *        Decodes TLV values and writes them into storage.solarXxx fields.
 *        Called on the CHIP task (Core 0) — storage writes are atomic for
 *        the scalar types used here (float, bool), no mutex needed.
 */
static void solarReportCallback(uint64_t /*remote_node_id*/,
                                 const chip::app::ConcreteDataAttributePath &path,
                                 chip::TLV::TLVReader *data)
{
    if (!data) return;

    const uint16_t ep  = path.mEndpointId;
    const uint32_t cid = path.mClusterId;
    const uint32_t aid = path.mAttributeId;

    // ── TemperatureMeasurement::MeasuredValue (int16, 0.01°C) ────────────────
    if (cid == kTempMeasClusterId && aid == kTempMeasAttrId) {
        int16_t raw = 0;
        if (data->Get(raw) == CHIP_NO_ERROR) {
            const float temp = raw / 100.0f;
            if      (ep == 1) { storage.solarRoofTemp    = temp; }
            else if (ep == 2) { storage.solarBoilerTemp  = temp; }
            else if (ep == 3) { storage.solarStorageTemp = temp; }
            else if (ep == 4) { storage.solarBackflowTemp = temp; }
            ESP_LOGD(TAG, "SolarControl EP%u temp: %.2f°C", ep, temp);
        }
    }
    // ── OnOff::OnOff (bool) ───────────────────────────────────────────────────
    else if (cid == kOnOffClusterId && aid == kOnOffAttrId) {
        bool val = false;
        if (data->Get(val) == CHIP_NO_ERROR) {
            if      (ep == s_solar_ep_pump)  { storage.solarPumpRunning = val; }
            else if (ep == s_solar_ep_valve) { storage.solarValvePool   = val; }
            ESP_LOGD(TAG, "SolarControl EP%u OnOff: %d", ep, (int)val);
        }
    }
    // ── BooleanState::StateValue (bool) ──────────────────────────────────────
    else if (cid == kBoolStateClusterId && aid == kBoolStateAttrId) {
        bool val = false;
        if (data->Get(val) == CHIP_NO_ERROR) {
            if (ep == 9) { storage.solarValveOK = val; }
            ESP_LOGD(TAG, "SolarControl EP%u BoolState: %d", ep, (int)val);
        }
    }
}

/**
 * @brief Load SolarControl Matter config from NVS.
 *        Called during matterBridgeInit().
 */
static void loadSolarConfig()
{
    Preferences nvs;
    if (!nvs.begin("PoolMaster", true)) {
        ESP_LOGW(TAG, "NVS open failed — using default solar config");
        return;
    }
    uint64_t nodeId = nvs.getULong64(NVS_KEY_SOLAR_NODE_ID, 0);
    if (nodeId != 0) {
        s_solar_node_id  = nodeId;
        s_solar_ep_pump  = nvs.getUShort(NVS_KEY_SOLAR_EP_PUMP,  5);
        s_solar_ep_valve = nvs.getUShort(NVS_KEY_SOLAR_EP_VALVE, 6);
        s_solar_ep_circ  = nvs.getUShort(NVS_KEY_SOLAR_EP_CIRC,  7);
        s_solar_ep_illum = nvs.getUShort(NVS_KEY_SOLAR_EP_ILLUM, 8);
        ESP_LOGI(TAG, "Solar config loaded: NodeId=0x%016llX pump=%u valve=%u circ=%u illum=%u",
                 s_solar_node_id, s_solar_ep_pump, s_solar_ep_valve,
                 s_solar_ep_circ, s_solar_ep_illum);
    } else {
        ESP_LOGI(TAG, "No SolarControl NodeId in NVS — use SET_SOLAR_NODE to configure");
    }
    nvs.end();
}

/**
 * @brief Save SolarControl Matter config to NVS.
 *        Called after SET_SOLAR_NODE serial command.
 */
static void saveSolarConfig()
{
    Preferences nvs;
    if (!nvs.begin("PoolMaster", false)) {
        ESP_LOGE(TAG, "NVS open for write failed");
        return;
    }
    nvs.putULong64(NVS_KEY_SOLAR_NODE_ID, s_solar_node_id);
    nvs.putUShort(NVS_KEY_SOLAR_EP_PUMP,  s_solar_ep_pump);
    nvs.putUShort(NVS_KEY_SOLAR_EP_VALVE, s_solar_ep_valve);
    nvs.putUShort(NVS_KEY_SOLAR_EP_CIRC,  s_solar_ep_circ);
    nvs.putUShort(NVS_KEY_SOLAR_EP_ILLUM, s_solar_ep_illum);
    nvs.end();
    ESP_LOGI(TAG, "Solar config saved to NVS");
}

#endif // CONFIG_ESP_MATTER_CONTROLLER_ENABLE

// =============================================================================
//  Matter callback: attribute update from controller
// =============================================================================
/**
 * @brief Called by the Matter stack (on the CHIP task) when a controller
 *        changes an attribute (e.g. turns a pump on/off via the Home app).
 *
 *  Only PRE_UPDATE is handled — we intercept the OnOff attribute changes and
 *  convert them to pool control commands in queueIn.  The actual pump state
 *  is reflected back to Matter on the next MatterSyncTask cycle (≤ 5 s), which
 *  is the canonical source of truth.
 */
static esp_err_t on_attribute_update(
    attribute::callback_type_t type,
    uint16_t endpoint_id,
    uint32_t cluster_id,
    uint32_t attribute_id,
    esp_matter_attr_val_t *val,
    void *priv_data)
{
    // Only act on PRE_UPDATE for the OnOff attribute
    if (type != PRE_UPDATE) return ESP_OK;
    if (cluster_id  != OnOff::Id)                   return ESP_OK;
    if (attribute_id != OnOff::Attributes::OnOff::Id) return ESP_OK;

    const bool on = val->val.b;
    char cmd[QUEUE_ITEM_SIZE] = {};

    // ── Map endpoint_id → JSON command key ───────────────────────────────────
    if (endpoint_id == s_ep_filt) {
        // FiltPump: 0 = stop, 1 = start
        snprintf(cmd, sizeof(cmd), "{\"FiltPump\":%d}", on ? 1 : 0);
    }
    else if (endpoint_id == s_ep_ph) {
        // PhPump: 0 = stop, 1 = start (interlocked with filtration in PoolServer)
        snprintf(cmd, sizeof(cmd), "{\"PhPump\":%d}", on ? 1 : 0);
    }
    else if (endpoint_id == s_ep_heat) {
        // HeatPump: 0 = stop, 1 = start
        snprintf(cmd, sizeof(cmd), "{\"HeatPump\":%d}", on ? 1 : 0);
    }
    else if (endpoint_id == s_ep_salt && storage.Salt_Chlor) {
        // SaltPump: only when salt/chlor mode is active
        snprintf(cmd, sizeof(cmd), "{\"SaltPump\":%d}", on ? 1 : 0);
    }
    else if (endpoint_id == s_ep_chl && storage.Salt_Chlor) {
        // ChlPump: only when salt/chlor mode is active
        snprintf(cmd, sizeof(cmd), "{\"ChlPump\":%d}", on ? 1 : 0);
    }
    else {
        // Unknown endpoint — ignore
        return ESP_OK;
    }

    queueCommand(cmd);
    return ESP_OK;
}

// =============================================================================
//  Matter callback: identification (e.g. "find my device" effect)
// =============================================================================
static esp_err_t on_identification(
    identification::callback_type_t type,
    uint16_t endpoint_id,
    uint8_t  effect_id,
    uint8_t  effect_variant,
    void    *priv_data)
{
    ESP_LOGI(TAG, "Identify: ep=%u effect=%u variant=%u", endpoint_id, effect_id, effect_variant);
    // Optional: flash buzzer or LED briefly to identify the device
    return ESP_OK;
}

// =============================================================================
//  Matter callback: platform/commissioning events
// =============================================================================
static void on_device_event(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Matter commissioning complete!");
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
            // Automatically subscribe to SolarControl if already configured
            if (s_solar_node_id != 0 && !s_solar_subscribed) {
                subscribeToSolarControl(s_solar_node_id);
            }
#endif
            break;
        case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
            ESP_LOGI(TAG, "Matter internet connectivity changed");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            ESP_LOGW(TAG, "Matter fabric removed");
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
            s_solar_subscribed = false;
#endif
            break;
        default:
            break;
    }
}

// =============================================================================
//  Endpoint factory: create a bridged On/Off Plugin Unit endpoint
// =============================================================================
/**
 * @param node        Root Matter node
 * @param label       Human-readable device name (max 32 chars for BridgedDeviceBasicInfo)
 * @param withPower   If true, add the ElectricalMeasurement cluster (power metering)
 * @returns           Pointer to created endpoint, or nullptr on failure
 */
static endpoint_t *createPumpEndpoint(node_t *node, const char *label, bool withPower)
{
    // ── On/Off Plugin Unit (device type 0x010A) with BRIDGE flag ─────────────
    endpoint::on_off_plugin_unit::config_t ep_cfg;
    memset(&ep_cfg, 0, sizeof(ep_cfg));
    ep_cfg.on_off.on_off             = false;   // Start in OFF state

    endpoint_t *ep = endpoint::on_off_plugin_unit::create(
        node, &ep_cfg,
        ENDPOINT_FLAG_BRIDGE | ENDPOINT_FLAG_DESTROYABLE,
        nullptr);

    if (!ep) {
        ESP_LOGE(TAG, "Failed to create endpoint for '%s'", label);
        return nullptr;
    }

    // ── BridgedDeviceBasicInformation cluster ─────────────────────────────────
    // Required on every bridged endpoint; provides name and reachability.
    // Note: config_t only has 'reachable'; node_label is set via attribute API.
    cluster::bridged_device_basic_information::config_t bi_cfg;
    bi_cfg.reachable = true;

    cluster_t *bi_cluster = cluster::bridged_device_basic_information::create(
        ep, &bi_cfg, CLUSTER_FLAG_SERVER);
    if (!bi_cluster) {
        ESP_LOGW(TAG, "BridgedDeviceBasicInfo cluster creation failed for '%s'", label);
    }

    // ── Optional: ElectricalMeasurement cluster (raw API — no typed config_t) ──
    // Cluster 0x0B04, ActivePower attribute 0x050B (int16, signed, in 0.1 W units)
    if (withPower) {
        cluster_t *em_cluster = cluster::create(ep, kElecMeasClusterId, CLUSTER_FLAG_SERVER);
        if (em_cluster) {
            // FeatureMap (0xFFFC): bit 3 (0x08) = AC power measurement
            attribute::create(em_cluster, 0xFFFC,
                              ATTRIBUTE_FLAG_NONE, esp_matter_bitmap32(0x08));
            // ActivePower attribute (0x050B) — initial 0 W
            attribute::create(em_cluster, kActivePowerAttrId,
                              ATTRIBUTE_FLAG_NONE, esp_matter_int16(0));
        } else {
            ESP_LOGW(TAG, "ElectricalMeasurement cluster creation failed for '%s'", label);
        }
    }

    return ep;
}

// =============================================================================
//  Public: Phase-1 initialisation
// =============================================================================
void matterBridgeInit()
{
    ESP_LOGI(TAG, "Initialising Matter Bridge (Phase 1 — node + endpoint creation)...");

    // ── Matter node configuration ──────────────────────────────────────────────
    // vendor_id and product_id are set via sdkconfig (CONFIG_DEVICE_VENDOR_ID /
    // CONFIG_DEVICE_PRODUCT_ID) or baked into the DAC — not via config_t here.
    node::config_t node_cfg;
    memset(&node_cfg, 0, sizeof(node_cfg));
    strncpy(node_cfg.root_node.basic_information.node_label,
            MATTER_DEVICE_NAME,
            sizeof(node_cfg.root_node.basic_information.node_label) - 1);

    // ── Create root node ───────────────────────────────────────────────────────
    s_node = node::create(&node_cfg, on_attribute_update, on_identification);
    if (!s_node) {
        ESP_LOGE(TAG, "FATAL: Failed to create Matter node!");
        return;
    }

    // ── Create Aggregator endpoint (bridge root, device type 0x000E) ───────────
    // All ENDPOINT_FLAG_BRIDGE child endpoints are automatically grouped under it.
    endpoint::aggregator::config_t agg_cfg;
    memset(&agg_cfg, 0, sizeof(agg_cfg));
    endpoint_t *aggregator = endpoint::aggregator::create(s_node, &agg_cfg, ENDPOINT_FLAG_NONE, nullptr);
    if (!aggregator) {
        ESP_LOGE(TAG, "FATAL: Failed to create Aggregator endpoint!");
        return;
    }
    ESP_LOGI(TAG, "Aggregator endpoint id=%u", endpoint::get_id(aggregator));

    // ── Create pump child endpoints (bridged) ─────────────────────────────────
    endpoint_t *ep_filt = createPumpEndpoint(s_node, "Filterpumpe",    true);   // with power
    endpoint_t *ep_ph   = createPumpEndpoint(s_node, "PH-Pumpe",       false);  // no power meter
    endpoint_t *ep_heat = createPumpEndpoint(s_node, "Waermepumpe",    true);   // with power
    endpoint_t *ep_salt = createPumpEndpoint(s_node, "Salzelektrolyse",true);   // with power, conditional
    endpoint_t *ep_chl  = createPumpEndpoint(s_node, "Chlor-Pumpe",   false);  // no power, conditional

    // ── Store pump endpoint IDs ────────────────────────────────────────────────
    if (ep_filt) s_ep_filt = endpoint::get_id(ep_filt);
    if (ep_ph)   s_ep_ph   = endpoint::get_id(ep_ph);
    if (ep_heat) s_ep_heat = endpoint::get_id(ep_heat);
    if (ep_salt) s_ep_salt = endpoint::get_id(ep_salt);
    if (ep_chl)  s_ep_chl  = endpoint::get_id(ep_chl);

    ESP_LOGI(TAG, "Pump EPs — filt:%u  ph:%u  heat:%u  salt:%u  chl:%u",
             s_ep_filt, s_ep_ph, s_ep_heat, s_ep_salt, s_ep_chl);

    // ── Create native temperature-sensor endpoints (read by SolarControl) ─────
    {
        endpoint::temperature_sensor::config_t cfgPoolTemp;
        memset(&cfgPoolTemp, 0, sizeof(cfgPoolTemp));
        endpoint_t *ep = endpoint::temperature_sensor::create(
            s_node, &cfgPoolTemp, ENDPOINT_FLAG_NONE, nullptr);
        if (ep) {
            s_ep_pool_temp = endpoint::get_id(ep);
            ESP_LOGI(TAG, "EP Pool-Temp: %u", s_ep_pool_temp);
        } else {
            ESP_LOGE(TAG, "Failed to create Pool-Temp endpoint");
        }
    }
    {
        endpoint::temperature_sensor::config_t cfgPoolSoll;
        memset(&cfgPoolSoll, 0, sizeof(cfgPoolSoll));
        endpoint_t *ep = endpoint::temperature_sensor::create(
            s_node, &cfgPoolSoll, ENDPOINT_FLAG_NONE, nullptr);
        if (ep) {
            s_ep_pool_soll = endpoint::get_id(ep);
            ESP_LOGI(TAG, "EP Pool-Soll: %u", s_ep_pool_soll);
        } else {
            ESP_LOGE(TAG, "Failed to create Pool-Soll endpoint");
        }
    }

    // ── Create Solar-Mode-Request OnOff endpoint (read by SolarControl) ───────
    {
        endpoint::on_off_plugin_unit::config_t cfgSolarMode;
        memset(&cfgSolarMode, 0, sizeof(cfgSolarMode));
        cfgSolarMode.on_off.on_off = false;
        endpoint_t *ep = endpoint::on_off_plugin_unit::create(
            s_node, &cfgSolarMode, ENDPOINT_FLAG_NONE, nullptr);
        if (ep) {
            s_ep_solar_mode = endpoint::get_id(ep);
            ESP_LOGI(TAG, "EP Solar-Mode-Request: %u", s_ep_solar_mode);
        } else {
            ESP_LOGE(TAG, "Failed to create Solar-Mode-Request endpoint");
        }
    }

    ESP_LOGI(TAG, "SolarControl EPs — pool_temp:%u  pool_soll:%u  solar_mode:%u",
             s_ep_pool_temp, s_ep_pool_soll, s_ep_solar_mode);
    ESP_LOGI(TAG, ">>> SET_POOL_NODE: use EP pool_temp=%u pool_soll=%u solar_mode=%u on SolarControl",
             s_ep_pool_temp, s_ep_pool_soll, s_ep_solar_mode);

#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
    // ── Load SolarControl config from NVS ─────────────────────────────────────
    loadSolarConfig();
#endif

    // ── Use example/test Device Attestation Credentials ───────────────────────
    // IMPORTANT: Replace with real DAC for production devices!
    chip::Credentials::SetDeviceAttestationCredentialsProvider(
        chip::Credentials::Examples::GetExampleDACProvider());

    ESP_LOGI(TAG, "Matter Bridge init complete (Phase 1).");
}

// =============================================================================
//  Public: Phase-2 start (call after WiFi is connected)
// =============================================================================
void matterBridgeStart()
{
    if (!s_node) {
        ESP_LOGE(TAG, "matterBridgeStart() called but node not created — did matterBridgeInit() run?");
        return;
    }

    ESP_LOGI(TAG, "Starting Matter stack (Phase 2)...");

    // Diagnostic: heap state before WiFi+BLE init inside esp_matter::start()
    ESP_LOGI(TAG, "HEAP before start: int_free=%u int_max=%u psram_free=%u",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    esp_err_t err = esp_matter::start(on_device_event);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() failed: %s", esp_err_to_name(err));
        return;
    }

    s_started = true;

    // Cache QR code and pairing code for WebUI (safe here — called from CHIP task context)
    {
        chip::MutableCharSpan qrSpan(s_qr_code, sizeof(s_qr_code) - 1);
        if (GetQRCode(qrSpan, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE)) == CHIP_NO_ERROR)
            s_qr_code[qrSpan.size()] = '\0';
        chip::MutableCharSpan codeSpan(s_pairing_code, sizeof(s_pairing_code) - 1);
        if (GetManualPairingCode(codeSpan, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE)) == CHIP_NO_ERROR)
            s_pairing_code[codeSpan.size()] = '\0';
    }

    // Print QR code + manual pairing code to Serial for commissioning
    PrintOnboardingCodes(
        chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));

    ESP_LOGI(TAG, "Matter Bridge started — waiting for commissioning.");

    // Apply initial reachability state for conditional endpoints
    matterUpdateConditionalEndpoints(storage.Salt_Chlor);
    s_salt_chl_was_active = storage.Salt_Chlor;
}

// =============================================================================
//  Public: Update conditional endpoint reachability
// =============================================================================
void matterUpdateConditionalEndpoints(bool active)
{
    if (!s_started) return;
    setReachable(s_ep_salt, active);
    setReachable(s_ep_chl,  active);
    ESP_LOGD(TAG, "Conditional endpoints (salt/chl) reachable=%s", active ? "true" : "false");
}

// =============================================================================
//  Public: Sync pool → Matter attributes
// =============================================================================
void matterBridgeSync()
{
    if (!s_started) return;

    // ── Filtration pump ────────────────────────────────────────────────────────
    {
        const bool running = FiltrationPump.IsRunning();
        updateOnOff(s_ep_filt, running);
        // Power: V × A, clamped to int16 range
        const int16_t pw = running
            ? static_cast<int16_t>(FILTER_VOLTAGE * storage.FilterCurrentValue)
            : 0;
        updateActivePower(s_ep_filt, pw);
    }

    // ── pH pump ───────────────────────────────────────────────────────────────
    updateOnOff(s_ep_ph, PhPump.IsRunning());

    // ── Heat pump ─────────────────────────────────────────────────────────────
    {
        const bool running = HeatPump.IsRunning();
        updateOnOff(s_ep_heat, running);
        const int16_t pw = running
            ? static_cast<int16_t>(HEAT_VOLTAGE * storage.HeatCurrentValue)
            : 0;
        updateActivePower(s_ep_heat, pw);
    }

    // ── Conditional: Salt electrolysis + Chlorine pump ────────────────────────
    {
        const bool active = storage.Salt_Chlor;

        // Sync reachability only when it actually changes (avoids redundant writes)
        if (active != s_salt_chl_was_active) {
            matterUpdateConditionalEndpoints(active);
            s_salt_chl_was_active = active;
        }

        if (active) {
            // Salt electrolysis
            const bool saltRunning = SaltPump.IsRunning();
            updateOnOff(s_ep_salt, saltRunning);
            const int16_t saltPw = saltRunning
                ? static_cast<int16_t>(ELECTROLYSIS_VOLTAGE * storage.SaltCurrentValue)
                : 0;
            updateActivePower(s_ep_salt, saltPw);

            // Chlorine pump
            updateOnOff(s_ep_chl, ChlPump.IsRunning());
        }
    }

    // ── Pool-Temp endpoint (TemperatureMeasurement) ───────────────────────────
    {
        const float poolTemp = static_cast<float>(storage.WaterSTemp);
        if (fabsf(poolTemp - s_last_pool_temp) >= 0.05f) {   // update on >0.05°C change
            updateTemperature(s_ep_pool_temp, poolTemp);
            s_last_pool_temp = poolTemp;
        }
    }

    // ── Pool-Soll endpoint (TemperatureMeasurement) ───────────────────────────
    {
        const float poolSoll = static_cast<float>(storage.WaterTemp_SetPoint);
        if (fabsf(poolSoll - s_last_pool_soll) >= 0.05f) {
            updateTemperature(s_ep_pool_soll, poolSoll);
            s_last_pool_soll = poolSoll;
        }
    }

    // ── Solar-Mode-Request endpoint (OnOff) ───────────────────────────────────
    // true  = PoolMaster requests solar pool heating
    //         conditions: AutoMode active AND PoolTemp < (Solltemp − Hysteresis)
    // false = no heating request (SolarControl heats boiler or stays idle)
    {
        const bool solarRequest = storage.AutoMode &&
            (storage.WaterSTemp < (storage.WaterTemp_SetPoint - SOLAR_MODE_HYSTERESIS));
        if (solarRequest != s_last_solar_mode) {
            updateOnOff(s_ep_solar_mode, solarRequest);
            s_last_solar_mode = solarRequest;
            ESP_LOGI(TAG, "Solar-Mode-Request → %s", solarRequest ? "ON (pool heating)" : "OFF");
        }
    }

    ESP_LOGV(TAG, "Matter sync complete");
}

// =============================================================================
//  Public: Subscribe to SolarControl attributes
// =============================================================================
void subscribeToSolarControl(uint64_t solarNodeId)
{
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
    using namespace esp_matter::controller;
    using chip::app::AttributePathParams;
    using chip::Platform::ScopedMemoryBufferWithSize;

    if (solarNodeId == 0) {
        ESP_LOGW(TAG, "subscribeToSolarControl: invalid NodeId 0 — skipping");
        return;
    }

    ESP_LOGI(TAG, "Subscribing to SolarControl NodeId=0x%016llX ...", solarNodeId);

    // Build 7 attribute paths covering all SolarControl data points:
    //   EP1-EP4: TemperatureMeasurement::MeasuredValue (temperatures)
    //   EP5:     OnOff::OnOff (pump running, EP from NVS)
    //   EP6:     OnOff::OnOff (valve position, EP from NVS)
    //   EP9:     BooleanState::StateValue (valve OK)
    ScopedMemoryBufferWithSize<AttributePathParams> attr_paths;
    ScopedMemoryBufferWithSize<chip::app::EventPathParams>  event_paths;

    attr_paths.Alloc(7);
    if (!attr_paths.Get()) {
        ESP_LOGE(TAG, "Failed to allocate attribute paths for SolarControl subscription");
        return;
    }

    // EP1-EP4: temperatures
    attr_paths[0] = AttributePathParams(1, kTempMeasClusterId, kTempMeasAttrId);
    attr_paths[1] = AttributePathParams(2, kTempMeasClusterId, kTempMeasAttrId);
    attr_paths[2] = AttributePathParams(3, kTempMeasClusterId, kTempMeasAttrId);
    attr_paths[3] = AttributePathParams(4, kTempMeasClusterId, kTempMeasAttrId);
    // EP5 / EP6: pump & valve (from NVS config)
    attr_paths[4] = AttributePathParams(s_solar_ep_pump,  kOnOffClusterId, kOnOffAttrId);
    attr_paths[5] = AttributePathParams(s_solar_ep_valve, kOnOffClusterId, kOnOffAttrId);
    // EP9: valve end-stop OK
    attr_paths[6] = AttributePathParams(9, kBoolStateClusterId, kBoolStateAttrId);

    // Allocate with new — subscription must outlive this function call.
    // auto_resubscribe=true ensures the subscription is re-established after loss.
    auto *sub = new subscribe_command(
        solarNodeId,
        std::move(attr_paths),
        std::move(event_paths),
        10,    // min report interval (s)
        60,    // max report interval (s)
        true,  // auto_resubscribe
        solarReportCallback,
        nullptr  // no event callback
    );

    if (sub->send_command() == ESP_OK) {
        s_solar_subscribed = true;
        s_solar_node_id    = solarNodeId;
        ESP_LOGI(TAG, "SolarControl subscription sent successfully");
    } else {
        ESP_LOGE(TAG, "SolarControl subscription failed");
        delete sub;
    }
#else
    ESP_LOGW(TAG, "subscribeToSolarControl: CONFIG_ESP_MATTER_CONTROLLER_ENABLE not set");
    (void)solarNodeId;
#endif
}

// =============================================================================
//  Public: Send OnOff commands to SolarControl
// =============================================================================
void sendCirculationCommand(bool on)
{
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
    if (s_solar_node_id == 0 || s_solar_ep_circ == 0) {
        ESP_LOGW(TAG, "sendCirculationCommand: SolarControl not configured");
        return;
    }
    const uint32_t cmdId = on
        ? chip::app::Clusters::OnOff::Commands::On::Id
        : chip::app::Clusters::OnOff::Commands::Off::Id;
    esp_err_t err = esp_matter::controller::send_invoke_cluster_command(
        s_solar_node_id, s_solar_ep_circ,
        chip::app::Clusters::OnOff::Id,
        cmdId, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sendCirculationCommand failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Circulation command → %s (EP%u)", on ? "ON" : "OFF", s_solar_ep_circ);
    }
#else
    ESP_LOGW(TAG, "sendCirculationCommand: CONFIG_ESP_MATTER_CONTROLLER_ENABLE not set");
    (void)on;
#endif
}

void sendIlluminationCommand(bool on)
{
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
    if (s_solar_node_id == 0 || s_solar_ep_illum == 0) {
        ESP_LOGW(TAG, "sendIlluminationCommand: SolarControl not configured");
        return;
    }
    const uint32_t cmdId = on
        ? chip::app::Clusters::OnOff::Commands::On::Id
        : chip::app::Clusters::OnOff::Commands::Off::Id;
    esp_err_t err = esp_matter::controller::send_invoke_cluster_command(
        s_solar_node_id, s_solar_ep_illum,
        chip::app::Clusters::OnOff::Id,
        cmdId, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sendIlluminationCommand failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Illumination command → %s (EP%u)", on ? "ON" : "OFF", s_solar_ep_illum);
    }
#else
    ESP_LOGW(TAG, "sendIlluminationCommand: CONFIG_ESP_MATTER_CONTROLLER_ENABLE not set");
    (void)on;
#endif
}

// =============================================================================
//  Serial command handler — called from MatterSyncTask
// =============================================================================
/**
 * @brief Non-blocking serial line reader + command dispatcher.
 *
 *  Supported commands:
 *    SET_SOLAR_NODE <nodeId_hex> <epPump> <epValve> <epCirc> <epIllum>
 *      — Stores the SolarControl NodeId and endpoint numbers to NVS,
 *        then immediately starts subscriptions.
 *      Example: SET_SOLAR_NODE 0000000000000002 5 6 7 8
 *
 *    GET_SOLAR_CONFIG
 *      — Prints current SolarControl NodeId and endpoint config to Serial.
 *
 *    GET_ENDPOINTS
 *      — Prints PoolMaster's new native EP IDs (pool_temp, pool_soll, solar_mode)
 *        for use in the SolarControl's SET_POOL_NODE command.
 */
static void handleSerialCommands()
{
    static char s_line_buf[96];
    static uint8_t s_line_len = 0;

    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;   // ignore CR
        if (c == '\n' || s_line_len >= sizeof(s_line_buf) - 1) {
            s_line_buf[s_line_len] = '\0';
            s_line_len = 0;

            // ── SET_SOLAR_NODE ──────────────────────────────────────────────
            if (strncmp(s_line_buf, "SET_SOLAR_NODE ", 15) == 0) {
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
                char hexId[20] = {};
                uint16_t epPump = 5, epValve = 6, epCirc = 7, epIllum = 8;
                int parsed = sscanf(s_line_buf + 15, "%19s %hu %hu %hu %hu",
                                    hexId, &epPump, &epValve, &epCirc, &epIllum);
                if (parsed >= 1) {
                    uint64_t nodeId = strtoull(hexId, nullptr, 16);
                    if (nodeId == 0) {
                        Serial.println("[Matter] ERROR: Invalid NodeId (0)");
                    } else {
                        s_solar_node_id  = nodeId;
                        s_solar_ep_pump  = epPump;
                        s_solar_ep_valve = epValve;
                        s_solar_ep_circ  = epCirc;
                        s_solar_ep_illum = epIllum;
                        saveSolarConfig();
                        Serial.printf("[Matter] SolarControl configured: NodeId=0x%016llX "
                                      "pump=%u valve=%u circ=%u illum=%u\r\n",
                                      nodeId, epPump, epValve, epCirc, epIllum);
                        // Start subscriptions immediately if already commissioned
                        if (s_started) {
                            s_solar_subscribed = false;
                            subscribeToSolarControl(s_solar_node_id);
                        }
                    }
                } else {
                    Serial.println("[Matter] Usage: SET_SOLAR_NODE <nodeId_hex> <epPump> <epValve> <epCirc> <epIllum>");
                }
#else
                Serial.println("[Matter] ERROR: CONFIG_ESP_MATTER_CONTROLLER_ENABLE not set");
#endif
            }
            // ── GET_SOLAR_CONFIG ────────────────────────────────────────────
            else if (strcmp(s_line_buf, "GET_SOLAR_CONFIG") == 0) {
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
                Serial.printf("[Matter] SolarControl NodeId=0x%016llX  pump=%u  valve=%u  circ=%u  illum=%u  subscribed=%d\r\n",
                              s_solar_node_id, s_solar_ep_pump, s_solar_ep_valve,
                              s_solar_ep_circ, s_solar_ep_illum, (int)s_solar_subscribed);
#else
                Serial.println("[Matter] CONFIG_ESP_MATTER_CONTROLLER_ENABLE not set");
#endif
            }
            // ── GET_ENDPOINTS ───────────────────────────────────────────────
            else if (strcmp(s_line_buf, "GET_ENDPOINTS") == 0) {
                Serial.printf("[Matter] PoolMaster EPs — pool_temp:%u  pool_soll:%u  solar_mode:%u\r\n",
                              s_ep_pool_temp, s_ep_pool_soll, s_ep_solar_mode);
                Serial.printf("[Matter] Pump EPs — filt:%u  ph:%u  heat:%u  salt:%u  chl:%u\r\n",
                              s_ep_filt, s_ep_ph, s_ep_heat, s_ep_salt, s_ep_chl);
            }
            // Unknown command (ignore silently — avoid noise from other serial traffic)
        } else {
            s_line_buf[s_line_len++] = c;
        }
    }
}

// =============================================================================
//  T14: MatterSyncTask — periodic state sync + serial command handler, Core 1
// =============================================================================
void MatterSyncTask(void *pvParameters)
{
    const TickType_t period   = pdMS_TO_TICKS(MATTER_SYNC_PERIOD_MS);
    TickType_t       lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        // Process serial commands (SET_SOLAR_NODE etc.)
        handleSerialCommands();

        if (startTasks && s_started) {
            matterBridgeSync();

#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
            // Start subscriptions once Matter is running and config is available
            if (s_solar_node_id != 0 && !s_solar_subscribed) {
                subscribeToSolarControl(s_solar_node_id);
            }
#endif
        }
    }
}

// =============================================================================
//  Public: Commissioning info helpers (for WebUI)
// =============================================================================

uint8_t matterFabricCount()
{
    // FabricCount is an atomic uint8 — safe to read without lock
    if (!s_started) return 0;
    return chip::Server::GetInstance().GetFabricTable().FabricCount();
}

bool matterGetQRCode(char* buf, size_t size)
{
    // Read from cache — written once during matterBridgeStart(), no lock needed
    if (!s_started || !buf || size < 2 || s_qr_code[0] == '\0') return false;
    snprintf(buf, size, "%s", s_qr_code);
    return true;
}

bool matterGetManualPairingCode(char* buf, size_t size)
{
    // Read from cache — written once during matterBridgeStart(), no lock needed
    if (!s_started || !buf || size < 2 || s_pairing_code[0] == '\0') return false;
    snprintf(buf, size, "%s", s_pairing_code);
    return true;
}

// ScheduleWork callback: opens commissioning window from within the CHIP task
static void openCommissioningWindowWork(intptr_t arg)
{
    uint16_t timeoutSec = static_cast<uint16_t>(arg);
    auto err = chip::Server::GetInstance().GetCommissioningWindowManager()
        .OpenBasicCommissioningWindow(chip::System::Clock::Seconds32(timeoutSec));
    if (err != CHIP_NO_ERROR)
        ESP_LOGE(TAG, "OpenBasicCommissioningWindow failed: %" CHIP_ERROR_FORMAT, err.Format());
    else
        ESP_LOGI(TAG, "Commissioning window opened (%u s)", timeoutSec);
}

bool matterOpenCommissioningWindow(uint16_t timeoutSec)
{
    if (!s_started) return false;
    // ScheduleWork posts to the CHIP event loop — safe from any task, no lock needed
    auto err = chip::DeviceLayer::PlatformMgr().ScheduleWork(openCommissioningWindowWork, static_cast<intptr_t>(timeoutSec));
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "ScheduleWork for commissioning window failed: %" CHIP_ERROR_FORMAT, err.Format());
        return false;
    }
    return true;
}

#endif // MATTER_ENABLED
