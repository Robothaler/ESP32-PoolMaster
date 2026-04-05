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
#include <setup_payload/OnboardingCodesUtil.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>

// ── CHIP cluster IDs (from Matter specification) ──────────────────────────────
#include <app/clusters/on-off-server/on-off-server.h>
// Use raw IDs for ElectricalMeasurement to avoid include-path fragility
// across esp_matter versions. Cluster 0x0B04, ActivePower attribute 0x050B.
static constexpr uint32_t kElecMeasClusterId = 0x0B04;
static constexpr uint32_t kActivePowerAttrId  = 0x050B;

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

// Endpoint IDs — populated after endpoint::create(), invalid until then
static uint16_t s_ep_filt = chip::kInvalidEndpointId;
static uint16_t s_ep_ph   = chip::kInvalidEndpointId;
static uint16_t s_ep_heat = chip::kInvalidEndpointId;
static uint16_t s_ep_salt = chip::kInvalidEndpointId;
static uint16_t s_ep_chl  = chip::kInvalidEndpointId;

// Last-known reachability for conditional endpoints (Salt/Chl)
static bool s_salt_chl_was_active = false;

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
            break;
        case chip::DeviceLayer::DeviceEventType::kInternetConnectivityChange:
            ESP_LOGI(TAG, "Matter internet connectivity changed");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            ESP_LOGW(TAG, "Matter fabric removed");
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

    // ── Create pump child endpoints ────────────────────────────────────────────
    endpoint_t *ep_filt = createPumpEndpoint(s_node, "Filterpumpe",    true);   // with power
    endpoint_t *ep_ph   = createPumpEndpoint(s_node, "PH-Pumpe",       false);  // no power meter
    endpoint_t *ep_heat = createPumpEndpoint(s_node, "Waermepumpe",    true);   // with power
    endpoint_t *ep_salt = createPumpEndpoint(s_node, "Salzelektrolyse",true);   // with power, conditional
    endpoint_t *ep_chl  = createPumpEndpoint(s_node, "Chlor-Pumpe",   false);  // no power, conditional

    // ── Store endpoint IDs for later attribute updates ─────────────────────────
    if (ep_filt) s_ep_filt = endpoint::get_id(ep_filt);
    if (ep_ph)   s_ep_ph   = endpoint::get_id(ep_ph);
    if (ep_heat) s_ep_heat = endpoint::get_id(ep_heat);
    if (ep_salt) s_ep_salt = endpoint::get_id(ep_salt);
    if (ep_chl)  s_ep_chl  = endpoint::get_id(ep_chl);

    ESP_LOGI(TAG, "Endpoints — filt:%u  ph:%u  heat:%u  salt:%u  chl:%u",
             s_ep_filt, s_ep_ph, s_ep_heat, s_ep_salt, s_ep_chl);

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

    esp_err_t err = esp_matter::start(on_device_event);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() failed: %s", esp_err_to_name(err));
        return;
    }

    s_started = true;

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

    ESP_LOGV(TAG, "Matter sync complete");
}

// =============================================================================
//  T14: MatterSyncTask — periodic state sync, Core 1
// =============================================================================
void MatterSyncTask(void *pvParameters)
{
    const TickType_t period   = pdMS_TO_TICKS(MATTER_SYNC_PERIOD_MS);
    TickType_t       lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        if (startTasks && s_started) {
            matterBridgeSync();
        }
    }
}

#endif // MATTER_ENABLED
