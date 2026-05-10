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
//  When MATTER_MINIMAL_DEVICE=1 (Config.h): one non-bridged On/Off plugin unit only
//  (no Aggregator, no temp/solar EPs) — for commissioning / PASE bring-up.
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
#include "MatterAppTaskSuspend.h"
#include "PoolMaster.h"
#include "PoolSolarBridge.h"

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
#include <esp_matter_attribute_utils.h>

// ── CHIP / Matter stack headers ───────────────────────────────────────────────
#include <platform/CHIPDeviceLayer.h>
#include <platform/ConnectivityManager.h>
#include <platform/PlatformManager.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <system/SystemClock.h>

// ── ESP-IDF NVS / system headers (needed for direct factory-reset fallback) ──
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

#include <sys/time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if MATTER_WIFI_STA_OFF_DURING_BLE_GAP || MATTER_BLE_GAP_DIAG_LISTENER || MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE
extern "C" {
#include "host/ble_gap.h"
}
#endif

// ── C++ atomics for cross-task BLE commissioning flag ─────────────────────────
#include <atomic>

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
static const char *TAG_BLE_GAP = "MatterBLE";

#if MATTER_AGENT_DEBUG_NDJSON
#include <cstdio>
// #region agent log
static std::atomic<uint32_t> s_agent_notify_tx_total{ 0 };

static void matterAgentDbgFn(const char *hypothesisId, const char *location, const char *message, int d1,
                              unsigned d2)
{
    const long long ts = (long long)esp_timer_get_time();
    printf( // NOLINT: raw UART NDJSON for Cursor debug ingest
        "NDJSON{\"sessionId\":\"e37f7a\",\"hypothesisId\":\"%s\",\"location\":\"%s\",\"message\":\"%s\","
        "\"data\":{\"d1\":%d,\"d2\":%u},\"timestamp\":%lld}\n",
        hypothesisId, location, message, d1, d2, ts);
}
#define MATTER_AGENT_DBG(hid, loc, msg, d1, d2) matterAgentDbgFn((hid), (loc), (msg), (d1), (d2))
// #endregion
#else
#define MATTER_AGENT_DBG(hid, loc, msg, d1, d2) ((void)0)
#endif

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

// ── BLE commissioning activity flag ───────────────────────────────────────────
// Set to true while a CHIPoBLE connection is open (during BTP/PASE handshake).
// Other tasks (MQTT publish, MatterSync) gate their WiFi traffic on this flag
// so the shared ESP32-S3 radio can prioritise BLE during commissioning.
// Atomic because it's read from PublishTopic()/MatterSyncTask (Core 1) and
// written from on_device_event() / BLE hold recompute (CHIP task / PlatformMgr work).
static std::atomic<bool> s_ble_commissioning_active{false};

// Set from ChipDeviceLayer BLE events — NumBLEConnections() can stay 0 on ESP32-NimBLE
// until long after the phone has started GATT (see serial: no "BLE link up", MQTT PING
// during CHIPoBLE). Session events fire on first RX write / subscribe / disconnect.
static std::atomic<bool> s_chipoble_session_evt{false};

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

/** Apple Home reads General Commissioning attrs 0x09–0x0C (Matter 1.3+). esp_matter does not
 *  create them on this cluster yet. Use ATTRIBUTE_FLAG_NONE (external storage): MANAGED_INTERNALLY
 *  leaves EmberAfAttributeMetadata type/size at 0, so IM reads fail even if the attr exists.
 */
static void matterPatchGeneralCommissioningAppleAttrs(node_t *node)
{
    constexpr uint32_t kClusterGc                  = 0x00000030u;
    constexpr uint32_t kAttrTcUpdateDeadline       = 0x00000009u;
    constexpr uint32_t kAttrRecoveryIdentifier     = 0x0000000Au;
    constexpr uint32_t kAttrNetworkRecoveryReason  = 0x0000000Bu;
    constexpr uint32_t kAttrIsCommissioningWoPower = 0x0000000Cu;

    endpoint_t *ep = endpoint::get(node, 0);
    if (!ep) {
        ESP_LOGW(TAG, "GC Apple patch: endpoint 0 missing");
        return;
    }
    cluster_t *cl = esp_matter::cluster::get(ep, kClusterGc);
    if (!cl) {
        ESP_LOGW(TAG, "GC Apple patch: GeneralCommissioning cluster missing");
        return;
    }

    auto add_if_missing = [&](uint32_t aid, uint16_t flags, esp_matter_attr_val_t val, uint16_t max_sz = 0) {
        if (attribute::get(cl, aid)) {
            return;
        }
        attribute_t *a = attribute::create(cl, aid, flags, val, max_sz);
        if (a) {
            ESP_LOGI(TAG, "GC Apple patch: registered attribute 0x%08lx", (unsigned long)aid);
        } else {
            ESP_LOGW(TAG, "GC Apple patch: attribute::create failed for 0x%08lx", (unsigned long)aid);
        }
    };

    // 0x09: nullable epoch-us — null = no TC deadline (wired pool controller).
    add_if_missing(kAttrTcUpdateDeadline, ATTRIBUTE_FLAG_NONE, esp_matter_nullable_uint64(nullable<uint64_t>()));
    add_if_missing(kAttrRecoveryIdentifier, ATTRIBUTE_FLAG_NONE, esp_matter_octet_str(nullptr, 0), 16);
    add_if_missing(kAttrNetworkRecoveryReason, ATTRIBUTE_FLAG_NONE, esp_matter_enum8(0));
    add_if_missing(kAttrIsCommissioningWoPower, ATTRIBUTE_FLAG_NONE, esp_matter_bool(false));
}

#if !(defined(CONFIG_ENABLE_ICD_SERVER) && CONFIG_ENABLE_ICD_SERVER)
/** Apple Home issues reads against ICD Management (0x46) on the root endpoint even when
 *  CONFIG_ENABLE_ICD_SERVER is off (no Thread MTD). Without this cluster, IM returns
 *  UnsupportedCluster (logged as err …5c3 / status 0xc3) and commissioning aborts.
 *  Provide a minimal non–sleepy stub: mains-powered Wi-Fi device (short idle, normal active).
 */
static void matterPatchIcdManagementAppleStub(node_t *node)
{
    endpoint_t *ep = endpoint::get(node, 0);
    if (!ep) {
        ESP_LOGW(TAG, "ICD Apple stub: endpoint 0 missing");
        return;
    }
    constexpr uint32_t kIcdClusterId = IcdManagement::Id;
    if (esp_matter::cluster::get(ep, kIcdClusterId)) {
        return;
    }
    cluster_t *cl = esp_matter::cluster::create(ep, kIcdClusterId, CLUSTER_FLAG_SERVER);
    if (!cl) {
        ESP_LOGW(TAG, "ICD Apple stub: cluster create failed");
        return;
    }
    constexpr uint16_t kIcdRevision = 2; // esp_matter_cluster_revisions icd_management
    esp_matter::cluster::global::attribute::create_feature_map(cl, 0);
    esp_matter::cluster::global::attribute::create_cluster_revision(cl, kIcdRevision);

    auto add = [&](uint32_t aid, esp_matter_attr_val_t val, uint16_t max_sz = 0) {
        if (attribute::get(cl, aid)) {
            return;
        }
        attribute_t *a = attribute::create(cl, aid, ATTRIBUTE_FLAG_NONE, val, max_sz);
        if (a) {
            ESP_LOGI(TAG, "ICD Apple stub: registered attribute 0x%08lx", (unsigned long)aid);
        } else {
            ESP_LOGW(TAG, "ICD Apple stub: attribute::create failed for 0x%08lx", (unsigned long)aid);
        }
    };

    add(IcdManagement::Attributes::IdleModeDuration::Id, esp_matter_uint32(1));
    add(IcdManagement::Attributes::ActiveModeDuration::Id, esp_matter_uint32(300));
    add(IcdManagement::Attributes::ActiveModeThreshold::Id, esp_matter_uint16(300));
    add(IcdManagement::Attributes::UserActiveModeTriggerHint::Id, esp_matter_bitmap32(0));
    add(IcdManagement::Attributes::UserActiveModeTriggerInstruction::Id, esp_matter_char_str(nullptr, 0),
        esp_matter::cluster::icd_management::attribute::k_user_active_mode_trigger_instruction_length);
}
#else
static void matterPatchIcdManagementAppleStub(node_t *) {}
#endif

/** Apple Home reads a manufacturer-extended cluster on endpoint 0 logged as clusterId **0x1349_FC00**
 *  (ChipLogFormatMEI): high 16 bits are Apple's Matter vendor id **0x1349**, low 16 bits **0xFC00**
 *  (manufacturer cluster range — same layout as ESP RainMaker **0x131BFC00**).
 *  Attribute **0x00000001** must be readable; without the cluster, IM returns UnsupportedCluster (**err …5c3**)
 *  and commissioning aborts (ArmFailSafe 0s / fail-safe expiry).
 */
static void matterPatchAppleHomeKitMeiStub(node_t *node)
{
    endpoint_t *ep = endpoint::get(node, 0);
    if (!ep) {
        ESP_LOGW(TAG, "Apple MEI stub: endpoint 0 missing");
        return;
    }
    constexpr uint32_t kAppleHomeKitMeiClusterId = 0x1349FC00u;
    if (esp_matter::cluster::get(ep, kAppleHomeKitMeiClusterId)) {
        return;
    }
    cluster_t *cl = esp_matter::cluster::create(ep, kAppleHomeKitMeiClusterId, CLUSTER_FLAG_SERVER);
    if (!cl) {
        ESP_LOGW(TAG, "Apple MEI stub: cluster create failed");
        return;
    }
    attribute::create(cl, Globals::Attributes::ClusterRevision::Id, ATTRIBUTE_FLAG_NONE, esp_matter_uint16(1));
    /* Type for attr 1 is not public; UINT32 zero is a safe first guess (capability / flags pattern). */
    constexpr uint32_t kAppleMeiAttr1 = 0x00000001u;
    attribute::create(cl, kAppleMeiAttr1, ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
    ESP_LOGI(TAG, "Apple MEI stub: cluster 0x1349FC00 + ClusterRevision + attr 1 on EP0");
}

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
            if      (ep == 1) {
                storage.solarRoofTemp = temp;
                if (storage.SolarLocExt)
                    storage.SolarTemp = (double)temp;
            }
            else if (ep == 2) { storage.solarBoilerTemp  = temp; }
            else if (ep == 3) { storage.solarStorageTemp = temp; }
            else if (ep == 4) {
                storage.solarBackflowTemp = temp;
                if (storage.SolarLocExt)
                    storage.SolarRLTemp = (double)temp;
            }
            if (storage.SolarLocExt)
                storage.SolarOnline = true;
            ESP_LOGD(TAG, "SolarControl EP%u temp: %.2f°C", ep, temp);
        }
    }
    // ── OnOff::OnOff (bool) ───────────────────────────────────────────────────
    else if (cid == kOnOffClusterId && aid == kOnOffAttrId) {
        bool val = false;
        if (data->Get(val) == CHIP_NO_ERROR) {
            if      (ep == s_solar_ep_pump)  {
                storage.solarPumpRunning = val;
                if (storage.SolarLocExt)
                    storage.SolarPumpStatus = val ? 1 : 0;
            }
            else if (ep == s_solar_ep_valve) {
                storage.solarValvePool   = val;
                if (storage.SolarLocExt)
                    storage.ValveStatus = val ? 1 : 0;
            }
            if (storage.SolarLocExt)
                storage.SolarOnline = true;
            ESP_LOGD(TAG, "SolarControl EP%u OnOff: %d", ep, (int)val);
        }
    }
    // ── BooleanState::StateValue (bool) ──────────────────────────────────────
    else if (cid == kBoolStateClusterId && aid == kBoolStateAttrId) {
        bool val = false;
        if (data->Get(val) == CHIP_NO_ERROR) {
            if (ep == 9) {
                storage.solarValveOK = val;
                if (storage.SolarLocExt)
                    storage.SolarOnline = true;
            }
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
    prefsLock();
    Preferences pnvs;
    if (!pnvs.begin("PoolMaster", true)) {
        prefsUnlock();
        ESP_LOGW(TAG, "NVS open failed — using default solar config");
        return;
    }
    uint64_t nodeId = pnvs.getULong64(NVS_KEY_SOLAR_NODE_ID, 0);
    if (nodeId != 0) {
        s_solar_node_id  = nodeId;
        s_solar_ep_pump  = pnvs.getUShort(NVS_KEY_SOLAR_EP_PUMP,  5);
        s_solar_ep_valve = pnvs.getUShort(NVS_KEY_SOLAR_EP_VALVE, 6);
        s_solar_ep_circ  = pnvs.getUShort(NVS_KEY_SOLAR_EP_CIRC,  7);
        s_solar_ep_illum = pnvs.getUShort(NVS_KEY_SOLAR_EP_ILLUM, 8);
        ESP_LOGI(TAG, "Solar config loaded: NodeId=0x%016llX pump=%u valve=%u circ=%u illum=%u",
                 s_solar_node_id, s_solar_ep_pump, s_solar_ep_valve,
                 s_solar_ep_circ, s_solar_ep_illum);
    } else {
        ESP_LOGI(TAG, "No SolarControl NodeId in NVS — use SET_SOLAR_NODE to configure");
    }
    pnvs.end();
    prefsUnlock();
}

/**
 * @brief Save SolarControl Matter config to NVS.
 *        Called after SET_SOLAR_NODE serial command.
 */
static void saveSolarConfig()
{
    prefsLock();
    Preferences pnvs;
    if (!pnvs.begin("PoolMaster", false)) {
        prefsUnlock();
        ESP_LOGE(TAG, "NVS open for write failed");
        return;
    }
    pnvs.putULong64(NVS_KEY_SOLAR_NODE_ID, s_solar_node_id);
    pnvs.putUShort(NVS_KEY_SOLAR_EP_PUMP,  s_solar_ep_pump);
    pnvs.putUShort(NVS_KEY_SOLAR_EP_VALVE, s_solar_ep_valve);
    pnvs.putUShort(NVS_KEY_SOLAR_EP_CIRC,  s_solar_ep_circ);
    pnvs.putUShort(NVS_KEY_SOLAR_EP_ILLUM, s_solar_ep_illum);
    pnvs.end();
    prefsUnlock();
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

static void syncBleGapRadioHoldFromStack();
static void processBleRadioHoldChipEvent(const chip::DeviceLayer::ChipDeviceEvent *event);

/** Mirrors critical commissioning lines to Serial (same as user logs), independent of esp_log tag level. */
static void matterSerialChipf(const char *line)
{
    Serial.print("[Matter] ");
    Serial.println(line);
}

// =============================================================================
//  Matter callback: platform/commissioning events
// =============================================================================
static void on_device_event(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg)
{
    using namespace chip::DeviceLayer;

    // esp_matter forwards a subset here; BLE internals also go to PlatformMgr AddEventHandler.
    processBleRadioHoldChipEvent(event);

    switch (event->Type) {
        case DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "Matter commissioning complete!");
            Serial.println("[Matter] Commissioning erfolgreich abgeschlossen — Fabric angelegt.");
#if defined(CONFIG_ESP_MATTER_CONTROLLER_ENABLE) && !MATTER_MINIMAL_DEVICE
            if (s_solar_node_id != 0 && !s_solar_subscribed) {
                subscribeToSolarControl(s_solar_node_id);
            }
#endif
            break;

        case DeviceEventType::kInternetConnectivityChange:
            ESP_LOGI(TAG, "Matter internet connectivity changed");
            break;

        case DeviceEventType::kFabricRemoved:
            ESP_LOGW(TAG, "Matter fabric removed");
#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
            s_solar_subscribed = false;
#endif
            break;

        default:
            break;
    }
}

// Second path for the same logic — sees kCHIPoBLE* events on the CHIP thread.
static void platformBleHoldEventHandler(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t /*arg*/)
{
    processBleRadioHoldChipEvent(event);
}

// =============================================================================
//  Public: BLE commissioning activity query (for MQTT / MatterSync gating)
// =============================================================================
bool matterIsBleCommissioning()
{
    return s_ble_commissioning_active.load(std::memory_order_acquire);
}

void matterYieldAppTasksIfChipobleBusy(void)
{
#if MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE
    if (!s_started) {
        return;
    }
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        return;
    }
    if (!s_chipoble_session_evt.load(std::memory_order_acquire)) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(MATTER_CHIPOBLE_APP_YIELD_MS));
#endif
}

void matterApplyRadioHoldAfterTimersReady()
{
#if MATTER_NO_MQTT_UNTIL_COMMISSIONED
    syncBleGapRadioHoldFromStack();
#endif
}

/**
 * Drive shared-radio hold for BLE commissioning.
 *
 * If MATTER_NO_MQTT_UNTIL_COMMISSIONED: hold MQTT whenever fabric count is 0 — CHIP
 * BLE events are not delivered to app handlers on this esp_matter build, so this is
 * the reliable fix for shared-radio PASE timeouts.
 *
 * Otherwise: use NumBLEConnections + s_chipoble_session_evt (best-effort).
 */
static void syncBleGapRadioHoldFromStack()
{
    if (!s_started) {
        return;
    }

    const uint8_t  fabrics   = chip::Server::GetInstance().GetFabricTable().FabricCount();
    const uint16_t ble_cons  = chip::DeviceLayer::ConnectivityMgr().NumBLEConnections();
    const bool     evt_sess  = s_chipoble_session_evt.load(std::memory_order_acquire);
#if MATTER_NO_MQTT_UNTIL_COMMISSIONED
    const bool want_hold = (fabrics == 0);
#else
    const bool want_hold = (fabrics == 0) && (ble_cons > 0 || evt_sess);
#endif

    const bool cur = s_ble_commissioning_active.load(std::memory_order_acquire);
    if (want_hold == cur) {
        return;
    }

    s_ble_commissioning_active.store(want_hold, std::memory_order_release);
    mqttSetBleRadioHold(want_hold);

#if MATTER_WIFI_PS_MAX_WHILE_UNCOMMISSIONED
    if (want_hold) {
        esp_err_t pe = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        ESP_LOGI(TAG, "WiFi PS MAX while uncommissioned (coexist): %s", esp_err_to_name(pe));
    } else {
        esp_err_t pe = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        ESP_LOGI(TAG, "WiFi PS MIN restored: %s", esp_err_to_name(pe));
    }
#endif

    if (want_hold) {
        ESP_LOGI(TAG,
                 "Matter MQTT hold ON (fabrics=%u, NumBLE=%u, evt_sess=%d) — shared 2.4 GHz radio free for "
                 "commissioning.",
                 static_cast<unsigned>(fabrics), static_cast<unsigned>(ble_cons), static_cast<int>(evt_sess));
        Serial.printf("[Matter] MQTT aus bis Pairing OK (fabrics=%u) — Funk frei für Matter/BLE.\r\n",
                      static_cast<unsigned>(fabrics));
    } else {
        ESP_LOGI(TAG,
                 "Matter MQTT hold OFF (fabrics=%u, NumBLE=%u, evt_sess=%d) — MQTT reconnect allowed.",
                 static_cast<unsigned>(fabrics), static_cast<unsigned>(ble_cons), static_cast<int>(evt_sess));
        Serial.printf("[Matter] Fabric aktiv — MQTT reconnect erlaubt (fabrics=%u).\r\n",
                      static_cast<unsigned>(fabrics));
    }
}

static void matterReleaseGapWifiCoexSession();

static void processBleRadioHoldChipEvent(const chip::DeviceLayer::ChipDeviceEvent *event)
{
    using namespace chip::DeviceLayer;

    static bool s_logged_first_chip_evt;
    if (!s_logged_first_chip_evt) {
        s_logged_first_chip_evt = true;
        matterSerialChipf("Erstes CHIP-DeviceLayer-Event (Logging-Pfad OK).");
        ESP_LOGW(TAG, "First ChipDeviceLayer event received (logging path OK).");
    }

    const uint8_t fabrics = s_started ? chip::Server::GetInstance().GetFabricTable().FabricCount() : 255;
    const uint16_t ble_cons = s_started ? chip::DeviceLayer::ConnectivityMgr().NumBLEConnections() : 0;
    const int64_t t_us = esp_timer_get_time();
    const unsigned heap_i = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    switch (event->Type) {
        case DeviceEventType::kCHIPoBLEConnectionEstablished:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLEConnectionEstablished t=%lldus fabrics=%u NumBLE=%u heap_int=%u",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics),
                     static_cast<unsigned>(ble_cons), heap_i);
            matterSerialChipf("CHIPoBLE: Verbindung hergestellt");
            if (s_started && fabrics == 0) {
                s_chipoble_session_evt.store(true, std::memory_order_release);
                syncBleGapRadioHoldFromStack();
            }
            break;

        case DeviceEventType::kCHIPoBLEWriteReceived:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLEWriteReceived t=%lldus fabrics=%u NumBLE=%u heap_int=%u",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics),
                     static_cast<unsigned>(ble_cons), heap_i);
            matterSerialChipf("CHIPoBLE: RX-Write (Phone schreibt)");
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H2", "CHIP_EVT", "WriteReceived", static_cast<int>(heap_i),
                             static_cast<unsigned>(ble_cons));
#endif
            if (s_started && fabrics == 0) {
                s_chipoble_session_evt.store(true, std::memory_order_release);
                syncBleGapRadioHoldFromStack();
            }
            break;

        case DeviceEventType::kCHIPoBLESubscribe:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLESubscribe (CCCD on) t=%lldus fabrics=%u NumBLE=%u heap_int=%u",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics),
                     static_cast<unsigned>(ble_cons), heap_i);
            matterSerialChipf("CHIPoBLE: CCCD Subscribe (Indications an)");
            break;

        case DeviceEventType::kCHIPoBLEUnsubscribe:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLEUnsubscribe t=%lldus fabrics=%u NumBLE=%u",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics),
                     static_cast<unsigned>(ble_cons));
            matterSerialChipf("CHIPoBLE: CCCD Unsubscribe");
            break;

        case DeviceEventType::kCHIPoBLEIndicateConfirm:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLEIndicateConfirm (TX indication ACK) t=%lldus fabrics=%u",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics));
            matterSerialChipf("CHIPoBLE: TX-Indication bestätigt (wichtig für BTP)");
            break;

        case DeviceEventType::kCHIPoBLENotifyConfirm:
            ESP_LOGD(TAG, "CHIP evt: CHIPoBLENotifyConfirm fabrics=%u", static_cast<unsigned>(fabrics));
            break;

        case DeviceEventType::kCHIPoBLEConnectionClosed:
        case DeviceEventType::kCHIPoBLEConnectionError:
            ESP_LOGI(TAG,
                     "CHIP evt: CHIPoBLE %s t=%lldus fabrics=%u NumBLE=%u",
                     event->Type == DeviceEventType::kCHIPoBLEConnectionError ? "ConnectionError" : "ConnectionClosed",
                     static_cast<long long>(t_us), static_cast<unsigned>(fabrics),
                     static_cast<unsigned>(ble_cons));
            matterSerialChipf(event->Type == DeviceEventType::kCHIPoBLEConnectionError
                                  ? "CHIPoBLE: Verbindungsfehler"
                                  : "CHIPoBLE: Verbindung geschlossen");
            if (event->Type == DeviceEventType::kCHIPoBLEConnectionError) {
                ESP_LOGI(TAG, "CHIPoBLEConnectionError reason=%s", chip::ErrorStr(event->CHIPoBLEConnectionError.Reason));
            }
            s_chipoble_session_evt.store(false, std::memory_order_release);
            matterReleaseGapWifiCoexSession();
            syncBleGapRadioHoldFromStack();
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
            /* Always resume: commissioning often continues over Wi‑Fi after BLE teardown while
             * FabricCount() is still 0 (until AddNOC). Leaving PoolMaster suspended that whole time
             * trips the task watchdog (see serial: PoolMaster did not reset TWDT). Idempotent. */
            matterResumeAppTasksAfterGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "CHIPoBLE", "task_resume_closed", (int)event->Type, (int)fabrics);
#endif
#endif
            break;

        case DeviceEventType::kCommissioningComplete:
            ESP_LOGI(TAG, "CHIP evt: CommissioningComplete t=%lldus", static_cast<long long>(t_us));
            matterSerialChipf("Commissioning abgeschlossen (Fabric)");
            s_chipoble_session_evt.store(false, std::memory_order_release);
            matterReleaseGapWifiCoexSession();
            syncBleGapRadioHoldFromStack();
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
            matterResumeAppTasksAfterGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "CommissioningComplete", "task_resume", 1, 0);
#endif
#endif
            break;

        case DeviceEventType::kFabricRemoved:
            ESP_LOGI(TAG, "CHIP evt: FabricRemoved t=%lldus", static_cast<long long>(t_us));
            matterSerialChipf("Fabric entfernt");
            s_chipoble_session_evt.store(false, std::memory_order_release);
            matterReleaseGapWifiCoexSession();
            syncBleGapRadioHoldFromStack();
            break;

        case DeviceEventType::kFailSafeTimerExpired:
            ESP_LOGW(TAG, "CHIP evt: FailSafeTimerExpired t=%lldus fabrics=%u", static_cast<long long>(t_us),
                     static_cast<unsigned>(fabrics));
            matterSerialChipf("Fail-Safe Timer abgelaufen");
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
            if (fabrics == 0) {
                matterResumeAppTasksAfterGapPase();
            }
#endif
            break;

        case DeviceEventType::kSecureSessionEstablished:
            ESP_LOGI(TAG, "CHIP evt: SecureSessionEstablished t=%lldus fabrics=%u", static_cast<long long>(t_us),
                     static_cast<unsigned>(fabrics));
            matterSerialChipf("Secure Session etabliert");
            break;

        default:
            // Internal events use ESP_LOGD in older code → invisible with default INFO console.
            // While uncommissioned, log internal + unknown at INFO so numeric type is visible in Apple-debug builds.
            if (s_started && fabrics == 0) {
                if (!event->IsPublic()) {
                    ESP_LOGI(TAG,
                             "CHIP evt (internal, fabric0) type=%u heap_int=%u NumBLE=%u platSpec=%d",
                             static_cast<unsigned>(event->Type), heap_i, static_cast<unsigned>(ble_cons),
                             static_cast<int>(event->IsPlatformSpecific()));
                    Serial.printf("[Matter] CHIP intern type=%u heap=%u NumBLE=%u\r\n",
                                  static_cast<unsigned>(event->Type), heap_i,
                                  static_cast<unsigned>(ble_cons));
                } else {
                    ESP_LOGD(TAG, "CHIP evt (public) type=%u", static_cast<unsigned>(event->Type));
                }
            }
            break;
    }
}

// Runs on the CHIP event loop — required so ConnectivityMgr / BLEMgr state matches
// what the stack sees (polling from Core 1 was a no-op in practice: MQTT kept
// running during GAP sessions, see serial logs).
static void bleRadioHoldPollWork(intptr_t /*arg*/)
{
    syncBleGapRadioHoldFromStack();
}

/**
 * Drive s_chipoble_session_evt from NimBLE GAP when ChipDeviceLayer omits
 * kCHIPoBLE* events (common on this port). Enables matterYieldAppTasksIfChipobleBusy()
 * so Core-1 loops yield during the BLE commissioning window.
 */
static void chipobleSessionHintFromBleGap(bool active)
{
#if MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE
    if (!s_started) {
        return;
    }
    const bool prev = s_chipoble_session_evt.load(std::memory_order_acquire);
    if (prev == active) {
        return;
    }
    s_chipoble_session_evt.store(active, std::memory_order_release);
    syncBleGapRadioHoldFromStack();
#else
    (void)active;
#endif
}

#if MATTER_BLE_GAP_DIAG_LISTENER
// ChipDeviceLayer often does not deliver kCHIPoBLESubscribe / IndicateConfirm on esp_matter;
// NimBLE posts GAP events for the same PHY actions — log them here for Apple commissioning.
static struct ble_gap_event_listener s_matter_ble_gap_diag_listener;

#if MATTER_AGENT_DEBUG_NDJSON
static int64_t s_agent_gap_connect_us;
static int64_t s_agent_tx_cccd_subscribe_us;
#endif

#if !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
static esp_timer_handle_t s_wifi_subscribe_relief_timer;
static std::atomic<bool>     s_subscribe_relief_sta_down{ false };

static void wifiSubscribeReliefTimerCb(void * /*arg*/)
{
#if MATTER_AGENT_DEBUG_NDJSON
    MATTER_AGENT_DBG("H1", "relief_cb", "entry", (int)(esp_timer_get_time() - s_agent_tx_cccd_subscribe_us),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif
    if (!s_started) {
        return;
    }
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        return;
    }
    ESP_LOGI(TAG,
             "Commissioning: WiFi STA disconnect (TX CCCD 0x%04x subscribed, deferred %u ms)",
             static_cast<unsigned>(MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE),
             static_cast<unsigned>(MATTER_WIFI_BLE_SUBSCRIBE_RELIEVE_US / 1000u));
    mqttSetMatterWifiReconnectHold(true);
    s_subscribe_relief_sta_down.store(true, std::memory_order_release);
    esp_err_t w = esp_wifi_disconnect();
#if MATTER_AGENT_DEBUG_NDJSON
    MATTER_AGENT_DBG("H1", "relief_cb", "post_wifi_disc", (int)w, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif
    if (w != ESP_OK && w != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "esp_wifi_disconnect (subscribe relief): %s", esp_err_to_name(w));
    }
}
#endif

static int matterBleGapDiagEvent(struct ble_gap_event *event, void * /*arg*/)
{
    if (!s_started) {
        return 0;
    }

    const uint8_t fabrics = chip::Server::GetInstance().GetFabricTable().FabricCount();
    if (fabrics != 0 && event->type != BLE_GAP_EVENT_DISCONNECT) {
        return 0;
    }

    const unsigned hi = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    const unsigned hi_max = static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    const int64_t t_us = esp_timer_get_time();

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGW(TAG_BLE_GAP, "GAP CONNECT status=%d t=%lldus heap_int=%u heap_max_blk=%u",
                 static_cast<int>(event->connect.status), static_cast<long long>(t_us), hi, hi_max);
        MATTER_AGENT_DBG("H5", "GAP_CONNECT", "evt", (int)event->connect.status, hi_max);
#if MATTER_AGENT_DEBUG_NDJSON
        if (event->connect.status == 0) {
            s_agent_gap_connect_us = esp_timer_get_time();
        }
#endif
#if !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
        if (s_wifi_subscribe_relief_timer) {
            (void)esp_timer_stop(s_wifi_subscribe_relief_timer);
        }
#endif
        if (event->connect.status == 0 && fabrics == 0) {
#if MATTER_AGENT_DEBUG_NDJSON
            s_agent_notify_tx_total.store(0, std::memory_order_relaxed);
#endif
            chipobleSessionHintFromBleGap(true);
            ESP_LOGI(TAG, "GAP: Session-Hint an (CPU-Yield fuer CHIPoBLE aktiv).");
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
            matterSuspendAppTasksForGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "GAP_CONNECT", "task_suspend", 1, 0);
#endif
#endif
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG_BLE_GAP, "GAP DISCONNECT reason=0x%02x t=%lldus heap_int=%u heap_max_blk=%u",
                 event->disconnect.reason, static_cast<long long>(t_us), hi, hi_max);
#if MATTER_AGENT_DEBUG_NDJSON
        MATTER_AGENT_DBG("H5", "GAP_DISCONNECT", "evt", (int)event->disconnect.reason,
                         (unsigned)((esp_timer_get_time() - s_agent_gap_connect_us) / 1000));
        MATTER_AGENT_DBG("H1", "GAP_DISCONNECT", "notify_tx_total_ms",
                         (int)s_agent_notify_tx_total.load(std::memory_order_relaxed),
                         (unsigned)((esp_timer_get_time() - s_agent_gap_connect_us) / 1000));
#endif
#if !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
        if (s_wifi_subscribe_relief_timer) {
            (void)esp_timer_stop(s_wifi_subscribe_relief_timer);
        }
        if (s_subscribe_relief_sta_down.exchange(false, std::memory_order_acq_rel)) {
            mqttSetMatterWifiReconnectHold(false);
            esp_err_t wr = esp_wifi_connect();
            if (wr != ESP_OK && wr != ESP_ERR_WIFI_CONN) {
                ESP_LOGW(TAG, "esp_wifi_connect (subscribe relief end): %s", esp_err_to_name(wr));
            }
            ESP_LOGI(TAG, "Commissioning: WiFi STA reconnect after BLE GAP disconnect");
        }
#endif
        chipobleSessionHintFromBleGap(false);
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
        if (fabrics != 0) {
            matterResumeAppTasksAfterGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "GAP_DISCONNECT", "task_resume", (int)event->disconnect.reason, 1);
#endif
        }
#endif
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGW(TAG_BLE_GAP,
                 "GAP SUBSCRIBE conn=%u attr=0x%04x reason=%u prev(n/i)=%u/%u cur(n/i)=%u/%u t=%lldus heap=%u",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 static_cast<unsigned>(event->subscribe.reason), event->subscribe.prev_notify,
                 event->subscribe.prev_indicate, event->subscribe.cur_notify, event->subscribe.cur_indicate,
                 static_cast<long long>(t_us), hi);
        MATTER_AGENT_DBG("H4", "GAP_SUBSCRIBE", "evt", (int)event->subscribe.attr_handle,
                         (unsigned)event->subscribe.cur_indicate | ((unsigned)event->subscribe.reason << 8u));
        if (fabrics == 0 && event->subscribe.cur_indicate != 0) {
            chipobleSessionHintFromBleGap(true);
#if MATTER_AGENT_DEBUG_NDJSON
            if (event->subscribe.attr_handle == MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE) {
                s_agent_tx_cccd_subscribe_us = esp_timer_get_time();
            }
#endif
#if !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
            if (event->subscribe.attr_handle == MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE &&
                s_wifi_subscribe_relief_timer &&
                !s_subscribe_relief_sta_down.load(std::memory_order_acquire)) {
                MATTER_AGENT_DBG("H2", "relief_arm", "pre_timer",
                                 (int)(esp_timer_get_time() - s_agent_gap_connect_us),
                                 (unsigned)(esp_timer_get_time() - s_agent_tx_cccd_subscribe_us));
                (void)esp_timer_stop(s_wifi_subscribe_relief_timer);
                esp_err_t te =
                    esp_timer_start_once(s_wifi_subscribe_relief_timer, MATTER_WIFI_BLE_SUBSCRIBE_RELIEVE_US);
                if (te != ESP_OK) {
                    ESP_LOGW(TAG, "subscribe-relief timer start: %s", esp_err_to_name(te));
                }
            }
#endif
        }
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGW(TAG_BLE_GAP, "GAP MTU conn=%u value=%u channel=0x%04x", event->mtu.conn_handle, event->mtu.value,
                 event->mtu.channel_id);
        break;
    case BLE_GAP_EVENT_NOTIFY_TX:
#if MATTER_AGENT_DEBUG_NDJSON
        s_agent_notify_tx_total.fetch_add(1, std::memory_order_relaxed);
#endif
        ESP_LOGW(TAG_BLE_GAP, "GAP NOTIFY_TX status=%d attr=0x%04x indication=%u t=%lldus heap=%u",
                 event->notify_tx.status, event->notify_tx.attr_handle, event->notify_tx.indication,
                 static_cast<long long>(t_us), hi);
        MATTER_AGENT_DBG("H3", "NOTIFY_TX", "evt", (int)event->notify_tx.status,
                         (unsigned)event->notify_tx.attr_handle | ((unsigned)event->notify_tx.indication << 16u));
        break;
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG_BLE_GAP, "GAP CONN_UPDATE status=%d", static_cast<int>(event->conn_update.status));
        break;
    default:
        break;
    }
    return 0;
}

static void registerMatterBleGapDiagListener()
{
#if !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
    if (!s_wifi_subscribe_relief_timer) {
        const esp_timer_create_args_t targs = {
            .callback = &wifiSubscribeReliefTimerCb,
            .name = "matter_sub_relief",
        };
        esp_err_t te = esp_timer_create(&targs, &s_wifi_subscribe_relief_timer);
        if (te != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create(matter_sub_relief): %s", esp_err_to_name(te));
        }
    }
#endif
    int r = ble_gap_event_listener_register(&s_matter_ble_gap_diag_listener, matterBleGapDiagEvent, nullptr);
    if (r != 0) {
        ESP_LOGW(TAG, "ble_gap_event_listener_register(diag) failed: %d", r);
    } else {
        ESP_LOGI(TAG, "NimBLE GAP diagnostic listener registered (SUBSCRIBE / NOTIFY_TX / MTU)");
        matterSerialChipf("NimBLE-GAP: Diagnose (SUBSCRIBE, NOTIFY_TX, MTU, …)");
    }
}
#endif // MATTER_BLE_GAP_DIAG_LISTENER

#if MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE && !MATTER_BLE_GAP_DIAG_LISTENER
static struct ble_gap_event_listener s_matter_ble_gap_session_listener;

static int matterBleGapSessionOnlyEvent(struct ble_gap_event *event, void * /*arg*/)
{
    if (!s_started) {
        return 0;
    }
    const uint8_t fabrics = chip::Server::GetInstance().GetFabricTable().FabricCount();
    if (fabrics != 0 && event->type != BLE_GAP_EVENT_DISCONNECT) {
        return 0;
    }
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0 && fabrics == 0) {
            chipobleSessionHintFromBleGap(true);
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
            matterSuspendAppTasksForGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "GAP_CONNECT", "task_suspend", 1, 0);
#endif
#endif
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        chipobleSessionHintFromBleGap(false);
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
        if (fabrics != 0) {
            matterResumeAppTasksAfterGapPase();
#if MATTER_AGENT_DEBUG_NDJSON
            MATTER_AGENT_DBG("H7", "GAP_DISCONNECT", "task_resume", (int)event->disconnect.reason, 1);
#endif
        }
#endif
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (fabrics == 0 && event->subscribe.cur_indicate != 0) {
            chipobleSessionHintFromBleGap(true);
        }
        break;
    default:
        break;
    }
    return 0;
}

static void registerMatterBleGapSessionListener()
{
    int r = ble_gap_event_listener_register(&s_matter_ble_gap_session_listener, matterBleGapSessionOnlyEvent, nullptr);
    if (r != 0) {
        ESP_LOGW(TAG, "ble_gap_event_listener_register(session) failed: %d", r);
    } else {
        ESP_LOGI(TAG, "NimBLE GAP session listener registered (CPU yield during CHIPoBLE)");
    }
}
#endif

#if MATTER_WIFI_STA_OFF_DURING_BLE_GAP
static struct ble_gap_event_listener s_matter_gap_wifi_coex_listener;
// Set when we intentionally dropped WiFi for an uncommissioned BLE session; must be
// cleared on GAP disconnect even if FabricCount() is already > 0 (commissioning
// just finished). Otherwise mqttSetMatterWifiReconnectHold stays true and WiFi
// never returns — Apple Home shows "Gerät nicht gefunden" after setup.
static std::atomic<bool> s_matter_gap_wifi_coex_engaged{ false };
static esp_timer_handle_t s_coex_wifi_disconnect_timer;

static void coexWifiDisconnectTimerCb(void * /*arg*/)
{
    if (!s_started) {
        return;
    }
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() != 0) {
        return;
    }
    ESP_LOGI(TAG, "Coexistence (deferred): WiFi STA disconnect for BLE (fabrics=0)");
    Serial.println("[Matter] WiFi getrennt (Coexist, verzoegert) fuer BLE.\r\n");
    // Before esp_wifi_disconnect(): suppress WiFi reconnect timer — otherwise
    // STA_DISCONNECTED immediately starts connectToWiFi() during PASE (0x213).
    mqttSetMatterWifiReconnectHold(true);
    s_matter_gap_wifi_coex_engaged.store(true, std::memory_order_release);
    esp_err_t w = esp_wifi_disconnect();
    if (w != ESP_OK && w != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "esp_wifi_disconnect: %s", esp_err_to_name(w));
    }
}

static int matterGapWifiCoexEvent(struct ble_gap_event *event, void * /*arg*/)
{
    if (!s_started) {
        return 0;
    }

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                const uint8_t fabrics = chip::Server::GetInstance().GetFabricTable().FabricCount();
                if (fabrics != 0) {
                    break;
                }
                // Never call esp_wifi_disconnect() directly from this NimBLE GAP callback —
                // it can break CHIPoBLE advertising (NimBLE "Adv reattempt failed; rc=3").
                if (!s_coex_wifi_disconnect_timer) {
                    break;
                }
                (void)esp_timer_stop(s_coex_wifi_disconnect_timer);
                esp_err_t te =
                    esp_timer_start_once(s_coex_wifi_disconnect_timer, MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US);
                if (te != ESP_OK) {
                    ESP_LOGW(TAG, "coex WiFi-off timer start: %s", esp_err_to_name(te));
                } else {
                    ESP_LOGI(TAG, "Coexistence: BLE GAP connected — WiFi STA disconnect in %u ms",
                             static_cast<unsigned>(MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US / 1000));
                    Serial.printf("[Matter] BLE verbunden — WiFi-Trennung in %u ms (Coexist).\r\n",
                                  static_cast<unsigned>(MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US / 1000));
                }
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            if (s_coex_wifi_disconnect_timer) {
                (void)esp_timer_stop(s_coex_wifi_disconnect_timer);
            }
            if (s_matter_gap_wifi_coex_engaged.exchange(false, std::memory_order_acq_rel)) {
                ESP_LOGI(TAG, "Coexistence: BLE GAP disconnected — WiFi STA reconnect");
                Serial.println("[Matter] BLE getrennt — WiFi reconnect.\r\n");
                mqttSetMatterWifiReconnectHold(false);
                esp_err_t w = esp_wifi_connect();
                if (w != ESP_OK && w != ESP_ERR_WIFI_CONN) {
                    ESP_LOGW(TAG, "esp_wifi_connect: %s", esp_err_to_name(w));
                }
            }
            break;
        default:
            break;
    }
    return 0;
}

static void registerMatterBleGapWifiCoexListener()
{
    if (!s_coex_wifi_disconnect_timer) {
        const esp_timer_create_args_t targs = {
            .callback = &coexWifiDisconnectTimerCb,
            .name = "matter_coex_wifi",
        };
        esp_err_t te = esp_timer_create(&targs, &s_coex_wifi_disconnect_timer);
        if (te != ESP_OK) {
            ESP_LOGE(TAG, "esp_timer_create(matter_coex_wifi): %s", esp_err_to_name(te));
        }
    }
    int r = ble_gap_event_listener_register(&s_matter_gap_wifi_coex_listener, matterGapWifiCoexEvent, nullptr);
    if (r != 0) {
        ESP_LOGW(TAG, "ble_gap_event_listener_register(coex) failed: %d", r);
    } else {
        ESP_LOGI(TAG, "NimBLE GAP coex listener registered (WiFi off during BLE while uncommissioned)");
    }
}
#endif // MATTER_WIFI_STA_OFF_DURING_BLE_GAP

/**
 * Clear Matter WiFi reconnect suppression; if aggressive STA coex was active, reconnect.
 * NimBLE GAP DISCONNECT and CHIP BLE/session events can arrive in either order — this
 * covers commissioning-complete and error paths when GAP alone would miss cleanup.
 */
static void matterReleaseGapWifiCoexSession()
{
    mqttSetMatterWifiReconnectHold(false);
#if MATTER_WIFI_STA_OFF_DURING_BLE_GAP
    if (s_coex_wifi_disconnect_timer) {
        (void)esp_timer_stop(s_coex_wifi_disconnect_timer);
    }
    if (s_matter_gap_wifi_coex_engaged.exchange(false, std::memory_order_acq_rel)) {
        esp_err_t w = esp_wifi_connect();
        if (w != ESP_OK && w != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "esp_wifi_connect (coex release): %s", esp_err_to_name(w));
        }
        ESP_LOGI(TAG, "Matter: coexistence released (CHIP path) — WiFi STA reconnect");
        Serial.println("[Matter] Coexistence beendet (CHIP) — WiFi verbindet wieder.\r\n");
    }
#endif
#if MATTER_BLE_GAP_DIAG_LISTENER && !MATTER_WIFI_STA_OFF_DURING_BLE_GAP && MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE
    if (s_wifi_subscribe_relief_timer) {
        (void)esp_timer_stop(s_wifi_subscribe_relief_timer);
    }
    if (s_subscribe_relief_sta_down.exchange(false, std::memory_order_acq_rel)) {
        esp_err_t w = esp_wifi_connect();
        if (w != ESP_OK && w != ESP_ERR_WIFI_CONN) {
            ESP_LOGW(TAG, "esp_wifi_connect (subscribe relief release): %s", esp_err_to_name(w));
        }
        ESP_LOGI(TAG, "Matter: subscribe-relief released — WiFi STA reconnect");
    }
#endif
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
    ESP_LOGI(TAG, "Initialising Matter (Phase 1 — node + endpoint creation)...");

#if MATTER_MINIMAL_DEVICE
    ESP_LOGW(TAG, "MATTER_MINIMAL_DEVICE: single On/Off plugin unit (no bridge) — PASE/commissioning debug");
#endif

    // ── Matter node configuration ──────────────────────────────────────────────
    // vendor_id and product_id are set via sdkconfig (CONFIG_DEVICE_VENDOR_ID /
    // CONFIG_DEVICE_PRODUCT_ID) or baked into the DAC — not via config_t here.
    node::config_t node_cfg;
    memset(&node_cfg, 0, sizeof(node_cfg));
    strncpy(node_cfg.root_node.basic_information.node_label,
            MATTER_DEVICE_NAME,
            sizeof(node_cfg.root_node.basic_information.node_label) - 1);

    /* memset zeroes nested cluster configs. Network Commissioning (0x31) FeatureMap must advertise at
     * least one network interface when the cluster exists — all-zero is out of spec; Apple Home then
     * stops commissioning after early reads and issues ArmFailSafe(0s). Match esp_matter defaults:
     * esp_matter_cluster.h network_commissioning::config_t ctor. */
#if CHIP_DEVICE_CONFIG_ENABLE_WIFI
    node_cfg.root_node.network_commissioning.feature_map =
        chip::to_underlying(NetworkCommissioning::Feature::kWiFiNetworkInterface);
#elif CHIP_DEVICE_CONFIG_ENABLE_THREAD
    node_cfg.root_node.network_commissioning.feature_map =
        chip::to_underlying(NetworkCommissioning::Feature::kThreadNetworkInterface);
#else
    node_cfg.root_node.network_commissioning.feature_map =
        chip::to_underlying(NetworkCommissioning::Feature::kEthernetNetworkInterface);
#endif

    // ── Create root node ───────────────────────────────────────────────────────
    s_node = node::create(&node_cfg, on_attribute_update, on_identification);
    if (!s_node) {
        ESP_LOGE(TAG, "FATAL: Failed to create Matter node!");
        return;
    }

#if MATTER_MINIMAL_DEVICE
    {
        endpoint::on_off_plugin_unit::config_t ep_cfg;
        memset(&ep_cfg, 0, sizeof(ep_cfg));
        ep_cfg.on_off.on_off = false;
        endpoint_t *ep =
            endpoint::on_off_plugin_unit::create(s_node, &ep_cfg, ENDPOINT_FLAG_NONE, nullptr);
        if (!ep) {
            ESP_LOGE(TAG, "FATAL: minimal On/Off endpoint failed");
            return;
        }
        s_ep_filt = endpoint::get_id(ep);
        ESP_LOGI(TAG, "Minimal OnOff EP id=%u (OnOff → FiltPump for debug)", s_ep_filt);
    }
#else  // !MATTER_MINIMAL_DEVICE

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

#endif  // !MATTER_MINIMAL_DEVICE

    matterPatchGeneralCommissioningAppleAttrs(s_node);
    matterPatchIcdManagementAppleStub(s_node);
    matterPatchAppleHomeKitMeiStub(s_node);

    // ── Use example/test Device Attestation Credentials ───────────────────────
    // IMPORTANT: Replace with real DAC for production devices!
    chip::Credentials::SetDeviceAttestationCredentialsProvider(
        chip::Credentials::Examples::GetExampleDACProvider());

#if MATTER_MINIMAL_DEVICE
    ESP_LOGI(TAG, "Matter minimal device init complete (Phase 1).");
#else
    ESP_LOGI(TAG, "Matter Bridge init complete (Phase 1).");
#endif
}

// Matter operational certs and CASE (SetEffectiveTime) need a wall clock in a sensible range.
// gettimeofday is often still 2000-01-01 (946684800) at esp_matter::start() because NTP runs
// later in Setup.cpp — that value must NOT be written into CHIP; it overwrites LKGT (~build time).
// Reject everything before 2020-01-01 UTC; use matterResyncChipWallClockAfterNtp() after NTP.
static constexpr int64_t kMatterPlausibleMinUnixSec = 1577836800; // 2020-01-01 00:00:00 UTC

/** ESP32 Matter: with CONFIG_ENABLE_SNTP_TIME_SYNC, CHIP reads wall time via gettimeofday().
 *  Push RTC/NTP-adjusted ESP time into CHIP so CASE Sigma3 (SetEffectiveTime) matches real UTC.
 */
static void matterSyncChipWallClockFromEsp()
{
    struct timeval tv {};
    if (gettimeofday(&tv, nullptr) != 0) {
        ESP_LOGW(TAG, "CHIP wall clock: gettimeofday failed");
        return;
    }
    if (tv.tv_sec < kMatterPlausibleMinUnixSec) {
        ESP_LOGW(TAG,
                 "CHIP wall clock: ESP time not plausible for Matter (tv_sec=%ld; need >= 2020 — "
                 "usually pre-NTP default). Not overwriting CHIP clock; LKGT/build time applies.",
                 (long) tv.tv_sec);
        return;
    }
    if (esp_matter::lock::chip_stack_lock(pdMS_TO_TICKS(MATTER_LOCK_TIMEOUT_MS)) != ESP_OK) {
        ESP_LOGW(TAG, "CHIP wall clock: chip stack lock timeout");
        return;
    }
    chip::System::Clock::Microseconds64 us(
        static_cast<uint64_t>(tv.tv_sec) * UINT64_C(1000000) + static_cast<uint64_t>(tv.tv_usec));
    CHIP_ERROR ce = chip::System::SystemClock().SetClock_RealTime(us);
    esp_matter::lock::chip_stack_unlock();
    if (ce == CHIP_NO_ERROR) {
        ESP_LOGI(TAG, "CHIP wall clock synced from ESP (Unix s=%ld)", (long)tv.tv_sec);
    } else {
        ESP_LOGW(TAG, "CHIP SetClock_RealTime failed: %" CHIP_ERROR_FORMAT, ce.Format());
    }
}

void matterResyncChipWallClockAfterNtp()
{
    if (!s_started) {
        return;
    }
    matterSyncChipWallClockFromEsp();
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

    // ── Verbose BLE/CHIP logging during commissioning debugging ───────────────
    // CONFIG_LOG_MAXIMUM_LEVEL is raised to 4 (DEBUG) at compile time, but the
    // global runtime default stays at INFO (CONFIG_LOG_DEFAULT_LEVEL=3) to keep
    // other components quiet. Here we selectively raise a handful of Matter/BLE
    // tags to DEBUG so that the full BTP handshake path is visible:
    //   • "chip[DL]"  — DeviceLayer: BLE GAP/GATT events, WiFi events, heartbeats
    //   • "chip[BLE]" — BleLayer: BTP engine, endpoint state machine
    //   • "Ble"       — BLEEndPoint / BTPEngine (DriveSending, ACK timers)
    //   • "NimBLE"    — host-controller activity (advertise, indicate, notify_tx)
    //   • "CHIP[DL]"  — uppercase variant used by some CHIP call sites
    //
    // This is what lets us see "Sending indication for CHIPoBLE TX…",
    // "Confirm received for CHIPoBLE TX…" and BTP state transitions during
    // the 15 s window where Apple Home would otherwise silently fail.
    esp_log_level_set("chip[DL]",  ESP_LOG_DEBUG);
    esp_log_level_set("chip[BLE]", ESP_LOG_DEBUG);
    esp_log_level_set("Ble",       ESP_LOG_DEBUG);
    esp_log_level_set("NimBLE",    ESP_LOG_DEBUG);
    esp_log_level_set("CHIP[DL]",  ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Verbose BLE/CHIP logging enabled for commissioning diagnosis.");

#if MATTER_LOG_EXTRA_CHIP_TAGS
    esp_log_level_set("chip[IN]", ESP_LOG_DEBUG);
    esp_log_level_set("chip[EM]", ESP_LOG_DEBUG);
    esp_log_level_set("chip[SC]", ESP_LOG_DEBUG);
    esp_log_level_set("chip[DMG]", ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Extra chip[IN/EM/SC/DMG] DEBUG enabled (MATTER_LOG_EXTRA_CHIP_TAGS).");
#endif

    esp_err_t err = esp_matter::start(on_device_event);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() failed: %s", esp_err_to_name(err));
        return;
    }

    s_started = true;

    matterSyncChipWallClockFromEsp();

    // Receive low-level BLE / commissioning ChipDeviceEvents that esp_matter does not pass
    // to on_device_event (verified on-device: CHIPoBLE RX writes in log but no hold ON).
    chip::DeviceLayer::PlatformMgr().AddEventHandler(platformBleHoldEventHandler, 0);
    matterSerialChipf("PlatformMgr: zweiter Event-Handler registriert (BLE-Details).");
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#if MATTER_BLE_GAP_DIAG_LISTENER
    esp_log_level_set(TAG_BLE_GAP, ESP_LOG_WARN);
    registerMatterBleGapDiagListener();
#elif MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE
    registerMatterBleGapSessionListener();
#endif

#if MATTER_WIFI_STA_OFF_DURING_BLE_GAP
    registerMatterBleGapWifiCoexListener();
#endif

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

#if MATTER_MINIMAL_DEVICE
    ESP_LOGI(TAG, "Matter minimal stack started — waiting for commissioning.");
#else
    ESP_LOGI(TAG, "Matter Bridge started — waiting for commissioning.");
#endif

    // Do not call syncBleGapRadioHoldFromStack() here — it drives mqttSetBleRadioHold(),
    // which uses FreeRTOS timers that are not created until Setup.cpp runs initTimers().

    // Apply initial reachability state for conditional endpoints (full bridge only)
#if !MATTER_MINIMAL_DEVICE
    matterUpdateConditionalEndpoints(storage.Salt_Chlor);
    s_salt_chl_was_active = storage.Salt_Chlor;
#endif
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

#if MATTER_MINIMAL_DEVICE
    // Single OnOff EP — no ElectricalMeasurement cluster on minimal device
    updateOnOff(s_ep_filt, FiltrationPump.IsRunning());
#else

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
    // true  = PoolMaster requests solar pool heating (same logic as HTTP /read + MQTT).
    {
        const bool solarRequest = poolSolarBridgeSolarModeRequest();
        if (solarRequest != s_last_solar_mode) {
            updateOnOff(s_ep_solar_mode, solarRequest);
            s_last_solar_mode = solarRequest;
            ESP_LOGI(TAG, "Solar-Mode-Request → %s", solarRequest ? "ON (pool heating)" : "OFF");
        }
    }

#endif  // !MATTER_MINIMAL_DEVICE

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
            // ── MATTER_FACTORY_RESET ────────────────────────────────────────
            //  Wipes all fabrics / NOCs / ACLs and reboots. Use when Apple Home
            //  (or any controller) refuses to add the device because it is
            //  "already paired" — i.e., leftover fabric state from an earlier
            //  commissioning attempt that was never successfully removed.
            else if (strcmp(s_line_buf, "MATTER_FACTORY_RESET") == 0) {
                if (matterFactoryReset()) {
                    Serial.println("[Matter] Factory reset scheduled — device will reboot in ~1 s.");
                } else {
                    Serial.println("[Matter] Factory reset FAILED (stack not started).");
                }
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
    const TickType_t periodFast = pdMS_TO_TICKS(100); // while uncommissioned: poll BLE link for GAP connect
    const TickType_t periodSlow = pdMS_TO_TICKS(MATTER_SYNC_PERIOD_MS);
    TickType_t       lastWake   = xTaskGetTickCount();
    bool             prev_scan_ble = false;

    for (;;) {
        matterYieldAppTasksIfChipobleBusy();
        const bool scanBle =
            (startTasks && s_started && chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);
        const TickType_t period = scanBle ? periodFast : periodSlow;
        if (scanBle != prev_scan_ble) {
            prev_scan_ble = scanBle;
            lastWake      = xTaskGetTickCount();
        }
        vTaskDelayUntil(&lastWake, period);

        // Process serial commands (SET_SOLAR_NODE etc.)
        handleSerialCommands();

        if (startTasks && s_started) {
            // bleRadioHoldPollWork posts to the CHIP Platform event queue. Doing that every
            // 100 ms while uncommissioned floods the queue — BLE GATT writes then fail with
            // "Failed to post event to CHIP Platform event queue" / 0x01000000 (see serial log).
#if !MATTER_NO_MQTT_UNTIL_COMMISSIONED
            if (scanBle) {
                const CHIP_ERROR sErr = chip::DeviceLayer::PlatformMgr().ScheduleWork(bleRadioHoldPollWork, 0);
                if (sErr != CHIP_NO_ERROR) {
                    ESP_LOGW(TAG, "ScheduleWork(bleRadioHoldPollWork) failed: %" CHIP_ERROR_FORMAT, sErr.Format());
                }
            }
#endif

            // Skip periodic sync while BLE commissioning is in progress:
            // each attribute::update() takes the CHIP stack lock and generates
            // subscription report traffic, competing with the BTP/PASE handshake
            // for both the CHIP task and the shared BLE/WiFi radio.
            if (matterIsBleCommissioning()) {
                ESP_LOGD(TAG, "MatterSync skipped — BLE commissioning in progress.");
            } else {
                matterBridgeSync();
            }

#ifdef CONFIG_ESP_MATTER_CONTROLLER_ENABLE
            // Start subscriptions once Matter is running and config is available
            if (s_solar_node_id != 0 && !s_solar_subscribed && !matterIsBleCommissioning()) {
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

// =============================================================================
//  Public: Factory reset — wipe all Matter fabrics & reboot
// =============================================================================
//
//  Implementation detail:
//    chip::Server::GetInstance().ScheduleFactoryReset() calls
//    ConfigurationMgr().InitiateFactoryReset() which
//      1. Erases all persistent Matter state
//         (fabrics, NOC chain, ACL, group keys, NVS counters).
//      2. Schedules an esp_restart() after a short delay (~500 ms) so the
//         response to the caller can still be flushed.
//    We wrap it with ScheduleWork so it always runs in the CHIP task context,
//    regardless of which task calls matterFactoryReset() (WebUI / Serial CLI).
//
// Direct NVS fallback: wipes exactly the same namespaces as Matter's
// ConfigurationManagerImpl::DoFactoryReset() — without requiring the CHIP task
// to run. We use this because when the CHIP event loop is starved (e.g. during
// a failed BTP handshake), ScheduleWork-based resets never actually execute.
//
// Namespaces cleared (all in default NVS partition):
//   • chip-config    — general device config + fabric descriptors
//   • chip-counters  — reliable-messaging counters, session counters
//   • chip-kvs       — key-value store (fabric keys, ACL entries, group keys)
// We deliberately KEEP:
//   • chip-factory   — persistent device identity (setup code, DAC, CD, VID/PID)
//   • namespaces used by PoolMaster for its own settings
//
// After NVS erase we also call esp_wifi_restore() to clear stored STA creds
// that Matter wrote at commissioning time, then esp_restart() to reboot.
static void directNvsFactoryReset()
{
    Serial.println("[Matter] Direct NVS factory reset: erasing chip-config / chip-counters / chip-kvs…");

    const char * namespaces_to_clear[] = { "chip-config", "chip-counters", "chip-kvs" };
    for (const char * ns : namespaces_to_clear) {
        nvs_handle_t h;
        esp_err_t e = nvs_open(ns, NVS_READWRITE, &h);
        if (e == ESP_OK) {
            esp_err_t ee = nvs_erase_all(h);
            esp_err_t ec = nvs_commit(h);
            nvs_close(h);
            Serial.printf("[Matter]   namespace '%s': erase=%s commit=%s\n",
                          ns, esp_err_to_name(ee), esp_err_to_name(ec));
        } else if (e == ESP_ERR_NVS_NOT_FOUND) {
            Serial.printf("[Matter]   namespace '%s': not present (already clean)\n", ns);
        } else {
            Serial.printf("[Matter]   namespace '%s': nvs_open failed: %s\n",
                          ns, esp_err_to_name(e));
        }
    }

    // Clear Matter-provisioned WiFi credentials (safe: PoolMaster re-writes them on boot)
    esp_err_t we = esp_wifi_restore();
    Serial.printf("[Matter]   esp_wifi_restore: %s\n", esp_err_to_name(we));

    Serial.println("[Matter] Direct NVS factory reset complete — rebooting in 500 ms…");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static void factoryResetWork(intptr_t /*arg*/)
{
    Serial.println("[Matter] factoryResetWork() running on CHIP task — calling ScheduleFactoryReset()");
    ESP_LOGW(TAG, "Matter factory reset triggered — wiping fabrics, ACL, NOC chain…");
    chip::Server::GetInstance().ScheduleFactoryReset();
}

// Watchdog task: if the CHIP task doesn't reboot the device within the
// graceful window, we do a direct NVS wipe + esp_restart() ourselves.
static void factoryResetWatchdog(void * /*arg*/)
{
    // Give the CHIP task 3 seconds to process ScheduleFactoryReset() and call
    // esp_restart(). If we're still alive afterwards, the CHIP task is stuck.
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("[Matter] CHIP task did not reboot within 3 s — falling back to direct NVS wipe.");
    directNvsFactoryReset();
    // esp_restart() doesn't return, but guard against it:
    vTaskDelete(nullptr);
}

bool matterFactoryReset()
{
    Serial.println("[Matter] matterFactoryReset() called.");

    // Even if the CHIP stack isn't started (s_started == false) we still need
    // to wipe NVS so the next boot comes up fresh. Don't bail early.
    if (!s_started) {
        ESP_LOGE(TAG, "matterFactoryReset(): Matter stack not started — doing direct NVS wipe.");
        Serial.println("[Matter] Matter stack not started — skipping ScheduleWork, wiping NVS directly.");
        directNvsFactoryReset();
        return true; // never returns (reboots)
    }

    // Step 1: schedule the graceful reset path (CHIP task will call esp_restart)
    auto err = chip::DeviceLayer::PlatformMgr().ScheduleWork(factoryResetWork, 0);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "ScheduleWork for factory reset failed: %" CHIP_ERROR_FORMAT, err.Format());
        Serial.printf("[Matter] ScheduleWork failed (%s) — falling back to direct NVS wipe.\n",
                      err.Format());
        directNvsFactoryReset();
        return true; // never returns (reboots)
    }

    // Step 2: start a watchdog that reboots us even if the CHIP task is stuck
    // (this was the observed failure mode — CHIP task starved during commissioning
    // and not processing queued events).
    TaskHandle_t dummy;
    BaseType_t ok = xTaskCreatePinnedToCore(factoryResetWatchdog, "FRWatchdog", 4096, nullptr,
                                            10, &dummy, 1); // high priority, Core 1
    if (ok != pdPASS) {
        Serial.println("[Matter] Watchdog task creation failed — doing immediate direct NVS wipe.");
        directNvsFactoryReset();
        return true;
    }

    Serial.println("[Matter] Factory reset scheduled; watchdog will force-wipe + reboot in 3 s if CHIP stalls.");
    return true;
}

#endif // MATTER_ENABLED
