#pragma once
// =============================================================================
//  MatterBridge.h — PoolMaster Matter Bridge
// =============================================================================
//  Exposes pool devices as a Matter Bridge (Aggregator) with child endpoints:
//    EP_FILT  – Filterpumpe         (OnOff + ElectricalMeasurement)
//    EP_PH    – PH-Pumpe            (OnOff)
//    EP_HEAT  – Wärmepumpe          (OnOff + ElectricalMeasurement)
//    EP_SALT  – Salzelektrolyse     (OnOff + ElectricalMeasurement, conditional)
//    EP_CHL   – Chlor-Pumpe         (OnOff, conditional)
//
//  Native (non-bridged) endpoints for SolarControl integration:
//    EP_POOL_TEMP   – Pool-Wassertemperatur     (TemperatureMeasurement, read by SolarControl)
//    EP_POOL_SOLL   – Pool-Solltemperatur       (TemperatureMeasurement, read by SolarControl)
//    EP_SOLAR_MODE  – Solar-Modus-Anforderung   (OnOff: true = Pool-Heizung anfordern)
//
//  Matter Controller (requires CONFIG_ESP_MATTER_CONTROLLER_ENABLE=y):
//    - Subscriptions to SolarControl temperature/status endpoints
//    - Commands to SolarControl circulation and illumination endpoints
//    - Serial command: SET_SOLAR_NODE <nodeId_hex> <epPump> <epValve> <epCirc> <epIllum>
//
//  IMPORTANT: Compile only when -DMATTER_ENABLED is set in build flags.
// =============================================================================

#ifdef MATTER_ENABLED

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Public API — bridge lifecycle
// ---------------------------------------------------------------------------

/**
 * @brief Phase-1 init: create Matter node + all endpoints.
 *        Call BEFORE WiFi is connected (but after NVS is initialised).
 *        Loads SolarControl config (NodeId + EP IDs) from NVS.
 */
void matterBridgeInit();

/**
 * @brief Phase-2 start: launch the Matter stack and begin BLE advertising.
 *        Call AFTER WiFi is connected.
 *        Prints QR-code / manual pairing code to Serial.
 */
void matterBridgeStart();

/**
 * @brief Sync current pool state to Matter attribute values.
 *        Updates Pool_Temp, Pool_Soll, Solar_Mode_Request endpoints.
 *        Called from MatterSyncTask every MATTER_SYNC_PERIOD_MS.
 */
void matterBridgeSync();

/**
 * @brief Update the "reachable" attribute of the conditional Salt / Chl endpoints.
 * @param active  true → endpoints reachable, false → hidden in controller
 */
void matterUpdateConditionalEndpoints(bool active);

/**
 * @brief FreeRTOS Task T14: periodic Matter sync + serial command handler.
 *        Runs on Core 1 at MATTER_SYNC_PERIOD_MS interval.
 */
void MatterSyncTask(void *pvParameters);

// ---------------------------------------------------------------------------
// Public API — commissioning info (for WebUI)
// ---------------------------------------------------------------------------

/**
 * @brief Number of commissioned fabrics (0 = not yet commissioned).
 */
uint8_t matterFabricCount();

/**
 * @brief Get the base38 QR payload string (e.g. "MT:YXXXXXXXXX").
 * @param buf   Output buffer.
 * @param size  Buffer size (≥ 96 bytes recommended).
 * @returns true on success.
 */
bool matterGetQRCode(char* buf, size_t size);

/**
 * @brief Get the manual pairing code (11-digit or 21-digit decimal string).
 * @param buf   Output buffer.
 * @param size  Buffer size (≥ 32 bytes recommended).
 * @returns true on success.
 */
bool matterGetManualPairingCode(char* buf, size_t size);

/**
 * @brief Open a basic commissioning window (DNS-SD only).
 * @param timeoutSec  Window duration in seconds (default 900 = 15 min).
 * @returns true if the window was opened successfully.
 */
bool matterOpenCommissioningWindow(uint16_t timeoutSec = 900);

/**
 * @brief Reflects Matter “radio quiet / MQTT hold” state used by PublishTopic.
 *
 *        When `MATTER_NO_MQTT_UNTIL_COMMISSIONED` is enabled, this is true for
 *        the entire time **no fabric** exists (`FabricCount()==0`) — not only
 *        while a phone is connected over BLE. That matches `mqttSetBleRadioHold`
 *        and intentionally suppresses noisy MQTT during possible commissioning.
 *
 *        Do **not** use this to gate unrelated UI (e.g. Nextion); it is not a
 *        “CHIPoBLE link up” detector. For BLE link state use ConnectivityMgr
 *        from the CHIP task or `NumBLEConnections()` where appropriate.
 *
 *        Safe to call from any task (atomic read).
 */
bool matterIsBleCommissioning();

/**
 * @brief Yield CPU when a CHIPoBLE commissioning session is in progress (no fabric yet).
 *
 *        Intended at the top of Core-1 control loops (PoolMaster, polling, …).
 *        Controlled by MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE / MATTER_CHIPOBLE_APP_YIELD_MS.
 *        No-op when Matter is disabled.
 */
void matterYieldAppTasksIfChipobleBusy();

/**
 * @brief Apply MATTER_NO_MQTT_UNTIL_COMMISSIONED radio/MQTT hold after mqtt_comm
 *        initTimers() has created the reconnect timers. Call once from Setup —
 *        must not run before initTimers() or xTimerStop asserts on a null handle.
 */
void matterApplyRadioHoldAfterTimersReady();

/**
 * @brief Wipe all Matter fabrics, ACL entries, group keys, NOC chain and
 *        counter state, then reboot.
 *
 *        Use this when Apple Home (or any other controller) reports
 *        "already paired / already added to another Home" — this happens when
 *        leftover fabric entries in NVS survive an incomplete commissioning
 *        or a remove-from-Home that did not reach the device.
 *
 *        The device will perform a full ESP restart after the reset completes
 *        (≈ 1-2 s). After reboot the commissioning window opens automatically
 *        again because no fabric is present.
 *
 * @returns true if the reset was successfully scheduled on the CHIP task
 *          (the function itself returns before the reboot happens).
 */
bool matterFactoryReset();

// ---------------------------------------------------------------------------
// Public API — SolarControl Matter Controller (CONFIG_ESP_MATTER_CONTROLLER_ENABLE)
// ---------------------------------------------------------------------------

/**
 * @brief Subscribe to all relevant SolarControl endpoints.
 *        Sets up attribute reports for temperatures, pump/valve status.
 *        Reports are delivered via solarReportCallback and written to storage.
 *        Safe to call multiple times (re-subscribes if config changes).
 * @param solarNodeId  SolarControl's Matter NodeId (from SET_SOLAR_NODE command)
 */
void subscribeToSolarControl(uint64_t solarNodeId);

/**
 * @brief Send OnOff command to the SolarControl circulation endpoint (EP_CIRC).
 *        The target NodeId and endpoint are read from NVS.
 * @param on  true = start circulation, false = stop
 */
void sendCirculationCommand(bool on);

/**
 * @brief Send OnOff command to the SolarControl illumination endpoint (EP_ILLUM).
 *        The target NodeId and endpoint are read from NVS.
 * @param on  true = turn on illumination, false = turn off
 */
void sendIlluminationCommand(bool on);

#else  // !MATTER_ENABLED

// Non-Matter builds: provide a no-op stub so callers (Publish.cpp etc.)
// don't need to sprinkle #ifdef MATTER_ENABLED around every call site.
#include <stdbool.h>
static inline bool matterIsBleCommissioning() { return false; }
static inline void matterYieldAppTasksIfChipobleBusy() {}

#endif // MATTER_ENABLED
