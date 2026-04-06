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

#endif // MATTER_ENABLED
