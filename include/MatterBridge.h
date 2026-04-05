#pragma once
// =============================================================================
//  MatterBridge.h — PoolMaster Matter Bridge
// =============================================================================
//  Exposes pool devices as a Matter Bridge (Aggregator) with 5 child endpoints:
//    EP_FILT  – Filterpumpe         (OnOff + ElectricalMeasurement)
//    EP_PH    – PH-Pumpe            (OnOff)
//    EP_HEAT  – Wärmepumpe          (OnOff + ElectricalMeasurement)
//    EP_SALT  – Salzelektrolyse     (OnOff + ElectricalMeasurement, conditional)
//    EP_CHL   – Chlor-Pumpe         (OnOff, conditional)
//
//  Conditional endpoints (SALT/CHL) have their "reachable" attribute set to
//  false when storage.Salt_Chlor == false, hiding them from Matter controllers.
//
//  Commands from Matter are fed into the existing queueIn FreeRTOS queue using
//  the same JSON format as the MQTT API — zero coupling to existing code paths.
//
//  IMPORTANT: Compile only when -DMATTER_ENABLED is set in build flags.
// =============================================================================

#ifdef MATTER_ENABLED

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Phase-1 init: create Matter node + all endpoints.
 *        Call BEFORE WiFi is connected (but after NVS is initialised).
 *        Sets up the attestation credentials provider.
 */
void matterBridgeInit();

/**
 * @brief Phase-2 start: launch the Matter stack and begin BLE advertising
 *        for commissioning.  Call AFTER WiFi is connected.
 *        Prints QR-code / manual pairing code to Serial.
 */
void matterBridgeStart();

/**
 * @brief Sync current pool state (pump running flags, power measurements)
 *        to Matter attribute values.
 *        Called from MatterSyncTask every MATTER_SYNC_PERIOD_MS.
 *        Thread-safe via esp_matter::lock.
 */
void matterBridgeSync();

/**
 * @brief Update the "reachable" attribute of the conditional Salt / Chl
 *        endpoints.  Call whenever storage.Salt_Chlor changes.
 * @param active  true  → endpoints are reachable (Salt/Chlor enabled)
 *                false → endpoints marked unreachable (hidden in controller)
 */
void matterUpdateConditionalEndpoints(bool active);

/**
 * @brief FreeRTOS Task T14: periodic Matter state synchronisation.
 *        Runs on Core 1 at MATTER_SYNC_PERIOD_MS interval.
 *        Created by createTasks() in Setup.cpp.
 */
void MatterSyncTask(void *pvParameters);

#endif // MATTER_ENABLED
