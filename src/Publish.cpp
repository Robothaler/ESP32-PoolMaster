// MQTT publish tasks
// - PublishSettings: Publishes system settings to Homebridge-compatible topics
// - PublishMeasures: Publishes system measures to Homebridge-compatible topics and SolarControl-compatible topics
// The tasks wait for notification to publish. PublishMeasures also has a timeout for regular publishing.

#undef __STRICT_ANSI__
#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"
#include "MatterBridge.h"   // matterIsBleCommissioning() — no-op stub in non-Matter builds

// Size of the buffer to store outgoing JSON messages
#define PAYLOAD_BUFFER_LENGTH 256 // Increased to accommodate potential larger payloads

// Homebridge-compatible topics
static const char* HomeTopicPumps         = POOLTOPIC"Pumps";
static const char* HomeTopicValves        = POOLTOPIC"Valves";
static const char* HomeTopicModes         = POOLTOPIC"Modes";
static const char* HomeTopicErrors        = POOLTOPIC"Errors";
static const char* HomeTopicRuntimes      = POOLTOPIC"Runtimes";
static const char* HomeTopicMeasurements  = POOLTOPIC"Measurements";
static const char* HomeTopicTemperatures  = POOLTOPIC"Temperatures";
static const char* HomeTopicSetSystem     = POOLTOPIC"Settings/System";
static const char* HomeTopicSetPID        = POOLTOPIC"Settings/PID";
static const char* HomeTopicSetCalib      = POOLTOPIC"Settings/Calibration";
static const char* HomeTopicSetTanks      = POOLTOPIC"Settings/Tanks";
static const char* HomeTopicSetModes      = POOLTOPIC"Settings/Modes";
static const char* HomeTopicSystem        = POOLTOPIC"System";

#ifndef MATTER_ENABLED
// SolarControl-compatible MQTT topic — replaced by Matter TemperatureMeasurement
// endpoint (s_ep_pool_temp) when MATTER_ENABLED.
static const char* PoolTopicTemperature   = "POOL/temperature";
#endif

// External MotorValve instances
extern MotorValve ELD_Treppe;
extern MotorValve ELD_Hinten;
extern MotorValve WP_Vorlauf;
extern MotorValve WP_Mischer;
extern MotorValve Bodenablauf;
extern MotorValve Solarvalve;

int freeRam(void);
void stack_mon(UBaseType_t&);

// Publishes a JSON document or string payload to a single topic.
// BUG FIX: uses the dedicated mqttMutex (not the I2C mutex) so MQTT publishing
// cannot block I2C operations in CombinedPollingTask / PCF8574Manager.
//
// BLE-COMMISSIONING GATE: The ESP32-S3 has a single antenna shared between
// BLE and WiFi. During Matter BLE commissioning the BTP handshake requires
// ~11 reliable GATT indications per second to complete in <15 s; heavy MQTT
// traffic (retained publishes, PUBACKs, TCP retransmits) starves BLE on the
// air and causes Apple Home to time out. While `matterIsBleCommissioning()`
// is true we skip all publishes — the measurements are retained on the
// broker from the previous cycle and the task will catch up automatically
// as soon as the commissioning BLE session ends (~30-90 s typical).
// In non-Matter builds matterIsBleCommissioning() is an inline `false` stub,
// so this branch compiles away to nothing.
void PublishTopic(const char* topic, const char* payload, size_t n)
{
    if (!mqttMutex) return; // Guard against pre-init calls

    if (matterIsBleCommissioning()) {
        Debug.print(DBG_DEBUG, "[PublishTopic] Skipped (BLE commissioning in progress): %s", topic);
        return;
    }

    if (!xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000))) {
        Debug.print(DBG_ERROR, "[PublishTopic] Failed to take mqttMutex for topic %s", topic);
        return;
    }
    bool publishSuccess = mqttClient.publish(topic, 1, true, payload, n) != 0;
    if (publishSuccess) {
        Debug.print(DBG_DEBUG, "[PublishTopic] Success to %s: %s - size: %d", topic, payload, n);
    } else {
        Debug.print(DBG_ERROR, "[PublishTopic] Failed to publish to %s: %s", topic, payload);
    }
    xSemaphoreGive(mqttMutex);
}

// Publishes system settings to MQTT broker
void SettingsPublish(void *pvParameters)
{
    Debug.print(DBG_INFO, "[TASKS] SettingsPublish started on core %d", xPortGetCoreID());
    
    // Wait for startTasks with timeout to avoid infinite loop
    uint32_t timeout = 60000; // 60 seconds
    while (!startTasks && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout -= 1000;
        Debug.print(DBG_WARNING, "[SettingsPublish] Waiting for startTasks, timeout remaining: %d ms", timeout);
    }
    if (!startTasks) {
        Debug.print(DBG_ERROR, "[SettingsPublish] startTasks timeout, proceeding anyway");
    }
    
    Debug.print(DBG_DEBUG, "[TASKS] SettingsPublish running...");
    vTaskDelay(DT12); // Scheduling offset 

    uint32_t mod1 = xTaskGetTickCount() % 1000;
    uint32_t mod2;
    TickType_t waitTime;

    static UBaseType_t hwm = 0;

#ifdef CHRONO
    unsigned long td;
    int t_act = 0, t_min = 999, t_max = 0;
    float t_mean = 0.;
    int n = 1;
#endif
  
    // Use a single JSON document to reduce memory usage
    StaticJsonDocument<JSON_OBJECT_SIZE(16)> root; // Increased size for Calibration (13 keys)

    for (;;) {
#ifdef CHRONO
        td = millis();
#endif

        Debug.print(DBG_DEBUG, "[PublishSettings] start, MQTT connected: %d", mqttClient.connected());

        if (mqttClient.connected()) {
            char Payload[PAYLOAD_BUFFER_LENGTH];
            size_t n;

            // System settings
            root.clear();
            root["FW"] = Firmw;                           // Firmware version
            root["FSta"] = storage.FiltrationStart;       // Filtration start time (hour)
            root["FStaM"] = storage.FiltrationStartMin;   // Filtration start time (minute)
            root["FDu"] = storage.FiltrationDuration;     // Filtration duration (hours)
            root["FStoM"] = storage.FiltrationStopMax;    // Maximum filtration stop time (hour)
            root["FSto"] = storage.FiltrationStop;        // Filtration stop time (hour)
            root["pHUTL"] = storage.PhPumpUpTimeLimit / 60; // pH pump uptime limit (minutes)
            root["ChlUTL"] = storage.ChlPumpUpTimeLimit / 60; // Chlorine pump uptime limit (minutes)
            root["SStaM"] = storage.SolarStartMin;        // Solar pump start time (minute)
            root["SStoM"] = storage.SolarStopMax;         // Solar pump stop time (minute)
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] System payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] System payload buffer overflow");
            }
            PublishTopic(HomeTopicSetSystem, Payload, n);

            // PID settings
            root.clear();
            root["pHWS"] = storage.PhPIDWindowSize / 1000 / 60; // pH PID window size (minutes)
            root["ChlWS"] = storage.OrpPIDWindowSize / 1000 / 60; // Chlorine PID window size (minutes)
            root["pHSP"] = storage.Ph_SetPoint * 100;           // pH setpoint (x100)
            root["OrpSP"] = storage.Orp_SetPoint;               // ORP setpoint (mV)
            root["WSP"] = storage.WaterTemp_SetPoint * 100;     // Water temperature setpoint (°C x100)
            root["WLT"] = storage.WaterTempLowThreshold * 100;  // Water temperature low threshold (°C x100)
            root["PSIHT"] = storage.PSI_HighThreshold * 100;    // High pressure threshold (bar x100)
            root["PSIMT"] = storage.PSI_MedThreshold * 100;     // Medium pressure threshold (bar x100)
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] PID payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] PID payload buffer overflow");
            }
            PublishTopic(HomeTopicSetPID, Payload, n);

            // Calibration settings
            root.clear();
            root["pHC0"] = storage.pHCalibCoeffs0; // pH calibration coefficient 0
            root["pHC1"] = storage.pHCalibCoeffs1; // pH calibration coefficient 1
            root["OrpC0"] = storage.OrpCalibCoeffs0; // ORP calibration coefficient 0
            root["OrpC1"] = storage.OrpCalibCoeffs1; // ORP calibration coefficient 1
            root["PSIC0"] = storage.PSICalibCoeffs0; // Pressure calibration coefficient 0
            root["PSIC1"] = storage.PSICalibCoeffs1; // Pressure calibration coefficient 1
            root["SaltC0"] = storage.SaltCurrentCalibCoeffs0; // Salt current calibration coefficient 0
            root["SaltC1"] = storage.SaltCurrentCalibCoeffs1; // Salt current calibration coefficient 1
            root["FiltC0"] = storage.FilterCurrentCalibCoeffs0; // Filter current calibration coefficient 0
            root["FiltC1"] = storage.FilterCurrentCalibCoeffs1; // Filter current calibration coefficient 1
            root["HeatC0"] = storage.HeatCurrentCalibCoeffs0; // Heat current calibration coefficient 0
            root["HeatC1"] = storage.HeatCurrentCalibCoeffs1; // Heat current calibration coefficient 1
            root["CellConst"] = storage.CellConstant; // Salt cell constant
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] Calibration payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] Calibration payload buffer overflow");
            }
            PublishTopic(HomeTopicSetCalib, Payload, n);

            // PID tuning settings
            root.clear();
            root["pHKp"] = storage.Ph_Kp;   // pH PID proportional gain
            root["pHKi"] = storage.Ph_Ki;   // pH PID integral gain
            root["pHKd"] = storage.Ph_Kd;   // pH PID derivative gain
            root["OrpKp"] = storage.Orp_Kp; // ORP PID proportional gain
            root["OrpKi"] = storage.Orp_Ki; // ORP PID integral gain
            root["OrpKd"] = storage.Orp_Kd; // ORP PID derivative gain
            root["Dpid"] = storage.DelayPIDs; // PID delay (ms)
            root["PubP"] = storage.PublishPeriod / 1000; // Publish period (seconds)
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] PID tuning payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] PID tuning payload buffer overflow");
            }
            PublishTopic(HomeTopicSetPID, Payload, n);

            // Tank settings
            root.clear();
            root["pHTV"] = storage.pHTankVol;  // pH tank volume (liters)
            root["ChlTV"] = storage.ChlTankVol; // Chlorine tank volume (liters)
            root["pHFR"] = storage.pHPumpFR;   // pH pump flow rate (mL/min)
            root["OrpFR"] = storage.ChlPumpFR; // Chlorine pump flow rate (mL/min)
            root["PoolVol"] = storage.PoolVolume; // Pool volume (liters)
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] Tanks payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] Tanks payload buffer overflow");
            }
            PublishTopic(HomeTopicSetTanks, Payload, n);

            // Mode settings
            root.clear();
            root["SolLoc"] = storage.SolarLocExt;         // Solar location (external/internal)
            root["SaltChl"] = storage.Salt_Chlor;         // Salt chlorination on/off
            root["SaltP"] = storage.SaltPolarity;         // Salt polarity
            root["ValvS"] = storage.ValveSwitch;          // Valve switch state
            root["WFUTL"] = storage.WaterFillUpTimeLimit / 1000; // Water fill uptime limit (seconds)
            root["WFDur"] = storage.WaterFillDuration / 1000;   // Water fill duration (seconds)
            root["WFAnC"] = storage.WaterFillAnCon;       // Water fill analog control
            root["SaltD"] = storage.SaltDiff;             // Salt difference
            n = serializeJson(root, Payload);
            Debug.print(DBG_DEBUG, "[SettingsPublish] Modes payload size: %d", n);
            if (n >= PAYLOAD_BUFFER_LENGTH) {
                Debug.print(DBG_ERROR, "[SettingsPublish] Modes payload buffer overflow");
            }
            PublishTopic(HomeTopicSetModes, Payload, n);
        } else {
            Debug.print(DBG_ERROR, "[SettingsPublish] MQTT not connected");
        }

        Debug.print(DBG_DEBUG, "[memCheck]: %db", freeRam());

#ifdef CHRONO
        t_act = millis() - td;
        if (t_act > t_max) t_max = t_act;
        if (t_act < t_min) t_min = t_act;
        t_mean += (t_act - t_mean) / n;
        ++n;
        Debug.print(DBG_INFO, "[PublishSettings] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
#endif

        stack_mon(hwm);
        // Wait for notification or timeout after 30 seconds to ensure periodic publishing
        if (ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(30000)) == 0) {
            Debug.print(DBG_WARNING, "[SettingsPublish] No notification received, publishing periodically");
        }
        mod2 = xTaskGetTickCount() % 1000;
        if (mod2 <= mod1)
            waitTime = mod1 - mod2;
        else
            waitTime = 1000 + mod1 - mod2;
        vTaskDelay(waitTime);
    }
}

//PublishData loop. Publishes system info/data to MQTT broker every XX secs (30 secs by default)
//or when notified.
//There is two timing computations:
//-If notified, wait for the next offset to always being at the same place in the scheduling cycle
//-Then compute the time spent to do the job and reload the timeout for the next cycle

// Publishes system measures to MQTT broker
void MeasuresPublish(void *pvParameters)
{
    Debug.print(DBG_INFO, "[TASKS] PublishMeasures started on core %d", xPortGetCoreID());
    
    // Wait for startTasks with timeout to avoid infinite loop
    uint32_t timeout = 60000; // 60 seconds
    while (!startTasks && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout -= 1000;
        Debug.print(DBG_WARNING, "[PublishMeasures] Waiting for startTasks, timeout remaining: %d ms", timeout);
    }
    if (!startTasks) {
        Debug.print(DBG_ERROR, "[PublishMeasures] startTasks timeout, proceeding anyway");
    }
    
    Debug.print(DBG_DEBUG, "[TASKS] PublishMeasures running...");
    vTaskDelay(DT11);

    uint32_t mod1 = xTaskGetTickCount() % 1000;
    TickType_t WaitTimeOut;
    TickType_t StartTime;
    TickType_t StopTime;
    TickType_t DeltaTime;
    uint32_t rc;
    uint32_t mod2;
    TickType_t waitTime;

    static UBaseType_t hwm = 0;

#ifdef CHRONO
    unsigned long td;
    int t_act = 0, t_min = 999, t_max = 0;
    float t_mean = 0.;
    int n = 1;
#endif

    WaitTimeOut = (TickType_t)storage.PublishPeriod / portTICK_PERIOD_MS;

    // Use separate JSON documents for each topic to manage memory efficiently
    StaticJsonDocument<JSON_OBJECT_SIZE(15)> root;         // For Measurements
    StaticJsonDocument<JSON_OBJECT_SIZE(12)> tempRoot;     // For Temperatures
    StaticJsonDocument<JSON_OBJECT_SIZE(8)> pumpsRoot;     // For Pumps
    StaticJsonDocument<JSON_OBJECT_SIZE(6)> valvesRoot;    // For Valves
    StaticJsonDocument<JSON_OBJECT_SIZE(12)> modesRoot;    // For Modes
    StaticJsonDocument<JSON_OBJECT_SIZE(8)> errorsRoot;    // For Errors
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> runtimesRoot; // For Runtimes
    StaticJsonDocument<JSON_OBJECT_SIZE(3)> systemRoot;    // For System

    // Track last published values for change detection
#ifndef MATTER_ENABLED
    static float lastWaterTemp = -100.0; // MQTT POOL/temperature change tracking
#endif
    static StaticJsonDocument<JSON_OBJECT_SIZE(12)> lastTempRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(15)> lastMeasurementsRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(8)> lastPumpsRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(6)> lastValvesRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(12)> lastModesRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(8)> lastErrorsRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(10)> lastRuntimesRoot;
    static StaticJsonDocument<JSON_OBJECT_SIZE(3)> lastSystemRoot;

    // Initialize last published documents
    lastTempRoot.clear();
    lastMeasurementsRoot.clear();
    lastPumpsRoot.clear();
    lastValvesRoot.clear();
    lastModesRoot.clear();
    lastErrorsRoot.clear();
    lastRuntimesRoot.clear();
    lastSystemRoot.clear();

    for (;;) {
        rc = ulTaskNotifyTake(pdFALSE, WaitTimeOut);

        if (rc != 0) {
            mod2 = xTaskGetTickCount() % 1000;
            if (mod2 <= mod1)
                waitTime = mod1 - mod2;
            else
                waitTime = 1000 + mod1 - mod2;
            vTaskDelay(waitTime);
        }

        StartTime = xTaskGetTickCount();
        Debug.print(DBG_DEBUG, "[PublishMeasures] start, MQTT connected: %d", mqttClient.connected());

#ifdef CHRONO
        td = millis();
#endif

        if (mqttClient.connected()) {
            char Payload[PAYLOAD_BUFFER_LENGTH];
            size_t n;

            // Temperatures
            tempRoot.clear();
            tempRoot["WaterS"] = storage.WaterSTemp * 100;   // Main pool water temperature (°C x100)
            tempRoot["WaterI"] = storage.WaterITemp * 100;   // Inlet water temperature (°C x100)
            tempRoot["WaterB"] = storage.WaterBTemp * 100;   // Bottom water temperature (°C x100)
            tempRoot["WaterWP"] = storage.WaterWPTemp * 100; // Heat pump water temperature (°C x100)
            tempRoot["WaterWT"] = storage.WaterWTTemp * 100; // Heat exchanger water temperature (°C x100)
            tempRoot["AirIn"] = storage.AirInTemp * 100;     // Air inlet temperature (°C x100)
            tempRoot["Solar"] = storage.SolarTemp * 100;     // Solar collector temperature (°C x100)
            tempRoot["SolarVL"] = storage.SolarVLTemp * 100; // Solar forward flow temperature (°C x100)
            tempRoot["SolarRL"] = storage.SolarRLTemp * 100; // Solar return flow temperature (°C x100)
            tempRoot["Air"] = storage.AirTemp * 100;         // Outdoor air temperature (°C x100)
            tempRoot["Hum"] = storage.AirHum * 100;          // Air humidity (% x100)
            tempRoot["Press"] = storage.AirPress * 100;      // Air pressure (hPa x100)

            // Check for temperature changes
            bool tempChanged = false;
            if (!lastTempRoot.isNull()) {
                for (const auto& item : tempRoot.as<JsonObject>()) {
                    if (lastTempRoot[item.key()] != item.value()) {
                        tempChanged = true;
                        break;
                    }
                }
            } else {
                tempChanged = true; // Publish on first iteration
            }
            if (tempChanged) {
                n = serializeJson(tempRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Temperatures payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Temperatures payload buffer overflow");
                }
                PublishTopic(HomeTopicTemperatures, Payload, n);
                lastTempRoot = tempRoot;
            }

#ifndef MATTER_ENABLED
            // Publish POOL/temperature for SolarControl via MQTT.
            // When MATTER_ENABLED this is handled by the EP_POOL_TEMP Matter endpoint
            // in MatterBridge.cpp (matterBridgeSync → updateTemperature).
            if (fabs(storage.WaterSTemp - lastWaterTemp) >= 0.1) {
                char tempPayload[10];
                snprintf(tempPayload, sizeof(tempPayload), "%.1f", storage.WaterSTemp);
                PublishTopic(PoolTopicTemperature, tempPayload, strlen(tempPayload));
                lastWaterTemp = storage.WaterSTemp;
            }
#endif

            // Measurements (non-temperature)
            root.clear();
            root["pH"] = storage.PhValue * 100; // pH value (x100)
            root["Orp"] = storage.OrpValue;     // ORP value (mV)
            root["PSI"] = storage.PSIValue * 100; // Pressure (bar x100)
            root["FLOW"] = storage.FLOWValue;   // Flow rate (L/min)
            root["FLOW2"] = storage.FLOW2Value; // Secondary flow rate (L/min)
            root["pHTF"] = PhPump.GetTankFill(); // pH tank fill level (%)
            root["FiltC"] = storage.FilterCurrentValue; // Filter current (mA)
            root["HeatC"] = storage.HeatCurrentValue;   // Heater current (mA)
            if (!storage.Salt_Chlor) {
                root["ChlTF"] = ChlPump.GetTankFill(); // Chlorine tank fill level (%)
            } else {
                root["SaltC"] = storage.SaltCurrentValue;   // Salt current (mA)
                root["SaltConc"] = storage.SaltConcentration; // Salt concentration (g/L)
                root["SaltNeed"] = storage.SaltNeeded;       // Salt needed (boolean)
            }
            // Check for measurement changes
            bool measChanged = false;
            if (!lastMeasurementsRoot.isNull()) {
                for (const auto& item : root.as<JsonObject>()) {
                    if (lastMeasurementsRoot[item.key()] != item.value()) {
                        measChanged = true;
                        break;
                    }
                }
            } else {
                measChanged = true; // Publish on first iteration
            }
            if (measChanged) {
                n = serializeJson(root, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Measurements payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Measurements payload buffer overflow");
                }
                PublishTopic(HomeTopicMeasurements, Payload, n);
                lastMeasurementsRoot = root;
            }

            // Pumps
            pumpsRoot.clear();
            pumpsRoot["Filt"] = FiltrationPump.IsRunning(); // Filtration pump state
            pumpsRoot["Ph"] = PhPump.IsRunning();           // pH pump state
            pumpsRoot["Robot"] = RobotPump.IsRunning();     // Robot pump state
            pumpsRoot["WF"] = WaterFill.IsRunning();        // Water fill pump state
            pumpsRoot["WH"] = WaterHeatPump.IsRunning();    // Water heat pump state
            pumpsRoot["Sol"] = SolarPump.IsRunning();       // Solar pump state
            pumpsRoot["Heat"] = HeatPump.IsRunning();       // Heat pump state
            if (!storage.Salt_Chlor) {
                pumpsRoot["Chl"] = ChlPump.IsRunning();    // Chlorine pump state
            } else {
                pumpsRoot["Salt"] = SaltPump.IsRunning();  // Salt pump state
            }
            // Check for pump state changes
            bool pumpsChanged = false;
            if (!lastPumpsRoot.isNull()) {
                for (const auto& item : pumpsRoot.as<JsonObject>()) {
                    if (lastPumpsRoot[item.key()] != item.value()) {
                        pumpsChanged = true;
                        break;
                    }
                }
            } else {
                pumpsChanged = true; // Publish on first iteration
            }
            if (pumpsChanged) {
                n = serializeJson(pumpsRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Pumps payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Pumps payload buffer overflow");
                }
                PublishTopic(HomeTopicPumps, Payload, n);
                lastPumpsRoot = pumpsRoot;
            }

            // Valves
            valvesRoot.clear();
            valvesRoot["ELD_Tr"] = ELD_Treppe.getStatus(); // ELD Treppe valve state
            valvesRoot["ELD_H"] = ELD_Hinten.getStatus();  // ELD Hinten valve state
            valvesRoot["WP_V"] = WP_Vorlauf.getStatus();   // WP Vorlauf valve state
            valvesRoot["WP_M"] = WP_Mischer.getStatus();   // WP Mischer valve status
            valvesRoot["Boden"] = Bodenablauf.getStatus(); // Bodenablauf valve state
            if (!storage.SolarLocExt) {
                valvesRoot["SolV"] = Solarvalve.getStatus(); // Solar valve state
            } // else: No solar valve status published when SolarLocExt is true (external solar control)
            // Check for valve state changes
            bool valvesChanged = false;
            if (!lastValvesRoot.isNull()) {
                for (const auto& item : valvesRoot.as<JsonObject>()) {
                    if (lastValvesRoot[item.key()] != item.value()) {
                        valvesChanged = true;
                        break;
                    }
                }
            } else {
                valvesChanged = true; // Publish on first iteration
            }
            if (valvesChanged) {
                n = serializeJson(valvesRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Valves payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Valves payload buffer overflow");
                }
                PublishTopic(HomeTopicValves, Payload, n);
                lastValvesRoot = valvesRoot;
            }

            // Modes
            modesRoot.clear();
            modesRoot["PhPID"] = PhPID.GetMode();          // pH PID mode
            modesRoot["OrpH"] = OrpPID.GetMode();          // ORP PID mode
            modesRoot["Auto"] = storage.AutoMode;          // Auto mode
            modesRoot["HeatPM"] = storage.HeatPumpMode;    // Heat pump mode
            modesRoot["SolMod"] = storage.SolarMode;       // Solar mode
            modesRoot["SaltM"] = storage.SaltMode;         // Salt mode
            modesRoot["ValvM"] = storage.ValveMode;        // Valve mode
            modesRoot["CleanM"] = storage.CleanMode;       // Cleaning mode
            modesRoot["WFMod"] = storage.WaterFillMode;    // Water fill mode
            modesRoot["Winter"] = storage.WinterMode;      // Winter mode
            modesRoot["WatHt"] = storage.WaterHeat;        // Water heating mode
            // Check for mode changes
            bool modesChanged = false;
            if (!lastModesRoot.isNull()) {
                for (const auto& item : modesRoot.as<JsonObject>()) {
                    if (lastModesRoot[item.key()] != item.value()) {
                        modesChanged = true;
                        break;
                    }
                }
            } else {
                modesChanged = true; // Publish on first iteration
            }
            if (modesChanged) {
                n = serializeJson(modesRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Modes payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Modes payload buffer overflow");
                }
                PublishTopic(HomeTopicModes, Payload, n);
                lastModesRoot = modesRoot;
            }

            // Errors
            errorsRoot.clear();
            errorsRoot["FLOW"] = FLOWError;                // Flow error
            errorsRoot["FLOW2"] = FLOW2Error;              // Secondary flow error
            errorsRoot["PSI"] = PSIError;                  // Pressure error
            errorsRoot["PhPUT"] = PhPump.UpTimeError;      // pH pump uptime error
            errorsRoot["PhTkLvl"] = !PhPump.TankLevel();   // pH pump tank level error
            errorsRoot["WFUT"] = WaterFill.UpTimeError;    // Water fill pump uptime error
            if (!storage.Salt_Chlor) {
                errorsRoot["ChlTkLvl"] = !ChlPump.TankLevel(); // Chlorine pump tank level error
                errorsRoot["ChlUT"] = ChlPump.UpTimeError;    // Chlorine pump uptime error
            } else {
                errorsRoot["SaltUT"] = SaltPump.UpTimeError;  // Salt pump uptime error
                errorsRoot["LOWSalt"] = (storage.SaltStatus == "Low Salt"); // Low salt level error
                errorsRoot["HIGHSalt"] = (storage.SaltStatus == "High Salt"); // High salt level error
            }
            // Check for error changes
            bool errorsChanged = false;
            if (!lastErrorsRoot.isNull()) {
                for (const auto& item : errorsRoot.as<JsonObject>()) {
                    if (lastErrorsRoot[item.key()] != item.value()) {
                        errorsChanged = true;
                        break;
                    }
                }
            } else {
                errorsChanged = true; // Publish on first iteration
            }
            if (errorsChanged) {
                n = serializeJson(errorsRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Errors payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Errors payload buffer overflow");
                }
                PublishTopic(HomeTopicErrors, Payload, n);
                lastErrorsRoot = errorsRoot;
            }

            // Runtimes
            runtimesRoot.clear();
            runtimesRoot["Filt"] = FiltrationPump.UpTime / 1000; // Filtration pump runtime
            runtimesRoot["PhP"] = PhPump.UpTime / 1000;         // pH pump runtime
            if (!storage.Salt_Chlor) {
                runtimesRoot["ChP"] = ChlPump.UpTime / 1000;    // Chlorine pump runtime
            } else {
                runtimesRoot["Salt"] = SaltPump.UpTime / 1000;  // Salt pump runtime
            }
            runtimesRoot["SltPlRT"] = storage.SaltPumpRunTime / 1000; // Salt pump runtime on polarity change
            runtimesRoot["Heat"] = HeatPump.UpTime / 1000;      // Heat pump runtime
            runtimesRoot["Sol"] = SolarPump.UpTime / 1000;      // Solar pump runtime
            runtimesRoot["WF"] = WaterFill.UpTime / 1000;       // Water fill pump runtime
            runtimesRoot["WH"] = WaterHeatPump.UpTime / 1000;   // Water heat pump runtime
            runtimesRoot["Robot"] = RobotPump.UpTime / 1000;    // Robot pump runtime
            // Check for runtime changes
            bool runtimesChanged = false;
            if (!lastRuntimesRoot.isNull()) {
                for (const auto& item : runtimesRoot.as<JsonObject>()) {
                    if (lastRuntimesRoot[item.key()] != item.value()) {
                        runtimesChanged = true;
                        break;
                    }
                }
            } else {
                runtimesChanged = true; // Publish on first iteration
            }
            if (runtimesChanged) {
                n = serializeJson(runtimesRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] Runtimes payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] Runtimes payload buffer overflow");
                }
                PublishTopic(HomeTopicRuntimes, Payload, n);
                lastRuntimesRoot = runtimesRoot;
            }

            // Reset reason and system info
            systemRoot.clear();
            systemRoot["ESPUpT"] = storage.Uptime; // ESP32 uptime
            systemRoot["Reason"] = resetReasonToString(esp_reset_reason()); // ESP32 reset reason
            systemRoot["ResetTS"] = storage.ResetTimestamp; // ESP32 reset timestamp
            // Check for system info changes
            bool systemChanged = false;
            if (!lastSystemRoot.isNull()) {
                for (const auto& item : systemRoot.as<JsonObject>()) {
                    if (lastSystemRoot[item.key()] != item.value()) {
                        systemChanged = true;
                        break;
                    }
                }
            } else {
                systemChanged = true; // Publish on first iteration
            }
            if (systemChanged) {
                n = serializeJson(systemRoot, Payload);
                Debug.print(DBG_DEBUG, "[PublishMeasures] System payload size: %d", n);
                if (n >= PAYLOAD_BUFFER_LENGTH) {
                    Debug.print(DBG_ERROR, "[PublishMeasures] System payload buffer overflow");
                }
                PublishTopic(HomeTopicSystem, Payload, n);
                lastSystemRoot = systemRoot;
            }
        } else {
            Debug.print(DBG_ERROR, "[PublishMeasures] MQTT not connected");
        }

        Debug.print(DBG_DEBUG, "[memCheck]: %db", freeRam());
        stack_mon(hwm);

#ifdef CHRONO
        t_act = millis() - td;
        if (t_act > t_max) t_max = t_act;
        if (t_act < t_min) t_min = t_act;
        t_mean += (t_act - t_mean) / n;
        ++n;
        Debug.print(DBG_INFO, "[PublishMeasures] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
#endif

        StopTime = xTaskGetTickCount();
        if (StartTime <= StopTime)
            DeltaTime = StopTime - StartTime;
        else
            DeltaTime = StopTime + (~TickType_t(0) - StartTime) + 1;

        WaitTimeOut = (TickType_t)storage.PublishPeriod / portTICK_PERIOD_MS - DeltaTime;
    }
}