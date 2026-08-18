#pragma once 

// Firmware revision
#define FIRMW           "ESP-3.3.11"
#define TFT_FIRMW       "TFT-2.0"

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION  11

// Matter NVS version — bump to force re-commissioning (erases chip-kvs/chip-counters/chip-config)
#define MATTER_NVS_VERSION  6

// While no Matter fabric exists, keep MQTT disconnected. ESP32-S3 shares 2.4 GHz for
// Wi‑Fi and BLE; MQTT traffic reliably breaks CHIPoBLE / PASE (Apple then often shows
// a misleading "already in a home" message). Set to 0 only if you accept broken pairing
// in exchange for MQTT before the device is paired.
#if !defined(MATTER_NO_MQTT_UNTIL_COMMISSIONED)
#define MATTER_NO_MQTT_UNTIL_COMMISSIONED  1
#endif

// Extra 2.4 GHz headroom for CHIPoBLE while still uncommissioned:
// • MAX power-save reduces how often STA holds the radio.
// • STA disconnect on BLE GAP connect can help PASE when NOTIFY_TX stalls, but Apple Home
//   often needs STA+BLE during setup → default OFF. If you set this to 1, disconnect is
//   deferred (MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US) so NimBLE is not wedged from the GAP
//   callback (avoids "Adv reattempt failed; rc=3" / "Gerät wurde nicht gefunden").
#if !defined(MATTER_WIFI_PS_MAX_WHILE_UNCOMMISSIONED)
#define MATTER_WIFI_PS_MAX_WHILE_UNCOMMISSIONED  1
#endif
#if !defined(MATTER_WIFI_STA_OFF_DURING_BLE_GAP)
#define MATTER_WIFI_STA_OFF_DURING_BLE_GAP  0
#endif
#if !defined(MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US)
#define MATTER_WIFI_STA_OFF_BLE_GAP_DELAY_US  300000u // 300 ms after GAP CONNECT
#endif

// When MATTER_WIFI_STA_OFF_DURING_BLE_GAP is 0: optional defer WiFi STA disconnect after
// CHIPoBLE TX CCCD SUBSCRIBE (handle MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE).
// **Default OFF:** serial-20260501_163018 showed correct trigger (0x0012 @ ~18230 ms, disconnect
// @ ~18630 ms) yet **zero** `MatterBLE: GAP NOTIFY_TX` and **0x213** — STA drop during the
// open BLE link appears to break BTP on this build. Set to 1 only for experiments.
#if !defined(MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE)
#define MATTER_WIFI_DISCONNECT_AFTER_BLE_INDICATE_SUBSCRIBE  0
#endif
#if !defined(MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE)
#define MATTER_CHIPOBLE_GAP_TX_CCCD_ATTR_HANDLE  0x0012u
#endif
#if !defined(MATTER_WIFI_BLE_SUBSCRIBE_RELIEVE_US)
#define MATTER_WIFI_BLE_SUBSCRIBE_RELIEVE_US  400000u
#endif

// Agent NDJSON lines — bei Pairing serial/aus (printf kann NimBLE blockieren).
#if !defined(MATTER_AGENT_DEBUG_NDJSON)
#define MATTER_AGENT_DEBUG_NDJSON  0
#endif

// NimBLE GAP listener: SUBSCRIBE / NOTIFY_TX (indication ACK) / MTU / disconnect — ESP_LOG
// only from the GAP callback (Serial there garbles chip[DL] lines and can block NimBLE).
// Set 0 to disable.
#if !defined(MATTER_BLE_GAP_DIAG_LISTENER)
#define MATTER_BLE_GAP_DIAG_LISTENER  1
#endif

// While CHIPoBLE session is active (phone wrote RX / subscribed), call a short
// vTaskDelay() once per loop in high-rate pool tasks so NimBLE/CHIP get CPU time.
// Safer than vTaskSuspend (no mutex deadlocks). Set to 0 to disable.
//
// On esp_matter + ESP32-NimBLE, kCHIPoBLEConnectionEstablished / WriteReceived
// often never reach PlatformMgr handlers even though chip[DL] logs GATT traffic.
// matterYieldAppTasksIfChipobleBusy() then never ran — CPU starvation, no TX
// indications (Apple → "schon in einem Zuhause"). When MATTER_BLE_GAP_DIAG_LISTENER
// is on, we mirror "session active" from NimBLE GAP CONNECT / SUBSCRIBE / DISCONNECT.
#if !defined(MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE)
#define MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE  1
#endif
#if !defined(MATTER_CHIPOBLE_APP_YIELD_MS)
#define MATTER_CHIPOBLE_APP_YIELD_MS  32
#endif

// Extra Matter transport logs (PASE/CASE, message layer). Very chatty — default off.
#if !defined(MATTER_LOG_EXTRA_CHIP_TAGS)
#define MATTER_LOG_EXTRA_CHIP_TAGS  1
#endif

// Defer otaTask's AsyncWebServer (WebUI) until Fabric existiert oder Fallback — stark empfohlen
// fürs Apple-Pairing (AsyncTCP/WebSocket konkurriert mit CHIPoBLE auf 2,4 GHz).
// Nach erfolgreicher Einrichtung (fabric>0) startet der Server sofort im otaTask.
#if !defined(MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED)
#define MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED  1
#endif
#if !defined(MATTER_HTTP_SERVER_FALLBACK_MS)
#define MATTER_HTTP_SERVER_FALLBACK_MS  (90UL * 1000UL)
#endif

// Suspend pool FreeRTOS tasks (see Tasks.cpp) on BLE GAP connect while fabric==0;
// resume on disconnect / CommissioningComplete / CHIPoBLE closed.
//
// Default OFF: suspending PoolMaster/PCF during GAP correlated with CHIP failing to post
// to the Platform event queue (0x01000000 / "Failed to schedule work") and BLE teardown
// 0x213 once the phone sends the first GATT write (see logs/serial-20260503_162037.log).
// MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE yields Core-1 loops instead — fewer deadlock risks.
//
// Enable (=1) only if you run heavy tasks (e.g. CombinedPolling) during commissioning and see
// chip[DL] Long dispatch / PASE timeouts — logs/serial-20260503_110709.log style.
#if !defined(MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE)
#define MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE  0
#endif

// Set to 1 to expose a **single** On/Off device (no Aggregator/bridge, no extra endpoints).
// Reduces RAM / descriptors — useful for commissioning/PASE debugging. Maps OnOff → Filterpump JSON.
// Switching minimal ↔ full usually requires a Matter factory reset (different endpoint layout).
#if !defined(MATTER_MINIMAL_DEVICE)
#define MATTER_MINIMAL_DEVICE  0
#endif

#define DEBUG_LEVEL     DBG_INFO    // Possible levels : NONE/ERROR/WARNING/INFO/DEBUG/VERBOSE

// WiFi credentials
// ------  Credentials are stored in include/credentials.h
// #define WIFI_NETWORK "YOUR_WIFI_NETWORK_ID"
// #define WIFI_PASSWORD "YOUR_WIFI_NETWORK_PWD"
// #define OTA_PWDHASH "Your_OTA_password_hash"

#ifdef DEVT
  #define HOSTNAME      "PoolMaster_Dev"
#else
  #define HOSTNAME      "PoolMaster"
#endif 

//IFTTT key to trigger event
#define IFTTT_key       "/trigger/PoolMaster/with/key/Your_IFTTT_Key"

// Mail parameters and credentials
//#define EMAIL_ALERT    // Comment this line to disable Email alerting
#define SMTP_HOST       "your smtp server"
#define SMTP_PORT       587      // check the port number
#define AUTHOR_EMAIL    "your email address"
#define AUTHOR_LOGIN    "your user name"
#define AUTHOR_PASSWORD "your password"
#define RECIPIENT_EMAIL "your recipient email address"


// PID Directions (either DIRECT or REVERSE depending on Ph/Orp correction vs water properties)
#define PhPID_DIRECTION   REVERSE
#define OrpPID_DIRECTION  DIRECT

// Polarity of the Electric-Connection (The polarity should be changed every 4 hours to prevent calcification of the electrolysis plates.)
#define POLARITY_DIRECT   0
#define POLARITY_REVERSE  1

// Configuration for Zodiac LM2-40 Salt Electrolysis
#define ELECTROLYSIS_VOLTAGE  24.0    // Voltage of electrolysis cell (V)
#define DEFAULT_CELL_CONSTANT 5.0     // Default cell constant (m⁻¹)
#define POOL_VOLUME           23.6    // Pool volume (m³)
#define TARGET_SALT_MIN       3.0     // Target salt content minimum (g/L)
#define TARGET_SALT_MAX       4.0     // Target salt content maximum (g/L)
#define LOW_SALT_THRESHOLD    2.8     // Threshold for "Low Salt" (g/L)
#define TARGET_SALT_REF       3.5     // Reference for salt amount calculation (g/L)

#define FILTER_VOLTAGE        230.0   // V, Voltage of filtration pump
#define HEAT_VOLTAGE          230.0   // V, Voltage of heat pump

// Define RELAY-PINS for all Pumps (Second PCF8574_I)
#define FILTRATION_PUMP   P0   // Filtration-Pump
#define HEAT_PUMP         P1   // Heat-Pump
#define SALT_PUMP         P2   // Salt-Manager
#define ROBOT_PUMP	      P3   // Cleaningrobot
#define PH_PUMP           P4   // PH-Pump
#define CHL_PUMP          P5   // Chlorine-Pump
#define SOLAR_PUMP        P6   // Solar, 3-way-valve for Warmwater Solarpanels
#define RELAY_R0          P7   // Spare I

// Define RELAY-PINS for MotorValves (Third PCF8574_II)
#define ESD_TRE_OPEN      P0   // ESD-Treppe (open)
#define ESD_TRE_CLOSE     P1   // ESD-Treppe (close)
#define ESD_HIN_OPEN      P2   // ESD-Hinten (open)
#define ESD_HIN_CLOSE     P3   // ESD-Hinten (close)
#define WP_VL_OPEN        P4   // WP-Vorlauf (open)
#define WP_VL_CLOSE       P5   // WP-Vorlauf (close)
#define WP_M_OPEN	        P6   // WP-Mischer (open)
#define WP_M_CLOSE        P7   // WP-Mischer (close)

// Define RELAY-PINS for MotorValves (Fourth PCF8574_III)
#define BODEN_OPEN	      P0   // Bodenablauf (open)
#define BODEN_CLOSE       P1   // Bodenablauf (close)
#define SOLAR_OPEN        P2   // SOLAR (open)
#define SOLAR_CLOSE       P3   // SOLAR (close)
#define SPARE_I_OPEN      P4   // SPARE (open)
#define SPARE_I_CLOSE     P5   // SPARE (close)
#define WATER_FILL        P6   // Freshwater-Valve to fillup the Pool -> Needs to be combined with Levelsensors in the skimmer
#define HEAT_ON           P7   // Switch for Heatdemand


#define LIGHT_POOL         7   // Pool Spotlight
#define LIGHT_ROOM        10   // Serviceroom light
#define SALT_POL          13   // Salt-Manager Polarity DIRECT / REVERSE
#define RELAY_R1          14   // Spare II
#define RELAY_R2          21   // Spare III
#define RELAY_R3          38   // Spare IV
#define RELAY_R4          45   // Spare V
#define RELAY_R5           6   // Spare VI

//Digital input pins connected to Flow-Meter additional security for Filtrationpump and dosing
#define FLOW              39   // Flow-Meter in Main-Pipe to be sure Filtrationpump is running
#define FLOW2             40   // Flow-Meter in Measure-Pipe to be sure water is flowing to get accurate values of ph and orp meter

//Digital input pins connected to level reed switches in pool to indicate low or high water level
//LOW = Switch is closed / HIGH = Switch is open
#define WATER_MAX_LVL     41   //
#define WATER_MIN_LVL     42   // 

//Digital input pins connected to level reed switches in canister indicate low pH or Chlorine level
//LOW = Switch is open
#define PH_LVL            15   // 
#define CHL_LVL           16   //

//One wire bus for the air/water temperature measurement
#define ONE_WIRE_BUS_A     4
#define ONE_WIRE_BUS_W     5
#define MAX_ADDRESSES      5   // 5 sensors max on the bus

//I2C bus for analog measurement with ADS1115 of pH, ORP and water pressure 
//and status LED through PCF8574A 
#define I2C_SDA			       8   //
#define I2C_SCL			       9   //
#define PCF8574_ADR       0x24 // for Status-LEDs
#define PCF8574_I_ADR     0x3F // for External Relais for 230V Apliances
#define PCF8574_II_ADR    0x3D // for Motorvalves
#define PCF8574_III_ADR   0x3B // for additional Motorvalves and Waterfillvalve

//Type of pH and Orp sensors acquisition :
//INT_ADS1115 : single ended signal with internal ADS1115 ADC (default)
//EXT_ADS1115 : differential signal with external ADS1115 ADC (Loulou74 board)
#define EXT_ADS1115
#define INT_ADS1115_ADDR  ADS1115ADDRESS // 0x48 is default address -> ADS1115ADDRESS // 0x49 address -> ADS1115ADDRESS+1
#define PH_ADS1115_ADDR   ADS1115ADDRESS+2 // 0x4A is default address -> ADS1115ADDRESS+2
#define ORP_ADS1115_ADDR  ADS1115ADDRESS+3 // 0x4B is default address -> ADS1115ADDRESS+3

// Buzzer
#define BUZZER             2  //

// MotorValve Constants
#define STARTANGLE_0       0   // StartAngle for MotorValves
#define MAX_45            45   // MaxAngle 45 degree for MotorValves
#define MAX_90            90   // MaxAngle 90 degree for MotorValves
#define TIMETOMAX_45      45   // Time to reach to maximum in Seconds
#define TIMETOMAX_90      90   // Time to reach to maximum in Seconds

#define WDT_TIMEOUT       30000  // ms — 30 s; gives CONFIG_ASYNC_TCP_MAX_ACK_TIME=5000 time to
                                  // disconnect throttled WebSocket clients before TWDT fires
#define MWDT_TIMEOUT_MS   15000  // ms — Motor WDT timeout (15 s)

// Server port
#define SERVER_PORT       8060

//OTA port
#define OTA_PORT          8063

//OTA host name
#define OTA_HOST          "PoolMaster"

// 12bits (0,06°C) temperature sensors resolution
// 9 bits (0.5°C) resolution is used for the DS18B20 sensors
#define TEMPERATURE_RESOLUTION 9

//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishement of measurement data
#define PUBLISHINTERVAL   30000

#define MQTT_SERVER_IP    IPAddress(192, 168, 178, 55)
#define MQTT_SERVER_PORT  1883

// Uncomment if MQTT broker needs login/pwd
//#define MQTT_LOGIN 				
#define MQTT_SERVER_ID    "ESP32Pool"		   // MQTT server ID

// -------> credentials are defined in credentials.h
//#define MQTT_SERVER_LOGIN "Your_Login"
//#define MQTT_SERVER_PWD   "Your_Pwd" 

// Topic used in DEVT or OPER mode

#ifdef DEVT
  #define POOLTOPIC       "Home/Pool6/"
#else
  #define POOLTOPIC       "Home/Pool/"
#endif 

// Robot pump timing
#define ROBOT_DELAY       60     // Robot start delay after filtration in mn
#define ROBOT_DURATION    90     // Robot cleaning duration in mn

// Pool heat pump (Wärmepumpe): AUTO mode compares WaterSTemp to WaterTemp_SetPoint (Schmitt trigger)
#define HEAT_PUMP_AUTO_HYST_BELOW_SP  0.45   // °C — start heating when temp < setpoint − this
#define HEAT_PUMP_AUTO_HYST_ABOVE_SP  0.45   // °C — stop heating when temp > setpoint + this

//Display timeout before blanking
//-------------------------------
#define TFT_SLEEP         60000L 

// Loop tasks scheduling parameters
//---------------------------------
// T1:  CombinedPolling (previously AnaloPoll)
// T2:  ProcessCommand (previously PoolServer)
// T3:  PoolMaster
// T4:  TempTask
// T5:  readBME280
// T6:  OrpRegulation
// T7:  SaltRegulation
// T8:  pHRegulation
// T9:  FlowMeasures
// T10: StatusLights
// T11: PublishMeasures
// T12: PublishSettings
// T13: OTATask (for Nextion display OTA updates)
// T14: MatterSyncTask (esp_matter state sync — created only if MATTER_ENABLED && ENABLE_TASK_T14)
// T15: PCF8574Manager update task (I2C expander writes)
//
// Task creation: 1 = all pool loop tasks, 0 = minimal set for Matter commissioning bisect
// (#else: T1+T2+T4…T13 off, T3 PoolMaster on; T14/T15 default 0 here but see MATTER_ENABLED override below.)
// T6 and T7 in comments are one task (ChlorSaltRegulation) — controlled by ENABLE_TASK_T6.
#ifndef POOLMASTER_ENABLE_ALL_POOL_TASKS
#define POOLMASTER_ENABLE_ALL_POOL_TASKS 1
#endif
#if POOLMASTER_ENABLE_ALL_POOL_TASKS
#define ENABLE_TASK_T1        1
#define ENABLE_TASK_T2        1
#define ENABLE_TASK_T3        1
#define ENABLE_TASK_T4        1
#define ENABLE_TASK_T5        1
#define ENABLE_TASK_T6        1
#define ENABLE_TASK_T8        1
#define ENABLE_TASK_T9        1
#define ENABLE_TASK_T10       1
#define ENABLE_TASK_T11       1
#define ENABLE_TASK_T12       1
#define ENABLE_TASK_T13       1
#define ENABLE_TASK_T14       1
#define ENABLE_TASK_T15       1
#else
#define ENABLE_TASK_T1        0
#define ENABLE_TASK_T2        0
#define ENABLE_TASK_T3        1
#define ENABLE_TASK_T4        0
#define ENABLE_TASK_T5        0
#define ENABLE_TASK_T6        0
#define ENABLE_TASK_T8        0
#define ENABLE_TASK_T9        0
#define ENABLE_TASK_T10       0
#define ENABLE_TASK_T11       0
#define ENABLE_TASK_T12       0
#define ENABLE_TASK_T13       0
#define ENABLE_TASK_T14       0
#define ENABLE_TASK_T15       0
#endif

// Minimal pool task set + Matter: must keep T14 (MatterSyncTask) and T15 (PCF worker).
// With T15=0, PoolMaster still drives PCF8574 sync every PT3 → endless DBG_ERROR / Serial spam,
// CHIP event loop starvation ("Long dispatch time" ~880 ms), FailSafe timeout, Apple Home abort.
#if !POOLMASTER_ENABLE_ALL_POOL_TASKS && defined(MATTER_ENABLED)
#undef ENABLE_TASK_T14
#define ENABLE_TASK_T14       1
#undef ENABLE_TASK_T15
#define ENABLE_TASK_T15       1
#endif

// Periods 
// Task12 period is initialized with PUBLISHINTERVAL and can be changed dynamically
#define PT1               125
#define PT2               500
#define PT3               500
#define PT4               2000 // (1 << (12 - TEMPERATURE_RESOLUTION))
#define PT5               2000
#define PT6               1000
#define PT7               1000
#define PT8               1000
#define PT9               1000
#define PT10              3000
#define PT11              30000
#define PT12              PUBLISHINTERVAL  // Period for MQTT publish of settings
#define PT13              1000  // Period for OTA task (symbolic, as it mainly waits for uploads)

// Start offsets to spread tasks along time
#define DT1               0/portTICK_PERIOD_MS
#define DT2               190/portTICK_PERIOD_MS
#define DT3               310/portTICK_PERIOD_MS
#define DT4               600/portTICK_PERIOD_MS
#define DT5               520/portTICK_PERIOD_MS
#define DT6               560/portTICK_PERIOD_MS
#define DT7               565/portTICK_PERIOD_MS
#define DT8               920/portTICK_PERIOD_MS
#define DT9               1060/portTICK_PERIOD_MS
#define DT10              100/portTICK_PERIOD_MS
#define DT11              570/portTICK_PERIOD_MS
#define DT12              940/portTICK_PERIOD_MS
#define DT13              980/portTICK_PERIOD_MS  // Start offset for OTA task to avoid overlap

// Task stack sizes (in bytes)
#define STACK_T1          8192  // CombinedPolling
#define STACK_T2          8192  // ProcessCommand
#define STACK_T3          5120  // PoolMaster
#define STACK_T4          8192  // TempTask
#define STACK_T5          3072  // readBME280
#define STACK_T6          3072  // OrpRegulation
#define STACK_T7          3072  // SaltRegulation
#define STACK_T8          3072  // pHRegulation
#define STACK_T9          3072  // FlowMeasures
#define STACK_T10         4096  // StatusLights
#define STACK_T11         8192  // PublishMeasures
#define STACK_T12         5120  // PublishSettings
#define STACK_T13         6144  // OTATask — SPIFFS + HardwareSerial + 512 B file buffer
#define STACK_T15         4096  // PCF8574Manager update (I2C / Wire)

// Task priorities angepasst für bessere Synchronisation
#define PRIORITY_T1       1    // CombinedPolling höchste Priorität
#define PRIORITY_T2       1
#define PRIORITY_T3       1
#define PRIORITY_T4       1
#define PRIORITY_T5       1
#define PRIORITY_T6       1
#define PRIORITY_T7       1
#define PRIORITY_T8       1
#define PRIORITY_T9       1
#define PRIORITY_T10      1
#define PRIORITY_T11      1
#define PRIORITY_T12      1
#define PRIORITY_T13      1         // Priority for OTA task
#define PRIORITY_T15      2         // PCF worker — above pool loops (legacy tskIDLE_PRIORITY+2)

// Timing Parameter für PCF8574
#define PCF_UPDATE_INTERVAL    50    // Minimale Zeit zwischen PCF Updates (ms)
#define PCF_VERIFY_TIMEOUT     50    // Timeout für Statusverifikation (ms)

// OTA-specific settings
#define OTA_NEXTION_PORT  80        // Port for Nextion OTA web server
#define OTA_NEXTION_PATH  "/upload" // Endpoint for Nextion OTA uploads

// =============================================================================
// Matter Bridge Configuration
// =============================================================================
// Uncomment MATTER_ENABLED here to build with Matter support, OR pass
// -D MATTER_ENABLED via the [env:matter_serial] / [env:matter_ota] PlatformIO
// environment (recommended — keeps non-Matter builds unaffected).
// #define MATTER_ENABLED

// Matter device identity  (override via -D in platformio.ini if desired)
// Use 0xFFF1/0x8000 range for test/development.
// Replace with Connectivity Standards Alliance (CSA) assigned IDs for production.
#define MATTER_VENDOR_ID        0xFFF1    // Test vendor ID
#define MATTER_PRODUCT_ID       0xCA01    // PoolMaster Bridge product ID
#define MATTER_DISCRIMINATOR    3840      // BLE discriminator for commissioning (0–4095)
#define MATTER_PASSCODE         20202021  // Setup passcode — CHANGE FOR PRODUCTION
#define MATTER_DEVICE_NAME      "PoolMaster Bridge"

// Sync period: how often pool state is pushed to Matter attribute cache (ms)
#define MATTER_SYNC_PERIOD_MS   5000

// T14: Matter sync task parameters (used by createTasks() when MATTER_ENABLED && ENABLE_TASK_T14)
#ifndef STACK_T14
  #define STACK_T14             4096
#endif
#ifndef PT14
  #define PT14                  MATTER_SYNC_PERIOD_MS
#endif
#define DT14                    (2000 / portTICK_PERIOD_MS)  // Start offset
#ifndef PRIORITY_T14
  #define PRIORITY_T14          1
#endif

// =============================================================================
// Matter SolarControl integration — NVS keys and constants
// =============================================================================
// These keys store the SolarControl's Matter NodeId and endpoint IDs in NVS so
// they survive reboots.  Configure them once via the serial command:
//   SET_SOLAR_NODE <nodeId_hex> <epPump> <epValve> <epCirc> <epIllum>
// Example:
//   SET_SOLAR_NODE 0000000000000002 5 6 7 8
// =============================================================================

// Legacy simple hysteresis (some diagnostics); external solar regulation uses
// SOLAR_EXT_* together with poolSolarBridgeSolarModeRequest().
#define SOLAR_MODE_HYSTERESIS  1.0f   // °C

// External solar (MQTT, HTTP pool–solar bridge, Matter): same thresholds as PoolMaster regulation.
#define SOLAR_EXT_COLLECTOR_DELTA_MIN  4.0   // °C — collector must exceed pool water by this to request pool heating
#define SOLAR_EXT_RL_STOP_MARGIN       2.0   // °C — prefer buffer when RL + margin <= pool water

// HTTP LAN poll of SolarControl remains a fallback while Matter subscriptions
// are not yet delivering reports (or in non-Matter builds).  Set to 0 to
// disable the client poll entirely (GET /read for SolarControl is kept).
#ifndef POOL_SOLAR_HTTP_FALLBACK
#define POOL_SOLAR_HTTP_FALLBACK  1
#endif

// Treat Matter SolarControl reports as stale after this many ms (max report interval is 60 s).
#ifndef MATTER_SOLAR_REPORT_STALE_MS
#define MATTER_SOLAR_REPORT_STALE_MS  90000u
#endif

#ifdef MATTER_ENABLED
constexpr char NVS_KEY_SOLAR_NODE_ID[]  = "solar_node_id";  // uint64_t — SolarControl Matter NodeId
constexpr char NVS_KEY_SOLAR_EP_PUMP[]  = "solar_ep_pump";  // uint16_t — SolarControl pump EP (default 5)
constexpr char NVS_KEY_SOLAR_EP_VALVE[] = "solar_ep_valve"; // uint16_t — SolarControl valve EP (default 6)
constexpr char NVS_KEY_SOLAR_EP_CIRC[]  = "solar_ep_circ";  // uint16_t — SolarControl circulation EP (default 7)
constexpr char NVS_KEY_SOLAR_EP_ILLUM[] = "solar_ep_illum"; // uint16_t — SolarControl illumination EP (default 8)

#endif // MATTER_ENABLED

#define CHRONO                    // Activate tasks timings traces for profiling
//#define SIMU                      // Used to simulate pH/ORP sensors. Very simple simulation:
                                    // the sensor value is computed from the output of the PID 
                                    // loop to reach linearly the theorical value produced by this
                                    // output after one hour