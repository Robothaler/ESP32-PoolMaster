// Firmware revision
#define FIRMW           "ESP-3.0"
#define TFT_FIRMW       "TFT-2.0"

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION  12

#define DEBUG_LEVEL     DBG_NONE    // Possible levels : NONE/ERROR/WARNING/INFO/DEBUG/VERBOSE

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

#define WDT_TIMEOUT       10000  // ms — Task WDT timeout (10 s); was incorrectly 10 ms
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
#define STACK_T11         4096  // PublishMeasures
#define STACK_T12         5120  // PublishSettings
#define STACK_T13         6144  // OTATask — SPIFFS + HardwareSerial + 512 B file buffer

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

// T14: Matter sync task parameters (used by createTasks() when MATTER_ENABLED)
#ifndef STACK_T14
  #define STACK_T14             4096
#endif
#ifndef PT14
  #define PT14                  MATTER_SYNC_PERIOD_MS
#endif
#define DT14                    (2000 / portTICK_PERIOD_MS)  // Start offset
#define PRIORITY_T14            1

#define CHRONO                    // Activate tasks timings traces for profiling
//#define SIMU                      // Used to simulate pH/ORP sensors. Very simple simulation:
                                    // the sensor value is computed from the output of the PID 
                                    // loop to reach linearly the theorical value produced by this
                                    // output after one hour