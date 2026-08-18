#pragma once
#define ARDUINOJSON_USE_DOUBLE 1  // Required to force ArduinoJSON to treat float as double

#include <Arduino_DebugUtils.h>   // Debug.print
#include <time.h>                 // Struct and function declarations for dealing with time
#include <TimeLib.h>              // Low level time and date functions
#include <RunningMedian.h>        // Determine the running median by means of a circular buffer
#include <PID_v1.h>               // PID regulation loop
#include "OneWire.h"              // Onewire communication
#include <Wire.h>                 // Two wires / I2C library
#include <stdlib.h>               // Definitions  for common types, variables, and functions
#include <ArduinoJson.h>          // JSON library
//#include <Pump.h>                 // Simple library to handle home-pool filtration and peristaltic pumps
#include <PCF_Pump.h>             // Simple library to handle home-pool filtration and peristaltic pumps
#include <MotorValve.h>           // Simple library to handle motor valves for home-pool
#include <DallasTemperature.h>    // Maxim (Dallas DS18B20) Temperature temperature sensor library
#include <MQTT.h>                 // MQTT library
#include <esp_task_wdt.h>         // ESP task management library
#include <Preferences.h>          // Non Volatile Storage management (ESP)
#include <WiFi.h>                 // ESP32 Wifi support
#include <WiFiClient.h>           // Base class that provides Client
#include <WiFiUdp.h>              // UDP support
#include <ESPmDNS.h>              // mDNS
#include <ArduinoOTA.h>           // Over The Air WiFi update
#include <ESPAsyncWebServer.h>    // Asynchronous Web Server 
#include "AsyncMqttClient.h"      // Async. MQTT client
#include "ADS1115.h"              // ADS1115 sensors library
#include "PCF8574.h"              // IO-Portexpander
#include <credentials.h>          // WIFI Credentials
// credentials.h defines MQTT_USER and MQTT_PORT as macros which clash with the
// identically named StoreStruct member fields below. Undefine them immediately
// so the struct compiles correctly. All runtime access goes through storage.MQTT_USER
// and storage.MQTT_PORT (struct members). Compile-time defaults use MQTT_SERVER_PORT
// and MQTT_SERVER_ID from Config.h.
#ifdef MQTT_USER
  #undef MQTT_USER
#endif
#ifdef MQTT_PORT
  #undef MQTT_PORT
#endif
#include "RTClib.h"               // Real Time Clock library
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "I2CConfig.h"
#include "PoolData.h"

extern StoreStruct storage;

inline PoolConfig& poolConfig() { return storage; }
inline PoolMeasures& poolMeasures() { return storage; }
inline PoolRuntime& poolRuntime() { return storage; }
inline PoolSolarRemote& poolSolarRemote() { return storage; }

extern const PCF_Pin NO_PIN;

extern SemaphoreHandle_t mutex;           // Mutex for I2C access
extern SemaphoreHandle_t mqttMutex;       // Mutex for MQTT publish (separate from I2C)
extern SemaphoreHandle_t i2cStatesMutex;  // Mutex for I2C states
extern SemaphoreHandle_t i2cOutputMutex;  // Mutex for I2C output states

/** Serializes Arduino Preferences (`Preferences nvs`) — not thread-safe across tasks. */
extern SemaphoreHandle_t prefsMutex;
void prefsLock(void);
void prefsUnlock(void);

/** Ensure libc TZ is Europe/Berlin (CET/CEST). Re-applies if anything (configTime, CHIP SNTP) stomped TZ. */
void poolEnsureEuropeBerlinTz(void);

/** Convert a Europe/Berlin civil `tm` to UTC epoch and write gettimeofday() for Matter/CHIP.
 *  Do not call this with a tm that getLocalTime() already produced after a successful NTP sync
 *  (SNTP already set UTC). Use it for RTC fallback and manual Date commands. */
void poolApplyEspSystemTimeFromLocalTm(struct tm *tmLocal);

bool lockI2C(); // Declaration of the lockI2C function
void unlockI2C(); // Declaration of the unlockI2C function
unsigned long getDurationSafe(unsigned long start, unsigned long current);
// Forward declaration — full inline definition is below near line 155

// pH Kp auto-scaling based on pool volume.
// Reference: Gixy31 original calibrated Kp=2,700,000 for a 50m³ pool.
// Only Kp changes with volume; Ki and Kd remain manually configurable.
// Returns 0 if volumeM3 <= 0 (caller should keep existing Kp).
double calcPhKpForVolume(float volumeM3);

//Queue object to store incoming JSON commands (up to 10)
#define QUEUE_ITEMS_NBR 20
#define QUEUE_ITEM_SIZE 150
extern QueueHandle_t queueIn;

//Set the I2C HEX Adress for the BME280 Temperature, Humidity and Airpressure-Sensor for the external temperature
extern Adafruit_BME280 bme;

//The seven pumps of the system (instanciate the Pump class)
//In this case, all pumps start/Stop are managed by relays
extern PCF_Pump FiltrationPump;
extern PCF_Pump PhPump;
extern PCF_Pump ChlPump;
extern PCF_Pump RobotPump;
extern PCF_Pump HeatPump;
extern PCF_Pump WaterHeatPump;
extern PCF_Pump SaltPump;
extern PCF_Pump SolarPump;
extern PCF_Pump WaterFill;

//The six motor valves of the system (instanciate the MotorValve class)
//In this case, all valves open/close are managed by relays
extern MotorValve ELD_Treppe;
extern MotorValve ELD_Hinten;
extern MotorValve WP_Vorlauf;
extern MotorValve WP_Mischer;
extern MotorValve Bodenablauf;
extern MotorValve Solarvalve;

//PIDs instances
//Specify the links and initial tuning parameters
extern PID PhPID;
extern PID OrpPID;

extern bool PSIError;
extern bool FLOWError;
extern bool FLOW2Error;
extern bool WaterFillError;
extern bool I2CError;

void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle);
bool saveParam(const char* key, const uint8_t* val, size_t size);
void publishPoolMode(int event);
void publishSolarMode(int event);
void mqttInit();
void mqttErrorPublish(const char* Payload);
void SetPhPID(bool Enable);
void SetOrpPID(bool Enable);
void connectToWiFi();
void connectToMqtt();
/** STA association + IP path (Matter: uses esp_wifi; never rely on `WiFi.status()` alone). */
bool wifiStaConnected(void);
/** Dotted IPv4 for STA; Matter uses `esp_netif` (not `WiFi.localIP()`). Returns false if no address. */
bool wifiStaGetIpv4String(char *buf, size_t bufLen);
/** Nextion-Leiste (`vaMqttState.txt`, `pXNetW`): „online“, wenn MQTT verbunden ist oder LAN ohne/fehlenden MQTT (Matter/BLE, MQTT aus). */
bool nextionNetStatusOnline(void);

// Converts ESP32 reset reason to a human-readable C-string literal.
// inline to avoid multiple-definition errors when included in several TUs.
inline const char* resetReasonToString(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN:    return "Unknown";
        case ESP_RST_POWERON:    return "Power-on";
        case ESP_RST_EXT:        return "External";
        case ESP_RST_SW:         return "Software";
        case ESP_RST_PANIC:      return "Panic";
        case ESP_RST_INT_WDT:    return "Int Watchdog";
        case ESP_RST_TASK_WDT:   return "Task Watchdog";
        case ESP_RST_WDT:        return "Other Watchdog";
        case ESP_RST_DEEPSLEEP:  return "Deep Sleep";
        case ESP_RST_BROWNOUT:   return "Brownout";
        case ESP_RST_SDIO:       return "SDIO";
        default:                 return "Invalid";
    }
}
// uint8_t overload — storage.ResetReason is uint8_t, not esp_reset_reason_t
inline const char* resetReasonToString(uint8_t reason) {
    return resetReasonToString(static_cast<esp_reset_reason_t>(reason));
}

// DS18B20 SENSOR-Mapping to save the sensoradress and Indexnumber to nvs
extern const char* NV_STORAGE_MAPPING_A[];
extern const char* NV_STORAGE_MAPPING_W[];

extern tm timeinfo;

// Firmware revision
extern String Firmw;

extern AsyncMqttClient mqttClient;                     // MQTT async. client

#ifdef MATTER_ENABLED
/**
 * When true, disconnects from the MQTT broker and suppresses auto-reconnect.
 * Used during Matter BLE commissioning: ESP32-S3 shares one radio for WiFi
 * and BLE; MQTT keepalive (PING) bypasses PublishTopic() and still consumes
 * airtime. Call with false when BLE is idle or commissioning has finished.
 */
void mqttSetBleRadioHold(bool hold);

/**
 * When true, WIFI_EVENT_STA_DISCONNECTED does not start wifiReconnectTimer and
 * connectToWiFi() is a no-op. Used while BLE GAP is up and we intentionally
 * called esp_wifi_disconnect() for radio coexistence — otherwise the reconnect
 * timer immediately re-associates WiFi during PASE and breaks commissioning.
 */
void mqttSetMatterWifiReconnectHold(bool hold);
#endif

// Various flags
extern volatile bool startTasks;                       // flag to start loop tasks       
extern bool MQTTConnection;                            // MQTT connected flag
extern bool EmergencyStopFiltPump;                     // Filtering pump stopped manually; needs to be cleared to restart
extern bool AntiFreezeFiltering;                       // Filtration anti freeze mode
extern bool PSIError;                                  // Water pressure alarm
extern bool FLOWError;                                 // Flow in Main-Pipe alarm
extern bool FLOW2Error;                                // Flow in Measure-Pipe alarm
extern bool WaterFillError;                            // Waterfill system is OK
extern bool cleaning_done;                             // Robot clean-up done