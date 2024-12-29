#pragma once
#define ARDUINOJSON_USE_DOUBLE 1  // Required to force ArduinoJSON to treat float as double

#include "Arduino_DebugUtils.h"   // Debug.print
#include <time.h>                 // Struct and function declarations for dealing with time
#include "TimeLib.h"              // Low level time and date functions
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
#include "AsyncMqttClient.h"      // Async. MQTT client
#include "ADS1115.h"              // ADS1115 sensors library
#include "PCF8574.h"              // IO-Portexpander
#include <credentials.h>          // WIFI Credentials
#include "RTClib.h"               // Real Time Clock library
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// General shared data structure
/*
struct StoreStruct
{
  uint8_t ConfigVersion;   // This is for testing if first time using eeprom or not
  String SSID, WIFI_PASS, MQTT_USER, MQTT_PASS, MQTT_NAME;
  IPAddress MQTT_IP;
  uint16_t MQTT_PORT;
  bool WIFI_OnOff, MQTTLOGIN_OnOff, BUS_A_B, Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, SolarLocExt, SolarMode, Salt_Chlor, SaltMode, SaltPolarity, WinterMode, WaterHeat, ValveMode, CleanMode, ValveSwitch, WaterFillMode;
  uint8_t FiltrationDuration, FiltrationStart, FiltrationStop, FiltrationStartMin, FiltrationStopMax, DelayPIDs, SolarStartMin, SolarStopMax;
  uint8_t address_A_0[8], address_A_1[8], address_A_2[8], address_A_3[8], address_A_4[8], Array_A[5];
  uint8_t address_W_0[8], address_W_1[8], address_W_2[8], address_W_3[8], address_W_4[8], Array_W[5];
  unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit, WaterFillUpTimeLimit, WaterFillDuration, SaltPumpRunTime, PublishPeriod;
  unsigned long PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime, WaterFillAnCon;
  double Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, PSI_MedThreshold, FLOW_Pulse, FLOW_HighThreshold, FLOW_MedThreshold, FLOW2_Pulse, FLOW2_HighThreshold, FLOW2_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1, SaltDiff;
  double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, PhValue, OrpValue, PSIValue, FLOWValue, FLOW2Value;
  double WaterSTemp, WaterITemp, WaterBTemp, WaterWPTemp, WaterWTTemp, AirInTemp, AirTemp, AirHum, AirPress, SolarTemp, SolarVLTemp, SolarRLTemp; 
  double AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR, WaterFillFR;
} ;
*/

struct StoreStruct
{
    uint8_t ConfigVersion;
    String SSID, WIFI_PASS, MQTT_USER, MQTT_PASS, MQTT_NAME;
    IPAddress MQTT_IP;
    uint16_t MQTT_PORT;
    bool WIFI_OnOff, MQTTLOGIN_OnOff, BUS_A_B, Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, SolarLocExt, SolarMode, Salt_Chlor, SaltMode, SaltPolarity, WinterMode, WaterHeat, ValveMode, CleanMode, ValveSwitch, WaterFillMode;
    uint8_t FiltrationDuration, FiltrationStart, FiltrationStop, FiltrationStartMin, FiltrationStopMax, DelayPIDs, SolarStartMin, SolarStopMax;
    uint8_t address_A_0[8], address_A_1[8], address_A_2[8], address_A_3[8], address_A_4[8], Array_A[5]; // Array for DS18B20-adress A
    uint8_t address_W_0[8], address_W_1[8], address_W_2[8], address_W_3[8], address_W_4[8], Array_W[5]; // Array for DS18B20-adress W
    unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit, WaterFillUpTimeLimit, WaterFillDuration, SaltPumpRunTime, PublishPeriod;
    unsigned long PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime, WaterFillAnCon;
    double Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, PSI_MedThreshold, FLOW_Pulse, FLOW_HighThreshold, FLOW_MedThreshold, FLOW2_Pulse, FLOW2_HighThreshold, FLOW2_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1, SaltDiff;
    double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, PhValue, OrpValue, PSIValue, FLOWValue, FLOW2Value;
    double WaterSTemp, WaterITemp, WaterBTemp, WaterWPTemp, WaterWTTemp, AirInTemp, AirTemp, AirHum, AirPress, SolarTemp, SolarVLTemp, SolarRLTemp;
    double AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR, WaterFillFR;
};

extern StoreStruct storage;

//Queue object to store incoming JSON commands (up to 10)
#define QUEUE_ITEMS_NBR 10
#define QUEUE_ITEM_SIZE 100
extern QueueHandle_t queueIn;

//Set the I2C HEX Adress for the BME280 Temperature, Humidity and Airpressure-Sensor for the external temperature
extern Adafruit_BME280 bme;

//Set the I2C HEX Adress for the second PCF8574A IO-Portexpander which manages the Pumps
extern PCF8574 pcf8574_I;
//Set the I2C HEX Adress for the third PCF8574A IO-Portexpander which manages the MotorValves
extern PCF8574 pcf8574_II;
//Set the I2C HEX Adress for the fourth PCF8574A IO-Portexpander which manages also MotorValves and Waterfill, ...
extern PCF8574 pcf8574_III;

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

bool saveParam(const char* key, const uint8_t* val, size_t size);
void publishPoolMode(int event);
void publishSolarMode(int event);

// DS18B20 SENSOR-Mapping to save the sensoradress and Indexnumber to nvs
extern const char* NV_STORAGE_MAPPING_A[];
extern const char* NV_STORAGE_MAPPING_W[];

extern tm timeinfo;

// Firmware revision
extern String Firmw;

extern AsyncMqttClient mqttClient;                     // MQTT async. client

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