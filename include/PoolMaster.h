#pragma once
#define ARDUINOJSON_USE_DOUBLE 1  // Required to force ArduinoJSON to treat float as double
#define DEBUG_LEVEL DBG_DEBUG      // Possible levels : NONE/ERROR/WARNING/INFO/DEBUG/VERBOSE
// #define CHRONO                    // Activate tasks timings traces for profiling
// #define SIMU                      // Used to simulate pH/ORP sensors. Very simple simulation:
                                    // the sensor value is computed from the output of the PID 
                                    // loop to reach linearly the theorical value produced by this
                                    // output after one hour

#include "Arduino_DebugUtils.h"   // Debug.print
#include <time.h>                 // Struct and function declarations for dealing with time
#include "TimeLib.h"              // Low level time and date functions
#include <RunningMedian.h>        // Determine the running median by means of a circular buffer
#include <PID_v1.h>               // PID regulation loop
#include "OneWire.h"              // Onewire communication
#include <Wire.h>                 // Two wires / I2C library
#include <stdlib.h>               // Definitions  for common types, variables, and functions
#include <ArduinoJson.h>          // JSON library
#include <Pump.h>                 // Simple library to handle home-pool filtration and peristaltic pumps
#include <PCF_Pump.h>             // Simple library to handle home-pool filtration and peristaltic pumps
#include <DallasTemperature.h>    // Maxim (Dallas DS18B20) Temperature temperature sensor library
#include <MQTT.h>                 // MQTT library
#include <esp_task_wdt.h>         // ESP task management library
#include <Preferences.h>          // Non Volatile Storage management (ESP)
#include <WiFi.h>                 // ESP32 Wifi support
#include <WiFiClient.h>           // Base class that provides Client
#include <WiFiUdp.h>              // UDP support
#include <ESPmDNS.h>              // mDNS
#include <ArduinoOTA.h>           // On The Air WiFi update 
#include "AsyncMqttClient.h"      // Async. MQTT client
#include "ADS1115.h"              // ADS1115 sensors library
#include "PCF8574.h"              // IO-Portexpander
#include <credentials.h>          // WIFI Credentials

// General shared data structure
struct StoreStruct
{
  uint8_t ConfigVersion;   // This is for testing if first time using eeprom or not
  bool Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, SaltMode, WinterMode, WaterHeat;
  uint8_t FiltrationDuration, FiltrationStart, FiltrationStop, FiltrationStartMin, FiltrationStopMax, DelayPIDs;
  unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit,PublishPeriod;
  unsigned long PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime;
  double Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, PSI_MedThreshold, FLOW_HighThreshold, FLOW_MedThreshold, FLOW2_HighThreshold, FLOW2_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, TempExternal, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1;
  double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, TempValue, PhValue, OrpValue, PSIValue, FLOWValue, FLOW2Value;
  double AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR;
} ;

extern StoreStruct storage;

//Queue object to store incoming JSON commands (up to 10)
#define QUEUE_ITEMS_NBR 10
#define QUEUE_ITEM_SIZE 100
extern QueueHandle_t queueIn;

//Set the I2C HEX Adress for the second PCF8574A IO-Portexpander
extern PCF8574 pcf8574;

//The six pumps of the system (instanciate the Pump class)
//In this case, all pumps start/Stop are managed by relays
extern Pump FiltrationPump;
extern Pump PhPump;
extern Pump ChlPump;
extern Pump RobotPump;
extern PCF_Pump HeatPump;
extern PCF_Pump SaltPump;
extern PCF_Pump HeatCirculatorPump;

//PIDs instances
//Specify the links and initial tuning parameters
extern PID PhPID;
extern PID OrpPID;

extern bool PSIError;
extern bool FLOWError;
extern bool FLOW2Error;

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