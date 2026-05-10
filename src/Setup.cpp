#undef __STRICT_ANSI__              // work-around for Time Zone definition
#include <stdint.h>                 // std lib (types definitions)
#include <Arduino.h>                // Arduino framework
#include <esp_sntp.h>
#include <nvs_flash.h>
#include <SPIFFS.h>
#include "esp_chip_info.h"
#include <esp_system.h>             // for esp_reset_reason
#include <sys/time.h>
#include <string.h>
#include <errno.h>

#include "Config.h"
#include "I2CConfig.h"
#include "PoolMaster.h"
#include "PoolSolarBridge.h"
#include "Ota.h"
#include "Tasks.h"
#include "PCF8574Manager.h"
#ifdef MATTER_ENABLED
#include "MatterBridge.h"
#include "esp_wifi.h"
static inline bool wifiIsConnected() {
  wifi_ap_record_t _ap;
  return esp_wifi_sta_get_ap_info(&_ap) == ESP_OK;
}
#else
static inline bool wifiIsConnected() { return WiFi.status() == WL_CONNECTED; }
#endif

#include <soc/gpio_struct.h>
#include <hal/gpio_ll.h>
//#include "nvs_flash.h"

extern gpio_dev_t GPIO; // access GPIO registers directly

#ifdef SIMU
bool init_simu = true;
double pHLastValue = 7.;
unsigned long pHLastTime = 0;
double OrpLastValue = 730.;
unsigned long OrpLastTime = 0;
double pHTab [3] {0.,0.,0.};
double ChlTab [3] {0.,0.,0.};
uint8_t iw = 0;
uint8_t jw = 0;
bool newpHOutput = false;
bool newChlOutput = false;
double pHCumul = 0.;
double ChlCumul = 0.;
#endif

// Firmware revision
String Firmw = FIRMW;

//Settings structure and its default values
// si pH+ : Kp=2250000.
// si pH- : Kp=2700000.

//Copy of Definition in PoolMaster.h
/*
struct StoreStruct
{
  uint8_t ConfigVersion;   // This is for testing if first time using eeprom or not
  String SSID, WIFI_PASS, MQTT_USER, MQTT_PASS, MQTT_NAME, SaltStatus, ResetTimestamp;
  IPAddress MQTT_IP;
  uint32_t MQTT_PORT, Uptime, LastUptimeUpdate;
  bool WIFI_OnOff, MQTTLOGIN_OnOff, BUS_A_B, Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, SolarLocExt, SolarOnline, SolarMode, Salt_Chlor, SaltMode, SaltPolarity, WinterMode, WaterHeat, ValveMode, CleanMode, ValveSwitch, WaterFillMode, HeatPumpMode;
  uint8_t FiltrationDuration, FiltrationStart, FiltrationStop, FiltrationStartMin, FiltrationStopMax, DelayPIDs, SolarStartMin, SolarStopMax, ResetReason, SolarPumpStatus, ValveStatus;
  uint8_t address_A_0[8], address_A_1[8], address_A_2[8], address_A_3[8], address_A_4[8], Array_A[5];
  uint8_t address_W_0[8], address_W_1[8], address_W_2[8], address_W_3[8], address_W_4[8], Array_W[5];
  unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit, WaterFillUpTimeLimit, WaterFillDuration, SaltPumpRunTime, PublishPeriod;
  unsigned long PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime, WaterFillAnCon;
  double Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, PSI_MedThreshold, FLOW_Pulse, FLOW_HighThreshold, FLOW_MedThreshold, FLOW2_Pulse, FLOW2_HighThreshold, FLOW2_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1, SaltDiff;
  double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, PhValue, PhRawValue, OrpValue, OrpRawValue, PSIValue, FLOWValue, FLOW2Value;
  double WaterSTemp, WaterITemp, WaterBTemp, WaterWPTemp, WaterWTTemp, AirInTemp, AirTemp, AirHum, AirPress, SolarTemp, SolarVLTemp, SolarRLTemp; 
  double AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR, WaterFillFR, SaltCurrentValue, FilterCurrentValue, HeatCurrentValue, SaltCurrentCalibCoeffs0, SaltCurrentCalibCoeffs1, FilterCurrentCalibCoeffs0, FilterCurrentCalibCoeffs1, HeatCurrentCalibCoeffs0HeatCurrentCalibCoeffs1;
  float SaltConcentration, CellConstant, SaltNeeded, PoolVolume,;
*/

#ifdef EXT_ADS1115

// Initialize StoreStruct with default values for External ADS1115
StoreStruct storage =
{ 
    CONFIG_VERSION, MATTER_NVS_VERSION/*MatterVersion*/,
    WIFI_SSID, WIFI_PASSWORD, ""/*MQTT_USER*/, ""/*MQTT_PASS*/, MQTT_SERVER_ID/*MQTT_NAME*/, "Unknown"/*SaltStatus*/,""/*ResetTimestamp*/,
    MQTT_SERVER_IP,
    MQTT_SERVER_PORT, 0U/*Uptime*/, 0U/*LastUptimeUpdate*/,
    1/*bool WIFI_OnOff*/, 1/*MQTTLOGIN_OnOff*/, 1/*BUS_A_B*/, 1/*Ph_RegulationOnOff*/, 0/*Orp_RegulationOnOff*/, 1/*AutoMode*/, 1/*SolarLocExt*/, 0/* SolarOnline*/, 1/*SolarMode*/, 1/*Salt_Chlor*/, 1/*SaltMode*/, 1/*SaltPolarity*/, 0/*WinterMode*/, 0/*WaterHeat*/, 1/*ValveMode*/, 0/*CleanMode*/, 0/*ValveSwitch*/, 0/*WaterFillMode*/, 0/*HeatPumpMode*/,
    13/*FiltrationDuration*/, 8/*FiltrationStart*/, 21/*FiltrationStop*/, 8/*FiltrationStartMin*/, 22/*FiltrationStopMax*/, 20/*DelayPIDs*/, 11/*SolarStartMin*/, 18/*SolarStopMax*/, 0U/*ResetReason*/, 0/*SolarPumpStatus*/, 0/*ValveStatus*/,
    // Air/Solar temperature sensor addresses
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // address_A_3: SolarTemp
    {0x28, 0xE1, 0xC4, 0xC0, 0x1B, 0x13, 0x01, 0x68}, // address_A_1: SolarVLTemp
    {0x28, 0x56, 0x26, 0xC6, 0x1B, 0x13, 0x01, 0xBA}, // address_A_2: SolarRLTemp
    {0x28, 0xAA, 0x6A, 0x96, 0x16, 0x13, 0x02, 0x57}, // address_A_0: AirInTemp
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // address_A_4: AirTemp
    {0, 1, 2, 3, 4}, // Array_A
    // Water temperature sensor addresses
    {0x28, 0x3C, 0x32, 0x6D, 0x1E, 0x13, 0x01, 0xDD}, // address_W_0: WaterSTemp -> Adresse ist ,it WaterITemp getauscht weil der Skimmer Sensor defekt ist!
    {0x28, 0xAA, 0x75, 0xA5, 0x13, 0x13, 0x02, 0xB1}, // address_W_1: WaterITemp
    {0x28, 0xAA, 0x12, 0x90, 0x16, 0x13, 0x02, 0x0D}, // address_W_2: WaterBTemp
    {0x28, 0xAA, 0x0F, 0x8D, 0x16, 0x13, 0x02, 0x38}, // address_W_3: WaterWPTemp
    {0x28, 0xAA, 0xCA, 0x93, 0x13, 0x13, 0x02, 0xD0}, // address_W_4: WaterWTTemp
    {0, 1, 2, 3, 4}, // Array_W
    900000UL/*PhPumpUpTimeLimit ms (15 min)*/, 2500000UL/*ChlPumpUpTimeLimit ms (~41.7 min, ex 2500 s)*/, 900000/*WaterFillUpTimeLimit*/, 300000/* WaterFillDuration*/, 0/*SaltPumpRunTime*/, 30000/*PublishPeriod*/,
    1800000/*PhPIDWindowSize*/, 1800000/*OrpPIDWindowSize*/, 0/*PhPIDwindowStartTime*/, 0/*OrpPIDwindowStartTime*/, 0/*WaterFillAnCon*/,
    7.2/*Ph_SetPoint*/, 740.0/*Orp_SetPoint*/, 1.5/*PSI_HighThreshold*/, 0.3/*PSI_MedThreshold*/, 0.8/*FLOW_Pulse*/, 120.0/*FLOW_HighThreshold*/, 60.0/*FLOW_MedThreshold*/, 4.5/*FLOW2_Pulse*/, 60/*FLOW2_HighThreshold*/, 1.0/*FLOW2_MedThreshold*/, 10.0/*WaterTempLowThreshold*/, 30.0/*WaterTemp_SetPoint*/, -2.3183/*pHCalibCoeffs0*/, 6.68/*pHCalibCoeffs1*/, 465.0/*OrpCalibCoeffs0*/, 0.0/*OrpCalibCoeffs1*/, 1.31/*PSICalibCoeffs0*/, -0.1/*PSICalibCoeffs1*/, 30.0/*SaltDiff*/,
    2700000.0/*Ph_Kp*/, 0.0/*Ph_Ki*/, 0.0/*Ph_Kd*/, 18000.0/*Orp_Kp*/, 0.0/*Orp_Ki*/, 0.0/*Orp_Kd*/, 0.0/*PhPIDOutput*/, 0.0/*OrpPIDOutput*/, 6.8/*PhValue*/, 0.0/*PhRawValue*/, 720./*OrpValue*/, 0.0/*OrpRawValue*/, 1.3/*PSIValue*/, 70/*FLOWValue*/, 9/*FLOW2Value*/,
    0.0/*WaterSTemp*/, 0.0/*WaterITemp*/, 0.0/*WaterBTemp*/, 0.0/*WaterWPTemp*/, 0.0/*WaterWTTemp*/, 0.0/*AirInTemp*/, 0.0/*AirTemp*/, 0.0/*AirHum*/, 0.0/*AirPress*/, 0.0/*SolarTemp*/, 0.0/*SolarVLTemp*/, 0.0/*SolarRLTemp*/,
    25.0/*AcidFill*/, 60.0/*ChlFill*/, 20.0/*pHTankVol*/, 20.0/*ChlTankVol*/, 2.7/*pHPumpFR*/, 2.7/*ChlPumpFR*/, 15.0/*WaterFillFR*/, 0.0/*SaltCurrentValue*/, 0.0/*FilterCurrentValue*/, 0.0/*HeatCurrentValue*/, 10.0/*SaltCurrentCalibCoeffs0*/, -25.0/*SaltCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A*/, 10.0/*FilterCurrentCalibCoeffs0*/, -25.0/*FilterCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A */, 10.0/*HeatCurrentCalibCoeffs0*/, -25.0/*HeatCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A*/,
    0.0/*SaltConcentration*/, 5.0/*CellConstant*/, 0.0/*SaltNeeded*/, POOL_VOLUME/*PoolVolume*/,
  };
#else
// Initialize StoreStruct with default values for Internal ADS1115
StoreStruct storage =
{
    CONFIG_VERSION, MATTER_NVS_VERSION/*MatterVersion*/,
    WIFI_SSID, WIFI_PASSWORD, ""/*MQTT_USER*/, ""/*MQTT_PASS*/, MQTT_SERVER_ID/*MQTT_NAME*/, "Unknown"/*SaltStatus*/, ""/*ResetTimestamp*/,
    MQTT_SERVER_IP,
    MQTT_SERVER_PORT, 0U/*Uptime*/, 0U/*LastUptimeUpdate*/,
    1/*bool WIFI_OnOff*/, 1/*MQTTLOGIN_OnOff*/, 1/*BUS_A_B*/, 1/*Ph_RegulationOnOff*/, 0/*Orp_RegulationOnOff*/, 1/*AutoMode*/, 1/*SolarLocExt*/, 0/* SolarOnline*/, 1/*SolarMode*/, 1/*Salt_Chlor*/, 1/*SaltMode*/, 1/*SaltPolarity*/, 0/*WinterMode*/, 0/*WaterHeat*/, 1/*ValveMode*/, 0/*CleanMode*/, 0/*ValveSwitch*/, 0/*WaterFillMode*/, 0/*HeatPumpMode*/,
    13/*FiltrationDuration*/, 8/*FiltrationStart*/, 21/*FiltrationStop*/, 8/*FiltrationStartMin*/, 22/*FiltrationStopMax*/, 20/*DelayPIDs*/, 11/*SolarStartMin*/, 18/*SolarStopMax*/, 0U/*ResetReason*/, 0/*SolarPumpStatus*/, 0/*ValveStatus*/,
    // Air/Solar temperature sensor addresses
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // address_A_3: unused
    {0x28, 0xE1, 0xC4, 0xC0, 0x1B, 0x13, 0x01, 0x68}, // address_A_1: SolarVLTemp
    {0x28, 0x56, 0x26, 0xC6, 0x1B, 0x13, 0x01, 0xBA}, // address_A_2: SolarRLTemp
    {0x28, 0xAA, 0x6A, 0x96, 0x16, 0x13, 0x02, 0x57}, // address_A_0: AirInTemp
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // address_A_4: unused
    {0, 1, 2, 3, 4}, // Array_A
    // Water temperature sensor addresses
    {0x28, 0xAA, 0x75, 0xA5, 0x13, 0x13, 0x02, 0xB1}, // address_W_0: WaterSTemp
    {0x28, 0x3C, 0x32, 0x6D, 0x1E, 0x13, 0x01, 0xDD}, // address_W_1: WaterITemp
    {0x28, 0xAA, 0x12, 0x90, 0x16, 0x13, 0x02, 0x0D}, // address_W_2: WaterBTemp
    {0x28, 0xAA, 0x0F, 0x8D, 0x16, 0x13, 0x02, 0x38}, // address_W_3: WaterWPTemp
    {0x28, 0xAA, 0xCA, 0x93, 0x13, 0x13, 0x02, 0xD0}, // address_W_4: WaterWTTemp
    {0, 1, 2, 3, 4}, // Array_W
    900000UL/*PhPumpUpTimeLimit ms (15 min)*/, 2500000UL/*ChlPumpUpTimeLimit ms (~41.7 min, ex 2500 s)*/, 900000/*WaterFillUpTimeLimit*/, 300000/* WaterFillDuration*/, 0/*SaltPumpRunTime*/, 30000/*PublishPeriod*/,
    1800000/*PhPIDWindowSize*/, 1800000/*OrpPIDWindowSize*/, 0/*PhPIDwindowStartTime*/, 0/*OrpPIDwindowStartTime*/, 0/*WaterFillAnCon*/,
    7.2/*Ph_SetPoint*/, 740.0/*Orp_SetPoint*/, 1.5/*PSI_HighThreshold*/, 0.3/*PSI_MedThreshold*/, 0.8/*FLOW_Pulse*/, 120.0/*FLOW_HighThreshold*/, 60.0/*FLOW_MedThreshold*/, 4.5/*FLOW2_Pulse*/, 60/*FLOW2_HighThreshold*/, 1.0/*FLOW2_MedThreshold*/, 10.0/*WaterTempLowThreshold*/, 30.0/*WaterTemp_SetPoint*/, 3.61078313/*pHCalibCoeffs0*/, -3.88020422/*pHCalibCoeffs1*/, -966.946396/*OrpCalibCoeffs0*/, 2526.88809/*OrpCalibCoeffs1*/, 1.31/*PSICalibCoeffs0*/, -0.1/*PSICalibCoeffs1*/, 30.0/*SaltDiff*/,
    2700000.0/*Ph_Kp*/, 0.0/*Ph_Ki*/, 0.0/*Ph_Kd*/, 18000.0/*Orp_Kp*/, 0.0/*Orp_Ki*/, 0.0/*Orp_Kd*/, 0.0/*PhPIDOutput*/, 0.0/*OrpPIDOutput*/, 6.8/*PhValue*/, 0.0/*PhRawValue*/, 720./*OrpValue*/, 0.0/*OrpRawValue*/, 1.3/*PSIValue*/, 70/*FLOWValue*/, 9/*FLOW2Value*/,
    0.0/*WaterSTemp*/, 0.0/*WaterITemp*/, 0.0/*WaterBTemp*/, 0.0/*WaterWPTemp*/, 0.0/*WaterWTTemp*/, 0.0/*AirInTemp*/, 0.0/*AirTemp*/, 0.0/*AirHum*/, 0.0/*AirPress*/, 0.0/*SolarTemp*/, 0.0/*SolarVLTemp*/, 0.0/*SolarRLTemp*/,
    25.0/*AcidFill*/, 60.0/*ChlFill*/, 20.0/*pHTankVol*/, 20.0/*ChlTankVol*/, 2.7/*pHPumpFR*/, 2.7/*ChlPumpFR*/, 15.0/*WaterFillFR*/, 0.0/*SaltCurrentValue*/, 0.0/*FilterCurrentValue*/, 0.0/*HeatCurrentValue*/, 10.0/*SaltCurrentCalibCoeffs0*/, -25.0/*SaltCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A*/, 10.0/*FilterCurrentCalibCoeffs0*/, -25.0/*FilterCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A */, 10.0/*HeatCurrentCalibCoeffs0*/, -25.0/*HeatCurrentCalibCoeffs1 | 100 mV/A, 2.5V bei 0A*/,
    0.0/*SaltConcentration*/, 5.0/*CellConstant*/, 0.0/*SaltNeeded*/, POOL_VOLUME/*PoolVolume*/,
  };
#endif

// RTC Declare module type
RTC_DS3231 rtc;

// Declare BME280 module
Adafruit_BME280 bme;

tm timeinfo;

struct PCFDevice {
    uint8_t address;
    void* instance;
};

const PCFDevice pcfDevices[] = {
    {PCF8574_ADR, nullptr},
    {PCF8574_I_ADR, nullptr},
    {PCF8574_II_ADR, nullptr},
    {PCF8574_III_ADR, nullptr}
};

// Define RELAY-PINS for all Pumps (Second PCF8574_I)
PCF_Pin FILTRATION_PUMP_PIN = {FILTRATION_PUMP, PCF8574_I_ADR}; // P0
PCF_Pin HEAT_PUMP_PIN       = {HEAT_PUMP, PCF8574_I_ADR};       // P1
PCF_Pin SALT_PUMP_PIN       = {SALT_PUMP, PCF8574_I_ADR};       // P2
PCF_Pin ROBOT_PUMP_PIN      = {ROBOT_PUMP, PCF8574_I_ADR};      // P3
PCF_Pin PH_PUMP_PIN         = {PH_PUMP, PCF8574_I_ADR};         // P4
PCF_Pin CHL_PUMP_PIN        = {CHL_PUMP, PCF8574_I_ADR};        // P5
PCF_Pin SOLAR_PUMP_PIN      = {SOLAR_PUMP, PCF8574_I_ADR};      // P6
// SALT_POL (GPIO 13) ist kein PCF8574-Pin — Polaritätsumschaltung via direktem digitalWrite() in Loops.cpp

// Define RELAY-PINS for MotorValves (Third PCF8574_II)
PCF_Pin ESD_TRE_OPEN_PIN    = {ESD_TRE_OPEN, PCF8574_II_ADR};   // P0
PCF_Pin ESD_TRE_CLOSE_PIN   = {ESD_TRE_CLOSE, PCF8574_II_ADR};  // P1
PCF_Pin ESD_HIN_OPEN_PIN    = {ESD_HIN_OPEN, PCF8574_II_ADR};   // P2
PCF_Pin ESD_HIN_CLOSE_PIN   = {ESD_HIN_CLOSE, PCF8574_II_ADR};  // P3
PCF_Pin WP_VL_OPEN_PIN      = {WP_VL_OPEN, PCF8574_II_ADR};     // P4
PCF_Pin WP_VL_CLOSE_PIN     = {WP_VL_CLOSE, PCF8574_II_ADR};    // P5
PCF_Pin WP_M_OPEN_PIN       = {WP_M_OPEN, PCF8574_II_ADR};      // P6
PCF_Pin WP_M_CLOSE_PIN      = {WP_M_CLOSE, PCF8574_II_ADR};     // P7

// Define RELAY-PINS for MotorValves (Fourth PCF8574_III)
PCF_Pin BODEN_OPEN_PIN      = {BODEN_OPEN, PCF8574_III_ADR};    // P0
PCF_Pin BODEN_CLOSE_PIN     = {BODEN_CLOSE, PCF8574_III_ADR};   // P1
PCF_Pin SOLAR_OPEN_PIN      = {SOLAR_OPEN, PCF8574_III_ADR};    // P2
PCF_Pin SOLAR_CLOSE_PIN     = {SOLAR_CLOSE, PCF8574_III_ADR};   // P3
PCF_Pin SPARE_I_OPEN_PIN    = {SPARE_I_OPEN, PCF8574_III_ADR};  // P4
PCF_Pin SPARE_I_CLOSE_PIN   = {SPARE_I_CLOSE, PCF8574_III_ADR}; // P5
PCF_Pin WATER_FILL_PIN      = {WATER_FILL, PCF8574_III_ADR};    // P6
PCF_Pin HEAT_ON_PIN         = {HEAT_ON, PCF8574_III_ADR};       // P7

// Mutex to share access to I2C bus among tasks: AnalogPoll, StatusLights, RTC, BME280
SemaphoreHandle_t mutex = NULL;
TaskHandle_t mutexOwner = NULL;

// Separate mutex for MQTT publish operations (must NOT reuse the I2C mutex —
// holding I2C while waiting for MQTT would stall all I2C peripherals).
SemaphoreHandle_t mqttMutex = NULL;

// Mutex to protect I2C states and outputs
SemaphoreHandle_t i2cStatesMutex = NULL;
SemaphoreHandle_t i2cOutputMutex = NULL;

// Various global flags
volatile bool startTasks = false;               // Signal to start loop tasks
bool RTCfound = false;

bool AntiFreezeFiltering = false;               // Filtration anti freeze mode
bool EmergencyStopFiltPump = false;             // flag will be (re)set by double-tapp button
bool PSIError = false;                          // Water pressure OK
bool FLOWError = false;                         // Water flow in Main-Pipe OK
bool FLOW2Error = false;                        // Water flow in Meassure-Pipe OK
bool WaterFillError = false;                    // Waterfill system is OK
bool I2CError = false;                          // I2C-Hardware is OK
bool cleaning_done = false;                     // daily cleaning done

// Queue object to store incoming JSON commands (up to 10)
QueueHandle_t queueIn;

SemaphoreHandle_t prefsMutex = nullptr;

void prefsLock(void)
{
    if (prefsMutex != nullptr) {
        xSemaphoreTakeRecursive(prefsMutex, portMAX_DELAY);
    }
}

void prefsUnlock(void)
{
    if (prefsMutex != nullptr) {
        xSemaphoreGiveRecursive(prefsMutex);
    }
}

// NVS Non Volatile SRAM (eqv. EEPROM)
Preferences nvs;     

// Instanciations of Pump and PID objects to make them global. But the constructors are then called 
// before loading of the storage struct. At run time, the attributes take the default
// values of the storage struct as they are compiled, just a few lines above, and not those which will 
// be read from NVS later. This means that the correct objects attributes must be set later in
// the setup function (fortunatelly, init methods exist).

// The seven pumps of the system (instanciate the Pump class)
// In this case, all pumps start/Stop are managed by relays. pH, ORP, Salt, SolarHeat, Heat and Robot pumps are interlocked with 
// filtration pump
PCF_Pump FiltrationPump(FILTRATION_PUMP_PIN, FILTRATION_PUMP_PIN, NO_PIN, NO_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump PhPump(PH_PUMP_PIN, PH_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, storage.pHPumpFR, storage.pHTankVol, storage.AcidFill, true);
PCF_Pump ChlPump(CHL_PUMP_PIN, CHL_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, storage.ChlPumpFR, storage.ChlTankVol, storage.ChlFill, true);
PCF_Pump RobotPump(ROBOT_PUMP_PIN, ROBOT_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump SaltPump(SALT_PUMP_PIN, SALT_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump HeatPump(HEAT_PUMP_PIN, HEAT_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump WaterHeatPump(HEAT_ON_PIN, HEAT_ON_PIN, NO_PIN, FILTRATION_PUMP_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump SolarPump(SOLAR_PUMP_PIN, SOLAR_PUMP_PIN, NO_PIN, FILTRATION_PUMP_PIN, 0.0, 0.0, 0.0, true);
PCF_Pump WaterFill(WATER_FILL_PIN, WATER_FILL_PIN, NO_PIN, NO_PIN, 0.0, 0.0, 0.0, true);

// Naming for six MotorValve Instances
char Motorvalve_1[] = "ELD_TREPPE";
char Motorvalve_2[] = "ELD_HINTEN";
char Motorvalve_3[] = "WP_VORLAUF";
char Motorvalve_4[] = "WP_MISCHER";
char Motorvalve_5[] = "BODENABLAUF";
char Motorvalve_6[] = "SOLAR_VALVE";

// The six motorvalves of the system (instanciate the MotorValve class)
MotorValve ELD_Treppe(ESD_TRE_OPEN_PIN, ESD_TRE_CLOSE_PIN, STARTANGLE_0, MAX_90, TIMETOMAX_90, COUNTER_CLOCKWISE, Motorvalve_1);
MotorValve ELD_Hinten(ESD_HIN_OPEN_PIN, ESD_HIN_CLOSE_PIN, STARTANGLE_0, MAX_90, TIMETOMAX_90, COUNTER_CLOCKWISE, Motorvalve_2);
MotorValve WP_Vorlauf(WP_VL_OPEN_PIN, WP_VL_CLOSE_PIN, STARTANGLE_0, MAX_90, TIMETOMAX_90, COUNTER_CLOCKWISE, Motorvalve_3);
MotorValve WP_Mischer(WP_M_OPEN_PIN, WP_M_CLOSE_PIN, STARTANGLE_0, MAX_45, TIMETOMAX_45, COUNTER_CLOCKWISE, Motorvalve_4);
MotorValve Bodenablauf(BODEN_OPEN_PIN, BODEN_CLOSE_PIN, STARTANGLE_0, MAX_90, TIMETOMAX_90, COUNTER_CLOCKWISE, Motorvalve_5);
MotorValve Solarvalve(SOLAR_OPEN_PIN, SOLAR_CLOSE_PIN, STARTANGLE_0, MAX_90, TIMETOMAX_90, COUNTER_CLOCKWISE, Motorvalve_6);

// PIDs instances
//Specify the direction and initial tuning parameters
PID PhPID(&storage.PhValue, &storage.PhPIDOutput, &storage.Ph_SetPoint, storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd, PhPID_DIRECTION);
PID OrpPID(&storage.OrpValue, &storage.OrpPIDOutput, &storage.Orp_SetPoint, storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd, OrpPID_DIRECTION);

// Publishing tasks handles to notify them
static TaskHandle_t pubSetTaskHandle;
static TaskHandle_t pubMeasTaskHandle;

// Functions prototypes
void checkI2CBus(void);
void scanI2CBus(void);
void StartTime(void);
void readLocalTime(void);
bool loadConfig(void);
bool saveConfig(void);
void WiFiEvent(WiFiEvent_t);
void initTimers(void);
void connectToWiFi(void);                  
void InitTFT(void);
void ResetTFT(void);
void PublishSettings(void);
void SetPhPID(bool);
void SetOrpPID(bool);
int  freeRam (void);
bool lockI2C(void);
void unlockI2C(void);
void AnalogInit(void);
void FlowInit(void);
void Flow2Init(void);
void TempInit(void);
void bme280Init(void);
void saveSensorMapping(const char* sensorMapping[], uint8_t ds18b20Mapping[], uint8_t numSensors);
void RTCInit(void);
void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle);
// Formats an uptime value (milliseconds) as "DDd HHh MMm SSs"
String formatUptime(uint32_t uptimeMs) {
    uint32_t s  = uptimeMs / 1000;
    uint32_t m  = s / 60;  s %= 60;
    uint32_t h  = m / 60;  m %= 60;
    uint32_t d  = h / 24;  h %= 24;
    char buf[32];
    snprintf(buf, sizeof(buf), "%ud %02uh %02um %02us", (unsigned)d, (unsigned)h, (unsigned)m, (unsigned)s);
    return String(buf);
}


bool saveParam(const char* key, uint8_t val);
bool saveParam(const char* key, bool val);
bool saveParam(const char* key, unsigned long val);
bool saveParam(const char* key, String val);
bool saveParam(const char* key, const uint8_t* val, size_t size);
bool saveParam(const char* key, double val);

unsigned stack_hwm();
void stack_mon(UBaseType_t&);
void setupPCF8574States(void);
void info();

// Functions used as Tasks
void PoolMaster(void*);
void AnalogPoll(void*);
void pHRegulation(void*);
void OrpRegulation(void*);
void SaltRegulation(void*);
void getTemp(void*);
void readBME280(void*);
void ProcessCommand(void*);
void FlowMeasures(void*);
void SettingsPublish(void*);
void MeasuresPublish(void*);
void StatusLights(void*);
void clearStatusLEDs();
void initPCF8574();

// Setup
void setup()
{
  //Serial port for debug info
  Serial.begin(115200);

  //nvs_flash_erase();

  // Set appropriate debug level. The level is defined in PoolMaster.h
  Debug.setDebugLevel(DEBUG_LEVEL);
  Debug.timestampOn();
  Debug.debugLabelOn();
  Debug.formatTimestampOn();

  //get board info
  info();

  // Initialize Nextion TFT
  InitTFT();
  ResetTFT();

  // Create I2C sharing mutex zuerst, bevor irgendwelche I2C-Operationen
  mutex = xSemaphoreCreateRecursiveMutex();
  if (!mutex) {
      Debug.print(DBG_ERROR, "[SETUP] Failed to create I2C mutex");
      while (1);  // Abbruch bei Fehler
  }

  // Create dedicated MQTT mutex (separate from I2C to avoid cross-blocking)
  mqttMutex = xSemaphoreCreateMutex();
  if (!mqttMutex) {
      Debug.print(DBG_ERROR, "[SETUP] Failed to create MQTT mutex");
      while (1);
  }

  prefsMutex = xSemaphoreCreateRecursiveMutex();
  if (!prefsMutex) {
      Debug.print(DBG_ERROR, "[SETUP] Failed to create prefs mutex");
      while (1);
  }

  // Start I2C
  static bool i2cInitialized = false;
  if (!i2cInitialized) {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeout(100); // 100 ms Timeout
    i2cInitialized = true;
    Debug.print(DBG_INFO, "[SETUP] I2C bus initialized (SDA=%d, SCL=%d)", I2C_SDA, I2C_SCL);
  } else {
    Debug.print(DBG_WARNING, "[SETUP] I2C bus already initialized, skipping Wire.begin()");
  }

  PCF8574Manager& pcfManager = PCF8574Manager::getInstance();
    pcfManager.init();
  Debug.print(DBG_INFO, "[SETUP] PCF8574Manager initialized");

  // Setze PCF8574 states (optional, da bereits in Deklaration 0xFF)
  Debug.print(DBG_INFO, "[SETUP] PCF states initialized to 0xFF (all off)");

  //Read ConfigVersion. If does not match expected value, restore default values
  if(nvs.begin("PoolMaster",true))
  {
    uint8_t vers = nvs.getUChar("ConfigVersion",0);
    Debug.print(DBG_INFO,"Stored version: %d",vers);
    nvs.end();

    if (vers == CONFIG_VERSION)
    {
      Debug.print(DBG_INFO,"Same version: %d / %d. Loading settings from NVS",vers,CONFIG_VERSION);
      if(loadConfig()) Debug.print(DBG_INFO,"Config loaded"); //Restore stored values from NVS
    }
    else
    {
      Debug.print(DBG_INFO,"New version: %d / %d. Loading new default settings",vers,CONFIG_VERSION);      
      if(saveConfig()) Debug.print(DBG_INFO,"Config saved");  //First time use. Save new default values to NVS
    }

  } else {
    Debug.print(DBG_ERROR,"NVS Error");
    nvs.end();
    Debug.print(DBG_INFO,"New version: %d. First saving of settings",CONFIG_VERSION);      
      if(saveConfig()) Debug.print(DBG_INFO,"Config saved");  //First time use. Save new default values to NVS

  }

  // Configure watch-dog (TWDT is already initialized by the framework;
  // use reconfigure to avoid the "TWDT already initialized" error in IDF 5.x)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT,
    .trigger_panic = true,
  };
  esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_config);
  if (wdt_err != ESP_OK) {
    // Fallback: deinit + reinit (e.g. if framework left it uninitialized)
    esp_task_wdt_deinit();
    esp_task_wdt_init(&wdt_config);
  }

  // Initalize the RTC module
  RTCInit();

  // ── Matter NVS version check ─────────────────────────────────────────────────
#ifdef MATTER_ENABLED
  if (storage.MatterVersion != MATTER_NVS_VERSION) {
    Debug.print(DBG_WARNING, "[SETUP] Matter NVS version mismatch (%d->%d), erasing Matter NVS...",
                storage.MatterVersion, MATTER_NVS_VERSION);
    prefsLock();
    Preferences matterNvs;
    for (const char* ns : {"chip-kvs", "chip-counters", "chip-config"}) {
      if (matterNvs.begin(ns, false)) {
        matterNvs.clear();
        matterNvs.end();
        Debug.print(DBG_INFO, "[SETUP] Erased NVS namespace: %s", ns);
      }
    }
    storage.MatterVersion = MATTER_NVS_VERSION;
    saveConfig();
    prefsUnlock();
    Debug.print(DBG_INFO, "[SETUP] Matter NVS reset — re-commissioning required");
  }
#endif

  // ── Matter Phase 1: create node + endpoints ────────────────────────────────
#ifdef MATTER_ENABLED
  Debug.print(DBG_INFO, "[SETUP] Matter Phase 1: creating endpoints...");
  matterBridgeInit();
#endif

  // ── Matter Phase 2: start CHIP stack + BLE (BEFORE WiFi) ───────────────────
  // BLE controller requires a large contiguous internal-RAM block. WiFi driver
  // init (esp_wifi_init inside WiFi.begin()) fragments the heap heavily, leaving
  // no block large enough for BLE → crash. Starting Matter/BLE here, before
  // WiFi allocates its buffers, ensures BLE gets the contiguous block it needs.
  // The CHIP stack handles WiFi-readiness internally via platform events.
#ifdef MATTER_ENABLED
  Debug.print(DBG_INFO, "[SETUP] Matter Phase 2: starting CHIP stack (before WiFi)...");
  matterBridgeStart();
#endif

  Debug.print(DBG_INFO, "[SETUP] Initializing MQTT...");
  mqttInit();

#ifndef MATTER_ENABLED
  // Initialize WiFi events management (on connect/disconnect).
  // In MATTER_ENABLED mode der Arduino-WiFi-Stack wird nie initialisiert
  // (siehe connectToWiFi() in mqtt_comm.cpp). Stattdessen registriert
  // registerMatterWiFiHandlers() eigene esp-idf-Handler — der Aufruf hier
  // hätte den Arduino-WiFi-Layer "lazy" angetriggert und Netifs ein zweites
  // Mal registrieren können (gleiches Failure-Muster wie SolarControl
  // matter_dev beschreibt).
  WiFi.onEvent(WiFiEvent);
#endif
  initTimers();
#ifdef MATTER_ENABLED
#if MATTER_NO_MQTT_UNTIL_COMMISSIONED
  matterApplyRadioHoldAfterTimersReady();
#endif
#endif
  connectToWiFi();

  // Wait for WiFi — but never block the core startup.
  // In MATTER_ENABLED mode the CHIP stack manages WiFi via esp-idf; WiFi.status()
  // always returns WL_DISCONNECTED from Arduino's view.  Use esp_wifi_sta_get_ap_info()
  // instead.  In non-Matter mode we give WiFi up to 15 s, then continue.
  {
    uint32_t wifiDeadline = millis() + 15000UL;
    bool wifiConnected = false;
    while (!wifiConnected && millis() < wifiDeadline) {
      wifiConnected = wifiIsConnected();
      if (!wifiConnected) { delay(500); Serial.print("."); }
    }
    if (!wifiConnected)
      Debug.print(DBG_WARNING, "[SETUP] WiFi not available — continuing without WiFi/MQTT");
  }

  // Time init: NTP if WiFi is up, RTC fallback otherwise.
  // Both StartTime() and readLocalTime() handle the no-WiFi case gracefully.
  StartTime();
  readLocalTime();
  setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year-100);
#ifdef MATTER_ENABLED
  // matterBridgeStart() runs earlier, before NTP; push real UTC into CHIP (not Y2K).
  matterResyncChipWallClockAfterNtp();
#endif
  Debug.print(DBG_INFO,"%d/%02d/%02d %02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());

  // Save reset reason and timestamp
  storage.ResetReason = esp_reset_reason();
  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
          year(), month(), day(), hour(), minute(), second());
  storage.ResetTimestamp = String(timestamp);
  saveParam("ResetReason", storage.ResetReason);
  saveParam("ResetTimestamp", storage.ResetTimestamp);
  Debug.print(DBG_INFO, "[SETUP] Reset reason: %s, Timestamp: %s",
              resetReasonToString(storage.ResetReason), storage.ResetTimestamp.c_str());

  // Initialize the mDNS library and OTA — only when WiFi is actually up
  if (storage.WIFI_OnOff && wifiIsConnected()) {
    if (!MDNS.begin("PoolMaster"))
      Debug.print(DBG_WARNING, "[SETUP] mDNS start failed — skipping");
    else
      MDNS.addService("http", "tcp", OTA_NEXTION_PORT);

    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOST);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { /* ... */ });
    ArduinoOTA.onEnd([]() { /* ... */ });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { /* ... */ });
    ArduinoOTA.onError([](ota_error_t error) { /* ... */ });
    ArduinoOTA.begin();
  }

// reset JTAG-Pins
  gpio_reset_pin(GPIO_NUM_39);  // IO39
  gpio_reset_pin(GPIO_NUM_40);  // IO40
  gpio_reset_pin(GPIO_NUM_41);  // IO41
  gpio_reset_pin(GPIO_NUM_42);  // IO42

  //Define pins directions
  pinMode(LIGHT_POOL, OUTPUT);
  pinMode(LIGHT_ROOM, OUTPUT);
  pinMode(SALT_POL, OUTPUT);
  pinMode(RELAY_R1, OUTPUT);
  pinMode(RELAY_R2, OUTPUT);
  pinMode(RELAY_R3, OUTPUT);
  pinMode(RELAY_R4, OUTPUT);
  pinMode(RELAY_R5, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  // As the relays on the board are activated by a LOW level, set all levels HIGH at startup
  digitalWrite(LIGHT_POOL,HIGH);
  digitalWrite(LIGHT_ROOM,HIGH);
  digitalWrite(SALT_POL,HIGH);
  digitalWrite(RELAY_R1,HIGH);
  digitalWrite(RELAY_R2,HIGH);
  digitalWrite(RELAY_R3,HIGH);
  digitalWrite(RELAY_R4,HIGH);  
  digitalWrite(RELAY_R5,HIGH);
  

// Warning: pins used here have no pull-ups, provide external ones
  pinMode(FLOW, INPUT);
  pinMode(FLOW2, INPUT);
  pinMode(WATER_MAX_LVL, INPUT_PULLUP);
  pinMode(WATER_MIN_LVL, INPUT_PULLUP);

  // Scan I2C-Bus and check if there errors
  //scanI2CBus();
  //checkI2CBus();

  // Initalize the BME280 sensor
  bme280Init();

  // Init Water and Air temperatures measurements with DS18B20-Sensors
  TempInit();

  // Init pH, ORP and PSI analog measurements
  AnalogInit();

  // Init Flow measurements
  FlowInit();
  Flow2Init();

  // Clear status LEDs
  clearStatusLEDs();

  // Initialize PIDs
  storage.PhPIDwindowStartTime  = millis();
  storage.OrpPIDwindowStartTime = millis();

  // Limit the PIDs output range in order to limit max. pumps runtime (safety first...)
  PhPID.SetTunings(storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd);
  PhPID.SetControllerDirection(PhPID_DIRECTION);
  PhPID.SetSampleTime((int)storage.PhPIDWindowSize);
  PhPID.SetOutputLimits(0, storage.PhPIDWindowSize);    //Whatever happens, don't allow continuous injection of Acid for more than a PID Window

  OrpPID.SetTunings(storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd);
  OrpPID.SetControllerDirection(OrpPID_DIRECTION);
  OrpPID.SetSampleTime((int)storage.OrpPIDWindowSize);
  OrpPID.SetOutputLimits(0, storage.OrpPIDWindowSize);  //Whatever happens, don't allow continuous injection of Chl for more than a PID Window

 // PIDs off at start
  SetPhPID (false);
  SetOrpPID(false);

  //Initialize pump instances with stored config data
  FiltrationPump.SetMaxUpTime(0);     //no runtime limit for the filtration pump
  SolarPump.SetMaxUpTime(0);          //no runtime limit for the Solarpump
  HeatPump.SetMaxUpTime(0);           //no runtime limit for the heatpump
  WaterHeatPump.SetMaxUpTime(0);      //no runtime limit for the waterheatpump
  SaltPump.SetMaxUpTime(0);           //no runtime limit for the saltmanager
  RobotPump.SetMaxUpTime(0);          //no runtime limit for the robot pump

  PhPump.SetFlowRate(storage.pHPumpFR);
  PhPump.SetTankVolume(storage.pHTankVol);
  PhPump.SetTankFill(storage.AcidFill);
  PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit);

  ChlPump.SetFlowRate(storage.ChlPumpFR);
  ChlPump.SetTankVolume(storage.ChlTankVol);
  ChlPump.SetTankFill(storage.ChlFill);
  ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit);

  //WaterFill.SetFlowRate(storage.WaterFillFR);
  WaterFill.SetMaxUpTime(storage.WaterFillUpTimeLimit);
  WaterFill.Stop(); // Safety: ensure valve is physically closed on every startup

  // Start filtration pump at power-on if within scheduled time slots -- You can choose not to do this and start pump manually
  if (storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
    FiltrationPump.Start();
  else FiltrationPump.Stop();

  // Initialize OTA for Nextion display with SPIFFS fix
  Debug.print(DBG_INFO, "[SETUP] Initializing OTA...");
  initOTA();

  // Create queue for external commands
  queueIn = xQueueCreate((UBaseType_t)QUEUE_ITEMS_NBR, (UBaseType_t)QUEUE_ITEM_SIZE);
  if (queueIn == NULL) {
    Debug.print(DBG_ERROR, "[SETUP] Failed to create queueIn");
    while (1);  // Stopp for debugging
  }
  Debug.print(DBG_INFO, "[SETUP] Queue created, handle: %p, item size: %d, items: %d", queueIn, QUEUE_ITEM_SIZE, QUEUE_ITEMS_NBR);

  int app_cpu = xPortGetCoreID();
  Debug.print(DBG_DEBUG, "Creating loop Tasks");
  createTasks(app_cpu, &pubSetTaskHandle, &pubMeasTaskHandle);

  // MQTT-Initialisierung und Verbindung
  Debug.print(DBG_INFO, "[SETUP] Connecting to MQTT...");
  connectToMqtt(); // Tries to connect to the mqtt-broker

  //display remaining RAM/Heap space.
  Debug.print(DBG_DEBUG,"[memCheck] Stack: %d bytes - Heap: %d bytes",stack_hwm(),freeRam());

  // Start loops tasks
  Debug.print(DBG_INFO,"Init done, starting loop tasks");
  startTasks = true;

  Debug.print(DBG_INFO, "[SETUP] Setup completed");
 
  delay(1000);          // wait for tasks to start

  // Calibrate MotorValves at start
  ELD_Treppe.calibrate();
  ELD_Hinten.calibrate();
  WP_Vorlauf.calibrate();
  WP_Mischer.calibrate();
  Bodenablauf.calibrate();
  if(storage.SolarLocExt)
  Solarvalve.calibrate();
  }

// NVS: Gixy31 speicherte Sekunden; dieser Fork speichert Millisekunden (ganze Minuten, Vielfaches von 60000).
static unsigned long migratePoolUpLimitToMs(unsigned long v) {
  if (v == 0) return 0;
  if (v >= 60000UL && (v % 60000UL) == 0UL) return v;
  if (v <= 86400UL) return v * 1000UL;
  return v;
}

namespace {
struct PrefsGuard {
  PrefsGuard() { prefsLock(); }
  ~PrefsGuard() { prefsUnlock(); }
};
} // namespace

bool loadConfig() {
  PrefsGuard g;
  if (!nvs.begin("PoolMaster", true)) {
      Debug.print(DBG_ERROR, "Failed to open NVS for reading");
      return false;
  }
  storage.ConfigVersion         = nvs.getUChar("ConfigVersion",0);
  storage.MatterVersion         = nvs.getUChar("MatterVersion",0);
  uint8_t tempIP[4] = {192, 168, 178, 55}; // Default IP if not found in NVS
  size_t len = 4;
  if (nvs.getBytes("MQTT_IP", tempIP, len) == 4) {
    storage.MQTT_IP = IPAddress(tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    Debug.print(DBG_INFO, "Loaded MQTT_IP: %s", storage.MQTT_IP.toString().c_str());
  } else {
    Debug.print(DBG_WARNING, "MQTT_IP not found or invalid, using default");
    storage.MQTT_IP = MQTT_SERVER_IP;
  }
  storage.MQTT_PORT             = nvs.getUInt("MQTT_PORT", MQTT_SERVER_PORT);
  storage.SSID                  = nvs.getString("SSID", "");
  storage.WIFI_PASS             = nvs.getString("WIFI_PASS", "");
  storage.WIFI_OnOff            = nvs.getBool("WIFI_OnOff",false);
  storage.MQTT_USER             = nvs.getString("MQTT_USER", "");
  storage.MQTT_PASS             = nvs.getString("MQTT_PASS", "");
  storage.MQTT_NAME             = nvs.getString("MQTT_NAME", "");
  storage.MQTTLOGIN_OnOff       = nvs.getBool("MQTTLOGIN_OnOff",false);
  storage.BUS_A_B               = nvs.getBool("BUS_A_B",false);
  storage.Ph_RegulationOnOff    = nvs.getBool("Ph_RegOnOff",true);
  storage.Orp_RegulationOnOff   = nvs.getBool("Orp_RegOnOff",false);  
  storage.AutoMode              = nvs.getBool("AutoMode",true);
  storage.SolarLocExt           = nvs.getBool("SolarLocExt",false);
  storage.SolarOnline           = nvs.getBool("SolarOnline",false);
  storage.SolarMode             = nvs.getBool("SolarMode",true);
  storage.Salt_Chlor            = nvs.getBool("Salt_Chlor",true);
  storage.SaltMode              = nvs.getBool("SaltMode",true);
  storage.SaltPolarity          = nvs.getBool("SaltPolarity",false);
  storage.WinterMode            = nvs.getBool("WinterMode",false);
  storage.WaterHeat             = nvs.getBool("Heat",false);
  storage.HeatPumpMode          = nvs.getBool("HeatPumpMode",true);
  storage.ValveMode             = nvs.getBool("ValveMode",true);
  storage.ValveSwitch           = nvs.getBool("ValveSwitch",false);
  storage.WaterFillMode         = nvs.getBool("WaterFillMode",true);
  storage.FiltrationDuration    = nvs.getUChar("FiltrDuration",12);
  storage.FiltrationStart       = nvs.getUChar("FiltrStart",8);
  storage.FiltrationStop        = nvs.getUChar("FiltrStop",20);
  storage.FiltrationStartMin    = nvs.getUChar("FiltrStartMin",8);
  storage.FiltrationStopMax     = nvs.getUChar("FiltrStopMax",22);
  storage.SolarStartMin         = nvs.getUChar("SolarStartMin",11);
  storage.SolarStopMax          = nvs.getUChar("SolarStopMax",18);
  storage.DelayPIDs             = nvs.getUChar("DelayPIDs",0);
  storage.PhPumpUpTimeLimit     = migratePoolUpLimitToMs(nvs.getULong("PhPumpUTL",900));
  storage.ChlPumpUpTimeLimit    = migratePoolUpLimitToMs(nvs.getULong("ChlPumpUTL",2500));
  storage.PublishPeriod         = nvs.getULong("PublishPeriod",30000);
  storage.PhPIDWindowSize       = nvs.getULong("PhPIDWSize",60000);
  storage.OrpPIDWindowSize      = nvs.getULong("OrpPIDWSize",60000);
  storage.PhPIDwindowStartTime  = nvs.getULong("PhPIDwStart",0);
  storage.OrpPIDwindowStartTime = nvs.getULong("OrpPIDwStart",0);
  storage.Ph_SetPoint           = nvs.getDouble("Ph_SetPoint",7.3);
  storage.Orp_SetPoint          = nvs.getDouble("Orp_SetPoint",750);
  storage.PSI_HighThreshold     = nvs.getDouble("PSI_High",1.8);
  storage.PSI_MedThreshold      = nvs.getDouble("PSI_Med",0.25);
  storage.FLOW_Pulse            = nvs.getDouble("FLOW_Pulse", 12.);
  storage.FLOW_HighThreshold    = nvs.getDouble("FLOW_High",90.);
  storage.FLOW_MedThreshold     = nvs.getDouble("FLOW_Med",50.);
  storage.FLOW2_Pulse           = nvs.getDouble("FLOW2_Pulse", 5880.);
  storage.FLOW2_HighThreshold   = nvs.getDouble("FLOW2_High",30.);
  storage.FLOW2_MedThreshold    = nvs.getDouble("FLOW2_Med",8.);
  storage.SaltDiff              = nvs.getDouble("SaltDiff", 30.);
  storage.WaterTempLowThreshold = nvs.getDouble("WaterTempLow",10.);
  storage.WaterTemp_SetPoint    = nvs.getDouble("WaterTempSet",27.);
  storage.pHCalibCoeffs0        = nvs.getDouble("pHCalibCoeffs0",4.3);
  storage.pHCalibCoeffs1        = nvs.getDouble("pHCalibCoeffs1",-2.63);
  storage.OrpCalibCoeffs0       = nvs.getDouble("OrpCalibCoeffs0",-1189.);
  storage.OrpCalibCoeffs1       = nvs.getDouble("OrpCalibCoeffs1",2564.);
  storage.PSICalibCoeffs0       = nvs.getDouble("PSICalibCoeffs0",1.11);
  storage.PSICalibCoeffs1       = nvs.getDouble("PSICalibCoeffs1",0.);
  storage.Ph_Kp                 = nvs.getDouble("Ph_Kp",2000000.);
  storage.Ph_Ki                 = nvs.getDouble("Ph_Ki",0.);
  storage.Ph_Kd                 = nvs.getDouble("Ph_Kd",0.);
  storage.Orp_Kp                = nvs.getDouble("Orp_Kp",2500.);
  storage.Orp_Ki                = nvs.getDouble("Orp_Ki",0.);
  storage.Orp_Kd                = nvs.getDouble("Orp_Kd",0.);
  storage.PhPIDOutput           = nvs.getDouble("PhPIDOutput",0.);
  storage.OrpPIDOutput          = nvs.getDouble("OrpPIDOutput",0.);
  storage.WaterSTemp            = nvs.getDouble("WaterSTemp",0.);
  storage.WaterITemp            = nvs.getDouble("WaterITemp",0.);
  storage.WaterBTemp            = nvs.getDouble("WaterBTemp",0.);
  storage.WaterWPTemp           = nvs.getDouble("WaterWPTemp",0.);
  storage.WaterWTTemp           = nvs.getDouble("WaterWTTemp",0.);
  storage.AirInTemp             = nvs.getDouble("AirInTemp",0.);
  storage.AirTemp               = nvs.getDouble("AirTemp",0.);
  storage.AirHum                = nvs.getDouble("AirHum",0.);
  storage.AirPress              = nvs.getDouble("AirPress",0.);
  storage.SolarTemp             = nvs.getDouble("SolarTemp",0.);
  storage.SolarVLTemp           = nvs.getDouble("SolarVLTemp",0.);
  storage.SolarRLTemp           = nvs.getDouble("SolarRLTemp",0.);
  storage.PhValue               = nvs.getDouble("PhValue",0.);
  storage.OrpValue              = nvs.getDouble("OrpValue",0.);
  storage.PSIValue              = nvs.getDouble("PSIValue",0.4);
  storage.FLOWValue             = nvs.getDouble("FLOWValue",40.);
  storage.FLOW2Value            = nvs.getDouble("FLOW2Value",8.);
  storage.AcidFill              = nvs.getDouble("AcidFill",100.);
  storage.ChlFill               = nvs.getDouble("ChlFill",100.);
  storage.pHTankVol             = nvs.getDouble("pHTankVol",20.);
  storage.ChlTankVol            = nvs.getDouble("ChlTankVol",20.);
  storage.pHPumpFR              = nvs.getDouble("pHPumpFR",1.5);
  storage.ChlPumpFR             = nvs.getDouble("ChlPumpFR",1.5);
  storage.WaterFillFR           = nvs.getDouble("WaterFillFR",15.0);
  storage.WaterFillAnCon        = nvs.getULong("WaterFillAnCon",0);
  storage.WaterFillUpTimeLimit  = migratePoolUpLimitToMs(nvs.getULong("WaterFillUTL",900000));
  storage.WaterFillDuration     = nvs.getULong("WaterFillDur",0);
  storage.SaltPumpRunTime       = nvs.getULong("SaltPumpRunTime",0);
  storage.SaltCurrentValue      = nvs.getDouble("SaltCurrentVal",0.0);
  storage.FilterCurrentValue    = nvs.getDouble("FilterCurrentVal",0.0);
  storage.HeatCurrentValue      = nvs.getDouble("HeatCurrentVal",0.0);
  storage.SaltCurrentCalibCoeffs0 = nvs.getDouble("SaltCurrCalib0",10.0);
  storage.SaltCurrentCalibCoeffs1 = nvs.getDouble("SaltCurrCalib1",-25.0);
  storage.FilterCurrentCalibCoeffs0 = nvs.getDouble("FiltCurrCalib0",10.0);
  storage.FilterCurrentCalibCoeffs1 = nvs.getDouble("FiltCurrCalib1",-25.0);
  storage.HeatCurrentCalibCoeffs0 = nvs.getDouble("HeatCurrCalib0",10.0);
  storage.HeatCurrentCalibCoeffs1 = nvs.getDouble("HeatCurrCalib1",-25.0);
  storage.SaltConcentration     = nvs.getDouble("SaltConc",0.0);
  storage.CellConstant          = nvs.getDouble("CellConstant",0.0);
  storage.SaltStatus            = nvs.getString("SaltStatus","Unknown");
  storage.SaltNeeded            = nvs.getDouble("SaltNeeded",0.0);
  storage.PoolVolume            = nvs.getDouble("PoolVolume",0.0);
  storage.Uptime                = nvs.getUInt("Uptime", 0U);
  storage.LastUptimeUpdate      = nvs.getUInt("LastUpdt", 0U);
  storage.ResetReason           = nvs.getUChar("ResetReason", 0U);
  storage.ResetTimestamp        = nvs.getString("ResetTimestamp", "");
  nvs.getBytes("address_A_0", storage.address_A_0, 8);
  nvs.getBytes("address_A_1", storage.address_A_1, 8);
  nvs.getBytes("address_A_2", storage.address_A_2, 8);
  nvs.getBytes("address_A_3", storage.address_A_3, 8);
  nvs.getBytes("address_A_4", storage.address_A_4, 8);
  nvs.getBytes("address_W_0", storage.address_W_0, 8);
  nvs.getBytes("address_W_1", storage.address_W_1, 8);
  nvs.getBytes("address_W_2", storage.address_W_2, 8);
  nvs.getBytes("address_W_3", storage.address_W_3, 8);
  nvs.getBytes("address_W_4", storage.address_W_4, 8);
  nvs.getBytes("Array_A", storage.Array_A, 5);
  nvs.getBytes("Array_W", storage.Array_W, 5);

  poolSolarBridgeLoadFromNvs(nvs);

  nvs.end();

  Debug.print(DBG_INFO,"%d",storage.ConfigVersion);
  Debug.print(DBG_INFO, "Loaded MQTT_IP: %d.%d.%d.%d, MQTT_PORT: %u", storage.MQTT_IP[0], storage.MQTT_IP[1], storage.MQTT_IP[2], storage.MQTT_IP[3], storage.MQTT_PORT);
  Debug.print(DBG_INFO,"%d",storage.MQTT_PORT);
  Debug.print(DBG_INFO,"%s,%s,%s,%s,%s",storage.SSID.c_str(),storage.WIFI_PASS.c_str(),storage.MQTT_USER.c_str(),storage.MQTT_PASS.c_str(),storage.MQTT_NAME.c_str());
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",storage.WIFI_OnOff,storage.MQTTLOGIN_OnOff,storage.BUS_A_B,storage.Ph_RegulationOnOff,storage.Orp_RegulationOnOff,storage.AutoMode,storage.SolarLocExt,storage.SolarOnline,storage.SolarMode,storage.Salt_Chlor,storage.SaltMode,storage.SaltPolarity,storage.WinterMode,storage.WaterHeat,storage.ValveMode,storage.CleanMode,storage.ValveSwitch, storage.WaterFillMode);
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d, %d, %d",storage.FiltrationDuration,storage.FiltrationStart,storage.FiltrationStop,
              storage.FiltrationStartMin,storage.FiltrationStopMax,storage.SolarStartMin,storage.SolarStopMax,storage.DelayPIDs);
  Debug.print(DBG_INFO, "Address_A: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", storage.address_A_0, storage.address_A_1, storage.address_A_2, storage.address_A_3, storage.address_A_4);
  for (int i = 0; i < MAX_ADDRESSES; i++) {
      Debug.print(DBG_INFO, "Array_A[%d]: %d (0x%02X)", i, storage.Array_A[i], storage.Array_A[i]);
  }
  Debug.print(DBG_INFO, "Address_W: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", storage.address_W_0, storage.address_W_1, storage.address_W_2, storage.address_W_3, storage.address_W_4);
  for (int i = 0; i < MAX_ADDRESSES; i++) {
      Debug.print(DBG_INFO, "Array_W[%d]: %d (0x%02X)", i, storage.Array_W[i], storage.Array_W[i]);
  }
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d, %d",storage.PhPumpUpTimeLimit,storage.ChlPumpUpTimeLimit,storage.WaterFillUpTimeLimit,storage.WaterFillDuration,storage.WaterFillDuration,storage.PublishPeriod);
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d",storage.PhPIDWindowSize,storage.OrpPIDWindowSize,storage.PhPIDwindowStartTime,storage.OrpPIDwindowStartTime,storage.WaterFillAnCon);
  Debug.print(DBG_INFO,"%3.1f, %4.0f, %3.1f, %3.1f, %3.1f, %3.0f, %3.0f, %3.1f, %3.0f, %3.0f, %3.1f, %3.1f, %8.6f, %9.6f, %11.6f, %11.6f, %3.1f, %3.1f, %3.0f",
              storage.Ph_SetPoint,storage.Orp_SetPoint,storage.PSI_HighThreshold,storage.FLOW_Pulse,storage.FLOW2_Pulse,storage.FLOW_HighThreshold,storage.FLOW2_HighThreshold,
              storage.PSI_MedThreshold,storage.FLOW_MedThreshold,storage.FLOW2_MedThreshold,storage.WaterTempLowThreshold,storage.WaterTemp_SetPoint,
              storage.pHCalibCoeffs0,storage.pHCalibCoeffs1,storage.OrpCalibCoeffs0,storage.OrpCalibCoeffs1,storage.SaltDiff,
              storage.PSICalibCoeffs0,storage.PSICalibCoeffs1);
  Debug.print(DBG_INFO,"%8.0f, %3.0f, %3.0f, %6.0f, %3.0f, %3.0f, %7.0f, %7.0f, %4.2f, %4.2f, %4.0f, %3.0f, %3.0f",
              storage.Ph_Kp,storage.Ph_Ki,storage.Ph_Kd,storage.Orp_Kp,storage.Orp_Ki,storage.Orp_Kd,storage.PhPIDOutput,storage.OrpPIDOutput,
              storage.PhValue,storage.OrpValue,storage.PSIValue,storage.FLOWValue,storage.FLOW2Value);
  Debug.print(DBG_INFO,"%5.1f, %5.1f, %6.2f, %7.2f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f",
              storage.WaterSTemp,storage.WaterITemp,storage.WaterBTemp,storage.WaterWPTemp,storage.WaterWTTemp,storage.AirInTemp,storage.AirTemp,storage.AirHum,storage.AirPress,storage.SolarTemp,storage.SolarVLTemp,storage.SolarRLTemp);
  Debug.print(DBG_INFO,"%3.0f, %3.0f, %3.0f, %3.0f, %3.1f, %3.1f, %3.1f",storage.AcidFill,storage.ChlFill,storage.pHTankVol,storage.ChlTankVol,
              storage.pHPumpFR,storage.ChlPumpFR,storage.WaterFillFR);
  Debug.print(DBG_INFO,"SaltCurrent: %4.2f, FilterCurrent: %4.2f, HeatCurrent: %4.2f", storage.SaltCurrentValue, storage.FilterCurrentValue, storage.HeatCurrentValue);
  Debug.print(DBG_INFO,"SaltCalib: %4.2f, %4.2f, FilterCalib: %4.2f, %4.2f, HeatCalib: %4.2f, %4.2f",
              storage.SaltCurrentCalibCoeffs0, storage.SaltCurrentCalibCoeffs1,
              storage.FilterCurrentCalibCoeffs0, storage.FilterCurrentCalibCoeffs1,
              storage.HeatCurrentCalibCoeffs0, storage.HeatCurrentCalibCoeffs1);
  Debug.print(DBG_INFO,"SaltConcentration: %4.2f, CellConstant: %4.2f", storage.SaltConcentration, storage.CellConstant);
  Debug.print(DBG_INFO,"SaltStatus: %s, SaltNeeded: %.2f", storage.SaltStatus.c_str(), (double)storage.SaltNeeded);
  Debug.print(DBG_INFO,"PoolVolume: %4.2f", storage.PoolVolume);
  Debug.print(DBG_INFO,"System: Uptime: %s, LastUptimeUpdate: %u",formatUptime(storage.Uptime).c_str(),storage.LastUptimeUpdate);
  Debug.print(DBG_INFO,"Reset: Reason: %s, Timestamp: %s", resetReasonToString(storage.ResetReason), storage.ResetTimestamp.c_str());
  Debug.print(DBG_INFO,"PhPIDwindowStartTime: %u, OrpPIDwindowStartTime: %u", storage.PhPIDwindowStartTime, storage.OrpPIDwindowStartTime);

  return (storage.ConfigVersion == CONFIG_VERSION);
}

bool saveConfig() {
  PrefsGuard g;
  if (!nvs.begin("PoolMaster", false)) {
      Debug.print(DBG_ERROR, "Failed to open NVS for writing");
      return false;
  }
  size_t i = nvs.putUChar("ConfigVersion",storage.ConfigVersion);
  i += nvs.putUChar("MatterVersion",storage.MatterVersion);
  uint8_t ipBytes[4] = {storage.MQTT_IP[0], storage.MQTT_IP[1], storage.MQTT_IP[2], storage.MQTT_IP[3]};
  i += nvs.putBytes("MQTT_IP", ipBytes, 4);
  i += nvs.putUInt("MQTT_PORT", storage.MQTT_PORT);
  i += nvs.putString("SSID",storage.SSID);
  i += nvs.putString("WIFI_PASS",storage.WIFI_PASS);
  i += nvs.putString("MQTT_USER",storage.MQTT_USER);
  i += nvs.putString("MQTT_PASS",storage.MQTT_PASS);
  i += nvs.putString("MQTT_NAME",storage.MQTT_NAME);
  i += nvs.putBool("WIFI_OnOff",storage.WIFI_OnOff);
  i += nvs.putBool("MQTTLOGIN_OnOff",storage.MQTTLOGIN_OnOff);
  i += nvs.putBool("BUS_A_B",storage.BUS_A_B);
  i += nvs.putBool("Ph_RegOnOff",storage.Ph_RegulationOnOff);
  i += nvs.putBool("Orp_RegOnOff",storage.Orp_RegulationOnOff);  
  i += nvs.putBool("AutoMode",storage.AutoMode);
  i += nvs.putBool("SolarLocExt",storage.SolarLocExt);
  i += nvs.putBool("SolarOnline",storage.SolarOnline);
  i += nvs.putBool("SolarMode",storage.SolarMode);
  i += nvs.putBool("Salt_Chlor",storage.Salt_Chlor);
  i += nvs.putBool("SaltMode",storage.SaltMode);
  i += nvs.putBool("SaltPolarity",storage.SaltPolarity);
  i += nvs.putBool("WinterMode",storage.WinterMode);
  i += nvs.putBool("Heat",storage.WaterHeat);
  i += nvs.putBool("HeatPumpMode",storage.HeatPumpMode);
  i += nvs.putBool("ValveMode",storage.ValveMode);
  i += nvs.putBool("CleanMode",storage.CleanMode);
  i += nvs.putBool("ValveSwitch",storage.ValveSwitch);
  i += nvs.putBool("WaterFillMode",storage.WaterFillMode);
  i += nvs.putUChar("FiltrDuration",storage.FiltrationDuration);
  i += nvs.putUChar("FiltrStart",storage.FiltrationStart);
  i += nvs.putUChar("FiltrStop",storage.FiltrationStop);
  i += nvs.putUChar("FiltrStartMin",storage.FiltrationStartMin);
  i += nvs.putUChar("FiltrStopMax",storage.FiltrationStopMax);
  i += nvs.putUChar("SolarStartMin",storage.SolarStartMin);
  i += nvs.putUChar("SolarStopMax",storage.SolarStopMax);
  i += nvs.putUChar("DelayPIDs",storage.DelayPIDs);
  i += nvs.putULong("PhPumpUTL",storage.PhPumpUpTimeLimit);
  i += nvs.putULong("ChlPumpUTL",storage.ChlPumpUpTimeLimit);
  i += nvs.putULong("PublishPeriod",storage.PublishPeriod);
  i += nvs.putULong("PhPIDWSize",storage.PhPIDWindowSize);
  i += nvs.putULong("OrpPIDWSize",storage.OrpPIDWindowSize);
  i += nvs.putULong("PhPIDwStart",storage.PhPIDwindowStartTime);
  i += nvs.putULong("OrpPIDwStart",storage.OrpPIDwindowStartTime);
  i += nvs.putDouble("Ph_SetPoint",storage.Ph_SetPoint);
  i += nvs.putDouble("Orp_SetPoint",storage.Orp_SetPoint);
  i += nvs.putDouble("PSI_High",storage.PSI_HighThreshold);
  i += nvs.putDouble("PSI_Med",storage.PSI_MedThreshold);
  i += nvs.putDouble("FLOW_Pulse",storage.FLOW_Pulse);
  i += nvs.putDouble("FLOW_High",storage.FLOW_HighThreshold);
  i += nvs.putDouble("FLOW_Med",storage.FLOW_MedThreshold);
  i += nvs.putDouble("FLOW2_Pulse",storage.FLOW2_Pulse);
  i += nvs.putDouble("FLOW2_High",storage.FLOW2_HighThreshold);
  i += nvs.putDouble("FLOW2_Med",storage.FLOW2_MedThreshold);
  i += nvs.putDouble("SaltDiff",storage.SaltDiff);
  i += nvs.putDouble("WaterTempLow",storage.WaterTempLowThreshold);
  i += nvs.putDouble("WaterTempSet",storage.WaterTemp_SetPoint);
  i += nvs.putDouble("pHCalibCoeffs0",storage.pHCalibCoeffs0);
  i += nvs.putDouble("pHCalibCoeffs1",storage.pHCalibCoeffs1);
  i += nvs.putDouble("OrpCalibCoeffs0",storage.OrpCalibCoeffs0);
  i += nvs.putDouble("OrpCalibCoeffs1",storage.OrpCalibCoeffs1);
  i += nvs.putDouble("PSICalibCoeffs0",storage.PSICalibCoeffs0);
  i += nvs.putDouble("PSICalibCoeffs1",storage.PSICalibCoeffs1);
  i += nvs.putDouble("Ph_Kp",storage.Ph_Kp);
  i += nvs.putDouble("Ph_Ki",storage.Ph_Ki);
  i += nvs.putDouble("Ph_Kd",storage.Ph_Kd);
  i += nvs.putDouble("Orp_Kp",storage.Orp_Kp);
  i += nvs.putDouble("Orp_Ki",storage.Orp_Ki);
  i += nvs.putDouble("Orp_Kd",storage.Orp_Kd);
  i += nvs.putDouble("PhPIDOutput",storage.PhPIDOutput);
  i += nvs.putDouble("OrpPIDOutput",storage.OrpPIDOutput);
  i += nvs.putDouble("WaterSTemp",storage.WaterSTemp);
  i += nvs.putDouble("WaterITemp",storage.WaterITemp);
  i += nvs.putDouble("WaterBTemp",storage.WaterBTemp);
  i += nvs.putDouble("WaterWPTemp",storage.WaterWPTemp);
  i += nvs.putDouble("WaterWTTemp",storage.WaterWTTemp);
  i += nvs.putDouble("AirInTemp",storage.AirInTemp);
  i += nvs.putDouble("AirTemp",storage.AirTemp);
  i += nvs.putDouble("AirHum",storage.AirHum);
  i += nvs.putDouble("AirPress",storage.AirPress);
  i += nvs.putDouble("SolarTemp",storage.SolarTemp);
  i += nvs.putDouble("SolarVLTemp",storage.SolarVLTemp);
  i += nvs.putDouble("SolarRLTemp",storage.SolarRLTemp);
  i += nvs.putDouble("PhValue",storage.PhValue);
  i += nvs.putDouble("OrpValue",storage.OrpValue);
  i += nvs.putDouble("PSIValue",storage.PSIValue);
  i += nvs.putDouble("FLOWValue",storage.FLOWValue);
  i += nvs.putDouble("FLOW2Value",storage.FLOW2Value);
  i += nvs.putDouble("AcidFill",storage.AcidFill);
  i += nvs.putDouble("ChlFill",storage.ChlFill);
  i += nvs.putDouble("pHTankVol",storage.pHTankVol);
  i += nvs.putDouble("ChlTankVol",storage.ChlTankVol);
  i += nvs.putDouble("pHPumpFR",storage.pHPumpFR);
  i += nvs.putDouble("ChlPumpFR",storage.ChlPumpFR);
  i += nvs.putDouble("WaterFillFR",storage.WaterFillFR);
  i += nvs.putULong("WaterFillAnCon",storage.WaterFillAnCon);
  i += nvs.putULong("WaterFillUTL",storage.WaterFillUpTimeLimit);
  i += nvs.putULong("WaterFillDur",storage.WaterFillDuration);
  i += nvs.putULong("SaltPumpRunTime",storage.SaltPumpRunTime);
  i += nvs.putDouble("SaltCurrentVal", storage.SaltCurrentValue);
  i += nvs.putDouble("FiltCurrentVal", storage.FilterCurrentValue);
  i += nvs.putDouble("HeatCurrentVal", storage.HeatCurrentValue);
  i += nvs.putDouble("SaltCurrCalib0", storage.SaltCurrentCalibCoeffs0);
  i += nvs.putDouble("SaltCurrCalib1", storage.SaltCurrentCalibCoeffs1);
  i += nvs.putDouble("FiltCurrCalib0", storage.FilterCurrentCalibCoeffs0);
  i += nvs.putDouble("FiltCurrCalib1", storage.FilterCurrentCalibCoeffs1);
  i += nvs.putDouble("HeatCurrCalib0", storage.HeatCurrentCalibCoeffs0);
  i += nvs.putDouble("HeatCurrCalib1", storage.HeatCurrentCalibCoeffs1);
  i += nvs.putDouble("SaltConc", storage.SaltConcentration);
  i += nvs.putDouble("CellConstant", storage.CellConstant);
  i += nvs.putString("SaltStatus", storage.SaltStatus);
  i += nvs.putDouble("SaltNeeded", storage.SaltNeeded);
  i += nvs.putDouble("PoolVolume", storage.PoolVolume);
  i += nvs.putUInt("Uptime", storage.Uptime);
  i += nvs.putUInt("LastUpdt", storage.LastUptimeUpdate);
  i += nvs.putUChar("ResetReason", storage.ResetReason);
  i += nvs.putString("ResetTimestamp", storage.ResetTimestamp);
  i += nvs.putBytes("address_A_0",storage.address_A_0, 8);
  i += nvs.putBytes("address_A_1",storage.address_A_1, 8);
  i += nvs.putBytes("address_A_2",storage.address_A_2, 8);
  i += nvs.putBytes("address_A_3",storage.address_A_3, 8);
  i += nvs.putBytes("address_A_4",storage.address_A_4, 8);
  i += nvs.putBytes("address_W_0",storage.address_W_0, 8);
  i += nvs.putBytes("address_W_1",storage.address_W_1, 8);
  i += nvs.putBytes("address_W_2",storage.address_W_2, 8);
  i += nvs.putBytes("address_W_3",storage.address_W_3, 8);
  i += nvs.putBytes("address_W_4",storage.address_W_4, 8);
  i += nvs.putBytes("Array_A",storage.Array_A, 5);
  i += nvs.putBytes("Array_W",storage.Array_W, 5);

  nvs.end();

  Debug.print(DBG_INFO,"Bytes saved: %d / %d\n",i,sizeof(storage));
  return (i == sizeof(storage)) ;

}

// functions to save any type of parameter (7 overloads with same name but different arguments)

// For uint8_t (z. B. ConfigVersion, FiltrationDuration)
bool saveParam(const char* key, uint8_t val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putUChar(key, val);
    ok = (i == sizeof(uint8_t));
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// For bool (z. B. WIFI_OnOff, MQTTLOGIN_OnOff)
bool saveParam(const char* key, bool val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putBool(key, val);
    ok = (i == sizeof(bool));
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// For unsigned long / uint32_t (z. B. PhPumpUpTimeLimit, MQTT_PORT)
bool saveParam(const char* key, unsigned long val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putULong(key, val);
    ok = (i == sizeof(unsigned long));
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// Für String (z. B. SSID, MQTT_NAME)
bool saveParam(const char* key, String val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putString(key, val);
    ok = (i == val.length());
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// Für uint8_t-Arrays (z. B. MQTT_IP, address_A_0)
bool saveParam(const char* key, const uint8_t* val, size_t size) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putBytes(key, val, size);
    ok = (i == size);
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// Für double (z. B. Ph_SetPoint, Ph_Kp)
bool saveParam(const char* key, double val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putDouble(key, val);
    ok = (i == sizeof(double));
    nvs.end();
  }
  prefsUnlock();
  return ok;
}

// Für Float (z. B. SaltConcentration, CellConstant)
bool saveParam(const char* key, float val) {
  prefsLock();
  bool ok = false;
  if (nvs.begin("PoolMaster", false)) {
    size_t i = nvs.putFloat(key, val);
    ok = (i == sizeof(float));
    nvs.end();
  }
  prefsUnlock();
  return ok;
}


//Compute free RAM
//useful to check if it does not shrink over time
int freeRam () {
  int v = xPortGetFreeHeapSize();
  return v;
}

// Get current free stack 
unsigned stack_hwm(){
  return uxTaskGetStackHighWaterMark(nullptr);
}

// Monitor free stack (display smallest value)
void stack_mon(UBaseType_t &hwm)
{
  UBaseType_t temp = uxTaskGetStackHighWaterMark(nullptr);
  if(!hwm || temp < hwm)
  {
    hwm = temp;
    Debug.print(DBG_DEBUG,"[stack_mon] %s: %d bytes",pcTaskGetName(NULL), hwm);
  }  
}


// Get exclusive access of I2C bus
static uint32_t lockFailCount = 0;
bool lockI2C() {
    if (mutex == NULL) {
        Debug.print(DBG_ERROR, "[I2C] Mutex is NULL - Cannot lock I2C!");
        return false;
    }
    const uint32_t timeout = 100;  // 100 ms timeout
    if (xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
        mutexOwner = xTaskGetCurrentTaskHandle();
        #ifdef DEBUG_I2C_LOCK
        Debug.print(DBG_DEBUG, "[I2C] Mutex locked by %s", pcTaskGetName(NULL));
        #endif
        return true;
    }
    lockFailCount++;
    Debug.print(DBG_WARNING, "[I2C] Failed to acquire recursive mutex after %d ms! (Fail count: %u)", timeout, lockFailCount);
    return false;
}

// Release I2C bus access
void unlockI2C() {
  if (mutex == NULL) {
    Debug.print(DBG_ERROR, "[I2C] Mutex is NULL - Cannot unlock I2C!");
    return;
  }
  if (mutexOwner != xTaskGetCurrentTaskHandle()) {
    Debug.print(DBG_ERROR, "[I2C] Unlock attempted by non-owner: %s!", pcTaskGetName(NULL));
    return;
  }
  mutexOwner = NULL;
  #ifdef DEBUG_I2C_LOCK
  Debug.print(DBG_DEBUG, "[I2C] Mutex unlocked by %s", pcTaskGetName(NULL));
  #endif
  xSemaphoreGiveRecursive(mutex);
}

void clearStatusLEDs(void) {
  PCF8574Manager& pcfManager = PCF8574Manager::getInstance();
  pcfManager.queueFullStateUpdate(PCF8574_ADR, 0xFF, NULL); // active-low: all high = off
}

// Connect to the BME280 sensor
void bme280Init() {
  if (!bme.begin(0x76, &Wire)) {
    Debug.print(DBG_ERROR, "[BME280] Could not find a valid BME280 sensor, check wiring!");
  } else {
    Debug.print(DBG_INFO, "[BME280] BME280 sensor initialized.");
  }
}

// Init RTC DS3231
  void RTCInit() {
    if (!rtc.begin(&Wire)) {
      Debug.print(DBG_ERROR, "[RTC] Failed to detect RTC module");
    } else {
      Debug.print(DBG_INFO, "[RTC] RTC module detected");
    }
  
    if (rtc.lostPower()) {
      Debug.print(DBG_WARNING, "[RTC] RTC lost power, lets set the time!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

// Match MatterBridge `kMatterPlausibleMinUnixSec` / WebUI — wall clock must not look like Y2K default.
static constexpr time_t kMinEpochWallClockUtc = 1577836800; // 2020-01-01 00:00 UTC

void poolEnsureEuropeBerlinTz(void)
{
  static bool configured = false;
  if (configured) {
    return;
  }
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
  configured = true;
}

void poolApplyEspSystemTimeFromLocalTm(struct tm *tmLocal)
{
  if (tmLocal == nullptr) {
    return;
  }
  poolEnsureEuropeBerlinTz();
  struct tm tmWork = *tmLocal;
  if (tmWork.tm_isdst < 0 || tmWork.tm_isdst > 1) {
    tmWork.tm_isdst = -1;
  }
  time_t utc = mktime(&tmWork);
  if (utc == (time_t) -1) {
    Debug.print(DBG_WARNING, "[Time] mktime failed — ESP system clock unchanged");
    return;
  }
  if (utc < kMinEpochWallClockUtc) {
    Debug.print(DBG_WARNING, "[Time] epoch %ld before 2020 — skip settimeofday", (long) utc);
    return;
  }
  struct timeval tv = {};
  tv.tv_sec = utc;
  tv.tv_usec = 0;
  if (settimeofday(&tv, nullptr) != 0) {
    Debug.print(DBG_WARNING, "[Time] settimeofday failed: errno=%d", errno);
    return;
  }
  Debug.print(DBG_INFO, "[Time] settimeofday OK (UTC epoch %ld) — libc/Matter gettimeofday aligned", (long) utc);
}

// Set time parameters, including DST
void StartTime()
{
  static bool ntpConfigured = false;
  if (storage.WIFI_OnOff && wifiIsConnected()) {
    if (!ntpConfigured) {
      Debug.print(DBG_INFO, "[NTP] Configuring time with NTP servers: 0.pool.ntp.org, 1.pool.ntp.org, 2.pool.ntp.org (CET/CEST)");
      poolEnsureEuropeBerlinTz();
      configTime(0, 0,"0.pool.ntp.org","1.pool.ntp.org","2.pool.ntp.org"); // 3 possible NTP servers
      ntpConfigured = true;
      Debug.print(DBG_INFO, "[NTP] NTP configuration completed");
    } else {
      Debug.print(DBG_INFO, "[NTP] NTP already configured, skipping configTime()");
    }
    for (int i = 0; i < 10; i++) {
      time_t now = time(nullptr);
      if (now > 1609459200) { // check if time is valid (after 2021-01-01)
        struct tm timeinfo;
        getLocalTime(&timeinfo, 0);
        Debug.print(DBG_INFO, "[NTP] Sync completed: %04d-%02d-%02d %02d:%02d:%02d, DST: %d",
          timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
          timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_isdst);
        char timeString[20];
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Debug.print(DBG_INFO, "[NTP] NTP sync completed, time: %ld (%s)", now, timeString);
        return;
      }
      Debug.print(DBG_INFO, "[NTP] Waiting for sync, attempt %d...", i + 1);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    Debug.print(DBG_WARNING, "[NTP] NTP sync not completed after 10 retries");
  } else {
    Debug.print(DBG_INFO, "[NTP] WiFi off or not connected, skipping NTP");
  }
}

void readLocalTime()
{
  poolEnsureEuropeBerlinTz();
  bool timeSynced = false;
  struct tm localTimeInfo;
  memset(&localTimeInfo, 0, sizeof(localTimeInfo));

  if (storage.WIFI_OnOff && wifiIsConnected()) {
    if (getLocalTime(&localTimeInfo, 5000U)) {
      Debug.print(DBG_INFO, "[NTP] Time from NTP: %04d-%02d-%02d %02d:%02d:%02d, DST: %d",
        localTimeInfo.tm_year + 1900, localTimeInfo.tm_mon + 1, localTimeInfo.tm_mday,
        localTimeInfo.tm_hour, localTimeInfo.tm_min, localTimeInfo.tm_sec, localTimeInfo.tm_isdst);
      timeSynced = true;
    } else {
      Debug.print(DBG_WARNING, "[NTP] Failed to obtain time");
    }
  }

  if (rtc.begin()) {
    RTCfound = true;
    Debug.print(DBG_INFO, "[RTC] RTC module detected");
    DateTime now = rtc.now();
    if (timeSynced) {
      // synchronize RTC with NTP time
      DateTime ntpTime(localTimeInfo.tm_year + 1900, localTimeInfo.tm_mon + 1, localTimeInfo.tm_mday,
                       localTimeInfo.tm_hour, localTimeInfo.tm_min, localTimeInfo.tm_sec);
      time_t ntp_time = ntpTime.unixtime();
      time_t rtc_time = now.unixtime();
      double diff = difftime(ntp_time, rtc_time);
      Debug.print(DBG_INFO, "[RTC] NTP time: %ld, RTC time: %ld, diff: %.1f", ntp_time, rtc_time, diff);
      if (fabs(diff) > 5.0) {
        rtc.adjust(ntpTime);
        Debug.print(DBG_INFO, "[RTC] RTC time adjusted to NTP time");
        DateTime adjustedTime = rtc.now();
        char adjustedTimeString[20];
        sprintf(adjustedTimeString, "%04d-%02d-%02d %02d:%02d:%02d",
                adjustedTime.year(), adjustedTime.month(), adjustedTime.day(),
                adjustedTime.hour(), adjustedTime.minute(), adjustedTime.second());
        Debug.print(DBG_INFO, "[RTC] New RTC time: %s", adjustedTimeString);
      } else {
        Debug.print(DBG_INFO, "[RTC] RTC time and NTP time are within 5 seconds, no adjustment necessary");
      }
    } else {
      Debug.print(DBG_INFO, "[RTC] No NTP time available, using RTC time");
      localTimeInfo.tm_year = now.year() - 1900;
      localTimeInfo.tm_mon = now.month() - 1;
      localTimeInfo.tm_mday = now.day();
      localTimeInfo.tm_hour = now.hour();
      localTimeInfo.tm_min = now.minute();
      localTimeInfo.tm_sec = now.second();
      timeSynced = true;
    }
  } else {
    Debug.print(DBG_ERROR, "[RTC] RTC module not detected");
    RTCfound = false;
    if (!timeSynced) {
      Debug.print(DBG_WARNING, "[NTP] Failed to obtain time from NTP, no RTC available");
      return;
    }
  }

  if (timeSynced) {
    // Arduino TimeLib alone does not update libc — Matter/CHIP reads gettimeofday().
    poolApplyEspSystemTimeFromLocalTm(&localTimeInfo);
    // Setze Time-Bibliothek mit lokaler Zeit (inkl. DST) direkt aus localTimeInfo
    setTime(localTimeInfo.tm_hour, localTimeInfo.tm_min, localTimeInfo.tm_sec,
            localTimeInfo.tm_mday, localTimeInfo.tm_mon + 1, localTimeInfo.tm_year + 1900);
    Debug.print(DBG_INFO, "[TimeLib] Set TimeLib to: %04d-%02d-%02d %02d:%02d:%02d",
      year(), month(), day(), hour(), minute(), second());
    timeinfo = localTimeInfo; // Für Anzeigezeit
  } else {
    Debug.print(DBG_ERROR, "[Time] Failed to synchronize time");
  }
}

// Notify PublishSettings task 
void PublishSettings()
{
  if (pubSetTaskHandle != nullptr) {
    xTaskNotifyGive(pubSetTaskHandle);
  }
}

// Notify PublishMeasures task
void PublishMeasures()
{
  if (pubMeasTaskHandle != nullptr) {
    xTaskNotifyGive(pubMeasTaskHandle);
  }
}

void scanI2CBus() {
  Debug.print(DBG_INFO, "Scanne I2C-Bus nach Geräten...");
  int devicesFound = 0;
  uint8_t foundAddresses[127] = {0}; // Array zur Verfolgung gefundener Adressen
  bool alreadyFound;

  for (uint8_t address = 1; address < 127; address++) {
      alreadyFound = false;
      for (int i = 0; i < devicesFound; i++) {
          if (foundAddresses[i] == address) {
              alreadyFound = true;
              break;
          }
      }

      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission();

      if (error == 0) {
          if (alreadyFound) {
              Debug.print(DBG_WARNING, "Doppelte Adresse gefunden bei 0x%02X - wird nicht erneut gezählt", address);
          } else {
              Debug.print(DBG_INFO, "I2C-Gerät gefunden bei Adresse 0x%02X", address);
              foundAddresses[devicesFound] = address;
              devicesFound++;
          }
      } else if (error == 4) {
          Debug.print(DBG_WARNING, "Fehler auf dem I2C-Bus bei Adresse 0x%02X (error %d)", address, error);
      }
  }

  Debug.print(DBG_INFO, "Anzahl gefundener Geräte: %d", devicesFound);
  if (devicesFound == 0) {
      Debug.print(DBG_ERROR, "Keine I2C-Geräte gefunden. Bitte Verkabelung prüfen!");
  } else {
      Debug.print(DBG_INFO, "Gefundene eindeutige Adressen:");
      for (int i = 0; i < devicesFound; i++) {
          Debug.print(DBG_INFO, " - 0x%02X", foundAddresses[i]);
      }
  }
}

void checkI2CBus() {
  Wire.beginTransmission(PCF8574_I_ADR); // Test the I2C bus to the PCF8574_I
  if (Wire.endTransmission() != 0) {
      Debug.print(DBG_ERROR, "I2C-Bus-Fehler bei Adresse 0x%02X!", PCF8574_I_ADR);
  } else {
      Debug.print(DBG_INFO, "I2C-Bus zu PCF8574_I OK");
  }
}

//board info
void info(){
  esp_chip_info_t out_info;
  esp_chip_info(&out_info);
  Debug.print(DBG_INFO,"CPU frequency       : %dMHz",ESP.getCpuFreqMHz());
  Debug.print(DBG_INFO,"CPU Cores           : %d",out_info.cores);
  Debug.print(DBG_INFO,"Flash size          : %dMB",ESP.getFlashChipSize()/1000000);
  Debug.print(DBG_INFO,"Total RAM (heap)    : %d bytes",ESP.getHeapSize());
  Debug.print(DBG_INFO,"Free RAM (heap)     : %d bytes",ESP.getFreeHeap());
  Debug.print(DBG_INFO,"Min heap            : %d bytes",esp_get_free_heap_size());
  Debug.print(DBG_INFO,"tskIDLE_PRIORITY    : %d",tskIDLE_PRIORITY);
  Debug.print(DBG_INFO,"confixMAX_PRIORITIES: %d",configMAX_PRIORITIES);
  Debug.print(DBG_INFO,"configTICK_RATE_HZ  : %d",configTICK_RATE_HZ);

  Debug.print(DBG_INFO,"Total PSRAM         : %d", ESP.getPsramSize());
  Debug.print(DBG_INFO,"Free PSRAM          : %d", ESP.getFreePsram());
  Debug.print(DBG_INFO,"Free entries in nvs : %d", nvs.freeEntries());
  nvs.end(); // end preferences library usage
}


// Pseudo loop, which deletes loopTask of the Arduino framework
void loop()
{
  delay(1000);
  vTaskDelete(nullptr);
}