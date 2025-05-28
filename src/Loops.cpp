#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"
#include "PCF8574Manager.h"

extern Preferences nvs;

// Setup oneWire instances to communicate with temperature sensors (one bus per sensor)
static OneWire oneWire_W(ONE_WIRE_BUS_W);
static OneWire oneWire_A(ONE_WIRE_BUS_A);
// Pass our oneWire reference to Dallas Temperature library instance
static DallasTemperature sensors_W(&oneWire_W);
static DallasTemperature sensors_A(&oneWire_A);

// global variable for numbers of connected sensors
uint8_t numSensors_W;
uint8_t numSensors_A;

// Plausibilitätsgrenzen für Temperaturen
constexpr float MIN_VALID_TEMP = -40.0;  // Minimale plausible Temperatur
constexpr float MAX_VALID_TEMP = 100.0;  // Maximale plausible Temperatur
constexpr uint32_t TEMP_MEASURE_TIMEOUT_MS = 100; // Timeout für 9-Bit-Messung
constexpr float ERROR_TEMP_VALUE = -127.0; // Fehlerwert für ungültige Temperaturen

// Plausibilitätsgrenzen für Strommessung (ACS712 20A)
constexpr float MIN_VALID_CURRENT = -20.0; // Minimale plausible Stromstärke (A)
constexpr float MAX_VALID_CURRENT = 20.0;  // Maximale plausible Stromstärke (A)

// DS18B20 SENSOR-Mapping to map the sensoradress with the Tempname
const char* NV_STORAGE_MAPPING_A[] = {"SolarTemp", "SolarVLTemp", "SolarRLTemp", "AirInTemp", "AirTemp"}; // Mapping of A-BUS-Sensors to NVS
const char* NV_STORAGE_MAPPING_W[] = {"WaterSTemp", "WaterITemp", "WaterBTemp", "WaterWPTemp", "WaterWTTemp"}; // Mapping of W-BUS-Sensors to NVS

// Setup an ADS1115 instance for analog measurements
static ADS1115Scanner adc_int(INT_ADS1115_ADDR);
#ifdef EXT_ADS1115
static ADS1115Scanner adc_ph(PH_ADS1115_ADDR);
static ADS1115Scanner adc_orp(ORP_ADS1115_ADDR);
#endif

static float ph_sensor_value;     // pH sensor current value
static float orp_sensor_value;    // ORP sensor current value
static float psi_sensor_value;    // PSI sensor current value
static float salt_current_sensor_value; // ACS712 Salt Electrolysis sensor value (mV)
static float filter_current_sensor_value; // ACS712 Filter Pump sensor value (mV)
static float heat_current_sensor_value; // ACS712 Heat Pump sensor value (mV)

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Setup instance for flow measurements
long flow_currentMillis = 0;
long flow_previousMillis = 0;
int flow_interval = 1000;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
// 13.51 is to make 100 for the full flow
float flow_calibrationFactor;
volatile byte flow_pulseCount;
byte flow_pulse1Sec = 0;

// Setup instance for flow2 measurements
long flow2_currentMillis = 0;
long flow2_previousMillis = 0;
int flow2_interval = 1007;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
// 13.51 is to make 100 for the full flow
float flow2_calibrationFactor;
volatile byte flow2_pulseCount;
byte flow2_pulse1Sec = 0;

// Signal filtering library sample buffers
static RunningMedian samples_A_Temp[5] = { RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11) };
static RunningMedian samples_W_Temp[5] = { RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11) };
static RunningMedian samples_Ph        = RunningMedian(11);
static RunningMedian samples_Orp       = RunningMedian(11);
static RunningMedian samples_PSI       = RunningMedian(11);
static RunningMedian samples_ATemp     = RunningMedian(11);
static RunningMedian samples_AHum      = RunningMedian(11);
static RunningMedian samples_AP        = RunningMedian(11);
static RunningMedian samples_Flow      = RunningMedian(11);
static RunningMedian samples_Flow2     = RunningMedian(11);
static RunningMedian samples_SaltCurrent = RunningMedian(11); // Für ACS712 Salzelektrolyse
static RunningMedian samples_FilterCurrent = RunningMedian(11); // Für ACS712 Filterpumpe
static RunningMedian samples_HeatCurrent = RunningMedian(11); // Für ACS712 Wärmepumpe

void stack_mon(UBaseType_t&);
bool lockI2C();
void unlockI2C();
void assignTemperature(double& targetField, float temperature, const char* fieldName, uint8_t sensorIndex, const char* bus);

// PulseCounter for FLOW
void IRAM_ATTR flow_pulseCounter() {
  flow_pulseCount++;
}

// PulseCounter for FLOW2
void IRAM_ATTR flow2_pulseCounter() {
  flow2_pulseCount++;
}

void AnalogInit() {
  #ifdef EXT_ADS1115
    adc_int.setSpeed(ADS1115_SPEED_16SPS);
    adc_int.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144); // ACS712 Salt Electrolysis
    adc_int.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144); // ACS712 Filter Pump
    adc_int.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144); // ACS712 Heat Pump
    adc_int.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144); // PSI
    adc_int.setSamples(8);
    adc_ph.setSpeed(ADS1115_SPEED_16SPS);
    adc_ph.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
    adc_ph.setSamples(4);
    adc_orp.setSpeed(ADS1115_SPEED_16SPS);
    adc_orp.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
    adc_orp.setSamples(4);
  #else
    adc_int.setSpeed(ADS1115_SPEED_16SPS);
    adc_int.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144); // ACS712 Salt Electrolysis
    adc_int.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144); // ACS712 Filter Pump
    adc_int.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144); // ACS712 Heat Pump
    adc_int.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144); // PSI
    adc_int.setSamples(4);
  #endif
  }

  void assignTemperature(double& targetField, float temperature, const char* fieldName, uint8_t sensorIndex, const char* bus)
{
    if (temperature < MIN_VALID_TEMP || temperature > MAX_VALID_TEMP)
    {
        Debug.print(DBG_WARNING, "[DS18B20] Sensor %s%d (%s) - invalid temperature: %.2f °C", bus, sensorIndex, fieldName, temperature);
        char errorMsg[100];
        snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Invalid temperature for %s%d (%s): %.2f °C\"}", bus, sensorIndex, fieldName, temperature);
        mqttErrorPublish(errorMsg);
        targetField = ERROR_TEMP_VALUE;
    }
    else
    {
        targetField = temperature;
        Debug.print(DBG_VERBOSE, "[DS18B20] Sensor %s%d (%s) - Temperature: %.2f °C", bus, sensorIndex, fieldName, temperature);
    }
}

void CombinedPollingTask(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] CombinedPollingTask started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] CombinedPollingTask running...");
  vTaskDelay(DT1); // Verwende DT1 (ursprünglich für AnalogPoll)

  esp_task_wdt_add(NULL);
  TickType_t period = PT1; // Verwende PT1 (125ms), da es häufiger als PT13 (100ms) ist und beide abdeckt
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act = 0, t_min = 999, t_max = 0;
  float t_mean = 0.;
  int n = 1;
  #endif

  // Init ADC-Scans (AnalogPoll)
  lockI2C();
  adc_int.start();
  #ifdef EXT_ADS1115
  adc_ph.start();
  adc_orp.start();
  #endif
  unlockI2C();
  vTaskDelayUntil(&ticktime, period);

  const TickType_t minimumUpdateInterval = pdMS_TO_TICKS(PCF_UPDATE_INTERVAL);
  static bool updateTaskStarted = false;
  PCF8574Manager& pcfManager = PCF8574Manager::getInstance();
  
  for (;;) {
      esp_task_wdt_reset();

      #ifdef CHRONO
      td = millis();
      #endif

      if (lockI2C()) {
          // read analog sensors
          #ifdef EXT_ADS1115
          adc_ph.update();
          if (adc_ph.ready()) {
              ph_sensor_value = adc_ph.readFilter(0);
              if (ph_sensor_value >= 32768) ph_sensor_value -= 65536;
              adc_ph.start();
              samples_Ph.add(ph_sensor_value);
              storage.PhValue = (samples_Ph.getAverage(5) * 0.1875 / 1000.0) * storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;
          }

          adc_orp.update();
          if (adc_orp.ready()) {
              orp_sensor_value = adc_orp.readFilter(0);
              if (orp_sensor_value >= 32768) orp_sensor_value -= 65536;
              adc_orp.start();
              samples_Orp.add(orp_sensor_value);
              storage.OrpValue = (samples_Orp.getAverage(5) * 0.1875 / 1000.0) * storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;
          }

          adc_int.update();
          if (adc_int.ready()) {
              salt_current_sensor_value = adc_int.readFilter(0); // Kanal A0 (Salt Electrolysis)
              filter_current_sensor_value = adc_int.readFilter(1); // Kanal A1 (Filter Pump)
              heat_current_sensor_value = adc_int.readFilter(2); // Kanal A2 (Heat Pump)
              psi_sensor_value = adc_int.readFilter(3); // Kanal A3 (PSI)
              adc_int.start();
              samples_PSI.add(psi_sensor_value);
              storage.PSIValue = (samples_PSI.getAverage(5) * 0.1875 / 1000.0) * storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;

              // ACS712 Salzelektrolyse
              samples_SaltCurrent.add(salt_current_sensor_value);
              float salt_current_mV = samples_SaltCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.SaltCurrentValue = (salt_current_mV / 1000.0) * storage.SaltCurrentCalibCoeffs0 + storage.SaltCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.SaltCurrentValue < MIN_VALID_CURRENT || storage.SaltCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Salt] Invalid current value: %.2f A", storage.SaltCurrentValue);
                  storage.SaltCurrentValue = 0.0; // Fehlerwert
              }

              // ACS712 Filterpumpe
              samples_FilterCurrent.add(filter_current_sensor_value);
              float filter_current_mV = samples_FilterCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.FilterCurrentValue = (filter_current_mV / 1000.0) * storage.FilterCurrentCalibCoeffs0 + storage.FilterCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.FilterCurrentValue < MIN_VALID_CURRENT || storage.FilterCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Filter] Invalid current value: %.2f A", storage.FilterCurrentValue);
                  storage.FilterCurrentValue = 0.0; // Fehlerwert
              }

              // ACS712 Wärmepumpe
              samples_HeatCurrent.add(heat_current_sensor_value);
              float heat_current_mV = samples_HeatCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.HeatCurrentValue = (heat_current_mV / 1000.0) * storage.HeatCurrentCalibCoeffs0 + storage.HeatCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.HeatCurrentValue < MIN_VALID_CURRENT || storage.HeatCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Heat] Invalid current value: %.2f A", storage.HeatCurrentValue);
                  storage.HeatCurrentValue = 0.0; // Fehlerwert
              }

              Debug.print(DBG_DEBUG, "pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar - SaltCurrent: %5.0f - %4.2fA - FilterCurrent: %5.0f - %4.2fA - HeatCurrent: %5.0f - %4.2fA\r",
                          ph_sensor_value, storage.PhValue, orp_sensor_value, storage.OrpValue, 
                          psi_sensor_value, storage.PSIValue, salt_current_sensor_value, storage.SaltCurrentValue,
                          filter_current_sensor_value, storage.FilterCurrentValue, heat_current_sensor_value, storage.HeatCurrentValue);
          }
          #else
          adc_int.update();
          if (adc_int.ready()) {
              salt_current_sensor_value = adc_int.readFilter(0); // Kanal A0 (Salt Electrolysis)
              filter_current_sensor_value = adc_int.readFilter(1); // Kanal A1 (Filter Pump)
              heat_current_sensor_value = adc_int.readFilter(2); // Kanal A2 (Heat Pump)
              psi_sensor_value = adc_int.readFilter(3); // Kanal A3 (PSI)
              adc_int.start();

              samples_Ph.add(ph_sensor_value);
              storage.PhValue = (samples_Ph.getAverage(5) * 0.1875 / 1000.0) * storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;

              samples_Orp.add(orp_sensor_value);
              storage.OrpValue = (samples_Orp.getAverage(5) * 0.1875 / 1000.0) * storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;

              samples_PSI.add(psi_sensor_value);
              storage.PSIValue = (samples_PSI.getAverage(5) * 0.1875 / 1000.0) * storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;

              // ACS712 Salzelektrolyse
              samples_SaltCurrent.add(salt_current_sensor_value);
              float salt_current_mV = samples_SaltCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.SaltCurrentValue = (salt_current_mV / 1000.0) * storage.SaltCurrentCalibCoeffs0 + storage.SaltCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.SaltCurrentValue < MIN_VALID_CURRENT || storage.SaltCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Salt] Invalid current value: %.2f A", storage.SaltCurrentValue);
                  storage.SaltCurrentValue = 0.0; // Fehlerwert
              }

              // ACS712 Filterpumpe
              samples_FilterCurrent.add(filter_current_sensor_value);
              float filter_current_mV = samples_FilterCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.FilterCurrentValue = (filter_current_mV / 1000.0) * storage.FilterCurrentCalibCoeffs0 + storage.FilterCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.FilterCurrentValue < MIN_VALID_CURRENT || storage.FilterCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Filter] Invalid current value: %.2f A", storage.FilterCurrentValue);
                  storage.FilterCurrentValue = 0.0; // Fehlerwert
              }

              // ACS712 Wärmepumpe
              samples_HeatCurrent.add(heat_current_sensor_value);
              float heat_current_mV = samples_HeatCurrent.getAverage(5) * 0.1875; // ADC-Wert in mV
              storage.HeatCurrentValue = (heat_current_mV / 1000.0) * storage.HeatCurrentCalibCoeffs0 + storage.HeatCurrentCalibCoeffs1; // Strom in Ampere
              if (storage.HeatCurrentValue < MIN_VALID_CURRENT || storage.HeatCurrentValue > MAX_VALID_CURRENT) {
                  Debug.print(DBG_WARNING, "[ACS712 Heat] Invalid current value: %.2f A", storage.HeatCurrentValue);
                  storage.HeatCurrentValue = 0.0; // Fehlerwert
              }

              Debug.print(DBG_DEBUG, "pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar - SaltCurrent: %5.0f - %4.2fA - FilterCurrent: %5.0f - %4.2fA - HeatCurrent: %5.0f - %4.2fA\r",
                          ph_sensor_value, storage.PhValue, orp_sensor_value, storage.OrpValue, 
                          psi_sensor_value, storage.PSIValue, salt_current_sensor_value, storage.SaltCurrentValue,
                          filter_current_sensor_value, storage.FilterCurrentValue, heat_current_sensor_value, storage.HeatCurrentValue);
          }
          #endif

          unlockI2C();
      }

      #ifdef CHRONO
      t_act = millis() - td;
      if (t_act > t_max) t_max = t_act;
      if (t_act < t_min) t_min = t_act;
      t_mean += (t_act - t_mean) / n;
      ++n;
      if (n % 10 == 0) {
          Debug.print(DBG_INFO, "[CombinedPolling] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
      }
      #endif

      stack_mon(hwm);
      Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
      vTaskDelayUntil(&ticktime, period);
  }
}

void FlowInit()
{
  flow_pulseCount = 0;
  flow_previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(FLOW), flow_pulseCounter, RISING);
}

void Flow2Init()
{
  flow2_pulseCount = 0;
  flow2_previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(FLOW2), flow2_pulseCounter, FALLING);
}

void StatusLights(void *pvParameters) {
    Debug.print(DBG_INFO, "[TASKS] StatusLights started on core %d", xPortGetCoreID());
    while (!startTasks);
    Debug.print(DBG_DEBUG, "[TASKS] StatusLights running...");
    vTaskDelay(DT10);

    esp_task_wdt_add(NULL);
    TickType_t period = PT10;
    TickType_t ticktime = xTaskGetTickCount();
    static UBaseType_t hwm = 0;

    #ifdef CHRONO
    unsigned long td;
    int t_act = 0, t_min = 999, t_max = 0;
    float t_mean = 0.;
    int n = 1;
    #endif

    static uint8_t line = 0;
    for (;;) {
        esp_task_wdt_reset();

        #ifdef CHRONO
        td = millis();
        #endif

        uint8_t status = 0;
        status |= (line & 1) << 1;
        if (line == 0) {
            line = 1;
            status |= (storage.AutoMode & 1) << 2;
            status |= (AntiFreezeFiltering & 1) << 3;
            status |= (PSIError & 1) << 7;
            status |= (FLOWError & 1) << 7;
            status |= (FLOW2Error & 1) << 7;
        } else {
            line = 0;
            status |= (PhPID.GetMode() & 1) << 2;
            status |= (OrpPID.GetMode() & 1) << 3;
            status |= (!PhPump.TankLevel() & 1) << 4;
            status |= (!ChlPump.TankLevel() & 1) << 5;
            status |= (PhPump.UpTimeError & 1) << 6;
            status |= (ChlPump.UpTimeError & 1) << 7;
        }
        (status & 0xF0) ? digitalWrite(BUZZER, HIGH) : digitalWrite(BUZZER, LOW);
        if (WiFi.status() == WL_CONNECTED) status |= 0x01;
        else status &= 0xFE;
        Debug.print(DBG_VERBOSE, "Status LED : 0x%02x", status);

        uint8_t invertedStatus = ~status;
        PCF8574Manager::getInstance().queueUpdate(PCF8574_ADR, invertedStatus);
        Debug.print(DBG_VERBOSE, "[StatusLights] Queued state 0x%02X for 0x24", status);

        #ifdef CHRONO
        t_act = millis() - td;
        if (t_act > t_max) t_max = t_act;
        if (t_act < t_min) t_min = t_act;
        t_mean += (t_act - t_mean) / n;
        ++n;
        Debug.print(DBG_INFO, "[StatusLights] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
        #endif

        stack_mon(hwm);
        Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil(&ticktime, period);
    }
}

//Ph regulation loop
void pHRegulation(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] pHRegulation started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] pHRegulation running...");
  vTaskDelay(DT8);

  esp_task_wdt_add(NULL);
  TickType_t period = PT8;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for (;;) {
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif

    if (FiltrationPump.IsRunning() && (PhPID.GetMode() == AUTOMATIC)) {
      if (PhPID.Compute()) {
        Debug.print(DBG_VERBOSE, "Ph regulation: %10.2f, %13.9f, %13.9f, %17.9f", storage.PhPIDOutput, storage.PhValue, storage.Ph_SetPoint, storage.Ph_Kp);
        if (storage.PhPIDOutput < 30000.0) storage.PhPIDOutput = 0;
        Debug.print(DBG_INFO, "Ph regulation: %10.2f", storage.PhPIDOutput);
#ifdef SIMU
        newpHOutput = true;
#endif
      }
#ifdef SIMU
      else newpHOutput = false;
#endif
      unsigned long now = millis();
      if (now - storage.PhPIDwindowStartTime > storage.PhPIDWindowSize) {
        storage.PhPIDwindowStartTime += storage.PhPIDWindowSize;
      }
      if ((unsigned long)storage.PhPIDOutput <= now - storage.PhPIDwindowStartTime)
        PhPump.Stop();
      else
        PhPump.Start();
    } else {
      PhPump.Stop(); // Ensure pump is off when conditions not met
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[pHRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    vTaskDelayUntil(&ticktime, period);
  }
}

void ChlorSaltRegulation(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] ChlorSaltRegulation started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] ChlorSaltRegulation running...");
  vTaskDelay(DT6);

  esp_task_wdt_add(NULL);
  TickType_t period = PT6;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  static bool polarity_reversed = storage.SaltPolarity;
  static unsigned long last_switch_time = 0;
  static unsigned long lastUpTime = 0;
  static unsigned long last_runtime = 0;
  static unsigned long switch_polarity_time = 240;

  // Initial polarity setup
  if (storage.SaltPolarity == 0) {
    digitalWrite(SALT_POL, DIRECT);
    polarity_reversed = true;
    Debug.print(DBG_INFO, "[SaltPump] Polarity initialized to DIRECT");
  } else {
    digitalWrite(SALT_POL, REVERSE);
    polarity_reversed = false;
    Debug.print(DBG_INFO, "[SaltPump] Polarity initialized to REVERSE");
  }

  for (;;) {
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif

    // Update SaltPump runtime for polarity switch
    if (storage.Salt_Chlor && SaltPump.IsRunning() && SaltPump.UpTime != lastUpTime) {
      last_runtime = SaltPump.UpTime - lastUpTime;
      storage.SaltPumpRunTime += last_runtime;
      lastUpTime = SaltPump.UpTime;
      Debug.print(DBG_VERBOSE, "[SaltPump] Updated runtime: %lu ms", storage.SaltPumpRunTime);
    }

    // Handle polarity switch
    if (storage.Salt_Chlor && storage.SaltPumpRunTime >= switch_polarity_time * 60 * 1000) {
      storage.SaltPumpRunTime = 0;
      polarity_reversed = !polarity_reversed;

      bool wasRunning = SaltPump.IsRunning();
      if (wasRunning) {
        SaltPump.Stop();
        unsigned long stop_time = millis();
        while (millis() - stop_time < 500) vTaskDelay(1);
        Debug.print(DBG_INFO, "[SaltPump] Stopped for polarity switch");
      }

      if (polarity_reversed) {
        digitalWrite(SALT_POL, DIRECT);
        storage.SaltPolarity = 0;
        Debug.print(DBG_INFO, "[SaltPump] Polarity switched to DIRECT");
      } else {
        digitalWrite(SALT_POL, REVERSE);
        storage.SaltPolarity = 1;
        Debug.print(DBG_INFO, "[SaltPump] Polarity switched to REVERSE");
      }

      unsigned long switch_time = millis();
      while (millis() - switch_time < 500) vTaskDelay(1);

      // Restart SaltPump if it was running
      if (wasRunning) {
        SaltPump.Start();
        Debug.print(DBG_INFO, "[SaltPump] Restarted after polarity switch");
      }
    }

    if (FiltrationPump.IsRunning()) {
      if (storage.Salt_Chlor) { // Salt-Mode
        if (storage.SaltMode) { // Automatic mode
          OrpPID.SetMode(MANUAL);
          storage.Orp_RegulationOnOff = 0;
          storage.OrpPIDOutput = 0.0;
          ChlPump.Stop();

          if (!FLOW2Error && !storage.WinterMode && storage.WaterSTemp >= storage.WaterTempLowThreshold) {
            if (storage.OrpValue < (storage.Orp_SetPoint - storage.SaltDiff)) {
              Debug.print(DBG_VERBOSE, "Salt regulation on: %13.9f, %12.9f, %17.9f", storage.OrpValue, storage.Orp_SetPoint, storage.SaltDiff);
              SaltPump.Start();
            } else if (storage.OrpValue > (storage.Orp_SetPoint + storage.SaltDiff)) {
              Debug.print(DBG_INFO, "Salt regulation off: %13.9f, %12.9f, %17.9f", storage.OrpValue, storage.Orp_SetPoint, storage.SaltDiff);
              SaltPump.Stop();
            }
          } else {
            SaltPump.Stop();
            Debug.print(DBG_INFO, "[SaltPump] Stopped in auto mode due to conditions (FLOW2Error=%d, WinterMode=%d, WaterSTemp=%.2f)", 
                        FLOW2Error, storage.WinterMode, storage.WaterSTemp);
          }
        } else { // Manual mode (!storage.SaltMode)
          // Do not interfere with direct SaltPump.Start()/Stop() calls
          Debug.print(DBG_VERBOSE, "[SaltPump] Manual mode: SaltPump state=%d", SaltPump.IsRunning());
        }
      } else { // Chlorine mode (ORP regulation)
        if (OrpPID.GetMode() == AUTOMATIC) {
          if (OrpPID.Compute()) {
            Debug.print(DBG_VERBOSE, "ORP regulation: %10.2f, %13.9f, %12.9f, %17.9f", storage.OrpPIDOutput, storage.OrpValue, storage.Orp_SetPoint, storage.Orp_Kp);
            if (storage.OrpPIDOutput < 30000.0) storage.OrpPIDOutput = 0;
            Debug.print(DBG_INFO, "Orp regulation: %10.2f", storage.OrpPIDOutput);
#ifdef SIMU
            newChlOutput = true;
#endif
          }
#ifdef SIMU
          else newChlOutput = false;
#endif
          unsigned long now = millis();
          if (now - storage.OrpPIDwindowStartTime > storage.OrpPIDWindowSize) {
            storage.OrpPIDwindowStartTime += storage.OrpPIDWindowSize;
          }
          if ((unsigned long)storage.OrpPIDOutput <= now - storage.OrpPIDwindowStartTime)
            ChlPump.Stop();
          else
            ChlPump.Start();
        }
      }
    } else {
      if (storage.Salt_Chlor) {
        SaltPump.Stop();
        Debug.print(DBG_INFO, "[SaltPump] Stopped: FiltrationPump not running");
      } else {
        ChlPump.Stop();
      }
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO, "[ChlorSaltRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
    #endif

    stack_mon(hwm);
    Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    vTaskDelayUntil(&ticktime, period);
  }
}

//Flow measurements loop
void FlowMeasures(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] FlowMeasures started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] FlowMeasures running...");
  vTaskDelay(DT9);

  esp_task_wdt_add(NULL); // Register with watchdog
  TickType_t period = PT9;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for (;;) {
    esp_task_wdt_reset(); // Reset watchdog

    #ifdef CHRONO
    td = millis();
    #endif

    if (flow_currentMillis - flow_previousMillis > flow_interval) {
      portENTER_CRITICAL(&mux);
      detachInterrupt(digitalPinToInterrupt(FLOW));
      byte pulseCount = flow_pulseCount;
      flow_pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(FLOW), flow_pulseCounter, RISING);
      portEXIT_CRITICAL(&mux);
      flow_pulse1Sec = pulseCount;
      flow_calibrationFactor = storage.FLOW_Pulse;
      storage.FLOWValue = ((1000.0 / (millis() - flow_previousMillis)) * flow_pulse1Sec) / flow_calibrationFactor;
      samples_Flow.add(storage.FLOWValue);
      storage.FLOWValue = samples_Flow.getAverage(5);
      flow_previousMillis = millis();
    }

    flow2_currentMillis = millis();
    if (flow2_currentMillis - flow2_previousMillis > flow2_interval) {
      portENTER_CRITICAL(&mux);
      detachInterrupt(digitalPinToInterrupt(FLOW2));
      byte pulseCount = flow2_pulseCount;
      flow2_pulseCount = 0;
      attachInterrupt(digitalPinToInterrupt(FLOW2), flow2_pulseCounter, FALLING);
      portEXIT_CRITICAL(&mux);
      flow2_pulse1Sec = pulseCount;
      flow2_calibrationFactor = storage.FLOW2_Pulse;
      storage.FLOW2Value = ((1000.0 / (millis() - flow2_previousMillis)) * flow2_pulse1Sec) / flow2_calibrationFactor;
      samples_Flow2.add(storage.FLOW2Value);
      storage.FLOW2Value = samples_Flow2.getAverage(5);
      flow2_previousMillis = millis();
    }

    static float lastFLOWValue = 0.0;
    if (abs(storage.FLOWValue - lastFLOWValue) > 0.1) {
    Debug.print(DBG_VERBOSE, "[FLOW] Flow: %4.1f l/min - Flow2: %4.1f l/min\r", storage.FLOWValue, storage.FLOW2Value);
    lastFLOWValue = storage.FLOWValue;
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[FlowMeasures] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    vTaskDelayUntil(&ticktime, period);
  }
}

//Init DS18B20 one-wire library
void TempInit()
{
    typedef std::array<uint8_t, 8> DeviceAddress;
    DeviceAddress tempDeviceAddress_A;
    DeviceAddress tempDeviceAddress_W;
    uint8_t sensorCount_A = 0;
    uint8_t sensorCount_W = 0;

    // Start up the library
    sensors_W.begin();
    sensors_W.begin(); // Workaround for OneWire library bug
    sensors_A.begin();

    Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire W devices: %d device(s) found", sensors_W.getDeviceCount());
    Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire A devices: %d device(s) found", sensors_A.getDeviceCount());

    // Schritt 1: Sammle alle aktuell erkannten Sensoradressen
    std::vector<DeviceAddress> foundAddresses_A;
    std::vector<DeviceAddress> foundAddresses_W;

    // Sensoren auf Bus A scannen
    Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus A");
    while (sensors_A.getAddress(tempDeviceAddress_A.data(), sensorCount_A) && sensorCount_A < MAX_ADDRESSES)
    {
        foundAddresses_A.push_back(tempDeviceAddress_A);
        Debug.print(DBG_VERBOSE, "[DS18B20] Sensor A%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", 
                    sensorCount_A, tempDeviceAddress_A[0], tempDeviceAddress_A[1], tempDeviceAddress_A[2],
                    tempDeviceAddress_A[3], tempDeviceAddress_A[4], tempDeviceAddress_A[5], 
                    tempDeviceAddress_A[6], tempDeviceAddress_A[7]);
        sensorCount_A++;
    }

    // Sensoren auf Bus W scannen
    Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus W");
    while (sensors_W.getAddress(tempDeviceAddress_W.data(), sensorCount_W) && sensorCount_W < MAX_ADDRESSES)
    {
        foundAddresses_W.push_back(tempDeviceAddress_W);
        Debug.print(DBG_VERBOSE, "[DS18B20] Sensor W%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", 
                    sensorCount_W, tempDeviceAddress_W[0], tempDeviceAddress_W[1], tempDeviceAddress_W[2],
                    tempDeviceAddress_W[3], tempDeviceAddress_W[4], tempDeviceAddress_W[5], 
                    tempDeviceAddress_W[6], tempDeviceAddress_W[7]);
        sensorCount_W++;
    }

    // Schritt 2: Initialisiere leere Slots mit Null-Adressen
    uint8_t zeroAddress[8] = {0};
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        bool slotUsed_A = false;
        bool slotUsed_W = false;
        for (const auto& addr : foundAddresses_A)
        {
            if (memcmp(&storage.address_A_0[i * 8], addr.data(), 8) == 0)
            {
                slotUsed_A = true;
                break;
            }
        }
        for (const auto& addr : foundAddresses_W)
        {
            if (memcmp(&storage.address_W_0[i * 8], addr.data(), 8) == 0)
            {
                slotUsed_W = true;
                break;
            }
        }
        if (!slotUsed_A)
        {
            saveParam(("address_A_" + String(i)).c_str(), zeroAddress, 8);
        }
        if (!slotUsed_W)
        {
            saveParam(("address_W_" + String(i)).c_str(), zeroAddress, 8);
        }
    }

    // Schritt 3: Vergleiche mit gespeicherten Adressen und aktualisiere NVS
    bool usedSlots_A[MAX_ADDRESSES] = {false};
    for (size_t i = 0; i < foundAddresses_A.size(); i++)
    {
        bool addressFound = false;
        for (uint8_t j = 0; j < MAX_ADDRESSES; j++)
        {
            uint8_t storedAddress[8];
            nvs.getBytes(("address_A_" + String(j)).c_str(), storedAddress, 8);
            if (memcmp(storedAddress, foundAddresses_A[i].data(), 8) == 0)
            {
                addressFound = true;
                usedSlots_A[j] = true;
                Debug.print(DBG_INFO, "[DS18B20 - INIT] Sensor A%d address already stored in slot %d", i, j);
                break;
            }
        }

        if (!addressFound)
        {
            for (uint8_t j = 0; j < MAX_ADDRESSES; j++)
            {
                if (usedSlots_A[j]) continue;

                uint8_t storedAddress[8];
                nvs.getBytes(("address_A_" + String(j)).c_str(), storedAddress, 8);
                bool isEmpty = (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0);
                bool isInvalid = true;
                if (!isEmpty)
                {
                    isInvalid = std::none_of(foundAddresses_A.begin(), foundAddresses_A.end(),
                                             [&](const DeviceAddress& addr) { 
                                                 return memcmp(addr.data(), storedAddress, 8) == 0; 
                                             });
                }

                if (isEmpty || isInvalid)
                {
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address for sensor A%d in slot %d", i, j);
                    saveParam(("address_A_" + String(j)).c_str(), foundAddresses_A[i].data(), 8);
                    usedSlots_A[j] = true;

                    if (storage.Array_A[j] >= MAX_ADDRESSES)
                        storage.Array_A[j] = j;
                    saveParam("Array_A", storage.Array_A, MAX_ADDRESSES);
                    break;
                }
            }
        }
    }

    bool usedSlots_W[MAX_ADDRESSES] = {false};
    for (size_t i = 0; i < foundAddresses_W.size(); i++)
    {
        bool addressFound = false;
        for (uint8_t j = 0; j < MAX_ADDRESSES; j++)
        {
            uint8_t storedAddress[8];
            nvs.getBytes(("address_W_" + String(j)).c_str(), storedAddress, 8);
            if (memcmp(storedAddress, foundAddresses_W[i].data(), 8) == 0)
            {
                addressFound = true;
                usedSlots_W[j] = true;
                Debug.print(DBG_INFO, "[DS18B20 - INIT] Sensor W%d address already stored in slot %d", i, j);
                break;
            }
        }

        if (!addressFound)
        {
            for (uint8_t j = 0; j < MAX_ADDRESSES; j++)
            {
                if (usedSlots_W[j]) continue;

                uint8_t storedAddress[8];
                nvs.getBytes(("address_W_" + String(j)).c_str(), storedAddress, 8);
                bool isEmpty = (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0);
                bool isInvalid = true;
                if (!isEmpty)
                {
                    isInvalid = std::none_of(foundAddresses_W.begin(), foundAddresses_W.end(),
                                             [&](const DeviceAddress& addr) { 
                                                 return memcmp(addr.data(), storedAddress, 8) == 0; 
                                             });
                }

                if (isEmpty || isInvalid)
                {
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address for sensor W%d in slot %d", i, j);
                    saveParam(("address_W_" + String(j)).c_str(), foundAddresses_W[i].data(), 8);
                    usedSlots_W[j] = true;

                    if (storage.Array_W[j] >= MAX_ADDRESSES)
                        storage.Array_W[j] = j;
                    saveParam("Array_W", storage.Array_W, MAX_ADDRESSES);
                    break;
                }
            }
        }
    }

    // Schritt 4: Setze die Auflösung für alle Sensoren
    sensors_W.setResolution(TEMPERATURE_RESOLUTION);
    sensors_A.setResolution(TEMPERATURE_RESOLUTION);

    // Schritt 5: Fehlerbehandlung
    if (foundAddresses_A.empty() && foundAddresses_W.empty())
    {
        Debug.print(DBG_ERROR, "[DS18B20 - INIT] No temperature sensors found");
    }
    else if (foundAddresses_A.size() > MAX_ADDRESSES || foundAddresses_W.size() > MAX_ADDRESSES)
    {
        Debug.print(DBG_WARNING, "[DS18B20 - INIT] Too many sensors detected (A: %d, W: %d, max: %d)",
        foundAddresses_A.size(), foundAddresses_W.size(), MAX_ADDRESSES);
    }
    else
    {
        Debug.print(DBG_INFO, "[DS18B20 - INIT] Temperature sensors initialized");
    }
}

// Asynchrone Temperaturmessung
void getTemp()
{
    static bool measurementStarted = false;
    static unsigned long startTime = 0;
    static float temp_A[MAX_ADDRESSES] = {ERROR_TEMP_VALUE};
    static float temp_W[MAX_ADDRESSES] = {ERROR_TEMP_VALUE};
    static bool errorReported_A[MAX_ADDRESSES] = {false};
    static bool errorReported_W[MAX_ADDRESSES] = {false};
    static unsigned long lastErrorReportTime = 0;
    static const unsigned long ERROR_REPORT_INTERVAL = 600000; // 10 Minuten

    // Prüfe, ob Sensor-Objekte initialisiert sind
    if (sensors_A.getDeviceCount() == 0 && sensors_W.getDeviceCount() == 0)
    {
        if (!errorReported_A[0] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
        {
            Debug.print(DBG_ERROR, "[DS18B20] Sensor objects not initialized or no sensors detected");
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"DS18B20 sensor objects not initialized or no sensors detected\"}");
            mqttErrorPublish(errorMsg);
            errorReported_A[0] = true;
            lastErrorReportTime = millis();
        }
        measurementStarted = false;
        return;
    }

    // Schritt 1: Starte die Messung (asynchron)
    if (!measurementStarted)
    {
        sensors_A.requestTemperatures();
        sensors_W.requestTemperatures();
        startTime = millis();
        measurementStarted = true;
        Debug.print(DBG_INFO, "[DS18B20] Temperature measurement started");
        return;
    }

    // Schritt 2: Prüfe, ob die Messung abgeschlossen ist
    if (millis() - startTime < TEMP_MEASURE_TIMEOUT_MS)
    {
        return; // Warte, bis die Messung abgeschlossen ist
    }

    measurementStarted = false; // Zurücksetzen für die nächste Messung

    // Schritt 3: Lies Temperaturen für Bus A
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t* address = &storage.address_A_0[i * 8];
        bool isValidAddress = false;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (address[j] != 0)
            {
                isValidAddress = true;
                break;
            }
        }

        if (!isValidAddress)
        {
            if (!errorReported_A[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Sensor A%d - invalid address", i);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Invalid address for Sensor A%d\"}", i);
                mqttErrorPublish(errorMsg);
                errorReported_A[i] = true;
                lastErrorReportTime = millis();
            }
            temp_A[i] = ERROR_TEMP_VALUE;
            continue; // Fahre mit dem nächsten Sensor fort
        }

        float temp = sensors_A.getTempC(address);
        if (temp == DEVICE_DISCONNECTED_C)
        {
            if (!errorReported_A[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Sensor A%d - disconnected", i);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Sensor A%d disconnected\"}", i);
                mqttErrorPublish(errorMsg);
                errorReported_A[i] = true;
                lastErrorReportTime = millis();
            }
            temp_A[i] = ERROR_TEMP_VALUE;
        }
        else
        {
            temp_A[i] = temp;
            errorReported_A[i] = false; // Fehlerstatus zurücksetzen
        }
    }

    // Schritt 4: Lies Temperaturen für Bus W
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t* address = &storage.address_W_0[i * 8];
        bool isValidAddress = false;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (address[j] != 0)
            {
                isValidAddress = true;
                break;
            }
        }

        if (!isValidAddress)
        {
            if (!errorReported_W[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Sensor W%d - invalid address", i);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Invalid address for Sensor W%d\"}", i);
                mqttErrorPublish(errorMsg);
                errorReported_W[i] = true;
                lastErrorReportTime = millis();
            }
            temp_W[i] = ERROR_TEMP_VALUE;
            continue; // Fahre mit dem nächsten Sensor fort
        }

        float temp = sensors_W.getTempC(address);
        if (temp == DEVICE_DISCONNECTED_C)
        {
            if (!errorReported_W[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Sensor W%d - disconnected", i);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Sensor W%d disconnected\"}", i);
                mqttErrorPublish(errorMsg);
                errorReported_W[i] = true;
                lastErrorReportTime = millis();
            }
            temp_W[i] = ERROR_TEMP_VALUE;
        }
        else
        {
            temp_W[i] = temp;
            errorReported_W[i] = false; // Fehlerstatus zurücksetzen
        }
    }

    // Schritt 5: Prüfe Array_A und Array_W auf doppelte oder ungültige Zuweisungen
    bool usedIndices_A[MAX_ADDRESSES] = {false};
    bool usedIndices_W[MAX_ADDRESSES] = {false};
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        if (storage.Array_A[i] >= MAX_ADDRESSES)
        {
            if (!errorReported_A[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Invalid assignment in Array_A[%d]: %d", i, storage.Array_A[i]);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Invalid assignment in Array_A[%d]: %d\"}", i, storage.Array_A[i]);
                mqttErrorPublish(errorMsg);
                errorReported_A[i] = true;
                lastErrorReportTime = millis();
            }
            temp_A[i] = ERROR_TEMP_VALUE; // Setze ungültige Zuweisung auf Fehlerwert
        }
        else if (usedIndices_A[storage.Array_A[i]])
        {
            if (!errorReported_A[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Duplicate assignment in Array_A[%d]: %s", i, NV_STORAGE_MAPPING_A[storage.Array_A[i]]);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Duplicate assignment in Array_A[%d]: %s\"}", i, NV_STORAGE_MAPPING_A[storage.Array_A[i]]);
                mqttErrorPublish(errorMsg);
                errorReported_A[i] = true;
                lastErrorReportTime = millis();
            }
            temp_A[i] = ERROR_TEMP_VALUE; // Setze doppelte Zuweisung auf Fehlerwert
        }
        else
        {
            usedIndices_A[storage.Array_A[i]] = true;
        }

        if (storage.Array_W[i] >= MAX_ADDRESSES)
        {
            if (!errorReported_W[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Invalid assignment in Array_W[%d]: %d", i, storage.Array_W[i]);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Invalid assignment in Array_W[%d]: %d\"}", i, storage.Array_W[i]);
                mqttErrorPublish(errorMsg);
                errorReported_W[i] = true;
                lastErrorReportTime = millis();
            }
            temp_W[i] = ERROR_TEMP_VALUE; // Setze ungültige Zuweisung auf Fehlerwert
        }
        else if (usedIndices_W[storage.Array_W[i]])
        {
            if (!errorReported_W[i] || (millis() - lastErrorReportTime > ERROR_REPORT_INTERVAL))
            {
                Debug.print(DBG_WARNING, "[DS18B20] Duplicate assignment in Array_W[%d]: %s", i, NV_STORAGE_MAPPING_W[storage.Array_W[i]]);
                char errorMsg[100];
                snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"Duplicate assignment in Array_W[%d]: %s\"}", i, NV_STORAGE_MAPPING_W[storage.Array_W[i]]);
                mqttErrorPublish(errorMsg);
                errorReported_W[i] = true;
                lastErrorReportTime = millis();
            }
            temp_W[i] = ERROR_TEMP_VALUE; // Setze doppelte Zuweisung auf Fehlerwert
        }
        else
        {
            usedIndices_W[storage.Array_W[i]] = true;
        }
    }

    // Schritt 6: RunningMedian für die Temperaturen
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++) {
        if (temp_A[i] != ERROR_TEMP_VALUE) {
            samples_A_Temp[i].add(temp_A[i]);
            temp_A[i] = samples_A_Temp[i].getAverage(5);
        }
        if (temp_W[i] != ERROR_TEMP_VALUE) {
            samples_W_Temp[i].add(temp_W[i]);
            temp_W[i] = samples_W_Temp[i].getAverage(5);
        }
    }

    // Schritt 7: Weise Temperaturen zu, basierend auf NV_STORAGE_MAPPING_A/W
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        if (storage.Array_A[i] < MAX_ADDRESSES && temp_A[i] != ERROR_TEMP_VALUE)
        {
            const char* fieldName = NV_STORAGE_MAPPING_A[storage.Array_A[i]];
            if (strcmp(fieldName, "SolarTemp") == 0)
                assignTemperature(storage.SolarTemp, temp_A[i], fieldName, i, "A");
            else if (strcmp(fieldName, "SolarVLTemp") == 0)
                assignTemperature(storage.SolarVLTemp, temp_A[i], fieldName, i, "A");
            else if (strcmp(fieldName, "SolarRLTemp") == 0)
                assignTemperature(storage.SolarRLTemp, temp_A[i], fieldName, i, "A");
            else if (strcmp(fieldName, "AirInTemp") == 0)
                assignTemperature(storage.AirInTemp, temp_A[i], fieldName, i, "A");
            else if (strcmp(fieldName, "AirTemp") == 0)
                assignTemperature(storage.AirTemp, temp_A[i], fieldName, i, "A");
        }

        if (storage.Array_W[i] < MAX_ADDRESSES && temp_W[i] != ERROR_TEMP_VALUE)
        {
            const char* fieldName = NV_STORAGE_MAPPING_W[storage.Array_W[i]];
            if (strcmp(fieldName, "WaterSTemp") == 0)
                assignTemperature(storage.WaterSTemp, temp_W[i], fieldName, i, "W");
            else if (strcmp(fieldName, "WaterITemp") == 0)
                assignTemperature(storage.WaterITemp, temp_W[i], fieldName, i, "W");
            else if (strcmp(fieldName, "WaterBTemp") == 0)
                assignTemperature(storage.WaterBTemp, temp_W[i], fieldName, i, "W");
            else if (strcmp(fieldName, "WaterWPTemp") == 0)
                assignTemperature(storage.WaterWPTemp, temp_W[i], fieldName, i, "W");
            else if (strcmp(fieldName, "WaterWTTemp") == 0)
                assignTemperature(storage.WaterWTTemp, temp_W[i], fieldName, i, "W");
        }
    }

    // Schritt 8: Fehlerprotokollierung (nur wenn kein Sensor gültige Werte liefert)
    bool anyValid = false;
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++) {
        if (temp_A[i] != ERROR_TEMP_VALUE || temp_W[i] != ERROR_TEMP_VALUE) {
            anyValid = true;
            break;
        }
    }
    if (!anyValid)
    {
        Debug.print(DBG_ERROR, "[DS18B20] No valid temperatures read");
    }
    else
    {
        Debug.print(DBG_INFO, "[DS18B20] Temperatures successfully read and assigned");
    }
}

// Temperaturmessungs-Task
void TempTask(void *pvParameters) {
    Debug.print(DBG_INFO, "[TASKS] TempTask started on core %d", xPortGetCoreID());
    while (!startTasks);
    Debug.print(DBG_DEBUG, "[TASKS] TempTask running...");
    vTaskDelay(DT4);

    esp_task_wdt_add(NULL); // Register with watchdog
    TickType_t period = PT4;
    TickType_t ticktime = xTaskGetTickCount();
    static UBaseType_t hwm = 0;

    #ifdef CHRONO
    unsigned long td;
    int t_act = 0, t_min = 999, t_max = 0;
    float t_mean = 0.;
    int n = 1;
    #endif

    for (;;) {
        esp_task_wdt_reset(); // Reset watchdog

        #ifdef CHRONO
        td = millis();
        #endif

        getTemp();

        #ifdef CHRONO
        t_act = millis() - td;
        if (t_act > t_max) t_max = t_act;
        if (t_act < t_min) t_min = t_act;
        t_mean += (t_act - t_mean) / n;
        ++n;
        Debug.print(DBG_INFO, "[TempTask] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
        #endif

        stack_mon(hwm);
        Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil(&ticktime, period);
    }
}

// Asynchronous reading of BME280 data
void readBME280(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] readBME280 started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] readBME280 running...");
  vTaskDelay(DT5);

  esp_task_wdt_add(NULL);
  TickType_t period = PT5;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  // Initialisiere BME280 einmal
  static bool bmeInitialized = false;
  if (!bmeInitialized)
  {
      if (bme.begin(0x76))
      {
          Debug.print(DBG_INFO, "[BME280] BME280 initialized successfully");
          bmeInitialized = true;
      }
      else
      {
          Debug.print(DBG_ERROR, "[BME280] Failed to initialize BME280");
      }
  }

  for (;;) {
      esp_task_wdt_reset();

      #ifdef CHRONO
      td = millis();
      #endif

      if (bmeInitialized)
      {
          lockI2C();
          storage.AirHum = bme.readHumidity();
          storage.AirPress = bme.readPressure() / 100.0F;
          storage.AirTemp = bme.readTemperature();
          samples_AHum.add(storage.AirHum);
          samples_AP.add(storage.AirPress);
          samples_ATemp.add(storage.AirTemp);
          storage.AirHum = samples_AHum.getAverage();
          storage.AirPress = samples_AP.getAverage();
          storage.AirTemp = samples_ATemp.getAverage();
          Debug.print(DBG_DEBUG, "[BME280] BME280: T=%6.2f°C P=%7.2fhPa H=%6.2f%%",
                      storage.AirTemp, storage.AirPress, storage.AirHum);
          unlockI2C();
      }
      else
      {
          Debug.print(DBG_WARNING, "[BME280] BME280 not initialized");
      }

      #ifdef CHRONO
      t_act = millis() - td;
      if(t_act > t_max) t_max = t_act;
      if(t_act < t_min) t_min = t_act;
      t_mean += (t_act - t_mean)/n;
      ++n;
      Debug.print(DBG_INFO,"[readBME280] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
      #endif

      stack_mon(hwm);
      Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
      vTaskDelayUntil(&ticktime, period);
  }
}