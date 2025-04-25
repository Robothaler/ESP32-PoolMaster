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
uint8_t sensorCount_A = 0;          // Amount of found sensors
uint8_t sensorCount_W = 0;          // Amount of found sensors

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
int flow2_interval = 1000;
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

void stack_mon(UBaseType_t&);
bool lockI2C();
void unlockI2C();

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
  adc_int.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144);
  adc_int.setSamples(8);
  adc_ph.setSpeed(ADS1115_SPEED_16SPS);
  adc_ph.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
  adc_ph.setSamples(4);
  adc_orp.setSpeed(ADS1115_SPEED_16SPS);
  adc_orp.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
  adc_orp.setSamples(4);
#else
  adc_int.setSpeed(ADS1115_SPEED_16SPS);
  adc_int.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  adc_int.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144);
  adc_int.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144);
  adc_int.setSamples(3);
#endif
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
              psi_sensor_value = adc_int.readFilter(0);
              adc_int.start();
              samples_PSI.add(psi_sensor_value);
              storage.PSIValue = (samples_PSI.getAverage(5) * 0.1875 / 1000.0) * storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;
              Debug.print(DBG_DEBUG, "pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar\r",
                          ph_sensor_value, storage.PhValue, orp_sensor_value, storage.OrpValue, psi_sensor_value, storage.PSIValue);
          }
          #else
          adc_int.update();
          if (adc_int.ready()) {
              orp_sensor_value = adc_int.readFilter(0);
              ph_sensor_value = adc_int.readFilter(1);
              psi_sensor_value = adc_int.readFilter(2);
              adc_int.start();

              samples_Ph.add(ph_sensor_value);
              storage.PhValue = (samples_Ph.getAverage(5) * 0.1875 / 1000.0) * storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;

              samples_Orp.add(orp_sensor_value);
              storage.OrpValue = (samples_Orp.getAverage(5) * 0.1875 / 1000.0) * storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;

              samples_PSI.add(psi_sensor_value);
              storage.PSIValue = (samples_PSI.getAverage(5) * 0.1875 / 1000.0) * storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;

              Debug.print(DBG_DEBUG, "pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar\r",
                          ph_sensor_value, storage.PhValue, orp_sensor_value, storage.OrpValue, psi_sensor_value, storage.PSIValue);
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

  attachInterrupt(digitalPinToInterrupt(FLOW2), flow2_pulseCounter, RISING);
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
            status |= (I2CError & 1) << 7;
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

  for (;;) {
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif

    if (FiltrationPump.IsRunning()) {
      if (storage.Salt_Chlor) { // Salt-Mode
        if (SaltPump.UpTime != lastUpTime) {
          last_runtime = SaltPump.UpTime - lastUpTime;
          storage.SaltPumpRunTime += last_runtime;
          lastUpTime = SaltPump.UpTime;
        }

        if (storage.SaltPumpRunTime >= switch_polarity_time * 60 * 1000) {
          storage.SaltPumpRunTime = 0;
          polarity_reversed = !polarity_reversed;

          if (SaltPump.IsRunning()) {
            SaltPump.Stop();
            unsigned long stop_time = millis();
            while (millis() - stop_time < 500) vTaskDelay(1);
          }

          if (polarity_reversed) {
            digitalWrite(SALT_POL, DIRECT);
            storage.SaltPolarity = 0;
          } else {
            digitalWrite(SALT_POL, REVERSE);
            storage.SaltPolarity = 1;
          }

          unsigned long switch_time = millis();
          while (millis() - switch_time < 500) vTaskDelay(1);
          SaltPump.Start();
        }

        if (storage.SaltMode && !FLOW2Error && !storage.WinterMode && storage.WaterSTemp >= storage.WaterTempLowThreshold) {
          OrpPID.SetMode(MANUAL);
          storage.Orp_RegulationOnOff = 0;
          storage.OrpPIDOutput = 0.0;
          ChlPump.Stop();

          if (storage.OrpValue < (storage.Orp_SetPoint - storage.SaltDiff)) {
            Debug.print(DBG_VERBOSE, "Salt regulation on: %13.9f, %12.9f, %17.9f", storage.OrpValue, storage.Orp_SetPoint, storage.SaltDiff);
            SaltPump.Start();
          } else if (storage.OrpValue > (storage.Orp_SetPoint + storage.SaltDiff)) {
            Debug.print(DBG_INFO, "Salt regulation off: %13.9f, %12.9f, %17.9f", storage.OrpValue, storage.Orp_SetPoint, storage.SaltDiff);
            SaltPump.Stop();
          }
        } else {
          SaltPump.Stop();
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
      if (storage.Salt_Chlor) SaltPump.Stop();
      else ChlPump.Stop();
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[ChlorSaltRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
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

    flow_currentMillis = millis();
    if (flow_currentMillis - flow_previousMillis > flow_interval) {
      flow_pulse1Sec = flow_pulseCount;
      flow_pulseCount = 0;
      flow_calibrationFactor = storage.FLOW_Pulse;
      storage.FLOWValue = ((1000.0 / (millis() - flow_previousMillis)) * flow_pulse1Sec) / flow_calibrationFactor;
      samples_Flow.add(storage.FLOWValue);
      storage.FLOWValue = samples_Flow.getAverage(5);
      flow_previousMillis = millis();
    }

    flow2_currentMillis = millis();
    if (flow2_currentMillis - flow2_previousMillis > flow2_interval) {
      flow2_pulse1Sec = flow2_pulseCount;
      flow2_pulseCount = 0;
      flow2_calibrationFactor = storage.FLOW2_Pulse;
      storage.FLOW2Value = ((1000.0 / (millis() - flow2_previousMillis)) * flow2_pulse1Sec) / flow2_calibrationFactor;
      samples_Flow2.add(storage.FLOW2Value);
      storage.FLOW2Value = samples_Flow2.getAverage(5);
      flow2_previousMillis = millis();
    }

    Debug.print(DBG_VERBOSE, "[FLOW] Flow: %4.1f l/min - Flow2: %4.1f l/min\r", storage.FLOWValue, storage.FLOW2Value);

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
  bool error = false;

  DeviceAddress tempDeviceAddress_A;  // Adresse des gefundenen Sensors
  DeviceAddress tempDeviceAddress_W;  // Adresse des gefundenen Sensors

  // Start up the library
  sensors_W.begin();
  sensors_W.begin(); // two times to work-around of a OneWire library bug for enumeration
  sensors_A.begin();

  Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire W devices: %d device(s) found", sensors_W.getDeviceCount());
  Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire A devices: %d device(s) found", sensors_A.getDeviceCount());

  // Search for all sensors on bus A and save their addresses in tempDeviceAddress
  Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus A\n");
  String foundSensors_A = ""; // Variable to store list of found sensors
  while (sensors_A.getAddress(tempDeviceAddress_A, sensorCount_A) && sensorCount_A < MAX_ADDRESSES)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
        foundSensors_A += String(tempDeviceAddress_A[i], HEX) + " ";
    }

    // Check if address is already stored in NVS
    bool addressFound = false;
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t storedAddress[8];
        nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress, 8);
        if (memcmp(storedAddress, tempDeviceAddress_A, 8) == 0)
        {
          addressFound = true;
          break;
        }
    }

    // Save address in NVS if not already stored
    if (!addressFound)
    {
        for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
        {
          uint8_t storedAddress[8];
          nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress, 8);
          if (memcmp(storedAddress, tempDeviceAddress_A, 8) == 0)
          {
                    // Address already stored
                    break;
          }
          else if (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0)
          {
                    // Unused storage space found
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address in NVS\n");
                    saveParam(("address_A_" + String(i)).c_str(), tempDeviceAddress_A, 8);
                    break;
          }
        }
    }

    // Debug output for each sensor found
    Debug.print(DBG_VERBOSE, "[DS18B20] Sensor A%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", sensorCount_A, tempDeviceAddress_A[0], tempDeviceAddress_A[1], tempDeviceAddress_A[2],
    tempDeviceAddress_A[3], tempDeviceAddress_A[4], tempDeviceAddress_A[5], tempDeviceAddress_A[6], tempDeviceAddress_A[7]);

    sensorCount_A++;
  }

  // Search for all sensors on bus W and save their addresses in tempDeviceAddress
  Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus W\n");
  String foundSensors_W = ""; // Variable to store list of found sensors
  while (sensors_W.getAddress(tempDeviceAddress_W, sensorCount_W) && sensorCount_W < MAX_ADDRESSES)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
        foundSensors_W += String(tempDeviceAddress_W[i], HEX) + " ";
    }

    // Check if address is already stored in NVS
    bool addressFound = false;
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t storedAddress[8];
        nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress, 8);
        if (memcmp(storedAddress, tempDeviceAddress_W, 8) == 0)
        {
          addressFound = true;
          break;
        }
    }

    // Save address in NVS if not already stored
    if (!addressFound)
    {
        for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
        {
          uint8_t storedAddress[8];
          nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress, 8);
          if (memcmp(storedAddress, tempDeviceAddress_W, 8) == 0)
          {
                    // Address already stored
                    break;
          }
          else if (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0)
          {
                    // Unused storage space found
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address in NVS\n");
                    saveParam(("address_W_" + String(i)).c_str(), tempDeviceAddress_W, 8);
                    break;
          }
        }
    }

    // Debug output for each sensor found
    Debug.print(DBG_VERBOSE, "[DS18B20]  Sensor W%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", sensorCount_W, tempDeviceAddress_W[0], tempDeviceAddress_W[1], tempDeviceAddress_W[2],
    tempDeviceAddress_W[3], tempDeviceAddress_W[4], tempDeviceAddress_W[5], tempDeviceAddress_W[6], tempDeviceAddress_W[7]);

    sensorCount_W++;
  }

    // set resolution for all sensors
    sensors_W.setResolution(TEMPERATURE_RESOLUTION);
    sensors_A.setResolution(TEMPERATURE_RESOLUTION);

    if (error)
    {
        Debug.print(DBG_ERROR, "[DS18B20 - INIT] Error initializing temperature sensors");
    }
    else
    {
        Debug.print(DBG_INFO, "[DS18B20 - INIT] Temperature sensors initialized\n");
    }
  }

//Request temperature asynchronously
//in case of reading error, the buffer is not updated and the last value is kept
void getTemp(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] getTemp started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] getTemp running...");
  vTaskDelay(DT4);

  esp_task_wdt_add(NULL);
  TickType_t period = PT4;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  // Asynchrone Initialisierung
  sensors_W.setWaitForConversion(false);
  sensors_A.setWaitForConversion(false);
  sensors_W.requestTemperatures();
  sensors_A.requestTemperatures();
  unsigned long lastRequest = millis();
  bool waitingForConversion = true;

  for (;;) {
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif

    if (waitingForConversion && millis() - lastRequest >= 800) {
      // Lese Sensoren auf Bus A
      for (int i = 0; i < sensorCount_A; i++) {
        byte currentAddress_A[8];
        uint8_t storedAddress_A[8];
        nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress_A, 8);
        memcpy(currentAddress_A, storedAddress_A, 8);

        float temp = sensors_A.getTempC(currentAddress_A);
        if (temp == NAN || temp == -127) {
          Debug.print(DBG_WARNING, "[DS18B20] Error getting temperature for sensor A%d", i);
        } else {
          samples_A_Temp[i].add(temp);
          float averagedTemp = samples_A_Temp[i].getAverage(5);
          switch (storage.Array_A[i]) {
            case 0: storage.SolarTemp = averagedTemp; break;
            case 1: storage.SolarVLTemp = averagedTemp; break;
            case 2: storage.SolarRLTemp = averagedTemp; break;
            case 3: storage.AirInTemp = averagedTemp; break;
            case 4: if (bme.begin(0x76)) storage.AirTemp = averagedTemp; break;
            default: break;
          }
          Debug.print(DBG_DEBUG, "[DS18B20] Sensor A%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X - Temperature: %6.2f°C", i,
                      currentAddress_A[0], currentAddress_A[1], currentAddress_A[2], currentAddress_A[3],
                      currentAddress_A[4], currentAddress_A[5], currentAddress_A[6], currentAddress_A[7], averagedTemp);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // CPU freigeben
      }

      // Lese Sensoren auf Bus W
      for (int i = 0; i < sensorCount_W; i++) {
        byte currentAddress_W[8];
        uint8_t storedAddress_W[8];
        nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress_W, 8);
        memcpy(currentAddress_W, storedAddress_W, 8);

        float temp = sensors_W.getTempC(currentAddress_W);
        if (temp == NAN || temp == -127) {
          Debug.print(DBG_WARNING, "[DS18B20] Error getting temperature for sensor W%d", i);
        } else {
          samples_W_Temp[i].add(temp);
          float averagedTemp = samples_W_Temp[i].getAverage(5);
          switch (storage.Array_W[i]) {
            case 0: storage.WaterSTemp = averagedTemp; break;
            case 1: storage.WaterITemp = averagedTemp; break;
            case 2: storage.WaterBTemp = averagedTemp; break;
            case 3: storage.WaterWPTemp = averagedTemp; break;
            case 4: storage.WaterWTTemp = averagedTemp; break;
            default: break;
          }
          Debug.print(DBG_DEBUG, "[DS18B20] Sensor W%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X - Temperature: %6.2f°C", i,
                      currentAddress_W[0], currentAddress_W[1], currentAddress_W[2], currentAddress_W[3],
                      currentAddress_W[4], currentAddress_W[5], currentAddress_W[6], currentAddress_W[7], averagedTemp);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      // Log all temperatures
      Debug.print(DBG_DEBUG, "[DS18B20] SolarTemp: %6.2f°C", storage.SolarTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] SolarVLTemp: %6.2f°C", storage.SolarVLTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] SolarRLTemp: %6.2f°C", storage.SolarRLTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] AirInTemp: %6.2f°C", storage.AirInTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] AirTemp: %6.2f°C", storage.AirTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] WaterSTemp: %6.2f°C", storage.WaterSTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] WaterITemp: %6.2f°C", storage.WaterITemp);
      Debug.print(DBG_DEBUG, "[DS18B20] WaterBTemp: %6.2f°C", storage.WaterBTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] WaterWPTemp: %6.2f°C", storage.WaterWPTemp);
      Debug.print(DBG_DEBUG, "[DS18B20] WaterWTTemp: %6.2f°C", storage.WaterWTTemp);

      waitingForConversion = false;
    }

    if (!waitingForConversion && millis() - lastRequest >= 1000) {
      sensors_W.requestTemperatures();
      sensors_A.requestTemperatures();
      lastRequest = millis();
      waitingForConversion = true;
      Debug.print(DBG_VERBOSE, "[DS18B20] Requested temperatures");
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[getTemp] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
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

  esp_task_wdt_add(NULL); // Register with watchdog
  TickType_t period = PT5;
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

    bool success = bme.begin(0x76);
    if (success) {
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
    } else {
      Debug.print(DBG_WARNING, "[BME280] Error getting BME280 data");
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