// MQTT publish tasks
// - PublishSettings (PoolTopicSet1-6)
// - PublishMeasures (PoolTopicMeas1-2)
// The tasks are waiting for notification to publish. PublishMeasures has also a timeout allowing to publish
// measures regularly

#undef __STRICT_ANSI__
#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"

// Size of the buffer to store outgoing JSON messages
#define PAYLOAD_BUFFER_LENGTH 150

// BitMaps with GPIO states
static uint8_t BitMap1 = 0;
static uint16_t BitMap2 = 0; // Extended to 16 bits to accommodate additional device states
static uint8_t BitMap3 = 0;

static const char* PoolTopicMeas1 = POOLTOPIC"Meas1";
static const char* PoolTopicMeas2 = POOLTOPIC"Meas2";
static const char* PoolTopicMeas3 = POOLTOPIC"Meas3";
static const char* PoolTopicMeas4 = POOLTOPIC"Meas4"; // Added for motor valve states
static const char* PoolTopicSet1  = POOLTOPIC"Set1";
static const char* PoolTopicSet2  = POOLTOPIC"Set2";
static const char* PoolTopicSet3  = POOLTOPIC"Set3";
static const char* PoolTopicSet4  = POOLTOPIC"Set4";
static const char* PoolTopicSet5  = POOLTOPIC"Set5";
static const char* PoolTopicSet6  = POOLTOPIC"Set6";
static const char* PoolTemp       = POOLTOPIC"POOL/temperature";

// External MotorValve instances
extern MotorValve ELD_Treppe;
extern MotorValve ELD_Hinten;
extern MotorValve WP_Vorlauf;
extern MotorValve WP_Mischer;
extern MotorValve Bodenablauf;
extern MotorValve Solarvalve;

int freeRam(void);
void stack_mon(UBaseType_t&);

// Encode digital inputs states into one Byte (more efficient to send over MQTT)
void EncodeBitMap()
{
  BitMap1 = 0;
  BitMap2 = 0;
  BitMap3 = 0;

  BitMap1 |= (FiltrationPump.IsRunning() & 1) << 4;
  BitMap1 |= (PhPump.IsRunning() & 1) << 3;
  BitMap1 |= (ChlPump.IsRunning() & 1) << 2;
  BitMap1 |= (PhPump.TankLevel() & 1) << 1;
  BitMap1 |= (ChlPump.TankLevel() & 1) << 0;

  BitMap2 |= (PhPID.GetMode() & 1) << 13;
  BitMap2 |= (OrpPID.GetMode() & 1) << 12;
  BitMap2 |= (storage.AutoMode & 1) << 11;
  BitMap2 |= (storage.WaterHeat & 1) << 10;
  BitMap2 |= (RobotPump.IsRunning() & 1) << 9;
  BitMap2 |= !digitalRead(RELAY_R0) << 8;
  BitMap2 |= !digitalRead(RELAY_R1) << 7;
  BitMap2 |= (storage.WinterMode & 1) << 6;
  BitMap2 |= (WaterFill.IsRunning() & 1) << 5;     // Water fill system running state
  BitMap2 |= (WaterHeatPump.IsRunning() & 1) << 4; // Water heat pump running state
  BitMap2 |= (SolarPump.IsRunning() & 1) << 3;     // Solar pump running state
  BitMap2 |= (HeatPump.IsRunning() & 1) << 2;      // Heat pump running state
  BitMap2 |= (SaltPump.IsRunning() & 1) << 1;      // Salt pump running state
  BitMap2 |= (0 & 1U) << 0;                        // Reserved

  BitMap3 |= (FLOW2Error & 1) << 4;
  BitMap3 |= (FLOWError & 1) << 3;
  BitMap3 |= (PSIError & 1) << 2;
  BitMap3 |= (PhPump.UpTimeError & 1) << 1;
  BitMap3 |= (ChlPump.UpTimeError & 1) << 0;  
}

void PublishTopic(const char* topic, JsonDocument& root)
{
  char Payload[PAYLOAD_BUFFER_LENGTH];
  size_t n = serializeJson(root, Payload);
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (mqttClient.publish(topic, 1, true, Payload, n) != 0)
  {
    Debug.print(DBG_DEBUG, "Publish: %s - size: %d/%d", Payload, root.size(), n);
  }
  else
  {
    Debug.print(DBG_DEBUG, "Unable to publish: %s", Payload);
  }
  xSemaphoreGive(mutex);
}

// Publishes system settings to MQTT broker
void SettingsPublish(void *pvParameters)
{
  Debug.print(DBG_INFO, "[TASKS] SettingsPublish started on core %d", xPortGetCoreID());
  while(!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] SettingsPublish running...");
  vTaskDelay(DT12); // Scheduling offset 

  uint32_t mod1 = xTaskGetTickCount() % 1000; // This is the offset to respect for future resume
  uint32_t mod2;
  TickType_t waitTime;

  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif
  
  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif

    Debug.print(DBG_DEBUG,"[PublishSettings] start");
         
    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(10) + 8; // +8 as there is the Firmw String
        StaticJsonDocument<capacity> root;

        root["FW"]     = Firmw;                            // Firmware revision
        root["FSta"]   = storage.FiltrationStart;          // Computed filtration start hour, in the morning (hours)
        root["FStaM"]  = storage.FiltrationStartMin;       // Earliest Filtration start hour, in the morning (hours)
        root["FDu"]    = storage.FiltrationDuration;       // Computed filtration duration based on water temperature (hours)
        root["FStoM"]  = storage.FiltrationStopMax;        // Latest hour for the filtration to run
        root["FSto"]   = storage.FiltrationStop;           // Computed filtration stop hour, equal to FSta + FDu (hour)
        root["pHUTL"]  = storage.PhPumpUpTimeLimit / 60;   // Max allowed daily run time for the pH pump (/!\ mins)
        root["ChlUTL"] = storage.ChlPumpUpTimeLimit / 60;  // Max allowed daily run time for the Chl pump (/!\ mins)
        root["SStaM"]  = storage.SolarStartMin;            // Earliest Solar start hour, in the morning (hours)
        root["SStoM"]  = storage.SolarStopMax;             // Latest hour for the solar to run

        PublishTopic(PoolTopicSet1, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(8);
        StaticJsonDocument<capacity> root;

        root["pHWS"]    = storage.PhPIDWindowSize / 1000 / 60;        // pH PID window size (/!\ mins)
        root["ChlWS"]   = storage.OrpPIDWindowSize / 1000 / 60;       // Orp PID window size (/!\ mins)
        root["pHSP"]    = storage.Ph_SetPoint * 100;                  // pH setpoint (/!\ x100)
        root["OrpSP"]   = storage.Orp_SetPoint;                       // Orp setpoint
        root["WSP"]     = storage.WaterTemp_SetPoint * 100;           // Water temperature setpoint (/!\ x100)
        root["WLT"]     = storage.WaterTempLowThreshold * 100;        // Water temperature low threshold to activate anti-freeze mode (/!\ x100)
        root["PSIHT"]   = storage.PSI_HighThreshold * 100;            // Water pressure high threshold to trigger error (/!\ x100)
        root["PSIMT"]   = storage.PSI_MedThreshold * 100;             // Water pressure medium threshold (unused yet) (/!\ x100)

        PublishTopic(PoolTopicSet2, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(12);
        StaticJsonDocument<capacity> root;

        root["pHC0"]    = storage.pHCalibCoeffs0;            // pH sensor calibration coefficient C0
        root["pHC1"]    = storage.pHCalibCoeffs1;            // pH sensor calibration coefficient C1
        root["OrpC0"]   = storage.OrpCalibCoeffs0;           // Orp sensor calibration coefficient C0
        root["OrpC1"]   = storage.OrpCalibCoeffs1;           // Orp sensor calibration coefficient C1
        root["PSIC0"]   = storage.PSICalibCoeffs0;           // Pressure sensor calibration coefficient C0
        root["PSIC1"]   = storage.PSICalibCoeffs1;           // Pressure sensor calibration coefficient C1
        root["SaltC0"]  = storage.SaltCurrentCalibCoeffs0;   // Salt current sensor calibration coefficient C0
        root["SaltC1"]  = storage.SaltCurrentCalibCoeffs1;   // Salt current sensor calibration coefficient C1
        root["FiltC0"]  = storage.FilterCurrentCalibCoeffs0; // Filter current sensor calibration coefficient C0
        root["FiltC1"]  = storage.FilterCurrentCalibCoeffs1; // Filter current sensor calibration coefficient C1
        root["HeatC0"]  = storage.HeatCurrentCalibCoeffs0;   // Heat current sensor calibration coefficient C0
        root["HeatC1"]  = storage.HeatCurrentCalibCoeffs1;   // Heat current sensor calibration coefficient C1

        PublishTopic(PoolTopicSet3, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(8);
        StaticJsonDocument<capacity> root;

        root["pHKp"]  = storage.Ph_Kp;                      // pH PID coefficient Kp
        root["pHKi"]  = storage.Ph_Ki;                      // pH PID coefficient Ki
        root["pHKd"]  = storage.Ph_Kd;                      // pH PID coefficient Kd
        root["OrpKp"] = storage.Orp_Kp;                     // Orp PID coefficient Kp
        root["OrpKi"] = storage.Orp_Ki;                     // Orp PID coefficient Ki
        root["OrpKd"] = storage.Orp_Kd;                     // Orp PID coefficient Kd
        root["Dpid"]  = storage.DelayPIDs;                  // Delay from FSta for the water regulation/PIDs to start (mins) 
        root["PubP"]  = storage.PublishPeriod/1000;         // Settings publish period in sec

        PublishTopic(PoolTopicSet4, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(4);
        StaticJsonDocument<capacity> root;

        root["pHTV"]    = storage.pHTankVol;                // Acid tank nominal volume (Liters)
        root["ChlTV"]   = storage.ChlTankVol;               // Chl tank nominal volume (Liters)
        root["pHFR"]    = storage.pHPumpFR;                 // Acid pump flow rate (L/hour)
        root["OrpFR"]   = storage.ChlPumpFR;                // Chl pump flow rate (L/hour)
        
        PublishTopic(PoolTopicSet5, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(15);
        StaticJsonDocument<capacity> root;

        root["PhReg"]   = storage.Ph_RegulationOnOff;       // pH regulation enabled/disabled
        root["OrpReg"]  = storage.Orp_RegulationOnOff;      // Orp regulation enabled/disabled
        root["SolLoc"]  = storage.SolarLocExt;              // Solar control local/external
        root["SolMod"]  = storage.SolarMode;                // Solar mode enabled/disabled
        root["SaltChl"] = storage.Salt_Chlor;               // Salt or chlorination enabled/disabled
        root["SaltM"]   = storage.SaltMode;                 // Salt regulation mode enabled/disabled
        root["SaltP"]   = storage.SaltPolarity;             // Salt system polarity
        root["ValvM"]   = storage.ValveMode;                // Valve mode enabled/disabled
        root["CleanM"]  = storage.CleanMode;                // Cleaning mode enabled/disabled
        root["ValvS"]   = storage.ValveSwitch;              // Valve switch state
        root["WFMod"]   = storage.WaterFillMode;            // Water fill mode enabled/disabled
        root["WFUTL"]   = storage.WaterFillUpTimeLimit / 1000; // Max allowed water fill run time (seconds)
        root["WFDur"]   = storage.WaterFillDuration / 1000;   // Water fill duration (seconds)
        root["WFAnC"]   = storage.WaterFillAnCon;           // Water fill control configuration
        root["SaltD"]   = storage.SaltDiff;                 // Salt regulation hysteresis threshold

        PublishTopic(PoolTopicSet6, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    // Display remaining RAM space. For debug
    Debug.print(DBG_DEBUG,"[memCheck]: %db",freeRam());

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[PublishSettings] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);    
    ulTaskNotifyTake(pdFALSE,portMAX_DELAY);
    mod2 = xTaskGetTickCount() % 1000;
    if(mod2 <= mod1)
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
  Debug.print(DBG_INFO,"[TASKS] PublishMeasures started on core %d", xPortGetCoreID());
  while(!startTasks);
  Debug.print(DBG_DEBUG,"[TASKS] PublishMeasures running...");
  vTaskDelay(DT11); // Scheduling offset 
  uint32_t mod1 = xTaskGetTickCount() % 1000; // This is the offset to respect for future resume

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
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  WaitTimeOut = (TickType_t)storage.PublishPeriod/portTICK_PERIOD_MS;

  for(;;)
  {   
    rc = ulTaskNotifyTake(pdFALSE,WaitTimeOut); 
  
    if(rc != 0) // Notification => wait for next offset
    {
      mod2 = xTaskGetTickCount() % 1000;
      if(mod2 <= mod1)
        waitTime = mod1 - mod2;
      else
        waitTime = 1000 + mod1 - mod2;
      vTaskDelay(waitTime);
    }

    StartTime = xTaskGetTickCount();   
    Debug.print(DBG_DEBUG,"[PublishMeasures] start");  

    #ifdef CHRONO
    td = millis();
    #endif    
    // Store the GPIO states in one Byte (more efficient over MQTT)
    EncodeBitMap();

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(9);
        StaticJsonDocument<capacity> root;

        root["TE"]      = storage.AirTemp * 100;        // Air temperature (/!\ x100)
        root["Tmp"]     = storage.WaterSTemp * 100;     // Water temperature (/!\ x100)
        root["pH"]      = storage.PhValue * 100;        // pH value (/!\ x100)
        root["PSI"]     = storage.PSIValue * 100;       // Pressure value (/!\ x100)
        root["FLOW"]    = storage.FLOWValue;            // Main flow value
        root["FLOW2"]   = storage.FLOW2Value;           // Secondary flow value
        root["Orp"]     = storage.OrpValue;             // Orp value
        root["PhUpT"]   = PhPump.UpTime / 1000;         // pH pump uptime (seconds)
        root["ChlUpT"]  = ChlPump.UpTime / 1000;        // Chlorine pump uptime (seconds)

        PublishTopic(PoolTopicMeas1, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(5);
        StaticJsonDocument<capacity> root;

        root["AcidF"] = PhPump.GetTankFill();           // Acid tank fill level
        root["ChlF"]  = ChlPump.GetTankFill();          // Chlorine tank fill level
        root["IO"]    = BitMap1;                        // GPIO states (pumps and tank levels)
        root["IO2"]   = BitMap2;                        // GPIO states (PID modes, modes, and additional devices)
        root["IO3"]   = BitMap3;                        // Error states

        PublishTopic(PoolTopicMeas2, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
        const int capacity = JSON_OBJECT_SIZE(10);
        StaticJsonDocument<capacity> root;

        root["FiltUpT"] = FiltrationPump.UpTime / 1000;     // Filtration pump uptime (seconds)
        root["SaltUpT"] = SaltPump.UpTime / 1000;           // Salt pump uptime (seconds)
        root["HeatUpT"] = HeatPump.UpTime / 1000;           // Heat pump uptime (seconds)
        root["SolUpT"]  = SolarPump.UpTime / 1000;          // Solar pump uptime (seconds)
        root["WFUpT"]   = WaterFill.UpTime / 1000;          // Water fill uptime (seconds)
        root["WHUpT"]   = WaterHeatPump.UpTime / 1000;      // Water heat pump uptime (seconds)
        root["RobUpT"]  = RobotPump.UpTime / 1000;          // Robot pump uptime (seconds)
        root["SaltRT"]  = storage.SaltPumpRunTime / 1000;   // Salt pump configured runtime (seconds)
        root["ESPUpT"]  = storage.Uptime;                   // ESP32 uptime (hours)
        root["RstRsn"]  = resetReasonToString(esp_reset_reason()); // Reset reason

        PublishTopic(PoolTopicMeas3, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Send a JSON to MQTT broker for motor valve states
        const int capacity = JSON_OBJECT_SIZE(6);
        StaticJsonDocument<capacity> root;

        root["ELD_Tre"] = ELD_Treppe.getStatus();   // ELD_Treppe valve state
        root["ELD_Hin"] = ELD_Hinten.getStatus();   // ELD_Hinten valve state
        root["WP_Vor"]  = WP_Vorlauf.getStatus();   // WP_Vorlauf valve state
        root["WP_Mis"]  = WP_Mischer.getStatus();   // WP_Mischer valve state
        root["Boden"]   = Bodenablauf.getStatus();  // Bodenablauf valve state
        root["Solar"]   = Solarvalve.getStatus();   // Solarvalve valve state

        PublishTopic(PoolTopicMeas4, root);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    if (mqttClient.connected())
    {
        // Third MQTT publish which publishes the water temperature measured in the skimmer
        char temperatureString[6];
        dtostrf(storage.WaterSTemp, 5, 2, temperatureString);
        xSemaphoreTake(mutex, portMAX_DELAY);
        mqttClient.publish(PoolTemp, 1, false, temperatureString);
        xSemaphoreGive(mutex);
    }
    else
        Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

    // Display remaining RAM space. For debug
    Debug.print(DBG_DEBUG,"[memCheck]: %db",freeRam());
    stack_mon(hwm);     

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[PublishMeasures] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif  

    // Compute elapsed time to adjust next waiting time, taking into account a possible rollover of ticks count
    StopTime = xTaskGetTickCount();
    if(StartTime <= StopTime)
      DeltaTime = StopTime - StartTime;
    else
      DeltaTime = StopTime + (~TickType_t(0) - StartTime) + 1;

    WaitTimeOut = (TickType_t)storage.PublishPeriod/portTICK_PERIOD_MS - DeltaTime;   
  }  
}