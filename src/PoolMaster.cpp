// Supervisory task

#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"
//#include <ESP_Mail_Client.h>

static WiFiClient wificlient;

// SMTPSession smtp;
// Session_Config config;
// SMTP_Message message;

// Functions prototypes
void ProcessCommand(char*);
void StartTime(void);
void readLocalTime(void);
bool saveParam(const char*,uint8_t );
bool saveParam(const char*,bool );
bool saveParam(const char*,unsigned long );
bool saveParam(const char*,double );
void SetPhPID(bool);
void SetOrpPID(bool);
void mqttErrorPublish(const char*);
void publishSolarMode(int event);
void UpdateTFT(void);
void stack_mon(UBaseType_t&);
// void smtpCallback(SMTP_Status);
// bool SMTP_Connect(void);
// void Send_Email(void);
void Send_IFTTTNotif(void);
void calibrateMotorValves();
void setStandardMotorValvePositions();
void setStandardHeatPumpMotorValvePositions();
void setMotorValvePositionsForHeatPump();
void setMotorValvePositionsForCleanMode();


void calibrateMotorValves() {
  ELD_Treppe.calibrate();
  ELD_Hinten.calibrate();
  WP_Vorlauf.calibrate();
  WP_Mischer.calibrate();
  Bodenablauf.calibrate();
  Solarvalve.calibrate();
}

void setStandardMotorValvePositions()
{
  ELD_Treppe.open();
  ELD_Hinten.open();
  WP_Vorlauf.close();
  WP_Mischer.open();
  Bodenablauf.open();
}

void setStandardHeatPumpMotorValvePositions()
{
    if (!storage.CleanMode) {
        WP_Vorlauf.close();
        WP_Mischer.open();
    }
}

void setMotorValvePositionsForHeatPump()
{
    if (!storage.CleanMode) {
        WP_Vorlauf.open();
        WP_Mischer.close();
    }
}

void setMotorValvePositionsForCleanMode()
{
  if (storage.CleanMode && !storage.ValveSwitch)
  {
    ELD_Treppe.close();
    ELD_Hinten.open();
    WP_Vorlauf.close();
    WP_Mischer.open();
    Bodenablauf.close();
    Solarvalve.close();
  } else if (storage.CleanMode && storage.ValveSwitch)
    {
      ELD_Treppe.open();
      ELD_Hinten.close();
      WP_Vorlauf.close();
      WP_Mischer.open();
      Bodenablauf.close();
      Solarvalve.close();
    }
}

void PoolMaster(void *pvParameters)
{

  bool DoneForTheDay = false;                     // Reset actions done once per day
  bool d_calc = false;                            // Filtration duration computed
  bool cleaning_done = false;                     // daily cleaning done 

  static UBaseType_t hwm=0;                       // free stack size

/*
  MailClient.networkReconnect(true);
  #ifndef SILENT_MODE
    smtp.debug(1);
  #endif
  smtp.callback(smtpCallback);
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_LOGIN;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "127.0.0.1";
  message.sender.name = F("PoolMaster");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = F("PoolMaster Event");
  message.addRecipient(F("Home"), RECIPIENT_EMAIL);
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  message.priority = esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;
*/

  while(!startTasks);
  vTaskDelay(DT3);                                // Scheduling offset 

  esp_task_wdt_add(nullptr);
  TickType_t period = PT3;  
  TickType_t ticktime = xTaskGetTickCount(); 

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {  
    // reset watchdog
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif    

    // Handle OTA update
    ArduinoOTA.handle();

    //update pumps
    FiltrationPump.loop();
    SolarPump.loop();
    HeatPump.loop();
    WaterHeatPump.loop();
    SaltPump.loop();
    PhPump.loop();
    ChlPump.loop();
    RobotPump.loop();
    WaterFill.loop();

    //update MotorValves
    ELD_Treppe.loop();
    ELD_Hinten.loop();
    WP_Vorlauf.loop();
    WP_Mischer.loop();
    Bodenablauf.loop();
    Solarvalve.loop();

      // Debug.print(DBG_VERBOSE, "[WIFI] SSID: %s", storage.SSID.c_str());
      // Debug.print(DBG_VERBOSE, "[WIFI] PASSWORD: %s", storage.WIFI_PASS.c_str());

    //reset time counters at midnight and send sync request to time server
    if (hour() == 0 && !DoneForTheDay)
    {
        //First store current Chl and Acid and water consumptions and the runtime of the SaltPump of the day in Eeprom
        storage.AcidFill = PhPump.GetTankFill();
        storage.ChlFill = ChlPump.GetTankFill();
        saveParam("AcidFill", storage.AcidFill);
        saveParam("ChlFill", storage.ChlFill);
        saveParam("WaterFillAnCon", storage.WaterFillAnCon);
        saveParam("SaltPumpRunTime", storage.SaltPumpRunTime);
        saveParam("SaltPolarity", storage.SaltPolarity);

        FiltrationPump.ResetUpTime();
        PhPump.ResetUpTime();
        PhPump.SetTankFill(storage.AcidFill);
        ChlPump.ResetUpTime();
        ChlPump.SetTankFill(storage.ChlFill);
        RobotPump.ResetUpTime();
        SolarPump.ResetUpTime();
        SaltPump.ResetUpTime();
        HeatPump.ResetUpTime();
        WaterHeatPump.ResetUpTime();
        WaterFill.ResetUpTime();

        EmergencyStopFiltPump = false;
        d_calc = false;
        DoneForTheDay = true;
        cleaning_done = false;

        readLocalTime();
        setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year-100);

    }
    else if(hour() == 1)
    {
        DoneForTheDay = false;
    }

    // Compute next Filtering duration and start/stop hours dynamicly througt the day
    // Wait at least 5mn after filtration start in order to let the temperature stabilizes in pipes, and to avoid
    // taking into account not yet measured temperature if the system starts at 15:xx. 
    // Depending on water temperature, the filtration duration is either 2 hours, temp/3 or temp/2 hours.
    #ifdef DEBUG
    if (second() == 0 && (millis() - FiltrationPump.LastStartTime) > 300000 && !d_calc)
    #else
    if (hour() == 15 && (millis() - FiltrationPump.LastStartTime) > 300000 && !d_calc)
    #endif
    {
        if (storage.WaterSTemp < storage.WaterTempLowThreshold){
            storage.FiltrationDuration = 2;}
        else if (storage.WaterSTemp >= storage.WaterTempLowThreshold && storage.WaterSTemp < storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.WaterSTemp / 3.);}
        else if (storage.WaterSTemp >= storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.WaterSTemp / 2.);}
    
        storage.FiltrationStart = 15 - (int)round(storage.FiltrationDuration / 2.);
        if (storage.FiltrationStart < storage.FiltrationStartMin)
        storage.FiltrationStart = storage.FiltrationStartMin;    
        storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;
        if (storage.FiltrationStop > storage.FiltrationStopMax)
        storage.FiltrationStop = storage.FiltrationStopMax;

        // save actual parameter to eeprom
        saveParam("FiltrStart",storage.FiltrationStart);  
        saveParam("FiltrStop",storage.FiltrationStop);
        saveParam("SaltPumpRunTime", storage.SaltPumpRunTime);
        saveParam("SaltPolarity", storage.SaltPolarity);
        saveParam("WaterFillAnCon", storage.WaterFillAnCon);

        Debug.print(DBG_INFO,"Filtration duration: %dh",storage.FiltrationDuration);
        Debug.print(DBG_INFO,"Start: %dh - Stop: %dh",storage.FiltrationStart,storage.FiltrationStop);

        d_calc = true;
    }
    #ifdef DEBUG
    if(second() == 30 && d_calc) d_calc = false;
    #endif

    //start filtration pump as scheduled
    if (!EmergencyStopFiltPump && !FiltrationPump.IsRunning() && storage.AutoMode &&
        !PSIError && !FLOWError && hour() >= storage.FiltrationStart && hour() < storage.FiltrationStop )
        FiltrationPump.Start();

    //start cleaning robot for 2 hours, 30mn after filtration start
    if (FiltrationPump.IsRunning() && storage.AutoMode && !storage.WinterMode && !RobotPump.IsRunning() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60) >= 30 && !cleaning_done)
    {
        RobotPump.Start();
        Debug.print(DBG_INFO,"Robot Start 30mn after Filtration");    
    }
    if(RobotPump.IsRunning() && storage.AutoMode && ((millis() - RobotPump.LastStartTime) / 1000 / 60) >= ROBOT_DURATION)
    {
        RobotPump.Stop();
        cleaning_done = true;
        Debug.print(DBG_INFO,"Robot Stop after: %d mn",(int)(millis()-RobotPump.LastStartTime)/1000/60);
    }

    // ******************************************************************************************
    // WATERHEAT -> Requests Heat from the House Heatingsystem (WaterHeatPump)
    // ******************************************************************************************    

    //If water heating is desired and filtration has been running for over 5mins (so that measured water temp is accurate), open/close the HEAT_ON relay as required
    //in order to regulate the water temp. When closing the HEAT_ON relay, my house heating system switches to a fixed water temperature mode and starts the pool water
    //circulator in order to heat-up the heat exchanger located on the pool filtration water circuit
    if (storage.AutoMode && storage.WaterHeat && FiltrationPump.IsRunning())
    {
    if (FiltrationPump.UpTime / 1000 / 60 > 5)
    {
      if (storage.WaterSTemp < (storage.WaterTemp_SetPoint - 0.2))
      {
        WaterHeatPump.Start();
      }
      else if (storage.WaterSTemp > (storage.WaterTemp_SetPoint + 0.2))
      {
        WaterHeatPump.Stop();
      }
    }
    }
    else
    {
    WaterHeatPump.Stop();
    }

    // ******************************************************************************************
    // SOLAR HEATING LOCAL
    // ******************************************************************************************

    //The circulator of the pool water heating circuit needs to run regularly to avoid blocking
    //Let it run every day at noon for 2 mins
    if (storage.AutoMode && ((hour() == 12) && (minute() == 0)))
    {
    SolarPump.Start();
    Solarvalve.open();
    }

    if (storage.AutoMode && ((hour() == 12) && (minute() == 2)))
    {
    SolarPump.Stop();
    Solarvalve.close();
    }

    //If solar heating (SolarLocExt) is set to "local" and solar mode is set to "auto" mode and filtration has been running for over 5mins (so that measured water temp is accurate), open/close the solarvalve as required
    //in order to regulate the water temp.    
    if (!storage.SolarLocExt && storage.SolarMode && FiltrationPump.IsRunning() && 
        FiltrationPump.UpTime / 1000 / 60 > 5 && 
        hour() >= storage.SolarStartMin && hour() < storage.SolarStopMax) // Check if it's within time range to activate solar heating
    {
        // Check if the temperature difference is large enough to turn on solar pump and valve
        if (storage.WaterSTemp < storage.WaterTemp_SetPoint && storage.SolarTemp > storage.WaterSTemp + 4)
        {
            SolarPump.Start();
            Solarvalve.open();
        }
        else
        {
            SolarPump.Stop();
            Solarvalve.close();
        }
    }

    // ******************************************************************************************
    // SOLAR HEATING EXTERNAL (MQTT)
    // ******************************************************************************************

    //If solar heating (SolarLocExt) is set to "external" and solar mode is set to "auto" mode and filtration has been running for over 5mins (so that measured water temp is accurate), open/close the solarvalve as required
    //in order to regulate the water temp.    
    if (storage.AutoMode && storage.SolarLocExt && storage.SolarMode && FiltrationPump.IsRunning() && 
        FiltrationPump.UpTime / 1000 / 60 > 5 && 
        hour() >= storage.SolarStartMin && hour() < storage.SolarStopMax) // Check if it's within time range to activate solar heating
    {
        // Check if the temperature difference is large enough to turn on solar pump and valve
        if (storage.WaterSTemp < storage.WaterTemp_SetPoint && storage.SolarTemp > storage.WaterSTemp + 4)
        {
            SolarPump.Start();
            publishSolarMode(1);
        }
        else if (storage.WaterSTemp >= storage.WaterTemp_SetPoint || storage.SolarRLTemp + 2 <= storage.WaterSTemp)
        {
            SolarPump.Stop();
            publishSolarMode(2);
        }
    } else
    {
      publishSolarMode(3);
    }

    if (storage.SolarLocExt)
    {
      if (storage.AutoMode)
      {
            publishPoolMode(1);
      }
      else
      {
            publishPoolMode(3);
      }
    }
    else if (!storage.AutoMode && SolarPump.IsRunning())
    {
      publishPoolMode(2);
    }
    else
    {
      publishPoolMode(3);
    }

    // ******************************************************************************************
    // Manage motor valves
    // ******************************************************************************************

    //The MotorValves should be calibrated daily, to do this we start calibration at 5 o´clock in the morning.
    if ((hour() == 5) && (minute() == 0))
    {
    calibrateMotorValves();
    }

    //After calibration we set the valves in to a standard position every morning.
    if ((hour() == 5) && (minute() == 3))
    {
    setStandardMotorValvePositions();
    }

    // Manage motor valves modes

    bool timerStarted = false;
    unsigned long timerStartTime;

    if (storage.ValveMode && FiltrationPump.IsRunning())
    {
        // Manage CleanMode Valves
        if (storage.CleanMode)
        {
            setMotorValvePositionsForCleanMode();
        } 
        else 
        {
            // Manage Heatpump Valves
            if (HeatPump.IsRunning()) 
            {
                setMotorValvePositionsForHeatPump();
            } 
            else 
            {
                setStandardHeatPumpMotorValvePositions();
            }

            // Manage WaterHeat Valves
            if (storage.WaterHeat)
            {
                Solarvalve.open();
            } 
            else 
            {
                Solarvalve.close();
            }
        }

        // Start or reset timer for CleanMode or ValveSwitch
        if (storage.CleanMode || storage.ValveSwitch)
        {
            if (!timerStarted)
            {
                timerStarted = true;
                timerStartTime = millis();
            }
            else if (millis() - timerStartTime >= 1800000) // 30 minutes in milliseconds
            {
                storage.CleanMode = 0;
                storage.ValveSwitch = 0;
                timerStarted = false;
            }
        }
        else
        {
            timerStarted = false;
        }
    } 
    if (storage.ValveMode && !FiltrationPump.IsRunning() && !HeatPump.IsRunning())
    {
        setStandardMotorValvePositions();

        // Reset CleanMode and ValveSwitch
        if (storage.CleanMode || storage.ValveSwitch)
        {
            storage.CleanMode = 0;
            storage.ValveSwitch = 0;
        }

        timerStarted = false;
    }
    if (storage.ValveMode && !storage.CleanMode && !HeatPump.IsRunning())
    {
        setStandardMotorValvePositions();
        timerStarted = false;
    }
    else
    {
        timerStarted = false;
    }

    // ******************************************************************************************
    // Mange WaterLevel and WaterFillMode
    // ******************************************************************************************
    unsigned long lastUpTime = 0;
    float waterConsumption = 0.0;
    float flowRate = storage.WaterFillFR;

    bool waterMaxLvl = digitalRead(WATER_MAX_LVL) == 0; // switch is open, Water Level is above max level
    bool waterMinLvl = digitalRead(WATER_MIN_LVL) == 0; // switch is open, Water level is ok
    Debug.print(DBG_VERBOSE, "[WaterFill] waterMaxLvl = %d", waterMaxLvl);
    Debug.print(DBG_VERBOSE, "[WaterFill] waterMinLvl = %d", waterMinLvl);
    Debug.print(DBG_ERROR, "[WaterFill] WaterFill Status: WaterFillDuration: %d ms and WaterFillUpTimeLimit: %d ms", WaterFill.UpTime, storage.WaterFillUpTimeLimit);
    unsigned long levelMinHighDelay = 1; // defines the delay until the water valve opens if min level switch is reached, in minutes

    static unsigned long LastWaterFillStartTime = 0;
    static unsigned long LastWaterFillStopTime = 0;
    static unsigned long timeSinceMinLvl = 0; // Global variable to store time since last fill stop

    // Stopp WaterFill valve in case WaterFill Error ist true
    if (WaterFillError && WaterFill.IsRunning()) {
        storage.WaterFillMode = 0;
        WaterFill.Stop();
        Debug.print(DBG_ERROR, "[WaterFill] WaterFill stopped. WaterFillError is >true<");
    }

    // Stopp WaterFill if WaterFill Duration reaches MaxUpTime
    if (WaterFill.UpTime >= storage.WaterFillUpTimeLimit) {
        storage.WaterFillMode = 0;
        WaterFill.Stop();
        WaterFillError = true;
        Debug.print(DBG_ERROR, "[WaterFill] WaterFill stopped. MaxUpTime is reached: WaterFillDuration: %d ms and WaterFillUpTimeLimit: %d ms", WaterFill.UpTime, storage.WaterFillUpTimeLimit);
    }

    // Check waterMinLvl and Timestamp since last MinLevel
    if (waterMinLvl && waterMaxLvl && timeSinceMinLvl == 0) {
        timeSinceMinLvl = millis();
    }

    if (storage.WaterFillMode && FiltrationPump.IsRunning()) { // Automatic mode
        if (waterMinLvl && waterMaxLvl && timeSinceMinLvl != 0 && !WaterFill.IsRunning() && !WaterFillError) {
            if (millis() - timeSinceMinLvl >= levelMinHighDelay * 60 * 1000) { // Check if the delay has been reached
                WaterFill.Start();
                Debug.print(DBG_VERBOSE, "[WaterFill] Starting WaterFill...");
            }
        }

        if (!waterMinLvl && waterMaxLvl && timeSinceMinLvl != 0 && LastWaterFillStartTime ==0) {
            LastWaterFillStartTime = millis();
        }            

        if (!waterMaxLvl && WaterFill.IsRunning()) { // Water level just reached maximum, stop timing the fill process
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill...");
            WaterFill.Stop();
            timeSinceMinLvl = 0;
            LastWaterFillStopTime = millis();
            unsigned long fillDuration = (round((LastWaterFillStopTime - LastWaterFillStartTime) / 60000.0) * 60000) - 1; // round up to the nearest minute
            storage.WaterFillDuration = fillDuration;
            Debug.print(DBG_VERBOSE, "[WaterFill] WaterFill stopped. Fill duration: %d ms", fillDuration);
            LastWaterFillStartTime = 0; // Reset the start time
            }

        if (!waterMinLvl && storage.WaterFillDuration != 0 && (storage.WaterFillDuration <= millis() - LastWaterFillStartTime) && WaterFill.IsRunning() && !WaterFillError) { // Water level reached waterFillduration, stop the fill process
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill bevor reaching max level");
            WaterFill.Stop();
            timeSinceMinLvl = 0;
            LastWaterFillStartTime = 0; // Reset the start time
        }        
    }
    else { // Manual mode
        if (!waterMaxLvl && WaterFill.IsRunning() || WaterFillError && WaterFill.IsRunning())
        { // Water level reached maximum, stop water filling
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill in Manual mode...");
            WaterFill.Stop();
            LastWaterFillStopTime = millis();
        }
    }

    // Calculate the water consumtion sice the last reset of the anual consumtion
    if (WaterFill.UpTime != lastUpTime) { // check if WaterFill.UpTime has changed        
        unsigned long fillDur = WaterFill.UpTime - lastUpTime;
        Debug.print(DBG_ERROR, "[WaterFill] TimeCalc: fillDuration: %d ms and lastUpTime: %d ms", fillDur, lastUpTime);
        waterConsumption = flowRate * fillDur / 60000.0; // calculate water consumption in liters
        Debug.print(DBG_ERROR, "[WaterFill] Flowrate * fillDuration / 60000: FlowRate: %s and fillDuration: %d ms and lastUpTime: %d ms", flowRate, fillDur, lastUpTime);
        storage.WaterFillAnCon += waterConsumption; // add to annual water consumption
        lastUpTime = WaterFill.UpTime;
    }

    // ******************************************************************************************
    // Manage PH and Chlor PIDS
    // ******************************************************************************************

    // start PIDs with delay after FiltrationStart in order to let the readings stabilize
    // start inhibited if water temperature below threshold and/or in winter mode
    if (FiltrationPump.IsRunning() && storage.AutoMode && !FLOW2Error && !storage.WinterMode && !PhPID.GetMode() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60 >= storage.DelayPIDs) &&
        (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop) &&
        storage.WaterSTemp >= storage.WaterTempLowThreshold)
    {
        //Start PIDs
        SetPhPID(true);
        SetOrpPID(true);
    } 

    //stop filtration pump and PIDs as scheduled unless we are in AntiFreeze mode
    if (storage.AutoMode && FiltrationPump.IsRunning() && !AntiFreezeFiltering && (hour() >= storage.FiltrationStop || hour() < storage.FiltrationStart))
    {
        SetPhPID(false);
        SetOrpPID(false);
        FiltrationPump.Stop();
    }

    //Outside regular filtration hours, start filtration in case of cold Air temperatures (<-2.0deg)
    if (!EmergencyStopFiltPump && storage.AutoMode && !PSIError && !FLOWError && !FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && (storage.AirTemp < -2.0))
    {
        FiltrationPump.Start();
        AntiFreezeFiltering = true;
    }

    //Outside regular filtration hours and if in AntiFreezeFiltering mode but Air temperature rose back above 2.0deg, stop filtration
    if (storage.AutoMode && FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && AntiFreezeFiltering && (storage.AirTemp > 2.0))
    {
        FiltrationPump.Stop();
        AntiFreezeFiltering = false;
    }

    //If filtration pump has been running for over 45secs but pressure is still low, stop the filtration pump, something is wrong, set error flag
    if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 45000) && (storage.PSIValue < storage.PSI_MedThreshold))
    {
        FiltrationPump.Stop();
        PSIError = true;
        mqttErrorPublish("{\"PSI Error\":1}");
    }

    //If filtration pump has been running for over 40secs but flow in Main-Pipe is still low, stop the filtration pump, something is wrong, set error flag
    if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 40000) && (storage.FLOWValue < storage.FLOW_MedThreshold))
    {
        FiltrationPump.Stop();
        SaltPump.Stop();
        FLOWError = true;
        mqttErrorPublish("{\"FLOW Error\":1}");
    }

    //If filtration pump has been running for over 60secs but flow in Meassure-Pipe is still low, something is wrong, set error flag
    if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 60000) && (storage.FLOW2Value < storage.FLOW2_MedThreshold))
    {
        SetPhPID(false);
        SetOrpPID(false);
        SaltPump.Stop();
        FLOW2Error = true;
        mqttErrorPublish("{\"FLOW2 Error\":1}");
    }

    // Over-pressure error
    if (storage.PSIValue > storage.PSI_HighThreshold)
    {
        FiltrationPump.Stop();
        PSIError = true;
        mqttErrorPublish("{\"PSI Error\":1}");
    } else if(storage.PSIValue >= storage.PSI_MedThreshold)
        PSIError = false;

        // HighFlow-Rate error
    if (storage.FLOWValue > storage.FLOW_HighThreshold)
    {
        FLOWError = true;
        mqttErrorPublish("{\"FLOW Error\":1}");
    } else if(storage.FLOWValue >= storage.FLOW_MedThreshold)
        FLOWError = false;

    // HighFlow2-Rate error
    if (storage.FLOW2Value > storage.FLOW2_HighThreshold)
    {
        FLOW2Error = true;
        mqttErrorPublish("{\"FLOW2 Error\":1}");
    } else if(storage.FLOW2Value >= storage.FLOW2_MedThreshold)
        FLOW2Error = false;

    // WaterFill error
    if (storage.WaterFillDuration > storage.WaterFillUpTimeLimit || waterMaxLvl == 0 && waterMinLvl == 1)
    {
        WaterFillError = true;
        mqttErrorPublish("{\"WATERFILL Error\":1}");
    } else if(storage.WaterFillDuration >= storage.WaterFillUpTimeLimit)
        WaterFillError = false;
        
    //UPdate Nextion TFT
    UpdateTFT();

    //Send IFTTT notifications if alarms occured
    Send_IFTTTNotif();

    //Send email if alarm(s) occured
    //Send_Email();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[PoolMaster] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif 

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }
}

//Enable/Disable pH PID
void SetPhPID(bool Enable)
{
  if (Enable)
  {
    //Start PhPID
    PhPump.ClearErrors();
    storage.PhPIDOutput = 0.0;
    storage.PhPIDwindowStartTime = millis();
    PhPID.SetMode(AUTOMATIC);
    storage.Ph_RegulationOnOff = 1;
  }
  else
  {
    //Stop PhPID
    PhPID.SetMode(MANUAL);
    storage.Ph_RegulationOnOff = 0;
    storage.PhPIDOutput = 0.0;
    PhPump.Stop();
  }
}

//Enable/Disable Orp PID
void SetOrpPID(bool Enable)
{
  if (Enable)
  {
    //Start OrpPID
    ChlPump.ClearErrors();
    storage.OrpPIDOutput = 0.0;
    storage.OrpPIDwindowStartTime = millis();
    OrpPID.SetMode(AUTOMATIC);
    storage.Orp_RegulationOnOff = 1;

  }
  else
  {
    //Stop OrpPID
    OrpPID.SetMode(MANUAL);
    storage.Orp_RegulationOnOff = 0;
    storage.OrpPIDOutput = 0.0;
    ChlPump.Stop();
  }
}

//Send notifications to IFTTT applet in case of alarm
void Send_IFTTTNotif(){
    static const String url1 = IFTTT_key;
    String url2 = "";
    static bool notif_sent[5] = {0,0,0,0,0};

    if(PSIError)
    {
        if(!notif_sent[0])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Water%20pressure&value2=");
                if(storage.PSIValue <= storage.PSI_MedThreshold)
                {
                    url2 += String("Low");
                } 
                else if (storage.PSIValue >= storage.PSI_HighThreshold)
                {
                    url2 += String("High");
                }
                url2 += String("%20pressure:%20") + String(storage.PSIValue) + String("bar");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[0] = true;
            }
        }    
    } else notif_sent[0] = false;

    if(FLOWError)
    {
        if(!notif_sent[1])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Water%20flow&value2=");
                if(storage.FLOWValue <= storage.FLOW_MedThreshold)
                {
                    url2 += String("Low");
                } 
                else if (storage.FLOWValue >= storage.FLOW_HighThreshold)
                {
                    url2 += String("High");
                }
                url2 += String("%20flow:%20") + String(storage.FLOWValue) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[1] = true;
            }
        }    
    } else notif_sent[1] = false;

    if(FLOW2Error)
    {
        if(!notif_sent[2])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Water%20flow2&value2=");
                if(storage.FLOW2Value <= storage.FLOW2_MedThreshold)
                {
                    url2 += String("Low");
                } 
                else if (storage.FLOW2Value >= storage.FLOW2_HighThreshold)
                {
                    url2 += String("High");
                }
                url2 += String("%20flow2:%20") + String(storage.FLOW2Value) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[2] = true;
            }
        }    
    } else notif_sent[2] = false;

    if(!ChlPump.TankLevel())
    {
        if(!notif_sent[3])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Chl%20level&value2=") + String(ChlPump.GetTankFill()) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[3] = true;
            }
        }
    } else notif_sent[3] = false;

    if(!PhPump.TankLevel())
    {
        if(!notif_sent[4])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=pH+%20level&value2=") + String(PhPump.GetTankFill()) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[4] = true;
            }
        }
    } else notif_sent[4] = false;

    if(ChlPump.UpTimeError)
    {
        if(!notif_sent[5])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Chl%20pump%20uptime&value2=") + String(round(ChlPump.UpTime/60000.)) + String("min");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[5] = true;
            }
        }
    } else notif_sent[5] = false;

    if(PhPump.UpTimeError)
    {
        if(!notif_sent[6])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=pH+%20pump%20uptime&value2=") + String(round(PhPump.UpTime/60000.)) + String("min");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[6] = true;
            }
        }
    } else notif_sent[6] = false;

    if(WaterFill.UpTimeError)
    {
        if(!notif_sent[7])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=water%20fill%20uptime&value2=") + String(round(WaterFill.UpTime/60000.)) + String("min");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[7] = true;
            }
        }
    } else notif_sent[7] = false;    
}

/*
bool SMTP_Connect(){
  Debug.print(DBG_DEBUG,"SMTP Connection starts");
  if (!smtp.connect(&config)){
    Debug.print(DBG_ERROR,"SMTP Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return false;
  } else Debug.print(DBG_INFO,"SMTP Connected");
  if (!smtp.isLoggedIn()) Debug.print(DBG_ERROR,"Not yet logged in.");
  else{
    if (smtp.isAuthenticated()) Debug.print(DBG_INFO,"SMTP Successfully logged in.");
    else Debug.print(DBG_ERROR,"SMTP Connected with no Auth.");
  }
  return true;
}
void Send_Email(){
    char texte[80];
    static bool notif_sent[5] = {0,0,0,0,0};

    if(PSIError)
    {
      if(!notif_sent[0])
      {
        sprintf(texte,"Water pressure alert: %4.2fbar",storage.PSIValue);
        message.text.content = texte;
        if(SMTP_Connect()){   
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());     
          else notif_sent[0] = true;
        }
      }    
    } else notif_sent[0] = false;

    if(!ChlPump.TankLevel())
    {
      if(!notif_sent[1])
      {
        sprintf(texte,"Chlorine level LOW: %3.0f %",ChlPump.GetTankFill());
        message.text.content = texte;
        if(SMTP_Connect()){
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());    
          else notif_sent[1] = true;
        }  
      }
    } else notif_sent[1] = false;

    if(!PhPump.TankLevel())
    {
      if(!notif_sent[2])
      {
        sprintf(texte,"Acid level LOW: %3.0f %",PhPump.GetTankFill());
        message.text.content = texte;
        if(SMTP_Connect()){ 
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());    
          else notif_sent[2] = true;
        }  
      }
    } else notif_sent[2] = false;

    if(ChlPump.UpTimeError)
    {
      if(!notif_sent[3])
      {
        sprintf(texte,"Chlorine pump uptime: %2.0fmn",round(ChlPump.UpTime/60000.));
        message.text.content = texte; 
        if(SMTP_Connect()){       
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());    
          else notif_sent[3] = true;
        }  
      }
    } else notif_sent[3] = false;

    if(PhPump.UpTimeError)
    {
      if(!notif_sent[4])
      {
        sprintf(texte,"Acid pump uptime: %2.0fmn",round(PhPump.UpTime/60000.));
        message.text.content = texte;
        if(SMTP_Connect()){
          if(!MailClient.sendMail(&smtp, &message))
            Debug.print(DBG_ERROR,"Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());    
          else notif_sent[4] = true;
        }  
      }
    } else notif_sent[4] = false; 
}

// Callback function to get the Email sending status
void smtpCallback(SMTP_Status status){
  // Print the current status
  Debug.print(DBG_INFO,"Email send status: %d",status.info());
  // Print the sending result
  if (status.success()){
    Debug.print(DBG_INFO,"Message sent success: %d", status.completedCount());
    Debug.print(DBG_INFO,"Message sent failed: %d", status.failedCount());
    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      // Get the result item
      SMTP_Result result = smtp.sendingResult.getItem(i);
      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      
      Debug.print(DBG_INFO,"Message No: %d", i + 1);
      Debug.print(DBG_INFO,"Status: %s", result.completed ? "success" : "failed");
      Debug.print(DBG_INFO,"Date/Time: %s", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      Debug.print(DBG_INFO,"Recipient: %s", result.recipients.c_str());
      Debug.print(DBG_INFO,"Subject: %s", result.subject.c_str());
    }
    // Clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}
*/

