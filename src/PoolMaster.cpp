// Supervisory task

#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"
//#include <ESP_Mail_Client.h>

static WiFiClient wificlient;

extern Arduino_DebugUtils Debug;

// SMTPSession smtp;
// Session_Config config;
// SMTP_Message message;

// Functions prototypes
void ProcessCommand(char*);
void StartTime(void);
void readLocalTime(void);
bool saveParam(const char* key, uint8_t val);
bool saveParam(const char* key, bool val);
bool saveParam(const char* key, unsigned long val);
bool saveParam(const char* key, String val);
bool saveParam(const char* key, const uint8_t* val, size_t size);
bool saveParam(const char* key, double val);
void SetPhPID(bool);
void SetOrpPID(bool);
void mqttErrorPublish(const char*);
void publishSolarMode(int event);
void UpdateTFT(void);
void stack_mon(UBaseType_t&);
void Send_IFTTTNotif(void);
void calibrateMotorValves();
void setStandardMotorValvePositions();
void setStandardHeatPumpMotorValvePositions();
void setMotorValvePositionsForHeatPump();
void setMotorValvePositionsForCleanMode();
// BUG FIX: removed incorrect "void getDurationSafe();" prototype — the actual
// function has signature unsigned long getDurationSafe(unsigned long, unsigned long)
// and is already declared in PoolMaster.h.
const char* resetReasonToString(uint8_t reason);
// void smtpCallback(SMTP_Status);
// bool SMTP_Connect(void);
// void Send_Email(void);

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

unsigned long getDurationSafe(unsigned long start, unsigned long current) {
    if (current < start) {
        // Handle millis() overflow
        return (ULONG_MAX - start) + current + 1;
    }
    return current - start;
}

void PoolMaster(void *pvParameters)
{

  bool DoneForTheDay = false;                     // Reset actions done once per day
  bool d_calc = false;                            // Filtration duration computed
  // BUG FIX: Do NOT redeclare cleaning_done here — use the global extern defined
  // in Setup.cpp and declared in PoolMaster.h.  The old local variable shadowed
  // the global, so MQTT commands from PoolServer (cleaning_done = true) had no
  // effect on the PoolMaster loop.

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
Debug.print(DBG_INFO, "[TASKS] PoolMaster started on core %d", xPortGetCoreID());
  while(!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] PoolMaster running...");
  vTaskDelay(DT3);                                // Scheduling offset 

  esp_task_wdt_add(NULL);
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

    // Uptime-Berechnung (alle 5 Minuten)
        static unsigned long lastUptimeSave = 0U;
        if (millis() - lastUptimeSave >= 300000U) { // 5 Minuten
            unsigned long currentMillis = millis();
            storage.Uptime += getDurationSafe(storage.LastUptimeUpdate, currentMillis) / 3600000U; // Stunden
            storage.LastUptimeUpdate = currentMillis;
            saveParam("Uptime", storage.Uptime);
            saveParam("LastUptimeUpdate", storage.LastUptimeUpdate);
            lastUptimeSave = currentMillis;
            Debug.print(DBG_INFO, "[Uptime] Updated: %lu hours", storage.Uptime);
        }

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

    //start cleaning robot after ROBOT_DELAY minutes after filtration start
    // BUG FIX: was hardcoded 30 min; Config.h defines ROBOT_DELAY = 60 (minutes)
    if (FiltrationPump.IsRunning() && storage.AutoMode && !storage.WinterMode && !RobotPump.IsRunning() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60) >= ROBOT_DELAY && !cleaning_done)
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
            //SolarPump.Start();
            publishSolarMode(1);
        }
        else if (storage.WaterSTemp >= storage.WaterTemp_SetPoint || storage.SolarRLTemp + 2 <= storage.WaterSTemp)
        {
            //SolarPump.Stop();
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
    // BUG FIX: timerStarted/timerStartTime must be static so the 30-minute
    // CleanMode/ValveSwitch timeout actually accumulates across task iterations.
    static bool timerStarted = false;
    static unsigned long timerStartTime = 0;

    if (storage.ValveMode && FiltrationPump.IsRunning())
    {
        // Manage CleanMode Valves (highest priority — overrides heat pump positions)
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

            // Manage WaterHeat Valves (Solar bypass)
            if (storage.WaterHeat)
            {
                Solarvalve.open();
            }
            else
            {
                Solarvalve.close();
            }
        }

        // 30-minute auto-reset timer for CleanMode / ValveSwitch
        if (storage.CleanMode || storage.ValveSwitch)
        {
            if (!timerStarted)
            {
                timerStarted = true;
                timerStartTime = millis();
            }
            else if (millis() - timerStartTime >= 1800000UL) // 30 minutes
            {
                storage.CleanMode = 0;
                storage.ValveSwitch = 0;
                timerStarted = false;
                Debug.print(DBG_INFO, "[ValveTimer] CleanMode/ValveSwitch auto-reset after 30 min");
            }
        }
        else
        {
            timerStarted = false;
        }
    }
    else if (storage.ValveMode && !FiltrationPump.IsRunning())
    {
        // Pump stopped: return to standard positions and cancel active modes
        setStandardMotorValvePositions();
        if (storage.CleanMode || storage.ValveSwitch)
        {
            storage.CleanMode = 0;
            storage.ValveSwitch = 0;
            Debug.print(DBG_INFO, "[ValveTimer] CleanMode/ValveSwitch cleared: filtration stopped");
        }
        timerStarted = false;
    }

    // ******************************************************************************************
    // Manage WaterLevel and WaterFillMode
    // ******************************************************************************************
    // BUG FIX: lastUpTime must be static so WaterFill consumption is only accumulated
    // for the NEW runtime since the previous task iteration, not the total UpTime every 500ms.
    static unsigned long lastUpTime = 0;
    float waterConsumption = 0.0;
    float flowRate = storage.WaterFillFR;

    static bool lastWaterMaxLvl = false; // Store previous state of waterMaxLvl
    static bool lastWaterMinLvl = false; // Store previous state of waterMinLvl
    static bool lastWaterFillRunning = false; // Store previous state of WaterFill.IsRunning()
    static bool upTimeWarningReported = false; // Track if unexpected UpTime warning was reported

    bool waterMaxLvl = digitalRead(WATER_MAX_LVL) == 1; // switch is open (HIGH), Water Level is above max level
    bool waterMinLvl = digitalRead(WATER_MIN_LVL) == 1; // switch is open (HIGH), Water level is ok
    Debug.print(DBG_VERBOSE, "[WaterFill] waterMaxLvl = %d (Max Level %s), waterMinLvl = %d (Min Level %s)",
                waterMaxLvl, waterMaxLvl ? "Above Max" : "Below Max",
                waterMinLvl, waterMinLvl ? "OK" : "Below Min");

    // Log WaterFill status only on state changes
    if (waterMaxLvl != lastWaterMaxLvl || waterMinLvl != lastWaterMinLvl || 
        WaterFill.IsRunning() != lastWaterFillRunning || WaterFill.UpTime != lastUpTime) {
        Debug.print(DBG_INFO, "[WaterFill] WaterFill Status: WaterFillDuration: %lu ms and WaterFillUpTimeLimit: %lu ms", 
                    WaterFill.UpTime, storage.WaterFillUpTimeLimit);
    }

    unsigned long levelMinHighDelay = 1; // defines the delay until the water valve opens if min level switch is reached, in minutes

    static unsigned long LastWaterFillStartTime = 0;
    static unsigned long LastWaterFillStopTime = 0;
    static unsigned long timeSinceMinLvl = 0; // Global variable to store time since last fill stop

    // Stop WaterFill valve if WaterFillError is true
    if (WaterFillError && WaterFill.IsRunning()) {
        storage.WaterFillMode = 0;
        WaterFill.Stop();
        Debug.print(DBG_ERROR, "[WaterFill] WaterFill stopped. WaterFillError is >true<");
    }

    // Stop WaterFill if WaterFill Duration reaches MaxUpTime
    if (WaterFill.UpTime >= storage.WaterFillUpTimeLimit) {
        storage.WaterFillMode = 0;
        WaterFill.Stop();
        WaterFillError = true;
        Debug.print(DBG_ERROR, "[WaterFill] WaterFill stopped. MaxUpTime is reached: WaterFillDuration: %lu ms and WaterFillUpTimeLimit: %lu ms", WaterFill.UpTime, storage.WaterFillUpTimeLimit);
        char errorMsg[100];
        snprintf(errorMsg, sizeof(errorMsg), "{\"error\":\"WaterFill stopped due to MaxUpTime reached\",\"Duration\":%lu,\"Limit\":%lu}", 
                WaterFill.UpTime, storage.WaterFillUpTimeLimit);
        mqttErrorPublish(errorMsg);
    }

    // Check waterMinLvl and Timestamp since last MinLevel
    if (!waterMinLvl && !waterMaxLvl && timeSinceMinLvl == 0) {
        timeSinceMinLvl = millis();
    }

    if (storage.WaterFillMode && FiltrationPump.IsRunning()) { // Automatic mode
        if (!waterMinLvl && !waterMaxLvl && timeSinceMinLvl != 0 && !WaterFill.IsRunning() && !WaterFillError) {
            if (millis() - timeSinceMinLvl >= levelMinHighDelay * 60 * 1000) { // Check if the delay has been reached
                WaterFill.Start();
                Debug.print(DBG_VERBOSE, "[WaterFill] Starting WaterFill...");
            }
        }

        if (!waterMinLvl && !waterMaxLvl && timeSinceMinLvl != 0 && LastWaterFillStartTime == 0) {
            LastWaterFillStartTime = millis();
            Debug.print(DBG_VERBOSE, "[WaterFill] Started timing fill process at: %lu", LastWaterFillStartTime);
        }            

        if (waterMaxLvl && WaterFill.IsRunning()) {
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill...");
            WaterFill.Stop();
            timeSinceMinLvl = 0;
            LastWaterFillStopTime = millis();
            
            if (LastWaterFillStartTime > 0) {  // Only calculate if we have a valid start time
                unsigned long fillDuration = getDurationSafe(LastWaterFillStartTime, LastWaterFillStopTime);
                storage.WaterFillDuration = fillDuration;
                Debug.print(DBG_VERBOSE, "[WaterFill] Fill duration: %lu ms", fillDuration);

                // Calculate water consumption in liters
                float fillDurationMinutes = fillDuration / 60000.0; // Convert milliseconds to minutes
                waterConsumption = flowRate * fillDurationMinutes; // flowRate in liters per minute
                storage.WaterFillAnCon += waterConsumption; // Accumulate annual consumption in liters
                Debug.print(DBG_VERBOSE, "[WaterFill] Added consumption: %.2f L (duration: %lu ms)", waterConsumption, fillDuration);
            }
            LastWaterFillStartTime = 0;
        }

        if (!waterMinLvl && storage.WaterFillDuration != 0 && (storage.WaterFillDuration <= millis() - LastWaterFillStartTime) && WaterFill.IsRunning() && !WaterFillError) { // Water level reached waterFillDuration, stop the fill process
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill before reaching max level");
            WaterFill.Stop();
            timeSinceMinLvl = 0;
            LastWaterFillStartTime = 0; // Reset the start time
        }        
    }
    else { // Manual mode
        if (waterMaxLvl && WaterFill.IsRunning() || WaterFillError && WaterFill.IsRunning()) { // Water level reached maximum, stop water filling
            Debug.print(DBG_VERBOSE, "[WaterFill] Stopping WaterFill in Manual mode...");
            WaterFill.Stop();
            LastWaterFillStopTime = millis();
        }
    }

    // Calculate the water consumption since the last reset of the annual consumption
    if (WaterFill.IsRunning() && WaterFill.UpTime != lastUpTime) { // Only calculate if valve is running and UpTime has changed        
        unsigned long fillDur = getDurationSafe(lastUpTime, WaterFill.UpTime);
        Debug.print(DBG_VERBOSE, "[WaterFill] TimeCalc: fillDuration: %lu ms, lastUpTime: %lu ms, ValveRunning: %d", 
                    fillDur, lastUpTime, WaterFill.IsRunning());
        
        // Calculate water consumption in liters
        float fillDurMinutes = fillDur / 60000.0; // Convert milliseconds to minutes
        waterConsumption = flowRate * fillDurMinutes; // flowRate in liters per minute
        storage.WaterFillAnCon += waterConsumption; // Add to annual water consumption in liters
        Debug.print(DBG_VERBOSE, "[WaterFill] Added consumption: %.2f L (duration: %lu ms)", waterConsumption, fillDur);
        
        lastUpTime = WaterFill.UpTime;
    }

    // Always update lastUpTime to prevent repeated status logs
    if (WaterFill.UpTime != lastUpTime && !upTimeWarningReported) {
        Debug.print(DBG_WARNING, "[WaterFill] UpTime changed unexpectedly: %lu ms (last: %lu ms), ValveRunning: %d",
                    WaterFill.UpTime, lastUpTime, WaterFill.IsRunning());
        upTimeWarningReported = true; // Prevent repeated warnings
    }
    lastUpTime = WaterFill.UpTime; // Update lastUpTime to sync with UpTime

    // WaterFill error handling with debouncing
    static bool waterFillErrorReported = false;
    static unsigned long lastErrorReportTime = 0;
    static const unsigned long ERROR_REPORT_INTERVAL = 600000; // 10 Minuten
    static unsigned long invalidStateStartTime = 0;
    static const unsigned long DEBOUNCE_THRESHOLD = 10000; // 10 Sekunden Entprellzeit

    if (storage.WaterFillDuration > storage.WaterFillUpTimeLimit || (waterMaxLvl && !waterMinLvl)) {
        if (invalidStateStartTime == 0) {
            invalidStateStartTime = millis();
            Debug.print(DBG_VERBOSE, "[WaterFill] Invalid state detected (waterMaxLvl=true, waterMinLvl=false), starting debounce timer...");
        } else if (millis() - invalidStateStartTime >= DEBOUNCE_THRESHOLD) {
            WaterFillError = true;
            if (!waterFillErrorReported || (millis() - lastErrorReportTime >= ERROR_REPORT_INTERVAL)) {
                Debug.print(DBG_ERROR, "[WaterFill] WaterFill Error: Invalid state persisted for %lu ms (waterMaxLvl=%d, waterMinLvl=%d)",
                            DEBOUNCE_THRESHOLD, waterMaxLvl, waterMinLvl);
                mqttErrorPublish("{\"WATERFILL Error\":1}");
                waterFillErrorReported = true;
                lastErrorReportTime = millis();
            }
        }
    } else {
        WaterFillError = false;
        waterFillErrorReported = false;
        invalidStateStartTime = 0; // Reset debounce timer
        Debug.print(DBG_VERBOSE, "[WaterFill] No invalid state, resetting debounce timer");
    }

    // Update last states for next iteration
    lastWaterMaxLvl = waterMaxLvl;
    lastWaterMinLvl = waterMinLvl;
    lastWaterFillRunning = WaterFill.IsRunning();

    // *******************************************************************************************
    // Manage SaltConcentration and SaltPump
    // *******************************************************************************************
    static unsigned long lastSaltMeasurement = 0;
    const unsigned long saltMeasurementInterval = 3600000; // 1 Stunde in ms

    // Check if measurement is needed: Hourly or if SaltConcentration is invalid
    bool isHourlyMeasurement = (minute() == 0 && millis() - lastSaltMeasurement >= saltMeasurementInterval);
    bool isInitialMeasurement = (storage.SaltConcentration <= 0.0);

    if ((isHourlyMeasurement || isInitialMeasurement) &&
        SaltPump.IsRunning() && FiltrationPump.IsRunning() &&
        storage.SaltCurrentValue > 0.0 && storage.SaltCurrentValue <= 9.0 &&
        storage.WaterSTemp >= 10.0 && storage.WaterSTemp <= 40.0) {
        // Calculate conductivity (in S/m)
        float conductivity = storage.SaltCurrentValue / (ELECTROLYSIS_VOLTAGE * storage.CellConstant);
        // Convert to mS/cm (1 S/m = 10 mS/cm)
        conductivity *= 10.0;
        // Temperature correction to 25°C
        float tempCorrectedConductivity = conductivity / (1.0 + 0.02 * (storage.WaterSTemp - 25.0));
        // Calculate salt concentration (in g/L)
        float saltConcentration = tempCorrectedConductivity / 0.18;

        // Store salt concentration
        storage.SaltConcentration = saltConcentration;
        saveParam("SaltConcentration", storage.SaltConcentration);

        Debug.print(DBG_DEBUG, "[Salt] Conductivity: %.2f mS/cm, TempCorrected: %.2f mS/cm, Concentration: %.2f g/L",
                    conductivity * 10.0, tempCorrectedConductivity, saltConcentration);

        // Check salt level
        if (saltConcentration < LOW_SALT_THRESHOLD) {
            storage.SaltStatus = "Low Salt";
            storage.SaltNeeded = (TARGET_SALT_REF - saltConcentration) * POOL_VOLUME / 1000.0;
            saveParam("SaltNeeded", storage.SaltNeeded);
            Debug.print(DBG_INFO, "[Salt] Low Salt: %.2f g/L, Add %.1f kg", saltConcentration, storage.SaltNeeded);
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg), "{\"SaltStatus\":\"Low Salt\",\"SaltConcentration\":%.2f,\"SaltNeeded\":%.1f}", 
                    saltConcentration, storage.SaltNeeded);
            mqttErrorPublish(errorMsg);
        }
        if (saltConcentration > TARGET_SALT_MAX) {
            storage.SaltStatus = "High Salt";
            storage.SaltNeeded = 0.0;
            saveParam("SaltNeeded", storage.SaltNeeded);
            Debug.print(DBG_INFO, "[Salt] High Salt: %.2f g/L", saltConcentration);
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg), "{\"SaltStatus\":\"High Salt\",\"SaltConcentration\":%.2f}", 
                    saltConcentration);
            mqttErrorPublish(errorMsg);
        } else {
                    storage.SaltStatus = "OK";
            storage.SaltNeeded = 0.0;
            saveParam("SaltNeeded", storage.SaltNeeded);
            Debug.print(DBG_INFO, "[Salt] OK: %.2f g/L", saltConcentration);
        }
        saveParam("SaltStatus", storage.SaltStatus);
        lastSaltMeasurement = millis();
    } else if (isHourlyMeasurement && (!SaltPump.IsRunning() || !FiltrationPump.IsRunning())) {
        // Reset status if SaltPump or FiltrationPump are not running
        storage.SaltStatus = "Unknown";
        storage.SaltConcentration = 0.0;
        storage.SaltNeeded = 0.0;
        saveParam("SaltStatus", storage.SaltStatus);
        saveParam("SaltConcentration", storage.SaltConcentration);
        saveParam("SaltNeeded", storage.SaltNeeded);
        Debug.print(DBG_INFO, "[Salt] No measurement: SaltPump=%d, FiltrationPump=%d", SaltPump.IsRunning(), FiltrationPump.IsRunning());
        lastSaltMeasurement = millis();
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

    //If filtration pump has been running for over 60secs but flow in Main-Pipe is still low, stop the filtration pump, something is wrong, set error flag
    if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 60000) && (storage.FLOWValue < storage.FLOW_MedThreshold))
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
    Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));

        vTaskDelayUntil(&ticktime, period);
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
    Debug.print(DBG_INFO, "[PhPID] Enabled (AUTOMATIC mode)");
  }
  else
  {
    //Stop PhPID
    PhPID.SetMode(MANUAL);
    storage.Ph_RegulationOnOff = 0;
    storage.PhPIDOutput = 0.0;
    PhPump.Stop();
    Debug.print(DBG_INFO, "[PhPID] Disabled (MANUAL mode)");
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

// Send notifications to IFTTT applet in case of alarm
void Send_IFTTTNotif() {
    static const String url1 = IFTTT_key;
    String url2 = "";
    static bool notif_sent[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
    if (PSIError) {
      if (!notif_sent[0]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Water%20pressure&value2=");
          if (storage.PSIValue <= storage.PSI_MedThreshold) {
            url2 += String("Low");
          } else if (storage.PSIValue >= storage.PSI_HighThreshold) {
            url2 += String("High");
          }
          url2 += String("%20pressure:%20") + String(storage.PSIValue) + String("bar");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[0] = true;
        }
      }
    } else {
      notif_sent[0] = false;
    }
  
    if (FLOWError) {
      if (!notif_sent[1]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Water%20flow&value2=");
          if (storage.FLOWValue <= storage.FLOW_MedThreshold) {
            url2 += String("Low");
          } else if (storage.FLOWValue >= storage.FLOW_HighThreshold) {
            url2 += String("High");
          }
          url2 += String("%20flow:%20") + String(storage.FLOWValue) + String("%");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[1] = true;
        }
      }
    } else {
      notif_sent[1] = false;
    }
  
    if (FLOW2Error) {
      if (!notif_sent[2]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Water%20flow2&value2=");
          if (storage.FLOW2Value <= storage.FLOW2_MedThreshold) {
            url2 += String("Low");
          } else if (storage.FLOW2Value >= storage.FLOW2_HighThreshold) {
            url2 += String("High");
          }
          url2 += String("%20flow2:%20") + String(storage.FLOW2Value) + String("%");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[2] = true;
        }
      }
    } else {
      notif_sent[2] = false;
    }
  
    if (!ChlPump.TankLevel()) {
      if (!notif_sent[3]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Chl%20level&value2=") + String(ChlPump.GetTankFill()) + String("%");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[3] = true;
        }
      }
    } else {
      notif_sent[3] = false;
    }
  
    if (!PhPump.TankLevel()) {
      if (!notif_sent[4]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=pH+%20level&value2=") + String(PhPump.GetTankFill()) + String("%");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[4] = true;
        }
      }
    } else {
      notif_sent[4] = false;
    }
  
    if (ChlPump.UpTimeError) {
      if (!notif_sent[5]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Chl%20pump%20uptime&value2=") + String(round(ChlPump.UpTime / 60000.)) + String("min");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[5] = true;
        }
      }
    } else {
      notif_sent[5] = false;
    }
  
    if (PhPump.UpTimeError) {
      if (!notif_sent[6]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=pH+%20pump%20uptime&value2=") + String(round(PhPump.UpTime / 60000.)) + String("min");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[6] = true;
        }
      }
    } else {
      notif_sent[6] = false;
    }
  
    if (WaterFill.UpTimeError) {
      if (!notif_sent[7]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=water%20fill%20uptime&value2=") + String(round(WaterFill.UpTime / 60000.)) + String("min");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[7] = true;
        }
      }
    } else {
      notif_sent[7] = false;
    }
  
    if (I2CError) {
      if (!notif_sent[8]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("I2X-Hardware Error! -> Check I2C-Hardware");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[8] = true;
        }
      }
    } else {
      notif_sent[8] = false;
    }
  
    if (storage.SaltStatus == "Low Salt") {
      if (!notif_sent[9]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Salt%20concentration&value2=Low%20salt:%20") + String(storage.SaltConcentration) + String("g/L,%20Add%20") + String(storage.SaltNeeded) + String("kg");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[9] = true;
        }
      }
    } else {
      notif_sent[9] = false;
    }
  
    if (storage.SaltStatus == "High Salt") {
      if (!notif_sent[10]) {
        if (wificlient.connect("maker.ifttt.com", 80)) {
          url2 = String("?value1=Salt%20concentration&value2=High%20salt:%20") + String(storage.SaltConcentration) + String("g/L");
          wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
          notif_sent[10] = true;
        }
      }
    } else {
      notif_sent[10] = false;
    }
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
