// Supervisory task

#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"

static WiFiClient wificlient;

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
void UpdateTFT(void);
void stack_mon(UBaseType_t&);
void Send_IFTTTNotif(void);

void PoolMaster(void *pvParameters)
{

  bool DoneForTheDay = false;                     // Reset actions done once per day
  bool d_calc = false;                            // Filtration duration computed
  bool cleaning_done = false;                     // daily cleaning done 

  static UBaseType_t hwm=0;                       // free stack size

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
    SolarHeatPump.loop();
    HeatPump.loop();
    SaltPump.loop();
    PhPump.loop();
    ChlPump.loop();
    RobotPump.loop();

    //update MotorValves
    ELD_Treppe.loop();
    ELD_Hinten.loop();
    WP_Vorlauf.loop();
    WP_Mischer.loop();
    Bodenablauf.loop();
    Solarvalve.loop();

    //reset time counters at midnight and send sync request to time server
    if (hour() == 0 && !DoneForTheDay)
    {
        //First store current Chl and Acid consumptions of the day in Eeprom
        storage.AcidFill = PhPump.GetTankFill();
        storage.ChlFill = ChlPump.GetTankFill();
        saveParam("AcidFill", storage.AcidFill);
        saveParam("ChlFill", storage.ChlFill);

        FiltrationPump.ResetUpTime();
        PhPump.ResetUpTime();
        PhPump.SetTankFill(storage.AcidFill);
        ChlPump.ResetUpTime();
        ChlPump.SetTankFill(storage.ChlFill);
        RobotPump.ResetUpTime();
        SolarHeatPump.ResetUpTime();
        SaltPump.ResetUpTime();
        HeatPump.ResetUpTime();

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

    // Compute next Filtering duration and start/stop hours at 15:00 (to filter during the hotest period of the day)
    // Wait at least 5mn after filtration start in order to let the temperature stabilizes in pipes, and to avoid
    // taking into account not yet measured temperature if the system starts at 15:xx. 
    // Depending on water temperature, the filtration duration is either 2 hours, temp/3 or temp/2 hours.
    #ifdef DEBUG
    if (second() == 0 && (millis() - FiltrationPump.LastStartTime) > 300000 && !d_calc)
    #else
    if (hour() == 15 && (millis() - FiltrationPump.LastStartTime) > 300000 && !d_calc)
    #endif
    {
        if (storage.TempValue < storage.WaterTempLowThreshold){
            storage.FiltrationDuration = 2;}
        else if (storage.TempValue >= storage.WaterTempLowThreshold && storage.TempValue < storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.TempValue / 3.);}
        else if (storage.TempValue >= storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.TempValue / 2.);}
    
        storage.FiltrationStart = 15 - (int)round(storage.FiltrationDuration / 2.);
        if (storage.FiltrationStart < storage.FiltrationStartMin)
        storage.FiltrationStart = storage.FiltrationStartMin;    
        storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;
        if (storage.FiltrationStop > storage.FiltrationStopMax)
        storage.FiltrationStop = storage.FiltrationStopMax;

        saveParam("FiltrStart",storage.FiltrationStart);  
        saveParam("FiltrStop",storage.FiltrationStop);  

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

    //start cleaning robot for 2 hours 30mn after filtration start
    if (FiltrationPump.IsRunning() && storage.AutoMode && !storage.WinterMode && !RobotPump.IsRunning() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60) >= 30 && !cleaning_done)
    {
        RobotPump.Start();
        Debug.print(DBG_INFO,"Robot Start 30mn after Filtration");    
    }
    if(RobotPump.IsRunning() && storage.AutoMode && ((millis() - RobotPump.LastStartTime) / 1000 / 60) >= 120)
    {
        RobotPump.Stop();
        cleaning_done = true;
        Debug.print(DBG_INFO,"Robot Stop after: %d mn",(int)(millis()-RobotPump.LastStartTime)/1000/60);
    }

    //If water heating is desired and filtration has been running for over 5mins (so that measured water temp is accurate), open/close the HEAT_ON relay as required
    //in order to regulate the water temp. When closing the HEAT_ON relay, my house heating system switches to a fixed water temperature mode and starts the pool water
    //circulator in order to heat-up the heat exchanger located on the pool filtration water circuit
    if (storage.WaterHeat && FiltrationPump.IsRunning())
    {
    if (FiltrationPump.UpTime / 1000 / 60 > 5)
    {
      if (storage.TempValue < (storage.WaterTemp_SetPoint - 0.2))
      {
        SolarHeatPump.Start();
        Solarvalve.open();
      }
      else if (storage.TempValue > (storage.WaterTemp_SetPoint + 0.2))
      {
        SolarHeatPump.Stop();
        Solarvalve.close();
      }
    }
    }
    else
    {
    SolarHeatPump.Stop();
    Solarvalve.close();
    }

    //The circulator of the pool water heating circuit needs to run regularly to avoid blocking
    //Let it run every day at noon for 2 mins
    if (storage.AutoMode && ((hour() == 12) && (minute() == 0)))
    {
    SolarHeatPump.Start();
    Solarvalve.open();
    }

    if (storage.AutoMode && ((hour() == 12) && (minute() == 2)))
    {
    SolarHeatPump.Stop();
    Solarvalve.close();
    }

    //The MotorValves should be calibrated daily, to do this we start calibration at 5 o´clock in the morning.
    if ((hour() == 5) && (minute() == 0))
    {
    ELD_Treppe.calibrate();
    ELD_Hinten.calibrate();
    WP_Vorlauf.calibrate();
    WP_Mischer.calibrate();
    Bodenablauf.calibrate();
    Solarvalve.calibrate();
    }      

    //After calibration we set the valves in to a standard position every morning.
    if ((hour() == 5) && (minute() == 3))
    {
    ELD_Treppe.open();
    ELD_Hinten.open();
    WP_Vorlauf.close();
    WP_Mischer.open();
    Bodenablauf.halfOpen();
    Solarvalve.close();
    }   

    //If Heatpump is turned on set MotorValves in correct position -> only if ValveMode is true
    if (storage.ValveMode)
    {
        if (HeatPump.IsRunning())
        {
            WP_Vorlauf.open();
            WP_Mischer.close();
        } else
        {
            WP_Vorlauf.close();
            WP_Mischer.open();
        }
    }

    // start PIDs with delay after FiltrationStart in order to let the readings stabilize
    // start inhibited if water temperature below threshold and/or in winter mode
    if (FiltrationPump.IsRunning() && storage.AutoMode && !FLOW2Error && !storage.WinterMode && !PhPID.GetMode() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60 >= storage.DelayPIDs) &&
        (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop) &&
        storage.TempValue >= storage.WaterTempLowThreshold)
    {
        //Start PIDs
        SetPhPID(true);
        SetOrpPID(true);
    }

    // start SaltPump with delay after FiltrationStart in order to let the readings stabilize
    // start inhibited if water temperature below threshold and/or in winter mode
    if (FiltrationPump.IsRunning() && storage.AutoMode && storage.SaltMode && storage.Salt_Chlor && !FLOW2Error && !storage.WinterMode &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60 >= storage.DelayPIDs) &&
        (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop) &&
        storage.TempValue >= storage.WaterTempLowThreshold)
    {
        //Start SaltPump
        SaltPump.Start();
    }

    // The polarity of the Salt electrolysis should be changed every 4 hours to prevent calcification of the electrolysis plates.
    if (storage.Salt_Chlor)
    {
        int last_time = 0;
        bool reverse = false;
        int run_time = 0;
        int saltpump_uptime = SaltPump.UpTime;
        run_time += saltpump_uptime - last_time;
        last_time = saltpump_uptime;
        if (run_time >= 4 * 60 * 60) {
            run_time = 0;
            reverse = !reverse;
        }
        if (reverse) {
            digitalWrite(SALT_POL, DIRECT);
        } else {
            digitalWrite(SALT_POL, REVERSE);
        }
    }

    //stop filtration pump and PIDs as scheduled unless we are in AntiFreeze mode
    if (storage.AutoMode && FiltrationPump.IsRunning() && !AntiFreezeFiltering && (hour() >= storage.FiltrationStop || hour() < storage.FiltrationStart))
    {
        SetPhPID(false);
        SetOrpPID(false);
        FiltrationPump.Stop();
    }

    //Outside regular filtration hours, start filtration in case of cold Air temperatures (<-2.0deg)
    if (!EmergencyStopFiltPump && storage.AutoMode && !PSIError && !FLOWError && !FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && (storage.TempExternal < -2.0))
    {
        FiltrationPump.Start();
        AntiFreezeFiltering = true;
    }

    //Outside regular filtration hours and if in AntiFreezeFiltering mode but Air temperature rose back above 2.0deg, stop filtration
    if (storage.AutoMode && FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && AntiFreezeFiltering && (storage.TempExternal > 2.0))
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
    
    //UPdate Nextion TFT
    UpdateTFT();

    //Send IFTTT notifications if alarms occured
    Send_IFTTTNotif();

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

//Enable/Disable Chl PID
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
        if(!notif_sent[0])
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
                notif_sent[0] = true;
            }
        }    
    } else notif_sent[0] = false;

    if(FLOW2Error)
    {
        if(!notif_sent[0])
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
                notif_sent[0] = true;
            }
        }    
    } else notif_sent[0] = false;

    if(!ChlPump.TankLevel())
    {
        if(!notif_sent[1])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Chl%20level&value2=") + String(ChlPump.GetTankFill()) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[1] = true;
            }
        }
    } else notif_sent[1] = false;

    if(!PhPump.TankLevel())
    {
        if(!notif_sent[2])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=pH+%20level&value2=") + String(PhPump.GetTankFill()) + String("%");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[2] = true;
            }
        }
    } else notif_sent[2] = false;

    if(ChlPump.UpTimeError)
    {
        if(!notif_sent[3])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=Chl%20pump%20uptime&value2=") + String(round(ChlPump.UpTime/60000.)) + String("min");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[3] = true;
            }
        }
    } else notif_sent[3] = false;

    if(PhPump.UpTimeError)
    {
        if(!notif_sent[4])
        {
            if(wificlient.connect("maker.ifttt.com",80))
            {
                url2 = String("?value1=pH+%20pump%20uptime&value2=") + String(round(PhPump.UpTime/60000.)) + String("min");
                wificlient.print(String("POST ") + url1 + url2 + String(" HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"));
                notif_sent[4] = true;
            }
        }
    } else notif_sent[4] = false;     
}