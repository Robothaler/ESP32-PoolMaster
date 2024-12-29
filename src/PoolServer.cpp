// Task to process JSON commands received via MQTT.
// If the comand modify a setting parameter, it is saved in NVS and published back
// for other MQTT clients (dashboards)

#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"

// Functions prototypes
bool saveParam(const char*,uint8_t );
bool saveParam(const char*,uint16_t );
bool saveParam(const char*,IPAddress );
bool saveParam(const char*,bool );
bool saveParam(const char*,unsigned long );
bool saveParam(const char*,double );
bool saveParam(const char*,String );
bool saveParam(const char* key, const uint8_t* val, size_t size);
void saveSensorMapping(const char* sensorMapping[], uint8_t ds18b20Mapping[], uint8_t numSensors);
void PublishSettings(void);
void simpLinReg(float * , float * , double & , double &, int );
void ProcessCommand(char*);
void Flow();
void PublishMeasures();
void SetPhPID(bool);
void SetOrpPID(bool);
void stack_mon(UBaseType_t&);
void connectToWiFi();
void readLocalTime();
void TempInit(void);

extern Preferences nvs;

// Definition von numSensors_A und DS18B20_New vor der Verwendung
uint8_t DS18B20_A_New[MAX_ADDRESSES][8];
uint8_t DS18B20_W_New[MAX_ADDRESSES][8];
extern int numSensors_A;
extern int numSensors_W;


void ProcessCommand(void *pvParameters)
{
  //Json Document
  StaticJsonDocument<200> command;
  char JSONCommand[150] = "";                     // JSON command to process  

  while (!startTasks) ;
  vTaskDelay(DT2);                                // Scheduling offset   

  TickType_t period = PT2;  
  static UBaseType_t hwm = 0;
  TickType_t ticktime = xTaskGetTickCount(); 

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;) {
    #ifdef CHRONO
    td = millis();
    #endif
    //Is there any incoming JSON commands
    if (uxQueueMessagesWaiting(queueIn) != 0)
    {  
      xQueueReceive(queueIn,&JSONCommand,0);

      //Parse Json object and find which command it is
      DeserializationError error = deserializeJson(command,JSONCommand);

      // Test if parsing succeeds.
      if (error)
      {
        Debug.print(DBG_WARNING,"Json parseObject() failed");
      }
      else
      {
        Debug.print(DBG_DEBUG,"Json parseObject() success: %s",JSONCommand);

        //Provide the external temperature. Should be updated regularly and will be used to start filtration for 10mins every hour when temperature is negative
        if (command.containsKey(F("TempExt")))
        {
          storage.AirTemp = command["TempExt"].as<float>();
          Debug.print(DBG_DEBUG,"External Temperature: %4.1f°C",storage.AirTemp);
        }

        //Provide the external solar temperature.
        if (command.containsKey(F("TempSolar")))
        {
          storage.SolarTemp = command["TempSolar"].as<float>();
          Debug.print(DBG_DEBUG,"External Solar Temperature: %4.1f°C",storage.SolarTemp);
        }

        //"WIFI_OnOff" command which switches WiFi On or Off
        else if (command.containsKey(F("WIFI_OnOff")))
        {
          if ((int)command[F("WIFI_OnOff")] == 0)
          {
            storage.WIFI_OnOff = 0;
            connectToWiFi();
          }
          else
          {
            storage.WIFI_OnOff = 1;
            connectToWiFi();
            readLocalTime();
            setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year-100);
          }
          saveParam("WIFI_OnOff",storage.WIFI_OnOff);
        }        

        //"WIFI" command which is called when new WIFI-SSID and WIFI-Pasword was entered
        //First parameter is WIFI-SSID, second parameter is WIFI-PASSWORD
        else if (command.containsKey(F("WIFI")))
        {
          storage.SSID = command[F("WIFI")][0].as<String>();
          storage.WIFI_PASS = command[F("WIFI")][1].as<String>();
          // Save parameter in eeprom
          saveParam("SSID", storage.SSID);
          saveParam("WIFI_PASS", storage.WIFI_PASS);
          PublishSettings();
        }

        //"MqttLogin" command which switches MQTT_Login with crdentials On or Off
        else if (command.containsKey(F("MqttLogin")))
        {
          if ((int)command[F("MqttLogin")] == 0)
          {
            storage.MQTTLOGIN_OnOff = 0;
          }
          else
          {
            storage.MQTTLOGIN_OnOff = 1;
          }
          saveParam("MQTTLOGIN_OnOff",storage.MQTTLOGIN_OnOff);
          PublishSettings();
        } 

        //"MqttPort" command which is called when new MQTT-PORT was entered
        else if (command.containsKey(F("MqttPort")))
        {
          storage.MQTT_PORT = (unsigned int)command[F("MqttPort")];
          // Save parameter in eeprom
          saveParam("MQTT_PORT", storage.MQTT_PORT);
          PublishSettings();
        }

        //"MqttIP" command which is called when new MQTT-SERVER IP-Address was entered
        else if (command.containsKey(F("MqttIP")))
        {
          String ipAddressString = "";
          for (int i = 0; i < 4; i++) {
            ipAddressString += String(command[F("MqttIP")][i].as<int>());
            if (i < 3) ipAddressString += ".";
          }
          IPAddress ipAddress;
          if (ipAddress.fromString(ipAddressString))
          {
            uint32_t ipAddressLong = ipAddress;
            storage.MQTT_IP = ipAddressLong;
            // Save parameter in nvs
            saveParam("MQTT_IP", storage.MQTT_IP);
            PublishSettings();
            Debug.print(DBG_DEBUG, "[MQTT] MQTT server IP address set to: %s", ipAddressString.c_str());
          }
          else
          {
            // Handle invalid IP address
            Debug.print(DBG_DEBUG, "Invalid IP address");
            uint32_t oldIpAddress = nvs.getULong("MQTT_IP", 0);
            storage.MQTT_IP = oldIpAddress;
          }
        }

        //"MQTTLOGIN" command which is called when new MQTT-USER, Password and Name was entered
        //First parameter is MQTT-USER, second parameter is MQTT-PASSWORD, third parameter is MQTT-NAME
        else if (command.containsKey(F("MQTTLOGIN")))
        {
          storage.MQTT_USER = command[F("MQTTLOGIN")][0].as<String>();
          storage.MQTT_PASS = command[F("MQTTLOGIN")][1].as<String>();
          storage.MQTT_NAME = command[F("MQTTLOGIN")][2].as<String>();
          // Save parameter in eeprom
          saveParam("MQTT_USER", storage.MQTT_USER);
          saveParam("MQTT_PASS", storage.MQTT_PASS);
          saveParam("MQTT_NAME", storage.MQTT_NAME);
          PublishSettings();
        }

        //"Bus_A_B" command which switches the DS18B20 Bus-Switch from A to B or visa versa
        else if (command.containsKey(F("Bus_A_B")))
        {
          if ((int)command[F("Bus_A_B")] == 0)
          {
            storage.BUS_A_B = 0;
          }
          else
          {
            storage.BUS_A_B = 1;
          }
          saveParam("BUS_A_B",storage.BUS_A_B);
          PublishSettings();
        }

        // "DS18B20A" command which is called when new Array for DS18B20A Sensor was entered
        else if (command.containsKey(F("DS18B20A")))
        {
          Debug.print(DBG_DEBUG, "DS18B20A command received");
          const JsonArray &jsonArray = command[F("DS18B20A")].as<JsonArray>();
          uint8_t newArray[MAX_ADDRESSES] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          int i = 0;
          for (auto value : jsonArray)
          {
            if (i > (MAX_ADDRESSES - 1))
              break; // Maximum of 5 sensors allowed
            int index = value.as<int>();
            if ((index >= 0) && (index <= (MAX_ADDRESSES - 1)))
            {
              newArray[i] = storage.Array_A[index];
              Debug.print(DBG_DEBUG, "New mapping for DS18B20_A sensor %d: %d", index, newArray[i]);
            }
            i++;
          }
          for (int i = 0; i < MAX_ADDRESSES; i++)
          {
            storage.Array_A[i] = newArray[i];
          }
          saveParam("Array_A", storage.Array_A, MAX_ADDRESSES);
          storage.SolarTemp = 0.0;
          storage.SolarVLTemp = 0.0;
          storage.SolarRLTemp = 0.0;
          storage.AirInTemp = 0.0;
          storage.AirTemp = 0.0;
          PublishSettings();
        }

        // "DS18B20W" command which is called when new Array for DS18B20W Sensor was entered
        else if (command.containsKey(F("DS18B20W")))
        {
          Debug.print(DBG_DEBUG, "DS18B20W command received");
          const JsonArray &jsonArray = command[F("DS18B20W")].as<JsonArray>();
          uint8_t newArray[MAX_ADDRESSES] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          int i = 0;
          for (auto value : jsonArray)
          {
            if (i > (MAX_ADDRESSES - 1))
              break; // Maximum of 5 sensors allowed
            int index = value.as<int>();
            if ((index >= 0) && (index <= (MAX_ADDRESSES - 1)))
            {
              newArray[i] = storage.Array_W[index];
              Debug.print(DBG_DEBUG, "New mapping for DS18B20_W sensor %d: %d", index, newArray[i]);
            }
            i++;
          }
          for (int i = 0; i < MAX_ADDRESSES; i++)
          {
            storage.Array_W[i] = newArray[i];
          }
          saveParam("Array_W", storage.Array_W, MAX_ADDRESSES);
          storage.WaterSTemp = 0.0;
          storage.WaterITemp = 0.0;
          storage.WaterBTemp = 0.0;
          storage.WaterWPTemp = 0.0;
          storage.WaterWTTemp = 0.0;
          PublishSettings();
        }

        //"PhCalib" command which computes and sets the calibration coefficients of the pH sensor response based on a multi-point linear regression
        //{"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
        else if (command.containsKey(F("PhCalib")))
        {
          float CalibPoints[12]; //Max six calibration point-couples! Should be plenty enough
          int NbPoints = (int)copyArray(command[F("PhCalib")].as<JsonArray>(),CalibPoints);        
          Debug.print(DBG_DEBUG,"PhCalib command - %d points received",NbPoints);
          for (int i = 0; i < NbPoints; i += 2)
            Debug.print(DBG_DEBUG,"%10.2f - %10.2f",CalibPoints[i],CalibPoints[i + 1]);

          if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
          {
            Debug.print(DBG_DEBUG,"2 points. Performing a simple offset calibration");

            //compute offset correction
            storage.pHCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

            //Set slope back to default value
            storage.pHCalibCoeffs0 = 3.76;

            Debug.print(DBG_DEBUG,"Calibration completed. Coeffs are: %10.2f, %10.2f",storage.pHCalibCoeffs0,storage.pHCalibCoeffs1);
          }
          else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
          {
            Debug.print(DBG_DEBUG,"%d points. Performing a linear regression calibration",NbPoints / 2);

            float xCalibPoints[NbPoints / 2];
            float yCalibPoints[NbPoints / 2];

            //generate array of x sensor values (in volts) and y rated buffer values
            //storage.PhValue = (storage.pHCalibCoeffs0 * ph_sensor_value) + storage.pHCalibCoeffs1;
            for (int i = 0; i < NbPoints; i += 2)
            {
              xCalibPoints[i / 2] = (CalibPoints[i] - storage.pHCalibCoeffs1) / storage.pHCalibCoeffs0;
              yCalibPoints[i / 2] = CalibPoints[i + 1];
            }

            //Compute linear regression coefficients
            simpLinReg(xCalibPoints, yCalibPoints, storage.pHCalibCoeffs0, storage.pHCalibCoeffs1, NbPoints / 2);

            Debug.print(DBG_DEBUG,"Calibration completed. Coeffs are: %10.2f, %10.2f",storage.pHCalibCoeffs0 ,storage.pHCalibCoeffs1);
          }
          //Store the new coefficients in eeprom
          saveParam("pHCalibCoeffs0",storage.pHCalibCoeffs0);
          saveParam("pHCalibCoeffs1",storage.pHCalibCoeffs1);          
          PublishSettings();
        }
        //"OrpCalib" command which computes and sets the calibration coefficients of the Orp sensor response based on a multi-point linear regression
        //{"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
        else if (command.containsKey(F("OrpCalib")))
        {
          float CalibPoints[12]; //Max six calibration point-couples! Should be plenty enough
          int NbPoints = (int)copyArray(command[F("OrpCalib")].as<JsonArray>(),CalibPoints);
          Debug.print(DBG_DEBUG,"OrpCalib command - %d points received",NbPoints);
          for (int i = 0; i < NbPoints; i += 2)
            Debug.print(DBG_DEBUG,"%10.2f - %10.2f",CalibPoints[i],CalibPoints[i + 1]);        
          if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
          {
            Debug.print(DBG_DEBUG,"2 points. Performing a simple offset calibration");

            //compute offset correction
            storage.OrpCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

            //Set slope back to default value
            storage.OrpCalibCoeffs0 = -1000;

            Debug.print(DBG_DEBUG,"Calibration completed. Coeffs are: %10.2f, %10.2f",storage.OrpCalibCoeffs0,storage.OrpCalibCoeffs1);
          }
          else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
          {
            Debug.print(DBG_DEBUG,"%d points. Performing a linear regression calibration",NbPoints / 2);

            float xCalibPoints[NbPoints / 2];
            float yCalibPoints[NbPoints / 2];

            //generate array of x sensor values (in volts) and y rated buffer values
            //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
            for (int i = 0; i < NbPoints; i += 2)
            {
              xCalibPoints[i / 2] = (CalibPoints[i] - storage.OrpCalibCoeffs1) / storage.OrpCalibCoeffs0;
              yCalibPoints[i / 2] = CalibPoints[i + 1];
            }

            //Compute linear regression coefficients
            simpLinReg(xCalibPoints, yCalibPoints, storage.OrpCalibCoeffs0, storage.OrpCalibCoeffs1, NbPoints / 2);

            Debug.print(DBG_DEBUG,"Calibration completed. Coeffs are: %10.2f, %10.2f",storage.OrpCalibCoeffs0,storage.OrpCalibCoeffs1);
          }
          //Store the new coefficients in eeprom
          saveParam("OrpCalibCoeffs0",storage.OrpCalibCoeffs0);
          saveParam("OrpCalibCoeffs1",storage.OrpCalibCoeffs1);          
          PublishSettings();
        }
        //"PSICalib" command which computes and sets the calibration coefficients of the Electronic Pressure sensor response based on a linear regression and a reference mechanical sensor (typically located on the sand filter)
        //{"PSICalib":[0,0,0.71,0.6]}   -> multi-point linear regression calibration (minimum 2 point-couple, 6 max.) in the form [ElectronicPressureSensorReading_0, MechanicalPressureSensorReading_0, xx, xx, ElectronicPressureSensorReading_n, ElectronicPressureSensorReading_n]
        else if (command.containsKey(F("PSICalib")))
        {
          float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough, typically use two point-couples (filtration ON and filtration OFF)
          int NbPoints = (int)copyArray(command[F("PSICalib")].as<JsonArray>(),CalibPoints);
          Debug.print(DBG_DEBUG,"PSICalib command - %d points received",NbPoints);
          for (int i = 0; i < NbPoints; i += 2)
            Debug.print(DBG_DEBUG,"%10.2f, %10.2f",CalibPoints[i],CalibPoints[i + 1]);

          if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
          {
            Debug.print(DBG_DEBUG,"%d points. Performing a linear regression calibration",NbPoints / 2);

            float xCalibPoints[NbPoints / 2];
            float yCalibPoints[NbPoints / 2];

            //generate array of x sensor values (in volts) and y rated buffer values
            //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
            //storage.PSIValue = (storage.PSICalibCoeffs0 * psi_sensor_value) + storage.PSICalibCoeffs1;
            for (int i = 0; i < NbPoints; i += 2)
            {
              xCalibPoints[i / 2] = (CalibPoints[i] - storage.PSICalibCoeffs1) / storage.PSICalibCoeffs0;
              yCalibPoints[i / 2] = CalibPoints[i + 1];
            }

            //Compute linear regression coefficients
            simpLinReg(xCalibPoints, yCalibPoints, storage.PSICalibCoeffs0, storage.PSICalibCoeffs1, NbPoints / 2);

            //Store the new coefficients in eeprom
            saveParam("PSICalibCoeffs0",storage.PSICalibCoeffs0);
            saveParam("PSICalibCoeffs1",storage.PSICalibCoeffs1);          
            PublishSettings();
            Debug.print(DBG_DEBUG,"Calibration completed. Coeffs are: %10.2f, %10.2f",storage.PSICalibCoeffs0,storage.PSICalibCoeffs1);
          }
        }
        //"Mode" command which sets regulation and filtration to manual or auto modes
        else if (command.containsKey(F("Mode")))
        {
          if ((int)command[F("Mode")] == 0)
          {
            storage.AutoMode = 0;

            //Stop PIDs
            SetPhPID(false);
            SetOrpPID(false);
          }
          else
          {
            storage.AutoMode = 1;
          }
          saveParam("AutoMode",storage.AutoMode);
        }
        //"SolarLocExt" command which sets regulation of Solar to Local or Extern 
        else if (command.containsKey(F("SolarLocExt")))
        {
          if ((int)command[F("SolarLocExt")] == 0)
          {
            storage.SolarLocExt = 0;
          }
          else
          {
            storage.SolarLocExt = 1;
          }
          saveParam("SolarLocExt",storage.SolarLocExt);
        }
        //"SolarMode" command which sets regulation of Solar Mode to manual or auto mode
        else if (command.containsKey(F("SolarMode")))
        {
          if ((int)command[F("SolarMode")] == 0)
          {
            storage.SolarMode = 0;
          }
          else
          {
            storage.SolarMode = 1;
          }
          saveParam("SolarMode",storage.SolarMode);
        }
        //"Salt_Chlor" command which switch between Salt desinfection or Chlor desinfection mode
        else if (command.containsKey(F("Salt_Chlor")))
        {
          if ((int)command[F("Salt_Chlor")] == 1)
          {
            storage.Salt_Chlor = 1;

            //Stop PIDs
            SetOrpPID(false);
          }
          else
          {
            storage.Salt_Chlor = 0;
          }
          saveParam("Salt_Chlor",storage.Salt_Chlor);
        }
        //"SaltMode" command which sets regulation of Salt Chlorinator to manual or auto mode
        else if (command.containsKey(F("SaltMode")))
        {
          if ((int)command[F("SaltMode")] == 0)
          {
            storage.SaltMode = 0;

            //Stop PIDs
            SetPhPID(false);
            //SetOrpPID(false);
          }
          else
          {
            storage.SaltMode = 1;
          }
          saveParam("SaltMode",storage.SaltMode);
        }
        //"SaltPolarity" command which switches electrical polarity of the electrolysis cell to avoid calcification of the plates
        else if (command.containsKey(F("SaltPolarity")))
        {
          if ((int)command[F("SaltPolarity")] == 0)
          {
            storage.SaltPolarity = 0;
          }
          else
          {
            storage.SaltPolarity = 1;
          }
          saveParam("SaltPolarity",storage.SaltPolarity);
        }
        //"ValveMode" command which sets automatic regulation of MoterValves
        else if (command.containsKey(F("ValveMode")))
        {
          if ((int)command[F("ValveMode")] == 0)
          {
            storage.ValveMode = 0;
          }
          else
          {
            storage.ValveMode = 1;
          }
          saveParam("ValveMode",storage.ValveMode);
        }
        //"CleanMode" command which sets regulation of MotorVales for predefined position for manual pool cleaning
        else if (command.containsKey(F("CleanMode")))
        {
          if ((int)command[F("CleanMode")] == 0)
          {
            storage.CleanMode = 0;
          }
          else
          {
            storage.CleanMode = 1;
          }
          saveParam("CleanMode",storage.CleanMode);
        }
        //"ValveSwitch" command which switches ELD-Nozzle from Treppe to Hinten
        else if (command.containsKey(F("ValveSwitch")))
        {
          if ((int)command[F("ValveSwitch")] == 0)
          {
            storage.ValveSwitch = 0;
          }
          else
          {
            storage.ValveSwitch = 1;
          }
          saveParam("ValveSwitch",storage.ValveSwitch);
        }
        //"FillMode" command which sets automatic regulation of Waterlevel (switches water tap on an dof based on level switches)
        else if (command.containsKey(F("FillMode")))
        {
          if ((int)command[F("FillMode")] == 0)
          {
            storage.WaterFillMode = 0;
            WaterFill.Stop();
          }
          else
          {
            storage.WaterFillMode = 1;
          }
          saveParam("FillMode",storage.WaterFillMode);
        }
        //"ELDT" command which turns the MotorValve of the ELDT-Nozzle to desired position
        else if (command.containsKey(F("ELDT")))
        {
          if (command[F("ELDT")] == "open")
          {
            ELD_Treppe.open(); //open MotorValve
          }
          else if (command[F("ELDT")] == "close")
          {
            ELD_Treppe.close(); //close MotorValve
          }
          else if (command[F("ELDT")] == "halfOpen")
          {
            ELD_Treppe.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("ELDT")] == "calibrate")
          {
            ELD_Treppe.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("ELDT")].as<int>();
            ELD_Treppe.setTargetAngle(target);
          }
        }
        //"ELDH" command which turns the MotorValve of the ELDH-Nozzle to desired position
        else if (command.containsKey(F("ELDH")))
        {
          if (command[F("ELDH")] == "open")
          {
            ELD_Hinten.open(); //open MotorValve
          }
          else if (command[F("ELDH")] == "close")
          {
            ELD_Hinten.close(); //close MotorValve
          }
          else if (command[F("ELDH")] == "halfOpen")
          {
            ELD_Hinten.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("ELDH")] == "calibrate")
          {
            ELD_Hinten.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("ELDH")].as<int>();
            ELD_Hinten.setTargetAngle(target);
          }
        }
        //"WPV" command which turns the MotorValve of the Heatpump prevalve to desired position
        else if (command.containsKey(F("WPV")))
        {
          if (command[F("WPV")] == "open")
          {
            WP_Vorlauf.open(); //open MotorValve
          }
          else if (command[F("WPV")] == "close")
          {
            WP_Vorlauf.close(); //close MotorValve
          }
          else if (command[F("WPV")] == "halfOpen")
          {
            WP_Vorlauf.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("WPV")] == "calibrate")
          {
            WP_Vorlauf.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("WPV")].as<int>();
            WP_Vorlauf.setTargetAngle(target);
          }
        }
        //"WPM" command which turns the MotorValve of the Heatpump mixing valve to desired position
        else if (command.containsKey(F("WPM")))
        {
          if (command[F("WPM")] == "open")
          {
            WP_Mischer.open(); //open MotorValve
          }
          else if (command[F("WPM")] == "close")
          {
            WP_Mischer.close(); //close MotorValve
          }
          else if (command[F("WPM")] == "halfOpen")
          {
            WP_Mischer.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("WPM")] == "calibrate")
          {
            WP_Mischer.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("WPM")].as<int>();
            WP_Mischer.setTargetAngle(target);
          }
        }
        //"BOTT" command which turns the MotorValve of the Bottom Intake valve to desired position
        else if (command.containsKey(F("BOTT")))
        {
          if (command[F("BOTT")] == "open")
          {
            Bodenablauf.open(); //open MotorValve
          }
          else if (command[F("BOTT")] == "close")
          {
            Bodenablauf.close(); //close MotorValve
          }
          else if (command[F("BOTT")] == "halfOpen")
          {
            Bodenablauf.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("BOTT")] == "calibrate")
          {
            Bodenablauf.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("BOTT")].as<int>();
            Bodenablauf.setTargetAngle(target);
          }
        }
        //"SolarValve" command which turns the MotorValve of the Solar valve to desired position
        else if (command.containsKey(F("SOLARVALVE")))
        {
          if (command[F("SOLARVALVE")] == "open")
          {
            Solarvalve.open(); //open MotorValve
          }
          else if (command[F("SOLARVALVE")] == "close")
          {
            Solarvalve.close(); //close MotorValve
          }
          else if (command[F("SOLARVALVE")] == "halfOpen")
          {
            Solarvalve.halfOpen(); //halfOpen MotorValve
          }
          else if (command[F("SOLARVALVE")] == "calibrate")
          {
            Solarvalve.calibrate(); //calibrate MotorValve
          }
          else
          {
            int target = (int)command[F("SOLARVALVE")].as<int>();
            Solarvalve.setTargetAngle(target);
          }
        }
        else if (command.containsKey(F("Heat"))) //"Heat" command which starts/stops water heating
        {
          if ((int)command[F("Heat")] == 0)
          {
            storage.WaterHeat = false;
            WaterHeatPump.Stop();
          }
          else
          {
            storage.WaterHeat = true;
          }
          saveParam("Heat",storage.WaterHeat);
        }
        else if (command.containsKey(F("Winter"))) //"Winter" command which activate/deactivate Winter Mode
          {
          (bool)command[F("Winter")] ? storage.WinterMode = true : storage.WinterMode = false;
          saveParam("WinterMode",storage.WinterMode);
          PublishSettings(); 
        }
        else if (command.containsKey(F("PhSetPoint"))) //"PhSetPoint" command which sets the setpoint for Ph
        {
          storage.Ph_SetPoint = command[F("PhSetPoint")].as<double>();
          Debug.print(DBG_DEBUG,"Command PhSetPoint: %13.9f",storage.Ph_SetPoint);
          saveParam("Ph_SetPoint",storage.Ph_SetPoint);
          PublishSettings();
        }
        else if (command.containsKey(F("OrpSetPoint"))) //"OrpSetPoint" command which sets the setpoint for ORP
        {
          storage.Orp_SetPoint = command[F("OrpSetPoint")].as<double>();
          saveParam("Orp_SetPoint",storage.Orp_SetPoint);
          PublishSettings();
        }
        else if (command.containsKey(F("WSetPoint"))) //"WSetPoint" command which sets the setpoint for Water temp (currently not in use)
        {
          storage.WaterTemp_SetPoint = (double)command[F("WSetPoint")];
          saveParam("WaterTempSet",storage.WaterTemp_SetPoint);
          PublishSettings();
        }
        /*
        else if (command.containsKey(F("PBwMaxSetPoint"))) //"PBwMaxSetPoint" command which sets the setpoint for Water temp (currently not in use)
        {
          storage.BackwashMaxPressure_SetPoint = (double)command[F("PBwMaxSetPoint")];
          saveParam("WaterTempSet",storage.BackwashMaxPressure_SetPoint);
          PublishSettings();
        }
        */
        //"pHTank" command which is called when the pH tank is changed or refilled
        //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
        else if (command.containsKey(F("pHTank")))
        {
          storage.pHTankVol = (double)command[F("pHTank")][0];
          PhPump.SetTankVolume(storage.pHTankVol);
          storage.AcidFill = (double)command[F("pHTank")][1];
          PhPump.SetTankFill(storage.AcidFill);
          saveParam("pHTankVol",storage.pHTankVol);
          saveParam("AcidFill",storage.AcidFill);               
          PublishSettings();
        }
        //"ChlTank" command which is called when the Chl tank is changed or refilled
        //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
        else if (command.containsKey(F("ChlTank")))
        {
          storage.ChlTankVol = (double)command[F("ChlTank")][0];
          ChlPump.SetTankVolume(storage.ChlTankVol);
          storage.ChlFill = (double)command[F("ChlTank")][1];
          ChlPump.SetTankFill(storage.ChlFill);
          saveParam("ChlTankVol",storage.ChlTankVol);
          saveParam("ChlFill",storage.ChlFill);
          PublishSettings();
        }
        else if (command.containsKey(F("WTempLow"))) //"WTempLow" command which sets the setpoint for Water temp low threshold
        {
          storage.WaterTempLowThreshold = (double)command[F("WTempLow")];
          saveParam("WaterTempLow",storage.WaterTempLowThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("PumpsMaxUp"))) //"PumpsMaxUp" command which sets the Max UpTime for pumps
        {
          storage.PhPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")] * 60 * 1000; // minutes in milliseconds
          PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit);
          storage.ChlPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")] * 60 * 1000; // minutes in milliseconds
          ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit);
          saveParam("PhPumpUTL",storage.PhPumpUpTimeLimit);
          saveParam("ChlPumpUTL",storage.ChlPumpUpTimeLimit);                    
          PublishSettings();
        }
        else if (command.containsKey(F("WFMaxUp"))) //"WFMaxUp" command which sets the Max UpTime for WaterFill
          {
          storage.WaterFillUpTimeLimit = (unsigned int)command[F("WFMaxUp")] * 60 * 1000; // minutes in milliseconds
          WaterFill.SetMaxUpTime(storage.WaterFillUpTimeLimit);
          saveParam("WFMaxUp",storage.WaterFillUpTimeLimit);
          PublishSettings();
        }
        else if (command.containsKey(F("OrpPIDParams"))) //"OrpPIDParams" command which sets the Kp, Ki and Kd values for Orp PID loop
        {
          storage.Orp_Kp = (double)command[F("OrpPIDParams")][0]*10000;
          storage.Orp_Ki = (double)command[F("OrpPIDParams")][1];
          storage.Orp_Kd = (double)command[F("OrpPIDParams")][2];
          saveParam("Orp_Kp",storage.Orp_Kp);
          saveParam("Orp_Ki",storage.Orp_Ki);
          saveParam("Orp_Kd",storage.Orp_Kd);
          OrpPID.SetTunings(storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd);
          PublishSettings();
        }
        else if (command.containsKey(F("PhPIDParams"))) //"PhPIDParams" command which sets the Kp, Ki and Kd values for Ph PID loop
        {
          storage.Ph_Kp = (double)command[F("PhPIDParams")][0]*10000;
          storage.Ph_Ki = (double)command[F("PhPIDParams")][1];
          storage.Ph_Kd = (double)command[F("PhPIDParams")][2];
          saveParam("Ph_Kp",storage.Ph_Kp);
          saveParam("Ph_Ki",storage.Ph_Ki);
          saveParam("Ph_Kd",storage.Ph_Kd);
          PhPID.SetTunings(storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd);
          PublishSettings();
        }
        else if (command.containsKey(F("OrpPIDWSize"))) //"OrpPIDWSize" command which sets the window size of the Orp PID loop
        {
          storage.OrpPIDWindowSize = (unsigned long)command[F("OrpPIDWSize")] * 60 * 1000; // minutes in milliseconds
          saveParam("OrpPIDWSize",storage.OrpPIDWindowSize);
          OrpPID.SetSampleTime((int)storage.OrpPIDWindowSize);
          OrpPID.SetOutputLimits(0, storage.OrpPIDWindowSize);  //Whatever happens, don't allow continuous injection of Chl for more than a PID Window
          PublishSettings();
        }
        else if (command.containsKey(F("PhPIDWSize"))) //"PhPIDWSize" command which sets the window size of the Ph PID loop
        {
          storage.PhPIDWindowSize = (unsigned long)command[F("PhPIDWSize")] * 60 * 1000; // minutes in milliseconds
          saveParam("PhPIDWSize",storage.PhPIDWindowSize);
          PhPID.SetSampleTime((int)storage.PhPIDWindowSize);
          PhPID.SetOutputLimits(0, storage.PhPIDWindowSize);    //Whatever happens, don't allow continuous injection of Acid for more than a PID Window
          PublishSettings();
        }
        else if (command.containsKey(F("Date"))) //"Date" command which sets the Date of RTC module
        {
          setTime((uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6], (uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][3]); //(Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
        }
        else if (command.containsKey(F("FiltT0"))) //"FiltT0" command which sets the earliest hour when starting Filtration pump
        {
          storage.FiltrationStartMin = (unsigned int)command[F("FiltT0")];
          saveParam("FiltrStartMin",storage.FiltrationStartMin);
          PublishSettings();
        }
        else if (command.containsKey(F("FiltT1"))) //"FiltT1" command which sets the latest hour for running Filtration pump
        {
          storage.FiltrationStopMax = (unsigned int)command[F("FiltT1")];
          saveParam("FiltrStopMax",storage.FiltrationStopMax);
          PublishSettings();
        }
        else if (command.containsKey(F("SolarT0"))) //"SolarT0" command which sets the earliest hour when starting Solar pump
        {
          storage.SolarStartMin = (unsigned int)command[F("SolarT0")];
          saveParam("SolarStartMin",storage.SolarStartMin);
          PublishSettings();
        }
        else if (command.containsKey(F("SolarT1"))) //"SolarT1" command which sets the latest hour for running Solar pump
        {
          storage.SolarStopMax = (unsigned int)command[F("SolarT1")];
          saveParam("SolarStopMax",storage.SolarStopMax);
          PublishSettings();
        }
        else if (command.containsKey(F("PubPeriod"))) //"PubPeriod" command which sets the periodicity for publishing system info to MQTT broker
        {
          storage.PublishPeriod = (unsigned long)command[F("PubPeriod")] * 1000; //in secs
          saveParam("PublishPeriod",storage.PublishPeriod);
          PublishSettings();
        }
        else if (command.containsKey(F("DelayPID"))) //"DelayPID" command which sets the delay from filtering start before PID loops start regulating
        {
          storage.DelayPIDs = (unsigned int)command[F("DelayPID")];
          saveParam("DelayPIDs",storage.DelayPIDs);
          PublishSettings();
        }
        else if (command.containsKey(F("FillDur"))) //"FillDur" command which sets the duration from Water low level trigger to start waterfill process until it stops
        {
          storage.WaterFillDuration = (unsigned int)command[F("FillDur")] * 60 * 1000; // minutes in milliseconds
          saveParam("FillDur",storage.WaterFillDuration);
          PublishSettings();
        }
        else if (command.containsKey(F("PSIHigh"))) //"PSIHigh" command which sets the water high-pressure threshold
        {
          storage.PSI_HighThreshold = (double)command[F("PSIHigh")];
          saveParam("PSI_High",storage.PSI_HighThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("PSILow"))) //"PSILow" command which sets the water low-pressure threshold
        {
          storage.PSI_MedThreshold = (double)command[F("PSILow")];
          saveParam("PSI_Med",storage.PSI_MedThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOWPulse"))) //"FLOWPulse" command which sets the pulse counter
        {
          storage.FLOW_Pulse = (double)command[F("FLOWPulse")];
          saveParam("FLOW_Pulse",storage.FLOW_Pulse);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOWHigh"))) //"FLOWHigh" command which sets the water high-pressure threshold
        {
          storage.FLOW_HighThreshold = (double)command[F("FLOWHigh")];
          saveParam("FLOW_High",storage.FLOW_HighThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOWLow"))) //"FLOWLow" command which sets the water low-pressure threshold
        {
          storage.FLOW_MedThreshold = (double)command[F("FLOWLow")];
          saveParam("FLOW_Med",storage.FLOW_MedThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOW2Pulse"))) //"FLOW2Pulse" command which sets the pulse counter
        {
          storage.FLOW2_Pulse = (double)command[F("FLOW2Pulse")];
          saveParam("FLOW2_Pulse",storage.FLOW2_Pulse);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOW2High"))) //"FLOW2High" command which sets the water high-pressure threshold
        {
          storage.FLOW2_HighThreshold = (double)command[F("FLOW2High")];
          saveParam("FLOW2_High",storage.FLOW2_HighThreshold);
          PublishSettings();
        }
        else if (command.containsKey(F("FLOW2Low"))) //"FLOW2Low" command which sets the water low-pressure threshold
        {
          storage.FLOW2_MedThreshold = (double)command[F("FLOW2Low")];
          saveParam("FLOW2_Med",storage.FLOW2_MedThreshold);
          PublishSettings();
        }       
        else if (command.containsKey(F("pHPumpFR")))//"PhPumpFR" set flow rate of Ph pump
        {
          storage.pHPumpFR = (double)command[F("pHPumpFR")];
          PhPump.SetFlowRate((double)command[F("pHPumpFR")]);
          saveParam("pHPumpFR",storage.pHPumpFR);
          PublishSettings();
        }
        else if (command.containsKey(F("ChlPumpFR")))//"ChlPumpFR" set flow rate of Chl pump
        {
          storage.ChlPumpFR = (double)command[F("ChlPumpFR")];
          ChlPump.SetFlowRate((double)command[F("ChlpumpFR")]);
          saveParam("ChlPumpFR",storage.ChlPumpFR);
          PublishSettings();
        }
        else if (command.containsKey(F("WaterFillFR")))//"PhPumpFR" set flow rate of Ph pump
        {
          storage.WaterFillFR = (double)command[F("WaterFillFR")];
          WaterFill.SetFlowRate((double)command[F("WaterFillFR")]);
          saveParam("WaterFillFR",storage.WaterFillFR);
          PublishSettings();
        }
        else if (command.containsKey(F("RstWatCons")))//"RstWatCons" reset the anualwater consumption 
        {
          storage.WaterFillAnCon = 0;
          saveParam("WaterFillAnCon", storage.WaterFillAnCon);
          PublishSettings();
        }
        else if (command.containsKey(F("RstpHCal")))//"RstpHCal" reset the calibration coefficients of the pH probe
        {
          storage.pHCalibCoeffs0 = 3.51;
          storage.pHCalibCoeffs1 = -2.73;
          saveParam("pHCalibCoeffs0",storage.pHCalibCoeffs0);
          saveParam("pHCalibCoeffs1",storage.pHCalibCoeffs1);
          PublishSettings();
        }
        else if (command.containsKey(F("RstOrpCal")))//"RstOrpCal" reset the calibration coefficients of the Orp probe
        {
          storage.OrpCalibCoeffs0 = (double)-930.;
          storage.OrpCalibCoeffs1 = (double)2455.;
          saveParam("OrpCalibCoeffs0",storage.OrpCalibCoeffs0);
          saveParam("OrpCalibCoeffs1",storage.OrpCalibCoeffs1);
          PublishSettings();
        }
        else if (command.containsKey(F("RstPSICal")))//"RstPSICal" reset the calibration coefficients of the pressure sensor
        {
          storage.PSICalibCoeffs0 = (double)1.31;
          storage.PSICalibCoeffs1 = (double)-0.1;
          saveParam("PSICalibCoeffs0",storage.PSICalibCoeffs0);
          saveParam("PSICalibCoeffs1",storage.PSICalibCoeffs1);
          PublishSettings();
        }
        else if (command.containsKey(F("Settings")))//Pubilsh settings to refresh data on remote displays
        {
          PublishSettings();
        }         
        else if (command.containsKey(F("FiltPump"))) //"FiltPump" command which starts or stops the filtration pump
        {
          if ((int)command[F("FiltPump")] == 0)
          {
            EmergencyStopFiltPump = true;
            FiltrationPump.Stop();  //stop filtration pump

            //Stop PIDs
            SetPhPID(false);
            SetOrpPID(false);
          }
          else
          {
            EmergencyStopFiltPump = false;
            FiltrationPump.Start();   //start filtration pump
          }
        }
        else if (command.containsKey(F("RobotPump"))) //"RobotPump" command which starts or stops the Robot pump
        {
          if ((int)command[F("RobotPump")] == 0){
            RobotPump.Stop();    //stop robot pump
            cleaning_done = true;
          } else {
            RobotPump.Start();   //start robot pump
            cleaning_done = false;
          }  
        }
        else if (command.containsKey(F("HeatPump"))) //"HeatPump" command which starts or stops the Heatpump
        {
          if ((int)command[F("HeatPump")] == 0){
            HeatPump.Stop();    //stop Heatpump
          } else {
            HeatPump.Start();   //start Heatpump
          }  
        }
        else if (command.containsKey(F("WaterFill"))) //"WaterFill" command which starts or stops the WaterFill tap
        {
          if ((int)command[F("WaterFill")] == 0){
            WaterFill.Stop();    //stop WaterFill tap
          } else {
            WaterFill.Start();   //start WaterFill tap
          }  
        }
        else if (command.containsKey(F("SolarPump"))) //"SolarPump" command which starts or stops the Solar pump
        {
          if ((int)command[F("SolarPump")] == 0){
            SolarPump.Stop();    //stop solar pump
            publishSolarMode(2); //sets SolarControl to "puffer" mode
          } else {
            SolarPump.Start();   //start solar pump
            publishSolarMode(1); //sets SolarControl to "pool" mode
          }  
        }
        else if (command.containsKey(F("SaltPump"))) //"SaltPump" command which starts or stops the Salt pump
        {
          if ((int)command[F("SaltPump")] == 0){
            SaltPump.Stop();    //stop salt pump
          } else {
            SaltPump.Start();   //start salt pump
          }  
        }
        else if (command.containsKey(F("SaltDiff"))) //"SaltDiff" command which sets the Differenz of the ORP-Value to turn on/off the saltpump
        {
          storage.SaltDiff = (double)command[F("SaltDiff")];
          saveParam("SaltDiff",storage.SaltDiff);
          PublishSettings();
        }
        else if (command.containsKey(F("PhPump"))) //"PhPump" command which starts or stops the Acid pump
        {
          if ((int)command[F("PhPump")] == 0)
            PhPump.Stop();       //stop Acid pump
          else
            PhPump.Start();      //start Acid pump
        }
        else if (command.containsKey(F("ChlPump"))) //"ChlPump" command which starts or stops the Acid pump
        {
          if ((int)command[F("ChlPump")] == 0)
            ChlPump.Stop();      //stop Chl pump
          else
            ChlPump.Start();     //start Chl pump
        }
        else if (command.containsKey(F("PhPID"))) //"PhPID" command which starts or stops the Ph PID loop
        {
          if ((int)command[F("PhPID")] == 0)
          {
            //Stop PID
            SetPhPID(false);
          }
          else
          {
            //Start PID
            SetPhPID(true);
          }
        }
        else if (command.containsKey(F("OrpPID"))) //"OrpPID" command which starts or stops the Orp PID loop
        {
          if ((int)command[F("OrpPID")] == 0)
          {
            //Stop PID
            SetOrpPID(false);
          }
          else
          {
            //Start PID
            SetOrpPID(true);
          }
        }
        //"Relay" command which is called to actuate relays
        //Parameter 1 is the relay number (R0 in this example), parameter 2 is the relay state (ON in this example).
        else if (command.containsKey(F("Relay")))
        {
          switch ((int)command[F("Relay")][0])
          {
            case 0:
              (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R0, LOW) : digitalWrite(RELAY_R0, HIGH);
              break;
            case 1:
              (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R1, LOW) : digitalWrite(RELAY_R1, HIGH);
              break;
          }
        }
        else if (command.containsKey(F("Reboot")))//"Reboot" command forces a reboot of the controller
        {
          delay(10000); // wait 10s then restart. Other tasks continue.
          esp_restart();
        }
        else if (command.containsKey(F("Clear"))) //"Clear" command which clears the UpTime and pressure errors of the Pumps
        {
          if (PSIError)
            PSIError = false;

          if (FLOWError)
            FLOWError = false;

          if (FLOW2Error)
            FLOW2Error = false;
          
          if (WaterFillError)
            WaterFillError = false;

          if (PhPump.UpTimeError)
            PhPump.ClearErrors();

          if (ChlPump.UpTimeError)
            ChlPump.ClearErrors();

          if (WaterFill.UpTimeError)
            WaterFill.ClearErrors();

          //start filtration pump if within scheduled time slots
          if (!EmergencyStopFiltPump && storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
            FiltrationPump.Start();
        }
        //Publish Update on the MQTT broker the status of our variables
        PublishMeasures();
      }
    }
    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[PoolServer] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif 
    stack_mon(hwm); 
    vTaskDelayUntil(&ticktime,period);
  }  
}

//Linear regression coefficients calculation function
// pass x and y arrays (pointers), lrCoef pointer, and n.
//The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is the length of the x and y arrays.
//http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
void simpLinReg(float * x, float * y, double & lrCoef0, double & lrCoef1, int n)
{
  // initialize variables
  float xbar = 0;
  float ybar = 0;
  float xybar = 0;
  float xsqbar = 0;

  // calculations required for linear regression
  for (int i = 0; i < n; i++)
  {
    xbar += x[i];
    ybar += y[i];
    xybar += x[i] * y[i];
    xsqbar += x[i] * x[i];
  }

  xbar /= n;
  ybar /= n;
  xybar /= n;
  xsqbar /= n;

  // simple linear regression algorithm
  lrCoef0 = (xybar - xbar * ybar) / (xsqbar - xbar * xbar);
  lrCoef1 = ybar - lrCoef0 * xbar;
}