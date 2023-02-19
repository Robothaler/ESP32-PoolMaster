#undef __STRICT_ANSI__              // work-around for Time Zone definition
#include <stdint.h>                 // std lib (types definitions)
#include <Arduino.h>                // Arduino framework
#include <esp_sntp.h>

#include "Config.h"
#include "PoolMaster.h"

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
StoreStruct storage =
{ 
  CONFIG_VERSION,
  1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0,
  13, 8, 21, 8, 22, 11, 18, 20,
  2700, 2700, 30000,
  1800000, 1800000, 0, 0,
  7.3, 740.0, 1.8, 12.0, 90, 5880.0, 40.0, 0.4, 30.0, 7.0, 10.0, 30.0, 3.0, 3.48464236, -2.27151021, -951.822669, 2421.45966, 1.0, 0.0, 30.0,
  2700000.0, 0.0, 0.0, 18000.0, 0.0, 0.0, 0.0, 0.0, 28.0, 7.3, 720., 1.3, 70, 9,
  60.0, 85.0, 20.0, 20.0, 1.5, 1.5,
};

/*
Description of above values

ConfigVersion
Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, Salt_Chlor, SaltMode, SaltPolarity, WinterMode, WaterHeat, ValveMode, CleanMode, ValveSwitch
FiltrationDuration, FiltrationStart, FiltrationStop, FiltrationStartMin, FiltrationStopMax, SolarStartMin, SolarStopMax, DelayPIDs
PhPumpUpTimeLimit, ChlPumpUpTimeLimit, PublishPeriod
PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime
Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, FLOW_Pulse, FLOW_HighThreshold, FLOW2_Pulse, FLOW2_HighThreshold, PSI_MedThreshold, FLOW_MedThreshold, FLOW2_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, TempExternal, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1, SaltDiff,
Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, TempValue, PhValue, OrpValue, PSIValue, FLOWValue, FLOW2Value);
AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR);

*/

tm timeinfo;

//Set the I2C HEX Adress for the second PCF8574A IO-Portexpander
PCF8574 pcf8574(0x3F);
PCF8574 pcf8574_3(0x20);
PCF8574 pcf8574_4(0x21);

// RTC Declare module type
RTC_DS3231 rtc;

// Various global flags
volatile bool startTasks = false;               // Signal to start loop tasks
bool RTCfound = false;

bool AntiFreezeFiltering = false;               // Filtration anti freeze mode
bool EmergencyStopFiltPump = false;             // flag will be (re)set by double-tapp button
bool PSIError = false;                          // Water pressure OK
bool FLOWError = false;                         // Water flow in Main-Pipe OK
bool FLOW2Error = false;                        // Water flow in Meassure-Pipe OK

// Queue object to store incoming JSON commands (up to 10)
QueueHandle_t queueIn;

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
PCF_Pump FiltrationPump(FILTRATION_PUMP, FILTRATION_PUMP);
PCF_Pump PhPump(PH_PUMP, PH_PUMP, NO_LEVEL, FILTRATION_PUMP, storage.pHPumpFR, storage.pHTankVol, storage.AcidFill);
PCF_Pump ChlPump(CHL_PUMP, CHL_PUMP, NO_LEVEL, FILTRATION_PUMP, storage.ChlPumpFR, storage.ChlTankVol, storage.ChlFill);
PCF_Pump RobotPump(ROBOT_PUMP, ROBOT_PUMP, NO_TANK, FILTRATION_PUMP);
PCF_Pump SaltPump(SALT_PUMP, SALT_PUMP, NO_TANK, FILTRATION_PUMP);
PCF_Pump HeatPump(HEAT_PUMP, HEAT_PUMP, NO_TANK, FILTRATION_PUMP);
PCF_Pump SolarHeatPump(HEAT_ON, HEAT_ON, NO_TANK, FILTRATION_PUMP);

// Naming for six MotorValve Instances
char Motorvalve_1[] = "ELD_TREPPE";
char Motorvalve_2[] = "ELD_HINTEN";
char Motorvalve_3[] = "WP_VORLAUF";
char Motorvalve_4[] = "WP_MISCHER";
char Motorvalve_5[] = "BODENABLAUF";
char Motorvalve_6[] = "SOLAR_VALVE";

// The six motorvalves of the system (instanciate the MotorValve class)
MotorValve ELD_Treppe(ESD_TRE_OPEN,ESD_TRE_CLOSE, STARTANGLE_0, MAX_90 , TIMETOMAX_90, COUNTER_CLOCKWISE, PCF8574_3, Motorvalve_1);
MotorValve ELD_Hinten(ESD_HIN_OPEN,ESD_HIN_CLOSE, STARTANGLE_0, MAX_90 , TIMETOMAX_90, COUNTER_CLOCKWISE, PCF8574_3, Motorvalve_2);
MotorValve WP_Vorlauf(WP_VL_OPEN,WP_VL_CLOSE, STARTANGLE_0, MAX_90 , TIMETOMAX_90, COUNTER_CLOCKWISE, PCF8574_3, Motorvalve_3);
MotorValve WP_Mischer(WP_M_OPEN,WP_M_CLOSE, STARTANGLE_0, MAX_45, TIMETOMAX_45, COUNTER_CLOCKWISE, PCF8574_3, Motorvalve_4);
MotorValve Bodenablauf(BODEN_OPEN,BODEN_CLOSE, STARTANGLE_0, MAX_90 , TIMETOMAX_90, COUNTER_CLOCKWISE, PCF8574_4, Motorvalve_5);
MotorValve Solarvalve(SOLAR_OPEN,SOLAR_CLOSE, STARTANGLE_0, MAX_90 , TIMETOMAX_90, COUNTER_CLOCKWISE, PCF8574_4, Motorvalve_6);

// PIDs instances
//Specify the direction and initial tuning parameters
PID PhPID(&storage.PhValue, &storage.PhPIDOutput, &storage.Ph_SetPoint, storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd, PhPID_DIRECTION);
PID OrpPID(&storage.OrpValue, &storage.OrpPIDOutput, &storage.Orp_SetPoint, storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd, OrpPID_DIRECTION);

// Publishing tasks handles to notify them
static TaskHandle_t pubSetTaskHandle;
static TaskHandle_t pubMeasTaskHandle;

// Mutex to share access to I2C bus among two tasks: AnalogPoll and StatusLights
static SemaphoreHandle_t mutex;

// Functions prototypes
void StartTime(void);
void readLocalTime(void);
bool loadConfig(void);
bool saveConfig(void);
void WiFiEvent(WiFiEvent_t);
void initTimers(void);
void connectToWiFi(void);
void mqttInit(void);                     
void InitTFT(void);
void ResetTFT(void);
void PublishSettings(void);
void SetPhPID(bool);
void SetOrpPID(bool);
int  freeRam (void);
void AnalogInit(void);
void FlowInit(void);
void Flow2Init(void);
void TempInit(void);
bool saveParam(const char*,uint8_t );
unsigned stack_hwm();
void stack_mon(UBaseType_t&);
void info();
void checkNTPServer(void *pvParameters);
void getTemperature();

// Functions used as Tasks
void PoolMaster(void*);
void AnalogPoll(void*);
void pHRegulation(void*);
void OrpRegulation(void*);
void getTemp(void*);
void ProcessCommand(void*);
void FlowMeasures(void*);
void SettingsPublish(void*);
void MeasuresPublish(void*);
void StatusLights(void*);

// Setup
void setup()
{
  //Serial port for debug info
  Serial.begin(115200);

  // Set appropriate debug level. The level is defined in PoolMaster.h
  Debug.setDebugLevel(DEBUG_LEVEL);
  Debug.timestampOn();

  //get board info
  info();

  // check if Wifi is connected
  bool checkWifi();
  
  // Initialize Nextion TFT
  InitTFT();
  ResetTFT();

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

  //Define pins directions
  pinMode(SALT_POL, OUTPUT);
  pinMode(LIGHT_POOL, OUTPUT);
  pinMode(LIGHT_ROOM, OUTPUT);
  pinMode(RELAY_R0, OUTPUT);
  pinMode(RELAY_R1, OUTPUT);
  pinMode(RELAY_R2, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  // As the relays on the board are activated by a LOW level, set all levels HIGH at startup
  digitalWrite(SALT_POL,HIGH);
  digitalWrite(LIGHT_POOL,HIGH);
  digitalWrite(LIGHT_ROOM,HIGH);
  digitalWrite(RELAY_R0,HIGH);
  digitalWrite(RELAY_R1,HIGH);
  digitalWrite(RELAY_R2,HIGH);

// Warning: pins used here have no pull-ups, provide external ones
  pinMode(FLOW, INPUT);
  pinMode(FLOW2, INPUT);
  pinMode(WATER_MAX_LVL, INPUT);
  pinMode(WATER_MIN_LVL, INPUT);

  // Initialize watch-dog
  esp_task_wdt_init(WDT_TIMEOUT, true);

  //Initialize MQTT
  mqttInit();

  // Initialize WiFi events management (on connect/disconnect)
  WiFi.onEvent(WiFiEvent);
  initTimers();
  connectToWiFi();

  delay(500);    // let task start-up and wait for connection
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Initialize the mDNS library.
  while (!MDNS.begin("PoolMaster")) {
    Debug.print(DBG_ERROR,"Error setting up MDNS responder!");
    delay(1000);
  }
  MDNS.addService("http", "tcp", SERVER_PORT);

  // Start I2C for ADS1115 and status lights through PCF8574A
  Wire.begin(I2C_SDA,I2C_SCL);

  // Init RTC DS3231
  if (rtc.begin()) {
  Debug.print(DBG_INFO, "[RTC] RTC module detected");
  } else {
    Debug.print(DBG_ERROR, "[RTC] Failed to detect RTC module");
  }

  // Check if lostPower flag of RTC ist true
  if (rtc.lostPower()) {
    Debug.print(DBG_WARNING, "[RTC] RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Config NTP, get time and set system time. This is done here in setup then every day at midnight
  // note: in timeinfo struct, months are from 0 to 11 and years are from 1900. Thus the corrections
  // to pass arguments to setTime which needs months from 1 to 12 and years from 2000...
  // DST (Daylight Saving Time) is managed automatically
  StartTime();
  readLocalTime();
  setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year-100);
  Debug.print(DBG_INFO,"%d/%02d/%02d %02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());

  // Init pH, ORP and PSI analog measurements
  AnalogInit();

  // Init Flow measurements
  FlowInit();

  // Init Flow2 measurements
  Flow2Init();
  
  // Init Water and Air temperatures measurements
  TempInit();

  // Clear status LEDs
  Wire.beginTransmission(0x38);
  Wire.write((uint8_t)0xFF);
  Wire.endTransmission();

  // Set pinMode of PCF8574 (Second for Pumps)
  for(int i=0;i<8;i++) {
    pcf8574.pinMode(i, OUTPUT);
  }
	Debug.print(DBG_DEBUG,"[PCF8574_RELAY] Init pcf8574...");
	if (pcf8574.begin()){
    Debug.print(DBG_DEBUG,"[PCF8574_RELAY] OK");
	}else{
    Debug.print(DBG_DEBUG,"[PCF8574_RELAY] not OK");
	}

  // Set pinMode of PCF8574_3 (Third for MotorValves)
  for(int i=0;i<8;i++) {
    pcf8574_3.pinMode(i, OUTPUT);
  }
	Debug.print(DBG_DEBUG,"[PCF8574_3_RELAY] Init pcf8574_3...");
	if (pcf8574_3.begin()){
    Debug.print(DBG_DEBUG,"[PCF8574_3_RELAY] OK");
	}else{
    Debug.print(DBG_DEBUG,"[PCF8574_3_RELAY] not OK");
	}

  // Set pinMode of PCF8574_4 (Fourth for MotorValves and Waterfill, ...)
  for(int i=0;i<8;i++) {
    pcf8574_4.pinMode(i, OUTPUT);
  }
	Debug.print(DBG_DEBUG,"[PCF8574_4_RELAY] Init pcf8574_4...");
	if (pcf8574_4.begin()){
    Debug.print(DBG_DEBUG,"[PCF8574_4_RELAY] OK");
	}else{
    Debug.print(DBG_DEBUG,"[PCF8574_4_RELAY] not OK");
	}

  // As the relays on the board are activated by a LOW level, set all levels HIGH at startup (Second PCF8574)
  pcf8574.digitalWrite(FILTRATION_PUMP, HIGH);
  pcf8574.digitalWrite(PH_PUMP, HIGH);
  pcf8574.digitalWrite(CHL_PUMP, HIGH);
  pcf8574.digitalWrite(SALT_PUMP, HIGH);
  pcf8574.digitalWrite(HEAT_PUMP, HIGH);
  pcf8574.digitalWrite(HEAT_ON, HIGH);
  pcf8574.digitalWrite(ROBOT_PUMP, HIGH);
  pcf8574.digitalWrite(WATER_FILL, HIGH);

  // As the relays on the board are activated by a LOW level, set all levels HIGH at startup (Second PCF8574)
  pcf8574_3.digitalWrite(ESD_TRE_OPEN, HIGH);
  pcf8574_3.digitalWrite(ESD_TRE_CLOSE, HIGH);
  pcf8574_3.digitalWrite(ESD_HIN_OPEN, HIGH);
  pcf8574_3.digitalWrite(ESD_HIN_CLOSE, HIGH);
  pcf8574_3.digitalWrite(WP_VL_OPEN, HIGH);
  pcf8574_3.digitalWrite(WP_VL_CLOSE, HIGH);
  pcf8574_3.digitalWrite(WP_M_OPEN, HIGH);
  pcf8574_3.digitalWrite(WP_M_CLOSE, HIGH);

    // As the relays on the board are activated by a LOW level, set all levels HIGH at startup (Second PCF8574)
  pcf8574_4.digitalWrite(BODEN_OPEN, HIGH);
  pcf8574_4.digitalWrite(BODEN_CLOSE, HIGH);
  pcf8574_4.digitalWrite(SOLAR_OPEN, HIGH);
  pcf8574_4.digitalWrite(SOLAR_CLOSE, HIGH);
  pcf8574_4.digitalWrite(SPARE_I_OPEN, HIGH);
  pcf8574_4.digitalWrite(SPARE_I_CLOSE, HIGH);
  pcf8574_4.digitalWrite(WATER_FILL, HIGH);
  pcf8574_4.digitalWrite(SPARE_II, HIGH);

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
  SolarHeatPump.SetMaxUpTime(0);      //no runtime limit for the Solar 3-way-valve
  HeatPump.SetMaxUpTime(0);           //no runtime limit for the heatpump
  SaltPump.SetMaxUpTime(0);           //no runtime limit for the saltmanager
  RobotPump.SetMaxUpTime(0);          //no runtime limit for the robot pump

  PhPump.SetFlowRate(storage.pHPumpFR);
  PhPump.SetTankVolume(storage.pHTankVol);
  PhPump.SetTankFill(storage.AcidFill);
  PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit * 1000);

  ChlPump.SetFlowRate(storage.ChlPumpFR);
  ChlPump.SetTankVolume(storage.ChlTankVol);
  ChlPump.SetTankFill(storage.ChlFill);
  ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit * 1000);

  // Start filtration pump at power-on if within scheduled time slots -- You can choose not to do this and start pump manually
  if (storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
    FiltrationPump.Start();
  else FiltrationPump.Stop();

  // pumps off at start
  RobotPump.Stop();
  HeatPump.Stop();
  SaltPump.Stop();
  SolarHeatPump.Stop();

  // Calibrate MotorValves at start
  ELD_Treppe.calibrate();
  ELD_Hinten.calibrate();
  WP_Vorlauf.calibrate();
  WP_Mischer.calibrate();
  Bodenablauf.calibrate();
  Solarvalve.calibrate();


  // Create queue for external commands
  queueIn = xQueueCreate((UBaseType_t)QUEUE_ITEMS_NBR,(UBaseType_t)QUEUE_ITEM_SIZE);

  // Create loop tasks in the scheduler.
  //------------------------------------
  int app_cpu = xPortGetCoreID();

  Debug.print(DBG_DEBUG,"Creating loop Tasks");

  // Create I2C sharing mutex
  mutex = xSemaphoreCreateMutex();

  // Check NTP-Status
  xTaskCreatePinnedToCore(
    checkNTPServer,
    "NTP Checker",
    2048,
    NULL,
    1,
    nullptr,
    app_cpu
    );

  // Analog measurement polling task
  xTaskCreatePinnedToCore(
    AnalogPoll,
    "AnalogPoll",
    3072,
    NULL,
    1,
    nullptr,
    app_cpu
  );

  // MQTT commands processing
  xTaskCreatePinnedToCore(
    ProcessCommand,
    "ProcessCommand",
    4096,
    NULL,
    1,
    nullptr,
    app_cpu
  );

  // PoolMaster: Supervisory task
  xTaskCreatePinnedToCore(
    PoolMaster,
    "PoolMaster",
    3072,
    NULL,
    1,
    nullptr,
    app_cpu
  );

  // Temperatures measurement
  xTaskCreatePinnedToCore(
    getTemp,
    "GetTemp",
    3072,
    NULL,
    1,
    nullptr,
    app_cpu
  );
  
 // ORP regulation loop
    xTaskCreatePinnedToCore(
    OrpRegulation,
    "ORPRegulation",
    2048,
    NULL,
    1,
    nullptr,
    app_cpu
  );

  // pH regulation loop
    xTaskCreatePinnedToCore(
    pHRegulation,
    "pHRegulation",
    2048,
    NULL,
    1,
    nullptr,
    app_cpu
  );

  // Status lights display
  xTaskCreatePinnedToCore(
    StatusLights,
    "StatusLights",
    2048,
    NULL,
    1,
    nullptr,
    app_cpu
  ); 

    // Flow measurement polling task
  xTaskCreatePinnedToCore(
    FlowMeasures,
    "FlowMeasures",
    3072,
    NULL,
    1,
    nullptr,
    app_cpu
  ); 

  // Measures MQTT publish 
  xTaskCreatePinnedToCore(
    MeasuresPublish,
    "MeasuresPublish",
    3072,
    NULL,
    1,
    &pubMeasTaskHandle,               // needed to notify task later
    app_cpu
  );

  // MQTT Settings publish 
  xTaskCreatePinnedToCore(
    SettingsPublish,
    "SettingsPublish",
    5120,
    NULL,
    1,
    &pubSetTaskHandle,                // needed to notify task later
    app_cpu
  );

  // Initialize OTA (On The Air update)
  //-----------------------------------
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname("PoolMaster");
  ArduinoOTA.setPasswordHash("510179c0211489b9625a5f2e41da8469"); // hash de Fgixy001
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Debug.print(DBG_INFO,"Start updating %s",type);
  });
  ArduinoOTA.onEnd([]() {
  Debug.print(DBG_INFO,"End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    esp_task_wdt_reset();           // reset Watchdog as upload may last some time...
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Debug.print(DBG_ERROR,"Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Debug.print(DBG_ERROR,"Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Debug.print(DBG_ERROR,"Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Debug.print(DBG_ERROR,"Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Debug.print(DBG_ERROR,"Receive Failed");
    else if (error == OTA_END_ERROR)     Debug.print(DBG_ERROR,"End Failed");
  });

  ArduinoOTA.begin();

  //display remaining RAM/Heap space.
  Debug.print(DBG_DEBUG,"[memCheck] Stack: %d bytes - Heap: %d bytes",stack_hwm(),freeRam());

  // Start loops tasks
  Debug.print(DBG_INFO,"Init done, starting loop tasks");
  startTasks = true;

  delay(1000);          // wait for tasks to start

}

bool loadConfig()
{
  nvs.begin("PoolMaster",true);

  storage.ConfigVersion         = nvs.getUChar("ConfigVersion",0);
  storage.Ph_RegulationOnOff    = nvs.getBool("Ph_RegOnOff",true);
  storage.Orp_RegulationOnOff   = nvs.getBool("Orp_RegOnOff",false);  
  storage.AutoMode              = nvs.getBool("AutoMode",true);
  storage.Salt_Chlor            = nvs.getBool("Salt_Chlor",true);
  storage.SaltMode              = nvs.getBool("SaltMode",true);
  storage.SaltPolarity          = nvs.getBool("SaltPolarity",false);
  storage.WinterMode            = nvs.getBool("WinterMode",false);
  storage.WaterHeat             = nvs.getBool("Heat",false);
  storage.ValveMode             = nvs.getBool("ValveMode",true);
  storage.CleanMode             = nvs.getBool("CleanMode",false);
  storage.ValveSwitch           = nvs.getBool("ValveSwitch",false);
  storage.FiltrationDuration    = nvs.getUChar("FiltrDuration",12);
  storage.FiltrationStart       = nvs.getUChar("FiltrStart",8);
  storage.FiltrationStop        = nvs.getUChar("FiltrStop",20);
  storage.FiltrationStartMin    = nvs.getUChar("FiltrStartMin",8);
  storage.FiltrationStopMax     = nvs.getUChar("FiltrStopMax",22);
  storage.SolarStartMin         = nvs.getUChar("SolarStartMin",11);
  storage.SolarStopMax          = nvs.getUChar("SolarStopMax",18);
  storage.DelayPIDs             = nvs.getUChar("DelayPIDs",0);
  storage.PhPumpUpTimeLimit     = nvs.getULong("PhPumpUTL",900);
  storage.ChlPumpUpTimeLimit    = nvs.getULong("ChlPumpUTL",2500);
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
  storage.TempExternal          = nvs.getDouble("TempExternal",3.);
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
  storage.TempValue             = nvs.getDouble("TempValue",18.);
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

  nvs.end();

  Debug.print(DBG_INFO,"%d",storage.ConfigVersion);
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d, %d, %d, %d, %d ,%d ,%d",storage.Ph_RegulationOnOff,storage.Orp_RegulationOnOff,storage.AutoMode,storage.Salt_Chlor,storage.SaltMode,storage.SaltPolarity,storage.WinterMode,storage.WaterHeat,storage.ValveMode,storage.CleanMode,storage.ValveSwitch);
  Debug.print(DBG_INFO,"%d, %d, %d, %d, %d, %d, %d, %d",storage.FiltrationDuration,storage.FiltrationStart,storage.FiltrationStop,
              storage.FiltrationStartMin,storage.FiltrationStopMax,storage.SolarStartMin,storage.SolarStopMax,storage.DelayPIDs);
  Debug.print(DBG_INFO,"%d, %d, %d",storage.PhPumpUpTimeLimit,storage.ChlPumpUpTimeLimit,storage.PublishPeriod);
  Debug.print(DBG_INFO,"%d, %d, %d, %d",storage.PhPIDWindowSize,storage.OrpPIDWindowSize,storage.PhPIDwindowStartTime,storage.OrpPIDwindowStartTime);
  Debug.print(DBG_INFO,"%3.1f, %4.0f, %3.1f, %3.1f, %3.1f, %3.0f, %3.0f, %3.1f, %3.0f, %3.0f, %4.1f, %8.6f, %9.6f, %11.6f, %11.6f, %3.1f, %3.1f, %3.0f",
              storage.Ph_SetPoint,storage.Orp_SetPoint,storage.PSI_HighThreshold,storage.FLOW_Pulse,storage.FLOW2_Pulse,storage.FLOW_HighThreshold,storage.FLOW2_HighThreshold,
              storage.PSI_MedThreshold,storage.FLOW_MedThreshold,storage.FLOW2_MedThreshold,storage.WaterTempLowThreshold,storage.WaterTemp_SetPoint,storage.TempExternal,
              storage.pHCalibCoeffs0,storage.pHCalibCoeffs1,storage.OrpCalibCoeffs0,storage.OrpCalibCoeffs1,storage.SaltDiff,
              storage.PSICalibCoeffs0,storage.PSICalibCoeffs1);
  Debug.print(DBG_INFO,"%8.0f, %3.0f, %3.0f, %6.0f, %3.0f, %3.0f, %7.0f, %7.0f, %4.2f, %4.2f, %4.0f, %3.0f, %3.0f",
              storage.Ph_Kp,storage.Ph_Ki,storage.Ph_Kd,storage.Orp_Kp,storage.Orp_Ki,storage.Orp_Kd,
              storage.PhPIDOutput,storage.OrpPIDOutput,storage.TempValue,storage.PhValue,storage.OrpValue,storage.PSIValue,storage.FLOWValue,storage.FLOW2Value);
  Debug.print(DBG_INFO,"%3.0f, %3.0f, %3.0f, %3.0f, %3.1f, %3.1f ",storage.AcidFill,storage.ChlFill,storage.pHTankVol,storage.ChlTankVol,
              storage.pHPumpFR,storage.ChlPumpFR);

  return (storage.ConfigVersion == CONFIG_VERSION);
}

bool saveConfig()
{
  nvs.begin("PoolMaster",false);

  size_t i = nvs.putUChar("ConfigVersion",storage.ConfigVersion);
  i += nvs.putBool("Ph_RegOnOff",storage.Ph_RegulationOnOff);
  i += nvs.putBool("Orp_RegOnOff",storage.Orp_RegulationOnOff);  
  i += nvs.putBool("AutoMode",storage.AutoMode);
  i += nvs.putBool("Salt_Chlor",storage.Salt_Chlor);
  i += nvs.putBool("SaltMode",storage.SaltMode);
  i += nvs.putBool("SaltPolarity",storage.SaltPolarity);
  i += nvs.putBool("WinterMode",storage.WinterMode);
  i += nvs.putBool("Heat",storage.WaterHeat);
  i += nvs.putBool("ValveMode",storage.ValveMode);
  i += nvs.putBool("CleanMode",storage.CleanMode);
  i += nvs.putBool("ValveSwitch",storage.ValveSwitch);
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
  i += nvs.putDouble("TempExternal",storage.TempExternal);
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
  i += nvs.putDouble("TempValue",storage.TempValue);
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

  nvs.end();

  Debug.print(DBG_INFO,"Bytes saved: %d / %d\n",i,sizeof(storage));
  return (i == sizeof(storage)) ;

}

// functions to save any type of parameter (4 overloads with same name but different arguments)

bool saveParam(const char* key, uint8_t val)
{
  nvs.begin("PoolMaster",false);
  size_t i = nvs.putUChar(key,val);
  return(i == sizeof(val));
}

bool saveParam(const char* key, bool val)
{
  nvs.begin("PoolMaster",false);
  size_t i = nvs.putBool(key,val);
  return(i == sizeof(val));
}

bool saveParam(const char* key, unsigned long val)
{
  nvs.begin("PoolMaster",false);
  size_t i = nvs.putULong(key,val);
  return(i == sizeof(val));
}

bool saveParam(const char* key, double val)
{
  nvs.begin("PoolMaster",false);
  size_t i = nvs.putDouble(key,val);
  return(i == sizeof(val));
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
    Debug.print(DBG_DEBUG,"[stack_mon] %s: %d bytes",pcTaskGetTaskName(NULL), hwm);
  }  
}

// Get exclusive access of I2C bus
void lockI2C(){
  xSemaphoreTake(mutex, portMAX_DELAY);
}

// Release I2C bus access
void unlockI2C(){
  xSemaphoreGive(mutex);  
}

// Set time parameters, including DST
void StartTime() {
  configTime(0, 0, "0.pool.ntp.org", "1.pool.ntp.org", "192.168.178.1"); // 3 possible NTP servers
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 3);                      // configure local time with automatic DST
  tzset();
  int retry = 0;
  const int retry_count = 15;
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
    Debug.print(DBG_INFO, ".");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  Debug.print(DBG_INFO, "");
  Debug.print(DBG_INFO, "[NTP] NTP configured");
}

void readLocalTime() {
  // Attempts to read the time from the RTC module
  if (rtc.begin()) {
    Debug.print(DBG_INFO, "[RTC] RTC module detected");
    DateTime now = rtc.now();
    char timeString[20];
    sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    Debug.print(DBG_INFO, timeString);

    // Compare RTC time with NTP time
    time_t ntp_time;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000U)) {
      ntp_time = mktime(&timeinfo);
      double diff = difftime(ntp_time, now.unixtime());
      if ((diff > 0.0) || (diff < 0.0)) {
        // RTC is behind or ahead of NTP, adjust RTC time
        rtc.adjust(DateTime(ntp_time));
        Debug.print(DBG_INFO, "[RTC] RTC time adjusted to NTP time");

        // Show new RTC time after adjustment
        DateTime adjustedTime = rtc.now();
        char adjustedTimeString[20];
        sprintf(adjustedTimeString, "%04d-%02d-%02d %02d:%02d:%02d", adjustedTime.year(), adjustedTime.month(), adjustedTime.day(), adjustedTime.hour(), adjustedTime.minute(), adjustedTime.second());
        Debug.print(DBG_INFO, "[RTC] New RTC time: %s", adjustedTimeString);
      } else {
        // RTC and NTP times are the same, do nothing
        Debug.print(DBG_INFO, "[RTC] RTC time and NTP time are the same, no adjustment necessary");
      }
    } else {
      Debug.print(DBG_WARNING, "[RTC] Failed to obtain NTP time, no adjustment made");
    }
  } else {
    // Attempts to retrieve the time via the NTP server network.
    if (!getLocalTime(&timeinfo, 5000U)) {
      Debug.print(DBG_WARNING, "[RTC] Failed to obtain time");
      StartTime();
    }
    char timeString[20];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Debug.print(DBG_INFO, "[RTC] %s", timeString);
  }
}

void checkNTPServer(void* parameter) {
  while (1) {
    vTaskDelay(24 * 60 * 60 * 1000 / portTICK_PERIOD_MS); // Wait 24 hours before checking the NTP server again
    if (WiFi.isConnected()) {
      if (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
        Debug.print(DBG_INFO, "[NTP] NTP sync failed, retrying...");
        Debug.print(DBG_WARNING, "[NTP] NTP sync failed, retrying...");
        StartTime();
      }
    } else {
      Debug.print(DBG_WARNING, "[NTP] WiFi not connected, cannot sync time");
    }
  }
}

// Notify PublishSettings task 
void PublishSettings()
{
  xTaskNotifyGive(pubSetTaskHandle);
}

// Notify PublishMeasures task
void PublishMeasures()
{
  xTaskNotifyGive(pubMeasTaskHandle);
}

//board info
void info(){
  esp_chip_info_t out_info;
  esp_chip_info(&out_info);
  Debug.print(DBG_INFO,"CPU frequency       : %dMHz",ESP.getCpuFreqMHz());
  Debug.print(DBG_INFO,"CPU Cores           : %d",out_info.cores);
  Debug.print(DBG_INFO,"Flash size          : %dMB",ESP.getFlashChipSize()/1000000);
  Debug.print(DBG_INFO,"Free RAM            : %d bytes",ESP.getFreeHeap());
  Debug.print(DBG_INFO,"Min heap            : %d bytes",esp_get_free_heap_size());
  Debug.print(DBG_INFO,"tskIDLE_PRIORITY    : %d",tskIDLE_PRIORITY);
  Debug.print(DBG_INFO,"confixMAX_PRIORITIES: %d",configMAX_PRIORITIES);
  Debug.print(DBG_INFO,"configTICK_RATE_HZ  : %d",configTICK_RATE_HZ);
}


// Pseudo loop, which deletes loopTask of the Arduino framework
void loop()
{
  delay(1000);
  vTaskDelete(nullptr);
}