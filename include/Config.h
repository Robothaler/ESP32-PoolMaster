// Firmware revision
#define FIRMW           "ESP-3.5"
#define TFT_FIRMW       "TFT-2.0"

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION  12

#define DEBUG_LEVEL     DBG_INFO     // Possible levels : NONE/ERROR/WARNING/INFO/DEBUG/VERBOSE

// WiFi credentials
// ------  Credentials are stored in include/credentials.h
// #define WIFI_NETWORK "YOUR_WIFI_NETWORK_ID"
// #define WIFI_PASSWORD "YOUR_WIFI_NETWORK_PWD"
// #define OTA_PWDHASH "Your_OTA_password_hash"

#ifdef DEVT
  #define HOSTNAME      "PoolMaster_Dev"
#else
  #define HOSTNAME      "PoolMaster"
#endif 

//IFTTT key to trigger event
#define IFTTT_key       "/trigger/PoolMaster/with/key/Your_IFTTT_Key"

// Mail parameters and credentials
//#define EMAIL_ALERT    // Comment this line to disable Email alerting
#define SMTP_HOST       "your smtp server"
#define SMTP_PORT       587      // check the port number
#define AUTHOR_EMAIL    "your email address"
#define AUTHOR_LOGIN    "your user name"
#define AUTHOR_PASSWORD "your password"
#define RECIPIENT_EMAIL "your recipient email address"


// PID Directions (either DIRECT or REVERSE depending on Ph/Orp correction vs water properties)
#define PhPID_DIRECTION   REVERSE
#define OrpPID_DIRECTION  DIRECT

// Polarity of the Electric-Connection (The polarity should be changed every 4 hours to prevent calcification of the electrolysis plates.)
#define POLARITY_DIRECT   0
#define POLARITY_REVERSE  1

// Define RELAY-PINS for all Pumps (Second PCF8574_I)
#define FILTRATION_PUMP   P0   // Filtration-Pump
#define HEAT_PUMP         P1   // Heat-Pump
#define SALT_PUMP         P2   // Salt-Manager
#define ROBOT_PUMP	      P3   // Cleaningrobot
#define PH_PUMP           P4   // PH-Pump
#define CHL_PUMP          P5   // Chlorine-Pump
#define SOLAR_PUMP        P6   // Solar, 3-way-valve for Warmwater Solarpanels
#define SALT_POL          P7   // Salt-Manager Polarity DIRECT / REVERSE

// Define RELAY-PINS for MotorValves (Third PCF8574_II)
#define ESD_TRE_OPEN      P0   // ESD-Treppe (open)
#define ESD_TRE_CLOSE     P1   // ESD-Treppe (close)
#define ESD_HIN_OPEN      P2   // ESD-Hinten (open)
#define ESD_HIN_CLOSE     P3   // ESD-Hinten (close)
#define WP_VL_OPEN        P4   // WP-Vorlauf (open)
#define WP_VL_CLOSE       P5   // WP-Vorlauf (close)
#define WP_M_OPEN	        P6   // WP-Mischer (open)
#define WP_M_CLOSE        P7   // WP-Mischer (close)

// Define RELAY-PINS for MotorValves (Fourth PCF8574_III)
#define BODEN_OPEN	      P0   // Bodenablauf (open)
#define BODEN_CLOSE       P1   // Bodenablauf (close)
#define SOLAR_OPEN        P2   // SOLAR (open)
#define SOLAR_CLOSE       P3   // SOLAR (close)
#define SPARE_I_OPEN      P4   // SPARE (open)
#define SPARE_I_CLOSE     P5   // SPARE (close)
#define WATER_FILL        P6   // Freshwater-Valve to fillup the Pool -> Needs to be combined with Levelsensors in the skimmer
#define HEAT_ON           P7   // Switch for Heatdemand


#define LIGHT_POOL         7   // (WAR 27)    Pool Spotlight
#define LIGHT_ROOM        10   // (WAR 4)     Serviceroom light
#define RELAY_R0          13   // (WAR 25)    Spare I
#define RELAY_R1          14   // (WAR 26)    Spare II
#define RELAY_R2          21   // (WAR 33)    Spare III
//#define RELAY_R3        38   // (WAR 32)    Spare IV
//#define RELAY_R4        45   // (ist NEU)   Spare V
//#define RELAY_R5        37   // (ist NEU)   Spare VI

//Digital input pins connected to Flow-Meter additional security for Filtrationpump and dosing
#define FLOW              11   // (WAR 39)    Flow-Meter in Main-Pipe to be sure Filtrationpump is running
#define FLOW2              6   // (WAR 36)    Flow-Meter in Measure-Pipe to be sure water is flowing to get accurate values of ph and orp meter

//Digital input pins connected to level reed switches in pool to indicate low or high water level
//LOW = Switch is closed / HIGH = Switch is open
#define WATER_MAX_LVL     38  // (WAR 34)
#define WATER_MIN_LVL     45  // (WAR 35)

//Digital input pins connected to level reed switches in canister indicate low pH or Chlorine level
//LOW = Switch is open
#define PH_LVL            15   // (WAR 34)
#define CHL_LVL           16   // (WAR 35)

//One wire bus for the air/water temperature measurement
#define ONE_WIRE_BUS_A     4   //  (WAR 18)
#define ONE_WIRE_BUS_W     5   //  (WAR 19)
#define MAX_ADDRESSES      5   // 5 sensors max on the bus

//I2C bus for analog measurement with ADS1115 of pH, ORP and water pressure 
//and status LED through PCF8574A 
#define I2C_SDA			       8  //  (WAR 21)
#define I2C_SCL			       9  //  (WAR 22)
#define PCF8574_ADR       0x38 // for Status-LEDs
#define PCF8574_I_ADR     0x3F // for External Relais for 230V Apliances
#define PCF8574_II_ADR    0x3D // for Motorvalves
#define PCF8574_III_ADR   0x3E // for additional Motorvalves and Waterfillvalve

//Type of pH and Orp sensors acquisition :
//INT_ADS1115 : single ended signal with internal ADS1115 ADC (default)
//EXT_ADS1115 : differential signal with external ADS1115 ADC (Loulou74 board)
#define EXT_ADS1115
#define INT_ADS1115_ADDR  ADS1115ADDRESS+1 // 0x49 is default address -> ADS1115ADDRESS+1
#define PH_ADS1115_ADDR   ADS1115ADDRESS+2 // 0x4A is default address -> ADS1115ADDRESS+2
#define ORP_ADS1115_ADDR  ADS1115ADDRESS+3 // 0x4B is default address -> ADS1115ADDRESS+3

// Buzzer
#define BUZZER             2  //  (WAR 2)

// MotorValve Constants
#define STARTANGLE_0       0   // StartAngle for MotorValves
#define MAX_45            45   // MaxAngle 45 degree for MotorValves
#define MAX_90            90   // MaxAngle 90 degree for MotorValves
#define TIMETOMAX_45      45   // Time to reach to maximum in Seconds
#define TIMETOMAX_90      90   // Time to reach to maximum in Seconds

#define WDT_TIMEOUT       10

// Server port
#define SERVER_PORT       8060

//OTA port
#define OTA_PORT          8063

//OTA host name
#define OTA_HOST          "PoolMaster"

//12bits (0,06°C) temperature sensors resolution
#define TEMPERATURE_RESOLUTION 12

//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishement of measurement data
#define PUBLISHINTERVAL   30000

#define MQTT_SERVER_IP    IPAddress(192, 168, 178, 55)
#define MQTT_SERVER_PORT  1883

// Uncomment if MQTT broker needs login/pwd
//#define MQTT_LOGIN 				
#define MQTT_SERVER_ID    "ESP32Pool"		   // MQTT server ID

// -------> credentials are defined in credentials.h
//#define MQTT_SERVER_LOGIN "Your_Login"
//#define MQTT_SERVER_PWD   "Your_Pwd" 

// Topic used in DEVT or OPER mode

#ifdef DEVT
  #define POOLTOPIC       "Home/Pool6/"
#else
  #define POOLTOPIC       "Home/Pool/"
#endif 

// Robot pump timing
#define ROBOT_DELAY       60     // Robot start delay after filtration in mn
#define ROBOT_DURATION    90     // Robot cleaning duration in mn

//Display timeout before blanking
//-------------------------------
#define TFT_SLEEP         60000L 

// Loop tasks scheduling parameters
//---------------------------------
// T1:  AnalogPoll
// T2:  PoolServer
// T3:  PoolMaster
// T4:  getTemp
// T5:  readBME280
// T6:  OrpRegulation
// T7:  SaltRegulation
// T8:  pHRegulation
// T9:  FlowMeasures
// T10: StatusLights
// T11: PublishMeasures
// T12: PublishSettings 

//Periods 
// Task11 period is initialized with PUBLISHINTERVAL and can be changed dynamically
#define PT1               125
#define PT2               500
#define PT3               500
#define PT4               1000 / (1 << (12 - TEMPERATURE_RESOLUTION))
#define PT5               1000
#define PT6               1000
#define PT7               1000
#define PT8               1000
#define PT9               1000
#define PT10              3000
#define PT11              30000 

//Start offsets to spread tasks along time
// Task1 has no delay
#define DT2               190/portTICK_PERIOD_MS
#define DT3               310/portTICK_PERIOD_MS
#define DT4               440/portTICK_PERIOD_MS
#define DT5               500/portTICK_PERIOD_MS
#define DT6               560/portTICK_PERIOD_MS
#define DT7               565/portTICK_PERIOD_MS  //-> Timing is similar to ORP Regulation because just one loop is running
#define DT8               920/portTICK_PERIOD_MS
#define DT9               1060/portTICK_PERIOD_MS
#define DT10              100/portTICK_PERIOD_MS
#define DT11              570/portTICK_PERIOD_MS
#define DT12              940/portTICK_PERIOD_MS

//#define CHRONO                    // Activate tasks timings traces for profiling
//#define SIMU                      // Used to simulate pH/ORP sensors. Very simple simulation:
                                    // the sensor value is computed from the output of the PID 
                                    // loop to reach linearly the theorical value produced by this
                                    // output after one hour