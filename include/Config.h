// Firmware revision
#define FIRMW "ESP-3.01"

// WiFi credentials
// #define WIFI_NETWORK "YOUR_WIFI_NETWORK_ID"
// #define WIFI_PASSWORD "YOUR_WIFI_NETWORK_PWD"

//IFTTT key to trigger event
#define IFTTT_key "/trigger/PoolMaster/with/key/Your_IFTTT_Key"

// PID Directions (either DIRECT or REVERSE depending on Ph/Orp correction vs water properties)
#define PhPID_DIRECTION REVERSE
#define OrpPID_DIRECTION DIRECT

// Polarity of the Electric-Connection (The polarity should be changed every 4 hours to prevent calcification of the electrolysis plates.)
#define POLARITY_DIRECT  0
#define POLARITY_REVERSE 1

// Define RELAY-PINS for all Pumps (first PCF8574)
#define FILTRATION_PUMP P0   // Filtration-Pump
#define PH_PUMP         P1   // PH-Pump
#define CHL_PUMP        P2   // Chlorine-Pump
#define SALT_PUMP       P3   // Salt-Manager
#define HEAT_PUMP       P4   // Heat-Pump
#define HEAT_ON         P5   // Solar, 3-way-valve for Warmwater Solarpanels
#define ROBOT_PUMP	    P6   // Cleaningrobot
#define SPARE           P7   // SPARE

// Define RELAY-PINS for MotorValves (second PCF8574_3)
#define ESD_TRE_OPEN    P0   // ESD-Treppe (open)
#define ESD_TRE_CLOSE   P1   // ESD-Treppe (close)
#define ESD_HIN_OPEN    P2   // ESD-Hinten (open)
#define ESD_HIN_CLOSE   P3   // ESD-Hinten (close)
#define WP_VL_OPEN      P4   // WP-Vorlauf (open)
#define WP_VL_CLOSE     P5   // WP-Vorlauf (close)
#define WP_M_OPEN	    P6   // WP-Mischer (open)
#define WP_M_CLOSE      P7   // WP-Mischer (close)

// Define RELAY-PINS for MotorValves (third PCF8574_4)
#define BODEN_OPEN	    P0   // Bodenablauf (open)
#define BODEN_CLOSE     P1   // Bodenablauf (close)
#define SOLAR_OPEN      P2   // SOLAR (open)
#define SOLAR_CLOSE     P3   // SOLAR (close)
#define SPARE_I_OPEN    P4   // SPARE (open)
#define SPARE_I_CLOSE   P5   // SPARE (close)
#define WATER_FILL      P6   // Freshwater-Valve to fillup the Pool -> Needs to be combined with Levelsensors in the skimmer
#define SPARE_II        P7   // SPARE

#define SALT_POL        32   // Salt-Manager Polarity DIRECT / REVERSE
#define LIGHT_POOL      27   // Pool Spotlight
#define LIGHT_ROOM       4   // Serviceroom light
#define RELAY_R0        25   // Spare I
#define RELAY_R1        26   // Spare II
#define RELAY_R2        33   // Spare II

// MotorValve Constants
#define STARTANGLE_0     0   // StartAngle for MotorValves
#define MAX_45          45   // MaxAngle 45 degree for MotorValves
#define MAX_90          90   // MaxAngle 90 degree for MotorValves
#define TIMETOMAX_45    45   // Time to reach to maximum in Seconds
#define TIMETOMAX_90    90   // Time to reach to maximum in Seconds

//Digital input pins connected to Flow-Meter additional security for Filtrationpump and dosing
#define FLOW            39   // Flow-Meter in Main-Pipe to be sure Filtrationpump is running
#define FLOW2           36   // Flow-Meter in Measure-Pipe to be sure water is flowing to get accurate values of ph and orp meter

//Digital input pins connected to level reed switches in pool to indicate low or high water level
#define WATER_MAX_LVL   34
#define WATER_MIN_LVL   35

//One wire bus for the air/water temperature measurement
#define ONE_WIRE_BUS_A  18
#define ONE_WIRE_BUS_W  19

//I2C bus for analog measurement with ADS1115 of pH, ORP and water pressure 
//and status LED through PCF8574A 
#define I2C_SDA			21
#define I2C_SCL			22

// Buzzer
#define BUZZER           2

#define WDT_TIMEOUT     10

// Server port
#define SERVER_PORT 8060

//OTA port
#define OTA_PORT    8063

//12bits (0,06°C) temperature sensors resolution
#define TEMPERATURE_RESOLUTION 12

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION 25

//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishes of measurement data
#define PUBLISHINTERVAL 30000

#define MQTT_SERVER_IP IPAddress(192, 168, 178, 55)
#define MQTT_SERVER_PORT 1883

//Display timeout before blanking
#define TFT_SLEEP 60000L 

// Loop tasks scheduling parameters
//---------------------------------
// T1:  AnalogPoll
// T2:  PoolServer
// T3:  PoolMaster
// T4:  getTemp
// T5:  OrpRegulation
// T6:  pHRegulation
// T7:  FlowMeasures
// T8:  StatusLights
// T9:  PublishMeasures
// T10: PublishSettings 

//Periods 
// Task10 period is initialized with PUBLISHINTERVAL and can be changed dynamically
#define PT1 125
#define PT2 500
#define PT3 500
#define PT4 1000 / (1 << (12 - TEMPERATURE_RESOLUTION))
#define PT5 1000
#define PT6 1000
#define PT7 1000
#define PT8 3000
#define PT9 30000

//Start offsets to spread tasks along time
// Task1 has no delay
#define DT2  190/portTICK_PERIOD_MS
#define DT3  310/portTICK_PERIOD_MS
#define DT4  440/portTICK_PERIOD_MS
#define DT5  560/portTICK_PERIOD_MS
#define DT6  920/portTICK_PERIOD_MS
#define DT7  1060/portTICK_PERIOD_MS
#define DT8  100/portTICK_PERIOD_MS
#define DT9  570/portTICK_PERIOD_MS
#define DT10 940/portTICK_PERIOD_MS