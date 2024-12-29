// MQTT related functions for PoolMaster, including WiFi functions
// Use JSON version 6

#undef __STRICT_ANSI__
#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"

AsyncMqttClient mqttClient;
extern Preferences nvs;

bool MQTTConnection = false;                                    // Status of connection to broker
static TimerHandle_t mqttReconnectTimer;                        // Reconnect timer for MQTT
static TimerHandle_t wifiReconnectTimer;                        // Reconnect timer for WiFi

#ifdef MQTT_LOGIN
 static const char* MqttServerClientID = MQTT_SERVER_ID;            
 static const char* MqttServerLogin    = MQTT_SERVER_LOGIN;                
 static const char* MqttServerPwd      = MQTT_SERVER_PWD;
#else
static const char* PoolTopicAPI       = "Home/Pool/API";
static const char* PoolTopicStatus    = "Home/Pool/status";
static const char* PoolTopicError     = "Home/Pool/Err";
static const char* PoolTopicMode      = "POOL/Pool_Mode";
static const char* SolarTopicMode     = "POOL/Solar_Mode";
#endif

// Functions prototypes
void initTimers(void);
void mqttInit(void);
void mqttErrorPublish(const char* );
void publishPoolMode(int event);
void publishSolarMode(int event);
void connectToWiFi(void);
void connectToMqtt(void);
void WiFiEvent(WiFiEvent_t );
void onMqttConnect(bool);
void onMqttDisconnect(AsyncMqttClientDisconnectReason);
void onMqttSubscribe(uint16_t, uint8_t);
void onMqttUnSubscribe(uint16_t);
void onMqttMessage(char* , char* , AsyncMqttClientMessageProperties , size_t , size_t , size_t );
void onMqttPublish(uint16_t);
void UpdateWiFi(bool);
int  freeRam(void);
bool saveParam(const char*,uint8_t );
bool saveParam(const char*,bool );
bool saveParam(const char*,unsigned long );
bool saveParam(const char*,double );
bool saveParam(const char*,String );

void initTimers() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));
}

void mqttInit() {
  //Init Async MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setWill(PoolTopicStatus,1,true,"{\"PoolMaster Online\":0}");

  // Read the MQTT server IP and port from the NVS.
  String mqttServerIPString = nvs.getString("MQTT_IP", storage.MQTT_IP.toString());
  IPAddress mqttServerIP;
  if (!mqttServerIP.fromString(mqttServerIPString)) {
    Debug.print(DBG_ERROR, "[MQTT] Failed to parse MQTT server IP from NVS");
    return;
  }
  Debug.print(DBG_DEBUG, "[MQTT] MQTT server IP address set to: %s", mqttServerIP.toString().c_str());
  uint16_t mqttServerPort = nvs.getUShort("MQTT_PORT", storage.MQTT_PORT);
  Debug.print(DBG_DEBUG, "[MQTT] MQTT server port: %d", mqttServerPort);
  
  mqttClient.setServer(mqttServerIP, mqttServerPort);

  // Check whether the login to MQTT should be enabled or not.
  bool MQTTLogin = nvs.getBool("MQTTLOGIN_OnOff", true);
  if (MQTTLogin) {
    Debug.print(DBG_INFO, "[MQTT] MQTT Login with credentials is turned on in NVS");
    mqttClient.setCredentials(MQTT_SERVER_LOGIN, MQTT_SERVER_PWD);
    mqttClient.setClientId(MQTT_SERVER_ID);
  } 
}

void mqttErrorPublish(const char* Payload){
  if (mqttClient.publish(PoolTopicError, 1, true, Payload) !=0)
  {
    Debug.print(DBG_WARNING,"[MQTT] Payload: %s - Payload size: %d",Payload, sizeof(Payload));
  }
  else
  {
    Debug.print(DBG_WARNING,"[MQTT] Unable to publish the following payload: %s",Payload);
  }
}

void publishPoolMode(int event) {
  static unsigned long lastPublishedTime = 0;
  static int lastPublishedValue = -1;
  unsigned long now = millis();

  if (event != lastPublishedValue || now - lastPublishedTime >= storage.PublishPeriod * 1000) {
    lastPublishedValue = event;
    mqttClient.publish(PoolTopicMode, 1, true, event == 1 ? "auto" : event == 2 ? "on" : "off");
    lastPublishedTime = now;
  }
}

void publishSolarMode(int event) {
  static unsigned long lastPublishedTime = 0;
  static int lastPublishedValue = -1;
  unsigned long now = millis();

  if (event != lastPublishedValue || now - lastPublishedTime >= storage.PublishPeriod * 1000) {
    lastPublishedValue = event;
    mqttClient.publish(SolarTopicMode, 1, true, event == 1 ? "pool" : event == 2 ? "puffer" : "off");
    lastPublishedTime = now;
  }
}

void connectToMqtt(){
  Debug.print(DBG_INFO,"[WiFi] Connecting to MQTT...");
  mqttClient.connect();
}

  void connectToWiFi()
  {
    Debug.print(DBG_INFO, "[WiFi] Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("PoolMaster");

    // Check if WiFi should be turned on or off
    bool wifiOn = nvs.getBool("WIFI_OnOff", false);
    if (!wifiOn)
    {
      Debug.print(DBG_INFO, "[WiFi] WiFi turned off in NVS");
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      return;
    }

    // Load the stored credentials from NVS
    String ssid_str = "";
    String pass_str = "";
    ssid_str = nvs.getString("SSID", "");
    pass_str = nvs.getString("WIFI_PASS", "");
    if (ssid_str != "" && pass_str != "")
    {
      Debug.print(DBG_INFO, "[WiFi] Using stored credentials...");
      WiFi.begin(ssid_str.c_str(), pass_str.c_str());
    }
    else
    {
      Debug.print(DBG_INFO, "[WiFi] Using default credentials...");
      WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    }
  }

  void WiFiEvent(WiFiEvent_t event)
  {
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Debug.print(DBG_INFO, "[WiFi] Connected to: %s", WiFi.SSID().c_str());
      Debug.print(DBG_INFO, "[WiFi] IP address: %s", WiFi.localIP().toString().c_str());
      Debug.print(DBG_INFO, "[WiFi] Hostname: %s", WiFi.getHostname());
      Debug.print(DBG_INFO, "[WiFi] Connecting to MQTT...");
      UpdateWiFi(true);
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Debug.print(DBG_WARNING, "[WiFi] Connection lost");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      UpdateWiFi(false);
      break;
    default:
      break;
    }
  }

// Once connected to MQTT broker, subscribe to the PoolTopicAPI topic in order to receive future commands
// then publish the "online" message on the "status" topic. If Ethernet connection is ever lost
// "status" will switch to "offline". Very useful to check that the system is alive and functional
void onMqttConnect(bool sessionPresent){
  Debug.print(DBG_INFO,"[MQTT] Connected to MQTT, present session: %d",sessionPresent);
  mqttClient.subscribe(PoolTopicAPI,2);
  mqttClient.publish(PoolTopicStatus,1,true,"{\"PoolMaster Online\":1}");
  MQTTConnection = true;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason){
  Debug.print(DBG_WARNING,"[MQTT] Disconnected from MQTT");
  if(WiFi.isConnected()) xTimerStart(mqttReconnectTimer,0);
  MQTTConnection = false;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos){
    Debug.print(DBG_INFO,"[MQTT] Subscribe ack., qos: %d",qos);
}

void onMqttUnSubscribe(uint16_t packetId){
    Debug.print(DBG_INFO,"[MQTT] unSubscribe ack.");
}

void onMqttPublish(uint16_t packetId){
    Debug.print(DBG_VERBOSE,"[MQTT] Publish ack., packetId: %d",packetId);
}

// MQTT callback
// This function is called when messages are published on the MQTT broker on the PoolTopicAPI topic to which we subscribed
// Add the received command to a message queue for later processing and exit the callback
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  //Pool commands. This check might be redundant since we only subscribed to this topic
  if (strcmp(topic,PoolTopicAPI)==0)
  {
    char Command[100] = "";

    for (uint8_t i=0 ; i<len ; i++){
      Command[i] = payload[i];
    }
    if (xQueueSendToBack(queueIn, &Command, 0) == pdPASS)
    {
      Debug.print(DBG_INFO,"[MQTT] Command added to queue: %s",Command);
    }
    else
    {
      Debug.print(DBG_ERROR,"[MQTT] Queue full, command: %s not added", Command);
    }
    Debug.print(DBG_DEBUG,"[MQTT] FreeRam: %d Queued messages: %d",freeRam(),uxQueueMessagesWaiting(queueIn));
  }
}