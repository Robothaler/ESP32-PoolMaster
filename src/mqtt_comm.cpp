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
 static const char* MqttServerClientID  = MQTT_SERVER_ID;            
 static const char* MqttServerLogin     = MQTT_SERVER_LOGIN;                
 static const char* MqttServerPwd       = MQTT_SERVER_PWD;
#else
static const char* PoolTopicAPI         = "Home/Pool/API";
static const char* PoolTopicStatus      = "Home/Pool/status";
static const char* PoolTopicError       = "Home/Pool/Err";
static const char* PoolTopicMode        = "POOL/Pool_Mode";
static const char* SolarTopicMode       = "POOL/Solar_Mode";
static const char* SolarControlTemp     = "/SolarControl/pt1";
static const char* SolarControlLWT      = "SolarControl/LWT";
static const char* SolarControlSolPump  = "SolarControl/SOLAR_PUMP";
static const char* SolarControlSolVal   = "SolarControl/VALVE_POOL";

#endif

// Functions prototypes
void StartTime();
void readLocalTime();
void initTimers(void);
void mqttInit(void);
void mqttErrorPublish(const char* Payload);
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
bool saveParam(const char* key, uint8_t val);
bool saveParam(const char* key, bool val);
bool saveParam(const char* key, unsigned long val);
bool saveParam(const char* key, String val);
bool saveParam(const char* key, const uint8_t* val, size_t size);
bool saveParam(const char* key, double val);

void initTimers() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));
}

void mqttInit() {
  // Event-Handler registrieren
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setWill(PoolTopicStatus, 1, true, "{\"PoolMaster Online\":0}");

  // Verwende direkt die geladenen Werte aus storage
  Debug.print(DBG_DEBUG, "[MQTT] MQTT server IP address set to: %s", storage.MQTT_IP.toString().c_str());
  Debug.print(DBG_DEBUG, "[MQTT] MQTT server port: %d", storage.MQTT_PORT);

  // Validierung der IP-Adresse
  if (storage.MQTT_IP == IPAddress(0, 0, 0, 0) || storage.MQTT_IP == IPAddress(255, 255, 255, 255) || storage.MQTT_IP[0] == 0) {
    Debug.print(DBG_ERROR, "[MQTT] Invalid MQTT IP address detected: %s", storage.MQTT_IP.toString().c_str());
    storage.MQTTLOGIN_OnOff = false;
    if (nvs.begin("PoolMaster", false)) {
      nvs.putBool("MQTTLOGIN_OnOff", false);
      nvs.end();
      Debug.print(DBG_INFO, "[MQTT] MQTT login disabled in NVS due to invalid IP");
    }
    return;  // Keine Blockierung
  }

  // Setze den Server nur hier
  mqttClient.setServer(storage.MQTT_IP, storage.MQTT_PORT);

  // MQTT-Login-Option
  if (storage.MQTTLOGIN_OnOff) {
    Debug.print(DBG_INFO, "[MQTT] MQTT Login with credentials is turned on in NVS");
    mqttClient.setCredentials(MQTT_SERVER_LOGIN, MQTT_SERVER_PWD);
    mqttClient.setClientId(MQTT_SERVER_ID);
  } else {
    mqttClient.setClientId(MQTT_SERVER_ID);
  }
}

static unsigned long lastMqttErrorTime = 0;
const unsigned long MQTT_ERROR_INTERVAL = 60000; // 1 Minute
void mqttErrorPublish(const char* Payload){
  if (millis() - lastMqttErrorTime > MQTT_ERROR_INTERVAL) {
  if (mqttClient.publish(PoolTopicError, 1, true, Payload) !=0)
  {
    Debug.print(DBG_WARNING,"[MQTT] Payload: %s - Payload size: %d",Payload, sizeof(Payload));
  }
  else
  {
    Debug.print(DBG_WARNING,"[MQTT] Unable to publish the following payload: %s",Payload);
  }
  lastMqttErrorTime = millis();
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

void connectToMqtt() {
  if (!storage.WIFI_OnOff || WiFi.status() != WL_CONNECTED) {
    Debug.print(DBG_INFO, "[MQTT] WiFi off or not connected, skipping MQTT");
    return;
  }

  if (!storage.MQTTLOGIN_OnOff) {
    Debug.print(DBG_INFO, "[MQTT] MQTT login disabled in NVS, skipping connection");
    return;
  }

  if (mqttClient.connected()) {
    Debug.print(DBG_DEBUG, "[MQTT] Already connected, skipping attempt");
    return;
  }

  // Da AsyncMqttClient keine direkte connecting()-Methode hat, verlassen wir uns auf den Timer
  Debug.print(DBG_DEBUG, "[MQTT] Attempting connection to %s:%d", storage.MQTT_IP.toString().c_str(), storage.MQTT_PORT);
  mqttClient.connect();

  // Verzögerung für Debugging (optional, später entfernen)
  delay(100);  // Gibt AsyncTCP Zeit, den Fehler zu melden
}

void connectToWiFi() {
  Debug.print(DBG_INFO, "[WiFi] Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("PoolMaster");

  if (!storage.WIFI_OnOff) {
    Debug.print(DBG_INFO, "[WiFi] WiFi turned off in NVS");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    return;
  }

  String ssid_str = nvs.getString("SSID", "");
  String pass_str = nvs.getString("WIFI_PASS", "");
  if (ssid_str != "" && pass_str != "") {
    Debug.print(DBG_INFO, "[WiFi] Using stored credentials...");
    WiFi.begin(ssid_str.c_str(), pass_str.c_str());
  } else {
    Debug.print(DBG_INFO, "[WiFi] Using default credentials...");
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
  }
}

  void WiFiEvent(WiFiEvent_t event) {
    Debug.print(DBG_DEBUG, "[WiFi] Event received: %d", event);
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_START:
        Debug.print(DBG_INFO, "[WiFi] STA started");
        break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Debug.print(DBG_INFO, "[WiFi] Connected to: %s", WiFi.SSID().c_str());
        Debug.print(DBG_INFO, "[WiFi] Hostname: %s", WiFi.getHostname());
        UpdateWiFi(true);
        break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Debug.print(DBG_INFO, "[WiFi] Got IP: %s", WiFi.localIP().toString().c_str());
        if (storage.WIFI_OnOff) {
          Debug.print(DBG_INFO, "[NTP] Starting NTP sync after IP...");
          StartTime();
          readLocalTime();
          Debug.print(DBG_INFO, "[Time] Current time (NTP): %d/%02d/%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
          Debug.print(DBG_INFO, "[WiFi] Connecting to MQTT...");
          connectToMqtt();  // mqttInit() wird bereits im Setup aufgerufen
        }
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Debug.print(DBG_WARNING, "[WiFi] Connection lost");
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        UpdateWiFi(false);
        break;
      default:
        Debug.print(DBG_DEBUG, "[WiFi] Unhandled event: %d", event);
        break;
    }
  }

// Once connected to MQTT broker, subscribe to the PoolTopicAPI topic in order to receive future commands
// then publish the "online" message on the "status" topic. If Ethernet connection is ever lost
// "status" will switch to "offline". Very useful to check that the system is alive and functional
void onMqttConnect(bool sessionPresent) {
  Debug.print(DBG_INFO, "[MQTT] Connected to MQTT, present session: %d", sessionPresent);
  mqttClient.subscribe(PoolTopicAPI, 2);
  mqttClient.subscribe(SolarControlTemp, 2);  // Abonnieren des Temperatur-Topics
  mqttClient.subscribe(SolarControlLWT, 2);   // Abonnieren des LWT-Topics (optional)
  mqttClient.subscribe(SolarControlSolPump, 2); // Abonnieren des Solar-Pumpen-Topics
  mqttClient.subscribe(SolarControlSolVal, 2);  // Abonnieren des Solar-Ventil-Topics
  mqttClient.publish(PoolTopicStatus, 1, true, "{\"PoolMaster Online\":1}");
  MQTTConnection = true;
  char resetPayload[64];
  snprintf(resetPayload, sizeof(resetPayload), "{\"ResetReason\":\"%s\"}", resetReasonToString(storage.ResetReason));
  mqttClient.publish(POOLTOPIC"ResetReason", 1, true, resetPayload);
  Debug.print(DBG_INFO, "[MQTT] Published ResetReason: %s", resetPayload);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Debug.print(DBG_WARNING, "[MQTT] Disconnected from MQTT, reason: %d", static_cast<int>(reason));
  MQTTConnection = false;
  if (WiFi.isConnected() && storage.MQTTLOGIN_OnOff) {
    Debug.print(DBG_DEBUG, "[MQTT] Scheduling reconnect in 2 seconds");
    xTimerStop(mqttReconnectTimer, 0);  // Stoppe Timer, falls noch aktiv
    xTimerStart(mqttReconnectTimer, 0); // Neu starten
  }
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
    if (queueIn == NULL) {
    Debug.print(DBG_ERROR, "[MQTT] queueIn is NULL");
    return;
    }  
    
    if (len >= 150) {
        Debug.print(DBG_ERROR, "[MQTT] Payload too large: %d", len);
        return;
    }

    // Dynamische Speicherallokation für die Nachricht
    char* command = (char*)heap_caps_malloc(len + 1, MALLOC_CAP_8BIT);
    if (command == NULL) {
        Debug.print(DBG_ERROR, "[MQTT] Failed to allocate memory for command");
        return;
    }
    strncpy(command, payload, len);
    command[len] = '\0';

    // Pool-Kommandos
    if (strcmp(topic, PoolTopicAPI) == 0)
    {
        if (xQueueSendToBack(queueIn, command, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Debug.print(DBG_INFO, "[MQTT] Command added to queue: %s", command);
        }
        else
        {
            Debug.print(DBG_ERROR, "[MQTT] Queue full, command: %s not added", command);
        }
    }
    // Solarkollektor-Temperatur
    else if (strcmp(topic, SolarControlTemp) == 0)
    {
        char tempStr[32];
        snprintf(tempStr, sizeof(tempStr), "{\"TempSolar\":%.*s}", len, payload);
        if (xQueueSendToBack(queueIn, tempStr, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Debug.print(DBG_INFO, "[MQTT] Solar temperature added to queue: %s", tempStr);
        }
        else
        {
            Debug.print(DBG_ERROR, "[MQTT] Queue full, solar temperature: %s not added", tempStr);
        }
    }
    // LWT-Status
    else if (strcmp(topic, SolarControlLWT) == 0)
    {
        char lwtStr[32];
        snprintf(lwtStr, sizeof(lwtStr), "{\"SolarLWT\":\"%.*s\"}", len, payload);
        if (xQueueSendToBack(queueIn, lwtStr, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Debug.print(DBG_INFO, "[MQTT] Solar LWT status added to queue: %s", lwtStr);
        }
        else
        {
            Debug.print(DBG_ERROR, "[MQTT] Queue full, solar LWT: %s not added", lwtStr);
        }
    }
    // Solar-Pumpensteuerung
    else if (strcmp(topic, SolarControlSolPump) == 0)
    {
        char solPumpStr[32];
        snprintf(solPumpStr, sizeof(solPumpStr), "{\"SolarPump\":%.*s}", len, payload);
        if (xQueueSendToBack(queueIn, solPumpStr, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Debug.print(DBG_INFO, "[MQTT] Solar pump command added to queue: %s", solPumpStr);
        }
        else
        {
            Debug.print(DBG_ERROR, "[MQTT] Queue full, solar pump command: %s not added", solPumpStr);
        }
    }
    // Solar-Ventilsteuerung
    else if (strcmp(topic, SolarControlSolVal) == 0)
    {
        char solValStr[32];
        snprintf(solValStr, sizeof(solValStr), "{\"SolarValve\":%.*s}", len, payload);
        if (xQueueSendToBack(queueIn, solValStr, pdMS_TO_TICKS(10)) == pdPASS)
        {
            Debug.print(DBG_INFO, "[MQTT] Solar valve command added to queue: %s", solValStr);
        }
        else
        {
            Debug.print(DBG_ERROR, "[MQTT] Queue full, solar valve command: %s not added", solValStr);
        }
    }
    else
    {
        Debug.print(DBG_WARNING, "[MQTT] Unknown topic: %s", topic);
    }

    // Speicher freigeben
    heap_caps_free(command);

    // Debugging-Informationen
    Debug.print(DBG_DEBUG, "[MQTT] FreeRam: %d Queued messages: %d", freeRam(), uxQueueMessagesWaiting(queueIn));
}