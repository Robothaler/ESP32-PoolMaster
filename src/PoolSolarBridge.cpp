#include "PoolSolarBridge.h"
#include "Config.h"
#include "PoolMaster.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <cstring>

extern Arduino_DebugUtils Debug;
extern PCF_Pump FiltrationPump;

bool saveParam(const char* key, String val);
bool saveParam(const char* key, unsigned long val);

static String s_readToken;
static String s_solarBaseUrl;
static uint32_t s_pollIntervalMs = 10000;

static char s_httpMode[20] = "";
static uint8_t s_httpModeEp1 = 0;
static bool s_httpCirc = false;
static bool s_httpIllum = false;
static bool s_httpPoolModeHw = false;
static int s_lastHttpCode = 0;
static uint32_t s_lastPollAttemptMs = 0;
static uint32_t s_lastPollSuccessMs = 0;

/** Default SolarControl LAN base (NVS key psolBrUrl overrides). */
static const char kDefaultSolarBaseUrl[] = "http://192.168.178.176";

static bool solarExternalRegulationWindowOk() {
    return storage.AutoMode && storage.SolarLocExt && storage.SolarMode &&
           FiltrationPump.IsRunning() &&
           (FiltrationPump.UpTime / 1000 / 60 > 5) &&
           (hour() >= (int)storage.SolarStartMin) &&
           (hour() < (int)storage.SolarStopMax);
}

int poolSolarBridgeExternalSolarPublishEvent(void) {
    if (!solarExternalRegulationWindowOk())
        return 3;
    if (storage.WaterSTemp < storage.WaterTemp_SetPoint &&
        storage.SolarTemp > storage.WaterSTemp + SOLAR_EXT_COLLECTOR_DELTA_MIN)
        return 1;
    if (storage.WaterSTemp >= storage.WaterTemp_SetPoint ||
        storage.SolarRLTemp + SOLAR_EXT_RL_STOP_MARGIN <= storage.WaterSTemp)
        return 2;
    return -1;
}

bool poolSolarBridgeSolarModeRequest(void) {
    return poolSolarBridgeExternalSolarPublishEvent() == 1;
}

void poolSolarBridgeLoadFromNvs(Preferences& p) {
    s_readToken = p.getString("psolBrTok", "");
    s_solarBaseUrl = p.getString("psolBrUrl", kDefaultSolarBaseUrl);
    uint32_t pollS = p.getUInt("psolPollS", 10);
    if (pollS < 5u) pollS = 5u;
    if (pollS > 300u) pollS = 300u;
    s_pollIntervalMs = pollS * 1000UL;
}

bool poolSolarBridgeSetToken(const String& tok) {
    s_readToken = tok;
    return saveParam("psolBrTok", s_readToken);
}

bool poolSolarBridgeSetSolarBaseUrl(const String& url) {
    s_solarBaseUrl = url;
    return saveParam("psolBrUrl", s_solarBaseUrl);
}

bool poolSolarBridgeSetPollIntervalSec(uint32_t sec) {
    if (sec < 5u) sec = 5u;
    if (sec > 300u) sec = 300u;
    s_pollIntervalMs = sec * 1000UL;
    return saveParam("psolPollS", (unsigned long)sec);
}

String poolSolarBridgeTokenMaskedForSettings() {
    return s_readToken.length() ? String(F("***")) : String();
}

const String& poolSolarBridgeSolarBaseUrlRef() { return s_solarBaseUrl; }

uint32_t poolSolarBridgePollIntervalSec() { return s_pollIntervalMs / 1000UL; }

bool poolSolarBridgeAuthorizeRead(AsyncWebServerRequest* req) {
    if (s_readToken.length() == 0)
        return true;
    if (!req)
        return false;
    const AsyncWebHeader* h = req->getHeader("Authorization");
    if (!h)
        return false;
    const String v = h->value();
    static const char kBearer[] = "Bearer ";
    if (!v.startsWith(kBearer))
        return false;
    return v.substring((int)strlen(kBearer)) == s_readToken;
}

String poolSolarBridgeBuildReadJson() {
    StaticJsonDocument<384> doc;
    doc["schema"] = "pool-solar-bridge/v1";
    doc["poolTemp_C"] = (float)storage.WaterSTemp;
    doc["poolSollTemp_C"] = (float)storage.WaterTemp_SetPoint;
    const bool solarReq = poolSolarBridgeSolarModeRequest();
    doc["solarModeRequest"] = solarReq;
    doc["poolSolarRequest"] = solarReq;
    doc["solarRequest"] = solarReq;
    doc["uptime_s"] = (uint32_t)(millis() / 1000);
    doc["firmware"] = Firmw;
    String out;
    serializeJson(doc, out);
    return out;
}

int poolSolarBridgeHttpPollLastCode() { return s_lastHttpCode; }

int32_t poolSolarBridgeHttpPollAgeMs() {
    if (s_lastPollSuccessMs == 0)
        return -1;
    return (int32_t)(millis() - s_lastPollSuccessMs);
}

const char* poolSolarBridgeHttpPollMode() { return s_httpMode; }

uint8_t poolSolarBridgeHttpPollModeEp1() { return s_httpModeEp1; }

bool poolSolarBridgeHttpPollCirculationOn() { return s_httpCirc; }

bool poolSolarBridgeHttpPollIlluminationOn() { return s_httpIllum; }

bool poolSolarBridgeHttpPollPoolModeRequestHw() { return s_httpPoolModeHw; }

static void applySolarPayload(const String& payload) {
    StaticJsonDocument<768> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Debug.print(DBG_DEBUG, "[PoolSolarBridge] solar JSON parse error: %s", err.c_str());
        if (storage.SolarLocExt)
            storage.SolarOnline = false;
        return;
    }
    const char* sch = doc["schema"] | "";
    if (strcmp(sch, "pool-solar-bridge/v1") != 0) {
        Debug.print(DBG_DEBUG, "[PoolSolarBridge] solar schema mismatch");
        if (storage.SolarLocExt)
            storage.SolarOnline = false;
        return;
    }
    if (doc.containsKey("roofTemp_C"))
        storage.solarRoofTemp = doc["roofTemp_C"].as<float>();
    if (doc.containsKey("boilerTemp_C"))
        storage.solarBoilerTemp = doc["boilerTemp_C"].as<float>();
    if (doc.containsKey("storageTemp_C"))
        storage.solarStorageTemp = doc["storageTemp_C"].as<float>();
    if (doc.containsKey("backflowTemp_C"))
        storage.solarBackflowTemp = doc["backflowTemp_C"].as<float>();
    else if (doc.containsKey("returnTemp_C"))
        storage.solarBackflowTemp = doc["returnTemp_C"].as<float>();
    if (doc.containsKey("pumpOn"))
        storage.solarPumpRunning = doc["pumpOn"].as<bool>();
    if (doc.containsKey("valvePool"))
        storage.solarValvePool = doc["valvePool"].as<bool>();
    if (doc.containsKey("valveFeedbackEp3"))
        storage.solarValveOK = doc["valveFeedbackEp3"].as<bool>();

    const char* mode = doc["mode"] | "";
    strncpy(s_httpMode, mode, sizeof(s_httpMode) - 1);
    s_httpMode[sizeof(s_httpMode) - 1] = '\0';
    s_httpModeEp1 = doc["modeEp1"] | 0;
    s_httpCirc = doc["circulationOn"] | false;
    s_httpIllum = doc["illuminationOn"] | false;
    s_httpPoolModeHw = doc["poolModeRequestHw"] | false;
    s_lastPollSuccessMs = millis();

    /* Mirror SolarControl LAN data into legacy regulation / Nextion / MQTT fields (external mode). */
    if (storage.SolarLocExt) {
        storage.SolarOnline = true;
        if (doc.containsKey("roofTemp_C"))
            storage.SolarTemp = (double)storage.solarRoofTemp;
        if (doc.containsKey("pumpOn"))
            storage.SolarPumpStatus = storage.solarPumpRunning ? 1 : 0;
        if (doc.containsKey("valvePool"))
            storage.ValveStatus = storage.solarValvePool ? 1 : 0;
        if (doc.containsKey("backflowTemp_C") || doc.containsKey("returnTemp_C"))
            storage.SolarRLTemp = (double)storage.solarBackflowTemp;
    }
}

void poolSolarBridgePollTick() {
    if (s_solarBaseUrl.length() == 0)
        return;
    if (!wifiStaConnected())
        return;
    const uint32_t now = millis();
    if (getDurationSafe(s_lastPollAttemptMs, now) < s_pollIntervalMs)
        return;
    s_lastPollAttemptMs = now;

    String url = s_solarBaseUrl;
    while (url.length() > 0 && url.endsWith("/"))
        url.remove(url.length() - 1);
    url += F("/api/pool-solar/v1/solar");

    WiFiClient client;
    HTTPClient http;
    http.setTimeout(4500);
    http.setConnectTimeout(3000);
    if (!http.begin(client, url)) {
        s_lastHttpCode = -1;
        if (storage.SolarLocExt)
            storage.SolarOnline = false;
        return;
    }
    const int code = http.GET();
    s_lastHttpCode = code;
    if (code == 200) {
        applySolarPayload(http.getString());
    } else {
        Debug.print(DBG_DEBUG, "[PoolSolarBridge] solar GET %s → HTTP %d", url.c_str(), code);
        if (storage.SolarLocExt)
            storage.SolarOnline = false;
    }
    http.end();
}
