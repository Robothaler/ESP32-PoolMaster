// WebUI backend — HTTP + WebSocket + persistent data logger for ESP32-PoolMaster
// Extends the shared AsyncWebServer (port 80) defined in Ota.cpp.
//
// ── Log files (SPIFFS) ────────────────────────────────────────────────────────
//   /sc.bin  current sensor log  (append-only, 400 KB max = 25,600 records × 16 B)
//   /sb.bin  backup sensor log   (previous rotation)
//   /ec.bin  current event log   (append-only, 32 KB max = 4,096 events × 8 B)
//   /eb.bin  backup event log
//
// ── Storage estimate ─────────────────────────────────────────────────────────
//   Delta thresholds → ~30-200 records/day depending on pool activity
//   At  80 rec/day: 400 KB / 80 = 320 days/file → 640 days (~1.8 yr) with backup
//   At  30 rec/day: 400 KB / 30 = 853 days/file → 1706 days (~4.7 yr) with backup
//
// ── REST API ─────────────────────────────────────────────────────────────────
//   GET /api/log     ?from=<ts>&to=<ts>&pts=<N>   sensor records (JSON)
//   GET /api/events  ?from=<ts>&to=<ts>            pump events (JSON)
//   GET /api/loginfo                               storage stats
//   POST /api/logclear                             delete all log files

#include "Ota.h"
#include "PoolMaster.h"
#include "Config.h"
#include "WebUI.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <time.h>
#include <esp_system.h>
#include <WiFi.h>
#ifdef MATTER_ENABLED
#include "MatterBridge.h"
#endif

extern Arduino_DebugUtils Debug;
extern Preferences nvs;
extern void connectToWiFi();
extern void connectToMqtt();

// ─── WebSocket ────────────────────────────────────────────────────────────────
static AsyncWebSocket ws("/ws");

// ─── Short-term RAM ring buffer (last 2 h, fast WebSocket push) ───────────────
static constexpr uint16_t HIST_SIZE = 240; // 240 × 30 s = 2 h
struct HistPoint { uint32_t ts; float ph, orp, psi, waterTemp, airTemp; };
static HistPoint  histBuf[HIST_SIZE];
static uint16_t   histHead  = 0;
static uint16_t   histCount = 0;
static uint32_t   lastHistMs = 0;

static void recordHistory() {
    if (millis() - lastHistMs < 30000UL) return;
    lastHistMs = millis();
    histBuf[histHead] = { (uint32_t)(millis()/1000),
        (float)storage.PhValue, (float)storage.OrpValue,
        (float)storage.PSIValue, (float)storage.WaterSTemp, (float)storage.AirTemp };
    histHead = (histHead + 1) % HIST_SIZE;
    if (histCount < HIST_SIZE) histCount++;
}

// ─── LONG-TERM SPIFFS LOGGER ─────────────────────────────────────────────────
namespace Logger {

#pragma pack(push, 1)
// 16-byte sensor record (binary, aligned)
struct SensorRec {
    uint32_t ts;        // Unix timestamp (seconds since 1970)
    int16_t  ph100;     // pH × 100  (e.g. 720 = pH 7.20)
    int16_t  orp;       // ORP [mV]
    int16_t  psi100;    // bar × 100
    int16_t  wTemp10;   // °C × 10
    int16_t  airT10;    // °C × 10
    uint8_t  pumps;     // bitmask: bit0=filt 1=ph 2=chl 3=heat 4=salt 5=robot 6=solar 7=fill
    uint8_t  modes;     // bitmask: bit0=auto 1=phReg 2=orpReg 3=fill 4=heat 5=salt 6=winter
};
// 8-byte event record
struct EventRec {
    uint32_t ts;        // Unix timestamp
    uint8_t  dev;       // device index (same as pumps bitmask positions above)
    uint8_t  state;     // 1 = on, 0 = off
    uint16_t pad;
};
#pragma pack(pop)
static_assert(sizeof(SensorRec) == 16, "SensorRec must be 16 bytes");
static_assert(sizeof(EventRec)  ==  8, "EventRec must be 8 bytes");

// ── File limits ───────────────────────────────────────────────────────────────
static constexpr size_t   SENSOR_MAX_BYTES = 400UL * 1024; // 400 KB → 25,600 records
static constexpr size_t   EVENT_MAX_BYTES  = 32UL  * 1024; // 32 KB  → 4,096 events
static constexpr size_t   SPIFFS_MIN_FREE  = 80UL  * 1024; // keep 80 KB free

// ── Delta thresholds (only write when any value exceeds these) ────────────────
static constexpr int16_t  PH_DELTA    = 3;   // 0.03 pH
static constexpr int16_t  ORP_DELTA   = 8;   // 8 mV
static constexpr int16_t  PSI_DELTA   = 5;   // 0.05 bar
static constexpr int16_t  TEMP_DELTA  = 2;   // 0.2 °C
static constexpr int16_t  AIRT_DELTA  = 5;   // 0.5 °C

// ── Timing ────────────────────────────────────────────────────────────────────
static constexpr uint32_t MIN_INTERVAL_S  = 60;   // never write more often than 1/min
static constexpr uint32_t MAX_INTERVAL_S  = 600;  // force write every 10 min even if stable

// ── Runtime state ────────────────────────────────────────────────────────────
static SensorRec  s_last      = {};
static uint8_t    s_lastPumps = 0xFF; // invalid → force first write
static uint32_t   s_lastTs    = 0;
static SemaphoreHandle_t s_mtx = nullptr;

// ── Internal helpers ─────────────────────────────────────────────────────────
static uint8_t pumpBitmask() {
    uint8_t b = 0;
    if (FiltrationPump.IsRunning()) b |= (1<<0);
    if (PhPump.IsRunning())         b |= (1<<1);
    if (ChlPump.IsRunning())        b |= (1<<2);
    if (HeatPump.IsRunning())       b |= (1<<3);
    if (SaltPump.IsRunning())       b |= (1<<4);
    if (RobotPump.IsRunning())      b |= (1<<5);
    if (SolarPump.IsRunning())      b |= (1<<6);
    if (WaterFill.IsRunning())      b |= (1<<7);
    return b;
}
static uint8_t modeBitmask() {
    uint8_t b = 0;
    if (storage.AutoMode)            b |= (1<<0);
    if (storage.Ph_RegulationOnOff)  b |= (1<<1);
    if (storage.Orp_RegulationOnOff) b |= (1<<2);
    if (storage.WaterFillMode)       b |= (1<<3);
    if (storage.HeatPumpMode)        b |= (1<<4);
    if (storage.SaltMode)            b |= (1<<5);
    if (storage.WinterMode)          b |= (1<<6);
    return b;
}

static bool appendBytes(const char* path, const void* data, size_t sz, size_t maxBytes) {
    if (SPIFFS.totalBytes() - SPIFFS.usedBytes() < SPIFFS_MIN_FREE) {
        Debug.print(DBG_WARNING, "[Logger] SPIFFS low, skipping write");
        return false;
    }
    // Rotate if file reached size limit
    {
        File fr = SPIFFS.open(path, FILE_READ);
        bool rotate = fr && (fr.size() + sz > maxBytes);
        if (fr) fr.close();
        if (rotate) {
            // backup: old backup → delete, current → backup, start fresh
            const char* bak = (strcmp(path, "/sc.bin") == 0) ? "/sb.bin" : "/eb.bin";
            if (SPIFFS.exists(bak))  SPIFFS.remove(bak);
            SPIFFS.rename(path, bak);
            Debug.print(DBG_INFO, "[Logger] Rotated %s → %s", path, bak);
        }
    }
    File f = SPIFFS.open(path, FILE_APPEND);
    if (!f) { Debug.print(DBG_WARNING, "[Logger] Cannot open %s for append", path); return false; }
    size_t written = f.write((const uint8_t*)data, sz);
    f.close();
    return written == sz;
}

// ── Public: record one sample (called every 5 s from webUIBroadcast) ─────────
void record() {
    uint32_t nowTs = (uint32_t)time(nullptr);
    // Require valid NTP time (after 2024-01-01 = 1704067200)
    if (nowTs < 1704067200UL) return;

    uint32_t elapsed = nowTs - s_lastTs;
    uint8_t  pumps   = pumpBitmask();

    // Build candidate record
    SensorRec cur;
    cur.ts      = nowTs;
    cur.ph100   = (int16_t)(storage.PhValue   * 100.0f);
    cur.orp     = (int16_t) storage.OrpValue;
    cur.psi100  = (int16_t)(storage.PSIValue  * 100.0f);
    cur.wTemp10 = (int16_t)(storage.WaterSTemp * 10.0f);
    cur.airT10  = (int16_t)(storage.AirTemp    * 10.0f);
    cur.pumps   = pumps;
    cur.modes   = modeBitmask();

    // ── Log pump state change events ─────────────────────────────────────────
    if (s_lastPumps != 0xFF) {
        uint8_t changed = pumps ^ s_lastPumps;
        if (changed) {
            if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int i = 0; i < 8; i++) {
                    if (changed & (1 << i)) {
                        EventRec ev;
                        ev.ts    = nowTs;
                        ev.dev   = (uint8_t)i;
                        ev.state = (pumps >> i) & 1;
                        ev.pad   = 0;
                        appendBytes("/ec.bin", &ev, sizeof(ev), EVENT_MAX_BYTES);
                    }
                }
                xSemaphoreGive(s_mtx);
            }
        }
    }
    s_lastPumps = pumps;

    // ── Sensor record: write when delta exceeds threshold or max interval ─────
    bool significant =
        (elapsed >= MAX_INTERVAL_S) ||
        (elapsed >= MIN_INTERVAL_S && (
            abs(cur.ph100   - s_last.ph100)   >= PH_DELTA   ||
            abs(cur.orp     - s_last.orp)      >= ORP_DELTA  ||
            abs(cur.psi100  - s_last.psi100)   >= PSI_DELTA  ||
            abs(cur.wTemp10 - s_last.wTemp10)  >= TEMP_DELTA ||
            abs(cur.airT10  - s_last.airT10)   >= AIRT_DELTA ||
            cur.pumps != s_last.pumps
        ));

    if (s_lastTs == 0) significant = true; // always write first record

    if (!significant) return;

    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        appendBytes("/sc.bin", &cur, sizeof(cur), SENSOR_MAX_BYTES);
        xSemaphoreGive(s_mtx);
    }
    s_last  = cur;
    s_lastTs = nowTs;
}

// ── Public: query sensor records ─────────────────────────────────────────────
// Returns compact JSON: { records:[{ts,ph,orp,psi,temp,airTemp,pumps,modes}, ...], total, step }
// Decimates to at most maxPts points.
String querySensor(uint32_t fromTs, uint32_t toTs, uint16_t maxPts) {
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) return "{\"records\":[]}";

    // Pass 1: count matching records across both files
    static const char* sFiles[] = {"/sb.bin", "/sc.bin"};
    uint32_t total = 0;
    for (const char* path : sFiles) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        SensorRec rec;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts >= fromTs && rec.ts <= toTs) total++;
        }
        f.close();
    }

    uint32_t step = (maxPts > 0 && total > maxPts) ? (total / maxPts) : 1;

    // Pass 2: build JSON (bounded by maxPts → max ~50 KB output)
    String out;
    out.reserve(min((uint32_t)52000U, total/step * 90U + 128U));
    out = F("{\"type\":\"log\",\"step\":");
    out += step;
    out += F(",\"total\":");
    out += total;
    out += F(",\"records\":[");

    bool first = true;
    for (const char* path : sFiles) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        SensorRec rec;
        uint32_t n = 0;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts < fromTs || rec.ts > toTs) { n++; continue; }
            if ((n % step) != 0) { n++; continue; }
            if (!first) out += ',';
            char buf[140];
            snprintf(buf, sizeof(buf),
                     "{\"ts\":%lu,\"ph\":%.2f,\"orp\":%d,\"psi\":%.2f,"
                     "\"temp\":%.1f,\"airTemp\":%.1f,\"pumps\":%u,\"modes\":%u}",
                     (unsigned long)rec.ts,
                     rec.ph100   / 100.0f, (int)rec.orp,
                     rec.psi100  / 100.0f,
                     rec.wTemp10 / 10.0f,  rec.airT10 / 10.0f,
                     (unsigned)rec.pumps,  (unsigned)rec.modes);
            out += buf;
            first = false;
            n++;
        }
        f.close();
    }
    out += F("]}");
    xSemaphoreGive(s_mtx);
    return out;
}

// ── Public: query events ──────────────────────────────────────────────────────
// Returns { events:[{ts,dev,state,name}, ...] }
String queryEvents(uint32_t fromTs, uint32_t toTs) {
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) return "{\"events\":[]}";
    static const char* eFiles[] = {"/eb.bin", "/ec.bin"};
    static const char* devNames[8] = {
        "Filterpumpe","pH-Pumpe","Chlorpumpe","Heizung",
        "Salzelektrolyse","Roboter","Solar","Befuellung"
    };
    String out = F("{\"type\":\"events\",\"events\":[");
    bool first = true;
    for (const char* path : eFiles) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        EventRec rec;
        while (f.available() >= (int)sizeof(rec)) {
            f.read((uint8_t*)&rec, sizeof(rec));
            if (rec.ts < fromTs || rec.ts > toTs) continue;
            if (!first) out += ',';
            const char* name = (rec.dev < 8) ? devNames[rec.dev] : "Unbekannt";
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "{\"ts\":%lu,\"dev\":%u,\"state\":%u,\"name\":\"%s\"}",
                     (unsigned long)rec.ts, rec.dev, rec.state, name);
            out += buf;
            first = false;
        }
        f.close();
    }
    out += F("]}");
    xSemaphoreGive(s_mtx);
    return out;
}

// ── Public: storage info ──────────────────────────────────────────────────────
struct Info {
    uint32_t sensorRecs, eventRecs;
    uint32_t oldestTs, newestTs;
    size_t   spiffsFree, spiffsTotal;
};

Info getInfo() {
    Info i = {};
    i.oldestTs  = UINT32_MAX;
    i.spiffsFree  = SPIFFS.totalBytes() - SPIFFS.usedBytes();
    i.spiffsTotal = SPIFFS.totalBytes();

    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(1000)) != pdTRUE) return i;

    static const char* sFiles[] = {"/sb.bin", "/sc.bin"};
    for (const char* path : sFiles) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        uint32_t cnt = f.size() / sizeof(SensorRec);
        i.sensorRecs += cnt;
        if (cnt > 0) {
            SensorRec rec;
            f.seek(0);
            if (f.read((uint8_t*)&rec, sizeof(rec)) == sizeof(rec))
                if (rec.ts < i.oldestTs) i.oldestTs = rec.ts;
            f.seek((cnt - 1) * sizeof(SensorRec));
            if (f.read((uint8_t*)&rec, sizeof(rec)) == sizeof(rec))
                if (rec.ts > i.newestTs) i.newestTs = rec.ts;
        }
        f.close();
    }

    static const char* eFiles[] = {"/eb.bin", "/ec.bin"};
    for (const char* path : eFiles) {
        if (!SPIFFS.exists(path)) continue;
        File f = SPIFFS.open(path, FILE_READ);
        if (!f) continue;
        i.eventRecs += f.size() / sizeof(EventRec);
        f.close();
    }

    if (i.oldestTs == UINT32_MAX) i.oldestTs = 0;
    xSemaphoreGive(s_mtx);
    return i;
}

void clearAll() {
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(2000)) != pdTRUE) return;
    SPIFFS.remove("/sc.bin"); SPIFFS.remove("/sb.bin");
    SPIFFS.remove("/ec.bin"); SPIFFS.remove("/eb.bin");
    s_lastTs = 0; s_lastPumps = 0xFF; s_last = {};
    Debug.print(DBG_INFO, "[Logger] All log files cleared");
    xSemaphoreGive(s_mtx);
}

void init() {
    s_mtx = xSemaphoreCreateMutex();
    Debug.print(DBG_INFO, "[Logger] Init — SPIFFS free: %u KB",
                (unsigned)((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024));
    auto info = getInfo();
    Debug.print(DBG_INFO, "[Logger] %u sensor records, %u events, oldest=%lu",
                info.sensorRecs, info.eventRecs, (unsigned long)info.oldestTs);
}

} // namespace Logger


// ─── WebSocket helpers ────────────────────────────────────────────────────────
static String buildStatusJson() {
    StaticJsonDocument<3072> doc;
    doc["type"] = "status";
    doc["ts"]   = (uint32_t)(millis() / 1000);
    doc["uptime"]      = (uint32_t)(millis() / 1000);   // seconds since boot
    doc["uptimeTotal"] = storage.Uptime;                // hours, cumulative across reboots
    doc["resetReason"] = resetReasonToString(storage.ResetReason);
    doc["heap"]        = (uint32_t)ESP.getFreeHeap();
    doc["firmware"]    = Firmw;

    JsonObject pumps = doc.createNestedObject("pumps");
    pumps["filt"]  = FiltrationPump.IsRunning() ? 1 : 0;
    pumps["ph"]    = PhPump.IsRunning()          ? 1 : 0;
    pumps["chl"]   = ChlPump.IsRunning()         ? 1 : 0;
    pumps["heat"]  = HeatPump.IsRunning()        ? 1 : 0;
    pumps["salt"]  = SaltPump.IsRunning()        ? 1 : 0;
    pumps["robot"] = RobotPump.IsRunning()       ? 1 : 0;
    pumps["solar"] = SolarPump.IsRunning()       ? 1 : 0;
    pumps["fill"]  = WaterFill.IsRunning()       ? 1 : 0;

    JsonObject modes = doc.createNestedObject("modes");
    modes["auto"]   = storage.AutoMode              ? 1 : 0;
    modes["phReg"]  = storage.Ph_RegulationOnOff    ? 1 : 0;
    modes["orpReg"] = storage.Orp_RegulationOnOff   ? 1 : 0;
    modes["fill"]   = storage.WaterFillMode         ? 1 : 0;
    modes["heat"]   = storage.HeatPumpMode          ? 1 : 0;
    modes["salt"]   = storage.SaltMode              ? 1 : 0;
    modes["winter"] = storage.WinterMode            ? 1 : 0;
    modes["solar"]  = storage.SolarMode             ? 1 : 0;

    JsonObject val = doc.createNestedObject("val");
    val["ph"]       = (double)storage.PhValue;
    val["orp"]      = (double)storage.OrpValue;
    val["psi"]      = (double)storage.PSIValue;
    val["flow"]     = (double)storage.FLOWValue;
    val["flow2"]    = (double)storage.FLOW2Value;
    val["wS"]       = (double)storage.WaterSTemp;
    val["wI"]       = (double)storage.WaterITemp;
    val["wB"]       = (double)storage.WaterBTemp;
    val["wWP"]      = (double)storage.WaterWPTemp;
    val["air"]      = (double)storage.AirTemp;
    val["airHum"]   = (double)storage.AirHum;
    val["airPress"] = (double)storage.AirPress;
    val["solarVL"]  = (double)storage.SolarVLTemp;
    val["solarRL"]  = (double)storage.SolarRLTemp;
    val["saltConc"] = (double)storage.SaltConcentration;

    JsonObject sp = doc.createNestedObject("sp");
    sp["ph"]  = (double)storage.Ph_SetPoint;
    sp["orp"] = (double)storage.Orp_SetPoint;
    sp["psi"] = (double)storage.PSI_HighThreshold;

    JsonObject pid = doc.createNestedObject("pid");
    pid["phOut"]  = storage.PhPIDOutput  / 1000.0;
    pid["orpOut"] = storage.OrpPIDOutput / 1000.0;
    pid["phKp"]   = storage.Ph_Kp  / 10000.0;
    pid["orpKp"]  = storage.Orp_Kp / 1000.0;
    pid["phMode"]  = PhPID.GetMode();
    pid["orpMode"] = OrpPID.GetMode();

    JsonObject tanks = doc.createNestedObject("tanks");
    tanks["acid"]       = (double)storage.AcidFill;
    tanks["chl"]        = (double)storage.ChlFill;
    tanks["wFillAnCon"] = storage.WaterFillAnCon;
    tanks["poolVol"]    = (double)storage.PoolVolume;
    tanks["saltNeeded"] = (double)storage.SaltNeeded;

    JsonObject err = doc.createNestedObject("err");
    err["psi"]       = PSIError              ? 1 : 0;
    err["flow"]      = FLOWError             ? 1 : 0;
    err["flow2"]     = FLOW2Error            ? 1 : 0;
    err["waterFill"] = WaterFillError        ? 1 : 0;
    err["i2c"]       = I2CError              ? 1 : 0;
    err["emergency"] = EmergencyStopFiltPump ? 1 : 0;
    err["phTank"]    = !PhPump.TankLevel()   ? 1 : 0;
    err["chlTank"]   = !ChlPump.TankLevel()  ? 1 : 0;

    JsonObject net = doc.createNestedObject("net");
    net["wifi"]    = storage.WIFI_OnOff ? 1 : 0;
    net["mqtt"]    = storage.MQTTLOGIN_OnOff ? 1 : 0;
    bool wifiUp    = (WiFi.status() == WL_CONNECTED);
    net["wifiUp"]  = wifiUp ? 1 : 0;
    net["ssid"]    = storage.SSID;
    net["ip"]      = wifiUp ? WiFi.localIP().toString() : String("–");
    net["rssi"]    = wifiUp ? WiFi.RSSI() : 0;
    net["mqttUp"]  = MQTTConnection ? 1 : 0;

    JsonObject solar = doc.createNestedObject("solar");
    solar["roofTemp"]    = (double)storage.solarRoofTemp;
    solar["boilerTemp"]  = (double)storage.solarBoilerTemp;
    solar["storageTemp"] = (double)storage.solarStorageTemp;
    solar["backflow"]    = (double)storage.solarBackflowTemp;
    solar["pump"]        = storage.solarPumpRunning ? 1 : 0;
    solar["valvePool"]   = storage.solarValvePool   ? 1 : 0;

    String out; serializeJson(doc, out);
    return out;
}

static String buildHistoryJson() {
    DynamicJsonDocument doc(20000);
    doc["type"] = "history";
    JsonArray arr = doc.createNestedArray("data");
    uint16_t start = (histCount < HIST_SIZE) ? 0 : histHead;
    for (uint16_t i = 0; i < histCount; i++) {
        uint16_t idx = (start + i) % HIST_SIZE;
        const HistPoint& p = histBuf[idx];
        JsonArray row = arr.createNestedArray();
        row.add(p.ts);
        row.add(p.ph);
        row.add(p.orp);
        row.add(p.psi);
        row.add(p.waterTemp);
        row.add(p.airTemp);
    }
    String out; serializeJson(doc, out);
    return out;
}

static bool queueCommand(const char* json, size_t len) {
    if (len >= QUEUE_ITEM_SIZE) return false;
    char buf[QUEUE_ITEM_SIZE];
    memcpy(buf, json, len);
    buf[len] = '\0';
    return xQueueSend(queueIn, buf, pdMS_TO_TICKS(100)) == pdTRUE;
}

// ─── WebSocket event handler ──────────────────────────────────────────────────
static void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Debug.print(DBG_INFO, "[WebUI] WS #%u connected", client->id());
        client->text(buildStatusJson());
        client->text(buildHistoryJson());
    } else if (type == WS_EVT_DISCONNECT) {
        Debug.print(DBG_INFO, "[WebUI] WS #%u disconnected", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->opcode == WS_TEXT && info->final && info->index == 0 && len > 0) {
            StaticJsonDocument<256> req;
            if (!deserializeJson(req, data, len) && req.containsKey("cmd")) {
                auto cmd = req["cmd"];
#ifdef MATTER_ENABLED
                if (cmd.containsKey("MatterStatus")) {
                    char qr[96] = {}, manual[32] = {};
                    bool ok = matterGetQRCode(qr, sizeof(qr));
                    matterGetManualPairingCode(manual, sizeof(manual));
                    uint8_t fabrics = matterFabricCount();
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        "{\"type\":\"matter_status\",\"commissioned\":%s,\"fabrics\":%u,"
                        "\"qr_code\":\"%s\",\"pairing_code\":\"%s\"}",
                        fabrics > 0 ? "true" : "false", fabrics,
                        ok ? qr : "", manual);
                    client->text(buf);
                    return;
                }
                if (cmd.containsKey("MatterOpenCommissioning")) {
                    bool opened = matterOpenCommissioningWindow(900);
                    client->text(opened ? "{\"ack\":1}" : "{\"ack\":0,\"err\":\"commissioning_window_failed\"}");
                    return;
                }
#endif
                String s; serializeJson(cmd, s);
                client->text(queueCommand(s.c_str(), s.length()) ?
                    "{\"ack\":1}" : "{\"ack\":0,\"err\":\"queue_full\"}");
            }
        }
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────
void initWebUI() {
    Logger::init();

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Static files from SPIFFS
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html")
          .setCacheControl("no-cache");

    // ── Live data ──────────────────────────────────────────────────────────────
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "application/json", buildStatusJson());
    });
    server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "application/json", buildHistoryJson());
    });

    // ── Settings ───────────────────────────────────────────────────────────────
    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<1024> doc;
        doc["poolVol"]    = (double)storage.PoolVolume;
        doc["phSP"]       = storage.Ph_SetPoint;
        doc["orpSP"]      = storage.Orp_SetPoint;
        doc["phKp"]       = storage.Ph_Kp  / 10000.0;
        doc["phKi"]       = storage.Ph_Ki;
        doc["phKd"]       = storage.Ph_Kd;
        doc["orpKp"]      = storage.Orp_Kp / 1000.0;
        doc["orpKi"]      = storage.Orp_Ki;
        doc["orpKd"]      = storage.Orp_Kd;
        doc["phWinSize"]  = storage.PhPIDWindowSize  / 60000;
        doc["orpWinSize"] = storage.OrpPIDWindowSize / 60000;
        doc["phTankVol"]  = storage.pHTankVol;
        doc["chlTankVol"] = storage.ChlTankVol;
        doc["psiHigh"]    = storage.PSI_HighThreshold;
        doc["psiMed"]     = storage.PSI_MedThreshold;
        doc["wFillMaxUp"] = storage.WaterFillUpTimeLimit / 60000;
        doc["wFillFR"]    = storage.WaterFillFR;
        doc["wFillDur"]   = storage.WaterFillDuration / 60000;
        doc["filtStart"]  = storage.FiltrationStart;
        doc["filtStop"]   = storage.FiltrationStop;
        doc["delayPID"]   = storage.DelayPIDs;
        doc["firmware"]   = Firmw;
        doc["uptime"]     = storage.Uptime;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // ── Command (POST body = JSON command) ────────────────────────────────────
    server.on("/api/command", HTTP_POST,
        [](AsyncWebServerRequest* req) {},
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            req->send(queueCommand((const char*)data, len) ? 200 : 503,
                      "application/json",
                      queueCommand((const char*)data, len) ? "{\"ok\":1}" : "{\"ok\":0}");
        }
    );

    // ── Log query: GET /api/log?from=<ts>&to=<ts>&pts=<N> ────────────────────
    server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint32_t nowTs = (uint32_t)time(nullptr);
        uint32_t fromTs = 0, toTs = nowTs;
        uint16_t maxPts = 500;
        if (req->hasParam("from")) fromTs = (uint32_t)req->getParam("from")->value().toInt();
        if (req->hasParam("to"))   toTs   = (uint32_t)req->getParam("to")->value().toInt();
        if (req->hasParam("pts"))  maxPts = (uint16_t)req->getParam("pts")->value().toInt();
        req->send(200, "application/json", Logger::querySensor(fromTs, toTs, maxPts));
    });

    // ── Event log: GET /api/events?from=<ts>&to=<ts> ─────────────────────────
    server.on("/api/events", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint32_t nowTs = (uint32_t)time(nullptr);
        uint32_t fromTs = 0, toTs = nowTs;
        if (req->hasParam("from")) fromTs = (uint32_t)req->getParam("from")->value().toInt();
        if (req->hasParam("to"))   toTs   = (uint32_t)req->getParam("to")->value().toInt();
        req->send(200, "application/json", Logger::queryEvents(fromTs, toTs));
    });

    // ── Log info: GET /api/loginfo ────────────────────────────────────────────
    server.on("/api/loginfo", HTTP_GET, [](AsyncWebServerRequest* req) {
        auto i = Logger::getInfo();
        unsigned long used = (i.spiffsTotal > i.spiffsFree)
                              ? (i.spiffsTotal - i.spiffsFree) : 0UL;
        char buf[320];
        snprintf(buf, sizeof(buf),
            "{\"sensorRecs\":%lu,\"eventRecs\":%lu,\"oldest\":%lu,\"newest\":%lu,"
            "\"used\":%lu,\"free\":%lu,\"total\":%lu}",
            (unsigned long)i.sensorRecs,  (unsigned long)i.eventRecs,
            (unsigned long)i.oldestTs,    (unsigned long)i.newestTs,
            used, (unsigned long)i.spiffsFree, (unsigned long)i.spiffsTotal);
        req->send(200, "application/json", buf);
    });

    // ── Network info (WiFi + MQTT credentials, plain text) ───────────────────
    server.on("/api/network", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<512> doc;
        doc["wifiOn"]   = storage.WIFI_OnOff;
        doc["mqttOn"]   = storage.MQTTLOGIN_OnOff;
        doc["ssid"]     = storage.SSID;
        doc["wifiPass"] = storage.WIFI_PASS;
        doc["mqttIp"]   = storage.MQTT_IP.toString();
        doc["mqttPort"] = storage.MQTT_PORT;
        doc["mqttUser"] = storage.MQTT_USER;
        doc["mqttPass"] = storage.MQTT_PASS;
        doc["mqttName"] = storage.MQTT_NAME;
        bool wifiUp     = (WiFi.status() == WL_CONNECTED);
        doc["wifiUp"]   = wifiUp;
        doc["ip"]       = wifiUp ? WiFi.localIP().toString() : String("–");
        doc["rssi"]     = wifiUp ? WiFi.RSSI() : 0;
        doc["mqttUp"]   = MQTTConnection;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // ── Log clear: POST /api/logclear ─────────────────────────────────────────
    server.on("/api/logclear", HTTP_POST, [](AsyncWebServerRequest* req) {
        Logger::clearAll();
        req->send(200, "application/json", "{\"ok\":1}");
    });

    // ── CSV export: GET /api/export.csv?from=<ts>&to=<ts> ─────────────────────
    // (streams full sensor log as CSV for download in Excel / analysis tools)
    server.on("/api/export.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint32_t nowTs  = (uint32_t)time(nullptr);
        uint32_t fromTs = 0, toTs = nowTs;
        if (req->hasParam("from")) fromTs = (uint32_t)req->getParam("from")->value().toInt();
        if (req->hasParam("to"))   toTs   = (uint32_t)req->getParam("to")->value().toInt();
        // Build CSV in String (bounded to ~200 KB for full data)
        String csv;
        csv.reserve(65536);
        csv = "timestamp,pH,ORP_mV,PSI_bar,WaterTemp_C,AirTemp_C,pumps,modes\r\n";
        static const char* sFiles[] = {"/sb.bin", "/sc.bin"};
        for (const char* path : sFiles) {
            if (!SPIFFS.exists(path)) continue;
            File f = SPIFFS.open(path, FILE_READ);
            if (!f) continue;
            Logger::SensorRec rec;
            while (f.available() >= (int)sizeof(rec)) {
                f.read((uint8_t*)&rec, sizeof(rec));
                if (rec.ts < fromTs || rec.ts > toTs) continue;
                char buf[96];
                snprintf(buf, sizeof(buf), "%lu,%.2f,%d,%.2f,%.1f,%.1f,%u,%u\r\n",
                    (unsigned long)rec.ts,
                    rec.ph100/100.0f, rec.orp, rec.psi100/100.0f,
                    rec.wTemp10/10.0f, rec.airT10/10.0f,
                    rec.pumps, rec.modes);
                csv += buf;
            }
            f.close();
        }
        AsyncWebServerResponse* resp = req->beginResponse(200, "text/csv", csv);
        resp->addHeader("Content-Disposition", "attachment; filename=\"poolmaster_log.csv\"");
        req->send(resp);
    });

    Debug.print(DBG_INFO, "[WebUI] All routes registered");
}

void webUIBroadcast() {
    recordHistory();      // fast RAM ring buffer (30s interval)
    Logger::record();     // SPIFFS delta logger (60s+ interval)
    ws.cleanupClients(3); // drop excess/stale clients; cap at 3 to limit TX heap pressure
    if (ws.count() == 0) return;
    // Skip broadcast when internal heap is dangerously low — throttled clients
    // accumulate TX buffers in internal RAM and can corrupt the heap allocator.
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 20000) return;
    String json = buildStatusJson();
    for (auto& client : ws.getClients()) {
        if (client.status() == WS_CONNECTED && !client.queueIsFull())
            client.text(json);
    }
}
