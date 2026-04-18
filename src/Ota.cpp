#include "Ota.h"
#include <WiFi.h>
#include <SPIFFS.h>
#ifdef MATTER_ENABLED
#include "esp_wifi.h"
#endif
#include <HardwareSerial.h>
#include "Arduino_DebugUtils.h"
#include "Config.h"
#include "PoolMaster.h"
#include "WebUI.h"

extern Arduino_DebugUtils Debug;

#define RXD2 16
#define TXD2 17
HardwareSerial nextionSerial(2);

AsyncWebServer server(OTA_NEXTION_PORT);

// Forward declarations
void updateNextion();
// Flag set by the upload handler; polled by otaTask to trigger Nextion flash
// from the task context (never call updateNextion() from the AsyncTCP callback).
static volatile bool s_nextion_update_pending = false;

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static File file;
  static bool writeError = false;

  if (!index) {
    writeError = false;
    Debug.print(DBG_INFO, "[OTA] Starting upload: %s", filename.c_str());
    file = SPIFFS.open("/nextion.tft", FILE_WRITE);
    if (!file) {
      Debug.print(DBG_ERROR, "[OTA] Could not open file for writing");
      request->send(500, "text/plain", "SPIFFS error");
      return;
    }
  }

  if (len) {
    if (file.write(data, len) != len) {
      Debug.print(DBG_ERROR, "[OTA] Write error");
      writeError = true;
    }
  }

  if (final) {
    file.close();
    if (writeError) {
      request->send(500, "text/plain", "Write error — Nextion update aborted");
    } else {
      Debug.print(DBG_INFO, "[OTA] Upload completed — scheduling Nextion flash");
      s_nextion_update_pending = true;   // otaTask will call updateNextion()
      request->send(200, "text/html",
        "<p>Upload OK. Nextion wird jetzt geflasht (~30 s).</p>"
        "<p>Seite nach 40 Sekunden neu laden.</p>");
    }
  }
}

void initOTA(void) {
}

void otaTask(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] otaTask started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] otaTask running...");
  vTaskDelay(DT13);

  TickType_t period = pdMS_TO_TICKS(980);
  TickType_t ticktime = xTaskGetTickCount();

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  // SPIFFS.begin(true) formats the partition on first boot — this can take >10 s,
  // which exceeds the TWDT timeout. Register with TWDT only after mount completes.
  Debug.print(DBG_INFO, "[OTA] Initializing SPIFFS...");
  bool spiffsOk = false;
  if (!SPIFFS.begin(true)) { // true = format on fail
    Debug.print(DBG_ERROR, "[OTA] SPIFFS mount failed");
  } else {
    Debug.print(DBG_INFO, "[OTA] SPIFFS mounted successfully");
    spiffsOk = true;
  }
  esp_task_wdt_add(NULL);  // register AFTER potentially slow SPIFFS format

  Debug.print(DBG_INFO, "[OTA] Initializing serial...");
  nextionSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Wait for WiFi — with timeout so we never block forever.
  // In MATTER_ENABLED mode, WiFi.status() always returns WL_DISCONNECTED because
  // Arduino's WiFi stack is never initialized (CHIP manages WiFi via esp-idf).
  // Use esp_wifi_sta_get_ap_info() instead to check actual connection state.
  Debug.print(DBG_INFO, "[OTA] Waiting for WiFi...");
  {
    uint32_t wifiDeadline = millis() + 60000UL;
    bool connected = false;
    while (millis() < wifiDeadline) {
#ifdef MATTER_ENABLED
      wifi_ap_record_t ap_info;
      connected = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
#else
      connected = (WiFi.status() == WL_CONNECTED);
#endif
      if (connected) break;
      vTaskDelay(pdMS_TO_TICKS(1000));
      esp_task_wdt_reset();
    }
    if (!connected)
      Debug.print(DBG_WARNING, "[OTA] WiFi not available — OTA web server disabled");
  }
  // Re-check connection state for server start decision
#ifdef MATTER_ENABLED
  wifi_ap_record_t _ota_ap;
  bool otaWiFiUp = (esp_wifi_sta_get_ap_info(&_ota_ap) == ESP_OK);
#else
  bool otaWiFiUp = (WiFi.status() == WL_CONNECTED);
#endif

  if (spiffsOk && otaWiFiUp) {
    Debug.print(DBG_INFO, "[OTA] Setting up server...");
    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='file' accept='.tft'><input type='submit' value='Upload'></form>");
    });
    server.on(OTA_NEXTION_PATH, HTTP_POST, [](AsyncWebServerRequest *request) {}, handleFileUpload);
    // Register WebUI routes (before server.begin())
    initWebUI();
    server.begin();
    Debug.print(DBG_INFO, "[OTA] Web server started on port %d", OTA_NEXTION_PORT);
  } else {
    Debug.print(DBG_WARNING, "[OTA] No web server - SPIFFS unavailable");
  }

  // Periodic WebSocket broadcast timer: every 5 s
  TimerHandle_t wsTimer = xTimerCreate("wsBC", pdMS_TO_TICKS(5000), pdTRUE, nullptr,
    [](TimerHandle_t){ webUIBroadcast(); });
  if (wsTimer) xTimerStart(wsTimer, 0);

  for (;;) {
    esp_task_wdt_reset();

    // Check if a new .tft file was uploaded and needs to be sent to Nextion.
    // updateNextion() uses delay() internally — call it only from this task,
    // never from the AsyncTCP upload callback.
    if (s_nextion_update_pending) {
      s_nextion_update_pending = false;
      Debug.print(DBG_INFO, "[OTA] Flashing Nextion...");
      updateNextion();
      Debug.print(DBG_INFO, "[OTA] Nextion flash done");
    }

    #ifdef CHRONO
    td = millis();
    #endif

    Debug.print(DBG_DEBUG, "[stack_mon] %s: %u bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[otaTask] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    vTaskDelayUntil(&ticktime, period);
  }
}

void updateNextion() {
  // Send "whmi-wri" command: write firmware at 115200 baud, from internal flash
  nextionSerial.print("whmi-wri 1,115200,0");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  // Give Nextion time to enter update mode; use vTaskDelay to feed the WDT
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(1500));

  File file = SPIFFS.open("/nextion.tft", FILE_READ);
  if (!file) {
    Debug.print(DBG_ERROR, "[OTA] Could not open TFT file");
    return;
  }

  size_t totalBytes = file.size();
  size_t sentBytes  = 0;
  uint8_t buffer[512];

  while (file.available()) {
    size_t bytesRead = file.readBytes((char *)buffer, sizeof(buffer));
    nextionSerial.write(buffer, bytesRead);
    sentBytes += bytesRead;
    // Yield every ~32 KB so the WDT stays happy during large uploads (~4 MB file)
    if ((sentBytes % (32 * 1024)) < sizeof(buffer)) {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
  file.close();
  esp_task_wdt_reset();

  Debug.print(DBG_INFO, "[OTA] Nextion update completed (%u bytes)", (unsigned)sentBytes);
}