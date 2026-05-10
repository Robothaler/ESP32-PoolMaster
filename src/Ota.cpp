#include "Ota.h"
#include <WiFi.h>
#include <SPIFFS.h>
#ifdef MATTER_ENABLED
#include "esp_wifi.h"
#include "MatterBridge.h"
#endif
#include <HardwareSerial.h>
#include "Arduino_DebugUtils.h"
#include "Config.h"
#include "PoolMaster.h"
#include "WebUI.h"
#include "PoolSolarBridge.h"

extern Arduino_DebugUtils Debug;

// Nextion HMI hooks (declared in Nextion.cpp — no Nextion.h in this project)
extern void nextionPause(void);
extern void nextionResume(void);

// IMPORTANT: the Nextion display is wired to Serial1 (default ESP32-S3 pins
// RX=17, TX=18 — see Nextion.cpp).  We must NOT instantiate a second UART
// on those pins: ESP-IDF's pin matrix would silently re-route GPIO 17 from
// "UART1 RX" to "UART2 TX", killing all touch events from the HMI to the
// MCU.  The Nextion's whmi-wri OTA protocol talks over the SAME wires, so
// we simply reuse Serial1 in updateNextion(); NextionListen() is paused
// for the duration of the flash via nextionPause()/nextionResume().

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

  // No separate Serial2 init here — see header comment in this file.
  // Serial1 is brought up by InitTFT()/ResetTFT() in Nextion.cpp at 115200 baud.

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

  const bool canStartHttp = spiffsOk && otaWiFiUp;
  static bool s_otaHttpServerStarted = false;

  auto tryStartOtaHttpServer = [&]() {
    if (s_otaHttpServerStarted || !canStartHttp)
      return;
#if defined(MATTER_ENABLED) && MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED
    if (matterFabricCount() == 0 && millis() < (unsigned long)MATTER_HTTP_SERVER_FALLBACK_MS)
      return;
#endif
    Debug.print(DBG_INFO, "[OTA] Setting up server...");
    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='file' accept='.tft'><input type='submit' value='Upload'></form>");
    });
    server.on(OTA_NEXTION_PATH, HTTP_POST, [](AsyncWebServerRequest *request) {}, handleFileUpload);
    initWebUI();
    server.begin();
    s_otaHttpServerStarted = true;
    Debug.print(DBG_INFO, "[OTA] Web server started on port %d", OTA_NEXTION_PORT);
#if defined(MATTER_ENABLED) && MATTER_AGENT_DEBUG_NDJSON
    // #region agent log
    printf(
        "NDJSON{\"sessionId\":\"e37f7a\",\"hypothesisId\":\"H6\",\"location\":\"Ota.cpp:otaTask\","
        "\"message\":\"http_server_begin\",\"data\":{\"fabric\":%u,\"ms\":%lu},\"timestamp\":%lu}\n",
        (unsigned)matterFabricCount(), (unsigned long)millis(), (unsigned long)millis());
    // #endregion
#endif
  };

  if (!canStartHttp) {
    Debug.print(DBG_WARNING, "[OTA] No web server - SPIFFS or WiFi unavailable");
  } else {
#if defined(MATTER_ENABLED) && MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED
    Debug.print(DBG_INFO, "[OTA] Web server may defer until Matter fabric>0 or %lu ms fallback",
                (unsigned long)MATTER_HTTP_SERVER_FALLBACK_MS);
#endif
#if !defined(MATTER_ENABLED) || !MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED
    tryStartOtaHttpServer();
#endif
  }

  // WebSocket broadcast runs directly in this task loop every 5 s (every 5 × 980 ms ticks).
  // Previously used a FreeRTOS software timer, but timer callbacks run in the shared
  // timer task whose stack (~4 KB) is too small for StaticJsonDocument<1280> +
  // SPIFFS Logger::record() I/O → stack overflow → MMU fault (null ptr in String::write).
  uint8_t broadcastTick = 0;

  for (;;) {
    esp_task_wdt_reset();

#if defined(MATTER_ENABLED) && MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED
    tryStartOtaHttpServer();
#endif

    // WebSocket status broadcast every ~5 s (only after server.begin)
    if (s_otaHttpServerStarted) {
      if (++broadcastTick >= 5) {
        broadcastTick = 0;
        webUIBroadcast();
      }
    }

    poolSolarBridgePollTick();

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
  // Pause the Nextion polling loop in PoolMaster (UpdateTFT → NextionListen)
  // so we have exclusive access to Serial1 while the .tft is streamed.
  nextionPause();

  // Drain any leftover bytes from the HMI before we start talking OTA.
  while (Serial1.available()) (void)Serial1.read();

  // Send "whmi-wri" command: write firmware at 115200 baud, from internal flash
  Serial1.print("whmi-wri 1,115200,0");
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  // Give Nextion time to enter update mode; use vTaskDelay to feed the WDT
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(1500));

  File file = SPIFFS.open("/nextion.tft", FILE_READ);
  if (!file) {
    Debug.print(DBG_ERROR, "[OTA] Could not open TFT file");
    nextionResume();
    return;
  }

  size_t totalBytes = file.size();
  size_t sentBytes  = 0;
  uint8_t buffer[512];

  while (file.available()) {
    size_t bytesRead = file.readBytes((char *)buffer, sizeof(buffer));
    Serial1.write(buffer, bytesRead);
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

  // The Nextion automatically reboots after a successful flash; give it a
  // moment to come back, then resume normal HMI polling.
  vTaskDelay(pdMS_TO_TICKS(2000));
  nextionResume();
}