#include "Ota.h"
#include <WiFi.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include "Arduino_DebugUtils.h"
#include "Config.h"
#include "PoolMaster.h"

extern Arduino_DebugUtils Debug;

#define RXD2 16
#define TXD2 17
HardwareSerial nextionSerial(2);

AsyncWebServer server(OTA_NEXTION_PORT);

// Forward declarations
void updateNextion();
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static File file;

  if (!index) {
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
    }
  }

  if (final) {
    file.close();
    Debug.print(DBG_INFO, "[OTA] Upload completed");
    request->send(200, "text/plain", "File uploaded successfully");
  }
}

void initOTA(void) {
}

void otaTask(void *pvParameters) {
  Debug.print(DBG_INFO, "[TASKS] otaTask started on core %d", xPortGetCoreID());
  while (!startTasks);
  Debug.print(DBG_DEBUG, "[TASKS] otaTask running...");
  vTaskDelay(DT13);

  esp_task_wdt_add(NULL);
  TickType_t period = pdMS_TO_TICKS(980);
  TickType_t ticktime = xTaskGetTickCount();

  #ifdef CHRONO
  unsigned long td;
  int t_act=0, t_min=999, t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  Debug.print(DBG_INFO, "[OTA] Initializing SPIFFS...");
  bool spiffsOk = false;
  if (!SPIFFS.begin(true)) { // true = format on fail
    Debug.print(DBG_ERROR, "[OTA] SPIFFS mount failed");
  } else {
    Debug.print(DBG_INFO, "[OTA] SPIFFS mounted successfully");
    spiffsOk = true;
  }

  Debug.print(DBG_INFO, "[OTA] Initializing serial...");
  nextionSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Debug.print(DBG_INFO, "[OTA] Waiting for WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_task_wdt_reset();
  }

  if (spiffsOk) {
    Debug.print(DBG_INFO, "[OTA] Setting up server...");
    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", "<form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='file' accept='.tft'><input type='submit' value='Upload'></form>");
    });
    server.on(OTA_NEXTION_PATH, HTTP_POST, [](AsyncWebServerRequest *request) {}, handleFileUpload);
    server.begin();
    Debug.print(DBG_INFO, "[OTA] Web server started on port %d", OTA_NEXTION_PORT);
  } else {
    Debug.print(DBG_WARNING, "[OTA] No web server - SPIFFS unavailable");
  }

  for (;;) {
    esp_task_wdt_reset();

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
  nextionSerial.print("whmi-wri 1,115200,0");
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  delay(1000);

  File file = SPIFFS.open("/nextion.tft", FILE_READ);
  if (!file) {
    Debug.print(DBG_ERROR, "[OTA] Could not open TFT file");
    return;
  }

  while (file.available()) {
    uint8_t buffer[512];
    size_t bytesRead = file.readBytes((char *)buffer, sizeof(buffer));
    nextionSerial.write(buffer, bytesRead);
    delay(10);
  }
  file.close();

  Debug.print(DBG_INFO, "[OTA] Nextion update completed");
}