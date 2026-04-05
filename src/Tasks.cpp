#include "Tasks.h"
#include "PoolMaster.h"
#include "Ota.h"
#include "Config.h"
#ifdef MATTER_ENABLED
#include "MatterBridge.h"
#endif

void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle) {
  BaseType_t result;
  Debug.print(DBG_INFO, "[TASKS] Free Heap before tasks: %d", ESP.getFreeHeap());

  #define CREATE_TASK(func, name, stack, prio, handle) \
  result = xTaskCreatePinnedToCore(func, name, stack, NULL, prio, handle, app_cpu); \
  if (result != pdPASS) { \
    Debug.print(DBG_ERROR, "[TASKS] Failed to create %s, Heap: %d", name, ESP.getFreeHeap()); \
    while (1) vTaskDelay(1000 / portTICK_PERIOD_MS); \
  } else { \
    Debug.print(DBG_INFO, "[TASKS] Created %s, Heap: %d", name, ESP.getFreeHeap()); \
  }

  // Analog measurement polling task
  xTaskCreatePinnedToCore(
    CombinedPollingTask,
    "CombinedPolling",
    STACK_T1,
    NULL,
    PRIORITY_T1,
    nullptr,
    app_cpu
  );

  // MQTT commands processing
  xTaskCreatePinnedToCore(
    ProcessCommand,
    "ProcessCommand",
    STACK_T2,
    NULL,
    PRIORITY_T2,
    nullptr,
    app_cpu
  );

  // PoolMaster: Supervisory task
  xTaskCreatePinnedToCore(
    PoolMaster,
    "PoolMaster",
    STACK_T3,
    NULL,
    PRIORITY_T3,
    nullptr,
    app_cpu
  );


  // Temperatures measurement
  xTaskCreatePinnedToCore(
    TempTask,
    "TempTask",
    STACK_T4,
    NULL,
    PRIORITY_T4,
    nullptr,
    app_cpu
  );

  // BME280 measurement
  xTaskCreatePinnedToCore(
    readBME280,
    "readBME280",
    STACK_T5,
    NULL,
    PRIORITY_T5,
    nullptr,
    app_cpu
  );

  // Combined Chlor/Salt regulation loop
  xTaskCreatePinnedToCore(
    ChlorSaltRegulation,
    "ChlorSaltRegulation",
    max(STACK_T6, STACK_T7), // Größerer Stack von beiden
    NULL,
    max(PRIORITY_T6, PRIORITY_T7), // Höhere Priorität (T6)
    nullptr,
    app_cpu
  );

  // pH regulation loop
  xTaskCreatePinnedToCore(
    pHRegulation,
    "pHRegulation",
    STACK_T8,
    NULL,
    PRIORITY_T8,
    nullptr,
    app_cpu
  );

  // Flow measurement polling task
  xTaskCreatePinnedToCore(
    FlowMeasures,
    "FlowMeasures",
    STACK_T9,
    NULL,
    PRIORITY_T9,
    nullptr,
    app_cpu
  );

  // Status lights display
  xTaskCreatePinnedToCore(
    StatusLights,
    "StatusLights",
    STACK_T10,
    NULL,
    PRIORITY_T10,
    nullptr,
    app_cpu
  );  

  // Measures MQTT publish
  xTaskCreatePinnedToCore(
    MeasuresPublish,
    "MeasuresPublish",
    STACK_T11,
    NULL,
    PRIORITY_T11,
    pubMeasTaskHandle,
    app_cpu
  );

  // MQTT Settings publish
  xTaskCreatePinnedToCore(
    SettingsPublish,
    "SettingsPublish",
    STACK_T12,
    NULL,
    PRIORITY_T12,
    pubSetTaskHandle,
    app_cpu
  );

  // OTA task: sets up SPIFFS, the Nextion web-upload server, and flashes
  // the Nextion display when a new .tft file has been uploaded.
  Debug.print(DBG_INFO, "[TASKS] Creating OTA task...");
  xTaskCreatePinnedToCore(
    otaTask,
    "OTATask",
    STACK_T13,
    NULL,
    PRIORITY_T13,
    nullptr,
    app_cpu
  );
  Debug.print(DBG_INFO, "[TASKS] OTA task created");

  // T14: Matter Bridge state sync (only when Matter is compiled in)
#ifdef MATTER_ENABLED
  xTaskCreatePinnedToCore(
    MatterSyncTask,   // Pushes pool state → Matter attribute cache every PT14 ms
    "MatterSync",
    STACK_T14,
    NULL,
    PRIORITY_T14,
    nullptr,
    app_cpu           // Core 1 — CHIP stack lock handles cross-core safety
  );
  Debug.print(DBG_INFO, "[TASKS] MatterSyncTask created (period=%d ms)", PT14);
#endif
}

