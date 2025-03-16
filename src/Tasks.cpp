#include "Tasks.h"
#include "PoolMaster.h"
#include "Ota.h"
#include "Config.h"

// External task functions
extern void PoolMaster(void*);
extern void AnalogPoll(void*);
extern void pHRegulation(void*);
extern void ChlorSaltRegulation(void*);
extern void getTemp(void*);
extern void readBME280(void*);
extern void ProcessCommand(void*);
extern void FlowMeasures(void*);
extern void SettingsPublish(void*);
extern void MeasuresPublish(void*);
extern void StatusLights(void*);
extern void I2CPollingTask(void*);

extern bool lockI2C();
extern void unlockI2C();

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
    AnalogPoll,
    "AnalogPoll",
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
    getTemp,
    "GetTemp",
    STACK_T4,
    NULL,
    PRIORITY_T4,
    nullptr,
    0
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

  // I2C Polling Task für PCF8574-Geräte
  xTaskCreatePinnedToCore(
    I2CPollingTask,
    "I2CPolling",
    STACK_T13,
    NULL,
    PRIORITY_T13,
    nullptr,
    app_cpu
  );

  // OTA task for Nextion display
  /*Debug.print(DBG_INFO, "[TASKS] Creating OTA task...");
  xTaskCreatePinnedToCore(
    otaTask,
    "OTATask",
    STACK_T14,
    NULL,
    PRIORITY_T14,
    nullptr,
    app_cpu
  );

  Debug.print(DBG_INFO, "[TASKS] OTA task created");
  vTaskDelay(DT14); // Apply start offset for OTA task */
}

