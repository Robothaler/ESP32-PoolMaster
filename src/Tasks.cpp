#include "Tasks.h"
#include "PoolMaster.h"
#include "Ota.h"
#include "Config.h"
#ifdef MATTER_ENABLED
#include "MatterBridge.h"
#endif
#include <freertos/idf_additions.h>

// Allocate task stacks in PSRAM to preserve ~70 KB of scarce internal DRAM.
// CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y and
// CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y are both set in sdkconfig.
//
// IMPORTANT: A task whose stack lives in PSRAM MUST NOT call any SPI-flash
// routine (SPIFFS, NVS, esp_partition_*, OTA, …). Such routines disable the
// flash cache, which makes PSRAM (and therefore the task's own stack)
// inaccessible. ESP-IDF guards against this with
// `assert(esp_task_stack_is_sane_cache_disabled())` in cache_utils.c — a
// failure of that assert reboots the device immediately. For flash-touching
// tasks use CREATE_TASK_INTERNAL below to keep the stack in DRAM.
#define CREATE_TASK_SPIRAM(func, name, stack, prio, handle, core) \
  do { \
    BaseType_t _r = xTaskCreatePinnedToCoreWithCaps( \
        func, name, stack, NULL, prio, handle, core, \
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); \
    if (_r != pdPASS) { \
      Debug.print(DBG_ERROR, "[TASKS] Failed to create %s (SPIRAM), falling back. Heap: %d", \
                  name, ESP.getFreeHeap()); \
      xTaskCreatePinnedToCore(func, name, stack, NULL, prio, handle, core); \
    } else { \
      Debug.print(DBG_INFO, "[TASKS] Created %s (SPIRAM stack), Heap: %d", \
                  name, ESP.getFreeHeap()); \
    } \
  } while (0)

// For tasks that touch SPI flash directly (SPIFFS, NVS, OTA, …): stack must
// live in internal DRAM, otherwise spi_flash_disable_interrupts_caches_…
// will assert and panic-reboot the device.
#define CREATE_TASK_INTERNAL(func, name, stack, prio, handle, core) \
  do { \
    BaseType_t _r = xTaskCreatePinnedToCore( \
        func, name, stack, NULL, prio, handle, core); \
    if (_r != pdPASS) { \
      Debug.print(DBG_ERROR, "[TASKS] Failed to create %s (internal stack). Heap: %d", \
                  name, ESP.getFreeHeap()); \
    } else { \
      Debug.print(DBG_INFO, "[TASKS] Created %s (internal stack), Heap: %d", \
                  name, ESP.getFreeHeap()); \
    } \
  } while (0)

void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle) {
  Debug.print(DBG_INFO, "[TASKS] Free Heap before tasks: %d", ESP.getFreeHeap());

  CREATE_TASK_SPIRAM(CombinedPollingTask, "CombinedPolling",    STACK_T1,  PRIORITY_T1,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(ProcessCommand,      "ProcessCommand",     STACK_T2,  PRIORITY_T2,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(PoolMaster,          "PoolMaster",         STACK_T3,  PRIORITY_T3,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(TempTask,            "TempTask",           STACK_T4,  PRIORITY_T4,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(readBME280,          "readBME280",         STACK_T5,  PRIORITY_T5,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(ChlorSaltRegulation, "ChlorSaltRegulation",
                     max(STACK_T6, STACK_T7), max(PRIORITY_T6, PRIORITY_T7), nullptr,   app_cpu);
  CREATE_TASK_SPIRAM(pHRegulation,        "pHRegulation",       STACK_T8,  PRIORITY_T8,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(FlowMeasures,        "FlowMeasures",       STACK_T9,  PRIORITY_T9,  nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(StatusLights,        "StatusLights",       STACK_T10, PRIORITY_T10, nullptr,         app_cpu);
  CREATE_TASK_SPIRAM(MeasuresPublish,     "MeasuresPublish",    STACK_T11, PRIORITY_T11, pubMeasTaskHandle, app_cpu);
  CREATE_TASK_SPIRAM(SettingsPublish,     "SettingsPublish",    STACK_T12, PRIORITY_T12, pubSetTaskHandle,  app_cpu);

  Debug.print(DBG_INFO, "[TASKS] Creating OTA task...");
  // OTATask calls SPIFFS.begin() → SPI-flash access → cache disabled.
  // Stack MUST be in internal DRAM (see comment above CREATE_TASK_SPIRAM).
  CREATE_TASK_INTERNAL(otaTask,           "OTATask",            STACK_T13, PRIORITY_T13, nullptr,         app_cpu);
  Debug.print(DBG_INFO, "[TASKS] OTA task created");

#ifdef MATTER_ENABLED
  CREATE_TASK_SPIRAM(MatterSyncTask, "MatterSync", STACK_T14, PRIORITY_T14, nullptr, app_cpu);
  Debug.print(DBG_INFO, "[TASKS] MatterSyncTask created (period=%d ms)", PT14);
#endif
}
