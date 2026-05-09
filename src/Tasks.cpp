#include "Tasks.h"
#include "PoolMaster.h"
#include "Ota.h"
#include "PCF8574Manager.h"
#include "Config.h"
#ifdef MATTER_ENABLED
#include "MatterAppTaskSuspend.h"
#include <esp_task_wdt.h>
#endif
#include <freertos/idf_additions.h>

// Task handles for optional Matter commissioning suspend (Tasks.cpp); always
// retained so createTasks() compiles in non-Matter builds too.
static TaskHandle_t s_t_combined;
static TaskHandle_t s_t_process;
static TaskHandle_t s_t_pool;
static TaskHandle_t s_t_temp;
static TaskHandle_t s_t_bme;
static TaskHandle_t s_t_chlor;
static TaskHandle_t s_t_ph;
static TaskHandle_t s_t_flow;
static TaskHandle_t s_t_status;
static TaskHandle_t s_t_meas_pub;
static TaskHandle_t s_t_set_pub;
static TaskHandle_t s_t_ota;
static TaskHandle_t s_t_pcf8574;
#ifdef MATTER_ENABLED
static TaskHandle_t s_t_matter_sync;
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
static bool s_gap_pase_tasks_suspended;
/** If true, next PoolMaster loop iteration re-calls esp_task_wdt_add (after PASE + TWDT delete). */
static volatile bool s_pool_wants_wdt_rearm_after_pase;

static void matterWdtUnsubscribeForSuspend(TaskHandle_t h)
{
    if (h == nullptr) {
        return;
    }
    (void) esp_task_wdt_delete(h);
}
#endif

void matterPoolMasterWdtRearmIfNeededAfterPase(void)
{
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
    if (!s_pool_wants_wdt_rearm_after_pase) {
        return;
    }
    s_pool_wants_wdt_rearm_after_pase = false;
    (void) esp_task_wdt_add(NULL);
#endif
}

static void matterSuspendOne(TaskHandle_t h)
{
    TaskHandle_t const self = xTaskGetCurrentTaskHandle();
    if (h != nullptr && h != self) {
        vTaskSuspend(h);
    }
}

static void matterResumeOne(TaskHandle_t h)
{
    if (h != nullptr) {
        vTaskResume(h);
    }
}

void matterSuspendAppTasksForGapPase(void)
{
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
    if (s_gap_pase_tasks_suspended) {
        return;
    }
    s_gap_pase_tasks_suspended = true;
    /* PoolMaster registers with TWDT; a suspended task cannot call esp_task_wdt_reset(). */
    matterWdtUnsubscribeForSuspend(s_t_pool);
    matterSuspendOne(s_t_combined);
    matterSuspendOne(s_t_process);
    matterSuspendOne(s_t_pool);
    matterSuspendOne(s_t_temp);
    matterSuspendOne(s_t_bme);
    matterSuspendOne(s_t_chlor);
    matterSuspendOne(s_t_ph);
    matterSuspendOne(s_t_flow);
    matterSuspendOne(s_t_status);
    matterSuspendOne(s_t_meas_pub);
    matterSuspendOne(s_t_set_pub);
    matterSuspendOne(s_t_ota);
    matterSuspendOne(s_t_pcf8574);
    /* Do NOT suspend MatterSyncTask: it uses esp_matter::chip_stack_lock via matterBridgeSync /
     * updateOnOff paths — suspending mid-lock freezes the CHIP stack and BLE then fails with
     * "Failed to post event to CHIP Platform event queue" / 0x01000000 (logs/serial-20260503_161044). */
    Debug.print(DBG_WARNING, "[TASKS] Matter BLE PASE: suspended pool app tasks (vTaskSuspend)");
#endif
}

void matterResumeAppTasksAfterGapPase(void)
{
#if MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE
    if (!s_gap_pase_tasks_suspended) {
        return;
    }
    s_gap_pase_tasks_suspended = false;
    if (s_t_pool != nullptr) {
        s_pool_wants_wdt_rearm_after_pase = true;
    }
    matterResumeOne(s_t_combined);
    matterResumeOne(s_t_process);
    matterResumeOne(s_t_pool);
    matterResumeOne(s_t_temp);
    matterResumeOne(s_t_bme);
    matterResumeOne(s_t_chlor);
    matterResumeOne(s_t_ph);
    matterResumeOne(s_t_flow);
    matterResumeOne(s_t_status);
    matterResumeOne(s_t_meas_pub);
    matterResumeOne(s_t_set_pub);
    matterResumeOne(s_t_ota);
    matterResumeOne(s_t_pcf8574);
    Debug.print(DBG_WARNING, "[TASKS] Matter BLE PASE: resumed pool app tasks (vTaskResume)");
#endif
}
#endif // MATTER_ENABLED

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

#if ENABLE_TASK_T1
  CREATE_TASK_SPIRAM(CombinedPollingTask, "CombinedPolling",    STACK_T1,  PRIORITY_T1,  &s_t_combined,   app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED CombinedPolling (ENABLE_TASK_T1=0)");
#endif
#if ENABLE_TASK_T2
  CREATE_TASK_SPIRAM(ProcessCommand,      "ProcessCommand",     STACK_T2,  PRIORITY_T2,  &s_t_process,    app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED ProcessCommand (ENABLE_TASK_T2=0)");
#endif
#if ENABLE_TASK_T3
  CREATE_TASK_SPIRAM(PoolMaster,          "PoolMaster",         STACK_T3,  PRIORITY_T3,  &s_t_pool,       app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED PoolMaster (ENABLE_TASK_T3=0)");
#endif
#if ENABLE_TASK_T4
  CREATE_TASK_SPIRAM(TempTask,            "TempTask",           STACK_T4,  PRIORITY_T4,  &s_t_temp,       app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED TempTask (ENABLE_TASK_T4=0)");
#endif
#if ENABLE_TASK_T5
  CREATE_TASK_SPIRAM(readBME280,          "readBME280",         STACK_T5,  PRIORITY_T5,  &s_t_bme,        app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED readBME280 (ENABLE_TASK_T5=0)");
#endif
#if ENABLE_TASK_T6
  CREATE_TASK_SPIRAM(ChlorSaltRegulation, "ChlorSaltRegulation",
                     max(STACK_T6, STACK_T7), max(PRIORITY_T6, PRIORITY_T7), &s_t_chlor,   app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED ChlorSaltRegulation T6/T7 (ENABLE_TASK_T6=0)");
#endif
#if ENABLE_TASK_T8
  CREATE_TASK_SPIRAM(pHRegulation,        "pHRegulation",       STACK_T8,  PRIORITY_T8,  &s_t_ph,         app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED pHRegulation (ENABLE_TASK_T8=0)");
#endif
#if ENABLE_TASK_T9
  CREATE_TASK_SPIRAM(FlowMeasures,        "FlowMeasures",       STACK_T9,  PRIORITY_T9,  &s_t_flow,       app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED FlowMeasures (ENABLE_TASK_T9=0)");
#endif
#if ENABLE_TASK_T10
  CREATE_TASK_SPIRAM(StatusLights,        "StatusLights",       STACK_T10, PRIORITY_T10, &s_t_status,     app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED StatusLights (ENABLE_TASK_T10=0)");
#endif
#if ENABLE_TASK_T11
  CREATE_TASK_SPIRAM(MeasuresPublish,     "MeasuresPublish",    STACK_T11, PRIORITY_T11, &s_t_meas_pub,   app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED MeasuresPublish (ENABLE_TASK_T11=0)");
#endif
#if ENABLE_TASK_T12
  CREATE_TASK_SPIRAM(SettingsPublish,     "SettingsPublish",    STACK_T12, PRIORITY_T12, &s_t_set_pub,    app_cpu);
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED SettingsPublish (ENABLE_TASK_T12=0)");
#endif

#if ENABLE_TASK_T13
  Debug.print(DBG_INFO, "[TASKS] Creating OTA task...");
  // OTATask calls SPIFFS.begin() → SPI-flash access → cache disabled.
  // Stack MUST be in internal DRAM (see comment above CREATE_TASK_SPIRAM).
  CREATE_TASK_INTERNAL(otaTask,           "OTATask",            STACK_T13, PRIORITY_T13, &s_t_ota,        app_cpu);
  Debug.print(DBG_INFO, "[TASKS] OTA task created");
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED OTATask (ENABLE_TASK_T13=0)");
#endif

#if ENABLE_TASK_T15
  {
    PCF8574Manager* pcfInst = &PCF8574Manager::getInstance();
    BaseType_t _r = xTaskCreatePinnedToCoreWithCaps(
        PCF8574Manager::updateTaskEntry, "PCF_Update", STACK_T15, pcfInst, PRIORITY_T15, &s_t_pcf8574, app_cpu,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (_r != pdPASS) {
      Debug.print(DBG_ERROR, "[TASKS] Failed to create PCF_Update (SPIRAM), falling back. Heap: %d",
                  ESP.getFreeHeap());
      xTaskCreatePinnedToCore(PCF8574Manager::updateTaskEntry, "PCF_Update", STACK_T15, pcfInst, PRIORITY_T15,
                              &s_t_pcf8574, app_cpu);
    } else {
      Debug.print(DBG_INFO, "[TASKS] Created PCF_Update (SPIRAM stack), Heap: %d", ESP.getFreeHeap());
    }
  }
#else
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED PCF8574 update worker (ENABLE_TASK_T15=0)");
#endif

#if defined(MATTER_ENABLED) && ENABLE_TASK_T14
  CREATE_TASK_SPIRAM(MatterSyncTask, "MatterSync", STACK_T14, PRIORITY_T14, &s_t_matter_sync, app_cpu);
  Debug.print(DBG_INFO, "[TASKS] MatterSyncTask created (period=%d ms)", PT14);
#elif defined(MATTER_ENABLED)
  Debug.print(DBG_WARNING, "[TASKS] SKIPPED MatterSyncTask (ENABLE_TASK_T14=0)");
#endif

  if (pubMeasTaskHandle != nullptr) {
    *pubMeasTaskHandle = s_t_meas_pub;
  }
  if (pubSetTaskHandle != nullptr) {
    *pubSetTaskHandle = s_t_set_pub;
  }
}
