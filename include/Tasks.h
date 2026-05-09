#ifndef TASKS_H
#define TASKS_H

#pragma once
#include "PoolMaster.h"
#include "Ota.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern void PoolMaster(void*);
extern void CombinedPollingTask(void*);
extern void pHRegulation(void*);
extern void ChlorSaltRegulation(void*);
extern void TempTask(void* pvParameters);
extern void readBME280(void*);
extern void ProcessCommand(void*);
extern void FlowMeasures(void*);
extern void SettingsPublish(void*);
extern void MeasuresPublish(void*);
extern void StatusLights(void*);
extern void otaTask(void*);

// T14: MatterSyncTask (created only if MATTER_ENABLED && ENABLE_TASK_T14)
#ifdef MATTER_ENABLED
extern void MatterSyncTask(void*);
#endif

// T15: PCF8574 I2C update worker — PCF8574Manager::updateTaskEntry, created in createTasks()
// when ENABLE_TASK_T15 (Config.h). Static class method; see PCF8574Manager.h.

// Globale Variablen
void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle);

#endif // TASKS_H