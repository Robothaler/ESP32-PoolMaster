#ifndef TASKS_H
#define TASKS_H

#include "Tasks.h"
#include "PoolMaster.h"
#include "Ota.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

// Globale Variablen
void createTasks(int app_cpu, TaskHandle_t* pubSetTaskHandle, TaskHandle_t* pubMeasTaskHandle);

#endif // TASKS_H