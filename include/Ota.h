#ifndef OTA_H
#define OTA_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESPAsyncWebServer.h>
extern AsyncWebServer server;

void initOTA(void);
void startOTATask(void);
void otaTask(void*);

#endif // OTA_H