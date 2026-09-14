#ifndef OTA_H
#define OTA_H

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESPAsyncWebServer.h>
extern AsyncWebServer server;

void initOTA(void);
void startOTATask(void);
void otaTask(void*);
/** Bind ArduinoOTA (UDP :OTA_PORT) once WiFi is up. Safe to call repeatedly. */
void poolArduinoOtaEnsureStarted(void);

#endif // OTA_H