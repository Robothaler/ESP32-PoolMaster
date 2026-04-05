#include "PCF8574Manager.h"
#include <Arduino_DebugUtils.h>
#include "Config.h"
#include <Wire.h>

bool lockI2C();
void unlockI2C();

extern Arduino_DebugUtils Debug;

extern bool I2CError;

const uint8_t PCF8574Manager::PCF_ADDRESSES[] = {PCF8574_ADR, PCF8574_I_ADR, PCF8574_II_ADR, PCF8574_III_ADR};

PCF8574Manager::PCF8574Manager() : updateQueue(NULL), stateMutex(NULL), taskHandle(NULL) {
    for (int i = 0; i < 4; i++) {
        states[i].shadowState = 0xFF; // Shadow-Register auf HIGH (inaktiv, active-low) vorinitialisieren
        states[i].lastUpdate = 0;
        states[i].errorCount = 0;
    }
}

void PCF8574Manager::init() {
    this->updateQueue = xQueueCreate(30, sizeof(PCFUpdate)); // Increased capacity
    this->stateMutex = xSemaphoreCreateMutex();
    if (!this->updateQueue || !this->stateMutex) {
        Debug.print(DBG_ERROR, "[PCF8574Manager] Failed to create queue or mutex");
        I2CError = true; // Set I2CError on initialization failure
        return;
    }

    for (int i = 0; i < 4; i++) {
        states[i].shadowState = 0xFF;
        states[i].lastUpdate = 0;
        states[i].errorCount = 0;
        queueUpdate(PCF_ADDRESSES[i], 0xFF, NULL); // Initialize hardware to match shadow
    }

    xTaskCreatePinnedToCore(updateTask, "PCF_Update", 4096, this, tskIDLE_PRIORITY + 2, &this->taskHandle, 1); // Higher priority
    Debug.print(DBG_INFO, "[PCF8574Manager] Initialized and update task started");
}

bool PCF8574Manager::queuePinUpdate(uint8_t address, uint8_t pin, bool state, QueueHandle_t responseQueue) {
    if (pin > 7) return false;

    // Shadow-Register atomar aktualisieren
    if (xSemaphoreTake(this->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (PCF_ADDRESSES[i] == address) {
                uint8_t pinMask = (1 << pin);
                // Active-Low: state=true → Pin LOW (Bit=0), state=false → Pin HIGH (Bit=1)
                states[i].shadowState = state ? (states[i].shadowState & ~pinMask) : (states[i].shadowState | pinMask);
                Debug.print(DBG_VERBOSE, "[PCF8574Manager] Shadow 0x%02X: pin %d → %d (shadow=0x%02X)",
                            address, pin, state ? 0 : 1, states[i].shadowState);
                break;
            }
        }
        xSemaphoreGive(this->stateMutex);
    } else {
        Debug.print(DBG_WARNING, "[PCF8574Manager] Mutex-Timeout beim Shadow-Update für 0x%02X", address);
        return false;
    }

    // Schreib-Signal in Queue stellen (updateTask liest Shadow zur Schreibzeit, nicht jetzt)
    return queueUpdate(address, 0 /*ignoriert*/, responseQueue);
}

bool PCF8574Manager::queueUpdate(uint8_t address, uint8_t /*state_ignored*/, QueueHandle_t responseQueue) {
    // Hinweis: Der state-Parameter wird hier nicht mehr in die Queue gelegt.
    // Der updateTask liest den Shadow zur Schreibzeit, damit keine veralteten
    // Snapshots auf die Hardware geschrieben werden (verhindert Glitches).
    PCFUpdate update;
    update.address = address;
    update.state = 0; // Platzhalter — updateTask liest aktuellen Shadow
    update.responseQueue = responseQueue;

    Debug.print(DBG_VERBOSE, "[PCF8574Manager] Queue-Signal für 0x%02X", address);
    if (xQueueSend(this->updateQueue, &update, pdMS_TO_TICKS(10)) != pdTRUE) {
        Debug.print(DBG_ERROR, "[PCF8574Manager] Queue voll für 0x%02X — Write-Signal verloren", address);
        I2CError = true;
        return false;
    }
    return true;
}

uint8_t PCF8574Manager::getState(uint8_t address) {
    return getShadowState(address); // Always use shadow state
}

uint8_t PCF8574Manager::getShadowState(uint8_t address) {
    uint8_t state = 0xFF;
    if (xSemaphoreTake(this->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (PCF_ADDRESSES[i] == address) {
                state = states[i].shadowState;
                break;
            }
        }
        xSemaphoreGive(this->stateMutex);
    } else {
        Debug.print(DBG_WARNING, "[PCF8574Manager] Failed to take state mutex for shadow read");
    }
    return state;
}

void PCF8574Manager::updateTask(void* parameter) {
    PCF8574Manager* manager = (PCF8574Manager*)parameter;
    PCFUpdate update;
    PCFUpdateResponse response;

    for (;;) {
        if (xQueueReceive(manager->updateQueue, &update, pdMS_TO_TICKS(10)) == pdTRUE) {
            response.success = false;
            response.errorCode = 0;

            if (lockI2C()) {
                // Shadow zur Schreibzeit lesen — nicht den Queue-Snapshot.
                // Damit werden mehrere gestaute Queue-Einträge für dieselbe Adresse
                // alle mit dem aktuellsten Sollwert geschrieben → keine Glitches durch
                // veraltete Zwischenzustände (z.B. kurzes OFF bei MotorValve-Richtungswechsel).
                uint8_t stateToWrite = 0xFF;
                if (xSemaphoreTake(manager->stateMutex, portMAX_DELAY) == pdTRUE) {
                    for (int i = 0; i < 4; i++) {
                        if (PCF_ADDRESSES[i] == update.address) {
                            stateToWrite = manager->states[i].shadowState;
                            break;
                        }
                    }
                    xSemaphoreGive(manager->stateMutex);
                }

                Debug.print(DBG_VERBOSE, "[PCF_Update] Schreibe 0x%02X → 0x%02X", update.address, stateToWrite);
                Wire.beginTransmission(update.address);
                Wire.write(stateToWrite);
                uint8_t result = Wire.endTransmission();

                if (result == 0) {
                    if (xSemaphoreTake(manager->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                manager->states[i].errorCount = 0;
                                manager->states[i].lastUpdate = millis();
                                response.success = true;
                                I2CError = false;
                                Debug.print(DBG_VERBOSE, "[PCF_Update] OK 0x%02X (shadow=0x%02X)",
                                            update.address, manager->states[i].shadowState);
                                break;
                            }
                        }
                        xSemaphoreGive(manager->stateMutex);
                    }
                } else {
                    response.errorCode = result;
                    Debug.print(DBG_ERROR, "[PCF_Update] Fehler 0x%02X, I2C-Code: %d", update.address, result);
                    if (xSemaphoreTake(manager->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                manager->states[i].errorCount++;
                                if (manager->states[i].errorCount >= 5) {
                                    I2CError = true;
                                    Debug.print(DBG_ERROR, "[PCF_Update] Kritische Fehlergrenze 0x%02X: %d Fehler",
                                                update.address, manager->states[i].errorCount);
                                }
                                break;
                            }
                        }
                        xSemaphoreGive(manager->stateMutex);
                    }
                }
                unlockI2C();
            } else {
                response.errorCode = 255; // Custom code for I2C lock failure
                I2CError = true; // Set I2CError on I2C lock failure
                Debug.print(DBG_ERROR, "[PCF_Update] Failed to lock I2C for 0x%02X", update.address);
            }

            if (update.responseQueue) {
                xQueueSend(update.responseQueue, &response, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}