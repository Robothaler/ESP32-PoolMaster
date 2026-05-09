#include "PCF8574Manager.h"
#include <Arduino_DebugUtils.h>
#include "Config.h"
#include <Wire.h>

bool lockI2C();
void unlockI2C();

extern Arduino_DebugUtils Debug;

extern bool I2CError;

const uint8_t PCF8574Manager::PCF_ADDRESSES[] = {PCF8574_ADR, PCF8574_I_ADR, PCF8574_II_ADR, PCF8574_III_ADR};

PCF8574Manager::PCF8574Manager() : updateQueue(NULL), stateMutex(NULL) {
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
    }

#if ENABLE_TASK_T15
    for (int i = 0; i < 4; i++) {
        queueUpdate(PCF_ADDRESSES[i], 0xFF, NULL); // Initialize hardware to match shadow (drained when T15 starts)
    }
#else
    // No worker task — push initial state synchronously
    for (int i = 0; i < 4; i++) {
        if (lockI2C()) {
            Wire.beginTransmission(PCF_ADDRESSES[i]);
            Wire.write(0xFF);
            uint8_t result = Wire.endTransmission();
            if (result != 0) {
                Debug.print(DBG_ERROR, "[PCF8574Manager] Initial sync write failed 0x%02X code=%u",
                            PCF_ADDRESSES[i], result);
                I2CError = true;
            }
            unlockI2C();
        }
    }
#endif
    Debug.print(DBG_INFO,
                "[PCF8574Manager] Initialized (T15 worker from createTasks() if ENABLE_TASK_T15)");
}

void PCF8574Manager::updateTaskEntry(void* parameter) {
    PCF8574Manager* manager = static_cast<PCF8574Manager*>(parameter);
    if (manager) {
        manager->runUpdateLoop();
    }
}

void PCF8574Manager::runUpdateLoop() {
    PCFUpdate update;
    PCFUpdateResponse response;

    for (;;) {
        if (xQueueReceive(this->updateQueue, &update, pdMS_TO_TICKS(10)) == pdTRUE) {
            response.success = false;
            response.errorCode = 0;

            if (lockI2C()) {
                uint8_t stateToWrite = 0xFF;
                if (xSemaphoreTake(this->stateMutex, portMAX_DELAY) == pdTRUE) {
                    for (int i = 0; i < 4; i++) {
                        if (PCF_ADDRESSES[i] == update.address) {
                            stateToWrite = this->states[i].shadowState;
                            break;
                        }
                    }
                    xSemaphoreGive(this->stateMutex);
                }

                Debug.print(DBG_VERBOSE, "[PCF_Update] Schreibe 0x%02X → 0x%02X", update.address, stateToWrite);
                Wire.beginTransmission(update.address);
                Wire.write(stateToWrite);
                uint8_t result = Wire.endTransmission();

                if (result == 0) {
                    if (xSemaphoreTake(this->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                this->states[i].errorCount = 0;
                                this->states[i].lastUpdate = millis();
                                response.success = true;
                                I2CError = false;
                                Debug.print(DBG_VERBOSE, "[PCF_Update] OK 0x%02X (shadow=0x%02X)",
                                            update.address, this->states[i].shadowState);
                                break;
                            }
                        }
                        xSemaphoreGive(this->stateMutex);
                    }
                } else {
                    response.errorCode = result;
                    Debug.print(DBG_ERROR, "[PCF_Update] Fehler 0x%02X, I2C-Code: %d", update.address, result);
                    if (xSemaphoreTake(this->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                this->states[i].errorCount++;
                                if (this->states[i].errorCount >= 5) {
                                    I2CError = true;
                                    Debug.print(DBG_ERROR, "[PCF_Update] Kritische Fehlergrenze 0x%02X: %d Fehler",
                                                update.address, this->states[i].errorCount);
                                }
                                break;
                            }
                        }
                        xSemaphoreGive(this->stateMutex);
                    }
                }
                unlockI2C();
            } else {
                response.errorCode = 255;
                I2CError = true;
                Debug.print(DBG_ERROR, "[PCF_Update] Failed to lock I2C for 0x%02X", update.address);
            }

            if (update.responseQueue) {
                xQueueSend(update.responseQueue, &response, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool PCF8574Manager::queuePinUpdate(uint8_t address, uint8_t pin, bool state, QueueHandle_t responseQueue) {
#if !ENABLE_TASK_T15
    (void)address;
    (void)pin;
    (void)state;
    (void)responseQueue;
    Debug.print(DBG_ERROR, "[PCF8574Manager] queuePinUpdate: ENABLE_TASK_T15=0 — no worker");
    return false;
#else
    if (pin > 7) return false;

    if (xSemaphoreTake(this->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (PCF_ADDRESSES[i] == address) {
                uint8_t pinMask = (1 << pin);
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

    return queueUpdate(address, 0, responseQueue);
#endif
}

bool PCF8574Manager::queueFullStateUpdate(uint8_t address, uint8_t state, QueueHandle_t responseQueue) {
#if !ENABLE_TASK_T15
    (void)address;
    (void)state;
    (void)responseQueue;
    return false;
#endif
    if (xSemaphoreTake(this->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool found = false;
        for (int i = 0; i < 4; i++) {
            if (PCF_ADDRESSES[i] == address) {
                states[i].shadowState = state;
                found = true;
                break;
            }
        }
        xSemaphoreGive(this->stateMutex);
        if (!found) return false;
    } else {
        Debug.print(DBG_WARNING, "[PCF8574Manager] Mutex-Timeout queueFullStateUpdate 0x%02X", address);
        return false;
    }
    return queueUpdate(address, 0, responseQueue);
}

bool PCF8574Manager::queueUpdate(uint8_t address, uint8_t /*state_ignored*/, QueueHandle_t responseQueue) {
#if !ENABLE_TASK_T15
    (void)address;
    (void)responseQueue;
    return false;
#endif
    // Der Worker liest den Shadow zur Schreibzeit, damit keine veralteten
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
