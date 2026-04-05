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
        states[i].shadowState = 0xFF; // Initialize shadow register to all high (default)
        states[i].pendingWrite = false;
        states[i].outputState = 0xFF;
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
        states[i].pendingWrite = false;
        states[i].outputState = 0xFF;
        states[i].outputMutex = xSemaphoreCreateMutex();
        states[i].lastUpdate = 0;
        states[i].errorCount = 0;
        queueUpdate(PCF_ADDRESSES[i], 0xFF, NULL); // Initialize hardware to match shadow
    }

    xTaskCreatePinnedToCore(updateTask, "PCF_Update", 4096, this, tskIDLE_PRIORITY + 2, &this->taskHandle, 1); // Higher priority
    Debug.print(DBG_INFO, "[PCF8574Manager] Initialized and update task started");
}

void PCF8574Manager::queuePinUpdate(uint8_t address, uint8_t pin, bool state, QueueHandle_t responseQueue) {
    if (pin > 7) return;
    
    // Update shadow register
    if (xSemaphoreTake(this->stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 4; i++) {
            if (PCF_ADDRESSES[i] == address) {
                uint8_t pinMask = (1 << pin);
                states[i].shadowState = state ? (states[i].shadowState & ~pinMask) : (states[i].shadowState | pinMask); // Active-Low
                Debug.print(DBG_VERBOSE, "[PCF8574Manager] Shadow state updated for 0x%02X, pin %d to %d (new shadow: 0x%02X)",
                            address, pin, state ? 0 : 1, states[i].shadowState);
                break;
            }
        }
        xSemaphoreGive(this->stateMutex);
    } else {
        Debug.print(DBG_WARNING, "[PCF8574Manager] Failed to take state mutex for shadow update");
    }

    // Queue update to hardware
    queueUpdate(address, getShadowState(address), responseQueue);
}

void PCF8574Manager::queueUpdate(uint8_t address, uint8_t state, QueueHandle_t responseQueue) {
    PCFUpdate update;
    update.address = address;
    update.state = state;
    update.responseQueue = responseQueue;

    Debug.print(DBG_VERBOSE, "[PCF8574Manager] Queuing update for 0x%02X with state 0x%02X", address, state);
    if (xQueueSend(this->updateQueue, &update, pdMS_TO_TICKS(10)) != pdTRUE) {
        Debug.print(DBG_ERROR, "[PCF8574Manager] Failed to queue update for 0x%02X: Queue full", address);
        I2CError = true; // Set I2CError on queue full
    }
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
                Debug.print(DBG_VERBOSE, "[PCF_Update] Writing state 0x%02X to 0x%02X", update.state, update.address);
                Wire.beginTransmission(update.address);
                Wire.write(update.state);
                uint8_t result = Wire.endTransmission();

                if (result == 0) {
                    // RACE CONDITION FIX: Do NOT overwrite shadowState here with update.state.
                    // update.state was captured at queue-time; by write-time another queuePinUpdate()
                    // may have already updated shadowState to a newer value.  Overwriting would
                    // silently lose those newer pin changes.  The shadow is authoritative — it is
                    // updated atomically in queuePinUpdate() and is always the desired state.
                    // We only clear pendingWrite and the error counter here.
                    if (xSemaphoreTake(manager->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                manager->states[i].pendingWrite = false;
                                manager->states[i].errorCount = 0;
                                response.success = true;
                                I2CError = false;
                                Debug.print(DBG_VERBOSE, "[PCF_Update] Write OK for 0x%02X (shadow preserved: 0x%02X)",
                                            update.address, manager->states[i].shadowState);
                                break;
                            }
                        }
                        xSemaphoreGive(manager->stateMutex);
                    }
                } else {
                    response.errorCode = result;
                    Debug.print(DBG_ERROR, "[PCF_Update] Write failed to 0x%02X, error: %d", update.address, result);
                    if (xSemaphoreTake(manager->stateMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < 4; i++) {
                            if (PCF_ADDRESSES[i] == update.address) {
                                manager->states[i].errorCount++;
                                if (manager->states[i].errorCount >= 5) {
                                    I2CError = true; // Set I2CError on critical error threshold
                                    Debug.print(DBG_ERROR, "[PCF_Update] Critical error threshold reached for 0x%02X: %d errors",
                                                update.address, manager->states[i].errorCount);
                                    // Optional: Reset attempt could be implemented here
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