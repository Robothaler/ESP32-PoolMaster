#ifndef PCF8574MANAGER_H
#define PCF8574MANAGER_H

#include <Arduino.h>
#include <Wire.h>

struct PCFUpdate {
    uint8_t address;
    uint8_t state;
    QueueHandle_t responseQueue;
};

struct PCFUpdateResponse {
    bool success;
    uint8_t errorCode; // 0 = Erfolg, sonst I2C-Fehlercode
};

struct PCF8574State {
    uint8_t shadowState;    // Shadow-Register für den gewünschten Zustand
    bool pendingWrite;
    uint8_t outputState;
    SemaphoreHandle_t outputMutex;
    unsigned long lastUpdate;
    uint8_t errorCount;
};

class PCF8574Manager {
public:
    static PCF8574Manager& getInstance() {
        static PCF8574Manager instance;
        return instance;
    }
    void init();
    void queuePinUpdate(uint8_t address, uint8_t pin, bool state, QueueHandle_t responseQueue = NULL);
    void queueUpdate(uint8_t address, uint8_t state, QueueHandle_t responseQueue = NULL);
    uint8_t getState(uint8_t address);

private:
    PCF8574Manager(); // Privater Konstruktor für Singleton
    static const uint8_t PCF_ADDRESSES[];
    PCF8574State states[4]; // Interne Zustände
    QueueHandle_t updateQueue; // Queue für Updates
    SemaphoreHandle_t stateMutex; // Mutex für Zustandszugriff
    TaskHandle_t taskHandle; // Task-Handle für updateTask
    uint8_t getShadowState(uint8_t address); // Private Methode für Shadow-State-Zugriff
    static void updateTask(void* parameter);
};

#endif