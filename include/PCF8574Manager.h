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
    uint8_t shadowState;    // Shadow-Register für den gewünschten Zustand (einzige Quelle der Wahrheit)
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
    /** Created from createTasks() when ENABLE_TASK_T15 (Config.h) — not from init(). */
    static void updateTaskEntry(void* parameter);
    bool queuePinUpdate(uint8_t address, uint8_t pin, bool state, QueueHandle_t responseQueue = NULL);
    /** Ersetzt den kompletten 8-Bit-Shadow und löst Schreiben aus (z. B. Status-LED-Port 0x24). */
    bool queueFullStateUpdate(uint8_t address, uint8_t state, QueueHandle_t responseQueue = NULL);
    bool queueUpdate(uint8_t address, uint8_t state, QueueHandle_t responseQueue = NULL);
    uint8_t getState(uint8_t address);

private:
    PCF8574Manager(); // Privater Konstruktor für Singleton
    static const uint8_t PCF_ADDRESSES[];
    PCF8574State states[4]; // Interne Zustände
    QueueHandle_t updateQueue; // Queue für Updates
    SemaphoreHandle_t stateMutex; // Mutex für Zustandszugriff
    uint8_t getShadowState(uint8_t address); // Private Methode für Shadow-State-Zugriff
    // Main loop for T15 (was static updateTask)
    void runUpdateLoop();
};

#endif