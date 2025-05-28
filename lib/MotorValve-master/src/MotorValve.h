#ifndef MOTORVALVE_H
#define MOTORVALVE_H

#include <Arduino.h>
#include "I2CConfig.h"

// Default-Wert für Pins ohne Zuordnung
extern const PCF_Pin NO_PIN;

#define ON 1
#define OFF 0
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

class MotorValve
{
public:
    MotorValve(PCF_Pin OpenPin, PCF_Pin ClosePin, int StartAngle, int MaxAngle,
               int TimeToMaxAngle, int CalibrationDirection, const char* Name);

    void open();
    void close();
    void halfOpen();
    void setTargetAngle(int target);
    void loop();
    void calibrate();
    boolean isOpen();
    boolean isClosed();
    boolean isHalfOpen();
    int StartAngle();
    int HalfAngle();
    int MaxAngle();
    int CurrentAngle();
    int getCurrentAngle();
    bool isOpening();
    bool isClosing();
    bool isOperating();
    bool isCalibrating();
    const char *getStatus();
    uint8_t getCurrentState(uint8_t address);
    void synchronizeWithShadow(); // Neue Methode für Synchronisierung

private:
    void setSignal(PCF_Pin pin, uint8_t state);
    void setOpenSignal();
    void setCloseSignal();
    void setIdle();
    PCF_Pin openPin;
    PCF_Pin closePin;
    int startAngle;
    int maxAngle;
    int halfAngle;
    int timeToMaxAngle;
    int calibrationDirection;
    const char *instanceName;
    int currentAngle;
    int targetAngle;
    bool operating = false;
    bool calibrating = false;
    bool opening = false;
    bool closing = false;
    bool openPinState = false;  // Neues Register für openPin
    bool closePinState = false; // Neues Register für closePin
    unsigned long operationStartTime = 0;
    unsigned long calibrationStartTime = 0;
};

#endif // MOTORVALVE_H