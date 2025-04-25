/*
            MotorValve - A simple library for Arduino to handle motorized valves for home-pools. 
                 (c) Robothaler <robothaler@web.de> 2023
Features: 
* offers an easy way to open, close and half-open a motorized valve or even just type in your required angle
* you can use this library for Belimo gearmotors or motorized valves with no angle sensor, the library keeps track of the angle
* set start and max angle and the time from start angle to max angle for calculations
* you can also see the status of the valves (open, closed, halfOpen, isCalibrating, isOperating)
* you can easily start calibration to ensure position, best is to do daily calibration
* You can use this library with standard Pins and different PCF8574-Pins
*/

#include "MotorValve.h"
#include "PCF8574.h"                // IO-Portexpander
#include "PCF8574Manager.h"         // PCF8574 Manager class
#include <Arduino_DebugUtils.h>     // Debug.print
#include <Arduino.h>
#include "I2CConfig.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern Arduino_DebugUtils Debug;

// Constructor
MotorValve::MotorValve(PCF_Pin OpenPin, PCF_Pin ClosePin, int StartAngle, int MaxAngle,
    int TimeToMaxAngle, int CalibrationDirection, const char* Name) {
    this->openPin = OpenPin;
    this->closePin = ClosePin;
    this->startAngle = StartAngle;
    this->maxAngle = MaxAngle;
    this->halfAngle = startAngle + (maxAngle - startAngle) / 2;
    this->timeToMaxAngle = TimeToMaxAngle;
    this->calibrationDirection = CalibrationDirection;
    this->instanceName = Name;
    this->currentAngle = StartAngle;
    this->targetAngle = StartAngle;
}

void MotorValve::open() {
    if (currentAngle != startAngle) {
        targetAngle = startAngle;
        Debug.print(DBG_DEBUG, "[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG, "[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::close() {
    if (currentAngle != maxAngle) {
        targetAngle = maxAngle;
        Debug.print(DBG_DEBUG, "[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG, "[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::halfOpen() {
    int halfAngle = startAngle + (maxAngle - startAngle) / 2;
    if (currentAngle != halfAngle) {
        targetAngle = halfAngle;
        Debug.print(DBG_DEBUG, "[MotorValve] %s Target Angle set to HalfOpen (%d).", instanceName, targetAngle);
        Debug.print(DBG_DEBUG, "[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::setTargetAngle(int target) {
    if (target != targetAngle) {
        targetAngle = target;
        if (targetAngle < startAngle) {
            targetAngle = startAngle;
        } else if (targetAngle > maxAngle) {
            targetAngle = maxAngle;
        } else if (targetAngle == startAngle + (maxAngle - startAngle) / 2) {
            int halfAngle = startAngle + (maxAngle - startAngle) / 2;
            targetAngle = halfAngle;
        }
        Debug.print(DBG_DEBUG, "[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG, "[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::loop() {
    uint8_t openState = getCurrentState(openPin.address);
    uint8_t closeState = getCurrentState(closePin.address);
    int angleDiff = abs(currentAngle - targetAngle);
    int operatingDuration = abs(angleDiff * timeToMaxAngle * 1000 / (maxAngle - startAngle));

    if (!operating && !calibrating) {
        if (currentAngle != targetAngle) {
            if (currentAngle > targetAngle) {
                setOpenSignal();
                opening = true;
            } else {
                setCloseSignal();
                closing = true;
            }
            operating = true;
            operationStartTime = millis();
            Debug.print(DBG_DEBUG, "[MotorValve] %s: Started operating, openState: 0x%02X, closeState: 0x%02X",
                        instanceName, openState, closeState);
        }
    }

    if (calibrating) {
        if ((millis() - calibrationStartTime) >= (timeToMaxAngle + 2) * 1000) {
            setIdle();
            calibrating = false;
            currentAngle = (calibrationDirection == CLOCKWISE) ? startAngle : maxAngle;
            Debug.print(DBG_DEBUG, "[MotorValve] %s: Calibration stopped at angle: %d, openState: 0x%02X, closeState: 0x%02X",
                        instanceName, currentAngle, getCurrentState(openPin.address), getCurrentState(closePin.address));
        }
    }

    if (operating) {
        unsigned long elapsedTime = millis() - operationStartTime;
        float progress = (float)elapsedTime / operatingDuration;
        if (progress > 1.0) progress = 1.0;

        int calculatedAngle;
        if (opening) {
            calculatedAngle = currentAngle - (angleDiff * progress);
            if (calculatedAngle < targetAngle) calculatedAngle = targetAngle;
        } else {
            calculatedAngle = currentAngle + (angleDiff * progress);
            if (calculatedAngle > targetAngle) calculatedAngle = targetAngle;
        }

        if (elapsedTime >= operatingDuration) {
            setIdle();
            opening = false;
            closing = false;
            operating = false;
            currentAngle = targetAngle;
            Debug.print(DBG_DEBUG, "[MotorValve] %s: Reached angle: %d, openState: 0x%02X, closeState: 0x%02X",
                        instanceName, currentAngle, getCurrentState(openPin.address), getCurrentState(closePin.address));
        } else {
            Debug.print(DBG_DEBUG, "[MotorValve] %s: Current angle: %d, openState: 0x%02X, closeState: 0x%02X",
                        instanceName, calculatedAngle, getCurrentState(openPin.address), getCurrentState(closePin.address));
        }
    }
}

void MotorValve::calibrate() {
    setIdle();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (calibrationDirection == CLOCKWISE) {
        setOpenSignal();
        Debug.print(DBG_INFO, "[MotorValve] %s calibrating clockwise, openPin %d on 0x%02X ON, closePin %d on 0x%02X OFF",
                    instanceName, openPin.pin, openPin.address, closePin.pin, closePin.address);
    } else {
        setCloseSignal();
        Debug.print(DBG_INFO, "[MotorValve] %s calibrating counter-clockwise, closePin %d on 0x%02X ON, openPin %d on 0x%02X OFF",
                    instanceName, closePin.pin, closePin.address, openPin.pin, openPin.address);
    }
    calibrationStartTime = millis();
    calibrating = true;
}

void MotorValve::setOpenSignal() {
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    // Deaktiviere Close-Pin zuerst
    if (closePin.address != 0xFF && closePin.pin <= 7) {
        manager.queuePinUpdate(closePin.address, closePin.pin, false); // OFF = HIGH bei activeLow
    }
    // Aktiviere Open-Pin
    if (openPin.address != 0xFF && openPin.pin <= 7) {
        manager.queuePinUpdate(openPin.address, openPin.pin, true);    // ON = LOW bei activeLow
    }
    Debug.print(DBG_INFO, "[MotorValve] %s: Queued openPin %d on 0x%02X to ON, closePin %d on 0x%02X to OFF",
                instanceName, openPin.pin, openPin.address, closePin.pin, closePin.address);
}

void MotorValve::setCloseSignal() {
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    // Deaktiviere Open-Pin zuerst
    if (openPin.address != 0xFF && openPin.pin <= 7) {
        manager.queuePinUpdate(openPin.address, openPin.pin, false);   // OFF = HIGH bei activeLow
    }
    // Aktiviere Close-Pin
    if (closePin.address != 0xFF && closePin.pin <= 7) {
        manager.queuePinUpdate(closePin.address, closePin.pin, true);  // ON = LOW bei activeLow
    }
    Debug.print(DBG_INFO, "[MotorValve] %s: Queued closePin %d on 0x%02X to ON, openPin %d on 0x%02X to OFF",
                instanceName, closePin.pin, closePin.address, openPin.pin, openPin.address);
}

void MotorValve::setIdle() {
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    // Deaktiviere beide Pins
    if (openPin.address != 0xFF && openPin.pin <= 7) {
        manager.queuePinUpdate(openPin.address, openPin.pin, false);   // OFF = HIGH bei activeLow
    }
    if (closePin.address != 0xFF && closePin.pin <= 7) {
        manager.queuePinUpdate(closePin.address, closePin.pin, false); // OFF = HIGH bei activeLow
    }
    Debug.print(DBG_INFO, "[MotorValve] %s: Queued both pins to OFF (openPin %d on 0x%02X, closePin %d on 0x%02X)",
                instanceName, openPin.pin, openPin.address, closePin.pin, closePin.address);
}

void MotorValve::setSignal(PCF_Pin pin, uint8_t state) {
    if (pin.address == 0xFF || pin.pin > 7) return;
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    manager.queuePinUpdate(pin.address, pin.pin, state == ON);
    Debug.print(DBG_VERBOSE, "[MotorValve] %s: Queued pin %d on 0x%02X to %d",
                instanceName, pin.pin, pin.address, state);
}

// returns true only if it is all the way open
boolean MotorValve::isOpen() {
    return (currentAngle == startAngle);
}

// returns true only if it is all the way closed
boolean MotorValve::isClosed() {
    return (currentAngle == maxAngle);
}

// returns true only if it is half open
boolean MotorValve::isHalfOpen() {
    return (currentAngle == startAngle + (maxAngle - startAngle) / 2);
}

// returns the start angle of the valve
int MotorValve::StartAngle() {
    return startAngle;
}

// returns the half angle of the valve
int MotorValve::HalfAngle() {
    return halfAngle;
}

// returns the maximum angle of the valve
int MotorValve::MaxAngle() {
    return maxAngle;
}

int MotorValve::CurrentAngle() {
    return currentAngle;
}

int MotorValve::getCurrentAngle() {
    return currentAngle;
}

bool MotorValve::isOpening() {
    return opening;
}

bool MotorValve::isClosing() {
    return closing;
}

bool MotorValve::isOperating() {
    return operating;
}

bool MotorValve::isCalibrating() {
    return calibrating;
}

const char* MotorValve::getStatus() {
    static char status[30];
    uint8_t currentState = getCurrentState(openPin.address);

    if (calibrating) {
        snprintf(status, sizeof(status), "calibr...");
    } else if (operating) {
        unsigned long elapsedTime = millis() - operationStartTime;
        int angleDiff = abs(currentAngle - targetAngle);
        int operatingDuration = abs(angleDiff * timeToMaxAngle * 1000 / (maxAngle - startAngle));
        float progress = (float)elapsedTime / operatingDuration;
        if (progress > 1.0) progress = 1.0;

        int calculatedAngle = opening ? currentAngle - (angleDiff * progress) : currentAngle + (angleDiff * progress);
        if (opening && calculatedAngle < targetAngle) calculatedAngle = targetAngle;
        if (closing && calculatedAngle > targetAngle) calculatedAngle = targetAngle;
        snprintf(status, sizeof(status), "%d°", calculatedAngle);
    } else if (currentAngle == startAngle) {
        snprintf(status, sizeof(status), "OPEN");
    } else if (currentAngle == halfAngle) {
        snprintf(status, sizeof(status), "HALFOPEN");
    } else if (currentAngle == maxAngle) {
        snprintf(status, sizeof(status), "CLOSED");
    } else {
        snprintf(status, sizeof(status), "%d°", currentAngle);
    }
    return status;
}

uint8_t MotorValve::getCurrentState(uint8_t address) {
    return PCF8574Manager::getInstance().getState(address);
}