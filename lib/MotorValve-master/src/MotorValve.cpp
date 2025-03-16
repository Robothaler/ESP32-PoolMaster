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
#include "PCF8574.h"                // IO-Portexpander  https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/   VERSION: 2.3.4
#include <Arduino_DebugUtils.h>     // Debug.print
#include <Arduino.h>
#include "I2CConfig.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern Arduino_DebugUtils Debug;

//Constructor
MotorValve::MotorValve(uint8_t OpenPin, uint8_t ClosePin, int StartAngle, int MaxAngle,
    int TimeToMaxAngle, int CalibrationDirection, uint8_t PcfAddress, const char* Name,
    bool useExternalMutex) {
    this->openPin = OpenPin;
    this->closePin = ClosePin;
    this->startAngle = StartAngle;
    this->maxAngle = MaxAngle;
    this->halfAngle = startAngle + (maxAngle - startAngle) / 2;
    this->timeToMaxAngle = TimeToMaxAngle;
    this->calibrationDirection = CalibrationDirection;
    this->pcfAddress = PcfAddress; // Neu: Adresse des PCF8574
    this->instanceName = Name;
    this->currentAngle = StartAngle;
    this->targetAngle = StartAngle;
}

void MotorValve::setExternalMutexControl(bool useExternal) {
    externalMutexControl = useExternal;
}

bool MotorValve::lockI2C() {
    if (externalMutexControl) return true;
    return ::lockI2C();
}
void MotorValve::unlockI2C() {
    if (!externalMutexControl) ::unlockI2C();
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
    int angleDiff = abs(currentAngle - targetAngle);
    int operatingDuration = abs(angleDiff * timeToMaxAngle * 1000 / (maxAngle - startAngle));
    uint8_t currentState = getPCFState();

    if (!operating && !calibrating) {
        if (currentAngle != targetAngle) {
            if (currentAngle > targetAngle) {
                setSignal(openPin, ON);
                opening = true;
            } else {
                setSignal(closePin, ON);
                closing = true;
            }
            operating = true;
            operationStartTime = millis();
        }
    }

    if (calibrating) {
        if ((millis() - calibrationStartTime) >= (timeToMaxAngle + 2) * 1000) {
            setSignal(openPin, OFF);
            setSignal(closePin, OFF);
            calibrating = false;
            currentAngle = (calibrationDirection == CLOCKWISE) ? startAngle : maxAngle;
            Debug.print(DBG_DEBUG, "[MotorValve] %s calibration stopped at angle: %d", instanceName, currentAngle);
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
            setSignal(openPin, OFF);
            setSignal(closePin, OFF);
            opening = false;
            closing = false;
            operating = false;
            currentAngle = targetAngle;
            Debug.print(DBG_DEBUG, "[MotorValve] %s reached angle: %d", instanceName, currentAngle);
        } else {
            Debug.print(DBG_DEBUG, "[MotorValve] %s current angle: %d", instanceName, calculatedAngle);
        }
    }
}

void MotorValve::calibrate() {
    if (calibrationDirection == CLOCKWISE) {
        setSignal(openPin, ON);
        Debug.print(DBG_DEBUG, "[MotorValve] %s is calibrating clockwise.", instanceName);
    } else {
        setSignal(closePin, ON);
        Debug.print(DBG_DEBUG, "[MotorValve] %s is calibrating counter-clockwise.", instanceName);
    }
    calibrationStartTime = millis();
    calibrating = true;
}

void MotorValve::setSignal(uint8_t pin, uint8_t state) {
    if (!lockI2C()) {
        Debug.print(DBG_WARNING, "[MotorValve] %s Skipping setSignal due to I2C lock failure", instanceName);
        return;
    }
    uint8_t currentState = getPCFState();
    if (state == ON) currentState |= (1 << pin);
    else currentState &= ~(1 << pin);
    writePCFState(currentState);
    unlockI2C();
}

// returns true only if it is all the way open
boolean MotorValve::isOpen() 
{
  if (currentAngle == startAngle)
    return true;
  else
    return false;
}

// returns true only if it is all the way closed
boolean MotorValve::isClosed() 
{
  if (currentAngle == maxAngle)
    return true;
  else
    return false;
}

// returns true only if it is half open
boolean MotorValve::isHalfOpen() 
{
  if (currentAngle == startAngle + (maxAngle - startAngle) / 2)
    return true;
  else
    return false;
}

// returns the start angle of the valve
bool MotorValve::StartAngle() {
  return startAngle;
}

// returns the maximum angle of the valve
boolean MotorValve::HalfAngle()
{
  return halfAngle;
}

// returns the maximum angle of the valve
boolean MotorValve::MaxAngle() 
{
  return maxAngle;
}

bool MotorValve::CurrentAngle() {
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
    static char status[20];

    if (calibrating) {
        strcpy(status, u8"calibr...");
    } else if (opening || closing) {
        unsigned long elapsedTime = millis() - operationStartTime;
        int angleDiff = abs(currentAngle - targetAngle);
        int operatingDuration = abs(angleDiff * timeToMaxAngle * 1000 / (maxAngle - startAngle));
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
        snprintf(status, sizeof(status), u8"%d°", calculatedAngle);
    } else if (currentAngle == startAngle) {
        strcpy(status, u8"OPEN");
    } else if (currentAngle == (startAngle + (maxAngle - startAngle) / 2)) {
        strcpy(status, u8"HALFOPEN");
    } else if (currentAngle == maxAngle) {
        strcpy(status, u8"CLOSED");
    } else {
        snprintf(status, sizeof(status), u8"%d°", currentAngle);
    }

    return status;
}

uint8_t MotorValve::getPCFState() {
    if (pcfAddress == PCF8574_I_ADR) return i2cStates.statePCF8574_I;
    if (pcfAddress == PCF8574_II_ADR) return i2cStates.statePCF8574_II;
    if (pcfAddress == PCF8574_III_ADR) return i2cStates.statePCF8574_III;
    return 0; // Fehlerfall
}

bool MotorValve::writePCFState(uint8_t state) {
    Wire.beginTransmission(pcfAddress);
    Wire.write(state);
    return Wire.endTransmission() == 0;
}