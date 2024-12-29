#include "MotorValve.h"
#include "PoolMaster.h"
#include "Arduino_DebugUtils.h"     // Debug.print

//Constructor
//OpenPin is the Arduino relay output pin number to be switched to open the motor valve
//ClosePin is the Arduino relay output pin number to be switched to close the motor valve
//StartAngle is the the angle of the valve where the calculation starts (usally use "0")
//MaxAngle is the the maximum angle of the valve ("90" -> 90° for example)
//TimeToMaxAngle is the time in seconds from StartAngle to reach MaxAngle ("90" -> 90s for example)
//CalibrationDirection can be set to CLOCKWISE or COUNTER_CLOCKWISE, some valves need to have calibration in CCW Direction
//"open", "close" and "halfopen" comands set the valve to the corresponding valve position
//"TargetAngle" is for setting a specific position
MotorValve::MotorValve(uint8_t OpenPin, uint8_t ClosePin, int StartAngle, int MaxAngle, int TimeToMaxAngle, int CalibrationDirection, int PcfTyp, char* Name) {
    this->openPin = OpenPin;
    this->closePin = ClosePin;
    this->startAngle = StartAngle;
    this->maxAngle = MaxAngle;
    this->halfAngle = startAngle + (maxAngle - startAngle) / 2;
    this->timeToMaxAngle = TimeToMaxAngle; //in seconds
    this->calibrationDirection = CalibrationDirection; //CLOCKWISE or COUNTER_CLOCKWISE
    this->pcftyp = PcfTyp; //NO_PCF = 0 / PCF8574 = 1 / PCF8574_3 = 2 / PCF8574_4 = 3
    this->instanceName = Name;

}

void MotorValve::open() {
    if (currentAngle != startAngle) {
        targetAngle = startAngle;
        Debug.print(DBG_DEBUG,"[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG,"[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::close() {
    if (currentAngle != maxAngle) {
        targetAngle = maxAngle;
        Debug.print(DBG_DEBUG,"[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG,"[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::halfOpen() {
    int halfAngle = startAngle + (maxAngle - startAngle) / 2;
    if (currentAngle != halfAngle) {
        targetAngle = halfAngle;
        Debug.print(DBG_DEBUG,"[MotorValve] %s Target Angle set to HalfOpen (%d).", instanceName, targetAngle);
        Debug.print(DBG_DEBUG,"[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
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
        Debug.print(DBG_DEBUG,"[MotorValve] %s TargetAngle set to: %d", instanceName, targetAngle);
        Debug.print(DBG_DEBUG,"[MotorValve] %s CurrentAngle is: %d", instanceName, currentAngle);
    }
}

void MotorValve::loop() 
{
    int angleDiff = abs(currentAngle - targetAngle);
    int operatingDuration = abs(angleDiff * timeToMaxAngle * 1000 / (maxAngle - startAngle));

    if (!operating && !calibrating) {
        if (currentAngle != targetAngle) {
            if (currentAngle > targetAngle) {
                setSignal(openPin, ON);
                opening = true;
                Debug.print(DBG_DEBUG,"[MotorValve] %s is operating to open.", instanceName);
            } else {
                setSignal(closePin, ON);
                closing = true;
                Debug.print(DBG_DEBUG,"[MotorValve] %s is operating to close.", instanceName);
            }

            operating = true;
            operationStartTime = millis();
        }
    }

    if (calibrating) {
        if ((millis() - calibrationStartTime) >= (timeToMaxAngle + 2) * 1000) {
            setSignal(openPin, OFF);
            setSignal(closePin, OFF);
            Debug.print(DBG_DEBUG,"[MotorValve] %s calibration finished.", instanceName);
            calibrating = false;
            /*if(calibrationDirection == CLOCKWISE) {
                currentAngle = maxAngle;
            } else {
                currentAngle = startAngle;
            }*/
        }
    }

    if (operating) {
        if ((millis() - operationStartTime) >= operatingDuration) {
            setSignal(openPin, OFF);
            setSignal(closePin, OFF);
            Debug.print(DBG_DEBUG,"[MotorValve] %s operation stopped.", instanceName);
            
            opening = false;
            closing = false;
            operating = false;
            currentAngle = targetAngle;

             if (currentAngle == targetAngle) {
                setSignal(openPin, OFF);
                setSignal(closePin, OFF);
                Debug.print(DBG_DEBUG,"[MotorValve] %s is turned off.", instanceName);
            }
        }
    }
}

void MotorValve::calibrate() {
    int calibrationTime = timeToMaxAngle;

    if (calibrationDirection == CLOCKWISE) {
        setSignal(openPin, ON);
        calibrationStartTime = millis();
        calibrating = true;
        Debug.print(DBG_DEBUG,"[MotorValve] %s is calibrating clockwise.", instanceName);
    } else {
        setSignal(closePin, ON);
        calibrationStartTime = millis();
        calibrating = true;
        Debug.print(DBG_DEBUG,"[MotorValve] %s is calibrating counter-clockwise.", instanceName);
    }
}

void MotorValve::setSignal(int pin, int state) {
    if (pcftyp == NO_PCF) {
        digitalWrite(pin, state);
    } else if (pcftyp == PCF8574_I) {
        pcf8574_I.digitalWrite(pin, state);
    } else if (pcftyp == PCF8574_II) {
        pcf8574_II.digitalWrite(pin, state);
    } else if (pcftyp == PCF8574_III) {
        pcf8574_III.digitalWrite(pin, state);
    }
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
boolean MotorValve::StartAngle() 
{
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

// returns the current angle of the valve
boolean MotorValve::CurrentAngle() 
{
  return currentAngle;
}

// returns the current angle of the valve
int MotorValve::getCurrentAngle() 
{
  return currentAngle;
}


// returns true only if it is actually opening
boolean MotorValve::isOpening() 
{
    return opening;
}

// returns true only if it is actually closing
boolean MotorValve::isClosing() 
{
    return closing;
}

// returns true only if it is actually operating
boolean MotorValve::isOperating() 
{
    return operating;
}

// returns true only if it is actually calibrating
boolean MotorValve::isCalibrating() 
{
    return calibrating;
}

std::string MotorValve::getStatus() {
    if (calibrating) {
        return u8"calibr...";
    }

    if (opening) {
        return u8"öffne";
    }

    if (closing) {
        return u8"schließe";
    }

    if (currentAngle == startAngle) {
        return u8"AUF";
    }

    if (currentAngle == (startAngle + (maxAngle - startAngle) / 2)) {
        return u8"HALB";
    }

    if (currentAngle == maxAngle) {
        return u8"ZU";
    }

    return std::to_string(currentAngle) + u8"°  ";
}
