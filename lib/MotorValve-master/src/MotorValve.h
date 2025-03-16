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

NB: all timings are in milliseconds
*/

#ifndef MOTOR_VALVE_H
#define MOTOR_VALVE_H

#define MOTOR_VALVE_VERSION "1.0.2"

// Constants used in some of the functions below
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define ON 0
#define OFF 1

#define NO_PCF 0
#define PCF8574_I 1
#define PCF8574_II 2
#define PCF8574_III 3

#include <Arduino.h>

extern bool lockI2C();
extern void unlockI2C();

class MotorValve {
public:
    // Constructor
    MotorValve(uint8_t OpenPin, uint8_t ClosePin, int StartAngle, int MaxAngle,
        int TimeToMaxAngle, int CalibrationDirection, uint8_t PcfAddress,
        const char* Name, bool useExternalMutex = false);

    // Function to set the correct pin type (standard or PCF8574)
    void setSignal(uint8_t pin, uint8_t state);

    // Function to set the correct MutexControl
    void setExternalMutexControl(bool useExternal);
    
    // Functions to open, close, and half-open the valve
    void open();
    void close();
    void halfOpen();

    // Function to calibrate the valve
    void calibrate();

    // Function to update the status of the valve
    void loop();

    // Function to set a specific target angle
    void setTargetAngle(int target);

    // Function to get the current angle of the valve
    int getCurrentAngle();

    // Functions to check the status of the valve
    bool isOpen();
    bool isClosed();
    bool isHalfOpen();
    bool StartAngle();
    bool HalfAngle();
    bool MaxAngle();
    bool CurrentAngle();
    bool isOpening();
    bool isClosing();
    bool isOperating();
    bool isCalibrating();

    // Function to get the current status as a C-string (char array)
    const char* getStatus();

private:

    bool externalMutexControl = false;
    bool lockI2C();
    void unlockI2C();
    uint8_t getPCFState();
    bool writePCFState(uint8_t state);uint8_t pcftyp;

    // Pin numbers for opening and closing the valve
    uint8_t openPin;
    uint8_t closePin;

    // Starting and maximum angle of the valve
    int startAngle;
    int halfAngle;
    int maxAngle;

    // Time required to reach the maximum angle (in seconds)
    int timeToMaxAngle;

    // Direction of calibration (CLOCKWISE or COUNTER_CLOCKWISE)
    int calibrationDirection;
    
    // Type of PCF8574 used (NO_PCF, PCF8574_I, PCF8574_II, PCF8574_III)
    uint8_t pcfAddress;
    

    // Instance name for easier debugging
    const char* Name;
    const char* instanceName;

    // Current angle and target angle of the valve
    int currentAngle;
    int targetAngle;

    // Variables to keep track of calibration and operation status
    bool calibrating;
    bool operating;
    bool opening;
    bool closing;

    // Timestamps for calibration and operation start time
    unsigned long calibrationStartTime;
    unsigned long operationStartTime;

    // Duration of the operation
    int operatingDuration;
};

#endif // MOTOR_VALVE_H