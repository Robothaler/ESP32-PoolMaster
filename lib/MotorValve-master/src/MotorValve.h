/*
            MotorValve - A simple library for Arduino to handle motorized valves for home-pools. 
                 (c) Robothaler <robothaler@web.de> 2023
Features: 

* offers an easy way to open, close and halfopen a motorized valve or even just type in your required angle
* you can use this library for bellimo gearmotors oder motorized valves with no angle sensor, the library keeps track of the angle
* set start and max angle and the time from start angle to max angle for caluclations
* you can also see the status of the 

NB: all timings are in milliseconds
*/

#ifndef MOTOR_VALVE_H
#define MOTOR_VALVE_H

#define MOTOR_VALVE_VERSION "1.0.0"

// Constants used in some of the functions below
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define ON 0
#define OFF 1

#define NO_PCF 0
#define PCF8574_2 1
#define PCF8574_3 2
#define PCF8574_4 3

#include <Arduino.h>

class MotorValve {
public:
    MotorValve(uint8_t openPin, uint8_t closePin, int startAngle, int maxAngle, int timeToMaxAngle, int calibrationDirection, int pcftyp, char* name);

    //Funktion to set the correct pintyp
    void setSignal(int pin, int state);
    
    // Functions to open, close and half-open the valve
    void open();
    void close();
    void halfOpen();

    // Function to calibrate the valve
    void calibrate();

    // Function to update the status of the valve
    void loop();

    void setTargetAngle(int target);

    int getCurrentAngle();

    // Functions to check the status of the valve
    boolean isOpen();
    boolean isClosed();
    boolean isHalfOpen();
    boolean CurrentAngle();
    boolean isOpening();
    boolean isClosing();
    boolean isOperating();
    boolean isCalibrating();
    std::string getStatus();

private:
    // Pin numbers for opening and closing the valve
    uint8_t openPin;
    uint8_t closePin;

    // Starting and maximum angle of the valve
    int startAngle;
    int maxAngle;

    // Time required to reach the maximum angle
    int timeToMaxAngle;

    // Direction of calibration
    int calibrationDirection;
    
    // You have to define on which PCF8574 the Valve is Configured
    // Make sure that both relais (open and close) are on the same PCF8574
    int pcftyp;

    // Instancename for easier debuging
    char* Name;
    char* instanceName;

    // Current angle and target angle of the valve
    int currentAngle;
    int targetAngle;

    // Variables to keep track of calibration and operation status
    boolean calibrating;
    boolean operating;
    boolean opening;
    boolean closing;

    // Timestamps for calibration and operation start time
    unsigned long calibrationStartTime;
    unsigned long operationStartTime;

    // Duration of the operation
    int operatingDuration;
};

#endif // MOTOR_VALVE_H
