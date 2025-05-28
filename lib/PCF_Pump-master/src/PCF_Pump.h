/*
    PCF_Pump - a simple library to handle home-pool filtration and peristaltic pumps
                 (c) Loic74 <loic74650@gmail.com> 2017-2020
Features: 
- keeps track of running time
- keeps track of Tank Levels
- set max running time limit
NB: all timings are in milliseconds
*/

#ifndef PCF_PUMP_h
#define PCF_PUMP_h
#define PCF_PUMP_VERSION "1.0.3" // Version erhöht wegen neuer Funktionalität

#include <Arduino.h>
#include "I2CConfig.h"

// Default-Wert für Pins ohne Zuordnung
extern const PCF_Pin NO_PIN;

// Constants used in some of the functions below
#define PUMP_ON  0
#define PUMP_OFF 1
#define TANK_FULL  1
#define TANK_EMPTY 0
#define INTERLOCK_OK  0
#define INTERLOCK_NOK 1
#define NO_LEVEL 170           // Pump with tank but without level switch
#define NO_TANK 255            // Pump without tank
#define NO_INTERLOCK 255  
#define DefaultMaxUpTime 30*60*1000 // default value is 30mins

class PCF_Pump {
public:
    // Constructor
    PCF_Pump(PCF_Pin startPin, PCF_Pin statePin, PCF_Pin levelPin = NO_PIN, PCF_Pin interlockPin = NO_PIN,
             double flowRate = 0.0, double tankVolume = 0.0, double tankFill = 100.0, bool activeLow = true);

    void loop();
    bool Start();
    bool Stop();
    bool IsRunning();
    bool TankLevel();
    double GetTankUsage();
    void SetTankVolume(double Volume);
    void SetFlowRate(double FlowRate);
    bool Interlock();
    void SetMaxUpTime(unsigned long Max);
    void ResetUpTime();
    void SetTankFill(double TankFill);
    double GetTankFill();
    void ClearErrors();
    bool queueUpdate(uint8_t address, uint8_t pin, bool state);
    void synchronizeWithShadow(); // Neue Methode für Synchronisierung

    // Public member variables
    unsigned long UpTime;
    unsigned long MaxUpTime;
    unsigned long CurrMaxUpTime;
    bool UpTimeError;
    unsigned long StartTime;
    unsigned long LastStartTime;
    unsigned long StopTime;
    double _flowRate, _tankVolume, _tankFill;

private:
    PCF_Pin _startPin;      // Start/Stop-Steuerpin
    PCF_Pin _statePin;      // Statuspin (z. B. Rückmeldung)
    PCF_Pin _levelPin;      // Tankfüllstandspin
    PCF_Pin _interlockPin;  // Interlock-Pin
    bool _activeLow;        // Logik: aktiv niedrig oder hoch
    bool pumpState;         // Neues Register für gewünschten Pin-Zustand

    unsigned long getDurationSafe(unsigned long start, unsigned long current) {
        if (current < start) {
            return (ULONG_MAX - start) + current + 1;
        }
        return current - start;
    }

    uint8_t getPCFState(uint8_t address);
    bool writePCFState(uint8_t address, uint8_t state);
};

#endif // PCF_PUMP_H