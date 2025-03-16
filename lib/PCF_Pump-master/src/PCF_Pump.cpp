#include "Arduino.h"
#include "PCF_Pump.h"
#include <Arduino_DebugUtils.h>   // Debug.print
#include "PCF8574.h"              // IO-Portexpander  https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/   VERSION: 2.3.4
#include "I2CConfig.h"

// External instances of PCF8574 objects defined in the main file
extern SemaphoreHandle_t mutex;
extern Arduino_DebugUtils Debug;

// Constructor
// PumpPin is the Arduino relay output pin number to be switched to start/stop the pump
// TankLevelPin is the Arduino digital input pin number connected to the tank level switch
// Interlockpin is the Arduino digital input number connected to an "interlock".
// If this input is LOW, pump is stopped and/or cannot start. This is used for instance to stop
// the Orp or pH pumps in case the filtration pump is not running
// IsRunningSensorPin is the pin which is checked to know whether the pump is running or not.
// It can be the same pin as "PumpPin" in case there is no sensor on the pump (pressure, current, etc) which is not as robust.
// This option is especially useful in the case where the filtration pump is not managed by the Arduino.
// FlowRate is the flow rate of the pump in Liters/Hour, typically 1.5 or 3.0 L/hour for peristaltic pumps for pools. This is used to compute how much of the tank we have emptied out
// TankVolume is used here to compute the percentage fill used
PCF_Pump::PCF_Pump(uint8_t startPin, uint8_t statePin, uint8_t levelPin, uint8_t interlockPin,
                   float flowRate, float tankVolume, float tankFill, uint8_t address,
                   uint8_t interlockAddress, bool activeLow)
    :   _startPin(startPin),
        _statePin(statePin),
        _levelPin(levelPin),
        _interlockPin(interlockPin),
        _address(address),
        _interlockAddress(interlockAddress),
        _activeLow(activeLow),
        _flowRate(flowRate),
        _tankVolume(tankVolume),
        _tankFill(tankFill),
        _lastStartTime(0),
        _upTime(0),
        _maxUpTime(0)
{
    if (_interlockAddress == 0xFF)
        _interlockAddress = _address; // Default: same address
}

unsigned long getDurationSafe(unsigned long start, unsigned long current) {
    // Calculate duration between start and current, handling millis() overflow (wraparound from ULONG_MAX to 0).
    if (current < start) {
        return (ULONG_MAX - start) + current + 1;
    }
    return current - start;
}

// Call this in the main loop, for every loop, as often as possible
void PCF_Pump::loop() {
    // Update uptime and tank fill if pump is running
    if (IsRunning() && _lastStartTime > 0) {
      _upTime = millis() - _lastStartTime;
  
      // Stop pump if max uptime is exceeded
      if (_maxUpTime > 0 && _upTime >= _maxUpTime) {
        Stop();
        Debug.print(DBG_WARNING, "[PCF_Pump] Max uptime reached for pin %d on address 0x%02X", _startPin, _address);
      }
  
      // Update virtual tank fill level (in percent) based on flow rate and tank volume
      if (_flowRate > 0 && _tankVolume > 0) {
        float consumed = _flowRate * (_upTime / 60000.0); // Liters consumed (flow rate in L/min * minutes)
        _tankFill -= (consumed / _tankVolume) * 100.0;    // Decrease tank fill percentage
        if (_tankFill < 0) _tankFill = 0;                 // Ensure tank fill doesn't go below 0%
        Debug.print(DBG_VERBOSE, "[PCF_Pump] Tank fill updated: %.2f%% for pin %d", _tankFill, _startPin);
      }
    }
  }

// Switch pump ON if over time was not reached, tank is not empty, and interlock is OK
bool PCF_Pump::Start() {
  // Start pump if interlock and tank level conditions are met
  if (lockI2C()) {
    if (!Interlock()) {
      Debug.print(DBG_WARNING, "[PCF_Pump] Interlock prevents start at pin %d", _startPin);
      unlockI2C();
      return false;
    }
    if (!TankLevel()) {
      Debug.print(DBG_WARNING, "[PCF_Pump] Tank level prevents start at pin %d", _startPin);
      unlockI2C();
      return false;
    }

    // Set start pin to active state (low if activeLow, high otherwise)
    uint8_t currentState = getPCFState(_address);
    uint8_t newState = _activeLow ? (currentState & ~(1 << _startPin)) : (currentState | (1 << _startPin));
    if (writePCFState(_address, newState)) {
      _lastStartTime = millis();
      Debug.print(DBG_INFO, "[PCF_Pump] Started pump at pin %d on address 0x%02X", _startPin, _address);
      unlockI2C();
      return true;
    }
    unlockI2C();
    return false;
  }
  Debug.print(DBG_WARNING, "[PCF_Pump] Failed to lock I2C for start at pin %d", _startPin);
  return false;
}

// Switch pump OFF
bool PCF_Pump::Stop() {
    // Stop pump and update uptime
    if (lockI2C()) {
      uint8_t currentState = getPCFState(_address);
      uint8_t newState = _activeLow ? (currentState | (1 << _startPin)) : (currentState & ~(1 << _startPin));
      if (writePCFState(_address, newState)) {
        _upTime = millis() - _lastStartTime;
        _lastStartTime = 0;
        Debug.print(DBG_INFO, "[PCF_Pump] Stopped pump at pin %d on address 0x%02X", _startPin, _address);
        unlockI2C();
        return true;
      }
      unlockI2C();
      return false;
    }
    Debug.print(DBG_WARNING, "[PCF_Pump] Failed to lock I2C for stop at pin %d", _startPin);
    return false;
  }

// Reset the tracking of running time
// This is typically called every day at midnight
void PCF_Pump::ResetUpTime() {
    StartTime = 0;
    StopTime = 0;
    UpTime = 0;
    CurrMaxUpTime = MaxUpTime;
}

// Set a maximum running time (in milliseconds) per day (in case ResetUpTime() is called once per day)
// Once reached, pump is stopped and "UpTimeError" error flag is raised
// Set "Max" to 0 to disable the limit
void PCF_Pump::SetMaxUpTime(unsigned long Max) {
    MaxUpTime = Max;
    CurrMaxUpTime = MaxUpTime;
}

// Clear "UpTimeError" error flag and allow the pump to run for an extra MaxUpTime
void PCF_Pump::ClearErrors() {
    if (UpTimeError) {
        CurrMaxUpTime += MaxUpTime;
        UpTimeError = false;
    }
}

// Tank level status (true = full, false = empty)
bool PCF_Pump::TankLevel() {
    // Return true if no tank is present (always "full")
    if (_levelPin == NO_TANK) return true;
  
    // Use virtual tank fill level if no physical level pin is defined (> 5% considered "not empty")
    if (_levelPin == NO_LEVEL) return (GetTankFill() > 5.0);
  
    // Check physical tank level pin state (assumed to be on the same PCF as startPin)
    uint8_t currentState = getPCFState(_address);
    bool levelState = (currentState & (1 << _levelPin)) == (_activeLow ? 0 : (1 << _levelPin));
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Tank level pin %d on address 0x%02X: %d", _levelPin, _address, levelState);
    return levelState;
  }

// Return the percentage used since the last reset of UpTime
double PCF_Pump::GetTankUsage() {
    float PercentageUsed = -1.0;
    if ((tankvolume != 0.0) && (flowrate != 0.0)) {
        double MinutesOfUpTime = (double)UpTime / 1000.0 / 60.0;
        double Consumption = flowrate / 60.0 * MinutesOfUpTime;
        PercentageUsed = Consumption / tankvolume * 100.0;
    }
    return (PercentageUsed);
}

// Return the remaining quantity in the tank in %. When resetting UpTime, SetTankFill must be called accordingly
double PCF_Pump::GetTankFill() {
    return (tankfill - this->PCF_Pump::GetTankUsage());
}

// Set Tank volume
// Typically call this function when changing the tank and set it to the full volume
void PCF_Pump::SetTankVolume(double Volume) {
    tankvolume = Volume;
}

// Set flow rate of the pump in Liters/hour
void PCF_Pump::SetFlowRate(double FlowRate) {
    flowrate = FlowRate;
}

// Set tank fill (percentage of tank volume)
void PCF_Pump::SetTankFill(double TankFill) {
    tankfill = TankFill;
}

// Interlock status
bool PCF_Pump::Interlock() {
    // Return true if no interlock is defined (always OK)
    if (_interlockPin == NO_INTERLOCK) return true;
  
    // Check interlock pin state on its specific PCF address
    uint8_t currentState = getPCFState(_interlockAddress);
    bool interlockState = (currentState & (1 << _interlockPin)) == (_activeLow ? 0 : (1 << _interlockPin));
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Interlock pin %d on address 0x%02X: %d", _interlockPin, _interlockAddress, interlockState);
    return interlockState;
  }

// Pump status
bool PCF_Pump::IsRunning() {
    // Check pump state on its specific PCF address
    uint8_t state = getPCFState(_address);
    bool running = (state & (1 << _statePin)) == (_activeLow ? 0 : (1 << _statePin));
    Debug.print(DBG_VERBOSE, "[PCF_Pump] IsRunning pin %d on address 0x%02X: %d", _statePin, _address, running);
    return running;
  }

  uint8_t PCF_Pump::getPCFState(uint8_t address) {
    // Retrieve current state from i2cStates based on the specified PCF address
    if (address == PCF8574_I_ADR) return i2cStates.statePCF8574_I;
    if (address == PCF8574_II_ADR) return i2cStates.statePCF8574_II;
    if (address == PCF8574_III_ADR) return i2cStates.statePCF8574_III;
    Debug.print(DBG_ERROR, "[PCF_Pump] Unknown PCF address 0x%02X", address);
    return 0; // Return 0 in case of an invalid address
  }
  
  bool PCF_Pump::writePCFState(uint8_t address, uint8_t state) {
    // Write new state to the specified PCF address with I2C mutex protection
    if (lockI2C()) {
      Wire.beginTransmission(address);
      Wire.write(state);
      bool success = (Wire.endTransmission() == 0);
      if (success) {
        // Update i2cStates to reflect the new state
        if (address == PCF8574_I_ADR) i2cStates.statePCF8574_I = state;
        else if (address == PCF8574_II_ADR) i2cStates.statePCF8574_II = state;
        else if (address == PCF8574_III_ADR) i2cStates.statePCF8574_III = state;
        Debug.print(DBG_VERBOSE, "[PCF_Pump] Wrote state 0x%02X to address 0x%02X", state, address);
      } else {
        Debug.print(DBG_ERROR, "[PCF_Pump] Failed to write state 0x%02X to address 0x%02X", state, address);
      }
      unlockI2C();
      return success;
    }
    Debug.print(DBG_WARNING, "[PCF_Pump] Failed to lock I2C for write to address 0x%02X", address);
    return false;
  }