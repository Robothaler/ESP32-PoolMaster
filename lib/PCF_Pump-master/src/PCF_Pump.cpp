#include "Arduino.h"
#include "PCF_Pump.h"
#include <Arduino_DebugUtils.h>   // Debug.print
#include "PCF8574.h"              // IO-Portexpander  https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/   VERSION: 2.3.4
#include "PCF8574Manager.h"  
#include "I2CConfig.h"

// External instances of PCF8574 objects defined in the main file
extern SemaphoreHandle_t mutex;
extern SemaphoreHandle_t i2cStatesMutex;
extern SemaphoreHandle_t i2cOutputMutex;
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
PCF_Pump::PCF_Pump(PCF_Pin startPin, PCF_Pin statePin, PCF_Pin levelPin, PCF_Pin interlockPin,
                   double flowRate, double tankVolume, double tankFill, bool activeLow) 
    : _startPin(startPin),
      _statePin(statePin),
      _levelPin(levelPin),
      _interlockPin(interlockPin),
      _activeLow(activeLow),
      _flowRate(flowRate),
      _tankVolume(tankVolume),
      _tankFill(tankFill),
      UpTime(0),
      MaxUpTime(DefaultMaxUpTime),
      CurrMaxUpTime(DefaultMaxUpTime),
      UpTimeError(false),
      StartTime(0),
      LastStartTime(0),
      StopTime(0)
{
  // Wenn kein Interlock-Pin angegeben ist, wird er ignoriert
  if (_interlockPin.address == 0xFF && _interlockPin.pin == 255)
  {
    _interlockPin = NO_PIN;
  }
}

// Main loop
void PCF_Pump::loop() {
  if (IsRunning()) {
      if (StartTime > 0) {
          UpTime += getDurationSafe(StartTime, millis());
          StartTime = millis();
      }

      if ((CurrMaxUpTime > 0) && (UpTime >= CurrMaxUpTime)) {
          Stop();
          UpTimeError = true;
          Debug.print(DBG_WARNING, "[PCF_Pump] Max uptime reached for pin %d on address 0x%02X", _startPin.pin, _startPin.address);
      }

      if (!TankLevel() || (_interlockPin.pin != NO_INTERLOCK && !Interlock())) {
          Stop();
      }
  }
}

// Switch pump ON (LOW = einschalten)
bool PCF_Pump::Start() {
  if (!IsRunning() && !UpTimeError && TankLevel() &&
      (_interlockPin.pin == NO_INTERLOCK || Interlock())) {
      Debug.print(DBG_VERBOSE, "[PCF_Pump] Start conditions met for pin %d", _startPin.pin);
      if (queueUpdate(_startPin.address, _startPin.pin, true)) {
          StartTime = LastStartTime = millis();
          Debug.print(DBG_VERBOSE, "[PCF_Pump] Start successful for pin %d", _startPin.pin);
          return true;
      }
      Debug.print(DBG_VERBOSE, "[PCF_Pump] queueUpdate failed for pin %d", _startPin.pin);
      return false;
  }
  Debug.print(DBG_VERBOSE, "[PCF_Pump] Start conditions not met for pin %d", _startPin.pin);
  return false;
}

// Switch pump OFF (HIGH = ausschalten)
bool PCF_Pump::Stop() {
  if (IsRunning()) {
      if (queueUpdate(_startPin.address, _startPin.pin, false)) {
          if (StartTime > 0) {
              UpTime += getDurationSafe(StartTime, millis());
              StartTime = 0;
          }
          Debug.print(DBG_VERBOSE, "[PCF_Pump] Stop successful for pin %d", _startPin.pin);
          return true;
      }
      Debug.print(DBG_VERBOSE, "[PCF_Pump] queueUpdate failed for pin %d", _startPin.pin);
      return false;
  }
  Debug.print(DBG_VERBOSE, "[PCF_Pump] Pump not running, no action for pin %d", _startPin.pin);
  return false;
}

// Reset the tracking of running time
void PCF_Pump::ResetUpTime() {
  StartTime = 0;
  StopTime = 0;
  UpTime = 0;
  CurrMaxUpTime = MaxUpTime;
}

// Set maximum running time
void PCF_Pump::SetMaxUpTime(unsigned long Max) {
  MaxUpTime = Max;
  CurrMaxUpTime = Max;
}

// Clear errors
void PCF_Pump::ClearErrors() {
  if (UpTimeError) {
      CurrMaxUpTime += MaxUpTime;
      UpTimeError = false;
  }
}

// Tank level status
bool PCF_Pump::TankLevel() {
  if (_levelPin.pin == NO_TANK) return true;
  if (_levelPin.pin == NO_LEVEL) return (GetTankFill() > 5.0);

  uint8_t state = PCF8574Manager::getInstance().getState(_levelPin.address);
  bool level = (state & (1 << _levelPin.pin)) != 0; // HIGH = voll
  Debug.print(DBG_VERBOSE, "[PCF_Pump] Tank level pin %d on 0x%02X: %d", _levelPin.pin, _levelPin.address, level);
  return level;
}

// Interlock status
bool PCF_Pump::Interlock() {
  if (_interlockPin.pin == NO_INTERLOCK) return true;
  uint8_t state = PCF8574Manager::getInstance().getState(_interlockPin.address);
  bool interlock = (state & (1 << _interlockPin.pin)) == 0; // LOW = Filterpumpe an
  Debug.print(DBG_VERBOSE, "[PCF_Pump] Interlock pin %d on 0x%02X: %d (state: 0x%02X)",
              _interlockPin.pin, _interlockPin.address, interlock, state);
  if (!interlock) {
      Debug.print(DBG_WARNING, "[PCF_Pump] Interlock failed for pin %d, filter pump not running", _startPin.pin);
  }
  return interlock;
}

// Pump status
bool PCF_Pump::IsRunning() {
  PCF8574Manager& manager = PCF8574Manager::getInstance();
  uint8_t state = manager.getState(_statePin.address);
  bool running = _activeLow ? ((state & (1 << _statePin.pin)) == 0) : ((state & (1 << _statePin.pin)) != 0);
  Debug.print(DBG_VERBOSE, "[PCF_Pump] Pin %d on 0x%02X running: %d (state: 0x%02X)",
              _statePin.pin, _statePin.address, running, state);
  return running;
}

// Tank usage and fill calculations
double PCF_Pump::GetTankUsage() {
  float PercentageUsed = -1.0;
  if (_tankVolume != 0.0 && _flowRate != 0.0) {
      double minutesOfUpTime = (double)UpTime / 1000.0 / 60.0;
      double consumption = _flowRate / 60.0 * minutesOfUpTime;
      PercentageUsed = consumption / _tankVolume * 100.0;
  }
  return PercentageUsed;
}

double PCF_Pump::GetTankFill() {
  return (_tankFill - GetTankUsage());
}

void PCF_Pump::SetTankVolume(double Volume) {
  _tankVolume = Volume;
}

void PCF_Pump::SetFlowRate(double FlowRate) {
  _flowRate = FlowRate;
}

void PCF_Pump::SetTankFill(double TankFill) {
  _tankFill = TankFill;
}

bool PCF_Pump::queueUpdate(uint8_t address, uint8_t pin, bool state) {
  PCF8574Manager& manager = PCF8574Manager::getInstance();
  manager.queuePinUpdate(address, pin, state);
  return true;
}