#include "Arduino.h"
#include "PCF_Pump.h"
#include <Arduino_DebugUtils.h>   // Debug.print
#include "PCF8574.h"              // IO-Portexpander
#include "PCF8574Manager.h"  
#include "I2CConfig.h"

// External instances of PCF8574 objects defined in the main file
extern SemaphoreHandle_t mutex;
extern SemaphoreHandle_t i2cStatesMutex;
extern SemaphoreHandle_t i2cOutputMutex;
extern Arduino_DebugUtils Debug;

// Constructor
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
      StopTime(0),
      pumpState(false)
{
    if (_interlockPin.address == 0xFF && _interlockPin.pin == 255) {
        _interlockPin = NO_PIN;
    }
    Debug.print(DBG_INFO, "[PCF_Pump] %s initialized with startPin %d on 0x%02X, statePin %d on 0x%02X, activeLow: %d",
                _startPin.address == 0xFF ? "Unnamed" : "Pump", _startPin.pin, _startPin.address, _statePin.pin, _statePin.address, _activeLow);
}

// Main loop
void PCF_Pump::loop() {
    if (IsRunning() && pumpState) { // Only update UpTime if pump is running and pumpState is true
        if (StartTime > 0) {
            UpTime += getDurationSafe(StartTime, millis());
            StartTime = millis();
            Debug.print(DBG_VERBOSE, "[PCF_Pump] UpTime updated: %lu ms, StartTime: %lu ms, pumpState: %d, IsRunning: %d",
                        UpTime, StartTime, pumpState, IsRunning());
        }
        if ((CurrMaxUpTime > 0) && (UpTime >= CurrMaxUpTime)) {
            Stop();
            UpTimeError = true;
            Debug.print(DBG_WARNING, "[PCF_Pump] Max uptime reached for pin %d on address 0x%02X", _startPin.pin, _startPin.address);
        }
        if (!TankLevel() || (_interlockPin.pin != NO_INTERLOCK && !Interlock())) {
            Stop();
        }
    } else if (IsRunning() && !pumpState) {
        Debug.print(DBG_WARNING, "[PCF_Pump] Mismatch detected: IsRunning=true, pumpState=false for pin %d on 0x%02X", 
                    _startPin.pin, _startPin.address);
        Stop(); // Force stop to synchronize state
    }
    synchronizeWithShadow();
}

// Switch pump ON (LOW = einschalten)
bool PCF_Pump::Start() {
    if (!UpTimeError && TankLevel() && (_interlockPin.pin == NO_INTERLOCK || Interlock())) {
        Debug.print(DBG_VERBOSE, "[PCF_Pump] Start conditions met for pin %d on 0x%02X", _startPin.pin, _startPin.address);
        pumpState = true;
        if (queueUpdate(_startPin.address, _startPin.pin, true)) {
            StartTime = LastStartTime = millis();
            Debug.print(DBG_VERBOSE, "[PCF_Pump] Start successful for pin %d on 0x%02X, pumpState=%d", _startPin.pin, _startPin.address, pumpState);
            synchronizeWithShadow();
            return true;
        }
        pumpState = false; // Rollback bei Fehler
        Debug.print(DBG_WARNING, "[PCF_Pump] queueUpdate failed for pin %d on 0x%02X", _startPin.pin, _startPin.address);
        return false;
    }
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Start conditions not met for pin %d on 0x%02X", _startPin.pin, _startPin.address);
    return false;
}

// Switch pump OFF (HIGH = ausschalten)
bool PCF_Pump::Stop() {
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Stop called for pin %d on 0x%02X, running=%d", _startPin.pin, _startPin.address, IsRunning());

    // Accumulate final runtime slice BEFORE updating shadow/pumpState.
    // IsRunning() would already return false after queueUpdate(), so we must
    // check pumpState (the logical desired state) here, not IsRunning().
    if (StartTime > 0 && pumpState) {
        UpTime += getDurationSafe(StartTime, millis());
        Debug.print(DBG_VERBOSE, "[PCF_Pump] UpTime updated on stop: %lu ms", UpTime);
    }

    StartTime = 0;
    pumpState = false;

    if (queueUpdate(_startPin.address, _startPin.pin, false)) {
        Debug.print(DBG_VERBOSE, "[PCF_Pump] Stop successful for pin %d on 0x%02X", _startPin.pin, _startPin.address);
        synchronizeWithShadow();
        return true;
    }

    // Rollback: hardware write queuing failed — pump is still running.
    // Reset StartTime to now so loop() continues accumulating from this point.
    pumpState = true;
    StartTime = millis();
    Debug.print(DBG_WARNING, "[PCF_Pump] queueUpdate failed for pin %d on 0x%02X", _startPin.pin, _startPin.address);
    return false;
}

// Synchronize pumpState with shadowState
void PCF_Pump::synchronizeWithShadow() {
    if (_startPin.address == 0xFF || _startPin.pin > 7) return;
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    uint8_t shadowState = manager.getState(_startPin.address);
    bool shadowPinState = _activeLow ? ((shadowState & (1 << _startPin.pin)) == 0) : ((shadowState & (1 << _startPin.pin)) != 0);
    if (shadowPinState != pumpState) {
        Debug.print(DBG_WARNING, "[PCF_Pump] Shadow state mismatch for pin %d on 0x%02X: pumpState=%d, shadowPinState=%d",
                    _startPin.pin, _startPin.address, pumpState, shadowPinState);
        queueUpdate(_startPin.address, _startPin.pin, pumpState);
        Debug.print(DBG_INFO, "[PCF_Pump] Synchronized pin %d on 0x%02X to pumpState=%d",
                    _startPin.pin, _startPin.address, pumpState);
    } else {
        Debug.print(DBG_VERBOSE, "[PCF_Pump] Shadow state matches for pin %d on 0x%02X: pumpState=%d",
                    _startPin.pin, _startPin.address, pumpState);
    }
}

// Rest der Methoden unverändert
void PCF_Pump::ResetUpTime() {
    StartTime = 0;
    StopTime = 0;
    UpTime = 0;
    CurrMaxUpTime = MaxUpTime;
    synchronizeWithShadow();
}

void PCF_Pump::SetMaxUpTime(unsigned long Max) {
    MaxUpTime = Max;
    CurrMaxUpTime = Max;
}

void PCF_Pump::ClearErrors() {
    if (UpTimeError) {
        CurrMaxUpTime += MaxUpTime;
        UpTimeError = false;
        synchronizeWithShadow();
    }
}

bool PCF_Pump::TankLevel() {
    if (_levelPin.pin == NO_TANK) return true;
    if (_levelPin.pin == NO_LEVEL) return (GetTankFill() > 5.0);
    uint8_t state = PCF8574Manager::getInstance().getState(_levelPin.address);
    bool level = (state & (1 << _levelPin.pin)) != 0;
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Tank level pin %d on 0x%02X: %d", _levelPin.pin, _levelPin.address, level);
    return level;
}

bool PCF_Pump::Interlock() {
    if (_interlockPin.pin == NO_INTERLOCK) return true;
    uint8_t state = PCF8574Manager::getInstance().getState(_interlockPin.address);
    bool interlock = (state & (1 << _interlockPin.pin)) == 0;
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Interlock pin %d on 0x%02X: %d (state: 0x%02X)",
                _interlockPin.pin, _interlockPin.address, interlock, state);
    if (!interlock) {
        Debug.print(DBG_WARNING, "[PCF_Pump] Interlock failed for pin %d, filter pump not running", _startPin.pin);
    }
    return interlock;
}

bool PCF_Pump::IsRunning() {
    PCF8574Manager& manager = PCF8574Manager::getInstance();
    uint8_t state = manager.getState(_statePin.address);
    bool running = _activeLow ? ((state & (1 << _statePin.pin)) == 0) : ((state & (1 << _statePin.pin)) != 0);
    Debug.print(DBG_VERBOSE, "[PCF_Pump] Pin %d on 0x%02X running: %d (state: 0x%02X)",
                _statePin.pin, _statePin.address, running, state);
    return running;
}

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