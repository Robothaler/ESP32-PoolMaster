#pragma once

#include <Arduino.h>
#include "Config.h"

// Central I2C states for PCF8574 devices 
struct I2CDeviceStates {
  uint8_t statePCF8574_I;   // State of PCF8574_I (pins: FILTRATION_PUMP, HEAT_PUMP, etc.)
  uint8_t statePCF8574_II;  // State of PCF8574_II (pins: ESD_TRE_OPEN, ESD_TRE_CLOSE, etc.) 
  uint8_t statePCF8574_III; // State of PCF8574_III (pins: BODEN_OPEN, BODEN_CLOSE, etc.)
};

extern I2CDeviceStates i2cStates;

// PCF8574 devices for polling (flexibly usable)
struct PCFDevice {
  uint8_t address;      // I2C address of PCF8574 device
  uint8_t* statePtr;    // Pointer to state in i2cStates
};

extern const PCFDevice pcfDevices[NUM_PCF_DEVICES];