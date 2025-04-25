#pragma once

#include <Arduino.h>
#include "Config.h"

// Struktur für Pin und Adresse
typedef struct {
  uint8_t pin;      // Pin-Nummer (0-7 für P0-P7)
  uint8_t address;  // I²C-Adresse des PCF8574
} PCF_Pin;

// Default-Wert für Pins ohne Zuordnung
const PCF_Pin NO_PIN = {255, 0xFF};
