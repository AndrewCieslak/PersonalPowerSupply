#pragma once
#include <stdint.h>

// sends voltage from 0-5v
void ad420_newVoltage(uint16_t voltage);
// clears DAC
void clearDAC(void);
// Sends a 16-bit code MSB-first over SPI and latches it into the DAC
uint16_t voltageToCode(float mv);