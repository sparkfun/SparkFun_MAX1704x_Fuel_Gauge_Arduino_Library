/******************************************************************************
SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h
By: Paul Clark
October 23rd 2020

Based extensively on:
SparkFunMAX17043.cpp
SparkFun MAX17043 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 22, 2015
https://github.com/sparkfun/SparkFun_MAX17043_Particle_Library

This file implements all functions of the MAX17043 class. Functions here range
from higher level stuff, like reading/writing MAX17043 registers to low-level,
hardware reads and writes.

This code is released under the MIT license.

Distributed as-is; no warranty is given.
******************************************************************************/
#include "SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h"

SFE_MAX1704X::SFE_MAX1704X()
{
  // Constructor
}

boolean SFE_MAX1704X::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  if (isConnected() == false)
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("begin: isConnected returned false"));
    }
    return (false);
  }

  return (true);
}

//Returns true if device answers on _deviceAddress
boolean SFE_MAX1704X::isConnected(void)
{
  _i2cPort->beginTransmission((uint8_t)MAX17043_ADDRESS);
  return (_i2cPort->endTransmission() == 0);
}

//Enable or disable the printing of debug messages
void SFE_MAX1704X::enableDebugging(Stream &debugPort)
{
  _debugPort = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true; //Should we print the commands we send? Good for debugging
}

void SFE_MAX1704X::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

uint8_t SFE_MAX1704X::quickStart()
{
  // A quick-start allows the MAX17043 to restart fuel-gauge calculations in the
  // same manner as initial power-up of the IC. If an application’s power-up
  // sequence is exceedingly noisy such that excess error is introduced into the
  // IC’s “first guess” of SOC, the host can issue a quick-start to reduce the
  // error. A quick-start is initiated by a rising edge on the QSTRT pin, or
  // through software by writing 4000h to MODE register.
  return write16(MAX17043_MODE_QUICKSTART, MAX17043_MODE);
}

float SFE_MAX1704X::getVoltage()
{
  uint16_t vCell;
  vCell = read16(MAX17043_VCELL);
  // vCell is a 12-bit register where each bit represents 1.25mV
  vCell = (vCell) >> 4;

  return ((float) vCell / 800.0);
}

float SFE_MAX1704X::getSOC()
{
  uint16_t soc;
  float percent;
  soc = read16(MAX17043_SOC);
  percent = (soc & 0xFF00) >> 8;
  percent += (float) (((uint8_t) soc) / 256.0);

  return percent;
}

uint16_t SFE_MAX1704X::getVersion()
{
  return read16(MAX17043_VERSION);
}

uint8_t SFE_MAX1704X::getThreshold()
{
  uint16_t configReg = read16(MAX17043_CONFIG);
  uint8_t threshold = (configReg & 0x001F);

  // It has an LSb weight of 1%, and can be programmed from 1% to 32%.
  // Values are in 2's complement, e.g.: 00000=32%, 00001=31%, 11111=1%.
  // Let's convert our percent to that first:
  threshold = 32 - threshold;
  return threshold;
}

uint8_t SFE_MAX1704X::setThreshold(uint8_t percent)
{
  // The alert threshold is a 5-bit value that sets the state of charge level
  // where an interrupt is generated on the ALRT pin.

  // It has an LSb weight of 1%, and can be programmed from 1% to 32%.
  // Values are in 2's complement, e.g.: 00000=32%, 00001=31%, 11111=1%.
  // Let's convert our percent to that first:
  percent = constrain(percent, 0, 32);
  percent = 32 - percent;

  // Read config reg, so we don't modify any other values:
  uint16_t configReg = read16(MAX17043_CONFIG);
  configReg &= 0xFFE0; // Mask out threshold bits
  configReg |= percent; // Add new threshold

  return write16(configReg, MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::clearAlert()
{
  // Read config reg, so we don't modify any other values:
  uint16_t configReg = read16(MAX17043_CONFIG);
  configReg &= ~(1<<MAX17043_CONFIG_ALERT); // Clear ALRT bit manually.

  return write16(configReg, MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::getAlert(bool clear)
{
  // Read config reg, so we don't modify any other values:
  uint16_t configReg = read16(MAX17043_CONFIG);
  if (configReg & (1<<MAX17043_CONFIG_ALERT))
  {
    if (clear) // If the clear flag is set
    {
      configReg &= ~(1<<MAX17043_CONFIG_ALERT); // Clear ALRT bit manually.
      write16(configReg, MAX17043_CONFIG);
    }
    return 1;
  }

  return 0;
}

uint8_t SFE_MAX1704X::sleep()
{
  // Read config reg, so we don't modify any other values:
  uint16_t configReg = read16(MAX17043_CONFIG);
  if (configReg & (1<<7))
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("sleep: MAX17043 is already sleeping!"));
    }
    return MAX17043_GENERIC_ERROR; // Already sleeping, do nothing but return an error
  }
  configReg |= (1<<7); // Set sleep bit

  return write16(configReg, MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::wake()
{
  // Read config reg, so we don't modify any other values:
  uint16_t configReg = read16(MAX17043_CONFIG);
  if (!(configReg & (1<<7)))
  {
    if (_printDebug == true)
    {
      _debugPort->println(F("sleep: MAX17043 is already awake!"));
    }
    return MAX17043_GENERIC_ERROR; // Already sleeping, do nothing but return an error
  }
  configReg &= ~(1<<7); // Clear sleep bit

  return write16(configReg, MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::reset()
{
  return write16(MAX17043_COMMAND_POR, MAX17043_COMMAND);
}

uint8_t SFE_MAX1704X::getCompensation()
{
  uint16_t configReg = read16(MAX17043_CONFIG);
  uint8_t compensation = (configReg & 0xFF00) >> 8;
  return compensation;
}

uint16_t SFE_MAX1704X::getConfigRegister()
{
  return read16(MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::setCompensation(uint8_t newCompensation)
{
  // The CONFIG register compensates the ModelGauge algorith. The upper 8 bits
  // of the 16-bit register control the compensation.
  // Read the original configReg, so we can leave the lower 8 bits alone:
  uint16_t configReg = read16(MAX17043_CONFIG);
  configReg &= 0x00FF; // Mask out compensation bits
  configReg |= ((uint16_t)newCompensation << 8) | configReg;
  return write16(configReg, MAX17043_CONFIG);
}

uint8_t SFE_MAX1704X::write16(uint16_t data, uint8_t address)
{
  uint8_t msb, lsb;
  msb = (data & 0xFF00) >> 8;
  lsb = (data & 0x00FF);
  Wire.beginTransmission(MAX17043_ADDRESS);
  Wire.write(address);
  Wire.write(msb);
  Wire.write(lsb);
  return (Wire.endTransmission());
}

uint16_t SFE_MAX1704X::read16(uint8_t address)
{
  uint8_t msb, lsb;
  int16_t timeout = 1000;

  Wire.beginTransmission(MAX17043_ADDRESS);
  Wire.write(address);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX17043_ADDRESS, 2);
  while ((Wire.available() < 2) && (timeout-- > 0))
    delay(1);
  msb = Wire.read();
  lsb = Wire.read();

  return ((uint16_t) msb << 8) | lsb;
}
