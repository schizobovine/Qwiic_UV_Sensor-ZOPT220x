//
// ZOPT2201.cpp - Library to read and control IDT's ZOPT2201 UV index sensor
//
// Based on code by Nathan Seidle provided for the SparkX breakout for the same sensor.
// Libraryification by Sean Caulfield.
//

#include <Arduino.h>
#include "ZOPT2201.h"

// Setup the sensor with default settings
bool ZOPT2201::begin() {
  return this->begin(this->my_addr);
}

// Alternate setup to allow for different i2c addresses
bool ZOPT2201::begin(uint8_t addr) {

  this->my_addr = addr;

  // Check if device is present
  uint8_t dev_id = this->read_byte(ZOPT220x_PART_ID);
  if (dev_id != SENSORTYPE_ZOPT2201 || dev_id != SENSORTYPE_ZOPT2202) {
    return false;
  }

  this->checkPowerOnStatus(); // check this to clear it?
  this->setMeasurementRate(ZOPT220x_RATE_100MS); //100ms default
  this->setResolution(ZOPT220x_RES_20_BIT); //20 bit, 400ms, needed for best
                                            //UVI calculations
  this->setGain(ZOPT220x_GAIN_18); //Default for UVB

  return true;
}

//Return the current UV index
//Assumes UVB sensor has been setup with default settings
float ZOPT2201::getUVIndex(void)
{
  uint8_t counter = 0;
  while(!this->dataAvailable())
  {
    delay(10);
    if(counter++ > 50) //We should have a reading in 400ms
    {
      //We've timed out waiting for the data. Return error
      return(-1.0);
    }
  }

  int32_t uvb = getUVB(); //Get new data
  float uvIndex = getAdjustedUVIndex(uvb);
  return(uvIndex);
}

//Given UVB reading, and gain and resolution settings, outputs UV index
//From App note UV Index Calculation from IDT
//Retrieves gain and resolution from current instance settings
float ZOPT2201::getAdjustedUVIndex(int32_t uvb) {
  uint8_t gainMode = this->getGainValue();
  uint8_t resolutionMode = this->getResolutionValue();

  int32_t uvAdjusted = 18 / gainMode * pow(2, 20 - resolutionMode) * uvb;

  //The correction value is default 1. If you want to calculate the actual
  //solar angle you'll need lat/long/day/hh/mm
  float uviCorrection = 1.0;

  //First we need to calculation the elevation angle
  /*float elevationAngle = 

  //Let's calculate the correction!
  float c0 = 3.1543;
  float c1 = -0.06204;
  float c2 = 0.0002186;
  float c3 = 0.0000035516;
  uviCorrection = c0 * pow(elevation, 0);
  uviCorrection += c1 * pow(elevation, 1);
  uviCorrection += c2 * pow(elevation, 2);
  uviCorrection += c3 * pow(elevation, 3);*/  
  
  float uvIndex = uvAdjusted / (5500.0 * uviCorrection);

  return(uvIndex);
}

//Returns the current UV data with temp compensation applied
//Assumes LS data bit 3 has been set (MAIN_STATUS)
//Poll dataAvailable() before calling this function
int32_t ZOPT2201::getUVB() 
{
  return this->read3(ZOPT220x_UVB_DATA);
}

//Returns the current Ambient Light data with temp compensation applied
//Assumes data is available (LS data bit 3 has been set in MAIN_STATUS)
//Poll dataAvailable() before calling this function
int32_t ZOPT2201::getALS() 
{
  return this->read3(ZOPT220x_ALS_DATA);
}

// Fetch status register and return true if the bit is set
bool ZOPT2201::checkPowerOnStatus() {
  uint8_t status = this->read_byte(ZOPT220x_MAIN_STATUS);
  return (status & ZOPT220x_STATUS_POS_MASK);
}

// Fetch status register and return true if data is available
bool ZOPT2201::dataAvailable() {
  uint8_t status = this->read_byte(ZOPT220x_MAIN_STATUS);
  return (status & ZOPT220x_STATUS_DATA_AVAIL_MASK);
}

// Enable sensor
void ZOPT2201::enable() {
  uint8_t ctrl = this->read_byte(ZOPT220x_MAIN_CTRL);
  ctrl |= ZOPT220x_CTRL_LS_EN;
  this->write_byte(ZOPT220x_MAIN_CTRL, ctrl);
}

// Put sensor to sleep
void ZOPT2201::disable() {
  uint8_t ctrl = this->read_byte(ZOPT220x_MAIN_CTRL);
  ctrl &= ~ZOPT220x_CTRL_LS_EN;
  this->write_byte(ZOPT220x_MAIN_CTRL, ctrl);
}

// Set measurement rate
//0 = 25ms per measurement
//1 = 50ms
//2 = 100ms (default)
//3 = 200ms
//4 = 500ms
//5 = 1000ms
//6 = 2000ms
//7 = 2000ms
void ZOPT2201::setMeasurementRate(enum zopt2201_meas_rate rate) {
  this->my_meas_rate = rate;

  uint8_t value = this->read_byte(ZOPT220x_LS_MEAS_RATE);
  value &= ZOPT220x_RATE_MASK;
  value |= (((uint8_t)rate) << ZOPT220x_RATE_SHIFT);

  this->write_byte(ZOPT220x_LS_MEAS_RATE, value);
}

//Set the measurement resolution
//0 = 20 bit / 400ms
//1 = 19 bit / 200ms
//2 = 18 bit / 100ms (default)
//3 = 17 bit / 50ms
//4 = 16 bit / 25ms
//5 = 13 bit / 3.125ms
void ZOPT2201::setResolution(enum zopt2201_resolution res) {
  this->my_res = res;

  uint8_t value = this->read_byte(ZOPT220x_LS_MEAS_RATE);
  value &= ZOPT220x_RES_MASK;
  value |= (((uint8_t)res) << ZOPT220x_RES_SHIFT);

  this->write_byte(ZOPT220x_LS_MEAS_RATE, value);
}

//Set the gain for the sensor
//Different for ALS and UVB. It depends which mode you're in. See datasheet for info.
//0 = Gain 1
//1 = Gain 3 (default)
//2 = Gain 6
//3 = Gain 9
//4 = Gain 18 (recommended for UVB)
void ZOPT2201::setGain(enum zopt2201_gain gain) {
  this->my_gain = gain;

  uint8_t value = this->read_byte(ZOPT220x_LS_GAIN);
  value &= ZOPT220x_GAIN_MASK;
  value |= (((uint8_t)gain) << ZOPT220x_GAIN_SHIFT);

  this->write_byte(ZOPT220x_LS_GAIN, value);
}

//Sets the mode of the sensor
//Mode 1: Standby (power on default)
//Mode 2: Ambient light sensing w/ compensation
//Mode 3: UVB sensing w/ compensation
//Mode 4: ALS/UVBS_Raw
void ZOPT2201::setMode(enum zopt2201_mode mode) {
  // Read current control register value
  uint8_t value = this->read_byte(ZOPT220x_MAIN_CTRL);

  switch (mode) {
    case ZOPT2201_MODE_STANDBY:
      value &= ~ZOPT220x_CTRL_LS_EN;
      value &= ~ZOPT220x_CTRL_RAW_SEL;
      value &= ~ZOPT220x_CTRL_LS_MODE;
      break;
    case ZOPT2201_MODE_ALS:
      value |= ZOPT220x_CTRL_LS_EN;
      value &= ~ZOPT220x_CTRL_RAW_SEL;
      value &= ~ZOPT220x_CTRL_LS_MODE;
      break;
    case ZOPT2201_MODE_UVB:
      value |= ZOPT220x_CTRL_LS_EN;
      value &= ~ZOPT220x_CTRL_RAW_SEL;
      value |= ZOPT220x_CTRL_LS_MODE;
      break;
    case ZOPT2201_MODE_RAW:
      //Send magic twiddly bytes
      this->write_byte(ZOPT220x_SPECIAL_MODE_2, 0xB5); //Write first special byte
      this->write_byte(ZOPT220x_SPECIAL_MODE_1, 0xDF); //Write second special byte
      this->write_byte(ZOPT220x_SPECIAL_MODE_2, 0x04); //Write third special byte
      value |= ZOPT220x_CTRL_LS_EN;
      value |= ZOPT220x_CTRL_RAW_SEL;
      value &= ~ZOPT220x_CTRL_LS_MODE;
      break;
    default:
      break; //XXX probably should do something here?
  }

  this->write_byte(ZOPT220x_MAIN_CTRL, value); //Write
}

// Get the gain multiplier value
uint8_t ZOPT2201::getGainValue() {
  switch (this->my_gain) {
    case ZOPT220x_GAIN_1:  return 1;
    case ZOPT220x_GAIN_3:  return 3;
    case ZOPT220x_GAIN_6:  return 6;
    case ZOPT220x_GAIN_9:  return 9;
    case ZOPT220x_GAIN_18: return 18;
    default:               return 0;
  }
}

// Get the number of resolution bits
uint8_t ZOPT2201::getResolutionValue() {
  switch (this->my_res) {
    case ZOPT220x_RES_20_BIT: return 20;
    case ZOPT220x_RES_19_BIT: return 19;
    case ZOPT220x_RES_18_BIT: return 18;
    case ZOPT220x_RES_17_BIT: return 17;
    case ZOPT220x_RES_16_BIT: return 16;
    case ZOPT220x_RES_13_BIT: return 13;
    default:	return 0;
  }
}

//Reads from a given location from the ZOPT2201
uint8_t ZOPT2201::read_byte(uint8_t reg_addr)
{
  uint8_t retval = 0xFF;

  Wire.beginTransmission(this->my_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(this->my_addr, (uint8_t)1);
  if (Wire.available()) {
    retval = Wire.read();
  }

  return retval;
}

//Write a value to a spot in the ZOPT220x
void ZOPT2201::write_byte(uint8_t reg_addr, uint8_t val)
{
  Wire.beginTransmission(this->my_addr);
  Wire.write(reg_addr);
  Wire.write(val);
  Wire.endTransmission();
}

//Read 20bit values from the given register address
int32_t ZOPT2201::read3(uint8_t reg_addr) {

  Wire.beginTransmission(this->my_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(this->my_addr, (uint8_t)3);

  // Wait for data to become available
  uint8_t i = 0;
  while (Wire.available() < 3) {
    delay(1);
    if (i++ > 25) {
      return -1;
    }
  }

  uint8_t b1, b2, b3;
  int32_t r;

  b1 = Wire.read();
  b2 = Wire.read();
  b3 = Wire.read();
  r = (b3 << 16) | (b2 << 8) | (b1 << 0);

  return r;

}

// vi: syntax=arduino ts=2 sw=2
