//
// ZOPT2201.h - Library to read and control IDT's ZOPT2201 UV index sensor
//
// Based on code by Nathan Seidle provided for the SparkX breakout for the same sensor.
// Libraryification by Sean Caulfield.
//

#ifndef _ZOPT2201_H
#define _ZOPT2201_H

#include <Arduino.h>
#include <Wire.h>

#define ZOPT2201_ADDR (0x53)

//Register addresses
#define ZOPT220x_MAIN_CTRL      (0x00)
#define ZOPT220x_LS_MEAS_RATE   (0x04)
#define ZOPT220x_LS_GAIN        (0x05)
#define ZOPT220x_PART_ID        (0x06)
#define ZOPT220x_MAIN_STATUS    (0x07)
#define ZOPT220x_ALS_DATA       (0x0D)
#define ZOPT220x_UVB_DATA       (0x10)
#define ZOPT220x_UV_COMP_DATA   (0x13)
#define ZOPT220x_COMP_DATA      (0x16)
#define ZOPT220x_INT_CFG        (0x19)
#define ZOPT220x_INT_PST        (0x1A)
#define ZOPT220x_DEVICE_CONFIG  (0x2F)
#define ZOPT220x_SPECIAL_MODE_1 (0x30)
#define ZOPT220x_SPECIAL_MODE_2 (0x31)

#define ZOPT220x_STATUS_POS_MASK        (1<<5)
#define ZOPT220x_STATUS_DATA_AVAIL_MASK (1<<3)

#define ZOPT220x_CTRL_LS_EN     (1<<1)
#define ZOPT220x_CTRL_RAW_SEL   (1<<2)
#define ZOPT220x_CTRL_LS_MODE   (1<<3)

// Really IDT!? You didn't use a different part number for different parts?
#define SENSORTYPE_ZOPT2201     0xB2
#define SENSORTYPE_ZOPT2202     0xB2

#define ZOPT220x_RATE_MASK    (~(0b00000111))
#define ZOPT220x_RATE_SHIFT   (0)

#define ZOPT220x_RES_MASK     (~(0b01110000))
#define ZOPT220x_RES_SHIFT    (4)

#define ZOPT220x_GAIN_MASK    (~(0b00000111))
#define ZOPT220x_GAIN_SHIFT   (0)

enum zopt2201_meas_rate {
  ZOPT220x_RATE_25MS    = 0x00,
  ZOPT220x_RATE_50MS    = 0x01,
  ZOPT220x_RATE_100MS   = 0x02, //default
  ZOPT220x_RATE_200MS   = 0x03,
  ZOPT220x_RATE_500MS   = 0x04,
  ZOPT220x_RATE_1000MS  = 0x05,
  ZOPT220x_RATE_2000MS  = 0x06,
  ZOPT220x_RATE_XXXXXX  = 0x07
};

enum zopt2201_resolution {
  ZOPT220x_RES_20_BIT   = 0x00, //default
  ZOPT220x_RES_19_BIT   = 0x01,
  ZOPT220x_RES_18_BIT   = 0x02,
  ZOPT220x_RES_17_BIT   = 0x03,
  ZOPT220x_RES_16_BIT   = 0x04,
  ZOPT220x_RES_13_BIT   = 0x05
};

enum zopt2201_gain {
  ZOPT220x_GAIN_1       = 0x00,
  ZOPT220x_GAIN_3       = 0x01, //default
  ZOPT220x_GAIN_6       = 0x02,
  ZOPT220x_GAIN_9       = 0x03,
  ZOPT220x_GAIN_18      = 0x04  //recommened for UVB
};

enum zopt2201_mode {
  ZOPT2201_MODE_STANDBY = 0x00,
  ZOPT2201_MODE_ALS     = 0x01,
  ZOPT2201_MODE_UVB     = 0x02,
  ZOPT2201_MODE_RAW     = 0x03
};

class ZOPT2201 {

  public:
    ZOPT2201(uint8_t addr) { this->my_addr = addr; };
    ZOPT2201() : ZOPT2201(ZOPT2201_ADDR) {};
    bool begin();
    bool begin(uint8_t addr);
    bool checkPowerOnStatus();
    bool dataAvailable();
    void enable();
    void disable();
    void setMeasurementRate(enum zopt2201_meas_rate rate);
    void setResolution(enum zopt2201_resolution res);
    void setGain(enum zopt2201_gain gain);
    void setMode(enum zopt2201_mode mode);
    float getUVIndex();
    float getAdjustedUVIndex(int32_t uvb);
    int32_t getUVB();
    int32_t getALS();

    uint8_t getGainValue();
    uint8_t getResolutionValue();

  protected:
    uint8_t my_addr;
    enum zopt2201_meas_rate my_meas_rate;
    enum zopt2201_resolution my_res;
    enum zopt2201_gain my_gain;
    enum zopt2201_mode my_mode;

    uint8_t read_byte(uint8_t reg_addr);
    void write_byte(uint8_t reg_addr, uint8_t val);
    int32_t read3(uint8_t reg_addr);

};

#endif
