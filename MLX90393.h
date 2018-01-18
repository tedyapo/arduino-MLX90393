//
// MLX90393.cpp : arduino driver for MLX90393 magnetometer
//
// Copyright 2016 Theodore C. Yapo
//
// released under MIT License (see file)
//

#ifndef MLX90393_H_INCLUDED
#define MLX90393_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>

class MLX90393
{
public:
  enum { STATUS_OK = 0, STATUS_ERROR = 0xff } return_status_t;
  enum { Z_FLAG = 0x8, Y_FLAG = 0x4, X_FLAG = 0x2, T_FLAG = 0x1 } axis_flag_t;
  enum { I2C_BASE_ADDR = 0x0c };
  enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x0070, GAIN_SEL_SHIFT = 4 };
  enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
  enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
  enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
  enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
  enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
  enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
  enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
  enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
  enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
  enum { X_OFFSET_REG = 4, Y_OFFSET_REG = 5, Z_OFFSET_REG = 6 };
  enum { WOXY_THRESHOLD_REG = 7, WOZ_THRESHOLD_REG = 8, WOT_THRESHOLD_REG = 9 };
  enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
         POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
         RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

  enum {
    CMD_NOP = 0x00,
    CMD_EXIT = 0x80,
    CMD_START_BURST = 0x10,
    CMD_WAKE_ON_CHANGE = 0x20,
    CMD_START_MEASUREMENT = 0x30,
    CMD_READ_MEASUREMENT = 0x40,
    CMD_READ_REGISTER = 0x50,
    CMD_WRITE_REGISTER = 0x60,
    CMD_MEMORY_RECALL = 0xd0,
    CMD_MEMORY_STORE = 0xe0,
    CMD_RESET = 0xf0
  };

  struct txyz
  {
    float t;
    float x;
    float y;
    float z;
  };
  struct txyzRaw
  {
    uint16_t t;
    uint16_t x;
    uint16_t y;
    uint16_t z;
  };
  MLX90393();

  // raw device commands
  uint8_t exit();
  uint8_t startBurst(uint8_t zyxt_flags);
  uint8_t startWakeOnChange(uint8_t zyxt_flags);
  uint8_t startMeasurement(uint8_t zyxt_flags);
  uint8_t readMeasurement(uint8_t zyxt_flags, txyzRaw& txyz_result);
  uint8_t readRegister(uint8_t address, uint16_t& data);
  uint8_t writeRegister(uint8_t address, uint16_t data);
  uint8_t reset();
  uint8_t memoryRecall();
  uint8_t memoryStore();
  uint8_t nop();
  uint8_t sendCommand(uint8_t cmd);
  uint8_t checkStatus(uint8_t status);
  txyz convertRaw(txyzRaw raw);
  uint16_t convDelayMillis();

  // higher-level API
  uint8_t begin(uint8_t A1 = 0, uint8_t A0 = 0, int DRDY_pin = -1, TwoWire &wirePort = Wire);

  // returns B (x,y,z) in uT, temperature in C
  uint8_t readData(txyz& data);
  uint8_t setGainSel(uint8_t gain_sel);
  uint8_t getGainSel(uint8_t& gain_sel);
  uint8_t setHallConf(uint8_t hallconf);
  uint8_t getHallConf(uint8_t& hallconf);
  uint8_t setBurstSel(uint8_t burst_sel);
  uint8_t getBurstSel(uint8_t& burst_sel);
  uint8_t setExtTrig(int8_t ext_trig);
  uint8_t getExtTrig(uint8_t& ext_trig);
  uint8_t setTrigIntSel(uint8_t trig_int_sel);
  uint8_t getTrigIntSel(uint8_t& trig_int_sel);
  uint8_t setOverSampling(uint8_t osr);
  uint8_t getOverSampling(uint8_t& osr);
  uint8_t setTemperatureOverSampling(uint8_t osr2);
  uint8_t getTemperatureOverSampling(uint8_t& osr2);
  uint8_t setDigitalFiltering(uint8_t dig_flt);
  uint8_t getDigitalFiltering(uint8_t& dig_flt);
  uint8_t setResolution(uint8_t res_x, uint8_t res_y, uint8_t res_z);
  uint8_t getResolution(uint8_t& res_x, uint8_t& res_y, uint8_t& res_z);
  uint8_t setTemperatureCompensation(uint8_t enabled);
  uint8_t getTemperatureCompensation(uint8_t& enabled);
  uint8_t setOffsets(uint16_t x, uint16_t y, uint16_t z);
  uint8_t setWOXYThreshold(uint16_t woxy_thresh);
  uint8_t setWOZThreshold(uint16_t woz_thresh);
  uint8_t setWOTThreshold(uint16_t wot_thresh);

private:

  // parameters are cached to avoid reading them from sensor unnecessarily
  uint8_t I2C_address;
  int DRDY_pin;
  uint8_t gain_sel;
  uint8_t hallconf;
  uint8_t res_x;
  uint8_t res_y;
  uint8_t res_z;
  uint8_t osr;
  uint8_t osr2;
  uint8_t dig_flt;
  uint8_t tcmp_en;

  struct cache_t {
    enum { SIZE = 3 };
    uint8_t dirty;
    uint16_t reg[SIZE];
  } cache;
  void cache_invalidate();
  void cache_set(uint8_t address, uint16_t data);
  uint8_t cache_fill();


  float gain_multipliers[8];
  float base_xy_sens_hc0;
  float base_z_sens_hc0;
  float base_xy_sens_hc0xc;
  float base_z_sens_hc0xc;

  private:
    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware

};
#endif // #ifndef MLX90393_H_INCLUDED
