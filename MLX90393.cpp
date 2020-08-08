//
// MLX90393.cpp : arduino driver for MLX90393 magnetometer
//
// Copyright 2016 Theodore C. Yapo
//
// released under MIT License (see file)
//

#include <MLX90393.h>

MLX90393::
MLX90393()
{
  I2C_address = 0;

  cache_invalidate();

  // gain steps derived from datasheet section 15.1.4 tables
  gain_multipliers[0] = 5.f;
  gain_multipliers[1] = 4.f;
  gain_multipliers[2] = 3.f;
  gain_multipliers[3] = 2.5f;
  gain_multipliers[4] = 2.f;
  gain_multipliers[5] = 1.66666667f;
  gain_multipliers[6] = 1.33333333f;
  gain_multipliers[7] = 1.f;

  // from datasheet
  // for hallconf = 0
  base_xy_sens_hc0 = 0.196f;
  base_z_sens_hc0 = 0.316f;
  // for hallconf = 0xc
  base_xy_sens_hc0xc = 0.150f;
  base_z_sens_hc0xc = 0.242f;
}

uint8_t
MLX90393::
begin(uint8_t A1, uint8_t A0, int DRDY_pin, TwoWire &wirePort)
{
  I2C_address = I2C_BASE_ADDR | (A1?2:0) | (A0?1:0);
  this->DRDY_pin = DRDY_pin;
  if (DRDY_pin >= 0){
    pinMode(DRDY_pin, INPUT);
  }

  _i2cPort = &wirePort; //Grab which port the user wants us to use

  uint8_t status1 = checkStatus(reset());
  uint8_t status2 = setGainSel(7);
  uint8_t status3 = setResolution(0, 0, 0);
  uint8_t status4 = setOverSampling(3);
  uint8_t status5 = setDigitalFiltering(7);
  uint8_t status6 = setTemperatureCompensation(0);

  return status1 | status2 | status3 | status4 | status5 | status6;
}

void
MLX90393::
cache_invalidate()
{
  cache.dirty = cache_t::ALL_DIRTY_MASK;
}

void
MLX90393::
cache_invalidate(uint8_t address)
{
  cache.dirty |= cache_t::ALL_DIRTY_MASK & (1<<address);
}

void
MLX90393::
cache_set(uint8_t address, uint16_t data){
  if (address < cache_t::SIZE){
    cache.reg[address] = data;
    cache.dirty &= ~(1 << address);
  }
}

uint8_t
MLX90393::
cache_fill() {
  if (cache.dirty != 0) {
    for (uint8_t address=0; address < cache_t::SIZE; ++address){
      if (cache.dirty & (1 << address)){
        if (hasError(this->readRegister(address, cache.reg[address]))) {
          return STATUS_ERROR;
        }
      }
    }
  }
  return STATUS_OK;
}

uint8_t
MLX90393::
checkStatus(uint8_t status)
{
  return (status & ERROR_BIT) ? STATUS_ERROR : STATUS_OK;
}

bool
MLX90393::
isOK(uint8_t status)
{
  return (status & ERROR_BIT) == 0;
}

bool
MLX90393::
hasError(uint8_t status)
{
  return (status & ERROR_BIT) != 0;
}


uint8_t
MLX90393::
sendCommand(uint8_t cmd)
{
  _i2cPort->beginTransmission(I2C_address);
  if (_i2cPort->write(cmd) != 1){ return STATUS_ERROR; }
  if (_i2cPort->endTransmission()){ return STATUS_ERROR; }
  if (_i2cPort->requestFrom(I2C_address, uint8_t(1)) != 1){ return STATUS_ERROR; }

  return _i2cPort->read();
}

uint8_t
MLX90393::
nop()
{
  return sendCommand(CMD_NOP);
}

uint8_t
MLX90393::
exit()
{
  return sendCommand(CMD_EXIT);
}

uint8_t
MLX90393::
startBurst(uint8_t zyxt_flags)
{
  cache_fill();
  uint8_t cmd = CMD_START_BURST | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
startWakeOnChange(uint8_t zyxt_flags)
{
  cache_fill();
  uint8_t cmd = CMD_WAKE_ON_CHANGE | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
startMeasurement(uint8_t zyxt_flags)
{
  cache_fill();
  uint8_t cmd = CMD_START_MEASUREMENT | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
readMeasurement(uint8_t zyxt_flags, txyzRaw& txyz_result)
{
  uint8_t cmd = CMD_READ_MEASUREMENT | (zyxt_flags & 0xf);
  _i2cPort->beginTransmission(I2C_address);
  if(_i2cPort->write(cmd) != 1){
    return STATUS_ERROR;
  }
  if (_i2cPort->endTransmission()){
    return STATUS_ERROR;
  }

  uint8_t buffer[9];
  uint8_t count = 1 + (((zyxt_flags & Z_FLAG)?2:0) +
                       ((zyxt_flags & Y_FLAG)?2:0) +
                       ((zyxt_flags & X_FLAG)?2:0) +
                       ((zyxt_flags & T_FLAG)?2:0) );

  if(_i2cPort->requestFrom(I2C_address, count) != count){
    return STATUS_ERROR;
  }
  for (uint8_t i=0; i < count; i++){
    if (_i2cPort->available()){
      buffer[i] = _i2cPort->read();
    } else {
      return STATUS_ERROR;
    }
  }

  uint8_t i = 1;
  if (zyxt_flags & T_FLAG){
    txyz_result.t =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.t = 0;
  }
  if (zyxt_flags & X_FLAG){
    txyz_result.x =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.x = 0;
  }
  if (zyxt_flags & Y_FLAG){
    txyz_result.y =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.y = 0;
  }
  if (zyxt_flags & Z_FLAG){
    txyz_result.z =  (uint16_t(buffer[i])<<8) | buffer[i+1];
    i += 2;
  } else {
    txyz_result.z = 0;
  }

  return buffer[0];
}

uint8_t
MLX90393::
readRegister(uint8_t address, uint16_t& data)
{
  _i2cPort->beginTransmission(I2C_address);
  if (_i2cPort->write(CMD_READ_REGISTER) != 1){ return STATUS_ERROR; }
  if (_i2cPort->write((address & 0x3f)<<2) != 1){ return STATUS_ERROR; }
  if (_i2cPort->endTransmission()){ return STATUS_ERROR; }
  if (_i2cPort->requestFrom(I2C_address, uint8_t(3)) != 3){ return STATUS_ERROR; }

  uint8_t status;
  if (!_i2cPort->available()){ return STATUS_ERROR; }
  status = _i2cPort->read();

  uint8_t b_h;
  if (!_i2cPort->available()){ return STATUS_ERROR; }
  b_h = _i2cPort->read();

  uint8_t b_l;
  if (!_i2cPort->available()){ return STATUS_ERROR; }
  b_l = _i2cPort->read();

  data = (uint16_t(b_h)<<8) | b_l;
  cache_set(address, data);
  return status;
}

uint8_t
MLX90393::
writeRegister(uint8_t address, uint16_t data)
{
  cache_invalidate(address);

  _i2cPort->beginTransmission(I2C_address);
  if (_i2cPort->write(CMD_WRITE_REGISTER) != 1){ return STATUS_ERROR; }
  if (_i2cPort->write((data & 0xff00) >> 8) != 1){ return STATUS_ERROR; }
  if (_i2cPort->write(data & 0x00ff) != 1){ return STATUS_ERROR; }
  if (_i2cPort->write((address & 0x3f)<<2) != 1){ return STATUS_ERROR; }
  if (_i2cPort->endTransmission()){ return STATUS_ERROR; }
  if (_i2cPort->requestFrom(I2C_address, uint8_t(1)) != 1){ return STATUS_ERROR; }
  if (!_i2cPort->available()){ return STATUS_ERROR; }

  const uint8_t status = _i2cPort->read();
  if (isOK(status)) {
    cache_set(address, data);
  }
  return status;
}

uint8_t
MLX90393::
reset()
{
  cache_invalidate();

  uint8_t status = sendCommand(CMD_RESET);
  //Device now resets. We must give it time to complete
  delay(2);
  // POR is 1.6ms max. Software reset time limit is not specified.
  // 2ms was found to be good.

  return status;
}

uint8_t
MLX90393::
memoryRecall()
{
  cache_invalidate();
  return sendCommand(CMD_MEMORY_RECALL);
}

uint8_t
MLX90393::
memoryStore()
{
  return sendCommand(CMD_MEMORY_STORE);
}

MLX90393::txyz
MLX90393::
convertRaw(MLX90393::txyzRaw raw)
{
  const uint8_t gain_sel = (cache.reg[GAIN_SEL_REG] & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  const uint8_t hallconf = (cache.reg[HALLCONF_REG] & HALLCONF_MASK) >> HALLCONF_SHIFT;
  const uint8_t res_xyz = (cache.reg[RES_XYZ_REG] & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
  const uint8_t res_x = (res_xyz >> 0) & 0x3;
  const uint8_t res_y = (res_xyz >> 2) & 0x3;
  const uint8_t res_z = (res_xyz >> 4) & 0x3;
  uint8_t tcmp_en = (cache.reg[TCMP_EN_REG] & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

  txyz data;
  float xy_sens;
  float z_sens;

  switch(hallconf){
  default:
  case 0:
    xy_sens = base_xy_sens_hc0;
    z_sens = base_z_sens_hc0;
    break;
  case 0xc:
    xy_sens = base_xy_sens_hc0xc;
    z_sens = base_z_sens_hc0xc;
    break;
  }

  float gain_factor = gain_multipliers[gain_sel & 0x7];

  if (tcmp_en){
    data.x = ( (raw.x - 32768.f) * xy_sens *
               gain_factor * (1 << res_x) );
  } else {
    switch(res_x){
    case 0:
    case 1:
      data.x = int16_t(raw.x) * xy_sens * gain_factor * (1 << res_x);
      break;
    case 2:
      data.x = ( (raw.x - 32768.f) * xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    case 3:
      data.x = ( (raw.x - 16384.f) * xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    }
  }

  if (tcmp_en){
    data.y = ( (raw.y - 32768.f) * xy_sens *
               gain_factor * (1 << res_y) );
  } else {
    switch(res_y){
    case 0:
    case 1:
      data.y = int16_t(raw.y) * xy_sens * gain_factor * (1 << res_y);
      break;
    case 2:
      data.y = ( (raw.y - 32768.f) * xy_sens *
                 gain_factor * (1 << res_y) );
      break;
    case 3:
      data.y = ( (raw.y - 16384.f) * xy_sens *
                 gain_factor * (1 << res_y) );
      break;
    }
  }

  if (tcmp_en){
    data.z = ( (raw.z - 32768.f) * z_sens *
               gain_factor * (1 << res_z) );
  } else {
    switch(res_z){
    case 0:
    case 1:
      data.z = int16_t(raw.z) * z_sens * gain_factor * (1 << res_z);
      break;
    case 2:
      data.z = ( (raw.z - 32768.f) * z_sens *
                 gain_factor * (1 << res_z) );
      break;
    case 3:
      data.z = ( (raw.z - 16384.f) * z_sens *
                 gain_factor * (1 << res_z) );
      break;
    }
  }

  data.t = 25 + (raw.t - 46244.f)/45.2f;
  return data;
}

uint16_t
MLX90393::
convDelayMillis() {
  const uint8_t osr = (cache.reg[OSR_REG] & OSR_MASK) >> OSR_SHIFT;
  const uint8_t osr2 = (cache.reg[OSR2_REG] & OSR2_MASK) >> OSR2_SHIFT;
  const uint8_t dig_flt = (cache.reg[DIG_FLT_REG] & DIG_FLT_MASK) >> DIG_FLT_SHIFT;

  return
    (DRDY_pin >= 0)? 0 /* no delay if drdy pin present */ :
                     // estimate conversion time from datasheet equations
                     ( 3 * (2 + (1 << dig_flt)) * (1 << osr) *0.064f +
                      (1 << osr2) * 0.192f ) *
                       1.3f;  // 30% tolerance
}

uint8_t
MLX90393::
readData(MLX90393::txyz& data)
{
  uint8_t status1 = startMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);

  // wait for DRDY signal if connected, otherwise delay appropriately
  if (DRDY_pin >= 0){
    delayMicroseconds(600);
    while(!digitalRead(DRDY_pin)){
      // busy wait
    }
  } else {
    delay(this->convDelayMillis());
  }

  txyzRaw raw_txyz;
  uint8_t status2 =
    readMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG, raw_txyz);
  data = convertRaw(raw_txyz);
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
setGainSel(uint8_t gain_sel)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(GAIN_SEL_REG, old_val);

  uint8_t status2 = writeRegister(GAIN_SEL_REG,
                                  (old_val & ~GAIN_SEL_MASK) |
                                  ((uint16_t(gain_sel) << GAIN_SEL_SHIFT) &
                                   GAIN_SEL_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getGainSel(uint8_t& gain_sel)
{
  uint16_t reg_val;
  uint8_t status = readRegister(GAIN_SEL_REG, reg_val);
  gain_sel = (reg_val & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setHallConf(uint8_t hallconf)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(HALLCONF_REG, old_val);
  uint8_t status2 = writeRegister(HALLCONF_REG,
                                  (old_val & ~HALLCONF_MASK) |
                                  ((uint16_t(hallconf) << HALLCONF_SHIFT) &
                                   HALLCONF_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getHallConf(uint8_t& hallconf)
{
  uint16_t reg_val;
  uint8_t status = readRegister(HALLCONF_REG, reg_val);
  hallconf = (reg_val & HALLCONF_MASK) >> HALLCONF_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setOverSampling(uint8_t osr)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(OSR_REG, old_val);
  uint8_t status2 = writeRegister(OSR_REG,
                                  (old_val & ~OSR_MASK) |
                                  ((uint16_t(osr) << OSR_SHIFT) & OSR_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getOverSampling(uint8_t& osr)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR_REG, reg_val);
  osr = (reg_val & OSR_MASK) >> OSR_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setTemperatureOverSampling(uint8_t osr2)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(OSR2_REG, old_val);
  uint8_t status2 = writeRegister(OSR2_REG,
                                  (old_val & ~OSR2_MASK) |
                                  ((uint16_t(osr2) << OSR2_SHIFT) & OSR2_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getTemperatureOverSampling(uint8_t& osr2)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR2_REG, reg_val);
  osr2 = (reg_val & OSR2_MASK) >> OSR2_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setDigitalFiltering(uint8_t dig_flt)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(DIG_FLT_REG, old_val);
  uint8_t status2 = writeRegister(DIG_FLT_REG,
                                  (old_val & ~DIG_FLT_MASK) |
                                  ((uint16_t(dig_flt) << DIG_FLT_SHIFT) &
                                   DIG_FLT_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getDigitalFiltering(uint8_t& dig_flt)
{
  uint16_t reg_val;
  uint8_t status = readRegister(DIG_FLT_REG, reg_val);
  dig_flt = (reg_val & DIG_FLT_MASK) >> DIG_FLT_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setResolution(uint8_t res_x, uint8_t res_y, uint8_t res_z)
{
  uint16_t res_xyz = ((res_z & 0x3)<<4) | ((res_y & 0x3)<<2) | (res_x & 0x3);
  uint16_t old_val;
  uint8_t status1 = readRegister(RES_XYZ_REG, old_val);
  uint8_t status2 = writeRegister(RES_XYZ_REG,
                                  (old_val & ~RES_XYZ_MASK) |
                                  (res_xyz << RES_XYZ_SHIFT) & RES_XYZ_MASK);
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getResolution(uint8_t& res_x, uint8_t& res_y, uint8_t& res_z)
{
  uint16_t reg_val;
  uint8_t status = readRegister(RES_XYZ_REG, reg_val);
  uint8_t res_xyz = (reg_val & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
  res_x = (res_xyz >> 0) & 0x3;
  res_y = (res_xyz >> 2) & 0x3;
  res_z = (res_xyz >> 4) & 0x3;
  return checkStatus(status);
}

uint8_t
MLX90393::
setTemperatureCompensation(uint8_t enabled)
{
  uint8_t tcmp_en = enabled?1:0;
  uint16_t old_val;
  uint8_t status1 = readRegister(TCMP_EN_REG, old_val);
  uint8_t status2 = writeRegister(TCMP_EN_REG,
                                  (old_val & ~TCMP_EN_MASK) |
                                  ((uint16_t(tcmp_en) << TCMP_EN_SHIFT) &
                                   TCMP_EN_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getTemperatureCompensation(uint8_t& enabled)
{
  uint16_t reg_val;
  uint8_t status = readRegister(TCMP_EN_REG, reg_val);
  enabled = (reg_val & TCMP_EN_MASK) >> TCMP_EN_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setBurstSel(uint8_t burst_sel)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(BURST_SEL_REG, old_val);
  uint8_t status2 = writeRegister(BURST_SEL_REG,
                                  (old_val & ~BURST_SEL_MASK) |
                                  ((uint16_t(burst_sel) << BURST_SEL_SHIFT) &
                                   BURST_SEL_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getBurstSel(uint8_t& burst_sel)
{
  uint16_t reg_val;
  uint8_t status = readRegister(BURST_SEL_REG, reg_val);
  burst_sel = (reg_val & BURST_SEL_MASK) >> BURST_SEL_SHIFT;
  return checkStatus(status);
}

uint8_t
MLX90393::
setExtTrig(int8_t ext_trig)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(EXT_TRIG_REG, old_val);
  uint8_t status2 = writeRegister(EXT_TRIG_REG,
                                  (old_val & ~EXT_TRIG_MASK) |
                                  ((uint16_t(ext_trig) << EXT_TRIG_SHIFT) &
                                   EXT_TRIG_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getExtTrig(uint8_t& ext_trig)
{
  uint16_t reg_val;
  uint8_t status = readRegister(EXT_TRIG_REG, reg_val);
  ext_trig = (reg_val & EXT_TRIG_MASK) >> EXT_TRIG_SHIFT;
  return checkStatus(status);

}

uint8_t
MLX90393::
setTrigIntSel(uint8_t trig_int_sel)
{
  uint16_t old_val;
  uint8_t status1 = readRegister(TRIG_INT_SEL_REG, old_val);
  uint8_t status2 = writeRegister(TRIG_INT_SEL_REG,
                                  (old_val & ~TRIG_INT_SEL_MASK) |
                                  ((uint16_t(trig_int_sel) << TRIG_INT_SEL_SHIFT) &
                                   TRIG_INT_SEL_MASK));
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getTrigIntSel(uint8_t& trig_int_sel)
{
  uint16_t reg_val;
  uint8_t status = readRegister(TRIG_INT_SEL_REG, reg_val);
  trig_int_sel = (reg_val & TRIG_INT_SEL_MASK) >> TRIG_INT_SEL_SHIFT;
  return checkStatus(status);
}


//
// Note: offsets are relative to 0x8000
//  the default value of 0 in these registers will give poor results
//
uint8_t
MLX90393::
setOffsets(uint16_t x, uint16_t y, uint16_t z)
{
  uint8_t status1 = writeRegister(X_OFFSET_REG, x);
  uint8_t status2 = writeRegister(Y_OFFSET_REG, y);
  uint8_t status3 = writeRegister(Z_OFFSET_REG, z);
  return checkStatus(status1) | checkStatus(status2) | checkStatus(status3);
}

uint8_t
MLX90393::
setWOXYThreshold(uint16_t woxy_thresh)
{
  uint8_t status = writeRegister(WOXY_THRESHOLD_REG, woxy_thresh);
  return checkStatus(status);
}

uint8_t
MLX90393::
setWOZThreshold(uint16_t woz_thresh)
{
  uint8_t status = writeRegister(WOZ_THRESHOLD_REG, woz_thresh);
  return checkStatus(status);
}


uint8_t
MLX90393::
setWOTThreshold(uint16_t wot_thresh)
{
  uint8_t status = writeRegister(WOT_THRESHOLD_REG, wot_thresh);
  return checkStatus(status);
}
