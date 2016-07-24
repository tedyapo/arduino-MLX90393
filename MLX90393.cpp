#include <MLX90393.h>

MLX90393::
MLX90393()
{
  I2C_address = 0;
  gain_sel = 0;
  gain_sel_dirty = 1;
  res_x = 0;
  res_y = 0;
  res_z = 0;
  res_xyz_dirty = 1;
  osr = 0;
  osr_dirty = 1;
  osr2 = 0;
  osr2_dirty = 1;
  dig_flt = 0;
  dig_flt_dirty = 1;
  tcmp_en = 0;
  tcmp_en_dirty = 1;

  // gain steps are exp(log(5)/7), i.e. 7 steps for 5x
  gain_multipliers[0] = 1.25849895064183f;
  gain_multipliers[1] = 1.58381960876658f;
  gain_multipliers[2] = 1.99323531563869f;
  gain_multipliers[3] = 2.50848455311352f;
  gain_multipliers[4] = 3.15692517779460f;
  gain_multipliers[5] = 3.97298702350926f;
  gain_multipliers[6] = 5.f;
 
  // from datasheet
  base_xy_sens = 0.161;
  base_z_sens = 0.294;
}

uint8_t
MLX90393::
begin(uint8_t A1, uint8_t A0, int DRDY_pin)
{
  I2C_address = I2C_BASE_ADDR | (A1?2:0) | (A0?1:0);
  this->DRDY_pin = DRDY_pin;
  if (DRDY_pin >= 0){
    pinMode(DRDY_pin, INPUT);
  }
  Wire.begin();
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
invalidateCache()
{
  gain_sel_dirty = 1;
  res_xyz_dirty = 1;
  osr_dirty = 1;
  osr2_dirty = 1;
  dig_flt_dirty = 1;
  tcmp_en_dirty = 1; 
}

uint8_t
MLX90393::
checkStatus(uint8_t status)
{
  if (status & ERROR_BIT){
    return STATUS_ERROR;
  } else {
    return STATUS_OK;
  }
}

uint8_t
MLX90393::
sendCommand(uint8_t cmd)
{
  Wire.beginTransmission(I2C_address);
  Wire.write(cmd);
  Wire.endTransmission();
  Wire.requestFrom(I2C_address, uint8_t(1));
  if (Wire.available()){
    return Wire.read();
  } else {
    return STATUS_ERROR;
  }
}

uint8_t
MLX90393::
exit(uint8_t address)
{
  return sendCommand(CMD_EXIT);
}

uint8_t
MLX90393::
startBurst(uint8_t zyxt_flags)
{
  uint8_t cmd = CMD_START_BURST | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
startWakeOnChange(uint8_t zyxt_flags)
{
  uint8_t cmd = CMD_WAKE_ON_CHANGE | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
startMeasurement(uint8_t zyxt_flags)
{
  uint8_t cmd = CMD_START_MEASUREMENT | (zyxt_flags & 0xf);
  return sendCommand(cmd);
}

uint8_t
MLX90393::
readMeasurement(uint8_t zyxt_flags, txyzRaw& txyz_result)
{
  uint8_t cmd = CMD_READ_MEASUREMENT | (zyxt_flags & 0xf);
  Wire.beginTransmission(I2C_address);
  Wire.write(cmd);
  Wire.endTransmission();

  uint8_t buffer[9];
  uint8_t count = 1 + (((zyxt_flags & Z_FLAG)?2:0) +
                       ((zyxt_flags & Y_FLAG)?2:0) + 
                       ((zyxt_flags & X_FLAG)?2:0) +
                       ((zyxt_flags & T_FLAG)?2:0) );
  
  Wire.requestFrom(I2C_address, count);
  for (uint8_t i=0; i < count; i++){
    if (Wire.available()){
      buffer[i] = Wire.read();
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
  uint8_t cmd1 = CMD_READ_REGISTER;
  uint8_t cmd2 = (address & 0x3f)<<2;
  Wire.beginTransmission(I2C_address);
  Wire.write(cmd1);
  Wire.write(cmd2);
  Wire.endTransmission();
  Wire.requestFrom(I2C_address, uint8_t(3));
  uint8_t status, b_h, b_l;
  if (Wire.available()){
    status = Wire.read();
  } else {
    return STATUS_ERROR;
  }
  if (Wire.available()){
    b_h = Wire.read();
  } else {
    return STATUS_ERROR;
  }
  if (Wire.available()){
    b_l = Wire.read();
  } else {
    return STATUS_ERROR;
  }
  data = (uint16_t(b_h)<<8) | b_l;
  return status;
}

uint8_t
MLX90393::
writeRegister(uint8_t address, uint16_t data)
{
  invalidateCache();
  uint8_t cmd1 = CMD_WRITE_REGISTER;
  uint8_t cmd2 = (data & 0xff00) >> 8;
  uint8_t cmd3 = (data & 0x00ff);
  uint8_t cmd4 = (address & 0x3f)<<2;
  Wire.beginTransmission(I2C_address);
  Wire.write(cmd1);
  Wire.write(cmd2);
  Wire.write(cmd3);
  Wire.write(cmd4);
  Wire.endTransmission();
  Wire.requestFrom(I2C_address, uint8_t(1));
  if (Wire.available()){
    return Wire.read();
  } else {
    return STATUS_ERROR;
  }
}

uint8_t
MLX90393::
reset()
{
  invalidateCache();
  uint8_t cmd = CMD_RESET;
  return sendCommand(cmd);
}

uint8_t
MLX90393::
memoryRecall()
{
  invalidateCache();
  uint8_t cmd = CMD_MEMORY_RECALL;
  return sendCommand(cmd);
}

uint8_t
MLX90393::
memoryStore()
{
  uint8_t cmd = CMD_MEMORY_STORE;
  return sendCommand(cmd);
}

MLX90393::txyz
MLX90393::
convertRaw(MLX90393::txyzRaw raw)
{
  txyz data;

  float gain_factor = gain_multipliers[7 - (gain_sel & 0x7)];

  if (tcmp_en){
    data.x = ( (raw.x - 32768.f) * base_xy_sens *
               gain_factor * (1 << res_x) );
  } else {
    switch(res_x){
    case 0:
    case 1:
      data.x = int16_t(raw.x) * base_xy_sens * gain_factor * (1 << res_x);
      break;
    case 2:
      data.x = ( (raw.x - 32768.f) * base_xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    case 3:
      data.x = ( (raw.x - 16384.f) * base_xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    }
  }

  if (tcmp_en){
    data.y = ( (raw.y - 32768.f) * base_xy_sens *
               gain_factor * (1 << res_y) );
  } else {
    switch(res_y){
    case 0:
    case 1:
      data.y = int16_t(raw.y) * base_xy_sens * gain_factor * (1 << res_y);
      break;
    case 2:
      data.y = ( (raw.y - 32768.f) * base_xy_sens * 
                 gain_factor * (1 << res_y) );
      break;
    case 3:
      data.y = ( (raw.y - 16384.f) * base_xy_sens *
                 gain_factor * (1 << res_y) );
      break;
    }
  }

  if (tcmp_en){
    data.z = ( (raw.z - 32768.f) * base_z_sens *
               gain_factor * (1 << res_z) );
  } else {
    switch(res_z){
    case 0:
    case 1:
      data.z = int16_t(raw.z) * base_z_sens * gain_factor * (1 << res_z);
      break;
    case 2:
      data.z = ( (raw.z - 32768.f) * base_z_sens *
                 gain_factor * (1 << res_z) );
      break;
    case 3:
      data.z = ( (raw.z - 16384.f) * base_z_sens * 
                 gain_factor * (1 << res_z) );
      break;
    }
  }

  data.t = 25 + (raw.t - 46244.f)/45.2f;
  return data;
}

uint8_t
MLX90393::
readData(MLX90393::txyz& data)
{
  // refresh cached values if dirty - used for scaling after read
  if (gain_sel_dirty){
    uint8_t gs;
    getGainSel(gs);
  }
  if (res_xyz_dirty){
    uint8_t rx, ry, rz;
    getResolution(rx, ry, rz);
  }
  if (tcmp_en_dirty){
    uint8_t en;
    getTemperatureCompensation(en);
  }
  if (DRDY_pin < 0){
    if (osr_dirty){
      uint8_t osr;
      getOverSampling(osr);
    }
    if (osr2_dirty){
      uint8_t osr2;
      getOverSampling(osr2);
    }
  }

  uint8_t status1 = startMeasurement(X_FLAG | Y_FLAG | Z_FLAG | T_FLAG);

  // wait for DRDY signal if connected, otherwise delay appropriately
  if (DRDY_pin >= 0){
    delayMicroseconds(600);
    while(!digitalRead(DRDY_pin)){
      // busy wait
    }
  } else {
    // estimate conversion time from datasheet equations
    float Tconv = ( 3 * (2 + (1 << dig_flt)) * (1 << osr) *0.064f +
                    (1 << osr2) * 0.192f );
    // add 30% tolerance
    delay(Tconv * 1.3f);
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
  this->gain_sel = ((uint16_t(gain_sel) << GAIN_SEL_SHIFT) &
                    GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  gain_sel_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getGainSel(uint8_t& gain_sel)
{
  uint16_t reg_val;
  uint8_t status = readRegister(GAIN_SEL_REG, reg_val);
  this->gain_sel = gain_sel = (reg_val & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  gain_sel_dirty = 0;
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
  this->osr = ((uint16_t(osr) << OSR_SHIFT) & OSR_MASK) >> OSR_SHIFT;
  osr_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getOverSampling(uint8_t& osr)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR_REG, reg_val);
  this->osr = osr = (reg_val & OSR_MASK) >> OSR_SHIFT;
  osr_dirty = 0;
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
  this->osr2 = ((uint16_t(osr2) << OSR2_SHIFT) & OSR2_MASK) >> OSR2_SHIFT;
  osr2_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getTemperatureOverSampling(uint8_t& osr2)
{
  uint16_t reg_val;
  uint8_t status = readRegister(OSR2_REG, reg_val);
  this->osr2 = osr2 = (reg_val & OSR2_MASK) >> OSR2_SHIFT;
  osr2_dirty = 0;
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
  this->dig_flt = ((uint16_t(dig_flt) << DIG_FLT_SHIFT) & 
                   DIG_FLT_MASK) >> DIG_FLT_SHIFT;
  dig_flt_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getDigitalFiltering(uint8_t& dig_flt)
{
  uint16_t reg_val;
  uint8_t status = readRegister(DIG_FLT_REG, reg_val);
  this->dig_flt = (reg_val & DIG_FLT_MASK) >> DIG_FLT_SHIFT;
  dig_flt_dirty = 0;
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
  this->res_x = res_x & 0x3;
  this->res_y = res_y & 0x3;
  this->res_z = res_z & 0x3;
  res_xyz_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getResolution(uint8_t& res_x, uint8_t& res_y, uint8_t& res_z)
{
  uint16_t reg_val;
  uint8_t status = readRegister(RES_XYZ_REG, reg_val);
  uint8_t res_xyz = (reg_val & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
  this->res_x = res_x = (res_xyz >> 0) & 0x3;
  this->res_y = res_y = (res_xyz >> 2) & 0x3;
  this->res_z = res_z = (res_xyz >> 4) & 0x3;
  res_xyz_dirty = 0;
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
  this->tcmp_en = tcmp_en;
  tcmp_en_dirty = 0;
  return checkStatus(status1) | checkStatus(status2);
}

uint8_t
MLX90393::
getTemperatureCompensation(uint8_t& enabled)
{
  uint16_t reg_val;
  uint8_t status = readRegister(TCMP_EN_REG, reg_val);
  this->tcmp_en = enabled = (reg_val & TCMP_EN_MASK) >> TCMP_EN_SHIFT;
  tcmp_en_dirty = 0;
  return checkStatus(status);
}

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
