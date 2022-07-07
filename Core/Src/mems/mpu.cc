
#include "mpu.h"

MPU9250::MPU9250(I2C_HandleTypeDef *dev)
{
  i2c = dev;
  _ok = false;
  aMult = 0;
  gMult = 0;
  mMult = 0;
  reset();

  _ok = initacc();
  _ok = initmag();
  /* Calculate multiplicators */
  aMult = 2.0f / 32768.0f;
  gMult = 250.0f / 32768.0f;
  mMult = 10.0f * 4912.0f / 32768.0f;
  aKalman.reset(new kalmanFilter);
  gKalman.reset(new kalmanFilter);
  mKalman.reset(new kalmanFilter);
}

MPU9250::~MPU9250()
{
}

bool MPU9250::ok()
{
  return _ok;
}

void MPU9250::reset()
{
  writeRegMpu(MPU9250_USER_CTRL, 0); // disable internal I2C bus
  // reset device
  writeRegMpu(MPU9250_PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
  writeRegMpu(MPU9250_USER_CTRL, 0x20);  // re-enable internal I2C bus
  HAL_Delay(100);                  // Wait for all registers to reset
}

bool MPU9250::writeRegMpu(uint8_t reg, uint8_t byte)
{
  uint8_t temp = byte;
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, reg, 1, &temp, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }
  return true;
}

bool MPU9250::write(uint8_t reg, uint8_t *byte, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, reg, 1, &byte[i], 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(i2c, 1000);
      return false;
    }
  }

  return true;
}

bool MPU9250::readRegMpu(uint8_t reg, uint8_t *byte)
{
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, reg, 1, byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }
  return true;
}

bool MPU9250::read(uint8_t reg, uint8_t *byte, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, reg, 1, &byte[i], 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(i2c, 1000);
      return false;
    }
  }
  return true;
}

bool MPU9250::writeRegMag(uint8_t reg, uint8_t byte)
{
  uint8_t temp = byte;
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, &temp, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    PrintF("Failed to write to MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::readm(uint8_t reg, uint8_t *byte)
{
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    PrintF("Failed to read MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::readRegMag(uint8_t reg, uint8_t *byte, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(i2c, 1000);
      PrintF("Failed to read MAG\r\n");
      return false;
    }
  }
  return true;
}

bool MPU9250::initacc()
{
  if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR, 10, 100))
  {
    PrintF("Accelerometer is online\r\n");
  }
  else
  {
    PrintF("Accelerometer is not online\r\n");
    return false;
  }
  uint8_t wai = 0;
  if (!readRegMpu(MPU9250_WHO_AM_I, &wai))
  {
    return false;
  }
  if (0x71 == wai)
  {
    PrintF("MPU 9250 is detected\r\n");
  }
  else
  {
    return false;
  }

  // Clear sleep mode bit (6), enable all sensors
  if (!writeRegMpu(MPU9250_PWR_MGMT_1, 0x00))
  {
    return false;
  }
  HAL_Delay(100); // Wait for all registers to reset

  // get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  if (!writeRegMpu(MPU9250_PWR_MGMT_1, 0x01))
  {
    return false;
  }
  HAL_Delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeRegMpu(MPU9250_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeRegMpu(MPU9250_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                                   // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t data = 0;
  readRegMpu(MPU9250_GYRO_CONFIG, &data); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  data &= ~0x02;     // Clear Fchoice bits [1:0]
  data &= ~0x18;     // Clear AFS bits [4:3]
  data |= 0x00 << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeRegMpu(MPU9250_GYRO_CONFIG, data); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  readRegMpu(MPU9250_ACCEL_CONFIG, &data); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  data &= ~0x18;                     // Clear AFS bits [4:3]
  data |= 0x00 << 3;                 // Set full scale range for the accelerometer
  writeRegMpu(MPU9250_ACCEL_CONFIG, data); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  readRegMpu(MPU9250_ACCEL_CONFIG_2, &data); // get current ACCEL_CONFIG2 register value
  data &= ~0x0F;                       // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  data |= 0x03;                        // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeRegMpu(MPU9250_ACCEL_CONFIG_2, data); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeRegMpu(MPU9250_INT_PIN_CFG, 0x22);
  writeRegMpu(MPU9250_INT_ENABLE, 0x01);
  PrintF("MPU is up\r\n");
  return true;
}

bool MPU9250::initmag()
{
  /* Check if device connected */
  if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR_MAG, 10, 100))
  {
    PrintF("MAG is online\r\n");
  }
  else
  {
    PrintF("MAG is not online\r\n");
    return false;
  }

  writeRegMpu(MPU9250_USER_CTRL, 0);
  writeRegMpu(MPU9250_INT_PIN_CFG, 0x02);

  // Case 2: Who am i test
  uint8_t wai = 0;
  if (!readm(AK9863_WIA, &wai))
  {
    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return false;
  }

  if (wai != 0x48)
  {
    return false;
  }

  PrintF("AK9863 is detected\r\n");

  writeRegMag(AK9863_CNTL1, 0x1F); // Power down magnetometer
  HAL_Delay(100);
  uint8_t temp[3] = {0};
  readRegMag(AK9863_ASAX | 0x80, temp, 3); // Enter Fuse ROM access mode
  HAL_Delay(100);
  writeRegMag(AK9863_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(100);
  writeRegMag(AK9863_CNTL1, 0x16); // Power down magnetometer  
  HAL_Delay(100);

  DEBUG_LOG("Started magnetometer calibration\r\n");
  float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;

  /* Hard Iron effect compensation */
  for (size_t i = 0; i < 1000; ++i)
  {
    auto axes = readMag();

    if (axes.x > X_max)
      X_max = axes.x;
    if (axes.y > Y_max)
      Y_max = axes.y;
    if (axes.z > Z_max)
      Z_max = axes.z;

    if (axes.x < X_min)
      X_min = axes.x;
    if (axes.y < Y_min)
      Y_min = axes.y;
    if (axes.z < Z_min)
      Z_min = axes.z;

    HAL_Delay(20);
  }

  float X_offset = (X_max + X_min) / 2;
  float Y_offset = (Y_max + Y_min) / 2;
  float Z_offset = (Z_max + Z_min) / 2;

  /* Soft Iron effect compensation */
  // float delta_x = (X_max - X_min) / 2;
  // float delta_y = (Y_max - Y_min) / 2;
  // float delta_z = (Z_max - Z_min) / 2;

  // float delta = (delta_x + delta_y + delta_z) / 3;

  // float mMultx = delta / delta_x;
  // float mMulty = delta / delta_y;
  // float mMultz = delta / delta_z;
  DEBUG_LOG("MAG calibration finished. Offsets [X %f, Y %f, Z %f]\r\n", X_offset, Y_offset, Z_offset);

  return true;
}

bool MPU9250::ready()
{
  uint8_t data = 0;
  if (!readRegMpu(MPU9250_INT_STATUS, &data))
  {
    return false;
  }

  if (data & 0x01)
  {
    return true;
  }
  return false;
}

void MPU9250::printall()
{
  while (!ready())
    ;
  readA();
  readG();
  readMag();
}

bool MPU9250::readA()
{
  uint8_t data[6];
  /* Read accelerometer data */
  read(MPU9250_ACCEL_XOUT_H, data, 6);

  float Ax_Raw = ((int16_t)data[0] << 8) | data[1];
  float Ay_Raw = ((int16_t)data[2] << 8) | data[3];
  float Az_Raw = ((int16_t)data[4] << 8) | data[5];

  float Ax = (float)Ax_Raw * aMult;
  float Ay = (float)Ay_Raw * aMult;
  float Az = (float)Az_Raw * aMult;
  PrintF("Acc %.4f, %.4f, %.4f\r\n", Ax, Ay, Az);
  return true;
}

axes MPU9250::readMag()
{
  uint8_t data[8];

  /* Check status */
  readRegMag(AK9863_ST1, data, 8);
  axes result = {0};
  if (data[0] & 0x01)
  {
    if (!(data[7] & 0x08))
    {
      float Mx_Raw = ((int16_t)data[2] << 8) | data[1];
      float My_Raw = ((int16_t)data[4] << 8) | data[3];
      float Mz_Raw = ((int16_t)data[6] << 8) | data[5];

      result.x = (float)Mx_Raw * mMult;
      result.y = (float)Mz_Raw * mMult;
      result.z = (float)My_Raw * mMult;
      PrintF("Magnetometer %.4f, %.4f, %.4f\r\n", result.x, result.y, result.z);
      return result;
    }
  }

  PrintF("MAG is not ready\r\n");
  return result;
}

bool MPU9250::readG()
{
  uint8_t data[6];

  read(MPU9250_GYRO_XOUT_H, data, 6);

  float Gx_Raw = ((int16_t)data[0] << 8) | data[1];
  float Gy_Raw = ((int16_t)data[2] << 8) | data[3];
  float Gz_Raw = ((int16_t)data[4] << 8) | data[5];

  float Gx = Gx_Raw * gMult;
  float Gy = Gy_Raw * gMult;
  float Gz = Gz_Raw * gMult;
  PrintF("Gyro %.4f, %.4f, %.4f\r\n", Gx, Gy, Gz);
  return true;
}

// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU9250::selfTest()
{
  DEBUG_LOG("Started self test\r\n");
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeRegMpu(MPU9250_SMPLRT_DIV, 0);
  writeRegMpu(MPU9250_CONFIG, 0x02);
  writeRegMpu(MPU9250_GYRO_CONFIG, FS << 3);
  writeRegMpu(MPU9250_ACCEL_CONFIG_2, 0x02);
  writeRegMpu(MPU9250_ACCEL_CONFIG, FS << 3);

  for (int ii = 0; ii < 200; ii++)
  { // get average current values of gyro and acclerometer
    // Read the six raw data registers into data array
    write(MPU9250_ACCEL_XOUT_H, rawData, 6);

    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    // Read the six raw data registers sequentially into data array
    read(MPU9250_GYRO_XOUT_H, rawData, 6);

    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    HAL_Delay(1);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test

  HAL_Delay(25); // osDelay a while to let the device stabilize
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeRegMpu(MPU9250_ACCEL_CONFIG, 0xE0);

  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

  writeRegMpu(MPU9250_GYRO_CONFIG, 0xE0);

  for (int ii = 0; ii < 200; ii++)
  { // get average self-test values of gyro and acclerometer

    // Read the six raw data registers into data array
    read(MPU9250_ACCEL_XOUT_H, rawData, 6);

    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    // Read the six raw data registers sequentially into data array
    read(MPU9250_GYRO_XOUT_H, rawData, 6);

    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    HAL_Delay(1);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation

  writeRegMpu(MPU9250_ACCEL_CONFIG, 0);

  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeRegMpu(MPU9250_GYRO_CONFIG, 0);
  HAL_Delay(25); // osDelay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  readRegMpu(MPU9250_SELF_TEST_X_ACCEL, &selfTest[0]);
  readRegMpu(MPU9250_SELF_TEST_Y_ACCEL, &selfTest[1]);
  readRegMpu(MPU9250_SELF_TEST_Z_ACCEL, &selfTest[2]);
  readRegMpu(MPU9250_SELF_TEST_X_GYRO, &selfTest[3]);
  readRegMpu(MPU9250_SELF_TEST_Y_GYRO, &selfTest[4]);
  readRegMpu(MPU9250_SELF_TEST_Z_GYRO, &selfTest[5]);

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  float result[6] = {0};
  for (int i = 0; i < 3; i++)
  {
    result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;         // Report percent differences
    result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
  for (int i = 0; i < 6; i++)
  {
    DEBUG_LOG("%.4f ", result[i]);
  }
  DEBUG_LOG("\r\nSelf test completed\r\n");
}

float MPU9250::getAzimuth()
{
  auto ax = readMag();
  float Xf = ax.x;
  float Yf = ax.y;
  float Zf = ax.z;
  float norm = sqrtf(Xf * Xf + Yf * Yf + Zf * Zf);
  if (norm == 0.0f)
  {
  }
  norm = 1.0f / norm;
  Xf *= norm;
  Yf *= norm;
  Zf *= norm;

  // DEBUG_LOG("X=%f Y=%f Z=%f\r\n", Xf, Yf, Zf);
  float az = 0.0;
  if (Yf > 0)
  {
    az = 90.0 - (atan(Xf / Yf)) * (180 / M_PI);
  }
  else if (Yf < 0)
  {
    az = 270.0 - (atan(Xf / Yf)) * (180 / M_PI);
  }
  else if (Yf == 0)
  {
    az = Xf < 0 ? 180.0 : 0;
  }

  az += MAGNETIC_DECLINATION;
  if (az > 360.0)
  {
    az -= 360;
  }
  DEBUG_LOG("AZ=%f\r\n", az);
  return az;
}

kalmanFilter::kalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float kalmanFilter::updateEstimate(float mea)
{
  float _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}