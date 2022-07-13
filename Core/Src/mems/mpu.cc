
#include "mpu.h"

MPU9250::MPU9250(I2C_HandleTypeDef *dev)
{
  i2c = dev;
  _ok = false;
  aMult = 0;
  gMult = 0;
  mMult = 0;
  // reset();

  _ok = initAcc();
  _ok = initMag();
  
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
  HAL_Delay(100);                        // Wait for all registers to reset
}

bool MPU9250::writeRegMpu(uint8_t reg, uint8_t *byte, size_t len)
{

  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }

  return true;
}

bool MPU9250::writeRegMpu(uint8_t reg, uint8_t &&byte)
{
  uint8_t temp = byte;
  uint8_t *data = &temp;  

  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, reg, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }

  return true;
}

bool MPU9250::readRegMpu(uint8_t reg, uint8_t *byte, size_t len)
{

  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }

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
  readAccel();
  readGyro();
  readMag();
}

void MPU9250::scanBus()
{
  for (size_t i = 0; i < 0xFF; i++)
  {
    if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, i, 10, 100))
    {
      DEBUG_LOG("IIC device found at address %#.2X\r\n", i);      
    }
  }
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