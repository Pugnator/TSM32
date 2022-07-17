
#include "mpu.h"

MPU9250::MPU9250(I2C_HandleTypeDef *dev)
{
  i2c = dev;
  _ok = false;
  corrX = 0;
  corrY = 0;
  corrZ = 0;

  /* Calculate multiplicators */
  aMult = 2.0f / 32768.0f;
  gMult = 250.0f / 32768.0f;
  gSensF = 10.0f * 4912.0f / 32768.0f;

  _ok = initAcc();
  _ok = initMag();
  if (!_ok)
  {
    DEBUG_LOG("I2C bus issue.\r\nRebooting the system.\r\n");
    NVIC_SystemReset();
  }
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

float MPU9250::filter(float newVal)
{
  float dt = 0.15;
  float sigma_process = 3.0;
  float sigma_noise = 0.7;

  static float xk_1, vk_1, a, b;
  static float xk, vk, rk;
  static float xm;
  float lambda = (float)sigma_process * dt * dt / sigma_noise;
  float r = (4 + lambda - (float)sqrt(8 * lambda + lambda * lambda)) / 4;
  a = (float)1 - r * r;
  b = (float)2 * (2 - a) - 4 * (float)sqrt(1 - a);
  xm = newVal;
  xk = xk_1 + ((float) vk_1 * dt );
  vk = vk_1;
  rk = xm - xk;
  xk += (float)a * rk;
  vk += (float)( b * rk ) / dt;
  xk_1 = xk;
  vk_1 = vk;
  return xk_1;
}