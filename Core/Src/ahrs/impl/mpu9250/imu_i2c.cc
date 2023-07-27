#include "tsm.h"
#include "imu_i2c.h"
#include "mpudefs.h"

namespace Mpu9250
{
  Mpu9250I2c::Mpu9250I2c(I2C_HandleTypeDef *interface, bool useDmp, MagMode magMode)
  {
    bus_ = interface;
    magMode_ = magMode;
    useDmp_ = useDmp;
    chipTemperature_ = 0;
    magFactoryCorrX = 0;
    magFactoryCorrY = 0;
    magFactoryCorrZ = 0;
    magScaleX = 1;
    magScaleY = 1;
    magScaleZ = 1;

    /* Calculate multiplicators */
    aMult = 2.0f / 32768.0f;
    gMult = 250.0f / 32768.0f;
    gSensF = 10.0f * 4912.0f / 32768.0f;

    ok_ = setup();

    if (!ok_)
    {
      DEBUG_LOG("Failed to setup an IMU.\r\n");
      return;
    }

    if (useDmp)
    {
      DEBUG_LOG("Configuring Digital Motion Processor.\r\n");
      ok_ = startDMP();
      return;
    }

    DEBUG_LOG("Configuring raw mode.\r\n");

    if (!configureGyroscope())
    {
      DEBUG_LOG("I2C: Failed to configure gyroscope.\r\n");
      ok_ = false;
      return;
    }

    DEBUG_LOG("Gyro OK.\r\n");

    if (ok_ && !configureAccelerometer())
    {
      DEBUG_LOG("I2C: Failed to configure accelerometer.\r\n");
      ok_ = false;
      return;
    }

    DEBUG_LOG("Accel OK.\r\n");

    if (magMode_ == MagMode::SlaveMode)
      ok_ = setMagnetometerAsSlave();

    if (ok_ && !configureMagnetometer())
    {
      DEBUG_LOG("I2C: Failed to configure magnetometer.\r\n");
      ok_ = false;
      return;
    }

    DEBUG_LOG("Mag OK.\r\n");
  }

  bool Mpu9250I2c::setup()
  {
    if (HAL_OK != HAL_I2C_IsDeviceReady(bus_, MPU9250_I2C_ADDR, 10, 100))
    {
      DEBUG_LOG("IMU is not online\r\n");
      return false;
    }

    DEBUG_LOG("IMU is online\r\n");

    uint8_t wai = 0;
    if (!mpuRead(MPU9250_WHO_AM_I, &wai))
      return false;

    if (0x71 == wai)
    {
      DEBUG_LOG("MPU9250 is detected [%.2X]\r\n", wai);
    }
    else
    {
      DEBUG_LOG("MPU9250 is NOT detected [%.2X]\r\n", wai);
      return false;
    }

    if (!mpuWrite(MPU9250_PWR_MGMT_1, 0))
      return false;

    HAL_Delay(100);

    if (!mpuWrite(MPU9250_PWR_MGMT_1, 0x01))
      return false;

    HAL_Delay(200);

    if (!mpuWrite(MPU9250_CONFIG, 0x03))
      return false;

    if (!useDmp_)
    {
      // 0b00100010
      if (!mpuWrite(MPU9250_INT_PIN_CFG, 0x22))
        return false;
    }

    if (!mpuWrite(MPU9250_INT_ENABLE, 0x01))
      return false;

    HAL_Delay(200);
    return true;
  }
}