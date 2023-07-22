#include "tsm.h"
#include "imu_spi.h"
#include "mpudefs.h"
#include "crc.h"

namespace Mpu9250
{
  Mpu9250Spi::Mpu9250Spi(SPI_HandleTypeDef *interface, bool useDmp, MagMode magMode)
  {
    mpuDeselect();
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

    if (useDmp_)
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
    }
  }

  bool Mpu9250Spi::setup()
  {
    // enable SPI mode
    uint8_t temp_ = 0;
    if (!mpuRead(MPU9250_USER_CTRL, &temp_))
      return false;

    if (!mpuWrite(MPU9250_USER_CTRL, temp_ | 0x10))
      return false;

    if (!mpuRead(MPU9250_USER_CTRL, &temp_))
      return false;

    if (!mpuWrite(MPU9250_USER_CTRL, temp_ | 0x20))
      return false;

    // Clear sleep mode bit (6), enable all sensors
    if (!mpuWrite(MPU9250_PWR_MGMT_1, 0x00))
    {
      return false;
    }
    HAL_Delay(100); // Wait for all registers to reset

    // get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    if (!mpuWrite(MPU9250_PWR_MGMT_1, 0x01))
    {
      return false;
    }

    HAL_Delay(200);

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
    return true;
  }
}