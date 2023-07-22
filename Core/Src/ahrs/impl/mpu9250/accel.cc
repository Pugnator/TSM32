#include "mpu9250_base.h"
#include "types.h"
#include <algorithm>

namespace Mpu9250
{
  bool Mpu9250base::configureAccelerometer()
  {
    uint8_t val = 0;
    uint8_t *temp_ = &val;
    // Set accelerometer full-scale range configuration
    if (!mpuRead(MPU9250_ACCEL_CONFIG, temp_)) // get current ACCEL_CONFIG register value
      return false;
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    *temp_ &= ~0x18;                                        // Clear AFS bits [4:3]
    *temp_ |= static_cast<int>(AHRS::Ascale::Scale2G) << 3; // Set full scale range for the accelerometer

    if (!mpuWrite(MPU9250_ACCEL_CONFIG, std::move(*temp_))) // Write new ACCEL_CONFIG register value
      return false;

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    if (!mpuRead(MPU9250_ACCEL_CONFIG_2, temp_)) // get current ACCEL_CONFIG2 register value
      return false;
    *temp_ &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    *temp_ |= 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

    if (!mpuWrite(MPU9250_ACCEL_CONFIG_2, std::move(*temp_))) // Write new ACCEL_CONFIG2 register value
      return false;
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled as master
    // 00000010
    accelCurrentBias();
    return true;
  }

  bool Mpu9250base::readAccelAxis(VectorFloat &result)
  {
    uint8_t data[8];
    if (!mpuRead(MPU9250_ACCEL_XOUT_H, data, 6))
      return false;

    int16_t accX = ((int16_t)data[0] << 8) | data[1];
    int16_t accY = ((int16_t)data[2] << 8) | data[3];
    int16_t accZ = ((int16_t)data[4] << 8) | data[5];

    uint16_t _temp = (data[6] << 8 | data[7]);
    chipTemperature_ = ((_temp - 21.0f) / 333.87f) + 21.0f;

    result.x = (float)accX * aMult;
    result.y = (float)accY * aMult;
    result.z = (float)accZ * aMult;
    return true;
  }

  bool Mpu9250base::accelCurrentBias()
  {
    DEBUG_LOG("Accelerometer offset cancelation\r\n");
    uint32_t mask = 1;
    uint8_t originalBias[6];
    uint8_t maskBit[3];

    // Get factory offsets
    if (!mpuRead(MPU9250_XA_OFFSET_H, originalBias, 2))
      return false;
    if (!mpuRead(MPU9250_YA_OFFSET_H, &originalBias[2], 2))
      return false;
    if (!mpuRead(MPU9250_ZA_OFFSET_H, &originalBias[4], 2))
      return false;

    // temperature compensation flag
    for (uint32_t i = 0; i < 3; i++)
    {
      if (originalBias[i] & mask)
        maskBit[i] = 1;
    }

    int32_t xOffset = 0;
    int32_t yOffset = 0;
    int32_t zOffset = 0;

    int16_t originalOffsetX = ((int16_t)originalBias[0] << 8) | originalBias[1];
    int16_t originalOffsetY = ((int16_t)originalBias[2] << 8) | originalBias[3];
    int16_t originalOffsetZ = ((int16_t)originalBias[4] << 8) | originalBias[5];

    const int32_t averageCount = 1000;
    uint8_t data[6];

    for (uint32_t i = 0; i < averageCount; i++)
    {
      if (!mpuRead(MPU9250_ACCEL_XOUT_H, data, 6))
        return false;

      int16_t axRaw = ((int16_t)data[0] << 8) | data[1];
      xOffset += axRaw;
      int16_t ayRaw = ((int16_t)data[2] << 8) | data[3];
      yOffset += ayRaw;
      int16_t azRaw = ((int16_t)data[4] << 8) | data[5];
      zOffset += azRaw;
    }

    xOffset /= averageCount;
    yOffset /= averageCount;
    zOffset /= averageCount;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (zOffset > 0)
      zOffset -= 16384;
    else
      zOffset += 16384;

    DEBUG_LOG("Calculated X offset: %d\r\n", xOffset);
    DEBUG_LOG("Calculated Y offset: %d\r\n", yOffset);
    DEBUG_LOG("Calculated Z offset: %d\r\n", zOffset);
    DEBUG_LOG("Factory X offset: %d\r\n", originalOffsetX);
    DEBUG_LOG("Factory Y offset: %d\r\n", originalOffsetY);
    DEBUG_LOG("Factory Z offset: %d\r\n", originalOffsetZ);

    originalOffsetX -= xOffset >> 3;
    originalOffsetY -= yOffset >> 3;
    originalOffsetZ -= zOffset >> 3;

    DEBUG_LOG("Corrected X offset: %d\r\n", originalOffsetX);
    DEBUG_LOG("Corrected Y offset: %d\r\n", originalOffsetY);
    DEBUG_LOG("Corrected Z offset: %d\r\n", originalOffsetZ);

    uint8_t resultedData[6] = {0};

    resultedData[0] = (originalOffsetX >> 8) & 0xff;
    resultedData[1] = originalOffsetX & 0xff;
    if (maskBit[0])
    {
      resultedData[0] &= ~maskBit[0];
    }
    else
    {
      resultedData[0] |= 0x0001;
    }

    resultedData[2] = (originalOffsetY >> 8) & 0xff;
    resultedData[3] = originalOffsetY & 0xff;
    if (maskBit[2])
    {
      resultedData[3] &= ~maskBit[1];
    }
    else
    {
      resultedData[3] |= 0x0001;
    }

    resultedData[4] = (originalOffsetZ >> 8) & 0xff;
    resultedData[5] = originalOffsetZ & 0xff;
    if (maskBit[3])
    {
      resultedData[5] &= ~maskBit[2];
    }
    else
    {
      resultedData[5] |= 0x0001;
    }

    if (!mpuWrite(MPU9250_XA_OFFSET_H, resultedData, 2))
      return false;

    if (!mpuWrite(MPU9250_YA_OFFSET_H, &resultedData[2], 2))
      return false;

    if (!mpuWrite(MPU9250_ZA_OFFSET_H, &resultedData[4], 2))
      return false;

    return true;
  }
}