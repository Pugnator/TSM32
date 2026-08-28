#include "mpu9250_base.h"
#include "types.h"
#include "axis_remap.h"
#include <algorithm>

namespace Mpu9250
{
  bool Mpu9250base::configureGyroscope()
  {
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz

    if (!mpuWrite(MPU9250_CONFIG, 0x03))
      return false;

    HAL_Delay(6);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above
    if (!mpuWrite(MPU9250_SMPLRT_DIV, 0x04))
      return false;

    uint8_t val = 0;
    uint8_t *temp_ = &val;
    // get current GYRO_CONFIG register value
    if (!mpuRead(MPU9250_GYRO_CONFIG, temp_))
      return false;

    *temp_ &= ~0xE0; // Clear self-test bits [7:5]
    /* Clear BOTH Fchoice_b bits [1:0] so Fchoice=11 and the gyro DLPF is
     * enabled.  The old mask ~0x02 left bit 0 untouched: harmless from a
     * cold boot (GYRO_CONFIG reads 0x00) but on a warm re-init it could
     * leave Fchoice_b=01, bypassing the DLPF and feeding 8.8 kHz-bandwidth
     * gyro noise into the fusion filter. */
    *temp_ &= ~0x03;     // Clear Fchoice_b bits [1:0] -> DLPF enabled
    *temp_ &= ~0x18;     // Clear GYRO_FS_SEL bits [4:3]
    *temp_ |= 0x00 << 3; // GYRO_FS_SEL = 00: full-scale range 250 DPS

    if (!mpuWrite(MPU9250_GYRO_CONFIG, *temp_))
      return false;

    gyroCurrentBias();

    return true;
  }

  bool Mpu9250base::readGyroAxis(VectorFloat &result)
  {
    uint8_t data[6];
    if (!mpuRead(MPU9250_GYRO_XOUT_H, data, 6))
      return false;

    int16_t gyroX = ((int16_t)data[0] << 8) | data[1];
    int16_t gyroY = ((int16_t)data[2] << 8) | data[3];
    int16_t gyroZ = ((int16_t)data[4] << 8) | data[5];

    result = Ahrs::sensorToBodyZ180((float)gyroX, (float)gyroY,
                                    (float)gyroZ, gMult);
    return true;
  }

  bool Mpu9250base::gyroCurrentBias()
  {
    DEBUG_LOG("Gyro offset cancelation\r\n");
    uint8_t data[6];

    int32_t xOffset = 0;
    int32_t yOffset = 0;
    int32_t zOffset = 0;
    
    const int32_t averageCount = 1000;

    for (uint32_t i = 0; i < averageCount; i++)
    {
      if (!mpuRead(MPU9250_GYRO_XOUT_H, data, 6))
        return false;

      int16_t gxRaw = ((int16_t)data[0] << 8) | data[1];
      xOffset += gxRaw;
      int16_t gyRaw = ((int16_t)data[2] << 8) | data[3];
      yOffset += gyRaw;
      int16_t gzRaw = ((int16_t)data[4] << 8) | data[5];
      zOffset += gzRaw;
    }
    xOffset /= averageCount;
    yOffset /= averageCount;
    zOffset /= averageCount;

    DEBUG_LOG("Calculated X offset: %d\r\n", xOffset);
    DEBUG_LOG("Calculated Y offset: %d\r\n", yOffset);
    DEBUG_LOG("Calculated Z offset: %d\r\n", zOffset);

    xOffset = -xOffset;
    yOffset = -yOffset;
    zOffset = -zOffset;

    uint8_t biasData[6] = {0};

    biasData[0] = (xOffset / 4 >> 8) & 0xff;
    biasData[1] = xOffset / 4 & 0xff;
    biasData[2] = (yOffset / 4 >> 8) & 0xff;
    biasData[3] = yOffset / 4 & 0xff;
    biasData[4] = (zOffset / 4 >> 8) & 0xff;
    biasData[5] = zOffset / 4 & 0xff;

    if (!mpuWrite(MPU9250_XG_OFFSET_H, &biasData[0], 2))
      return false;

    if (!mpuWrite(MPU9250_YG_OFFSET_H, &biasData[2], 2))
      return false;

    if (!mpuWrite(MPU9250_ZG_OFFSET_H, &biasData[4], 2))
      return false;

    return true;
  }
}
