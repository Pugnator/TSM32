#include "mpu.h"
#include "MEMS/types.h"
#include <algorithm>

bool MPU9250::configureGyroscope()
{

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz

  if (!mpuWrite(MPU9250_CONFIG, 0x03))
  {
    return false;
  }

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above
  if (!mpuWrite(MPU9250_SMPLRT_DIV, 0x04))
  {
    return false;
  }

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t val = 0;
  uint8_t *temp_ = &val;
  // get current GYRO_CONFIG register value
  if (!mpuRead(MPU9250_GYRO_CONFIG, temp_))
  {
    return false;
  }
  *temp_ = *temp_ & ~0xE0; // Clear self-test bits [7:5]
  *temp_ &= ~0x02;         // Clear Fchoice bits [1:0]
  *temp_ &= ~0x18;         // Clear AFS bits [4:3]
  *temp_ |= 0x00 << 3;     // Set full scale range for the gyro (250DPS)
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  if (!mpuWrite(MPU9250_GYRO_CONFIG, std::move(*temp_))) // Write new GYRO_CONFIG value to register
  {
    return false;
  }
  return true;
}

void MPU9250::gyroAutoOffset(VectorFloat &axes)
{
  if (!isCalibration_)
  {
    return;
  }
  // Update gyroMaxX, gyroMaxY, gyroMaxZ with the maximum values
  if (axes.x > gyroMaxX)
  {
    gyroMaxX = axes.x;
  }
  if (axes.y > gyroMaxY)
  {
    gyroMaxY = axes.y;
  }
  if (axes.z > gyroMaxZ)
  {
    gyroMaxZ = axes.z;
  }

  // Update gyroMinX, gyroMinY, gyroMinZ with the minimum values
  if (axes.x < gyroMinX)
  {
    gyroMinX = axes.x;
  }
  if (axes.y < gyroMinY)
  {
    gyroMinY = axes.y;
  }
  if (axes.z < gyroMinZ)
  {
    gyroMinZ = axes.z;
  }

  gyroOffsetX = (gyroMaxX + gyroMinX) / 2.0;
  gyroOffsetY = (gyroMaxY + gyroMinY) / 2.0;
  gyroOffsetZ = (gyroMaxZ + gyroMinZ) / 2.0;
}

bool MPU9250::readGyroAxis(VectorFloat &result)
{
  uint8_t data[6];
  if (!mpuRead(MPU9250_GYRO_XOUT_H, data, 6))
  {
    return false;
  }

  float Gx_Raw = ((int16_t)data[0] << 8) | data[1];
  float Gy_Raw = ((int16_t)data[2] << 8) | data[3];
  float Gz_Raw = ((int16_t)data[4] << 8) | data[5];

  result.x = Gx_Raw * gMult;
  result.y = Gy_Raw * gMult;
  result.z = Gz_Raw * gMult;

  gyroAutoOffset(result);

  result.x -= gyroOffsetX;
  result.y -= gyroOffsetY;
  result.z -= gyroOffsetZ;
  // gAxes.x = Gx_Raw * gMult;
  // gAxes.y = Gy_Raw * gMult;
  // gAxes.z = Gz_Raw * gMult;
  return true;
}