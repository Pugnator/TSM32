#include "tsm.h"
#include "mpu.h"
#include "eeprom.h"
#include <algorithm>

bool MPU9250::magReset()
{
  bool result = magWrite(AK8963_CNTL2, 0b0000001);
  if (!result)
  {
    return false;
  }
  HAL_Delay(10);
  return true;
}

bool MPU9250::magSetMode(magMode mode)
{
  bool result = false;
  switch (mode)
  {
  case MAG_MODE_PD:
    result = magWrite(AK8963_CNTL1, 0);
    break;
  case MAG_MODE_FUSE_ROM:
    result = magWrite(AK8963_CNTL1, 0b00001111);
    break;
  case MAG_MODE_SINGLE:
    result = magWrite(AK8963_CNTL1, 0b00010001);
    break;
  case MAG_MODE_CONT_8HZ:
    result = magWrite(AK8963_CNTL1, 0b00010010);
    break;
  case MAG_MODE_CONT_100HZ:
    result = magWrite(AK8963_CNTL1, 0b00010110); // 100Hz 16bit
    break;
  case MAG_MODE_SELF_TEST:
    result = magWrite(AK8963_CNTL1, 0b00011000);
    break;
  default:
    DEBUG_LOG("Wrong mode specified {%.2X}\r\n", mode);
    return false;
  }

  if (!result)
  {
    return false;
  }
  HAL_Delay(10);
  return true;
}

bool MPU9250::configureMagnetometer()
{
#if defined(IMU_I2C_MODE)
  if (HAL_OK != HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR_MAG, 10, 100))
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("MAG is not online\r\n");
    return false;
  }
#endif

  magReset();

  uint8_t wai = 0;
  magRead(AK8963_WIA, &wai);
  if (wai != 0x48)
  {
    return false;
  }
  DEBUG_LOG("AK8963 is detected\r\n");

  magSetMode(MAG_MODE_PD);
  // Entering FUSE rom read state
  if (!magSetMode(MAG_MODE_FUSE_ROM))
  {
    return false;
  }
  HAL_Delay(10);
  uint8_t fuseRom[3] = {0};
  if (!magRead(AK8963_ASAX, &fuseRom[0], 3))
  {
    return false;
  }
  magFactoryCorrX = (((fuseRom[0] - 128.0f) * 0.5f) / 128.0f) + 1.0f;
  magFactoryCorrY = (((fuseRom[1] - 128.0f) * 0.5f) / 128.0f) + 1.0f;
  magFactoryCorrZ = (((fuseRom[2] - 128.0f) * 0.5f) / 128.0f) + 1.0f;

  DEBUG_LOG("Factory corrections:\r\nX=%.4f\r\nY=%.4f\r\nZ=%.4f\r\n", magFactoryCorrX, magFactoryCorrY, magFactoryCorrZ);

  // Entering Continous mode, 100Hz
  magSetMode(MAG_MODE_PD);
  if (!magSetMode(MAG_MODE_CONT_100HZ))
  {
    return false;
  }

  DEBUG_LOG("Mag is ready\r\n");
  return true;
}

void MPU9250::magAutoOffset(Axes3D &axes)
{
  if (!isCalibration_)
  {
    return;
  }
  // Update magMaxX, magMaxY, magMaxZ with the maximum values
  if (axes.x > magMaxX)
  {
    magMaxX = axes.x;
  }
  if (axes.y > magMaxY)
  {
    magMaxY = axes.y;
  }
  if (axes.z > magMaxZ)
  {
    magMaxZ = axes.z;
  }

  // Update magMinX, magMinY, magMinZ with the minimum values
  if (axes.x < magMinX)
  {
    magMinX = axes.x;
  }
  if (axes.y < magMinY)
  {
    magMinY = axes.y;
  }
  if (axes.z < magMinZ)
  {
    magMinZ = axes.z;
  }

  magOffsetX = (magMaxX + magMinX) / 2.0;
  magOffsetY = (magMaxY + magMinY) / 2.0;
  magOffsetZ = (magMaxZ + magMinZ) / 2.0;
}

bool MPU9250::readMagAxis(Axes3D &result)
{
  uint8_t data[7];
  uint8_t state = 0;
  do
  {
    magRead(AK8963_ST1, &state);
  } while (!(state & 0x01));

  if(!magRead(AK8963_HXL, &data[0], 7))
    return false;
  // Check if overflow occurred

  float Mx_Raw = ((int16_t)((int16_t)data[1] << 8) | data[0]);
  float My_Raw = ((int16_t)((int16_t)data[3] << 8) | data[2]);
  float Mz_Raw = ((int16_t)((int16_t)data[5] << 8) | data[4]);

  if (data[6] & (1 << 3))
  {
    DEBUG_LOG("Magnetic sensor overflow occurred\r\n");
  }

  /* Calculate uG value for XYZ axis */

  static const float mRes = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss

  result.x = Mx_Raw * magFactoryCorrX * mRes;
  result.y = My_Raw * magFactoryCorrY * mRes;
  result.z = Mz_Raw * magFactoryCorrZ * mRes;

  magAutoOffset(result);

  // magOffsetX = 231.4684;
  // magOffsetY = 89.9165;
  // magOffsetZ = -130.869;

  result.x -= magOffsetX;
  result.y -= magOffsetY;
  result.z -= magOffsetZ;

  // mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];
  //  my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
  //  mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];

  return true;
}

float MPU9250::getHeadingAngle()
{
  Axes3D ax;
  readMagAxis(ax);
  float Xf = ax.x;
  float Yf = ax.y;
  float Zf = ax.z;

  // Euclidean normalization
  float norm = sqrtf(Xf * Xf + Yf * Yf + Zf * Zf);
  if (!norm)
  {
    return -1;
  }
  norm = 1.0f / norm;
  Xf *= norm;
  Yf *= norm;
  Zf *= norm;

  float az = 0.0;
  if (Yf > 0)
  {
    az = 90.0f - (atan(Xf / Yf)) * (180.0f / M_PI);
  }
  else if (Yf < 0)
  {
    az = 270.0f - (atan(Xf / Yf)) * (180.0f / M_PI);
  }
  else if (Yf == 0)
  {
    az = Xf < 0 ? 180.0f : 0;
  }

  az += MAGNETIC_DECLINATION;
  if (az > 360.0f)
  {
    az -= 360.0f;
  }

  return az;
}
