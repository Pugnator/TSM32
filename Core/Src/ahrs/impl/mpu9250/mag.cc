#include "tsm.h"
#include "mpu9250_base.h"
#include <algorithm>

namespace Mpu9250
{
  bool Mpu9250base::magReset()
  {
    if (!magWrite(AK8963_CNTL2, 0b0000001))
      return false;

    HAL_Delay(10);
    return true;
  }

  bool Mpu9250base::magSetMode(magMode mode)
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

  bool Mpu9250base::configureMagnetometer()
  {
    if (!magReset())
      return false;

    if (!magWrite(AK8963_CNTL1, 0x0F))
      return false;

    uint8_t wai = 0;
    magRead(AK8963_WIA, &wai);
    if (wai != 0x48)
      return false;

    DEBUG_LOG("AK8963 is detected [%.2X]\r\n", wai);

    if (!magSetMode(MAG_MODE_PD))
      return false;
    // Entering FUSE rom read state
    if (!magSetMode(MAG_MODE_FUSE_ROM))
      return false;

    HAL_Delay(10);
    uint8_t fuseRom[3] = {0};
    if (!magRead(AK8963_ASAX, &fuseRom[0], 3))
      return false;

    magFactoryCorrX = (((fuseRom[0] - 128.0f) * 0.5f) / 128.0f) + 1.0f;
    magFactoryCorrY = (((fuseRom[1] - 128.0f) * 0.5f) / 128.0f) + 1.0f;
    magFactoryCorrZ = (((fuseRom[2] - 128.0f) * 0.5f) / 128.0f) + 1.0f;

    DEBUG_LOG("Factory corrections:\r\nX=%.4f\r\nY=%.4f\r\nZ=%.4f\r\n", magFactoryCorrX, magFactoryCorrY, magFactoryCorrZ);

    if (!magSetMode(MAG_MODE_PD))
      return false;

    if (!magSetMode(MAG_MODE_CONT_8HZ))
      return false;

    DEBUG_LOG("Mag is ready\r\n");
    return true;
  }

  bool Mpu9250base::readMagAxis(VectorFloat &result, bool blocking)
  {
    const uint32_t readTimeout = 100;
    uint8_t data[7];
    uint8_t state = 0;
    uint32_t beginTime = HAL_GetTick();
    for (;;)
    {
      if (!magRead(AK8963_ST1, &state))
        return false;

      if (!(state & 0x01))
      {
        if (blocking && HAL_GetTick() - beginTime < readTimeout)
          continue;

        return false;
      }
      break;
    }

    if (!magRead(AK8963_HXL, &data[0], 7))
      return false;
    // Check if overflow occurred

    float magX = ((int16_t)((int16_t)data[1] << 8) | data[0]);
    float magY = ((int16_t)((int16_t)data[3] << 8) | data[2]);
    float magZ = ((int16_t)((int16_t)data[5] << 8) | data[4]);

    if (data[6] & (1 << 3))
    {
      DEBUG_LOG("Magnetic sensor overflow occurred\r\n");
      return false;
    }

    /* Calculate uG value for XYZ axis */

    static const float mRes = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss

    result.y = magX * magFactoryCorrX * mRes;
    result.x = magY * magFactoryCorrY * mRes;
    result.z = magZ * magFactoryCorrZ * mRes;
    return true;
  }
}