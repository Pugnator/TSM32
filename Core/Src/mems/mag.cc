#include "mpu.h"

bool MPU9250::writeRegMag(uint8_t reg, uint8_t *byte, size_t len)
{
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::writeRegMag(uint8_t reg, uint8_t &&byte)
{
  uint8_t temp = byte;
  uint8_t *data = &temp;

  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::readRegMag(uint8_t reg, uint8_t *byte, size_t len)
{
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to read MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::magRST()
{
  bool result = writeRegMag(AK8963_CNTL2, 0b0000001);
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
    result = writeRegMag(AK8963_CNTL1, 0);
    break;
  case MAG_MODE_FUSE_ROM:
    result = writeRegMag(AK8963_CNTL1, 0b00001111);
    break;
  case MAG_MODE_SINGLE:
    result = writeRegMag(AK8963_CNTL1, 0b00010001);
    break;
  case MAG_MODE_CONT_8HZ:
    result = writeRegMag(AK8963_CNTL1, 0b00010010);
    break;
  case MAG_MODE_CONT_100HZ:
    result = writeRegMag(AK8963_CNTL1, 0b00010110);
    break;
  case MAG_MODE_SELF_TEST:
    result = writeRegMag(AK8963_CNTL1, 0b00011000);
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

void MPU9250::magCalibration()
{
  DEBUG_LOG("Hard and Soft Iron effect compensation\r\n");
  float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;
  uint8_t data[7];
  uint8_t state = 0;  
  /* Hard Iron effect compensation */
  for (uint32_t i = 0; i < 50; ++i)
  {

    do
    {
      readRegMag(AK8963_ST1, &state);
    } while (!(state & 0x01));

    readRegMag(AK8963_HXL, &data[0], 7);
    float Mx_Raw = ((data[1] << 8) | data[0]);
    float My_Raw = ((data[3] << 8) | data[2]);
    float Mz_Raw = ((data[5] << 8) | data[4]);

    if (Mx_Raw > X_max)
      X_max = Mx_Raw;
    if (My_Raw > Y_max)
      Y_max = My_Raw;
    if (Mz_Raw > Z_max)
      Z_max = Mz_Raw;

    if (Mx_Raw < X_min)
      X_min = Mx_Raw;
    if (My_Raw < Y_min)
      Y_min = My_Raw;
    if (Mz_Raw < Z_min)
      Z_min = Mz_Raw;

    HAL_Delay(10);
  }

  magOffsetX = (X_max + X_min) / 2;
  magOffsetY = (Y_max + Y_min) / 2;
  magOffsetZ = (Z_max + Z_min) / 2;

  /* Soft Iron effect compensation */
  float delta_x = (X_max - X_min) / 2;
  float delta_y = (Y_max - Y_min) / 2;
  float delta_z = (Z_max - Z_min) / 2;

  float delta = (delta_x + delta_y + delta_z) / 3;

  mScaleX = delta / delta_x;
  mScaleY = delta / delta_y;
  mScaleZ = delta / delta_z;
  DEBUG_LOG("Offsets [X %f, Y %f, Z %f]\r\n", magOffsetX, magOffsetY, magOffsetZ);
  DEBUG_LOG("Scale [X %f, Y %f, Z %f]\r\n", mScaleX, mScaleY, mScaleZ);
}

bool MPU9250::magSelfTest()
{
  if (!magSetMode(MAG_MODE_PD))
  {
    return false;
  }

  if (!writeRegMag(AK8963_ASTC, 0b01000000))
  {
    return false;
  }

  if (!magSetMode(MAG_MODE_SELF_TEST))
  {
    return false;
  }

  uint8_t data[7];
  uint8_t state = 0;
  do
  {
    readRegMag(AK8963_ST1, &state);
  } while (!(state & 0x01));

  axes a = {0};
  readRegMag(AK8963_HXL, &data[0], 7);
  float Mx_Raw = ((data[1] << 8) | data[0]);
  float My_Raw = ((data[3] << 8) | data[2]);
  float Mz_Raw = ((data[5] << 8) | data[4]);
  /* Calculate uT (micro Tesla) value for XYZ axis */
  a.x = Mx_Raw * corrX;
  a.y = My_Raw * corrY;
  a.z = Mz_Raw * corrZ;

  if (!writeRegMag(AK8963_ASTC, 0))
  {
    return false;
  }
  if (!magSetMode(MAG_MODE_PD))
  {
    return false;
  }
  DEBUG_LOG("Self Test: %.4f %.4f %.4f\r\n", a.x, a.y, a.z);
  return true;
}

bool MPU9250::initMag()
{
  /* Check if device connected */
  if (HAL_OK != HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR_MAG, 10, 100))
  {
    DEBUG_LOG("MAG is not online\r\n");
    return false;
  }

  magRST();

  // Case 2: Who am i test
  uint8_t wai = 0;
  readRegMag(AK8963_WIA, &wai);

  if (wai != 0x48)
  {
    return false;
  }

  DEBUG_LOG("AK8963 is detected [%#.2X]\r\n", wai);
  magSetMode(MAG_MODE_PD);
  // Entering FUSE rom read state
  if (!magSetMode(MAG_MODE_FUSE_ROM))
  {
    return false;
  }
  HAL_Delay(10);
  uint8_t fuseRom[3] = {0};
  if (!readRegMag(AK8963_ASAX, &fuseRom[0], 3))
  {
    return false;
  }
  corrX = (((fuseRom[0] - 128.0) * 0.5) / 128) + 1;
  corrY = (((fuseRom[1] - 128.0) * 0.5) / 128) + 1;
  corrZ = (((fuseRom[2] - 128.0) * 0.5) / 128) + 1;

  DEBUG_LOG("Corrections:\r\nX=%.4f\r\nY=%.4f\r\nZ=%.4f\r\n", corrX, corrY, corrZ);

  magSelfTest();  
  // Entering Continous mode, 8Hz
  magSetMode(MAG_MODE_PD);
  if (!magSetMode(MAG_MODE_CONT_8HZ))
  {
    return false;
  }

  magCalibration();
  

  DEBUG_LOG("Mag is ready\r\n");
  return true;
}

axes MPU9250::readMag()
{
  uint8_t data[7];
  uint8_t state = 0;
  do
  {
    readRegMag(AK8963_ST1, &state);
  } while (!(state & 0x01));

  axes result = {0};
  readRegMag(AK8963_HXL, &data[0], 7);
  float Mx_Raw = ((data[1] << 8) | data[0]) - magOffsetX;
  float My_Raw = ((data[3] << 8) | data[2]) - magOffsetY;
  float Mz_Raw = ((data[5] << 8) | data[4]) - magOffsetZ;
  /* Calculate uT (micro Tesla) value for XYZ axis */
  result.x = Mx_Raw * corrX * gSensF * mScaleX;
  result.y = My_Raw * corrY * gSensF * mScaleY;
  result.z = Mz_Raw * corrZ * gSensF * mScaleZ;
  result.z = result.z * -1;

  return result;
}

float MPU9250::getAzimuth()
{
  auto ax = readMag();
  float Xf = ax.x;
  float Yf = ax.y;
  float Zf = ax.z;
  float norm = sqrtf(Xf * Xf + Yf * Yf + Zf * Zf);
  if (!norm)
  {
    return -1;
  }
  norm = 1.0f / norm;
  Xf *= norm;
  Yf *= norm;
  Zf *= norm;

  DEBUG_LOG("X=%f Y=%f Z=%f\r\n", Xf, Yf, Zf);
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
