#include "mpu.h"


bool MPU9250::writeRegMag(uint8_t reg, uint8_t byte)
{  
  uint8_t temp = byte;
  DEBUG_LOG("MAG write: %.2X to %.2X\r\n", temp, reg);
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, &temp, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
  return true;
}

bool MPU9250::readRegMag(uint8_t reg, uint8_t *byte)
{
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to read MAG\r\n");
    return false;
  }
  DEBUG_LOG("Read #%.2X from MAG\r\n", *byte);
  return true;
}

bool MPU9250::readRegMag(uint8_t reg, uint8_t *byte, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(i2c, 1000);
      DEBUG_LOG("Failed to read MAG\r\n");
      return false;
    }
  }
  return true;
}

void MPU9250::resetMag()
{
  writeRegMpu(AK8963_CNTL2, AKM_RESET);
  HAL_Delay(100);
}

bool MPU9250::initMag()
{
  /* Check if device connected */
  if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR_MAG, 10, 100))
  {
    DEBUG_LOG("MAG is online\r\n");
  }
  else
  {
    DEBUG_LOG("MAG is not online\r\n");
    return false;
  }  

  // Case 2: Who am i test
  uint8_t wai = 0;
  readRegMag(AK8963_WIA, &wai);  

  if (wai != 0x48)
  {
    return false;
  }

  DEBUG_LOG("AK8963 is detected [#%.2X]\r\n", wai);  
  uint8_t reg = 0;  
  readRegMag(AK8963_INFO, &reg);
  readRegMag(AK8963_ASAX, &reg);
  readRegMag(AK8963_ASAY, &reg);
  readRegMag(AK8963_ASAZ, &reg);

  

#ifdef MPU_CALIBRATE
  DEBUG_LOG("Started magnetometer calibration\r\n");
  float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;

  /* Hard Iron effect compensation */
  for (size_t i = 0; i < 2; ++i)
  {
    auto axes = readMag();

    if (axes.x > X_max)
      X_max = axes.x;
    if (axes.y > Y_max)
      Y_max = axes.y;
    if (axes.z > Z_max)
      Z_max = axes.z;

    if (axes.x < X_min)
      X_min = axes.x;
    if (axes.y < Y_min)
      Y_min = axes.y;
    if (axes.z < Z_min)
      Z_min = axes.z;

    HAL_Delay(20);
  }

  float X_offset = (X_max + X_min) / 2;
  float Y_offset = (Y_max + Y_min) / 2;
  float Z_offset = (Z_max + Z_min) / 2;

  /* Soft Iron effect compensation */
  // float delta_x = (X_max - X_min) / 2;
  // float delta_y = (Y_max - Y_min) / 2;
  // float delta_z = (Z_max - Z_min) / 2;

  // float delta = (delta_x + delta_y + delta_z) / 3;

  // float mMultx = delta / delta_x;
  // float mMulty = delta / delta_y;
  // float mMultz = delta / delta_z;
  DEBUG_LOG("MAG calibration finished. Offsets [X %f, Y %f, Z %f]\r\n", X_offset, Y_offset, Z_offset);
#endif

  return true;
}

axes MPU9250::readMag()
{
  uint8_t data[8];

  /* Check status */
  readRegMag(AK8963_ST1, data, 8);
  
  axes result = {0};
  if (data[0] & 0x01)
  {
    DEBUG_LOG("Data is ready\r\n");
    
      uint8_t status2 = 0;
      readRegMag(AK8963_ST2, &status2, 1);      
      float Mx_Raw = ((int16_t)data[2] << 8) | data[1];
      float My_Raw = ((int16_t)data[4] << 8) | data[3];
      float Mz_Raw = ((int16_t)data[6] << 8) | data[5];

      result.x = (float)Mx_Raw * mMult;
      result.y = (float)Mz_Raw * mMult;
      result.z = (float)My_Raw * mMult;

      DEBUG_LOG("Magnetometer %.4f, %.4f, %.4f\r\n", result.x, result.y, result.z);
      return result;
    
  }
  uint8_t cntl1 = 0;
  readRegMag(AK8963_CNTL1, &cntl1, 1);
  DEBUG_LOG("MAG is not ready, mode = 0%.8b [%.8b]\r\n", cntl1, data[0]);   
  return result;
}

float MPU9250::getAzimuth()
{
  auto ax = readMag();
  float Xf = ax.x;
  float Yf = ax.y;
  float Zf = ax.z;
  float norm = sqrtf(Xf * Xf + Yf * Yf + Zf * Zf);
  if (norm == 0.0f)
  {
  }
  norm = 1.0f / norm;
  Xf *= norm;
  Yf *= norm;
  Zf *= norm;

  // DEBUG_LOG("X=%f Y=%f Z=%f\r\n", Xf, Yf, Zf);
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