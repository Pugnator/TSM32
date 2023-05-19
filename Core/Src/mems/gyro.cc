#include "mpu.h"

Axis3D MPU9250::readGyro()
{
  uint8_t data[6];
  Axis3D result = {0};
  readRegMpu(MPU9250_GYRO_XOUT_H, data, 6);

  float Gx_Raw = ((int16_t)data[0] << 8) | data[1];
  float Gy_Raw = ((int16_t)data[2] << 8) | data[3];
  float Gz_Raw = ((int16_t)data[4] << 8) | data[5];

  float Gx = Gx_Raw * gMult;
  float Gy = Gy_Raw * gMult;
  float Gz = Gz_Raw * gMult;
  DEBUG_LOG("Gyro %.4f, %.4f, %.4f\r\n", Gx, Gy, Gz);
  return result;
}