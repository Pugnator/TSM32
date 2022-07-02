#include "mpu.h"
#ifdef __cplusplus
extern "C"
{
#endif
MPU9250_Error_code MPU9250_Magnetometer_Configuration(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu)
{

  uint8_t Byte_temp = 0x00;
  uint8_t __temp[3] = {0};

  uint8_t addr = 0x0C << 1;
  mpu->Magnetometer_sesitivity_factor = 0.1499; /* 4912/32768 */

  // Case 2: I2C master interface
  Byte_temp = 0x00;

  if (HAL_I2C_Mem_Write(I2Cx, addr, MPU9250_USER_CTRL, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  // Case 3: Enable the bypass multiplexer
  Byte_temp = 0x02;

  if (HAL_I2C_Mem_Write(I2Cx, addr, MPU9250_INT_PIN_CFG, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  // Case 1: Is device connected ?
  if (HAL_I2C_IsDeviceReady(I2Cx, addr, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  // Case 2: Who am i test
  if (HAL_I2C_Mem_Read(I2Cx, addr, AK9863_WIA, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Init_FAIL;
  }

  if (Byte_temp != 0x48)
  {
    return MPU9250_Init_FAIL;
  }

  // Case 4: Setup to fuse ROM access mode and 16-bit output
  Byte_temp = 0x1F;

  if (HAL_I2C_Mem_Write(I2Cx, addr, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  HAL_Delay(100);

  // Case 5: Read from the fuse ROM sensitivity adjustment values
  if (HAL_I2C_Mem_Read(I2Cx, addr, AK9863_ASAX | 0x80, 1, __temp, 3, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  mpu->Magnetometer_ASAX = (((__temp[0] - 128.0) * 0.5) / 128.0) + 1.0;
  mpu->Magnetometer_ASAY = (((__temp[1] - 128.0) * 0.5) / 128.0) + 1.0;
  mpu->Magnetometer_ASAZ = (((__temp[2] - 128.0) * 0.5) / 128.0) + 1.0;

  // Case 6: Reset to power down mode
  Byte_temp = 0x00;

  if (HAL_I2C_Mem_Write(I2Cx, addr, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  // Case 7: Enable continuous mode 2 and 16-bit output
  Byte_temp = 0x16; // 0x16

  if (HAL_I2C_Mem_Write(I2Cx, addr, AK9863_CNTL1, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Magnetometer_Config_FAIL;
  }

  HAL_Delay(100);

  return MPU9250_Magnetometer_Config_OK;
}

void MPU9250_Calibration_Mag(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu)
{
  DEBUG_LOG("Started magnetometer calibration\r\n");
  float X_max = -99999, X_min = 99999, Y_max = -99999, Y_min = 99999, Z_max = -99999, Z_min = 99999;

  /* Hard Iron effect compensation */
  for (int i = 0; i < 1000; ++i)
  {

    if (MPU9250_Read_Magnetometer_FAIL == MPU9250_Read_Magnetometer(I2Cx, mpu))
    {
      DEBUG_LOG("Failed to read magnetometer\r\n");
      continue;
    }

    if (mpu->Magnetometer_X > X_max)
      X_max = mpu->Magnetometer_X;
    if (mpu->Magnetometer_Y > Y_max)
      Y_max = mpu->Magnetometer_Y;
    if (mpu->Magnetometer_Z > Z_max)
      Z_max = mpu->Magnetometer_Z;

    if (mpu->Magnetometer_X < X_min)
      X_min = mpu->Magnetometer_X;
    if (mpu->Magnetometer_Y < Y_min)
      Y_min = mpu->Magnetometer_Y;
    if (mpu->Magnetometer_Z < Z_min)
      Z_min = mpu->Magnetometer_Z;

    HAL_Delay(20);
  }

  mpu->Magnetometer_X_offset = (X_max + X_min) / 2;
  mpu->Magnetometer_Y_offset = (Y_max + Y_min) / 2;
  mpu->Magnetometer_Z_offset = (Z_max + Z_min) / 2;

  /* Soft Iron effect compensation */
  float delta_x = (X_max - X_min) / 2;
  float delta_y = (Y_max - Y_min) / 2;
  float delta_z = (Z_max - Z_min) / 2;

  float delta = (delta_x + delta_y + delta_z) / 3;

  mpu->Magnetometer_X_scale = delta / delta_x;
  mpu->Magnetometer_Y_scale = delta / delta_y;
  mpu->Magnetometer_Z_scale = delta / delta_z;
  DEBUG_LOG("Gyro calibration finished. Offsets [X %f, Y %f, Z %f]\r\n", mpu->Magnetometer_X_offset, mpu->Magnetometer_Y_offset, mpu->Magnetometer_Z_offset);
}

MPU9250_Error_code MPU9250_Read_Magnetometer(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu)
{

  uint8_t __temp[8] = {0x00};

  /* Case x: Read measured values from registers */
  if (HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, AK9863_ST1, 1, __temp, 8, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    return MPU9250_Read_Magnetometer_FAIL;
  }

  if (__temp[0] & 0x00)
  {

    return MPU9250_Read_Magnetometer_FAIL;
  }

  mpu->Magnetometer_X = ((__temp[2] << 8 | __temp[1]) - mpu->Magnetometer_X_offset);
  mpu->Magnetometer_Y = ((__temp[4] << 8 | __temp[3]) - mpu->Magnetometer_Y_offset);
  mpu->Magnetometer_Z = ((__temp[6] << 8 | __temp[5]) - mpu->Magnetometer_Z_offset);

  /* Case x: Calculate uT (micro Tesla) value for XYZ axis */
  mpu->Magnetometer_X_uT = mpu->Magnetometer_X * mpu->Magnetometer_ASAX * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_X_scale;
  mpu->Magnetometer_Y_uT = mpu->Magnetometer_Y * mpu->Magnetometer_ASAY * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_Y_scale;
  mpu->Magnetometer_Z_uT = mpu->Magnetometer_Z * mpu->Magnetometer_ASAZ * mpu->Magnetometer_sesitivity_factor * mpu->Magnetometer_Z_scale;

  float a = mpu->Magnetometer_X_uT;
  float b = mpu->Magnetometer_Y_uT;
  float c = mpu->Magnetometer_Z_uT;

  mpu->Magnetometer_X_uT = b;
  mpu->Magnetometer_Y_uT = a;
  mpu->Magnetometer_Z_uT = -c;

  return MPU9250_Read_Magnetometer_OK;
}
#ifdef __cplusplus
}
#endif