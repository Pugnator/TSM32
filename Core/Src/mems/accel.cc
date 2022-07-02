#include "mpu.h"
#ifdef __cplusplus
extern "C"
{
#endif
MPU9250_Error_code MPU9250_Accelerometer_Configuration(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu, MPU9250_Acce_range Range)
{

  uint8_t Byte_temp = 0x00;

  /* Case 1: Set accelerometer sensitivity range */
  Byte_temp = Range << 3;

  if (HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, MPU9250_ACCEL_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK)
  {

    return MPU9250_Accelerometer_Config_FAIL;
  }

  /* Case 2: Set accelerometer low pass filter cut-off frequency */
  /*
  Byte_temp = 0x0E;
  if( HAL_I2C_Mem_Write(I2Cx, mpu->Device_addres, MPU9250_ACCEL_CONFIG_2, 1, &Byte_temp, 1, 1000) != HAL_OK ) {
    return MPU9250_Accelerometer_Config_FAIL;
  }
  */

  /* Case 3: Save configuration to data structure */
  if (Range == MPU9250_Acce_2G)
    mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_2G;
  else if (Range == MPU9250_Acce_4G)
    mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_4G;
  else if (Range == MPU9250_Acce_8G)
    mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_8G;
  else if (Range == MPU9250_Acce_16G)
    mpu->Accelerometer_sensitivity_factor = MPU9250_ACCE_SENSITIVITY_FACTOR_16G;

  mpu->Accelerometer_X_offset = 0;
  mpu->Accelerometer_Y_offset = 0;
  mpu->Accelerometer_Z_offset = 0;

  return MPU9250_Accelerometer_Config_OK;
}

void MPU9250_Calibration_Accel(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu)
{

  float Acce_X_offset = 0, Acce_Y_offset = 0, Acce_Z_offset = 0;

  for (int i = 0; i < 1000; ++i)
  {

    MPU9250_Read_Accelerometer(I2Cx, mpu);

    Acce_X_offset = Acce_X_offset + mpu->Accelerometer_X;
    Acce_Y_offset = Acce_Y_offset + mpu->Accelerometer_Y;
    Acce_Z_offset = Acce_Z_offset + mpu->Accelerometer_Z;
    HAL_Delay(20);
  }

  mpu->Accelerometer_X_offset = Acce_X_offset / 1000;
  mpu->Accelerometer_Y_offset = Acce_Y_offset / 1000;
  mpu->Accelerometer_Z_offset = Acce_Z_offset / 1000;

  mpu->Accelerometer_Z_offset = mpu->Accelerometer_Z_offset - mpu->Accelerometer_sensitivity_factor;
}

MPU9250_Error_code MPU9250_Read_Accelerometer(I2C_HandleTypeDef *I2Cx, struct MPU9250 *mpu)
{

  uint8_t __temp[8] = {0x00};

  if (HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, MPU9250_ACCEL_XOUT_H, 1, __temp, 8, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(&hi2c1, 1000);
    DEBUG_LOG("Error reading MPU9250_ACCEL_XOUT_H\r\n");
    return MPU9250_Read_Accelerometer_FAIL;
  }

  mpu->Accelerometer_X = (__temp[0] << 8 | __temp[1]) - mpu->Accelerometer_X_offset;
  mpu->Accelerometer_Y = (__temp[2] << 8 | __temp[3]) - mpu->Accelerometer_Y_offset;
  mpu->Accelerometer_Z = (__temp[4] << 8 | __temp[5]) - mpu->Accelerometer_Z_offset;
  uint16_t temp = (__temp[6] << 8 | __temp[7]);
  mpu->temp = ((temp - 21) / 333.87) + 21;
  return MPU9250_Read_Accelerometer_OK;
}
#ifdef __cplusplus
}
#endif