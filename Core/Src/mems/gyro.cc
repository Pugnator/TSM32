#include "mpu.h"
#ifdef __cplusplus
extern "C"
{
#endif

  MPU9250_Error_code MPU9250_Gyroscope_Configuration(I2C_HandleTypeDef *I2Cx, MPU9250_Gyro_range Range, struct MPU9250 *mpu)
  {

    uint8_t Byte_temp = 0x00;

    /* Case 1: Set gyroscope sensitivity range */
    Byte_temp = Range << 3;

    if (HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, MPU9250_GYRO_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(&hi2c1, 1000);
      return MPU9250_Gyroscope_Config_FAIL;
    }

    /* Case 2: Set gyroscope low pass filter cut-off frequency */
    Byte_temp = 0x0E;

    if (HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, MPU9250_CONFIG, 1, &Byte_temp, 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(&hi2c1, 1000);
      return MPU9250_Gyroscope_Config_FAIL;
    }

    /* Case 3: Save configuration to data structure */
    if (Range == MPU9250_Gyro_250s)
      mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_250s;
    else if (Range == MPU9250_Gyro_500s)
      mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_500s;
    else if (Range == MPU9250_Gyro_1000s)
      mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_1000s;
    else if (Range == MPU9250_Gyro_2000s)
      mpu->Gyroscope_sensitivity_factor = MPU9250_GYRO_SENSITIVITY_FACTOR_2000s;

    mpu->Gyroscope_X_offset = 0;
    mpu->Gyroscope_Y_offset = 0;
    mpu->Gyroscope_Z_offset = 0;

    return MPU9250_Gyroscope_Config_OK;
  }

  void MPU9250_Calibration_Gyro(I2C_HandleTypeDef *I2Cx,
                                struct MPU9250 *mpu)
  {
    DEBUG_LOG("Started gyro calibration\r\n");
    float Gyro_X_offset = 0, Gyro_Y_offset = 0, Gyro_Z_offset = 0;

    for (int i = 0; i < 1000; ++i)
    {

      if (MPU9250_Read_Gyroscope_FAIL == MPU9250_Read_Gyroscope(I2Cx, mpu))
      {
        DEBUG_LOG("Failed to read the gyro\r\n");
        continue;
      }

      Gyro_X_offset += mpu->Gyroscope_X;
      Gyro_Y_offset += mpu->Gyroscope_Y;
      Gyro_Z_offset += mpu->Gyroscope_Z;
      HAL_Delay(20);
    }

    mpu->Gyroscope_X_offset = Gyro_X_offset / 1000;
    mpu->Gyroscope_Y_offset = Gyro_Y_offset / 1000;
    mpu->Gyroscope_Z_offset = Gyro_Z_offset / 1000;
    DEBUG_LOG("Gyro calibration finished. Offsets [X %f, Y %f, Z %f]\r\n", mpu->Gyroscope_X_offset, mpu->Gyroscope_Y_offset, mpu->Gyroscope_Z_offset);
  }

  MPU9250_Error_code MPU9250_Read_Gyroscope(I2C_HandleTypeDef *I2Cx,
                                            struct MPU9250 *mpu)
  {

    uint8_t __temp[6] = {0x00};

    if (HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, MPU9250_GYRO_XOUT_H, 1, __temp, 6, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(&hi2c1, 1000);
      return MPU9250_Read_Gyroscope_FAIL;
    }

    mpu->Gyroscope_X = (__temp[0] << 8 | __temp[1]) - mpu->Gyroscope_X_offset;
    mpu->Gyroscope_Y = (__temp[2] << 8 | __temp[3]) - mpu->Gyroscope_Y_offset;
    mpu->Gyroscope_Z = (__temp[4] << 8 | __temp[5]) - mpu->Gyroscope_Z_offset;

    /* Case x: Calculate dgs/s values for XYZ axis */
    mpu->Gyroscope_X_dgs = (float)(mpu->Gyroscope_X) / mpu->Gyroscope_sensitivity_factor;
    mpu->Gyroscope_Y_dgs = (float)(mpu->Gyroscope_Y) / mpu->Gyroscope_sensitivity_factor;
    mpu->Gyroscope_Z_dgs = (float)(mpu->Gyroscope_Z) / mpu->Gyroscope_sensitivity_factor;

    return MPU9250_Read_Gyroscope_OK;
  }
#ifdef __cplusplus
}
#endif