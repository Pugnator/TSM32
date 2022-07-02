
#include "mpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT_Pin)
  {
    DEBUG_LOG("Interrupt!\r\n");
  }
}

MPU9250_Error_code MPU9250_Init(I2C_HandleTypeDef *I2Cx,
                                struct MPU9250 *mpu,
                                MPU9250_Acce_range Acce_range,
                                MPU9250_Gyro_range Gyro_range)
{

  DEBUG_LOG("Device address 0x%X\r\n", IMU_ADDR);
  if (HAL_I2C_IsDeviceReady(I2Cx, IMU_ADDR, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(I2Cx, 1000);
    DEBUG_LOG("I2C bus is not ready\r\n");
    return MPU9250_Init_FAIL;
  }
  DEBUG_LOG("I2C bus ready\r\n");

  MPU9250_reset(I2Cx);

  uint8_t __temp = 0;

  if (!MPU9250_read(I2Cx, MPU9250_WHO_AM_I, &__temp))
  {
    DEBUG_LOG("Device ack is not OK\r\n");
    return MPU9250_Init_FAIL;
  }

  if (__temp != 0x71)
  {
    DEBUG_LOG("Device ack is not OK\r\n");
    return MPU9250_Init_FAIL;
  }

  intConfig cfg;
  cfg.reg = 0;
  cfg.ACTL = 0;
  cfg.OPEN = 1;
  cfg.LATCH_INT_EN = 0;
  cfg.INT_ANYRD_2CLEAR = 1;
  cfg.ACTL_FSYNC = 0;
  cfg.FSYNC_INT_MODE_EN = 0;
  cfg.BYPASS_EN = 0;

  if (!MPU9250_write(I2Cx, MPU9250_INT_PIN_CFG, cfg.reg))
  {
    return MPU9250_Init_FAIL;
  }

  intEnable inten;
  inten.reg = 0x12;

  if (!MPU9250_write(I2Cx, MPU9250_INT_ENABLE, 0x22))
  {
    return MPU9250_Init_FAIL;
  }

  DEBUG_LOG("Device ack OK\r\n");
  // MPU9250_WOM(I2Cx, mpu);

  if (!MPU9250_write(I2Cx, MPU9250_CONFIG, 0x03))
  {
    return MPU9250_Init_FAIL;
  }

  if (!MPU9250_write(I2Cx, MPU9250_SMPLRT_DIV, 0x04))
  {
    return MPU9250_Init_FAIL;
  }

  MPU9250_Magnetometer_Configuration(I2Cx, mpu);
  // MPU9250_Accelerometer_Configuration(I2Cx, mpu, MPU9250_Acce_2G);
  // MPU9250_Gyroscope_Configuration(I2Cx, mpu, MPU9250_Gyro_250s);

  mpu->Magnetometer_X_scale = 1;
  mpu->Magnetometer_Y_scale = 1;
  mpu->Magnetometer_Z_scale = 1;

  mpu->Magnetometer_X_offset = 0;
  mpu->Magnetometer_Y_offset = 0;
  mpu->Magnetometer_Z_offset = 0;

  return MPU9250_Init_OK;
}

void MPU9250_Set_Offsets(I2C_HandleTypeDef *I2Cx,
                         struct MPU9250 *mpu,
                         float Acce_X_offset, float Acce_Y_offset, float Acce_Z_offset,
                         float Gyro_X_offset, float Gyro_Y_offset, float Gyro_Z_offset,
                         float Mag_X_offset, float Mag_Y_offset, float Mag_Z_offset,
                         float Mag_X_scale, float Mag_Y_scale, float Mag_Z_scale)
{

  mpu->Accelerometer_X_offset = Acce_X_offset;
  mpu->Accelerometer_Y_offset = Acce_Y_offset;
  mpu->Accelerometer_Z_offset = Acce_Z_offset;

  mpu->Gyroscope_X_offset = Gyro_X_offset;
  mpu->Gyroscope_Y_offset = Gyro_Y_offset;
  mpu->Gyroscope_Z_offset = Gyro_Z_offset;

  mpu->Magnetometer_X_offset = Mag_X_offset;
  mpu->Magnetometer_Y_offset = Mag_Y_offset;
  mpu->Magnetometer_Z_offset = Mag_Z_offset;

  mpu->Magnetometer_X_scale = Mag_X_scale;
  mpu->Magnetometer_Y_scale = Mag_Y_scale;
  mpu->Magnetometer_Z_scale = Mag_Z_scale;
}

void mpu_test()
{
  DEBUG_LOG("MPU app started\r\n");
  struct MPU9250 mpu;
  int retr = 100;
  HAL_Delay(1000);
  for (; retr && MPU9250_Init(&hi2c1, &mpu, MPU9250_Acce_2G, MPU9250_Gyro_250s) != MPU9250_Init_OK; retr--)
  {
    HAL_Delay(100);
  }

  if (!retr)
  {
    DEBUG_LOG("Failed to init MPU\r\n");
    return;
  }

  MPU9250_Set_Offsets(&hi2c1, &mpu, 0, 0, 0, 0, 0, 0, -26.536, 0.992, 0.968, 1.04, 1, 1);
  // MPU9250_Calibration_Accel(&hi2c1, &mpu);
  // MPU9250_Calibration_Gyro(&hi2c1, &mpu);
  MPU9250_Calibration_Mag(&hi2c1, &mpu);

  DEBUG_LOG("MPU started and inited\r\n");

  for (uint32_t i = 0; i < 100000; i++)
  {
    if (MPU9250_Read_Magnetometer_FAIL == MPU9250_Read_Magnetometer(&hi2c1, &mpu))
    {
      continue;
    }

    // if (MPU9250_Read_Gyroscope_FAIL == MPU9250_Read_Gyroscope(&hi2c1, &mpu))
    {
      // continue;
    }

    // float Xf = (float)(mpu.Accelerometer_X) / mpu.Accelerometer_sensitivity_factor;
    // float Yf = (float)(mpu.Accelerometer_Y) / mpu.Accelerometer_sensitivity_factor;
    // float Zf = (float)(mpu.Accelerometer_Z) / mpu.Accelerometer_sensitivity_factor;
    /*
    Direction (y>0) = 90 - [arcTAN(x/y)]*180/π
    Direction (y<0) = 270 - [arcTAN(x/y)]*180/π
    Direction (y=0, x<0) = 180.0
    Direction (y=0, x>0) = 0.0
    */
    /*
     float Xg = mpu.Gyroscope_X_dgs;
     float Yg = mpu.Gyroscope_Y_dgs;
     float Zg = mpu.Gyroscope_Z_dgs;
 */
    float Xf = mpu.Magnetometer_X_uT;
    float Yf = mpu.Magnetometer_Y_uT;
    float Zf = mpu.Magnetometer_Z_uT;
    //  DEBUG_LOG("Gyro X=%f, Y=%f, Z=%f\r\n", Xg, Yg, Zg);

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
    HAL_Delay(500);
  }
}
#ifdef __cplusplus
}
#endif