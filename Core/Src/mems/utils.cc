#include "mpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MPU9250_reset(I2C_HandleTypeDef *I2Cx)
{

  MPU9250_write(I2Cx, MPU9250_USER_CTRL, 0); // disable internal I2C bus

  // reset device
  MPU9250_write(I2Cx, MPU9250_PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250

  MPU9250_write(I2Cx, MPU9250_USER_CTRL, 0x20); // re-enable internal I2C bus
  HAL_Delay(100);                       // Wait for all registers to reset
}

bool MPU9250_write(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t byte)
{
  if (HAL_I2C_Mem_Write(I2Cx, IMU_ADDR, reg, 1, &byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(I2Cx, 1000);
    return false;
  }
  return true;
}

bool MPU9250_read(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t *byte)
{
  if (HAL_I2C_Mem_Read(I2Cx, IMU_ADDR, reg, 1, byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(I2Cx, 1000);
    return false;
  }
  return true;
}

void MPU9250_WOM(I2C_HandleTypeDef *I2Cx)
{
    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    uint8_t __temp = 0;
    MPU9250_read(I2Cx, MPU9250_ACCEL_CONFIG_2, &__temp);     // get current ACCEL_CONFIG2 register value
    __temp = __temp & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    __temp = __temp | 0x01;  // Set accelerometer rate to 1 kHz and bandwidth to 184 Hz
    MPU9250_write(I2Cx, MPU9250_ACCEL_CONFIG_2, __temp); // Write new ACCEL_CONFIG2 register value

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master 
    MPU9250_write(I2Cx, MPU9250_INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
    MPU9250_write(I2Cx, MPU9250_INT_ENABLE, 0x41);   // Enable data ready (bit 0) and wake on motion (bit 6)  interrupt

    // enable wake on motion detection logic (bit 7) and compare current sample to previous sample (bit 6)
    MPU9250_write(I2Cx, MPU9250_MOT_DETECT_CTRL, 0xC0);  

    // set accel threshold for wake up at  mG per LSB, 1 - 255 LSBs == 0 - 1020 mg), pic 0x19 for 25 mg
    MPU9250_write(I2Cx, MPU9250_WOM_THR, 0x19);

    // set sample rate in low power mode
    /* choices are 0 == 0.24 Hz, 1 == 0.49 Hz, 2 == 0.98 Hz, 3 == 1.958 Hz, 4 == 3.91 Hz, 5 == 7.81 Hz
     *             6 == 15.63 Hz, 7 == 31.25 Hz, 8 == 62.50 Hz, 9 = 125 Hz, 10 == 250 Hz, and 11 == 500 Hz
     */
    MPU9250_write(I2Cx, MPU9250_LP_ACCEL_ODR, 0x02);
    
    MPU9250_read(I2Cx, MPU9250_PWR_MGMT_1, &__temp);
    MPU9250_write(I2Cx, MPU9250_PWR_MGMT_1, __temp | 0x20);     // Write bit 5 to enable accel cycling

    //gyroMagSleep();
    HAL_Delay(100); // Wait for all registers to reset 

}

#ifdef __cplusplus
}
#endif