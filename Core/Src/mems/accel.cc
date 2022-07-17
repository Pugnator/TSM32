#include "mpu.h"

bool MPU9250::initAcc()
{
  if (HAL_OK != HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR, 10, 100))  
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Accelerometer is not online\r\n");
    return false;
  }
  uint8_t wai = 0;
  if (!readRegMpu(MPU9250_WHO_AM_I, &wai))
  {
    return false;
  }
  if (0x71 == wai)
  {
    DEBUG_LOG("MPU 9250 is detected\r\n");
  }
  else
  {
    return false;
  }

  // Clear sleep mode bit (6), enable all sensors
  if (!writeRegMpu(MPU9250_PWR_MGMT_1, 0x00))
  {
    return false;
  }
  HAL_Delay(100); // Wait for all registers to reset

  // get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else  
  if (!writeRegMpu(MPU9250_PWR_MGMT_1, 0x01))
  {
    return false;
  }
  HAL_Delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  
  writeRegMpu(MPU9250_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)  
  writeRegMpu(MPU9250_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                                         // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t val = 0;
  uint8_t *data = &val;
  readRegMpu(MPU9250_GYRO_CONFIG, data); // get current GYRO_CONFIG register value
  *data = *data & ~0xE0; // Clear self-test bits [7:5]
  *data &= ~0x02;     // Clear Fchoice bits [1:0]
  *data &= ~0x18;     // Clear AFS bits [4:3]
  *data |= 0x00 << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
  writeRegMpu(MPU9250_GYRO_CONFIG, std::move(*data)); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  readRegMpu(MPU9250_ACCEL_CONFIG, data); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  *data &= ~0x18;                           // Clear AFS bits [4:3]
  *data |= 0x00 << 3;                       // Set full scale range for the accelerometer
  
  writeRegMpu(MPU9250_ACCEL_CONFIG, std::move(*data)); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  readRegMpu(MPU9250_ACCEL_CONFIG_2, data); // get current ACCEL_CONFIG2 register value
  *data &= ~0x0F;                             // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  *data |= 0x03;                              // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  
  writeRegMpu(MPU9250_ACCEL_CONFIG_2, std::move(*data)); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  //0010 0010
  writeRegMpu(MPU9250_INT_PIN_CFG, 0x22);  
  //01000000
  writeRegMpu(MPU9250_INT_ENABLE, 0x2);
  DEBUG_LOG("MPU is up\r\n");
  return true;
}

bool MPU9250::readAccel()
{
  uint8_t data[8];
  /* Read accelerometer data */
  readRegMpu(MPU9250_ACCEL_XOUT_H, data, 6);

  float Ax_Raw = ((int16_t)data[0] << 8) | data[1];
  float Ay_Raw = ((int16_t)data[2] << 8) | data[3];
  float Az_Raw = ((int16_t)data[4] << 8) | data[5];

  uint16_t _temp = (data[6] << 8 | data[7]);
  float temp = ((_temp - 21) / 333.87) + 21;
  DEBUG_LOG("Temp = %.2f\r\n", temp);

  float Ax = (float)Ax_Raw * aMult;
  float Ay = (float)Ay_Raw * aMult;
  float Az = (float)Az_Raw * aMult;
  DEBUG_LOG("Acc %.4f, %.4f, %.4f\r\n", Ax, Ay, Az);
  return true;
}