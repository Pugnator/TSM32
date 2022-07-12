#include "mpu.h"

// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU9250::selfTest()
{
  DEBUG_LOG("Started self test\r\n");
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeRegMpu(MPU9250_SMPLRT_DIV, 0);
  writeRegMpu(MPU9250_CONFIG, 0x02);
  writeRegMpu(MPU9250_GYRO_CONFIG, FS << 3);
  writeRegMpu(MPU9250_ACCEL_CONFIG_2, 0x02);
  writeRegMpu(MPU9250_ACCEL_CONFIG, FS << 3);

  for (int ii = 0; ii < 200; ii++)
  { // get average current values of gyro and acclerometer
    // Read the six raw data registers into data array
    writeRegMpu(MPU9250_ACCEL_XOUT_H, rawData, 6);

    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    // Read the six raw data registers sequentially into data array
    readRegMpu(MPU9250_GYRO_XOUT_H, rawData, 6);

    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    HAL_Delay(1);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test

  HAL_Delay(25); // osDelay a while to let the device stabilize
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeRegMpu(MPU9250_ACCEL_CONFIG, 0xE0);

  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

  writeRegMpu(MPU9250_GYRO_CONFIG, 0xE0);

  for (int ii = 0; ii < 200; ii++)
  { // get average self-test values of gyro and acclerometer

    // Read the six raw data registers into data array
    readRegMpu(MPU9250_ACCEL_XOUT_H, rawData, 6);

    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    // Read the six raw data registers sequentially into data array
    readRegMpu(MPU9250_GYRO_XOUT_H, rawData, 6);

    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

    HAL_Delay(1);
  }

  for (int ii = 0; ii < 3; ii++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation

  writeRegMpu(MPU9250_ACCEL_CONFIG, 0);

  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeRegMpu(MPU9250_GYRO_CONFIG, 0);
  HAL_Delay(25); // osDelay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  readRegMpu(MPU9250_SELF_TEST_X_ACCEL, &selfTest[0]);
  readRegMpu(MPU9250_SELF_TEST_Y_ACCEL, &selfTest[1]);
  readRegMpu(MPU9250_SELF_TEST_Z_ACCEL, &selfTest[2]);
  readRegMpu(MPU9250_SELF_TEST_X_GYRO, &selfTest[3]);
  readRegMpu(MPU9250_SELF_TEST_Y_GYRO, &selfTest[4]);
  readRegMpu(MPU9250_SELF_TEST_Z_GYRO, &selfTest[5]);

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  float result[6] = {0};
  for (int i = 0; i < 3; i++)
  {
    result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;         // Report percent differences
    result[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }
  for (int i = 0; i < 6; i++)
  {
    DEBUG_LOG("%.4f ", result[i]);
  }
  DEBUG_LOG("\r\nSelf test completed\r\n");
}