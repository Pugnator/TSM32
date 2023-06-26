#include "mpu.h"
#include "MEMS/types.h"
#include <algorithm>

bool MPU9250::configureAccelerometer()
{
  uint8_t val = 0;
  uint8_t *temp_ = &val;
  // Set accelerometer full-scale range configuration
  mpuRead(MPU9250_ACCEL_CONFIG, temp_); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  *temp_ &= ~0x18;                                        // Clear AFS bits [4:3]
  *temp_ |= static_cast<int>(AHRS::Ascale::Scale2G) << 3; // Set full scale range for the accelerometer

  mpuWrite(MPU9250_ACCEL_CONFIG, std::move(*temp_)); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  mpuRead(MPU9250_ACCEL_CONFIG_2, temp_); // get current ACCEL_CONFIG2 register value
  *temp_ &= ~0x0F;                           // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  *temp_ |= 0x03;                            // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  mpuWrite(MPU9250_ACCEL_CONFIG_2, std::move(*temp_)); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  // 00000010
  mpuWrite(MPU9250_INT_PIN_CFG, 0x2);
  // 00010001
  mpuWrite(MPU9250_INT_ENABLE, 0);
  DEBUG_LOG("MPU is up\r\n");
  return true;
}

void MPU9250::accAutoOffset(VectorFloat &axes)
{
  if (!isCalibration_)
  {
    return;
  }
  // Update accMaxX, accMaxY, accMaxZ with the maximum values
  if (axes.x > accMaxX)
  {
    accMaxX = axes.x;
  }
  if (axes.y > accMaxY)
  {
    accMaxY = axes.y;
  }
  if (axes.z > accMaxZ)
  {
    accMaxZ = axes.z;
  }

  // Update accMinX, accMinY, accMinZ with the minimum values
  if (axes.x < accMinX)
  {
    accMinX = axes.x;
  }
  if (axes.y < accMinY)
  {
    accMinY = axes.y;
  }
  if (axes.z < accMinZ)
  {
    accMinZ = axes.z;
  }

  accOffsetX = (accMaxX + accMinX) / 2.0;
  accOffsetY = (accMaxY + accMinY) / 2.0;
  accOffsetZ = (accMaxZ + accMinZ) / 2.0;
}

bool MPU9250::readAccelAxis(VectorFloat &result)
{
  uint8_t data[8];
  mpuRead(MPU9250_ACCEL_XOUT_H, data, 6);

  float Ax_Raw = ((int16_t)data[0] << 8) | data[1];
  float Ay_Raw = ((int16_t)data[2] << 8) | data[3];
  float Az_Raw = ((int16_t)data[4] << 8) | data[5];

  uint16_t _temp = (data[6] << 8 | data[7]);
  chipTemperature_ = ((_temp - 21.0f) / 333.87f) + 21.0f;

  result.x = (float)Ax_Raw * aMult;
  result.y = (float)Ay_Raw * aMult;
  result.z = (float)Az_Raw * aMult;

  accAutoOffset(result);

  result.x -= accOffsetX;
  result.y -= accOffsetY;
  result.z -= accOffsetZ;

  return true;
}