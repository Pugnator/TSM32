#include "mpu9250_base.h"
#include "types.h"
#include "axis_remap.h"
#include <algorithm>

namespace Mpu9250
{
  bool Mpu9250base::configureAccelerometer()
  {
    uint8_t val = 0;
    uint8_t *temp_ = &val;
    // Set accelerometer full-scale range configuration
    if (!mpuRead(MPU9250_ACCEL_CONFIG, temp_)) // get current ACCEL_CONFIG register value
      return false;
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    *temp_ &= ~0x18;                                        // Clear AFS bits [4:3]
    *temp_ |= static_cast<int>(AHRS::Ascale::Scale2G) << 3; // Set full scale range for the accelerometer

    if (!mpuWrite(MPU9250_ACCEL_CONFIG, std::move(*temp_))) // Write new ACCEL_CONFIG register value
      return false;

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    if (!mpuRead(MPU9250_ACCEL_CONFIG_2, temp_)) // get current ACCEL_CONFIG2 register value
      return false;
    *temp_ &= ~0x0F;          // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    *temp_ |= ACCEL_DLPF_CFG; // Accel rate 1 kHz, bandwidth per ahrs_config.h (default 10.2 Hz)

    if (!mpuWrite(MPU9250_ACCEL_CONFIG_2, std::move(*temp_))) // Write new ACCEL_CONFIG2 register value
      return false;
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled as master
    // 00000010
    return true;
  }

  bool Mpu9250base::readAccelAxis(VectorFloat &result)
  {
    uint8_t data[6];
    if (!mpuRead(MPU9250_ACCEL_XOUT_H, data, 6))
      return false;

    int16_t accX = ((int16_t)data[0] << 8) | data[1];
    int16_t accY = ((int16_t)data[2] << 8) | data[3];
    int16_t accZ = ((int16_t)data[4] << 8) | data[5];

    // NOTE: chip temperature is read separately - the TEMP_OUT registers
    // are not contiguous with the accel block.  Reading them here required
    // a 2-byte over-read into uninitialised stack memory.  See issue #32
    // for the proper readChipTemperature() implementation.

    result = Ahrs::sensorToBodyZ180((float)accX, (float)accY,
                                    (float)accZ, aMult);
    return true;
  }
}
