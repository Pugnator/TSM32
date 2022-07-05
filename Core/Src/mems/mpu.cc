
#include "mpu.h"

MPU9250::MPU9250(I2C_HandleTypeDef *dev)
{
  i2c = dev;
  _ok = false;
  aMult = 0;
  gMult = 0;
  mMult = 0;

  _err_measure = 0;
  _err_estimate = 0;
  _q = 0;
  _current_estimate = 0;
  _last_estimate = 0;
  _kalman_gain = 0;
  _ok = initacc();
  _ok = initmag();
  /* Calculate multiplicators */
  aMult = 2.0f / 32768.0f;
  gMult = 250.0f / 32768.0f;
  mMult = 10.0f * 4912.0f / 32768.0f;
  kalmanInit(2, 2, 0.01);
}

MPU9250::~MPU9250()
{
}

bool MPU9250::ok()
{
  return _ok;
}

void MPU9250::reset()
{

  write(MPU9250_USER_CTRL, 0); // disable internal I2C bus
  // reset device
  write(MPU9250_PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
  write(MPU9250_USER_CTRL, 0x20);  // re-enable internal I2C bus
  HAL_Delay(100);                  // Wait for all registers to reset
}

bool MPU9250::write(uint8_t reg, uint8_t byte)
{
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, reg, 1, &byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }
  return true;
}

bool MPU9250::read(uint8_t reg, uint8_t *byte)
{
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, reg, 1, byte, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    return false;
  }
  return true;
}

bool MPU9250::read(uint8_t reg, uint8_t *byte, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, reg, 1, byte, 1, 1000) != HAL_OK)
    {
      I2C_ClearBusyFlagErratum(i2c, 1000);
      return false;
    }
  }
  return true;
}

bool MPU9250::initacc()
{
  if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR, 10, 100))
  {
    PrintF("Accelerometer is online\r\n");
  }
  else
  {
    PrintF("Accelerometer is not online\r\n");
    return false;
  }
  uint8_t wai = 0;
  if (!read(MPU9250_WHO_AM_I, &wai))
  {
    return false;
  }
  if (0x71 == wai)
  {
    PrintF("MPU 9250 is detected\r\n");
  }
  else
  {
    return false;
  }

  // Clear sleep mode bit (6), enable all sensors
  if (!write(MPU9250_PWR_MGMT_1, 0x00))
  {
    return false;
  }
  HAL_Delay(100); // Wait for all registers to reset

  // get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  if (!write(MPU9250_PWR_MGMT_1, 0x01))
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
  write(MPU9250_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  write(MPU9250_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
                                   // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t data = 0;
  read(MPU9250_GYRO_CONFIG, &data); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  data &= ~0x02;     // Clear Fchoice bits [1:0]
  data &= ~0x18;     // Clear AFS bits [4:3]
  data |= 0x00 << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  write(MPU9250_GYRO_CONFIG, data); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  read(MPU9250_ACCEL_CONFIG, &data); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  data &= ~0x18;                     // Clear AFS bits [4:3]
  data |= 0x00 << 3;                 // Set full scale range for the accelerometer
  write(MPU9250_ACCEL_CONFIG, data); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  read(MPU9250_ACCEL_CONFIG_2, &data); // get current ACCEL_CONFIG2 register value
  data &= ~0x0F;                       // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  data |= 0x03;                        // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  write(MPU9250_ACCEL_CONFIG_2, data); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  write(MPU9250_INT_PIN_CFG, 0x22);
  write(MPU9250_INT_ENABLE, 0x01);
  PrintF("MPU is up\r\n");
  return true;
}

bool MPU9250::initmag()
{
  /* Check if device connected */
  if (HAL_OK == HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR_MAG, 10, 100))
  {
    PrintF("MAG is online\r\n");
  }
  else
  {
    PrintF("MAG is not online\r\n");
    return false;
  }

  write(AK9863_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  write(AK9863_CNTL1, 0x0F); // Enter Fuse ROM access mode
  HAL_Delay(10);
  write(AK9863_CNTL1, 0x00); // Power down magnetometer
  HAL_Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // Set magnetometer data resolution and sample ODR
  write(AK9863_CNTL1, 1 << 4 | 2);
  HAL_Delay(10);
  PrintF("MAG is up\r\n");
  return true;
}

bool MPU9250::ready()
{
  uint8_t data = 0;
  if (!read(MPU9250_INT_STATUS, &data))
  {
    return false;
  }

  if (data & 0x01)
  {
    return true;
  }
  return false;
}

bool MPU9250::readA()
{
  return true;
}

bool MPU9250::readM()
{
  return true;
}

bool MPU9250::readG()
{
  uint8_t data[6];
  read(MPU9250_ACCEL_XOUT_H, data, 6);

  float Gx_Raw = ((int16_t)data[0] << 8) | data[1];
  float Gy_Raw = ((int16_t)data[2] << 8) | data[3];
  float Gz_Raw = ((int16_t)data[4] << 8) | data[5];

  float Gx = Gx_Raw * gMult;
  float Gy = Gy_Raw * gMult;
  float Gz = Gz_Raw * gMult;
  PrintF("%f, %f, %f\r\n", Gx, Gy, Gz);
  return true;
}

void MPU9250::kalmanInit(float mea_e, float est_e, float q)
{
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float MPU9250::updateEstimate(float mea)
{
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void MPU9250::setMeasurementError(float mea_e)
{
  _err_measure = mea_e;
}

void MPU9250::setEstimateError(float est_e)
{
  _err_estimate = est_e;
}

void MPU9250::setProcessNoise(float q)
{
  _q = q;
}

float MPU9250::getKalmanGain()
{
  return _kalman_gain;
}

float MPU9250::getEstimateError()
{
  return _err_estimate;
}