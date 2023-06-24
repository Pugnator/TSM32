#include "tsm.h"
#include "mpu.h"
#include "crc.h"

#define EEPROM_MPU_CALIB_ADDRESS 0

MPU9250::MPU9250(I2C_HandleTypeDef *dev, bool dmpEnable)
{
  mpuDeselect();
  i2c = dev;
  isCalibration_ = false;
  dmpEnabled_ = dmpEnable;
  chipTemperature_ = 0;
  magFactoryCorrX = 0;
  magFactoryCorrY = 0;
  magFactoryCorrZ = 0;
  magScaleX = 1;
  magScaleY = 1;
  magScaleZ = 1;

  magOffsetX = 0;
  magOffsetY = 0;
  magOffsetZ = 0;
  // used to calculate offset/bias
  magMaxX = 0;
  magMaxY = 0;
  magMaxZ = 0;
  magMinX = 0;
  magMinY = 0;
  magMinZ = 0;

  accMaxX = 0;
  accMaxY = 0;
  accMaxZ = 0;
  accMinX = 0;
  accMinY = 0;
  accMinZ = 0;

  gyroMaxX = 0;
  gyroMaxY = 0;
  gyroMaxZ = 0;
  gyroMinX = 0;
  gyroMinY = 0;
  gyroMinZ = 0;

  /* Calculate multiplicators */
  aMult = 2.0f / 32768.0f;
  gMult = 250.0f / 32768.0f;
  gSensF = 10.0f * 4912.0f / 32768.0f;

  if (dmpEnabled_)
  {
    DEBUG_LOG("Configuring Digital Motion Processor.\r\n");
    ok_ = startDMP();
    return;
  }

  DEBUG_LOG("Configuring basic mode.\r\n");

  ok_ = reset();

  if (!ok_)
  {
    return;
  }

  if (!configureGyroscope())
  {
    DEBUG_LOG("Failed to configure gyroscope.\r\n");
    ok_ = false;
  }
  if (!configureAccelerometer())
  {
    DEBUG_LOG("Failed to configure accelerometer.\r\n");
    ok_ = false;
  }
  if (!configureMagnetometer())
  {
    DEBUG_LOG("Failed to configure magnetometer.\r\n");
    ok_ = false;
  }
  if (!ok_)
  {
    // NVIC_SystemReset();
  }
}

MPU9250::~MPU9250()
{
}

bool MPU9250::dmpEnabled()
{
  return dmpEnabled_;
}

bool MPU9250::ok()
{
  return ok_;
}

bool MPU9250::startDMP()
{
  // Configure Power Management Registers
  mpuWrite(MPU9250_PWR_MGMT_1, 0);
  mpuWrite(MPU9250_PWR_MGMT_2, 0);
  // Configure Gyroscope Parameters
  mpuWrite(MPU9250_CONFIG, 0x03);
  mpuWrite(MPU9250_GYRO_CONFIG, 0x18);
  // Configure Accelerometer Parameters
  mpuWrite(MPU9250_ACCEL_CONFIG, 0);
  // Configure FIFO and Interrupts
  // Now DMP has the control over the FIFO
  mpuWrite(MPU9250_FIFO_EN, 0);
  mpuWrite(MPU9250_INT_ENABLE, 0);
  // Reset the FIFO
  mpuWrite(MPU9250_USER_CTRL, 0x04);
  // Configure Sensor Sample Rate
  mpuWrite(MPU9250_SMPLRT_DIV, 0x04);
  // Load Firmware One Byte at a Time
  uint32_t address = 0;
  uint8_t temp = 0;
  while (address < dmpFirmwareSize)
  {
    mpuWrite(MPU9250_DMP_CTRL_1, address >> 8);
    mpuWrite(MPU9250_DMP_CTRL_2, address & 0xFF);
    mpuWrite(MPU9250_DMP_CTRL_3, dmpFirmware[address]);
    address++;
  }
  DEBUG_LOG("Uploaded %u bytes to MPU\r\n", address);
  // Load Firmware Start Value
  mpuWrite(MPU9250_FW_START_1, MPU9250_DMP_JUMP_ADDRESS >> 8);
  mpuWrite(MPU9250_FW_START_2, MPU9250_DMP_JUMP_ADDRESS & 0xFF);

  // Enable I2C bypass in order to use AK9638 via I2C
  mpuWrite(MPU9250_INT_PIN_CFG, 0x2);

  if (!configureMagnetometer())
  {
    DEBUG_LOG("Failed to configure magnetometer.\r\n");
    return false;
  }

  mpuWrite(MPU9250_USER_CTRL, 0x20);
  mpuWrite(MPU9250_USER_CTRL, 0x24);
  mpuWrite(MPU9250_USER_CTRL, 0x20);
  mpuWrite(MPU9250_USER_CTRL, 0xE8);
  HAL_Delay(10);
  mpuWrite(MPU9250_INT_ENABLE, 0x02);

  return true;
}

void MPU9250::getQuaternion(Quaternion &q)
{
  q.w = (float)(quat[0] >> 16) / 16384.0f;
  q.x = (float)(quat[1] >> 16) / 16384.0f;
  q.y = (float)(quat[2] >> 16) / 16384.0f;
  q.z = (float)(quat[3] >> 16) / 16384.0f;
}

void MPU9250::getYawPitchRoll(float *ypr, Quaternion &q)
{
  // yaw: (about Z axis)
  ypr[0] = atan2(2.0f * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  // pitch: (nose up/down, about Y axis)
  ypr[1] = -asin(2.0f * (q.x * q.z - q.w * q.y));
  // roll: (tilt left/right, about X axis)
  ypr[2] = atan2(2.0f * (q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

void MPU9250::getEuler(float *output, Quaternion &q)
{
  // psi
  output[0] = atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
  // theta
  output[1] = -asin(2 * q.x * q.z + 2 * q.w * q.y);
  // phi
  output[2] = atan2(2 * q.y * q.z - 2 * q.w * q.x, 2 * q.w * q.w + 2 * q.z * q.z - 1);
}

void MPU9250::yprToDegrees(float *ypr, float *xyz)
{
  const float radians2degrees = 180.0 / M_PI;
  for (int i = 0; i < 3; i++)
  {
    xyz[i] = ypr[i] * radians2degrees;
  }
  if (xyz[0] < -180)
    xyz[0] += 360;
}

void MPU9250::yprToRadians(float *ypr, float *xyz)
{
  const float degrees2radians = M_PI / 180.0;
  for (int i = 0; i < 3; i++)
  {
    ypr[i] = xyz[i] * degrees2radians;
  }
}

void MPU9250::getGravity(VectorFloat &v, Quaternion &q)
{
  v.x = 2 * (q.x * q.z - q.w * q.y);
  v.y = 2 * (q.w * q.x + q.y * q.z);
  v.z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
}

void MPU9250::getAccel(VectorInt16 &v)
{
  v.x = accel[0];
  v.y = accel[1];
  v.z = accel[2];
}

void MPU9250::getLinearAccel(VectorInt16 &v, VectorInt16 &vRaw, VectorFloat &gravity)
{
  // get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is +-2g)
  v.x = vRaw.x - gravity.x * 16384;
  v.y = vRaw.y - gravity.y * 16384;
  v.z = vRaw.z - gravity.z * 16384;
}

bool MPU9250::staticCalibration(Eeprom *mem)
{
  DEBUG_LOG("Starting calibration\r\n");
  isCalibration_ = true;
  Axes3D temp;
  for (uint32_t i = 1 * 3000; i > 0; i--)
  {
    readMagAxis(temp);
  }
  DEBUG_LOG("Mag offsets: %.2f, %.2f, %.2f\r\n", magOffsetX, magOffsetY, magOffsetZ);

  float x = 0;
  float y = 0;
  float z = 0;
  for (uint32_t i = 25000; i > 0; i--)
  {
    readAccelAxis(temp);
    x += temp.x;
    y += temp.y;
    z += temp.z;
  }
  x /= 25000.0;
  y /= 25000.0;
  z /= 25000.0;
  DEBUG_LOG("Acc avareges: %.2f, %.2f, %.2f\r\n", x, y, z);
  DEBUG_LOG("Acc offsets: %.2f, %.2f, %.2f\r\n", accOffsetX, accOffsetY, accOffsetZ);

  x = 0;
  y = 0;
  z = 0;
  for (uint32_t i = 25000; i > 0; i--)
  {
    readGyroAxis(temp);
    x += temp.x;
    y += temp.y;
    z += temp.z;
  }
  isCalibration_ = false;
  x /= 25000.0;
  y /= 25000.0;
  z /= 25000.0;
  DEBUG_LOG("Gyro avareges: %.2f, %.2f, %.2f\r\n", x, y, z);
  DEBUG_LOG("Gyro offsets: %.2f, %.2f, %.2f\r\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);

  if (mem)
  {
    static float calibData[9] = {0};
    calibData[0] = magOffsetX;
    calibData[1] = magOffsetY;
    calibData[2] = magOffsetZ;

    calibData[3] = accOffsetX;
    calibData[4] = accOffsetY;
    calibData[5] = accOffsetZ;

    calibData[6] = gyroOffsetX;
    calibData[7] = gyroOffsetY;
    calibData[8] = gyroOffsetZ;

    uint32_t crc = HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t *>(calibData), sizeof(calibData) / sizeof(uint32_t));
    DEBUG_LOG("CRC = %.8X\r\n", crc);

    if (mem->write(EEPROM_MPU_CALIB_ADDRESS, reinterpret_cast<uint8_t *>(calibData), sizeof(calibData)))
    {
      DEBUG_LOG("Saved to the eeprom\r\n");
    }
    else
    {
      DEBUG_LOG("Failed to save to the eeprom\r\n");
    }

    if (mem->write(EEPROM_MPU_CALIB_ADDRESS + sizeof(calibData), reinterpret_cast<uint8_t *>(&crc), sizeof(crc)))
    {
      DEBUG_LOG("Saved crc\r\n");
    }
  }
  DEBUG_LOG("Finished calibration\r\n");
  return true;
}

bool MPU9250::loadCalibration(Eeprom *mem)
{
  static float calibData[9] = {};
  if (mem->read(EEPROM_MPU_CALIB_ADDRESS, reinterpret_cast<uint8_t *>(calibData), sizeof(calibData)))
  {
    DEBUG_LOG("Load from the eeprom\r\n");
  }
  else
  {
    DEBUG_LOG("Failed to load from the eeprom\r\n");
    return false;
  }

  uint32_t crcRead = 0;
  if (mem->read(EEPROM_MPU_CALIB_ADDRESS + sizeof(calibData), reinterpret_cast<uint8_t *>(&crcRead), sizeof(crcRead)))
  {
    DEBUG_LOG("Loaded crc\r\n");
  }
  else
  {
    DEBUG_LOG("Failed to load crc\r\n");
    return false;
  }

  DEBUG_LOG("CRC read = %.8X\r\n", crcRead);
  uint32_t crcNew = HAL_CRC_Calculate(&hcrc, reinterpret_cast<uint32_t *>(calibData), sizeof(calibData) / sizeof(uint32_t));
  DEBUG_LOG("CRC calculated = %.8X\r\n", crcNew);
  if (crcNew == 0xFFFFFFFF || crcRead == 0xFFFFFFFF)
  {
    DEBUG_LOG("No calibration data found\r\n");
    return false;
  }

  if (crcRead != crcNew)
  {
    DEBUG_LOG("CRC of calibration block is incorrect\r\n");
    return false;
  }

  DEBUG_LOG("Read valid calibration block\r\n");

  magOffsetX = calibData[0];
  magOffsetY = calibData[1];
  magOffsetZ = calibData[2];

  accOffsetX = calibData[3];
  accOffsetY = calibData[4];
  accOffsetZ = calibData[5];

  gyroOffsetX = calibData[6];
  gyroOffsetY = calibData[7];
  gyroOffsetZ = calibData[8];

  DEBUG_LOG("Mag offsets: %.2f, %.2f, %.2f\r\n", magOffsetX, magOffsetY, magOffsetZ);
  DEBUG_LOG("Acc offsets: %.2f, %.2f, %.2f\r\n", accOffsetX, accOffsetY, accOffsetZ);
  DEBUG_LOG("Gyro offsets: %.2f, %.2f, %.2f\r\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  return true;
}

bool MPU9250::reset()
{
  if (!mpuWrite(MPU9250_USER_CTRL, 0)) // disable internal I2C bus
  {
    return false;
  }
  // reset device
  mpuWrite(MPU9250_PWR_MGMT_1, 0x80); // Set bit 7 to reset MPU9250
  mpuWrite(MPU9250_USER_CTRL, 0x20);  // re-enable internal I2C bus
  HAL_Delay(100);                     // Wait for all registers to reset
#if defined(IMU_I2C_MODE)
  if (HAL_OK != HAL_I2C_IsDeviceReady(i2c, MPU9250_I2C_ADDR, 10, 100))
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Accelerometer is not online\r\n");
    return false;
  }
#endif
  uint8_t wai = 0;
  if (!mpuRead(MPU9250_WHO_AM_I, &wai))
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
  if (!mpuWrite(MPU9250_PWR_MGMT_1, 0x00))
  {
    return false;
  }
  HAL_Delay(100); // Wait for all registers to reset

  // get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  if (!mpuWrite(MPU9250_PWR_MGMT_1, 0x01))
  {
    return false;
  }
  HAL_Delay(200);
  return true;
}

bool MPU9250::fifoReset()
{
  mpuWrite(MPU9250_FIFO_EN, 0);
  mpuWrite(MPU9250_INT_ENABLE, 0);
  mpuWrite(MPU9250_USER_CTRL, 0);
  HAL_Delay(50);
  mpuWrite(MPU9250_USER_CTRL, 0x20);
  mpuWrite(MPU9250_USER_CTRL, 0x24);
  mpuWrite(MPU9250_USER_CTRL, 0x20);
  mpuWrite(MPU9250_USER_CTRL, 0xE8);

  mpuWrite(MPU9250_INT_ENABLE, 0x02);
  return true;
}

void MPU9250::parseDmpPacket()
{
  quat[0] = ((int32_t)fifoBuffer[0] << 24) | ((int32_t)fifoBuffer[1] << 16) | ((int32_t)fifoBuffer[2] << 8) | fifoBuffer[3];
  quat[1] = ((int32_t)fifoBuffer[4] << 24) | ((int32_t)fifoBuffer[5] << 16) | ((int32_t)fifoBuffer[6] << 8) | fifoBuffer[7];
  quat[2] = ((int32_t)fifoBuffer[8] << 24) | ((int32_t)fifoBuffer[9] << 16) | ((int32_t)fifoBuffer[10] << 8) | fifoBuffer[11];
  quat[3] = ((int32_t)fifoBuffer[12] << 24) | ((int32_t)fifoBuffer[13] << 16) | ((int32_t)fifoBuffer[14] << 8) | fifoBuffer[15];
  uint8_t index = 16;
  accel[0] = ((int16_t)fifoBuffer[index] << 8) | fifoBuffer[index + 1];
  accel[1] = ((int16_t)fifoBuffer[index + 2] << 8) | fifoBuffer[index + 3];
  accel[2] = ((int16_t)fifoBuffer[index + 4] << 8) | fifoBuffer[index + 5];
  index += 6;
  gyro[0] = ((int16_t)fifoBuffer[index] << 8) | fifoBuffer[index + 1];
  gyro[1] = ((int16_t)fifoBuffer[index + 2] << 8) | fifoBuffer[index + 3];
  gyro[2] = ((int16_t)fifoBuffer[index + 4] << 8) | fifoBuffer[index + 5];
}

uint16_t MPU9250::fifoRead()
{
  uint16_t dataSize = fifoDataReady();
  if (!dataSize)
    return 0;

  if (dataSize >= DMP_FIFO_SIZE)
  {
    fifoReset();
    return 0xFFFF;
  }

  if (dataSize < DMP_PACKET_SIZE)
  {
    return 0;
  }

  if (!mpuRead(MPU9250_FIFO_R_W, fifoBuffer, DMP_PACKET_SIZE))
  {
    return 0xFFFF;
  }
  parseDmpPacket();
  return DMP_PACKET_SIZE;
}

uint16_t MPU9250::fifoDataReady()
{
  uint8_t dataH = 0;
  // Latch data first and ready high bits
  if (!mpuRead(MPU9250_FIFO_COUNTH, &dataH))
  {
    return 0xFFFF;
  }
  uint8_t dataL = 0;
  if (!mpuRead(MPU9250_FIFO_COUNTL, &dataL))
  {
    return 0xFFFF;
  }
  uint16_t dataSize = (dataH << 8) | dataL;
  return dataSize;
}

InterruptSource MPU9250::interruptStatus()
{
  uint8_t data = 0;
  if (!mpuRead(MPU9250_INT_STATUS, &data))
  {
    return InterruptSource::NoInterrupt;
  }

  if (data & 0x01)
  {
    return InterruptSource::DataReady;
  }

  if (data & 0x02)
  {
    return InterruptSource::DmpInterrupt;
  }

  if (data & 0x08)
  {
    return InterruptSource::FifoOverflow;
  }

  return InterruptSource::NoInterrupt;
}

float MPU9250::getTemperature()
{
  return chipTemperature_;
}

bool MPU9250::enableDMP(bool enable)
{  
  uint8_t out = 0;
  if (!mpuRead(MPU9250_USER_CTRL, &out))
  {
    return false;
  }
  //7th bit is a DMP enable flag
  out = enable ? (out | (1 << 7)) : (out & ~(1 << 7));
  if(!mpuWrite(MPU9250_USER_CTRL, out))
  {
    return false;
  }
  
  mpuWrite(MPU9250_INT_ENABLE, enable ? 0x02 : 0);
  dmpEnabled_ = enable;
  return true;
}

bool MPU9250::resetDMP()
{
  uint8_t out = 0;
  if (!mpuRead(MPU9250_USER_CTRL, &out))
  {
    return false;
  }
  //3rd bit is a DMP reset, set it to reset
  out = out | (1 << 3);
  fifoReset();
  return mpuWrite(MPU9250_USER_CTRL, out);  
}