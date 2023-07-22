
#include <cstdint>
#include "mpudefs.h"
#include "mpu9250_base.h"

namespace Mpu9250
{
  bool Mpu9250base::startDMP()
  {
    // Configure Power Management Registers
    if (!mpuWrite(MPU9250_PWR_MGMT_1, 0))
      return false;
    if (!mpuWrite(MPU9250_PWR_MGMT_2, 0))
      return false;
    // Configure Gyroscope Parameters
    if (!mpuWrite(MPU9250_CONFIG, 0x03))
      return false;
    if (!mpuWrite(MPU9250_GYRO_CONFIG, 0x18))
      return false;
    // Configure Accelerometer Parameters
    if (!mpuWrite(MPU9250_ACCEL_CONFIG, 0))
      return false;
    // Configure FIFO and Interrupts
    // Now DMP has the control over the FIFO
    if (!mpuWrite(MPU9250_FIFO_EN, 0))
      return false;
    if (!mpuWrite(MPU9250_INT_ENABLE, 0))
      return false;
    // Reset the FIFO
    if (!mpuWrite(MPU9250_USER_CTRL, 0x04))
      return false;
    // Configure Sensor Sample Rate
    if (!mpuWrite(MPU9250_SMPLRT_DIV, 0x04))
      return false;

    // Enable I2C bypass in order to use AK9638 via I2C
    if (!mpuWrite(MPU9250_INT_PIN_CFG, 0x22))
      return false;

    if (!configureMagnetometer())
    {
      DEBUG_LOG("Failed to configure magnetometer.\r\n");
      ok_ = false;
    }

    // magSetMode(MAG_MODE_PD);

    // Load Firmware One Byte at a Time
    uint32_t address = 0;
    while (address < dmpFirmwareSize)
    {
      if (!mpuWrite(MPU9250_DMP_CTRL_1, address >> 8))
        return false;
      if (!mpuWrite(MPU9250_DMP_CTRL_2, address & 0xFF))
        return false;
      if (!mpuWrite(MPU9250_DMP_CTRL_3, dmpFirmware[address]))
        return false;
      address++;
    }
    DEBUG_LOG("Uploaded %u bytes to MPU\r\n", address);

    // Load Firmware Start Value
    if (!mpuWrite(MPU9250_FW_START_1, MPU9250_DMP_JUMP_ADDRESS >> 8))
      return false;
    if (!mpuWrite(MPU9250_FW_START_2, MPU9250_DMP_JUMP_ADDRESS & 0xFF))
      return false;

    if (!mpuWrite(MPU9250_USER_CTRL, 0x20))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0x24))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0x20))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0xE8))
      return false;

    if (!setMagnetometerAsSlave())
      return false;

    HAL_Delay(10);

    if (!mpuWrite(MPU9250_INT_ENABLE, 0x02))
      return false;

    return true;
  }

  bool Mpu9250base::dmpEnabled()
  {
    return useDmp_;
  }

  void Mpu9250base::parseDmpPacket()
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

  bool Mpu9250base::enableDMP(bool enable)
  {
    uint8_t out = 0;
    if (!mpuRead(MPU9250_USER_CTRL, &out))
    {
      return false;
    }
    // 7th bit is a DMP enable flag
    out = enable ? (out | (1 << 7)) : (out & ~(1 << 7));
    if (!mpuWrite(MPU9250_USER_CTRL, out))
    {
      return false;
    }

    mpuWrite(MPU9250_INT_ENABLE, enable ? 0x02 : 0);
    useDmp_ = true;
    return true;
  }

  bool Mpu9250base::resetDMP()
  {
    uint8_t out = 0;
    if (!mpuRead(MPU9250_USER_CTRL, &out))
    {
      return false;
    }
    // 3rd bit is a DMP reset, set it to reset
    out = out | (1 << 3);
    fifoReset();
    return mpuWrite(MPU9250_USER_CTRL, out);
  }
}