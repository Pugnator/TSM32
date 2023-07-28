#include "tsm.h"
#include "mpu9250_base.h"

namespace Mpu9250
{
  bool Mpu9250base::ok()
  {
    return ok_;
  }

  void Mpu9250base::getDmpQuaternion(Quaternion &q)
  {
    q.w = (float)(quat[0] >> 16) / 16384.0f;
    q.x = (float)(quat[1] >> 16) / 16384.0f;
    q.y = (float)(quat[2] >> 16) / 16384.0f;
    q.z = (float)(quat[3] >> 16) / 16384.0f;
  }

  void Mpu9250base::getDmpAccel(VectorInt16 &v)
  {
    v.x = accel[0];
    v.y = accel[1];
    v.z = accel[2];
  }

  bool Mpu9250base::mpuWriteBit(uint8_t address, uint8_t bitNumber, bool state)
  {
    uint8_t byte;

    if (!mpuRead(address, &byte, 1))
      return false;

    if (state)
      byte |= (1 << bitNumber);
    else
      byte &= ~(1 << bitNumber);

    if (!mpuWrite(address, byte))
      return false;

    return true;
  }

  bool Mpu9250base::mpuReadBit(uint8_t address, uint8_t bitNumber, bool *state)
  {
    uint8_t byte;

    if (!mpuRead(address, &byte, 1))
      return false;

    *state = (byte >> bitNumber) & 0x01;

    return true;
  }

  bool Mpu9250base::mpuWriteBits(uint8_t address, uint8_t startBit, uint8_t length, uint8_t data)
  {
    uint8_t byte;

    if (!mpuRead(address, &byte, 1))
      return false;

    // Calculate the bit mask
    uint8_t bitMask = ((1 << length) - 1) << startBit;

    // Clear the bits within the mask
    byte &= ~bitMask;

    // Shift and mask the data to fit into the target bit positions
    byte |= (data << startBit) & bitMask;

    if (!mpuWrite(address, byte))
      return false;

    return true;
  }

  bool Mpu9250base::mpuReadBits(uint8_t address, uint8_t startBit, uint8_t length, uint8_t *data)
  {
    uint8_t byte;

    if (!mpuRead(address, &byte, 1))
      return false;

    // Calculate the bit mask
    uint8_t bitMask = ((1 << length) - 1) << startBit;

    // Extract the bits within the mask
    *data = (byte & bitMask) >> startBit;

    return true;
  }

  bool Mpu9250base::fifoReset()
  {
    if (!mpuWrite(MPU9250_FIFO_EN, 0))
      return false;
    if (!mpuWrite(MPU9250_INT_ENABLE, 0))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0))
      return false;
    HAL_Delay(50);
    if (!mpuWrite(MPU9250_USER_CTRL, 0x20))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0x24))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0x20))
      return false;
    if (!mpuWrite(MPU9250_USER_CTRL, 0xE8))
      return false;

    mpuWrite(MPU9250_INT_ENABLE, 0x02);
    return true;
  }

  uint16_t Mpu9250base::dmpFifoRead()
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

    // DEBUG_LOG("Got packet, %u\r\n", dataSize);

    if (!mpuRead(MPU9250_FIFO_R_W, fifoBuffer, DMP_PACKET_SIZE))
    {
      return 0xFFFF;
    }
    parseDmpPacket();
    return DMP_PACKET_SIZE;
  }

  uint16_t Mpu9250base::fifoDataReady()
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
    captureTime_ = HAL_GetTick();
    uint16_t dataSize = (dataH << 8) | dataL;
    return dataSize;
  }

  uint32_t Mpu9250base::captureTime()
  {
    return captureTime_;
  }

  InterruptSource Mpu9250base::interruptStatus()
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

  float Mpu9250base::getTemperature()
  {
    return chipTemperature_;
  }

  bool Mpu9250base::setMagnetometerAsSlave()
  {
    DEBUG_LOG("Magnetometer as a slave I2C device.\r\n");
    // Setup AK8963 as a slave I2C device

    if (!mpuWrite(MPU9250_I2C_SLV0_ADDR, MPU9250_I2C_ADDR_MAG | 0x80))
      return false;

    if (!mpuWrite(MPU9250_I2C_SLV0_REG, 0x01))
      return false;
    if (!mpuWrite(MPU9250_I2C_SLV0_CTRL, 0xDA))
      return false;

    if (!mpuWrite(MPU9250_I2C_SLV2_ADDR, MPU9250_I2C_ADDR_MAG))
      return false;
    if (!mpuWrite(MPU9250_I2C_SLV2_REG, 0x0A))
      return false;
    if (!mpuWrite(MPU9250_I2C_SLV2_CTRL, 0x81))
      return false;
    if (!mpuWrite(MPU9250_I2C_SLV2_DO, 0x01))
      return false;

    // setup I2C timing/delay control
    if (!mpuWrite(MPU9250_I2C_SLV4_CTRL, 0x18))
      return false;
    if (!mpuWrite(MPU9250_I2C_MST_DELAY_CTRL, 0x05))
      return false;

    return true;
  }
}