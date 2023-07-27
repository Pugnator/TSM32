#include "tsm.h"
#include "imu_i2c.h"
namespace Mpu9250
{
  bool Mpu9250I2c::mpuWrite(uint8_t address, uint8_t *byte, uint32_t len)
  {
    if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
    {
      DEBUG_LOG("I2C bus write error: address: %#.2X\r\n");
      return false;
    }
    return true;
  }

  bool Mpu9250I2c::mpuWrite(uint8_t address, uint8_t byte)
  {
    uint8_t temp = byte;
    uint8_t *data = &temp;
    if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR, address, 1, data, 1, 1000) != HAL_OK)
    {
      return false;
    }
    return true;
  }

  bool Mpu9250I2c::mpuRead(uint8_t address, uint8_t *byte, uint32_t len)
  {
    if (HAL_I2C_Mem_Read(bus_, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
    {
      return false;
    }
    return true;
  }

  bool Mpu9250I2c::magWriteRegI2c(uint8_t reg, uint8_t *byte, uint32_t len)
  {
    if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 1000) != HAL_OK)
    {
      DEBUG_LOG("Failed to write to MAG\r\n");
      return false;
    }
    return true;
  }

  bool Mpu9250I2c::magWriteRegI2c(uint8_t reg, uint8_t byte)
  {
    uint8_t temp = byte;
    uint8_t *data = &temp;

    if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, data, 1, 1000) != HAL_OK)
    {
      DEBUG_LOG("Failed to write to MAG\r\n");
      return false;
    }

    return true;
  }

  bool Mpu9250I2c::magReadRegI2c(uint8_t reg, uint8_t *byte, uint32_t len, uint32_t timeout_ms)
  {
    uint32_t start_time = HAL_GetTick(); // Record the start time

    while (HAL_GetTick() - start_time < timeout_ms)
    {
      if (HAL_I2C_Mem_Read(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 100) == HAL_OK)
      {
        return true; // Read operation successful, return true
      }
    }
    DEBUG_LOG("Failed to read MAG\r\n");

    return false; // Timeout reached, return false
  }

  bool Mpu9250I2c::magWriteRegI2cSlave(uint8_t address, uint8_t *byte, uint32_t len)
  {
    static const uint8_t magAddress = MPU9250_I2C_SLAVE_ADDR_MAG;
    if (!mpuWrite(MPU9250_I2C_SLV4_ADDR, magAddress))
      return false;

    HAL_Delay(1);

    for (uint8_t i = 0; i < len; i++)
    {
      if (!mpuWrite(MPU9250_I2C_SLV4_REG, address + i))
        return false;

      HAL_Delay(1);
      if (!mpuWrite(MPU9250_I2C_SLV4_DO, byte[i]))
        return false;

      HAL_Delay(1);
      if (!mpuWrite(MPU9250_I2C_SLV4_CTRL, 0x80)) // Enable
        return false;

      HAL_Delay(1);
      uint8_t status = 0;
      do
      {
        if (!mpuRead(MPU9250_I2C_MST_STATUS, &status))
          return false;

        HAL_Delay(1);
      } while ((status & 0x40) == 0); // Done

      if (status & 0x10) // NACK
        return false;
    }
    return true;
  }
  bool Mpu9250I2c::magWriteRegI2cSlave(uint8_t address, uint8_t byte)
  {
    static const uint8_t magAddress = MPU9250_I2C_SLAVE_ADDR_MAG;
    if (!mpuWrite(MPU9250_I2C_SLV4_ADDR, magAddress))
      return false;

    HAL_Delay(1);
    if (!mpuWrite(MPU9250_I2C_SLV4_REG, address))
      return false;

    HAL_Delay(1);
    if (!mpuWrite(MPU9250_I2C_SLV4_DO, byte))
      return false;

    HAL_Delay(1);
    if (!mpuWrite(MPU9250_I2C_SLV4_CTRL, 0x80)) // Enable
      return false;

    HAL_Delay(1);
    uint8_t status = 0;
    do
    {
      if (!mpuRead(MPU9250_I2C_MST_STATUS, &status))
        return false;

      HAL_Delay(1);
    } while ((status & 0x40) == 0); // Done

    if (status & 0x10) // NACK
      return false;

    return true;
  }
  bool Mpu9250I2c::magReadRegI2cSlave(uint8_t address, uint8_t *byte, uint32_t len, uint32_t timeout_ms)
  {
    static const uint8_t magAddress = MPU9250_I2C_SLAVE_ADDR_MAG | 0x80;

    if (!mpuWrite(MPU9250_I2C_SLV4_ADDR, magAddress))
      return false;

    HAL_Delay(1);

    for (uint8_t i = 0; i < len; i++)
    {
      if (!mpuWrite(MPU9250_I2C_SLV4_REG, address + i))
        return false;

      HAL_Delay(1);

      if (!mpuWrite(MPU9250_I2C_SLV4_CTRL, 0x80)) // Enable
        return false;

      HAL_Delay(1);
      uint8_t status = 0;
      do
      {
        if (!mpuRead(MPU9250_I2C_MST_STATUS, &status))
          return false;

        HAL_Delay(1);
      } while ((status & 0x40) == 0); // Done

      if (!mpuRead(MPU9250_I2C_SLV4_DI, &byte[i]))
        return false;
    }
    return true;
  }

  bool Mpu9250I2c::magWrite(uint8_t reg, uint8_t *byte, uint32_t len)
  {
    if (magMode_ == MagMode::SlaveMode)
    {
      return magWriteRegI2cSlave(reg, byte, len);
    }
    else
    {
      return magWriteRegI2c(reg, byte, len);
    }
  }

  bool Mpu9250I2c::magWrite(uint8_t reg, uint8_t byte)
  {
    if (magMode_ == MagMode::SlaveMode)
    {
      return magWriteRegI2cSlave(reg, byte);
    }
    else
    {
      return magWriteRegI2c(reg, byte);
    }
  }

  bool Mpu9250I2c::magRead(uint8_t reg, uint8_t *byte, uint32_t len, uint32_t timeout_ms)
  {
    if (magMode_ == MagMode::SlaveMode)
    {
      return magReadRegI2cSlave(reg, byte, len, timeout_ms);
    }
    else
    {
      return magReadRegI2c(reg, byte, len, timeout_ms);
    }
  }
}