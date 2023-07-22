#include "tsm.h"
#include "imu_spi.h"

namespace Mpu9250
{
  bool Mpu9250Spi::mpuWrite(uint8_t address, uint8_t *byte, uint32_t len)
  {
    mpuSelect();
    HAL_SPI_Transmit(bus_, &address, 1, 100);
    uint8_t temp_val = 0;
    for (uint32_t i = 0; i < len; i++)
    {
      if (HAL_SPI_TransmitReceive(bus_, &byte[i], &temp_val, 1, 100) != HAL_OK)
      {
        return false;
      }
    }
    mpuDeselect();
    return true;
  }

  bool Mpu9250Spi::mpuWrite(uint8_t address, uint8_t byte)
  {
    mpuSelect();
    if (HAL_SPI_Transmit(bus_, &address, 1, 100) != HAL_OK)
    {
      mpuDeselect();
      return false;
    }

    uint8_t temp = 0;
    if (HAL_SPI_TransmitReceive(bus_, &byte, &temp, 1, 100) != HAL_OK)
    {
      mpuDeselect();
      return false;
    }
    mpuDeselect();
    return true;
  }

  bool Mpu9250Spi::mpuRead(uint8_t address, uint8_t *byte, uint32_t len)
  {
    mpuSelect();
    uint8_t raddr = address | 0x80;
    if (HAL_SPI_Transmit(bus_, &raddr, 1, 100) != HAL_OK)
    {
      mpuDeselect();
      return false;
    }

    uint8_t temp = 0;
    for (uint32_t i = 0; i < len; i++)
    {
      if (HAL_SPI_TransmitReceive(bus_, &temp, &byte[i], 1, 100) != HAL_OK)
      {
        mpuDeselect();
        return false;
      }
    }
    mpuDeselect();
    return true;
  }

  void Mpu9250Spi::mpuDeselect()
  {
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
  }

  void Mpu9250Spi::mpuSelect()
  {
    HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
  }

  bool Mpu9250Spi::magWrite(uint8_t address, uint8_t *byte, uint32_t len)
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

  bool Mpu9250Spi::magWrite(uint8_t address, uint8_t byte)
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

  bool Mpu9250Spi::magRead(uint8_t address, uint8_t *byte, uint32_t len, uint32_t timeout_ms)
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
}