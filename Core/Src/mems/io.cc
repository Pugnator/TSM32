#include "tsm.h"
#include "mpu.h"

bool MPU9250::mpuWrite(uint8_t address, uint8_t *byte, size_t len)
{
#if defined(IMU_SPI_MODE)
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
#elif defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(bus_, 1000);
    return false;
  }

  return true;
#else
#error "Select either SPI or I2C mode."
#endif
  return true;
}

bool MPU9250::mpuWrite(uint8_t address, uint8_t byte)
{
#if defined(IMU_SPI_MODE)
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
#elif defined(IMU_I2C_MODE)
  uint8_t temp = byte;
  uint8_t *data = &temp;
  if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR, address, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(bus_, 1000);
    return false;
  }
#else
#error "Select either SPI or I2C mode."
#endif

  return true;
}

bool MPU9250::mpuRead(uint8_t address, uint8_t *byte, size_t len)
{
#if defined(IMU_SPI_MODE)
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
#elif defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Read(bus_, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(bus_, 1000);
    return false;
  }
#else
#error "Select either SPI or I2C mode."
#endif

  return true;
}

void MPU9250::mpuDeselect()
{
#ifdef IMU_SPI_MODE
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
#endif
}

void MPU9250::mpuSelect()
{
#ifdef IMU_SPI_MODE
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);
#endif
}

bool MPU9250::magWriteRegI2c(uint8_t reg, uint8_t *byte, size_t len)
{
#if defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(bus_, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
#endif
  return true;
}

bool MPU9250::magWriteRegI2c(uint8_t reg, uint8_t byte)
{
#if defined(IMU_I2C_MODE)
  uint8_t temp = byte;
  uint8_t *data = &temp;

  if (HAL_I2C_Mem_Write(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(bus_, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
#endif
  return true;
}

bool MPU9250::magReadRegI2c(uint8_t reg, uint8_t *byte, size_t len, uint32_t timeout_ms)
{
#if defined(IMU_I2C_MODE)
  uint32_t start_time = HAL_GetTick(); // Record the start time

  while (HAL_GetTick() - start_time < timeout_ms)
  {
    if (HAL_I2C_Mem_Read(bus_, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 100) == HAL_OK)
    {
      return true; // Read operation successful, return true
    }
  }

  I2C_ClearBusyFlagErratum(bus_, 1000);
  DEBUG_LOG("Failed to read MAG\r\n");
#endif
  return false; // Timeout reached, return false
}

bool MPU9250::magWriteRegSpi(uint8_t address, uint8_t *byte, size_t len)
{
#if defined(IMU_SPI_MODE)
  static const uint8_t magAddress = MPU9250_SPI_ADDR_MAG;
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

#endif
  return true;
}
bool MPU9250::magWriteRegSpi(uint8_t address, uint8_t byte)
{
#if defined(IMU_SPI_MODE)
  static const uint8_t magAddress = MPU9250_SPI_ADDR_MAG;
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

#endif
  return true;
}
bool MPU9250::magReadRegSpi(uint8_t address, uint8_t *byte, size_t len, uint32_t timeout_ms)
{
#if defined(IMU_SPI_MODE)
  static const uint8_t magAddress = MPU9250_SPI_ADDR_MAG | 0x80;

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

#endif
  return true;
}

bool MPU9250::magWrite(uint8_t reg, uint8_t *byte, size_t len)
{
#if defined(IMU_I2C_MODE)
  return magWriteRegI2c(reg, byte, len);
#elif defined(IMU_SPI_MODE)
  return magWriteRegSpi(reg, byte, len);
#else
#error "Specify IMU mode"
#endif
}

bool MPU9250::magWrite(uint8_t reg, uint8_t byte)
{
#if defined(IMU_I2C_MODE)
  return magWriteRegI2c(reg, byte);
#elif defined(IMU_SPI_MODE)
  return magWriteRegSpi(reg, byte);
#else
#error "Specify IMU mode"
#endif
}

bool MPU9250::magRead(uint8_t reg, uint8_t *byte, size_t len, uint32_t timeout_ms)
{
#if defined(IMU_I2C_MODE)
  return magReadRegI2c(reg, byte, len, timeout_ms);
#elif defined(IMU_SPI_MODE)
  return magReadRegSpi(reg, byte, len, timeout_ms);
#else
#error "Specify IMU mode"
#endif
}