#include "tsm.h"
#include "mpu.h"

bool MPU9250::mpuWrite(uint8_t address, uint8_t *byte, size_t len)
{
#if defined(IMU_SPI_MODE)
  mpuSelect();
  HAL_SPI_Transmit(&MPU_SPI_PORT, &address, 1, 100);
  uint8_t temp_val = 0;
  for (uint32_t i = 0; i < len; i++)
  {
    if (HAL_SPI_TransmitReceive(&MPU_SPI_PORT, &byte[i], &temp_val, 1, 100) != HAL_OK)
    {
      return false;
    }
  }
  mpuDeselect();
#elif defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
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
  if (HAL_SPI_Transmit(&MPU_SPI_PORT, &address, 1, 100) != HAL_OK)
  {
    mpuDeselect();
    return false;
  }

  uint8_t temp = 0;
  if (HAL_SPI_TransmitReceive(&MPU_SPI_PORT, &byte, &temp, 1, 100) != HAL_OK)
  {
    mpuDeselect();
    return false;
  }
  mpuDeselect();
#elif defined(IMU_I2C_MODE)
  uint8_t temp = byte;
  uint8_t *data = &temp;
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR, address, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
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
  if (HAL_SPI_Transmit(&MPU_SPI_PORT, &raddr, 1, 100) != HAL_OK)
  {
    mpuDeselect();
    return false;
  }

  uint8_t temp = 0;
  for (uint32_t i = 0; i < len; i++)
  {
    if (HAL_SPI_TransmitReceive(&MPU_SPI_PORT, &temp, &byte[i], 1, 100) != HAL_OK)
    {
      mpuDeselect();
      return false;
    }
  }
  mpuDeselect();
#elif defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR, address, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
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

bool MPU9250::magWriteRegIIC(uint8_t reg, uint8_t *byte, size_t len)
{
#if defined(IMU_I2C_MODE)
  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
#endif
  return true;
}

bool MPU9250::magWriteRegIIC(uint8_t reg, uint8_t byte)
{
#if defined(IMU_I2C_MODE)
  uint8_t temp = byte;
  uint8_t *data = &temp;

  if (HAL_I2C_Mem_Write(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, data, 1, 1000) != HAL_OK)
  {
    I2C_ClearBusyFlagErratum(i2c, 1000);
    DEBUG_LOG("Failed to write to MAG\r\n");
    return false;
  }
#endif
  return true;
}

bool MPU9250::magReadRegIIC(uint8_t reg, uint8_t *byte, size_t len, uint32_t timeout_ms)
{
#if defined(IMU_I2C_MODE)
  uint32_t start_time = HAL_GetTick(); // Record the start time

  while (HAL_GetTick() - start_time < timeout_ms)
  {
    if (HAL_I2C_Mem_Read(i2c, MPU9250_I2C_ADDR_MAG, reg, 1, byte, len, 100) == HAL_OK)
    {
      return true; // Read operation successful, return true
    }
  }

  I2C_ClearBusyFlagErratum(i2c, 1000);
  DEBUG_LOG("Failed to read MAG\r\n");
#endif
  return false; // Timeout reached, return false
}

bool MPU9250::magWrite(uint8_t reg, uint8_t *byte, size_t len)
{
#if defined(IMU_I2C_MODE)
  return magWriteRegIIC(reg, byte, len);
#elif defined(IMU_SPI_MODE)
  return true; // magWriteRegSpi(reg, byte, len);
#else
#error "Specify IMU mode"
#endif
}

bool MPU9250::magWrite(uint8_t reg, uint8_t byte)
{
#if defined(IMU_I2C_MODE)
  return magWriteRegIIC(reg, byte);
#elif defined(IMU_SPI_MODE)
  return true; // magWriteRegSpi(reg, byte);
#else
#error "Specify IMU mode"
#endif
}

bool MPU9250::magRead(uint8_t reg, uint8_t *byte, size_t len, uint32_t timeout_ms)
{
#if defined(IMU_I2C_MODE)
  return magReadRegIIC(reg, byte, len, timeout_ms);
#elif defined(IMU_SPI_MODE)
  return true; // magReadRegIIC(reg, byte, len, timeout_ms);
#else
#error "Specify IMU mode"
#endif
}