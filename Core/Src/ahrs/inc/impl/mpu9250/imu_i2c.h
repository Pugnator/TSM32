#pragma once

#include "mpu9250_base.h"

#if !__has_include("i2c.h")
//! Dummy implementation if I2C is not enabled

typedef struct I2C_HandleTypeDef
{
} I2C_HandleTypeDef;

__attribute__((weak)) HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *, uint16_t, uint32_t, uint32_t)
{
  return HAL_OK;
}

__attribute__((weak)) HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t)
{
  return HAL_OK;
}

__attribute__((weak)) HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t, uint32_t)
{
  return HAL_OK;
}

#endif

namespace Mpu9250
{
  class Mpu9250I2c : public Mpu9250base
  {
  public:
    Mpu9250I2c(I2C_HandleTypeDef *interface, bool useDmp, MagMode magMode = Mpu9250::MagMode::MasterMode);
    ~Mpu9250I2c(){};

  private:
    bool setup() override;

    bool mpuWrite(uint8_t address, uint8_t *byte, uint32_t len) override;
    bool mpuWrite(uint8_t address, uint8_t byte) override;
    bool mpuRead(uint8_t address, uint8_t *byte, uint32_t len = 1) override;

    bool magWrite(uint8_t address, uint8_t *byte, uint32_t len) override;
    bool magWrite(uint8_t address, uint8_t byte) override;
    bool magRead(uint8_t address, uint8_t *byte, uint32_t len = 1, uint32_t timeout_ms = 1000) override;
    bool magWriteRegI2c(uint8_t address, uint8_t *byte, uint32_t len);
    bool magWriteRegI2c(uint8_t address, uint8_t byte);
    bool magReadRegI2c(uint8_t address, uint8_t *byte, uint32_t len = 1, uint32_t timeout_ms = 1000);
    bool magWriteRegI2cSlave(uint8_t address, uint8_t *byte, uint32_t len);
    bool magWriteRegI2cSlave(uint8_t address, uint8_t byte);
    bool magReadRegI2cSlave(uint8_t address, uint8_t *byte, uint32_t len = 1, uint32_t timeout_ms = 1000);

    I2C_HandleTypeDef *bus_;
  };
}