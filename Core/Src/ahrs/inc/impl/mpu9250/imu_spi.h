#pragma once

#include "mpu9250_base.h"

#if !__has_include("spi.h")
//! Dummy implementation if SPI is not enabled

typedef struct SPI_HandleTypeDef
{
} SPI_HandleTypeDef;

__attribute__((weak)) HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t)
{
  return HAL_OK;
}

__attribute__((weak)) HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t)
{
  return HAL_OK;
}
#endif

namespace Mpu9250
{
  class Mpu9250Spi : public Mpu9250base
  {
  public:
    Mpu9250Spi(SPI_HandleTypeDef *interface, bool dmpMode, MagMode magMode);
    ~Mpu9250Spi(){};

  private:
    bool setup() override;

    void mpuSelect();
    void mpuDeselect();

    bool mpuWrite(uint8_t address, uint8_t *byte, uint32_t len) override;
    bool mpuWrite(uint8_t address, uint8_t byte) override;
    bool mpuRead(uint8_t address, uint8_t *byte, uint32_t len = 1) override;

    bool magWrite(uint8_t address, uint8_t *byte, uint32_t len) override;
    bool magWrite(uint8_t address, uint8_t byte) override;
    bool magRead(uint8_t address, uint8_t *byte, uint32_t len = 1, uint32_t timeout_ms = 1000) override;
    
    SPI_HandleTypeDef *bus_;
  };
}