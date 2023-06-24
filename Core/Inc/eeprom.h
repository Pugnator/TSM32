#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <cstddef>
#include "i2c.h"

#define _EEPROM_ADDRESS 0xA0

enum class eSize : uint32_t
{
  KBIT_1 = 1,
  KBIT_2 = 2,
  KBIT_4 = 4,
  KBIT_8 = 8,
  KBIT_16 = 16
};

struct Eeprom
{
  ~Eeprom() = default;
  virtual bool read(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout = 1000) = 0;
  virtual bool write(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout = 1000) = 0;
  virtual bool erase() = 0;
  virtual bool init() = 0;

private:
  bool lock_;
};

struct EepromFlash : Eeprom
{
  EepromFlash(){};

  bool read(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout = 1000) override;
  bool write(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout = 1000) override;
  bool erase() override;
  bool init() override;

private:
  bool lock_;
};

struct Eeprom24C : Eeprom
{
  Eeprom24C(I2C_HandleTypeDef *io, eSize size)
      : i2c(io),
        romSize_(size)
  {
    lock_ = false;
    switch (romSize_)
    {
    case eSize::KBIT_1:
    case eSize::KBIT_2:
      pageSize_ = 8;
      break;
    case eSize::KBIT_4:
    case eSize::KBIT_8:
    case eSize::KBIT_16:
      pageSize_ = 16;
    }
  }
  bool init() override;
  bool write(uint32_t address, uint8_t *buffer, uint32_t bufferSize, uint32_t timeout = 1000) override;
  bool read(uint32_t address, uint8_t *buffer, uint32_t bufferSize, uint32_t timeout = 1000) override;
  bool erase() override;

private:
  bool lock_;
  I2C_HandleTypeDef *i2c;
  eSize romSize_;
  size_t pageSize_;
};
