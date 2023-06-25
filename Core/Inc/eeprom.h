#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <cstddef>

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


