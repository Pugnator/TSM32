
#include "i2c.h"
#include "eeprom.h"
#include "trace.h"

bool Eeprom24C::init()
{
  if (HAL_I2C_IsDeviceReady(i2c, _EEPROM_ADDRESS, 2, 100) == HAL_OK)
    return true;
  else
    return false;
}

bool Eeprom24C::write(uint32_t address, uint8_t *buffer, uint32_t len, uint32_t timeout)
{
  if (lock_)
  {
    DEBUG_LOG("Eeprom is locked");
    return false;
  }

  lock_ = true;
  const uint32_t startTime = HAL_GetTick();
  uint32_t block = pageSize_ - (address % pageSize_);
  if (block > len)
  {
    block = len;
  }

  while (1)
  {

    if ((romSize_ == eSize::KBIT_1 || romSize_ == eSize::KBIT_2) && HAL_I2C_Mem_Write(i2c, _EEPROM_ADDRESS, address, 1, buffer, block, timeout) != HAL_OK)
    {
      return lock_ = false;
    }
    else if (romSize_ == eSize::KBIT_4 && HAL_I2C_Mem_Write(i2c, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), 1, buffer, block, timeout) != HAL_OK)
    {
      return lock_ = false;
    }
    else if (romSize_ == eSize::KBIT_8 && HAL_I2C_Mem_Write(i2c, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), 1, buffer, block, timeout) != HAL_OK)
    {
      return lock_ = false;
    }
    else if (romSize_ == eSize::KBIT_16 && HAL_I2C_Mem_Write(i2c, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), 1, buffer, block, timeout) != HAL_OK)
    {
      return lock_ = false;
    }

    HAL_Delay(10);
    len -= block;
    buffer += block;
    address += block;
    if (len == 0)
    {
      return !(lock_ = false);
    }

    if (HAL_GetTick() - startTime > timeout)
    {
      return lock_ = false;
    }
  }
  return !(lock_ = false);
}

bool Eeprom24C::read(uint32_t address, uint8_t *buffer, uint32_t len, uint32_t timeout)
{
  if (lock_)
  {
    DEBUG_LOG("Eeprom is locked");
    return false;
  }

  lock_ = true;

  if ((romSize_ == eSize::KBIT_1 || romSize_ == eSize::KBIT_2) && HAL_I2C_Mem_Read(i2c, _EEPROM_ADDRESS, address, 1, buffer, len, timeout) != HAL_OK)
  {
    return lock_ = false;
  }
  else if (romSize_ == eSize::KBIT_4 && HAL_I2C_Mem_Read(i2c, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), 1, buffer, len, timeout) != HAL_OK)
  {
    return lock_ = false;
  }
  else if (romSize_ == eSize::KBIT_8 && HAL_I2C_Mem_Read(i2c, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), 1, buffer, len, timeout) != HAL_OK)
  {
    return lock_ = false;
  }
  else if (romSize_ == eSize::KBIT_16 && HAL_I2C_Mem_Read(i2c, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), 1, buffer, len, timeout) != HAL_OK)
  {
    return lock_ = false;
  }

  return !(lock_ = false);
}

bool Eeprom24C::erase(void)
{
  const uint8_t erasebuffer[32] = {0xFF};
  uint32_t bytes = 0;
  while (bytes < (static_cast<uint32_t>(romSize_) * 256))
  {
    if (write(bytes, (uint8_t *)erasebuffer, sizeof(erasebuffer), 100) == false)
      return false;
    bytes += sizeof(erasebuffer);
  }
  return true;
}

bool EepromFlash::read(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout)
{
  return true;
}
bool EepromFlash::write(uint32_t address, uint8_t *buffer, uint32_t size, uint32_t timeout)
{
  return true;
}
bool EepromFlash::erase()
{  
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef flashErase;
  
  HAL_FLASH_Lock();
  return true;
}
bool EepromFlash::init()
{
  return true;
}