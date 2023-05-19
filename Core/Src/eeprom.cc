#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"

#include "eeprom.h"
#include "trace.h"

bool eeproInit()
{
  FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t sectorError;

  HAL_FLASH_Unlock(); // unlock flash memory for write access

  eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInitStruct.Sector = FLASH_SECTOR_FOR_EEPROM; // sector 11 is used for emulated EEPROM
  eraseInitStruct.NbSectors = 1;
  eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // voltage range 3: 2.7V to 3.6V
  if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return false;
  }

  HAL_FLASH_Lock(); // lock flash memory after write access

  return true;
}

void eepromWrite(uint16_t addr, uint8_t data)
{
  uint32_t flashAddress = FLASH_USER_START_ADDRESS + addr;

  HAL_FLASH_Unlock(); // unlock flash memory for write access

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress, data) != HAL_OK)
  {
    // error handling
  }

  HAL_FLASH_Lock(); // lock flash memory after write access
}

uint8_t eepromRead(uint16_t addr)
{
  uint32_t flashAddress = FLASH_USER_START_ADDRESS + addr;

  return (*(__IO uint8_t *)flashAddress);
}

bool isLastStartupSucceeded()
{
  auto result = eepromRead(0);
  DEBUG_LOG("Last Boot = 0x%X\r\n", result);
  return true;
}