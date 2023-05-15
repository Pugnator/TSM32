#pragma once

#if defined(STM32F410Tx) || defined(STM32F410Cx) || defined(STM32F410Rx)
#define FLASH_SECTOR_FOR_EEPROM FLASH_SECTOR_4
#define FLASH_SECTOR_FOR_EEPROM_SIZE (128 * 1024) // 128 KB
#define FLASH_USER_START_ADDRESS ((uint32_t)(FLASH_BASE + (FLASH_SECTOR_FOR_EEPROM * FLASH_SECTOR_FOR_EEPROM_SIZE)))
#define FLASH_USER_END_ADDRESS ((uint32_t)(FLASH_USER_START_ADDRESS + FLASH_SECTOR_FOR_EEPROM_SIZE - 1))
#else
#error "Unsupported device"
#endif

bool eeproInit();
void eepromWrite(uint16_t addr, uint8_t data);
uint8_t eepromRead(uint16_t addr);
bool isLastStartupSucceeded();