#pragma once

#define   _EE_USE_FLASH_PAGE_OR_SECTOR              (127)
#define   _EE_USE_RAM_BYTE                          (1024)
#define   _EE_VOLTAGE                               FLASH_VOLTAGE_RANGE_3 //  use in some devices

bool      ee_init(void);
bool      ee_format(bool keepRamData);
bool      ee_read(uint32_t startVirtualAddress, uint32_t len, uint8_t* data);
bool      ee_write(uint32_t startVirtualAddress, uint32_t len, uint8_t* data);
bool      ee_writeToRam(uint32_t startVirtualAddress, uint32_t len, uint8_t* data); //  only use when _EE_USE_RAM_BYTE is enabled
bool      ee_commit(void);  //  only use when _EE_USE_RAM_BYTE is enabled
uint32_t  ee_maxVirtualAddress(void);


void eeprom_test();