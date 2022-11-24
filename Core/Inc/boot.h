#pragma once

#define APP_ADDRESS (uint32_t)0x08008000
#define END_ADDRESS (uint32_t)0x080FFFFB
#define CRC_ADDRESS (uint32_t)0x080FFFFC
#define SYSMEM_ADDRESS (uint32_t)0x1FFF0000

/* Defines -------------------------------------------------------------------*/
#define APP_SIZE (uint32_t)(((END_ADDRESS - APP_ADDRESS) + 3) / 4)

#define FLASH_PAGE_NBPERBANK (256)

/* MCU RAM information (to check whether flash contains valid application) */
#define RAM_BASE SRAM1_BASE
#define RAM_SIZE SRAM1_SIZE_MAX