#include "tsm.h"
#include "id.h"

/*Define the type of STM32 MCU*/

uint32_t idAddr[] =
    {
        0x1FFFF7AC, /*STM32F0 unique ID start address*/
        0x1FFFF7E8, /*STM32F1 unique ID starting address*/
        0x1FFF7A10, /*STM32F2 unique ID starting address*/
        0x1FFFF7AC, /*STM32F3 unique ID starting address*/
        0x1FFF7A10, /*STM32F4 unique ID starting address*/
        0x1FF0F420, /*STM32F7 unique ID starting address*/
        0x1FF80050, /*STM32L0 unique ID starting address*/
        0x1FF80050, /*STM32L1 unique ID starting address*/
        0x1FFF7590, /*STM32L4 unique ID starting address*/
        0x1FF0F420  /*STM32H7 unique ID starting address*/
};

/*Get the unique ID of the MCU*/
void getCPUid(uint32_t *id, MCUTypedef type)
{
  if (!id)
  {
    return;
  }
  id[0] = *(uint32_t *)(idAddr[type]);
  id[1] = *(uint32_t *)(idAddr[type] + 4);
  id[2] = *(uint32_t *)(idAddr[type] + 8);
}