#pragma once
#include <stdint.h>

typedef enum
{
    STM32F0_t,
    STM32F1_t,
    STM32F2_t,
    STM32F3_t,
    STM32F4_t,
    STM32F7_t,
    STM32L0_t,
    STM32L1_t,
    STM32L4_t,
    STM32H7_t,
} MCUTypedef;

void getCPUid(uint32_t *id, MCUTypedef type);