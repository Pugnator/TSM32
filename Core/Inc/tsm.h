#pragma once

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "stdint.h"
#include <stdbool.h>
#include "trace.h"
#include "settings.h"
#include "assert.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * The STM32 factory-programmed UUID memory.
 * Three values of 32 bits each starting at this address
 * Use like this: STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]
 */
#define STM32_UUID ((uint32_t *)0x1FFF7A10)

#define USE_STATIC_ALLOC

#define BLINKER_TIMER htim9
#define BLINKER_TIMER_INSTANCE TIM9
#define J1850_EOF_TIMER htim6
#define J1850_TIMER_INSTANCE TIM6

#define LEFT_BUTTON (HAL_GPIO_ReadPin(LT_BUTTON_GPIO_Port, LT_BUTTON_Pin))
#define RIGHT_BUTTON (HAL_GPIO_ReadPin(RT_BUTTON_GPIO_Port, RT_BUTTON_Pin))
#define PRESSED (GPIO_PIN_RESET)
#define DEPRESSED (GPIO_PIN_SET)

#define LEFT_PWM_OUT (TIM1->CCR3)
#define RIGHT_PWM_OUT (TIM1->CCR4)

    extern volatile uint8_t currentSidemarkBrightness;
    extern bool stopAppExecuting;

    void adcHandler();

    extern bool leftEnabled;
    extern bool rightEnabled;
    extern bool hazardEnabled;
    extern bool overtakeMode;
    extern bool blinkPause;
    extern uint32_t blinkCounter;

    void leftSideOff();
    void leftSideToggle();
    void rightSideOff();
    void rightSideToggle();

    void blinkerOff();
    void blinkerOn();
    void hazardToggle();

    void enableStarter();
    void disableStarter();

    void adcHandler();
    void memsHandler();
    void j1850Handler();
    void blinkerHandler();
    void workerLoop();

    extern uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];
    extern bool adcDMAcompleted;

#ifdef __cplusplus
}
#endif