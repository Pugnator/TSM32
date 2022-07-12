#pragma once

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
//#include "dma.h"
#include "stdint.h"
#include <stdbool.h>
#include <SEGGER_RTT.h>

#include "trace.h"

/**
 * The STM32 factory-programmed UUID memory.
 * Three values of 32 bits each starting at this address
 * Use like this: STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]
 */
#define STM32_UUID ((uint32_t *)0x1FFF7A10)

#define USE_STATIC_ALLOC

#define MIN_PRESS_TIME 250

#define LEFT_PWM_OUT (TIM1->CCR3)
#define RIGHT_PWM_OUT (TIM1->CCR4)

#define LEFT_BUTTON (HAL_GPIO_ReadPin(LT_BUTTON_GPIO_Port, LT_BUTTON_Pin))
#define RIGHT_BUTTON (HAL_GPIO_ReadPin(RT_BUTTON_GPIO_Port, RT_BUTTON_Pin))
#define PRESSED (GPIO_PIN_RESET)
#define DEPRESSED (GPIO_PIN_SET)

#ifdef __cplusplus
extern "C"
{
#endif

    extern bool leftEnabled;
    extern bool rightEnabled;
    extern bool hazardEnabled;
    extern bool overtakeMode;

    void blinkerLeftSideOff();
    void blinkerLeftsideToggle();
    void blinkerRightSideOff();
    void blinkerRightsideToggle();
    void blinkerOff();
    void blinkerHazardToggle();
    void blinkerWorker();

    void enable_starter();
    void disable_starter();

    void kalmanInit(float mea_e, float est_e, float q);
    float updateEstimate(float mea);


    void DWT_Init();
    void DWT_Delay(uint32_t us);

#ifdef __cplusplus
}
#endif