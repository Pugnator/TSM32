#pragma once

#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "dma.h"
#include "stdint.h"
#include <stdbool.h>
#include <SEGGER_RTT.h>

#define LOGGING_CHANNEL 0

#ifdef NDEBUG
#define DEBUG_LOG PrintF
#else
#define DEBUG_LOG(...)
#endif

#define Print(x) SEGGER_RTT_WriteString(LOGGING_CHANNEL, x)
#define PrintF(...) SEGGER_RTT_printf(LOGGING_CHANNEL, ##__VA_ARGS__)

#define LEFT_PWM_OUT (TIM1->CCR3)
#define RIGHT_PWM_OUT (TIM1->CCR4)

#define LEFT_BUTTON (HAL_GPIO_ReadPin(LT_BUTTON_GPIO_Port, LT_BUTTON_Pin))
#define RIGHT_BUTTON (HAL_GPIO_ReadPin(RT_BUTTON_GPIO_Port, RT_BUTTON_Pin))
#define PRESSED (GPIO_PIN_RESET)
#define DEPRESSED (GPIO_PIN_SET)

extern bool left_blinker_enabled;
extern bool right_blinker_enabled;
extern bool hazard_blinker_enabled;
extern bool overtakeMode;

void blinker_leftside_off();
void blinkerLeftsideToggle();
void blinker_rightside_off();
void blinkerRightsideToggle();
void blinkerHazardToggle();
void blinker_worker();

void kalmanInit(float mea_e, float est_e, float q);