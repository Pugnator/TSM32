#pragma once
// ---------------------------------------------------------------------------
// Hardware stubs for host-side unit tests.
// Included by tests/tsm.h (the shadow of Core/Inc/tsm.h).
// All globals use C++17 'inline' so this file is safe to include from
// multiple translation units without ODR violations.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

// ── Core HAL types ───────────────────────────────────────────────────────────
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct { uint32_t _pad; } GPIO_TypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

// TIM peripheral register layout — only fields we use are needed.
typedef struct {
    uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
    uint32_t CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;

typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } ADC_HandleTypeDef;

// ── TIM peripheral instances ─────────────────────────────────────────────────
// TIM1 holds the PWM compare registers: CCR3=left turn, CCR4=right turn.
// (tsm.h defines LEFT_PWM_OUT as TIM1->CCR3 / RIGHT_PWM_OUT as TIM1->CCR4)
inline TIM_TypeDef _fakeTIM1 = {};

#define TIM1 (&_fakeTIM1)
#define TIM2 ((TIM_TypeDef*)0x02u)
#define TIM3 ((TIM_TypeDef*)0x03u)
#define TIM4 ((TIM_TypeDef*)0x04u)

// Timer handles — inline so the same object is used in all TUs.
inline TIM_HandleTypeDef htim1 = { TIM1 };
inline TIM_HandleTypeDef htim2 = { TIM2 };
inline TIM_HandleTypeDef htim3 = { TIM3 };
inline TIM_HandleTypeDef htim4 = { TIM4 };

// ── ADC simulation ──────────────────────────────────────────────────────────
#define ADC1 ((void*)0x01u)
inline ADC_HandleTypeDef hadc1 = { ADC1 };
inline HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef*) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t*, uint32_t) { return HAL_OK; }

// ── GPIO simulation ──────────────────────────────────────────────────────────
inline GPIO_PinState fakeLeftPin  = GPIO_PIN_SET;   // GPIO_PIN_SET = DEPRESSED
inline GPIO_PinState fakeRightPin = GPIO_PIN_SET;

#define GPIO_PIN_12  12u
#define GPIO_PIN_13  13u
#define GPIOA  ((GPIO_TypeDef*)0x40010800u)
#define GPIOB  ((GPIO_TypeDef*)0x40010C00u)
#define GPIOC  ((GPIO_TypeDef*)0x40011000u)

// Button pin macros (used by tests/tsm.h's LEFT_BUTTON / RIGHT_BUTTON macros)
#define LT_BUTTON_Pin       GPIO_PIN_13
#define LT_BUTTON_GPIO_Port GPIOB
#define RT_BUTTON_Pin       GPIO_PIN_12
#define RT_BUTTON_GPIO_Port GPIOB

inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint32_t pin)
{
    if (pin == GPIO_PIN_13) return fakeLeftPin;
    if (pin == GPIO_PIN_12) return fakeRightPin;
    return GPIO_PIN_SET;
}

inline void HAL_GPIO_WritePin(GPIO_TypeDef*, uint32_t, GPIO_PinState) {}

// ── Timer simulation ─────────────────────────────────────────────────────────
inline uint32_t fakeTick = 0;
inline uint32_t HAL_GetTick() { return fakeTick; }

inline HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef*) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
#define __HAL_TIM_SET_COUNTER(h, v)   ((void)0)
#define __HAL_TIM_CLEAR_FLAG(h, f)    ((void)0)
#define __HAL_TIM_GET_FLAG(h, f)      0u
#define TIM_SR_UIF     0u
#define TIM_FLAG_UPDATE 0u
#define TIM_CHANNEL_1  0u
#define TIM_CHANNEL_2  1u

// ── PWM output aliases ────────────────────────────────────────────────────────
// tsm.h defines LEFT_PWM_OUT as (TIM1->CCR3) and RIGHT_PWM_OUT as (TIM1->CCR4).
// Tests reference fakeLeftPWM / fakeRightPWM which map to the same storage.
#define fakeLeftPWM   (_fakeTIM1.CCR3)
#define fakeRightPWM  (_fakeTIM1.CCR4)

// ── Misc stubs ────────────────────────────────────────────────────────────────
#define assert(x)  ((void)(x))
inline void Error_Handler(void) {}
