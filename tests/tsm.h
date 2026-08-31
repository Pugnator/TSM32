#pragma once
// ---------------------------------------------------------------------------
// Shadow of Core/Inc/tsm.h for host-side unit tests.
// Provides all declarations/macros without pulling in any STM32 HAL headers.
// Found first by the compiler when -I tests precedes -I Core/Inc, because
// switch_ctrl.cc and turn_ctrl.cc live in Core/Src/ (no local tsm.h there).
// ---------------------------------------------------------------------------

#include "stubs.h"
#include "../Core/Inc/settings.h"

// ── Logging macros (normally provided by trace.h → printf.h) ────────────────
// Silence them by default; define TEST_VERBOSE before including test_env.h
// if you want to see production log output during tests.
#ifndef printf_
static inline int printf_(const char *, ...) { return 0; }
#endif
#ifndef PrintF
#  define PrintF(...)    ((void)0)
#endif
#ifndef DEBUG_LOG
#  define DEBUG_LOG(...) ((void)0)
#endif
#ifndef INFO_LOG
#  define INFO_LOG(...)  ((void)0)
#endif
#ifndef WARN_LOG
#  define WARN_LOG(...)  ((void)0)
#endif
#ifndef TRACE_LOG
#  define TRACE_LOG(...) ((void)0)
#endif

// ── Button / PWM macros (mirror of real tsm.h) ───────────────────────────────
#define LEFT_BUTTON  (HAL_GPIO_ReadPin(LT_BUTTON_GPIO_Port, LT_BUTTON_Pin))
#define RIGHT_BUTTON (HAL_GPIO_ReadPin(RT_BUTTON_GPIO_Port, RT_BUTTON_Pin))
#define PRESSED      (GPIO_PIN_RESET)
#define DEPRESSED    (GPIO_PIN_SET)

// LEFT_PWM_OUT / RIGHT_PWM_OUT expand to TIM1->CCR3/CCR4.
// Those map to _fakeTIM1.CCR3/CCR4 (= fakeLeftPWM/fakeRightPWM).
#define LEFT_PWM_OUT  (TIM1->CCR3)
#define RIGHT_PWM_OUT (TIM1->CCR4)

extern bool stopAppExecuting;
extern bool goOn;

#ifdef __cplusplus
extern "C"
{
#endif

    extern volatile uint8_t  currentSidemarkBrightness;
    extern volatile bool     leftEnabled;
    extern volatile bool     rightEnabled;
    extern volatile bool     hazardEnabled;
    extern volatile bool     overtakeMode;
    extern volatile bool     postTurnTailActive;
    extern volatile uint32_t blinkCounter;
    extern volatile bool     settingsMode;

    void leftSideOff();
    void leftSideToggle();
    void rightSideOff();
    void rightSideToggle();

    void blinkerDoBlink();
    void blinkerAutoCancelHandler();
    void discardButtonEvents();
    bool securityStarterPermitted();
    void startOvertakeMode();
    void startPostTurnTail();
    void hazardToggle();
    void blinkerHandler();

    void enableStarter();
    void disableStarter();
    void adcHandler();

    extern uint32_t       adcDMAbuffer[ADC_DMA_BUF_SIZE];
    extern volatile bool  adcDMAcompleted;

#ifdef __cplusplus
}
#endif
