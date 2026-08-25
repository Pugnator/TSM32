#pragma once
// ---------------------------------------------------------------------------
// Shim that makes Core/Inc/tsm.h and the HAL-dependent headers compile on
// the host toolchain.  Include this as the very first header in test files.
// ---------------------------------------------------------------------------

// 1. HAL / peripheral stubs (stm32f1xx_hal.h will pull this in from the
//    production sources via #include "stm32f1xx_hal.h", but we also include
//    it explicitly here so all types are visible to the test driver itself).
#include "stubs.h"

// 2. Real settings (plain #defines, no HAL deps after stubs are in place).
#include "../Core/Inc/settings.h"

// 3. Forward-declare all symbols from the production units under test.
extern "C"
{
    // --- turn_ctrl.cc ---
    extern volatile uint8_t  currentSidemarkBrightness;
    extern volatile bool     leftEnabled;
    extern volatile bool     rightEnabled;
    extern volatile bool     hazardEnabled;
    extern volatile bool     overtakeMode;
    extern volatile uint32_t blinkCounter;

    void leftSideOff();
    void leftSideToggle();
    void rightSideOff();
    void rightSideToggle();
    void blinkerDoBlink();
    void hazardToggle();

    // --- switch_ctrl.cc ---
    void blinkerHandler();

    // HAL callbacks routed to production code by the test driver
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
}

// 4. Definitions for symbols that are extern-declared in tsm.h / j1850.h
//    but live in other translation units not compiled during tests.
extern "C"
{
    volatile bool settingsMode      = false;
    volatile bool stopAppExecuting  = false;
    volatile bool goOn              = true;

    // Referenced by switch_ctrl.cc (from j1850.h)
    volatile bool messageCollected  = false;

    // Referenced by j1850.h externs (declarations only; never called from
    // switch_ctrl.cc / turn_ctrl.cc, so definitions are never needed).
    uint16_t rpms = 0;
    uint16_t kph  = 0;
    volatile uint32_t rpmLastUpdateTick = 0;
    volatile uint32_t speedLastUpdateTick = 0;
    volatile bool rpmSignalSeen = false;
    volatile bool speedSignalSeen = false;
}

// EOF processing belongs to the J1850 receiver, which is intentionally not
// linked into the blinker/engine host tests.
namespace J1850VPW
{
    void onEofTimeout() {}
}
