#include "tsm.h"
#include "settings.h"
#include "j1850.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Period of BLINKER_TIMER (TIM4) in milliseconds, configured in CubeMX (tim.c).
 * Keep this value in sync with the timer's prescaler/period. */
#define BLINKER_TIMER_PERIOD_MS 110
#define LONG_PRESS_COUNT (LONG_PRESS_TIME / BLINKER_TIMER_PERIOD_MS)

  /** \brief Left button processing event triggered */
  static volatile bool leftButtonEvent = false;
  /** \brief Right button processing event triggered */
  static volatile bool rightButtonEvent = false;

  /** \brief We're waiting for a long press */
  static volatile bool waitLongPress = false;

  /** \brief Number of timer events passed */
  static volatile uint32_t timerHitCounter = 0;
  /** \brief How many timer events passed with a button pressed */
  static volatile uint32_t longPressCounter = 0;
  static volatile uint32_t startTime = 0;

  /* Raw edge flags set by EXTI ISR; consumed in the main loop by
   * processButtonEvents() inside blinkerHandler() (Fixes #42). */
  static volatile bool leftButtonRawEvent = false;
  static volatile bool rightButtonRawEvent = false;
  /* Set by TIM4 ISR each tick; consumed by blinkerTimerFSM() in the main
   * loop inside blinkerHandler() (Fixes #40). */
  static volatile bool blinkerTick = false;

  static void stopBlinkerTimer()
  {
    HAL_TIM_Base_Stop_IT(&BLINKER_TIMER);
    __HAL_TIM_SET_COUNTER(&BLINKER_TIMER, 0);
    __HAL_TIM_CLEAR_FLAG(&BLINKER_TIMER, TIM_SR_UIF);
    timerHitCounter = 0;
  }

  static void startBlinkerTimer()
  {
    stopBlinkerTimer();
    __HAL_TIM_CLEAR_FLAG(&BLINKER_TIMER, TIM_SR_UIF);
    __HAL_TIM_SET_COUNTER(&BLINKER_TIMER, 0);
    startTime = HAL_GetTick();
    HAL_TIM_Base_Start_IT(&BLINKER_TIMER);
  }

  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
#ifdef IMU_INT_Pin
    if (GPIO_Pin == IMU_INT_Pin)
    {
      return;
    }
#endif
    /* ISR sets raw flags only; debounce / timer start logic runs in
     * processButtonEvents() in the main loop (Fixes #42). */
    if (GPIO_Pin == LT_BUTTON_Pin && LEFT_BUTTON == PRESSED)
    {
      leftButtonRawEvent = true;
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && RIGHT_BUTTON == PRESSED)
    {
      rightButtonRawEvent = true;
    }
  }

  void resetEvent()
  {
    stopBlinkerTimer();
    timerHitCounter = 0;
    waitLongPress = false;
    leftButtonEvent = false;
    rightButtonEvent = false;
    longPressCounter = 0;
  }

  /*
    TIM1  - PWM, bulbs
    TIM2  - J1850 input capture (PA1 / TIM2_CH2)
    TIM3  - J1850 EOF idle-detect timer (~248 us one-shot)
    TIM4  - Blinker delay timer
  */
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
#if J1850_ENABLED
    // J1850 service timer, 200us
    if (J1850_EOF_TIMER_INSTANCE == htim->Instance)
    {
      messageCollected = true;
      HAL_TIM_Base_Stop_IT(&J1850_EOF_TIMER);
      return;
    }
#endif

#if BLINKER_ENABLED
    if (BLINKER_TIMER_INSTANCE == htim->Instance)
    {
      /* Just set the tick flag; full FSM runs in blinkerTimerFSM()
       * via blinkerHandler() in the main loop (Fixes #40). */
      blinkerTick = true;
      return;
    }
#endif
  }

  /* Processes raw button flags set by the EXTI ISR.  Starts the blinker
   * debounce timer on first press.  Must be called from the main loop. */
  static void processButtonEvents()
  {
    const bool wasIdle = !leftButtonEvent && !rightButtonEvent;

    if (leftButtonRawEvent)
    {
      leftButtonRawEvent = false;
      if (!leftButtonEvent)
      {
        if (wasIdle)
        {
          startBlinkerTimer();
        }
        leftButtonEvent = true;
        DEBUG_LOG("[%u] Left switch activated.\r\n", startTime);
      }
    }

    if (rightButtonRawEvent)
    {
      rightButtonRawEvent = false;
      if (!rightButtonEvent)
      {
        if (wasIdle)
        {
          startBlinkerTimer();
        }
        rightButtonEvent = true;
        DEBUG_LOG("[%u] Right switch activated.\r\n", startTime);
      }
    }
  }

  /* Button state-machine tick.  Formerly the body of the TIM4 ISR branch.
   * Called from blinkerHandler() after observing blinkerTick (Fixes #40). */
  static void blinkerTimerFSM()
  {
    timerHitCounter = timerHitCounter + 1;
    uint32_t currentTime = HAL_GetTick();
    /* Unsigned subtraction handles HAL_GetTick() 32-bit wraparound
     * (~49.7 days) correctly without an explicit guard. */
    uint32_t pressDuration = currentTime - startTime;
    DEBUG_LOG("[%u] %u since the click [%u], L = %u, R = %u.\r\n", currentTime, pressDuration, startTime,
              LEFT_BUTTON,
              RIGHT_BUTTON);

    if (MAX_PRESS_WAIT_TIME <= pressDuration)
    {
      DEBUG_LOG("Button wait timeout, resetting the event.\r\n");
      resetEvent();
      return;
    }

    if (waitLongPress)
    {
      /* The side toggle has already been applied. We now wait the full
       * LONG_PRESS_COUNT timer ticks before deciding:
       *   - button still pressed at the deadline -> regular turn signal
       *   - button released before the deadline  -> overtake (lane-change) */
      if (longPressCounter < LONG_PRESS_COUNT)
      {
        DEBUG_LOG("Waiting for a long press [%u].\r\n", longPressCounter);
        longPressCounter = longPressCounter + 1;
        return;
      }

      DEBUG_LOG("Check for a long press.\r\n");

      if (LEFT_BUTTON == PRESSED ||
          RIGHT_BUTTON == PRESSED)
      {
        DEBUG_LOG("Long press detected after %ums.\r\n", pressDuration);
        overtakeMode = false;
      }
      else
      {
        DEBUG_LOG("Short press.\r\n");
        overtakeMode = true;
      }

      resetEvent();
      return;
    }

    /* if both buttons are pressed */
    if (LEFT_BUTTON == PRESSED &&
        RIGHT_BUTTON == PRESSED)
    {
      DEBUG_LOG("Both switches were ON for %ums.\r\n", pressDuration);
      hazardToggle();
      resetEvent();
      return;
    }
    /* if left button is still pressed */
    else if (!hazardEnabled &&
             LEFT_BUTTON == PRESSED)
    {
      stopBlinkerTimer();
      DEBUG_LOG("LT was pressed for %u.\r\n", pressDuration);
      leftButtonEvent = false;
      leftSideToggle();
      waitLongPress = true;
      startBlinkerTimer();
      return;
    }
    /* if right button is still pressed */
    else if (!hazardEnabled &&
             RIGHT_BUTTON == PRESSED)
    {
      stopBlinkerTimer();
      DEBUG_LOG("RT was pressed for %u.\r\n", pressDuration);
      rightButtonEvent = false;
      rightSideToggle();
      waitLongPress = true;
      startBlinkerTimer();
      return;
    }
    /* left button was pressed and released before the timer fired */
    else if (!hazardEnabled && leftButtonEvent)
    {
      DEBUG_LOG("LT short press (released before timer) for %ums.\r\n", pressDuration);
      leftButtonEvent = false;
      leftSideToggle();
      resetEvent();
      return;
    }
    /* right button was pressed and released before the timer fired */
    else if (!hazardEnabled && rightButtonEvent)
    {
      DEBUG_LOG("RT short press (released before timer) for %ums.\r\n", pressDuration);
      rightButtonEvent = false;
      rightSideToggle();
      resetEvent();
      return;
    }

    // no condition was met.
    resetEvent();
  }

  /* Main-loop entry point for button + blinker-tick processing.
   * Replaces the heavy work that previously ran inside ISR callbacks. */
  void blinkerHandler()
  {
    processButtonEvents();
    if (blinkerTick)
    {
      blinkerTick = false;
      blinkerTimerFSM();
    }
  }

#ifdef __cplusplus
}
#endif