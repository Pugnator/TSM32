#include "tsm.h"
#include "settings.h"
#include "j1850.h"
#include "mpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TIM9_PERIOD 110
#define LONG_PRESS_COUNT (LONG_PRESS_TIME / TIM9_PERIOD)

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
    if (GPIO_Pin == IMU_INT_Pin)
    {
      DEBUG_LOG("MPU Interrupt.\r\n");
      return;
    }    

    if (GPIO_Pin == LT_BUTTON_Pin && !leftButtonEvent)
    {
      startBlinkerTimer();
      leftButtonEvent = true;
      DEBUG_LOG("[%u] Left switch activated.\r\n", startTime);
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && !rightButtonEvent)
    {
      startBlinkerTimer();
      rightButtonEvent = true;
      DEBUG_LOG("[%u] Right switch activated.\r\n", startTime);
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
    TIM5  - J1850 capture
    TIM6  - J1850 - EOF timer
    TIM9  - Blinker delay timer    
  */
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
#if J1850_ENABLED
    // J1850 service timer, 200us
    if (J1850_TIMER_INSTANCE == htim->Instance)
    {
      messageCollected = true;
      HAL_GPIO_TogglePin(J1850TX_GPIO_Port, J1850TX_Pin);
      if (rxQueryNotEmpty)
      {
        j1850SendMessage();
      }
      HAL_TIM_Base_Stop_IT(&htim6);
      return;
    }
#endif

#if BLINKER_ENABLED
    if (BLINKER_TIMER_INSTANCE == htim->Instance)
    {
      timerHitCounter++;
      uint32_t currentTime = HAL_GetTick();
      if (startTime > currentTime)
      {
        DEBUG_LOG("Shouldn't happen. The event is in the past. %u (trigger) > %u (now)\r\n", startTime, currentTime);
        resetEvent();
        return;
      }
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
        // We're waiting for a long press, but the button is depressed - stop
        if (timerHitCounter > 2 && (LEFT_BUTTON == GPIO_PIN_SET || LEFT_BUTTON == GPIO_PIN_SET))
        {
          DEBUG_LOG("No button is pressed while waiting for a long press. Stop.\r\n");
          overtakeMode = true;
          resetEvent();
          return;
        }

        if (longPressCounter++ != LONG_PRESS_COUNT)
        {
          DEBUG_LOG("Waiting for a long press [%u].\r\n", longPressCounter);
          return;
        }

        DEBUG_LOG("Check for a long press.\r\n");

        if (LEFT_BUTTON == GPIO_PIN_RESET ||
            RIGHT_BUTTON == GPIO_PIN_RESET)
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

      // too quick press - skip
      if (DEBOUNCE_MIN_TIME >= pressDuration)
      {
        DEBUG_LOG("Bounce detected: T=[%ums].\r\n", pressDuration, DEBOUNCE_MIN_TIME);
        return;
      }
      /* if both buttons are pressed */
      if (LEFT_BUTTON == GPIO_PIN_RESET &&
          RIGHT_BUTTON == GPIO_PIN_RESET)
      {
        DEBUG_LOG("Both switches were ON for %ums.\r\n", pressDuration);
        leftButtonEvent = false;
        rightButtonEvent = false;
        hazardToggle();
        waitLongPress = true;
        startBlinkerTimer();
        return;
      }
      /* if left button is still pressed */
      else if (!hazardEnabled &&
               LEFT_BUTTON == GPIO_PIN_RESET)
      {
        stopBlinkerTimer();
        DEBUG_LOG("LT was pressed for %u.\r\n", pressDuration);
        leftButtonEvent = false;
        leftSideToggle();
        // Check if it's a long press
        waitLongPress = true;
        startBlinkerTimer();
        return;
      }
      /* if right button is still pressed */
      else if (!hazardEnabled &&
               RIGHT_BUTTON == GPIO_PIN_RESET)
      {
        stopBlinkerTimer();
        DEBUG_LOG("RT was pressed for %u.\r\n", pressDuration);
        rightButtonEvent = false;
        rightSideToggle();
        // Check if it's a long press
        waitLongPress = true;
        startBlinkerTimer();
        return;
      }

      // no condition was met.
      resetEvent();
    }
#endif
  }

#ifdef __cplusplus
}
#endif