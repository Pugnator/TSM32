#include "tsm.h"
#include "settings.h"
#include "j1850.h"
#include "mpu.h"

#include <atomic>

#ifdef __cplusplus
extern "C"
{
#endif
  
  #define TIM9_PERIOD 110
  #define LONG_PRESS_COUNT (LONG_PRESS_TIME/TIM9_PERIOD)

  /** \brief Left button processing event triggered */
  static std::atomic_bool leftButtonEvent(false);
  /** \brief Right button processing event triggered */
  static std::atomic_bool rightButtonEvent(false);
  
  /** \brief We're waiting for a long press */
  static bool waitLongPress = false;

  /** \brief Number of timer events passed */
  static uint32_t timerHitCounter = 0;
  /** \brief How many timer events passed with a button pressed */
  static uint32_t longPressCounter = 0;
  uint32_t triggerTime = 0;

  static uint32_t leftButtonClickCount = 0;
  static uint32_t rightButtonClickCount = 0;
  static uint32_t toggleButtonClickCount = 0;

  void printStates()
  {
    DEBUG_LOG(
        "States:\r\nRight %s\r\nLeft %s\r\nOvertake %s\r\nWaiting for Long Press: %s\r\nLong Press counter %u\r\n",
        rightButtonEvent ? "ON" : "OFF", leftButtonEvent ? "ON" : "OFF", overtakeMode ? "ON" : "OFF", waitLongPress ? "ON" : "OFF", longPressCounter);
  }

  static void stopTim9()
  {
    HAL_TIM_Base_Stop_IT(&htim9);
    __HAL_TIM_SET_COUNTER(&htim9, 0);
    __HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
    timerHitCounter = 0;
  }

  static void startTim9()
  {
    stopTim9();
    __HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
    __HAL_TIM_SET_COUNTER(&htim9, 0);
    HAL_TIM_Base_Start_IT(&htim9);
  }

  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {

    if (GPIO_Pin == IMU_INT_Pin)
    {
      DEBUG_LOG("MPU Interrupt.\r\n");
      return;
    }

    triggerTime = HAL_GetTick();
    if (GPIO_Pin == LT_BUTTON_Pin && !leftButtonEvent)
    {
      startTim9();
      leftButtonEvent = true;
      DEBUG_LOG("Left switch activated.\r\n");
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && !rightButtonEvent)
    {
      startTim9();
      rightButtonEvent = true;
      DEBUG_LOG("Right switch activated.\r\n");
    }
  }

  void resetEvent()
  {
    stopTim9();
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
    TIM11 - MPU update
  */
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
#ifdef MEMS_ENABLED
    // Accelerometer update
    if (TIM11 == htim->Instance)
    {
      az = ahrs->getAzimuth();
      return;
    }
#endif

#ifdef J1850_ENABLED
    // J1850 service timer, 200us
    if (TIM6 == htim->Instance)
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
    if (TIM9 == htim->Instance)
    {
      timerHitCounter++;
      uint32_t eventTime = HAL_GetTick();
      uint32_t pressTime = eventTime - triggerTime;
      DEBUG_LOG("Button wait event at %u, L = %u, R = %u.\r\n", pressTime,
                LEFT_BUTTON,
                RIGHT_BUTTON);

      if (MAX_PRESS_WAIT_TIME <= pressTime)
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
          DEBUG_LOG("Long press detected after %ums.\r\n", pressTime);
          printStates();
          overtakeMode = false;
        }
        else
        {
          DEBUG_LOG("Short press.\r\n");
          printStates();
          overtakeMode = true;
        }

        resetEvent();
        return;
      }

      // too quick press - skip
      if (DEBOUNCE_MIN_TIME >= pressTime)
      {
        DEBUG_LOG("Bounce detected: T=[%ums].\r\n", pressTime, DEBOUNCE_MIN_TIME);
        return;
      }
      /* if both buttons are pressed */
      if (LEFT_BUTTON == GPIO_PIN_RESET &&
          RIGHT_BUTTON == GPIO_PIN_RESET)
      {
        DEBUG_LOG("Both switches were ON for %ums.\r\n", pressTime);
        printStates();
        leftButtonEvent = false;
        rightButtonEvent = false;
        hazardToggle();
        waitLongPress = true;
        startTim9();
        return;
      }
      /* if left button is still pressed */
      else if (!hazardEnabled &&
               LEFT_BUTTON == GPIO_PIN_RESET)
      {
        stopTim9();
        DEBUG_LOG("LT was pressed for %u.\r\n", pressTime);
        leftButtonEvent = false;
        leftSideToggle();
        // Check if it's a long press
        waitLongPress = true;
        startTim9();
        return;
      }
      /* if right button is still pressed */
      else if (!hazardEnabled &&
               RIGHT_BUTTON == GPIO_PIN_RESET)
      {
        stopTim9();
        DEBUG_LOG("RT was pressed for %u.\r\n", pressTime);
        rightButtonEvent = false;
        rightSideToggle();
        // Check if it's a long press
        waitLongPress = true;
        startTim9();
        return;
      }

      //no condition was met.
      resetEvent();
    }
#endif
  }

#ifdef __cplusplus
}
#endif