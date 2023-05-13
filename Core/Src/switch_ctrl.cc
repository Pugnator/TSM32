#include "tsm.h"
#include "settings.h"
#include "j1850.h"
#include "mpu.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static bool leftButtonEvent = false;
  static bool rightButtonEvent = false;

  static bool waitLongPress = false;
  static uint32_t longPressCounter = 0;
  uint32_t triggerTime = 0;

  void printButtStates()
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
      DEBUG_LOG("MPU Interrupt!\r\n");
      return;
    }

    triggerTime = HAL_GetTick();
    DEBUG_LOG("Button pressed at %u [L=%u R=%u]\r\n", triggerTime, leftButtonEvent, rightButtonEvent);
    if (GPIO_Pin == LT_BUTTON_Pin && !leftButtonEvent)
    {
      startTim9();
      leftButtonEvent = true;
      DEBUG_LOG("Left switch activated\r\n");
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && !rightButtonEvent)
    {
      startTim9();
      rightButtonEvent = true;
      DEBUG_LOG("Right switch activated\r\n");
    }
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
    // Accelerometer update
#ifdef MEMS_ENABLED
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
    if (TIM9 != htim->Instance)
    {
      return;
    }

    uint32_t eventTime = HAL_GetTick();
    uint32_t pressTime = eventTime - triggerTime;

    if (waitLongPress)
    {
      if (longPressCounter++ != 3)
      {
        return;
      }

      if (LEFT_BUTTON == PRESSED || RIGHT_BUTTON == PRESSED)
      {
        DEBUG_LOG("Long press for %ums\r\n", pressTime);
        printButtStates();
        overtakeMode = false;
      }
      else
      {
        DEBUG_LOG("Short press\r\n");
        printButtStates();
        overtakeMode = true;
      }
      stopTim9();

      waitLongPress = false;
      leftButtonEvent = false;
      rightButtonEvent = false;
      longPressCounter = 0;
      return;
    }

    // too quick press - skip
    if (MIN_PRESS_TIME > pressTime)
    {
      DEBUG_LOG("Press time [%ums] is less than required [%ums], ignoring event\r\n", pressTime, MIN_PRESS_TIME);
      rightButtonEvent = false;
      leftButtonEvent = false;
      return;
    }
    /* if both buttons are pressed */
    if (LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED)
    {
      DEBUG_LOG("Both switches were ON for %ums\r\n", pressTime);
      printButtStates();
      leftButtonEvent = false;
      rightButtonEvent = false;
      hazardToggle();
      waitLongPress = true;
      startTim9();
    }
    /* if left button is still pressed */
    else if (!hazardEnabled && LEFT_BUTTON == PRESSED)
    {
      stopTim9();
      DEBUG_LOG("LT pressed for %u\r\n", pressTime);
      leftButtonEvent = false;
      leftSideToggle();
      waitLongPress = true;
      startTim9();
    }
    /* if right button is still pressed */
    else if (!hazardEnabled && RIGHT_BUTTON == PRESSED)
    {
      stopTim9();
      DEBUG_LOG("RT pressed for %u\r\n", pressTime);
      rightButtonEvent = false;
      rightSideToggle();
      waitLongPress = true;
      startTim9();
    }
  }

#ifdef __cplusplus
}
#endif