#include "tsm.h"
#include "j1850.h"
#include "dwtdelay.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static bool leftButtonState = true;
  static bool rightButtonState = true;

  static bool waitLongPress = false;
  static uint32_t longPressCounter = 0;

  uint32_t triggerTime = 0;

  inline void sendMessage();

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
    triggerTime = HAL_GetTick();
    DEBUG_LOG("Button pressed at %u [L=%u R=%u]\r\n", triggerTime, leftButtonState, rightButtonState);
    if (GPIO_Pin == LT_BUTTON_Pin && leftButtonState)
    {
      startTim9();
      leftButtonState = false;
      DEBUG_LOG("Left switch\r\n");
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && rightButtonState)
    {
      startTim9();
      rightButtonState = false;
      DEBUG_LOG("Right switch\r\n");
    }
  }

  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    if (TIM6 != htim->Instance && TIM9 != htim->Instance)
    {
      return;
    }

    // J1850 service timer, 200us
    if (TIM6 == htim->Instance)
    {
      messageCollected = true;
      HAL_GPIO_TogglePin(J1850TX_GPIO_Port, J1850TX_Pin);
      if (rxQueryNotEmpty)
      {
        sendMessage();
      }
      HAL_TIM_Base_Stop_IT(&htim6);
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
        overtakeMode = false;
      }
      else
      {
        DEBUG_LOG("Short press\r\n");
        overtakeMode = true;
      }
      stopTim9();

      waitLongPress = false;
      leftButtonState = true;
      rightButtonState = true;
      longPressCounter = 0;
      return;
    }

    //too quick press - skip
    if (MIN_PRESS_TIME >= pressTime)
    {
      DEBUG_LOG("Press time [%ums] is less than required [%ums], ignoring event\r\n", pressTime, MIN_PRESS_TIME);
      DEBUG_LOG("States:\r\nRight %s\r\nLeft %s\r\nOvertake %s\r\nWaiting for Long Press: %s\r\nLong Press counter %u\r\n",
             rightButtonState ? "ON" : "OFF", leftButtonState ? "ON" : "OFF", overtakeMode ? "ON" : "OFF", waitLongPress ? "ON" : "OFF", longPressCounter);
      return;
    }
    /* if both buttons are pressed */
    if (LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED)
    {
      DEBUG_LOG("Both switches was ON for %ums\r\n", pressTime);
      leftButtonState = true;
      rightButtonState = true;
      blinkerHazardToggle();
      waitLongPress = true;
      // stopTim9();
      startTim9();
    }
    /* if left button is still pressed */
    else if (!hazardEnabled && LEFT_BUTTON == PRESSED)
    {
      stopTim9();
      DEBUG_LOG("LT pressed for %u\r\n", pressTime);
      leftButtonState = true;
      blinkerLeftsideToggle();
      waitLongPress = true;
      startTim9();
    }
    /* if right button is still pressed */
    else if (!hazardEnabled && RIGHT_BUTTON == PRESSED)
    {
      stopTim9();
      DEBUG_LOG("RT pressed for %u\r\n", pressTime);
      rightButtonState = true;
      blinkerRightsideToggle();
      waitLongPress = true;
      startTim9();
    }
  }

  inline void sendMessage()
  {
    bool bitActive = false;
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
    DWT_Delay(TX_SOF);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    for (size_t i = 0; i < sendBufLen; i++)
    {
      size_t bit = 7;
      uint8_t temp = sendBufJ1850[i];
      while (bit >= 0)
      {
        if (temp & 0x01)
        {
          // 1
          // DEBUG_LOG("bit %d is 1\n", bit);
          if (bitActive)
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
            DWT_Delay(TX_SHORT);
          }
          else
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
            DWT_Delay(TX_LONG);
          }
        }
        else
        {
          // 0
          // DEBUG_LOG("bit %d is 0\n", bit);
          if (bitActive)
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
            DWT_Delay(TX_LONG);
          }
          else
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
            DWT_Delay(TX_SHORT);
          }
        }

        bit--;
        bitActive = !bitActive;
        temp = temp >> 1;
      }
    }
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    rxQueryNotEmpty = false;
    sendBufLen = 0;
  }

#ifdef __cplusplus
}
#endif