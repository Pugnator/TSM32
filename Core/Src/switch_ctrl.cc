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
  static uint32_t longPressConter = 0;

  uint32_t triggerTime = 0;
#define MIN_PRESS_TIME 500

  inline void sendMessage();

  static void stopTim9()
  {
    HAL_TIM_Base_Stop_IT(&htim9);
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
    PrintF("Button pressed at %u\r\n", triggerTime);
    if (GPIO_Pin == LT_BUTTON_Pin && leftButtonState)
    {
      startTim9();
      leftButtonState = false;
      PrintF("Left switch\r\n");
    }
    else if (GPIO_Pin == RT_BUTTON_Pin && rightButtonState)
    {
      startTim9();
      rightButtonState = false;
      PrintF("Right switch\r\n");
    }
  }
  // HAL_TIM_IRQHandler
  void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
    uint32_t eventTime = HAL_GetTick();
    uint32_t pressTime = eventTime - triggerTime;

    if (TIM9 == htim->Instance)
    {
      if (waitLongPress)
      {
        longPressConter++;
        if (longPressConter == 3)
        {
          if (LEFT_BUTTON == PRESSED || RIGHT_BUTTON == PRESSED)
          {
            PrintF("Long press for %ums\r\n", pressTime);
            overtakeMode = false;
          }
          else
          {
            Print("Short press\r\n");
            overtakeMode = true;
          }
          __HAL_TIM_SET_COUNTER(&htim9, 0);
          HAL_TIM_Base_Stop_IT(&htim9);
          waitLongPress = false;
          longPressConter = 0;
        }
      }

      if (MIN_PRESS_TIME >= pressTime)
      {
        PrintF("Press time [%ums] is less than required [%ums], ignoring event\r\n", pressTime, MIN_PRESS_TIME);
        return;
      }
      /* if both buttons are pressed */
      if (LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED)
      {
        PrintF("Both switches was ON for %ums\r\n", pressTime);
        leftButtonState = true;
        rightButtonState = true;
        blinkerHazardToggle();
        stopTim9();
      }
      /* if left button is still pressed */
      else if (!hazard_blinker_enabled && LEFT_BUTTON == PRESSED)
      {
        stopTim9();
        PrintF("LT pressed for %u\r\n", pressTime);
        leftButtonState = true;
        blinkerLeftsideToggle();
        waitLongPress = true;
        startTim9();
      }
      /* if right button is still pressed */
      else if (!hazard_blinker_enabled && RIGHT_BUTTON == PRESSED)
      {
        stopTim9();
        PrintF("RT pressed for %u\r\n", pressTime);
        rightButtonState = true;
        blinkerRightsideToggle();
        waitLongPress = true;
        startTim9();
      }
    }
    // J1850 service timer, 200us
    else if (TIM6 == htim->Instance)
    {
      messageCollected = true;
      HAL_GPIO_TogglePin(J1850TX_GPIO_Port, J1850TX_Pin);
      if (rxQueryNotEmpty)
      {
        sendMessage();
      }
      HAL_TIM_Base_Stop_IT(&htim6);
    }
    else
    {
      PrintF("Unknown event\r\n");
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
          // PrintF("bit %d is 1\n", bit);
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
          // PrintF("bit %d is 0\n", bit);
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