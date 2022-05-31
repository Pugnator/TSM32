#include "tsm.h"
#include "j1850.h"
#include "dwtdelay.h"

static bool leftButtonState = true;
static bool rightButtonState = true;

static bool waitLongPress = false;
static uint32_t longPressConter = 0;

uint32_t triggerTime = 0;
#define MIN_PRESS_TIME 500

inline void sendMessage();

static void stopTim4()
{
  HAL_TIM_Base_Stop_IT(&htim4);
}

static void startTim4()
{
  stopTim4();
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  HAL_TIM_Base_Start_IT(&htim4);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  triggerTime = HAL_GetTick();
  PrintF("Button pressed at %u\r\n", triggerTime);
  if (GPIO_Pin == LT_BUTTON_Pin && leftButtonState)
  {
    startTim4();
    leftButtonState = false;
  }
  else if (GPIO_Pin == RT_BUTTON_Pin && rightButtonState)
  {
    startTim4();
    rightButtonState = false;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t eventTime = HAL_GetTick();
  uint32_t pressTime = eventTime - triggerTime;
  if (TIM4 == htim->Instance)
  {
    if (waitLongPress)
    {
      longPressConter++;
      if (longPressConter == 3)
      {
        if (LEFT_BUTTON == PRESSED || RIGHT_BUTTON == PRESSED)
        {
          PrintF("Long press for %u\r\n", pressTime);
          overtakeMode = false;
        }
        else
        {
          Print("Short press\r\n");
          overtakeMode = true;
        }
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Stop_IT(&htim4);
        waitLongPress = false;
        longPressConter = 0;
      }
    }
    if (MIN_PRESS_TIME >= pressTime)
    {
      PrintF("Press time [%u] is less than required [%u], ignoring event\r\n", pressTime, MIN_PRESS_TIME);
      return;
    }
    /* if both buttons are pressed */
    if (LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED)
    {
      PrintF("Hazard light for %u\r\n", pressTime);
      leftButtonState = true;
      rightButtonState = true;
      blinkerHazardToggle();
      stopTim4();
    }
    /* if left button is still pressed */
    else if (LEFT_BUTTON == PRESSED)
    {
      stopTim4();
      PrintF("LT pressed for %u\r\n", pressTime);
      leftButtonState = true;
      blinkerLeftsideToggle();
      waitLongPress = true;
      startTim4();
    }
    /* if right button is still pressed */
    else if (RIGHT_BUTTON == PRESSED)
    {
      stopTim4();
      PrintF("RT pressed for %u\r\n", pressTime);
      rightButtonState = true;
      blinkerRightsideToggle();
      waitLongPress = true;
      startTim4();
    }
  }
  // J1850 service timer, 200us
  else if (TIM3 == htim->Instance)
  {
    messageCollected = true;
    HAL_GPIO_TogglePin(J1850TX_GPIO_Port, J1850TX_Pin);
    if (rxQueryNotEmpty)
    {
      sendMessage();
    }
    HAL_TIM_Base_Stop_IT(&htim3);
  }
}

inline void sendMessage()
{
  bool bitActive = false;
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
  DWT_Delay(TX_SOF);
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
  for (int i = 0; i < sendBufLen; i++)
  {
    int bit = 7;
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