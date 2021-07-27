#include "tsm.h"

static bool left_button_state = true;
static bool right_button_state = true;

uint32_t trigger_time = 0;
#define MIN_PRESS_TIME 500

static void stop_tim4()
{
  HAL_TIM_Base_Stop_IT(&htim4);
}

static void stop_tim3()
{
  HAL_TIM_Base_Stop_IT(&htim3);
}

static void start_tim3()
{
  stop_tim3();
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Base_Start_IT(&htim3);
}

static void start_tim4()
{
  stop_tim4();
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  HAL_TIM_Base_Start_IT(&htim4);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  trigger_time = HAL_GetTick();
  PrintF("Button pressed at %u\r\n", trigger_time);
  if (GPIO_Pin == LT_BUTTON_Pin && left_button_state)
  {
    start_tim4();
    left_button_state = false;
  }
  else if (GPIO_Pin == RT_BUTTON_Pin && right_button_state)
  {
    start_tim4();
    right_button_state = false;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t event_time = HAL_GetTick();
  uint32_t press_time = event_time - trigger_time;
  if (TIM4 == htim->Instance)
  {
    if(MIN_PRESS_TIME >= press_time)
    {
      PrintF("Press time [%u] is less than required [%u], ignoring event\r\n", press_time, MIN_PRESS_TIME);
      return;
    }
    /* if both buttons are pressed */
    if (LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED)
    {
      PrintF("Hazard light for %u\r\n", press_time);
      left_button_state = true;
      right_button_state = true;
      blinker_hazard_toggle();
      stop_tim4();
    }
    /* if left button is still pressed */
    else if (LEFT_BUTTON == PRESSED)
    {
      stop_tim4();
      PrintF("LT pressed for %u\r\n", press_time);
      left_button_state = true;
      blinker_leftside_toggle();
      start_tim3();
    }
    /* if right button is still pressed */
    else if (RIGHT_BUTTON == PRESSED)
    {      
      stop_tim4();
      PrintF("RT pressed for %u\r\n", press_time);
      right_button_state = true;
      blinker_rightside_toggle();
      start_tim3();
    }
  }
  else if (TIM3 == htim->Instance)
  {
    if (LEFT_BUTTON == PRESSED || RIGHT_BUTTON == PRESSED)
    {
      PrintF("Long press for %u\r\n", press_time);
      overtake_mode = false;
    }
    else
    {
      Print("Short press\r\n");
      overtake_mode = true;
    }
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Stop_IT(&htim3);
  }
}