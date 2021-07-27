#include "tsm.h"
#include "j1850.h"

bool rising_edge_previously = false;
uint32_t rise_edge_time = 0;
uint32_t fall_edge_time = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  #if J1850_ENABLED  
  if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
  {
    return;
  }
  if (!rising_edge_previously)
  {
    rising_edge_previously = true;

    rise_edge_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (rise_edge_time > fall_edge_time)
    {
      uint32_t passive_pulse = rise_edge_time - fall_edge_time;
      if (passive_pulse > RX_IFS_MIN)
      {
        PrintF("IFS, %uus\r\n", passive_pulse);
      }
      else if (passive_pulse > RX_EOF_MIN)
      {
        PrintF("EOF, %uus\r\n", passive_pulse);
      }
      else if (RX_EOD_MAX >= passive_pulse && passive_pulse > RX_EOD_MIN)
      {
        PrintF("EOD, %uus\r\n", passive_pulse);
      }
      else if (RX_LONG_MAX >= passive_pulse && passive_pulse > RX_LONG_MIN)
      {
        PrintF("Passive 1, %uus\r\n", passive_pulse);
      }
      else if (RX_SHORT_MAX >= passive_pulse && passive_pulse > RX_SHORT_MIN)
      {
        PrintF("Passive 0, %uus\r\n", passive_pulse);
      }
    }
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else
  {
    __HAL_TIM_SET_COUNTER(htim, 0);
    fall_edge_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (fall_edge_time > rise_edge_time)
    {
      uint32_t active_pulse = fall_edge_time - rise_edge_time;
      if (1000 * 1000 > active_pulse && active_pulse > 239)
      {
        //PrintF("Break, %uus\r\n", active_pulse);
      }
      else if (RX_SOF_MAX >= active_pulse && active_pulse > RX_SOF_MIN)
      {
        PrintF("SOF, %uus\r\n", active_pulse);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
      else if (RX_LONG_MAX >= active_pulse && active_pulse > RX_LONG_MIN)
      {
        PrintF("Active 0, %uus\r\n", active_pulse);
      }
      else if (RX_SHORT_MAX >= active_pulse && active_pulse > RX_SHORT_MIN)
      {
        PrintF("Active 1, %uus\r\n", active_pulse);
      }
    }

    rising_edge_previously = false;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
  }
  #endif
}

void j1850_break()
{
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, 1);
  HAL_Delay(5);
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, 0);
}