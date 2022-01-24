#include "tsm.h"
#include "mpu.h"
#include "j1850.h"
#include "eeprom.h"
#include <stdio.h>

void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

void tsmRunApp()
{
  PrintF("TSM %s %s (%s) started\r\n", VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);
  // kalmanInit(2, 2, 0.01);
  // eeprom_test();
  /*Battery watchdog*/
  // HAL_ADC_Start(&hadc1);
  /*J1850 logger*/
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  /*Blinker bulb PWM*/
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /*Starter enable*/
  // HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);

  // blinker_leftside_off();
  // blinker_rightside_off();
  // mems_setup();
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
  
  while (1)
  {
    if (messageCollected)
    {
      printFrameJ1850();
      messageReset();
      messageCollected = false;     
    }
    // blinker_worker();
    // HAL_ADC_Start_IT(&hadc1);
  }
}