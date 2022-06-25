#include "tsm.h"
#include "mpu.h"
#include "usart.h"
#include "j1850.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CPU_CORE_FREQUENCY_HZ 100000000 

void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

void tsmRunApp()
{ 

  Print("Test\r\n");
  return;
  
  PrintF("TSM %s %s (%s) started\r\n", VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

  kalmanInit(2, 2, 0.01);
  // eeprom_test();
  /*Battery watchdog*/
  HAL_ADC_Start(&hadc1);
  
  /*J1850 logger*/
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  //SWO_Init(0x1, CPU_CORE_FREQUENCY_HZ);
  //ITM_SendChar('a');
  /*Blinker bulb PWM*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /*Starter enable*/
  HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);

  blinker_leftside_off();
  blinker_rightside_off();
  // mems_setup();
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);

  while (1)
  {
    HAL_GPIO_TogglePin(J1850TX_GPIO_Port, J1850TX_Pin);
    /*
    if (messageCollected)
    {
      printFrameJ1850();
      messageReset();
      messageCollected = false;     
    }
    */
    blinker_worker();
    //HAL_ADC_Start_IT(&hadc1);
    
  }
}

#ifdef __cplusplus
}
#endif