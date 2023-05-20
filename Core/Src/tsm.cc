#include "tsm.h"
#include "settings.h"
#include "mpu.h"
#include "j1850.h"
#include <stdio.h>
#include "id.h"
#include "vmmu.h"
#include <memory>
#include "assert.h"


#ifdef __cplusplus
extern "C"
{
#endif

  bool stopAppExecuting = true;

  void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }
  /*
    static volatile uint8_t rxd1[32];
    static volatile uint8_t rxd2[32];

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      if (UartHandle->Instance == USART1)
      {
        DEBUG_LOG("UART1\r\n");
        // HAL_UART_Transmit(&huart3, (uint8_t *)rxd, 1, 0xFFFF);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)rxd1, 1);
      }
      if (UartHandle->Instance == USART2)
      {
        DEBUG_LOG("UART2\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)rxd2, 1, 100);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)rxd2, 1);
      }
    }
  */

  void tsmRunApp()
  {    
    uint32_t id[3] = {0};
    getCPUid(id, STM32F4_t);
    DEBUG_LOG("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
              id[0], id[1], id[2],
              VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    /*Battery watchdog*/
    // HAL_ADC_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);
    HAL_ADC_Start_IT(&hadc1);

    /*J1850 logger*/
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    /*Blinker bulb PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /*Starter enable*/

    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    leftSideOff();
    rightSideOff();    

#if MEMS_ENABLED
    std::unique_ptr<MPU9250> ahrs;
    // Start AHRS
    ahrs.reset(new MPU9250(&hi2c1));
    if (ahrs->ok())
    {
      // Start autoupdate
      HAL_TIM_Base_Start_IT(&htim11);
    }
#endif
    stopAppExecuting = false;
    workerLoop();
    DEBUG_LOG("Stop!\r\n");
  }

#ifdef __cplusplus
}
#endif