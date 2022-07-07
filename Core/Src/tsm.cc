#include "tsm.h"
#include "mpu.h"
#include "usart.h"
#include "j1850.h"
#include <stdio.h>
#include "1wire.h"
#include <memory>

#ifdef __cplusplus
extern "C"
{
#endif

  void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }

#define DS2401_READ_ROM_COMMAND 0x33
#define DS2401_FAMILY_CODE 0x01

  void tsmRunApp()
  {
    DEBUG_LOG("TSM %s %s (%s) started\r\n", VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);
    DWT_Init();

    std::unique_ptr<MPU9250> mpu = std::make_unique<MPU9250>(&hi2c1);    

    kalmanInit(2, 2, 0.01);
    // eeprom_test();
    /*Battery watchdog*/
    HAL_ADC_Start(&hadc1);

    /*J1850 logger*/
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    /*Blinker bulb PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /*Starter enable*/
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);

    blinkerLeftSideOff();
    blinkerRightSideOff();    
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);

    while (1)
    {

      /*
      if (messageCollected)
      {
        printFrameJ1850();
        messageReset();
        messageCollected = false;
      }
      */
      //blinkerWorker();      
      
      HAL_Delay(2000);
      mpu->getAzimuth();
      // HAL_ADC_Start_IT(&hadc1);
    }
  }

#ifdef __cplusplus
}
#endif