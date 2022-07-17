#include "tsm.h"
#include "settings.h"

#include "mpu.h"
#include "usart.h"
#include "j1850.h"
#include <stdio.h>
#include "dwtdelay.h"
#include "id.h"
#include "vmmu.h"
#include <memory>

#ifdef __cplusplus
extern "C"
{
#endif

  void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }

  void tsmRunApp()
  {
    uint32_t id[3] = {0};
    getCPUid(id, STM32F4_t);
    DEBUG_LOG("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n", id[0], id[1], id[2], VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);
    DWT_Init();

    kalmanInit(2, 2, 0.01);

    /*Battery watchdog*/
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start_IT(&hadc1);

    /*J1850 logger*/
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    /*Blinker bulb PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /*Starter enable*/
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);

    leftSideOff();
    rightSideOff();
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);

    std::unique_ptr<MPU9250> mpu = std::make_unique<MPU9250>(&hi2c1);
    float az = 0;
    uint32_t azCount = 0;
    while (1)
    {
      if (mpu->ok())
      {
        az += mpu->getAzimuth();
        azCount++;
        if (100 == azCount)
        {
          az /= 100;
          DEBUG_LOG("\rAZ=%.1f\r", az);
          azCount = 0;
        }
      }

      if (!hazardEnabled && !leftEnabled && !rightEnabled)
      {
        continue;
      }

      if (overtakeMode && OVERTAKE_BLINK_COUNT <= blink_counter)
      {
        blinkerOff();
        overtakeMode = false;
        leftEnabled = false;
        rightEnabled = false;
        hazardEnabled = false;
        blink_counter = 0;
        continue;
      }

      if (!blink_pause)
      {
        blink_counter++;
        blinkerOn();
      }
      else
      {
        blinkerOff();
      }

      blink_pause = !blink_pause;

      /*
      if (messageCollected)
      {
        printFrameJ1850();
        messageReset();
        messageCollected = false;
      }


      */
    }
  }

#ifdef __cplusplus
}
#endif