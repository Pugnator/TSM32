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
#include <math.h>

//#define ENABLED

std::unique_ptr<MPU9250> ahrs;
float initialAzimuth = -1;
float az = 0;

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
    DEBUG_LOG("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
              id[0], id[1], id[2],
              VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

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

    // Start AHRS
    ahrs.reset(new MPU9250(&hi2c1));
    if (ahrs->ok())
    {
      HAL_TIM_Base_Start_IT(&htim11);
    }

    while (1)
    {
      *((char *)0) = 0;

      ahrs->readAccel();
      HAL_Delay(100);
#ifdef ENABLED
      if (!hazardEnabled && !leftEnabled && !rightEnabled)
      {
        continue;
      }

      if (overtakeMode && OVERTAKE_BLINK_COUNT <= blinkCounter)
      {
        blinkerOff();
        overtakeMode = false;
        leftEnabled = false;
        rightEnabled = false;
        hazardEnabled = false;
        blinkCounter = 0;
        continue;
      }

      if (!blinkPause)
      {
        blinkCounter++;
        blinkerOn();
      }
      else
      {
        blinkerOff();
      }
#ifdef USE_MEMS
      if (initialAzimuth == -1)
      {
        initialAzimuth = az;
        DEBUG_LOG("Turning at AZ=%.1f\r\n", initialAzimuth);
      }
      if (!overtakeMode && initialAzimuth != -1)
      {
        if (fabs(initialAzimuth - az) > 25)
        {
          DEBUG_LOG("\r\nTurned at AZ=%.1f\r\n", az);
          blinkerOff();
          overtakeMode = false;
          leftEnabled = false;
          rightEnabled = false;
          hazardEnabled = false;
          blink_counter = 0;
          initialAzimuth = -1;
        }
      }
#endif

      blinkPause = !blinkPause;

#ifdef USE_J1850
      if (messageCollected)
      {
        printFrameJ1850();
        messageReset();
        messageCollected = false;
      }
#endif

#endif
    }
  }

#ifdef __cplusplus
}
#endif