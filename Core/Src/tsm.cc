#include "tsm.h"
#include "settings.h"
#include "mpu.h"
#include "j1850.h"

#include <stdio.h>
#include "id.h"
#include "vmmu.h"
#include <memory>
#include "assert.h"
#include "dwtdelay.h"

bool stopAppExecuting = true;

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
    INFO_LOG("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
             id[0], id[1], id[2],
             VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    startupSettingsHandler();

/*Battery watchdog*/
// HAL_ADC_Start(&hadc1);
#if AUTO_LIGHT_ENABLE
    HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);
    HAL_ADC_Start_IT(&hadc1);
#endif

/*J1850 logger*/
#if J1850_ENABLED
    HAL_TIM_IC_Start_IT(&J1850_IC_INSTANCE, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
#endif
    /*Blinker bulb PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    /*Starter enable*/

    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    leftSideOff();
    rightSideOff();

    // uint8_t frame[2] = {0xAA, 0xAA};
    // J1850VPW::sendFrame(frame, 2);

#if MEMS_ENABLED
    std::unique_ptr<MPU9250> ahrs;
    ahrs.reset(new MPU9250(&hi2c1));
    if (ahrs->ok())
    {
      // Start autoupdate
      HAL_TIM_Base_Start_IT(&htim11);
    }
#endif
    stopAppExecuting = false;
    while (!stopAppExecuting)
    {

#if AUTO_LIGHT_ENABLE
      adcHandler();
#endif

#if J1850_ENABLED
      if (messageCollected)
      {
        J1850VPW::printFrame();
        J1850VPW::messageReset();
      }
#endif

#if MEMS_ENABLED
      auto az = ahrs->getHeadingAngle();
      DEBUG_LOG("Turning at AZ=%.1f\r\n", az);
#endif

#if BLINKER_ENABLED
      if (hazardEnabled || leftEnabled || rightEnabled)
      {
        blinkerDoBlink();
      }

      if (overtakeMode && OVERTAKE_BLINK_COUNT < blinkCounter)
      {
        overtakeMode = false;
        leftEnabled = false;
        rightEnabled = false;
        hazardEnabled = false;
        blinkCounter = 0;
      }
#endif
    }
    DEBUG_LOG("Stop!\r\n");
  }

#ifdef __cplusplus
}
#endif