#include "tsm.h"
#include "settings.h"
#include "j1850.h"
#include "spi.h"
#include "imu_spi.h"
#include "ahrs.h"

#include <stdio.h>
#include "id.h"
#include "vmmu.h"
#include "assert.h"
#include "dwtdelay.h"

bool stopAppExecuting = true;

Quaternion mpuQ;
float ypr[3];
float yprDeg[3];
bool trackingEnabled = false;
int16_t initialYaw = INT16_MIN;
uint32_t initialTime = 0;

#ifdef __cplusplus
extern "C"
{
#endif

  void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }

  static bool detectTurn(int16_t initialYaw, int16_t currentYaw, int16_t threshold)
  {
    int16_t yaw_difference = currentYaw - initialYaw;
    if (yaw_difference > 180)
      yaw_difference -= 360;
    else if (yaw_difference < -180)
      yaw_difference += 360;

    if (abs(yaw_difference) > threshold)
    {
      //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      return true;
    }

    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    return false;
  }

  void tsmRunApp()
  {
    uint32_t id[3] = {0};
    getCPUid(id, STM32F1_t);
    INFO_LOG("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
             id[0], id[1], id[2],
             VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    startupSettingsHandler();

/*Battery watchdog*/
#if AUTO_LIGHT_ENABLE
    HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);
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

    leftSideOff();
    rightSideOff();      

    /*
    uint8_t test = 0xAA;
    uint8_t res = 0;
    HAL_SPI_TransmitReceive(&hspi1, &test, &res, 1, 100);
    DEBUG_LOG("Send %.2X, got %.2X\r\n", test, res);
    return;  
    */
    // uint8_t frame[2] = {0xAA, 0xAA};
    // J1850VPW::sendFrame(frame, 2);

#if MEMS_ENABLED
    std::unique_ptr<Ahrs::AhrsBase<Mpu9250::Mpu9250Spi>> mpu(new Ahrs::AhrsBase<Mpu9250::Mpu9250Spi>(&hspi1, false, Mpu9250::MagMode::MasterMode));
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

#if BLINKER_ENABLED
      if (hazardEnabled || leftEnabled || rightEnabled)
      {
        if (!hazardEnabled && !trackingEnabled)
        {
          trackingEnabled = true;
          initialTime = HAL_GetTick();
          initialYaw = INT16_MIN;
          DEBUG_LOG("Blinker started at %lu\r\n", initialTime);
        }
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

#if MEMS_ENABLED
      if (!mpu->ok())
        continue;

      mpu->sampleQuant();

      if (hazardEnabled || (!leftEnabled && !rightEnabled))
      {
        trackingEnabled = false;
        continue;
      }

      auto ypr = mpu->getYawPitchRollD();
     
      if (initialYaw == INT16_MIN)
      {
        initialYaw = ypr.x;
        DEBUG_LOG("Initial yaw = %.3d\r\n", initialYaw);
        continue;
      }

      if (!detectTurn(initialYaw, ypr.x, TURN_ANGLE_THRESHOLD) &&
          HAL_GetTick() - initialTime < TURN_MAX_TIME_MS)
        continue;

      DEBUG_LOG("Turn detected, yaw = %.3d\r\n", ypr.x);

      if (leftEnabled)
      {
        leftSideToggle();
      }
      else if (rightEnabled)
      {
        rightSideToggle();
      }

      trackingEnabled = false;
      initialYaw = INT16_MIN;

#endif
    }
    DEBUG_LOG("Stop!\r\n");
  }

#ifdef __cplusplus
}
#endif