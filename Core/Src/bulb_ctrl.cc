#include "tsm.h"
#include "settings.h"
#include "kalman.h"

#ifdef __cplusplus
extern "C"
{
#endif

  bool adcDMAcompleted = false;
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];
  static volatile uint32_t voltageThresholdStartTime = 0;
  static volatile bool overVoltage = false;

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  {
    if (ADC1 != hadc->Instance || hazardEnabled)
    {
      return;
    }

    adcDMAcompleted = true;
    HAL_ADC_Stop_DMA(&hadc1);
  }

  void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
  {
    // unused as had unstable results IRL
  }

#ifdef __cplusplus
}
#endif

void adcHandler()
{
  if (!adcDMAcompleted)
  {
    return;
  }

  static uint32_t meanVoltage = 0;

  for (uint32_t i = 0; i < ADC_DMA_BUF_SIZE; i++)
  {
    meanVoltage += adcDMAbuffer[i] & 0xFFF;
  }

  meanVoltage /= ADC_DMA_BUF_SIZE;
  //DEBUG_LOG("Mean voltage is %u\r\n", meanVoltage);

  if (ADC_13V_VALUE < meanVoltage)
  {
    if (!overVoltage)
    {
      overVoltage = true;
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }

    if (HAL_GetTick() - voltageThresholdStartTime > VOLTAGE_DETECTION_THRESHOLD)
    {
      disableStarter();
#if AUTO_LIGHT_ENABLE
      currentSidemarkBrightness = DLR_BRIGHTNESS_VALUE;
#else
      currentSidemarkBrightness = 0;
#endif
    }
  }
  else if (!leftEnabled && !rightEnabled) //we don't want to turn off sidemarks if blinkers are enabled
  {
    if (overVoltage)
    {
      overVoltage = false;
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }

    if (HAL_GetTick() - voltageThresholdStartTime > VOLTAGE_DETECTION_THRESHOLD)
    {
      enableStarter();
      currentSidemarkBrightness = 0;
    }
  }
  LEFT_PWM_OUT = leftEnabled ? LEFT_PWM_OUT : currentSidemarkBrightness;
  RIGHT_PWM_OUT = rightEnabled ? RIGHT_PWM_OUT : currentSidemarkBrightness;
  adcDMAcompleted = false;
  HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);
}
