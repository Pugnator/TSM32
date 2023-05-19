#include "tsm.h"
#include "settings.h"
#include "kalman.h"

#ifdef __cplusplus
extern "C"
{
#endif

  bool adcDMAcompleted = false;
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];

  uint8_t SIDEMARK_BRIGHTNESS = 10;

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
  if (adcDMAcompleted)
  {
    for (uint32_t i = 0; i < ADC_DMA_BUF_SIZE; i++)
    {
      adcfilter->updateEstimate(adcDMAbuffer[i] & 0xFFF);
    }

    if (ADC_13V_VALUE < adcfilter->getEstimate())
    {
      disableStarter();
#if AUTO_LIGHT_ENABLE
      SIDEMARK_BRIGHTNESS = DLR_BRIGHTNESS_VALUE;
#endif
    }
    else
    {
      enableStarter();
      SIDEMARK_BRIGHTNESS = 0;
    }
    LEFT_PWM_OUT = leftEnabled ? LEFT_PWM_OUT : SIDEMARK_BRIGHTNESS;
    RIGHT_PWM_OUT = rightEnabled ? RIGHT_PWM_OUT : SIDEMARK_BRIGHTNESS;
    adcDMAcompleted = false;
    HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, sizeof(adcDMAbuffer));
  }
}
