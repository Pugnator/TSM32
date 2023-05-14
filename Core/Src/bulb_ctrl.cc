#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static uint32_t running_voltage_count = 0;
  static uint32_t stopped_voltage_count = 0;

  uint8_t SIDEMARK_BRIGHTNESS = 10;  

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  {
    if (ADC1 == hadc->Instance)
    {
      if (hazardEnabled)
      {
        return;
      }

      uint32_t k = (uint32_t)updateEstimate(HAL_ADC_GetValue(&hadc1));

      if (ADC_13V_VALUE < k)
      {
        stopped_voltage_count = 0;
        if (ADC_STABLE_COUNT < running_voltage_count++)
        {
          disableStarter();
          SIDEMARK_BRIGHTNESS = 10;
        }
      }
      else
      {
        running_voltage_count = 0;
        if (ADC_STABLE_COUNT < stopped_voltage_count++)
        {
          enableStarter();
          SIDEMARK_BRIGHTNESS = 0;
        }
      }
      LEFT_PWM_OUT = leftEnabled ? LEFT_PWM_OUT : SIDEMARK_BRIGHTNESS;
      RIGHT_PWM_OUT = rightEnabled ? RIGHT_PWM_OUT : SIDEMARK_BRIGHTNESS;
    }
  }

  void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
  {
  }

#ifdef __cplusplus
}
#endif