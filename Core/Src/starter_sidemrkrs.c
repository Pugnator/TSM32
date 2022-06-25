#include "tsm.h"
#include "settings.h"
#include <math.h>

static float _err_measure;
static float _err_estimate;
static float _q;
static float _current_estimate;
static float _last_estimate;
static float _kalman_gain;

static uint32_t running_voltage_count = 0;
static uint32_t stopped_voltage_count = 0;

uint8_t SIDEMARK_BRIGHTNESS = 10;

void kalmanInit(float mea_e, float est_e, float q)
{
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float updateEstimate(float mea)
{
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void setMeasurementError(float mea_e)
{
  _err_measure = mea_e;
}

void setEstimateError(float est_e)
{
  _err_estimate = est_e;
}

void setProcessNoise(float q)
{
  _q = q;
}

float getKalmanGain()
{
  return _kalman_gain;
}

float getEstimateError()
{
  return _err_estimate;
}

static void disable_starter()
{
#if STARTER_LOCK_ON
  HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);
#endif
}

static void enable_starter()
{
#if STARTER_LOCK_ON
  HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
#endif
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (ADC1 == hadc->Instance)
  {
    if (hazard_blinker_enabled)
    {
      return;
    }
    
    uint32_t k = (uint32_t)updateEstimate(HAL_ADC_GetValue(&hadc1));

    if (ADC_13V_VALUE <= k)
    {
      stopped_voltage_count = 0;
      if (100 < running_voltage_count++)
      {
        disable_starter();
        SIDEMARK_BRIGHTNESS = 10;
      }
    }
    else
    {
      running_voltage_count = 0;
      if (100 < stopped_voltage_count++)
      {
        enable_starter();
        SIDEMARK_BRIGHTNESS = 0;
      }
    }
    LEFT_PWM_OUT = left_blinker_enabled ? LEFT_PWM_OUT : SIDEMARK_BRIGHTNESS;
    RIGHT_PWM_OUT = right_blinker_enabled ? RIGHT_PWM_OUT : SIDEMARK_BRIGHTNESS;
  }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{

}