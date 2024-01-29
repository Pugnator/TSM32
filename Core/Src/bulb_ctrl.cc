#include "tsm.h"
#include "settings.h"
#include <cmath>
#include <cstring>

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

constexpr int MAX_WINDOW_SIZE = 256;

class MovingAverage
{
private:
  uint16_t values[MAX_WINDOW_SIZE];
  int size;
  uint16_t index;
  uint64_t sum;

public:
  explicit MovingAverage(int windowSize) : size(windowSize), index(0), sum(0)
  {
    if (windowSize > MAX_WINDOW_SIZE)
    {
      size = MAX_WINDOW_SIZE;
    }
    memset(values, 0, sizeof(values));
  }

  uint16_t filter(uint16_t newValue)
  {
    sum -= values[index];
    sum += newValue;
    values[index] = newValue;
    index = (index + 1) % size;
    return sum / size;
  }
};

uint32_t calculateEMA(uint32_t currentEMA, uint32_t newValue, double alpha)
{
  return alpha * newValue + (1 - alpha) * currentEMA;
}

void adcHandler()
{
  if (!adcDMAcompleted)
  {
    return;
  }

  // alpha is the smoothing factor, the less it is the more smoothing is applied
  static double alpha = 0.1;
  // accumulator for smoothed average
  static uint32_t smoothedAverage = 0;
  static uint32_t prevSample = HAL_GetTick();
  // R1 is the resistor between ADC and battery power supply
  static const float R1 = 10.0; // in kOhm
  // R2 is the resistor between ADC and GND
  static const float R2 = 2.7; // in kOhm
  // voltage divider factor is used to calculate actual voltage from ADC value
  static const float voltageDividerFactor = (R1 + R2) / R2;
  static MovingAverage filter(ADC_DMA_BUF_SIZE);

  for (int i = 1; i < ADC_DMA_BUF_SIZE; i++)
  {
    smoothedAverage = filter.filter(adcDMAbuffer[i]);
  }

  // complensation of the resistor's tolerance
  // smoothedAverage *= 0.97f;

#if DEBUG
  if (HAL_GetTick() - prevSample > 1000)
  {
    prevSample = HAL_GetTick();
    float voltage = smoothedAverage * 3.3 / 4095 * voltageDividerFactor;
    DEBUG_LOG("V = %0.2f ADC: %u\r\n", voltage, smoothedAverage);
  }
#endif

  if (smoothedAverage > ADC_12_8V_VALUE)
  {
    // if voltage is below 12.6V we want to turn off sidemarks
    if (!overVoltage)
    {
      overVoltage = true;
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }

    // check if voltage is ABOVE 12.6V for more than 15 seconds
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
  // we don't want to turn off sidemarks if blinkers are enabled
  else if (!leftEnabled && !rightEnabled && smoothedAverage <= ADC_12_2V_VALUE)
  {
    // if previously upper threshold was exceeded we want to reset the timer
    if (overVoltage)
    {
      overVoltage = false;
      // start measuring time when voltage is below 12V
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }
    // check if voltage is BELOW 12V for more than 15 seconds
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
