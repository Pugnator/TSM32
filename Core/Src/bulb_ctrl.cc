#include "tsm.h"
#include "settings.h"
#include <cmath>
#include <cstring>
#include <algorithm>

constexpr uint32_t MAX_WINDOW_SIZE = 128;
constexpr uint32_t ADC_SAMPLE_PAUSE = 1000;
// R1 is the resistor between ADC and battery power supply
constexpr float R1 = 10.0; // in kOhm
// R2 is the resistor between ADC and GND
constexpr float R2 = 2.7; // in kOhm
// voltage divider factor is used to calculate actual voltage from ADC value
constexpr float voltageDividerFactor = (R1 + R2) / R2;
constexpr float mvPerBit = 3300.0 / 4095.0;
constexpr float voltageChangeRate = 1000.; // in mV/sample
constexpr int32_t maxADConvertorChange = voltageChangeRate / mvPerBit / voltageDividerFactor;

#ifdef __cplusplus
extern "C"
{
#endif

  bool adcDMAcompleted = false;
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];
  static volatile uint32_t voltageThresholdStartTime = 0;
  static volatile bool wasOverVoltage = false;

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

class MovingAverage
{
private:
  uint32_t values[MAX_WINDOW_SIZE];
  uint32_t size;
  uint32_t index;
  uint64_t sum;

public:
  explicit MovingAverage(uint32_t windowSize) : size(windowSize), index(0), sum(0)
  {
    if (windowSize > MAX_WINDOW_SIZE)
    {
      size = MAX_WINDOW_SIZE;
    }
    for (uint32_t i = 0; i < size; ++i)
    {
      values[i] = ADC_12_2V_VALUE;
      sum += ADC_12_2V_VALUE;
    }
  }

  uint32_t filter(uint32_t newValue)
  {
    sum -= values[index];
    sum += newValue;
    values[index] = newValue;
    index = (index + 1) % size;
    return sum / size;
  }
};

void adcHandler()
{
  static uint32_t prevSample = HAL_GetTick();

  if (HAL_GetTick() - prevSample < ADC_SAMPLE_PAUSE || !adcDMAcompleted)
  {
    return;
  }

  prevSample = HAL_GetTick();
  static uint32_t smoothedAverage = ADC_12_2V_VALUE;

  static MovingAverage filter(ADC_DMA_BUF_SIZE);
  uint32_t prevSmoothedAverage = smoothedAverage;

  uint32_t currentSampleAverage = 0;
  for (int i = 1; i < ADC_DMA_BUF_SIZE; i++)
  {
    currentSampleAverage += adcDMAbuffer[i];
  }
  currentSampleAverage /= ADC_DMA_BUF_SIZE;

  // calculate voltage rise rate
  int32_t voltageRate = int32_t(currentSampleAverage - prevSmoothedAverage);
  if (voltageRate > maxADConvertorChange)
  {
    voltageRate = maxADConvertorChange;
  }
  else if (voltageRate < -maxADConvertorChange)
  {
    voltageRate = -maxADConvertorChange;
  }
  // Update the smoothed average based on the limited voltage rate
  currentSampleAverage = prevSmoothedAverage + voltageRate;
  smoothedAverage = filter.filter(currentSampleAverage);

  // complensation of the resistor's tolerance
  // smoothedAverage *= 0.97f;
#ifdef DEBUG
  float voltage = smoothedAverage * 3.3 / 4095 * voltageDividerFactor;
  DEBUG_LOG("V = %0.2f ADC: %u\r\n", voltage, smoothedAverage);
#endif

  if (smoothedAverage > ADC_12_8V_VALUE)
  {
    // if voltage is above 12.8V we want to turn on sidemarks
    if (!wasOverVoltage)
    {
      wasOverVoltage = true;
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }

    // check if voltage is ABOVE 12.8V for more than 15 seconds
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
    if (wasOverVoltage)
    {
      wasOverVoltage = false;
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
