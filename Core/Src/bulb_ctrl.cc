#include "tsm.h"
#include "settings.h"
#include "engine_state.h"
#include <cmath>
#include <cstring>
#include <algorithm>

constexpr uint32_t MAX_WINDOW_SIZE = 128;
constexpr uint32_t ADC_SAMPLE_PAUSE = 500;
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

  volatile bool adcDMAcompleted = false;
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];
  static volatile uint32_t voltageThresholdStartTime = 0;
  static volatile bool wasOverVoltage = false;

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  {
    if (ADC1 != hadc->Instance)
    {
      return;
    }
    /* Always flag completion so adcHandler() can restart DMA.
     * Hazard-mode gating is done in adcHandler() to avoid permanent
     * DMA stop after hazard ends (was bug: hazardEnabled here stopped
     * DMA forever once hazard was used). */
    adcDMAcompleted = true;
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
      values[i] = ADC_11_1V_VALUE;
      sum += ADC_11_1V_VALUE;
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
  static uint32_t smoothedAverage = ADC_11_1V_VALUE;

  static MovingAverage filter(ADC_DMA_BUF_SIZE);
  uint32_t prevSmoothedAverage = smoothedAverage;

  uint32_t currentSampleAverage = 0;
  for (int i = 0; i < ADC_DMA_BUF_SIZE; i++)
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

  /* Always restart DMA before any early return so the ISR keeps firing
   * and adcHandler() never processes stale data (was bug: early returns
   * inside the voltage FSM bypassed this block entirely). */
  adcDMAcompleted = false;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);

  /* Freeze FRL/starter state machine during hazard but keep DMA cycling
   * so monitoring resumes immediately after hazard ends. */
  if (hazardEnabled)
  {
    return;
  }

#ifdef DEBUG
  [[maybe_unused]] float voltage = smoothedAverage * 3.3f / 4095.0f * voltageDividerFactor * ADC_VCAL;
  static bool voltageReported = false;
  if (!voltageReported)
  {
    // Use the unfiltered DMA burst mean so the seed value of smoothedAverage
    // does not corrupt the startup reading.
    float rawV = currentSampleAverage * 3.3f / 4095.0f * voltageDividerFactor * ADC_VCAL;
    PrintF("ADC startup: V = %.2f (raw=%u)\r\n", rawV, (unsigned)currentSampleAverage);
    voltageReported = true;
  }
  TRACE_LOG("V = %0.2f ADC: %u\r\n", voltage, smoothedAverage);
#endif

  if (smoothedAverage > ADC_13_4V_VALUE)
  {
    // if voltage is above charging threshold we want to turn on sidemarks
    if (!wasOverVoltage)
    {
      wasOverVoltage = true;
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }

    // check if voltage is above threshold for more than 15 seconds.
    // Only act when J1850 engine state is unknown (no live bus) so the
    // J1850 FSM in engine_state.cc remains the authoritative source.
    if (Engine::getState() == Engine::State::Unknown &&
        HAL_GetTick() - voltageThresholdStartTime > VOLTAGE_DETECTION_THRESHOLD)
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
  else if (!leftEnabled && !rightEnabled && smoothedAverage <= ADC_11_1V_VALUE)
  {
    // if previously upper threshold was exceeded we want to reset the timer
    if (wasOverVoltage)
    {
      wasOverVoltage = false;
      // start measuring time when voltage is below threshold
      voltageThresholdStartTime = HAL_GetTick();
      return;
    }
    // check if voltage is below threshold for more than the low-voltage
    // debounce window (long enough to ignore idle+stoplight droop, cranking).
    // Voltage FSM only acts when J1850 engine state is unknown.
    if (Engine::getState() == Engine::State::Unknown &&
        HAL_GetTick() - voltageThresholdStartTime > LOW_VOLTAGE_DETECTION_THRESHOLD)
    {
      enableStarter();
      currentSidemarkBrightness = 0;
    }
  }
  LEFT_PWM_OUT = leftEnabled ? LEFT_PWM_OUT : currentSidemarkBrightness;
  RIGHT_PWM_OUT = rightEnabled ? RIGHT_PWM_OUT : currentSidemarkBrightness;
}
