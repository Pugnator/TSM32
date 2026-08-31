#include "tsm.h"
#include "settings.h"
#include "engine_state.h"
#include "voltage_policy.h"

constexpr uint32_t ADC_SAMPLE_PAUSE = 500;
// R1 is the resistor between ADC and battery power supply
constexpr float R1 = 10.0; // in kOhm
// R2 is the resistor between ADC and GND
constexpr float R2 = 2.7; // in kOhm
// voltage divider factor is used to calculate actual voltage from ADC value
constexpr float voltageDividerFactor = (R1 + R2) / R2;

#ifdef __cplusplus
extern "C"
{
#endif

  volatile bool adcDMAcompleted = false;
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
  {
    if (ADC1 != hadc->Instance)
    {
      return;
    }
    /* Always flag completion so adcHandler() can restart DMA. Voltage and
     * engine-state policy continue while turn/hazard PWM owns the outputs. */
    adcDMAcompleted = true;
  }

  void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
  {
    (void)hadc;
    // unused as had unstable results IRL
  }

#ifdef __cplusplus
}
#endif

void adcHandler()
{
  static uint32_t prevSample = HAL_GetTick();
  static VoltagePolicy::Controller voltageController;
  static VoltagePolicy::Region appliedFallbackRegion = VoltagePolicy::Region::Middle;

  if (HAL_GetTick() - prevSample < ADC_SAMPLE_PAUSE || !adcDMAcompleted)
  {
    return;
  }

  prevSample = HAL_GetTick();

  uint32_t currentSampleAverage = 0;
  for (int i = 0; i < ADC_DMA_BUF_SIZE; i++)
  {
    currentSampleAverage += adcDMAbuffer[i];
  }
  currentSampleAverage /= ADC_DMA_BUF_SIZE;
  const VoltagePolicy::Result voltageSample =
      voltageController.update(currentSampleAverage, HAL_GetTick());

  /* Always restart DMA before any early return so the ISR keeps firing
   * and adcHandler() never processes stale data (was bug: early returns
   * inside the voltage FSM bypassed this block entirely). */
  adcDMAcompleted = false;
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);

#ifdef DEBUG
  [[maybe_unused]] float voltage = voltageSample.filteredAdc * 3.3f / 4095.0f * voltageDividerFactor * ADC_VCAL;
  static bool voltageReported = false;
  if (!voltageReported)
  {
    // Report the unfiltered DMA burst mean as the startup measurement.
    [[maybe_unused]] float rawV = currentSampleAverage * 3.3f / 4095.0f * voltageDividerFactor * ADC_VCAL;
    PrintF("ADC startup: V = %.2f (raw=%u)\r\n", rawV, (unsigned)currentSampleAverage);
    voltageReported = true;
  }
  TRACE_LOG("V = %0.2f ADC: %u\r\n", voltage, voltageSample.filteredAdc);
#endif

  const Engine::State engineState = Engine::getState();
  if (engineState == Engine::State::Running ||
      engineState == Engine::State::Moving)
  {
#if AUTO_LIGHT_ENABLE
    currentSidemarkBrightness = DLR_BRIGHTNESS_VALUE;
#else
    currentSidemarkBrightness = 0;
#endif
    appliedFallbackRegion = VoltagePolicy::Region::Middle;
  }
  else if (engineState == Engine::State::Off)
  {
    currentSidemarkBrightness = 0;
    appliedFallbackRegion = VoltagePolicy::Region::Middle;
  }
  else if (voltageSample.highQualified &&
           appliedFallbackRegion != VoltagePolicy::Region::High)
  {
    disableStarter();
#if AUTO_LIGHT_ENABLE
    currentSidemarkBrightness = DLR_BRIGHTNESS_VALUE;
#else
    currentSidemarkBrightness = 0;
#endif
    appliedFallbackRegion = VoltagePolicy::Region::High;
  }
  else if (voltageSample.lowQualified &&
           appliedFallbackRegion != VoltagePolicy::Region::Low)
  {
    enableStarter();
    currentSidemarkBrightness = 0;
    appliedFallbackRegion = VoltagePolicy::Region::Low;
  }

  /* Turn/hazard PWM temporarily owns active channels, but voltage and J1850
   * policy keep running underneath so sidemarks restore to the current DRL
   * state as soon as the signal ends. */
  if (!hazardEnabled)
  {
    if (!leftEnabled)
    {
      LEFT_PWM_OUT = currentSidemarkBrightness;
    }
    if (!rightEnabled)
    {
      RIGHT_PWM_OUT = currentSidemarkBrightness;
    }
  }
}
