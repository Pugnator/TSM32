#include "tsm.h"
#include "mpu.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // SAE J590b and associated standards specify 60 - 120 flashes per minute for turn signals, with 90 per minute as a target

  bool leftEnabled = false;
  bool rightEnabled = false;
  bool hazardEnabled = false;
  bool overtakeMode = false;  
  uint8_t volatile currentSidemarkBrightness = 0;

  uint32_t blinkCounter = 0;

  void leftSideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    DEBUG_LOG("Left Side toggle\r\n");  
    blinkCounter = 0;
    leftEnabled = !leftEnabled;
    if (leftEnabled)
    {
      rightSideOff();
    }
    else
    {
      leftSideOff();
    }
  }

  void leftSideOff()
  {
    DEBUG_LOG("Left side off\r\n");
    leftEnabled = false;
    LEFT_PWM_OUT = currentSidemarkBrightness;
  }

  void rightSideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    DEBUG_LOG("Right Side toggle\r\n");
    blinkCounter = 0;    
    rightEnabled = !rightEnabled;
    if (rightEnabled)
    {
      leftSideOff();
    }
    else
    {
      rightSideOff();
    }
  }

  void rightSideOff()
  {
    DEBUG_LOG("Right side off\r\n");
    rightEnabled = false;
    RIGHT_PWM_OUT = currentSidemarkBrightness;
  }

  void hazardToggle()
  {
    DEBUG_LOG("Hazard toggle [%u]\r\n", hazardEnabled);
    hazardEnabled = !hazardEnabled;
    if (!hazardEnabled)
    {
      DEBUG_LOG("Turning off the hazard\r\n");
      leftSideOff();
      rightSideOff();
      overtakeMode = false;
      blinkCounter = 0;
    }
  }

  void blinkerDoBlink()
  {
    static uint32_t startTick = 0;
    static uint16_t period = 0;
    static bool turnOffStage = false;
    static bool turnOnStage = false;
    static bool pauseStage = false;
    static bool initialized = false;

    if (!initialized)
    {
      initialized = true;
      startTick = HAL_GetTick();
      period = 0;
      turnOffStage = false;
      turnOnStage = true;
      pauseStage = false;
    }

    uint32_t currentTick = HAL_GetTick();
    uint32_t elapsed = currentTick - startTick;

    if (turnOnStage)
    {
      if (elapsed >= PWM_DUTY_DELAY)
      {
        startTick = currentTick;
        if (period < 96)
        {
          period += hazardEnabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP;
        }
        else
        {
          turnOnStage = false;
          turnOffStage = true;
        }

        if (hazardEnabled)
        {
          LEFT_PWM_OUT = period;
          RIGHT_PWM_OUT = period;
        }
        else if (leftEnabled)
        {
          LEFT_PWM_OUT = period;
        }
        else if (rightEnabled)
        {
          RIGHT_PWM_OUT = period;
        }
      }
    }

    if (turnOffStage && elapsed > TURN_OFF_DELAY)
    {
      if (hazardEnabled)
      {
        LEFT_PWM_OUT = 0;
        RIGHT_PWM_OUT = 0;
      }
      else if (leftEnabled || hazardEnabled)
      {
        LEFT_PWM_OUT = 0;
      }
      else if (rightEnabled || hazardEnabled)
      {
        RIGHT_PWM_OUT = 0;
      }
      startTick = currentTick;
      turnOffStage = false;
      pauseStage = true;
    }

    if (pauseStage && elapsed > TURN_OFF_PAUSE)
    {
      initialized = false;
      blinkCounter++;
    }
  }

#ifdef __cplusplus
}
#endif