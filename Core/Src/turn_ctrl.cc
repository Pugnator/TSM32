#include "tsm.h"
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
  bool blink_pause = false;

  uint32_t blink_counter = 0;

  void blinkerOff()
  {
    Print("Both sides off\r\n");
    hazardEnabled = false;
    leftEnabled = false;
    rightEnabled = false;
    RIGHT_PWM_OUT = 0;
    LEFT_PWM_OUT = 0;
  }

  void blinkerLeftsideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    Print("Left Side toggle\r\n");
    blink_pause = false;
    blink_counter = 0;
    leftEnabled = !leftEnabled;
    if (leftEnabled)
    {
      blinkerRightSideOff();
    }
    else
    {
      blinkerLeftSideOff();
    }
  }

  void blinkerLeftSideOff()
  {
    Print("Left side off\r\n");
    leftEnabled = false;
    LEFT_PWM_OUT = 0;
  }

  void blinkerRightsideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    Print("Right Side toggle\r\n");
    blink_counter = 0;
    blink_pause = false;
    rightEnabled = !rightEnabled;
    if (rightEnabled)
    {
      blinkerLeftSideOff();
    }
    else
    {
      blinkerRightSideOff();
    }
  }

  void blinkerRightSideOff()
  {
    Print("Right side off\r\n");
    rightEnabled = false;
    RIGHT_PWM_OUT = 0;
  }

  void blinkerHazardToggle()
  {
    Print("Hazard toggle\r\n");
    hazardEnabled = !hazardEnabled;
    if (!hazardEnabled)
    {
      blinkerOff();
    }
  }

  static void blinker_on()
  {
    for (uint16_t period = 0; period <= 99; period += hazardEnabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP)
    {
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
      HAL_Delay(PWM_DUTY_DELAY);
    }
    HAL_Delay(TURN_OFF_DELAY);
    if (leftEnabled || hazardEnabled)
    {
      LEFT_PWM_OUT = 0;
    }
    else if (rightEnabled || hazardEnabled)
    {
      RIGHT_PWM_OUT = 0;
    }
  }

  static void blinker_off()
  {
    for (uint16_t period = 0; period <= 99; period += hazardEnabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP)
    {
      if (hazardEnabled)
      {
        LEFT_PWM_OUT = 0;
        RIGHT_PWM_OUT = 0;
      }
      else if (leftEnabled)
      {
        LEFT_PWM_OUT = 0;
      }
      else if (rightEnabled)
      {
        RIGHT_PWM_OUT = 0;
      }
      HAL_Delay(PWM_DUTY_DELAY);
    }
    HAL_Delay(TURN_OFF_DELAY);
    if (leftEnabled || hazardEnabled)
    {
      LEFT_PWM_OUT = 0;
    }
    else if (rightEnabled || hazardEnabled)
    {
      RIGHT_PWM_OUT = 0;
    }
  }

  void blinkerWorker()
  {
    if (!hazardEnabled && !leftEnabled && !rightEnabled)
    {
      return;
    }

    if (overtakeMode && OVERTAKE_BLINK_COUNT <= blink_counter)
    {
      blinkerOff();
      overtakeMode = false;
      blink_counter = 0;
      return;
    }

    if (!blink_pause)
    {
      blink_counter++;
      blinker_on();
    }
    else
    {
      blinker_off();
    }

    blink_pause = !blink_pause;
  }

#ifdef __cplusplus
}
#endif