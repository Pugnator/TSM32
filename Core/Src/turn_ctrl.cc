#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // SAE J590b and associated standards specify 60 - 120 flashes per minute for turn signals, with 90 per minute as a target

  bool left_blinker_enabled = false;
  bool right_blinker_enabled = false;
  bool hazard_blinker_enabled = false;
  bool overtakeMode = false;
  bool blink_pause = false;

  uint32_t blink_counter = 0;

  void blinkerLeftsideToggle()
  {
    if (hazard_blinker_enabled)
    {
      return;
    }
    Print("Left Side toggle\r\n");
    blink_pause = false;
    blink_counter = 0;
    left_blinker_enabled = !left_blinker_enabled;
    if (left_blinker_enabled)
    {
      blinkerRightSideOff();
    }
  }

  void blinkerLeftSideOff()
  {
    Print("Left side blinker off\r\n");
    left_blinker_enabled = false;
  }

  void blinkerRightsideToggle()
  {
    if (hazard_blinker_enabled)
    {
      return;
    }
    Print("Right Side toggle\r\n");
    blink_counter = 0;
    blink_pause = false;
    right_blinker_enabled = !right_blinker_enabled;
    if (right_blinker_enabled)
    {
      blinkerLeftSideOff();
    }
  }

  void blinkerRightSideOff()
  {
    Print("Right side blinker off\r\n");
    right_blinker_enabled = false;
  }

  static void blinker_on()
  {
    for (uint16_t period = 0; period <= 99; period += hazard_blinker_enabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP)
    {
      if (hazard_blinker_enabled)
      {
        LEFT_PWM_OUT = period;
        RIGHT_PWM_OUT = period;
      }
      else if (left_blinker_enabled)
      {
        LEFT_PWM_OUT = period;
      }
      else if (right_blinker_enabled)
      {
        RIGHT_PWM_OUT = period;
      }
      HAL_Delay(PWM_DUTY_DELAY);
    }
    HAL_Delay(TURN_OFF_DELAY);
    if (left_blinker_enabled || hazard_blinker_enabled)
    {
      LEFT_PWM_OUT = 0;
    }
    else if (right_blinker_enabled || hazard_blinker_enabled)
    {
      RIGHT_PWM_OUT = 0;
    }
  }

  static void blinker_off()
  {
    for (uint16_t period = 0; period <= 99; period += hazard_blinker_enabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP)
    {
      if (hazard_blinker_enabled)
      {
        LEFT_PWM_OUT = 0;
        RIGHT_PWM_OUT = 0;
      }
      else if (left_blinker_enabled)
      {
        LEFT_PWM_OUT = 0;
      }
      else if (right_blinker_enabled)
      {
        RIGHT_PWM_OUT = 0;
      }
      HAL_Delay(PWM_DUTY_DELAY);
    }
    HAL_Delay(TURN_OFF_DELAY);
    if (left_blinker_enabled || hazard_blinker_enabled)
    {
      LEFT_PWM_OUT = 0;
    }
    else if (right_blinker_enabled || hazard_blinker_enabled)
    {
      RIGHT_PWM_OUT = 0;
    }
  }

  void blinker_worker()
  {
    if (!hazard_blinker_enabled && !left_blinker_enabled && !right_blinker_enabled)
    {
      return;
    }

    if (overtakeMode && OVERTAKE_BLINK_COUNT <= blink_counter)
    {      
      blinkerLeftSideOff();
      blinkerRightSideOff();
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

  void blinkerHazardToggle()
  {
    Print("Hazard toggle\r\n");
    hazard_blinker_enabled = !hazard_blinker_enabled;
    if (hazard_blinker_enabled)
    {
      right_blinker_enabled = false;
      left_blinker_enabled = false;
    }
  }

#ifdef __cplusplus
}
#endif