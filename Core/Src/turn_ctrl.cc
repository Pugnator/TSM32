#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // SAE J590b and associated standards specify 60 - 120 flashes per minute for turn signals, with 90 per minute as a target

  /* These flags are written from ISR context (switch_ctrl.cc) and read
   * from both the ISR and the main loop (tsm.cc). Mark them volatile
   * so the compiler does not cache them across function boundaries
   * under -O3 / -flto. */
  volatile bool leftEnabled = false;
  volatile bool rightEnabled = false;
  volatile bool hazardEnabled = false;
  volatile bool overtakeMode = false;
  volatile bool postTurnTailActive = false;
  uint8_t volatile currentSidemarkBrightness = 0;

  volatile uint32_t blinkCounter = 0;
  /* Set by leftSideOff()/rightSideOff() to force blinkerDoBlink() to restart
   * from the beginning of the ON ramp on the next call, so turning the blinker
   * off mid-cycle and then back on always starts a fresh cycle. */
  static volatile bool blinkerResetPending = false;

  static void cancelTimedModes()
  {
    overtakeMode = false;
    postTurnTailActive = false;
    blinkCounter = 0;
  }

  static void restartTimedBlinkCycle()
  {
    /* A cycle that began before mode classification/turn detection is not a
     * complete timed flash. Restart from dark so the configured count is
     * exact and entirely after the transition. */
    if (leftEnabled)
    {
      LEFT_PWM_OUT = 0;
    }
    if (rightEnabled)
    {
      RIGHT_PWM_OUT = 0;
    }
    blinkerResetPending = true;
  }

  void leftSideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    DEBUG_LOG("Left Side toggle\r\n");
    cancelTimedModes();
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

  void rightSideToggle()
  {
    if (hazardEnabled)
    {
      return;
    }

    DEBUG_LOG("Right Side toggle\r\n");
    rightEnabled = !rightEnabled;
    cancelTimedModes();
    if (rightEnabled)
    {
      leftSideOff();
    }
    else
    {
      rightSideOff();
    }
  }

  void leftSideOff()
  {
    DEBUG_LOG("Left side off\r\n");
    leftEnabled = false;
    LEFT_PWM_OUT = currentSidemarkBrightness;
    blinkerResetPending = true;
  }

  void rightSideOff()
  {
    DEBUG_LOG("Right side off\r\n");
    rightEnabled = false;
    RIGHT_PWM_OUT = currentSidemarkBrightness;
    blinkerResetPending = true;
  }

  void hazardToggle()
  {
    DEBUG_LOG("Hazard toggle [%u]\r\n", hazardEnabled);
    if (hazardEnabled)
    {
      DEBUG_LOG("Turning off the hazard\r\n");
      hazardEnabled = false;
      leftSideOff();
      rightSideOff();
      cancelTimedModes();
      return;
    }

    /* Hazard owns both outputs and must never inherit a lane-change or
     * post-turn countdown from the previously active side. */
    leftSideOff();
    rightSideOff();
    cancelTimedModes();
    hazardEnabled = true;
  }

  void startPostTurnTail()
  {
    if (hazardEnabled || (!leftEnabled && !rightEnabled))
    {
      return;
    }

    overtakeMode = false;
    postTurnTailActive = true;
    blinkCounter = 0;
    restartTimedBlinkCycle();
    DEBUG_LOG("Post-turn blinker tail armed\r\n");
  }

  void startOvertakeMode()
  {
    if (hazardEnabled || (!leftEnabled && !rightEnabled))
    {
      return;
    }

    postTurnTailActive = false;
    overtakeMode = true;
    blinkCounter = 0;
    restartTimedBlinkCycle();
    DEBUG_LOG("Overtake blinker mode armed\r\n");
  }

  void blinkerAutoCancelHandler()
  {
    if (hazardEnabled)
    {
      return;
    }

    const uint32_t blinkLimit = postTurnTailActive
                                    ? POST_TURN_BLINK_COUNT
                                    : OVERTAKE_BLINK_COUNT;
    if ((!overtakeMode && !postTurnTailActive) || blinkCounter < blinkLimit)
    {
      return;
    }

    DEBUG_LOG("Deactivating timed blinker after %u complete flashes\r\n",
              (unsigned)blinkCounter);
    cancelTimedModes();
    leftSideOff();
    rightSideOff();
  }

  void blinkerDoBlink()
  {
    static uint32_t startTick = 0;
    static uint16_t period = 0;
    static bool turnOffStage = false;
    static bool turnOnStage = false;
    static bool pauseStage = false;
    static bool initialized = false;

    if (!initialized || blinkerResetPending)
    {
      initialized = true;
      blinkerResetPending = false;
      startTick = HAL_GetTick();
      period = 0;
      turnOffStage = false;
      turnOnStage = true;
      pauseStage = false;
    }

    uint32_t currentTick = HAL_GetTick();
    /* Re-zeroed by every stage transition below, because each transition also
     * restarts startTick. Without that, one slow main-loop iteration (J1850
     * frame burst, ADC handler, RTT trace) could carry a single large elapsed
     * value through two stage tests in the same call: the OFF transition and
     * then the pause test, skipping the dark pause entirely and showing up as
     * a brief off/on flicker instead of a proper gap between flashes. */
    uint32_t elapsed = currentTick - startTick;

    if (turnOnStage)
    {
      if (elapsed >= PWM_DUTY_DELAY)
      {
        startTick = currentTick;
        elapsed = 0;
        if (period < 96)
        {
          period += hazardEnabled ? PWM_HAZARD_DUTY_STEP : PWM_ON_DUTY_STEP;
        }
        else
        {
          DEBUG_LOG("Blink ON\r\n");
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
      DEBUG_LOG("Blink OFF\r\n");
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
      elapsed = 0;
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
