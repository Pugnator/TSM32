/**
 * @file security.cc
 * @brief Security PIN immobilizer and boot settings menu. See security.h.
 *
 * Non-blocking: securityHandler() runs from the main loop in place of
 * blinkerHandler(), so the J1850 emulation (presence heartbeat, security
 * handshake) is never interrupted by PIN entry.
 *
 * Button input is polled and level-debounced here, independent of the EXTI
 * raw-event path used by the blinker; discardButtonEvents() drains that path
 * every iteration so no stale press leaks into the blinker after unlock.
 */
#include "tsm.h"
#include "security.h"
#include "eeprom.h"

#include <string.h>

#define LAMP_FULL 100u /* TIM1 period is 100 -> CCR 100 = full brightness */

#ifdef __cplusplus
extern "C"
{
#endif

  /* ---- state ------------------------------------------------------------ */

  static security_state_t state = SECURITY_STATE_IDLE;
  static bool starterAllowedByReset = false;
  static bool settingsAfterUnlock = false;
  static uint32_t storedPin = 0; /* 0 = no PIN configured */
  static bool unlocked = false;

  /* digit entry */
  static uint8_t pressCount = 0;
  static uint8_t digitsEntered = 0;
  static uint32_t enteredValue = 0;
  static uint32_t lastActivity = 0;

  /* attempts / lockout */
  static uint8_t failedAttempts = 0;
  static uint32_t lockoutStart = 0;
  static uint32_t lockoutBlip = 0;

  /* debounced button levels + edges */
  static bool lStable = false, rStable = false; /* true = pressed */
  static bool lRaw = false, rRaw = false;
  static uint32_t lRawSince = 0, rRawSince = 0;
  static bool lEdge = false, rEdge = false;

  /* both-buttons chord */
  static bool chordActive = false;
  static bool chordConsumed = false;
  static uint32_t chordSince = 0;

  /* lamp feedback pattern player */
  static struct
  {
    bool active;
    bool major; /* error/success/welcome - not interruptible by press acks */
    bool left, right;
    bool phaseOn;
    uint8_t remaining; /* ON phases left */
    uint16_t onMs, offMs;
    uint32_t phaseStart;
  } fb;

  static security_indication_t lastIndication = SECURITY_IND_NONE;

  /* ---- lamp feedback ----------------------------------------------------- */

  static void lampSet(bool left, bool right, bool on)
  {
    if (left)
    {
      LEFT_PWM_OUT = on ? LAMP_FULL : 0;
    }
    if (right)
    {
      RIGHT_PWM_OUT = on ? LAMP_FULL : 0;
    }
  }

  static void fbStart(uint8_t count, uint16_t onMs, uint16_t offMs,
                      bool left, bool right, bool major)
  {
    if (fb.active && fb.major && !major)
    {
      return; /* press acks never interrupt an error/success flash */
    }
    fb.active = true;
    fb.major = major;
    fb.left = left;
    fb.right = right;
    fb.remaining = count;
    fb.onMs = onMs;
    fb.offMs = offMs;
    fb.phaseOn = true;
    fb.phaseStart = HAL_GetTick();
    lampSet(left, right, true);
  }

  static void fbService(void)
  {
    if (!fb.active)
    {
      return;
    }
    const uint32_t now = HAL_GetTick();
    if (fb.phaseOn)
    {
      if (now - fb.phaseStart >= fb.onMs)
      {
        lampSet(fb.left, fb.right, false);
        fb.phaseOn = false;
        fb.phaseStart = now;
        fb.remaining--;
      }
    }
    else if (now - fb.phaseStart >= fb.offMs)
    {
      if (fb.remaining == 0)
      {
        fb.active = false;
        fb.major = false;
        /* Hand the lamps back in their canonical off/DRL state. */
        leftSideOff();
        rightSideOff();
        return;
      }
      lampSet(fb.left, fb.right, true);
      fb.phaseOn = true;
      fb.phaseStart = now;
    }
  }

  static void indicateError(void)
  {
    lastIndication = SECURITY_IND_ERROR;
    fbStart(6, 100, 100, true, true, true);
  }

  static void indicateSuccess(void)
  {
    lastIndication = SECURITY_IND_SUCCESS;
    fbStart(2, 400, 250, true, true, true);
  }

  static void indicateWelcome(void)
  {
    fbStart(1, 800, 100, true, true, true);
  }

  /* ---- entry helpers ----------------------------------------------------- */

  static void entryReset(void)
  {
    pressCount = 0;
    digitsEntered = 0;
    enteredValue = 0;
  }

  static void debounceButtons(void)
  {
    const uint32_t now = HAL_GetTick();
    const bool l = LEFT_BUTTON == PRESSED;
    const bool r = RIGHT_BUTTON == PRESSED;
    lEdge = rEdge = false;

    if (l != lRaw)
    {
      lRaw = l;
      lRawSince = now;
    }
    else if (l != lStable && now - lRawSince >= SECURITY_DEBOUNCE_MS)
    {
      lStable = l;
      lEdge = l; /* press edge only */
    }

    if (r != rRaw)
    {
      rRaw = r;
      rRawSince = now;
    }
    else if (r != rStable && now - rRawSince >= SECURITY_DEBOUNCE_MS)
    {
      rStable = r;
      rEdge = r;
    }
  }

  static void unlockSucceeded(void)
  {
    unlocked = true;
    failedAttempts = 0;
    entryReset();
    indicateSuccess();
    if (starterAllowedByReset)
    {
      enableStarter();
    }
    if (settingsAfterUnlock)
    {
      settingsAfterUnlock = false;
      state = SECURITY_STATE_MENU;
    }
    else
    {
      state = SECURITY_STATE_IDLE;
    }
  }

  static void commitDigit(void)
  {
    const uint8_t digit = pressCount;
    pressCount = 0;

    switch (state)
    {
    case SECURITY_STATE_LOCKED:
      if (digit == 0)
      {
        entryReset();
        indicateError();
        return;
      }
      enteredValue = enteredValue * 10u + digit;
      digitsEntered++;
      if (digitsEntered < SECURITY_PIN_LENGTH)
      {
        return;
      }
      if (enteredValue == storedPin)
      {
        unlockSucceeded();
      }
      else
      {
        entryReset();
        indicateError();
        if (++failedAttempts >= SECURITY_MAX_ATTEMPTS)
        {
          state = SECURITY_STATE_LOCKOUT;
          lockoutStart = HAL_GetTick();
          lockoutBlip = lockoutStart;
        }
      }
      return;

    case SECURITY_STATE_MENU:
      if (digit == 0)
      {
        /* empty commit = leave the menu */
        entryReset();
        indicateSuccess();
        state = SECURITY_STATE_IDLE;
      }
      else if (digit == 1)
      {
        entryReset();
        indicateWelcome();
        state = SECURITY_STATE_SET_PIN;
      }
      else
      {
        entryReset();
        indicateError(); /* unknown menu item */
      }
      return;

    case SECURITY_STATE_SET_PIN:
      if (digit == 0)
      {
        if (digitsEntered == 0)
        {
          /* empty FIRST digit clears the PIN (lock disabled) */
          if (ee_write(EE_KEY_SECURITY_PIN, 0))
          {
            storedPin = 0;
            unlocked = true;
            indicateSuccess();
            state = SECURITY_STATE_IDLE;
          }
          else
          {
            indicateError();
          }
        }
        else
        {
          entryReset();
          indicateError(); /* a digit must be 1-9 */
        }
        return;
      }
      enteredValue = enteredValue * 10u + digit;
      digitsEntered++;
      if (digitsEntered < SECURITY_PIN_LENGTH)
      {
        return;
      }
      if (ee_write(EE_KEY_SECURITY_PIN, enteredValue))
      {
        storedPin = enteredValue;
        unlocked = true;
        entryReset();
        indicateSuccess();
        state = SECURITY_STATE_IDLE;
      }
      else
      {
        entryReset();
        indicateError(); /* flash write failed; PIN unchanged */
      }
      return;

    default:
      return;
    }
  }

  /* ---- public API -------------------------------------------------------- */

  void securityInit(bool settingsRequested, bool starterAllowed)
  {
    starterAllowedByReset = starterAllowed;
    uint32_t v = 0;
    storedPin = ee_read(EE_KEY_SECURITY_PIN, &v) ? v : 0;
    unlocked = storedPin == 0;
    failedAttempts = 0;
    entryReset();
    memset(&fb, 0, sizeof(fb));
    lastIndication = SECURITY_IND_NONE;
    lStable = rStable = lRaw = rRaw = false;
    chordActive = chordConsumed = false;

    if (!starterAllowed)
    {
      /* Fail-lock: a non-power-on reset keeps the starter latched off for
       * the whole cycle - deliberately even past a correct PIN. */
      disableStarter();
    }
    else if (storedPin == 0)
    {
      enableStarter();
    }
    /* else: PIN configured - the relay stays in its boot-low state (no
     * disableStarter() call, so the STARTER_UNLOCK_DISABLE latch is not
     * tripped) and a successful entry enables it. */

    if (storedPin != 0)
    {
      state = SECURITY_STATE_LOCKED;
      settingsAfterUnlock = settingsRequested;
    }
    else if (settingsRequested)
    {
      state = SECURITY_STATE_MENU;
      indicateWelcome();
    }
    else
    {
      state = SECURITY_STATE_IDLE;
    }
    lastActivity = HAL_GetTick();
  }

  bool securityBusy(void)
  {
    return state != SECURITY_STATE_IDLE || fb.active;
  }

  bool securityStarterPermitted(void)
  {
    return storedPin == 0 || unlocked;
  }

  security_state_t securityState(void)
  {
    return state;
  }

  security_indication_t securityLastIndication(void)
  {
    return lastIndication;
  }

  void securityHandler(void)
  {
    /* Drain the EXTI raw-event path so no press collected during PIN entry
     * leaks into the blinker logic after unlock. */
    discardButtonEvents();
    fbService();

    if (state == SECURITY_STATE_IDLE)
    {
      return; /* only finishing a feedback pattern */
    }

    const uint32_t now = HAL_GetTick();
    debounceButtons();

    /* Both-buttons chord: held >= SECURITY_CHORD_HAZARD_MS toggles hazard
     * (roadside safety works even while locked); a short both-tap restarts
     * the entry. Single-button edges are suppressed while both are down. */
    if (lStable && rStable)
    {
      if (!chordActive)
      {
        chordActive = true;
        chordConsumed = false;
        chordSince = now;
      }
      else if (!chordConsumed && now - chordSince >= SECURITY_CHORD_HAZARD_MS)
      {
        chordConsumed = true;
        hazardToggle();
      }
      lastActivity = now;
      return;
    }
    if (chordActive)
    {
      if (!lStable && !rStable)
      {
        if (!chordConsumed)
        {
          entryReset();
          fbStart(2, 100, 100, true, true, false); /* entry restarted */
        }
        chordActive = false;
      }
      return; /* swallow the release edges of the chord */
    }

    if (hazardEnabled)
    {
      /* Hazard owns the lamps (blinkerDoBlink runs from the main loop);
       * suspend entry until it is chorded off again. */
      return;
    }

    /* lockout: ignore input, blip both lamps periodically */
    if (state == SECURITY_STATE_LOCKOUT)
    {
      if (now - lockoutStart >= SECURITY_LOCKOUT_MS)
      {
        failedAttempts = 0;
        state = SECURITY_STATE_LOCKED;
        return;
      }
      if (now - lockoutBlip >= 2000u)
      {
        lockoutBlip = now;
        fbStart(1, 60, 60, true, true, false);
      }
      return;
    }

    /* inactivity: abort a partial entry with the error flash; an idle
     * settings menu simply closes so the blinkers come back. */
    if (now - lastActivity >= SECURITY_ENTRY_TIMEOUT_MS)
    {
      const bool partial = pressCount > 0 || digitsEntered > 0;
      if (partial)
      {
        entryReset();
        indicateError();
      }
      if (state == SECURITY_STATE_MENU || state == SECURITY_STATE_SET_PIN)
      {
        state = unlocked ? SECURITY_STATE_IDLE : SECURITY_STATE_LOCKED;
      }
      lastActivity = now;
      return;
    }

    if (lEdge)
    {
      if (pressCount < 9)
      {
        pressCount++;
      }
      lastActivity = now;
      fbStart(1, 80, 40, true, false, false); /* press ack */
    }
    if (rEdge)
    {
      lastActivity = now;
      fbStart(1, 80, 40, false, true, false); /* commit ack */
      commitDigit();
    }
  }

#ifdef __cplusplus
}
#endif
