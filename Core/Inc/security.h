/**
 * @file security.h
 * @brief Security PIN (starter immobilizer) and boot settings menu.
 *
 * When a PIN is configured (EEPROM key EE_KEY_SECURITY_PIN, non-zero), the
 * starter relay stays off after boot until the PIN is entered on the turn
 * signal buttons. The J1850 TSM emulation keeps running the whole time - the
 * FSM is non-blocking and driven from the main loop.
 *
 * Gestures (constants in settings.h):
 *   digit  - press LEFT 1-9 times (each press blips the left lamp),
 *            press RIGHT once to commit (right lamp blips)
 *   abort  - both buttons tapped together restarts the entry
 *   hazard - both buttons held >= SECURITY_CHORD_HAZARD_MS toggles the hazard
 *            lights even while locked (roadside safety)
 *
 * Visual feedback on the turn lamps:
 *   wrong PIN    - both lamps flash rapidly 6x, entry resets
 *   correct PIN  - both lamps two long flashes, starter enabled
 *   lockout      - after SECURITY_MAX_ATTEMPTS wrong PINs, entry is ignored
 *                  for SECURITY_LOCKOUT_MS; both lamps blip every 2 s
 *
 * Settings menu: hold BOTH buttons during power-on (after entering the PIN
 * first if one is set). Item selection uses the same digit gesture:
 *   1 - set/change the PIN (enter SECURITY_PIN_LENGTH digits; committing an
 *       EMPTY first digit instead clears the PIN and disables the lock)
 *   0 (empty commit) - leave the menu
 *
 * Starter policy: the relay boots low (gpio.c). With no PIN it is enabled at
 * init when the reset cause allows; with a PIN it is enabled only on a
 * successful entry - and never on a non-power-on reset, preserving the
 * fail-lock. starter_ctrl additionally refuses enableStarter() while a
 * configured PIN is not unlocked (securityStarterPermitted()), so no fallback
 * path can bypass the lock.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    SECURITY_STATE_IDLE = 0, /**< no gating; normal blinker operation      */
    SECURITY_STATE_LOCKED,   /**< PIN configured, waiting for entry        */
    SECURITY_STATE_LOCKOUT,  /**< too many wrong PINs; input ignored       */
    SECURITY_STATE_MENU,     /**< settings menu (item selection)           */
    SECURITY_STATE_SET_PIN,  /**< collecting a new PIN                     */
  } security_state_t;

  typedef enum
  {
    SECURITY_IND_NONE = 0,
    SECURITY_IND_ERROR,   /**< wrong PIN / invalid input flash             */
    SECURITY_IND_SUCCESS, /**< PIN accepted / saved / cleared flash        */
  } security_indication_t;

  /** Decide the boot state from the stored PIN and the reset cause.
   *  Call after ee_init() and after the lamp PWM timers are started.
   *  settingsRequested = both buttons held at power-on. starterAllowed =
   *  watchdog_reset_allows_starter(resetCause). */
  void securityInit(bool settingsRequested, bool starterAllowed);

  /** Non-blocking FSM step. Call once per main-loop iteration INSTEAD of
   *  blinkerHandler() while securityBusy() - it owns the buttons and lamps. */
  void securityHandler(void);

  /** True while PIN entry / settings menu / feedback owns buttons and lamps. */
  bool securityBusy(void);

  /** Strong override of the weak hook in starter_ctrl.cc: false while a
   *  configured PIN has not been entered this power cycle. */
  bool securityStarterPermitted(void);

  /** Current FSM state (diagnostics and host tests). */
  security_state_t securityState(void);

  /** Last major indication started (diagnostics and host tests). */
  security_indication_t securityLastIndication(void);

#ifdef __cplusplus
}
#endif
