#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif
  static bool starterDisabled = false;
  static bool starterEnabledLogged = false;
  static bool starterDisabledLogged = false;

  /* Weak default: permitted. The security module (Core/Src/security.cc)
   * provides the strong override that refuses while a configured PIN has not
   * been entered this power cycle, so no caller (engine FSM, voltage
   * fallback) can enable the starter past the immobilizer. Host tests that
   * do not link the security module get the permissive default. */
  __attribute__((weak)) bool securityStarterPermitted(void)
  {
    return true;
  }

  void disableStarter()
  {
#if STARTER_LOCK_ENABLE
    if (starterDisabled)
    {
      return;
    }

    /* No hidden grace period here: callers (engine-state FSM, voltage FSM)
     * already debounce their detection and expect this call to take effect
     * immediately (Fixes #58). */
    starterDisabled = true;
    starterEnabledLogged = false;
    if (!starterDisabledLogged)
    {
      DEBUG_LOG("Starter disabled.\r\n");
      starterDisabledLogged = true;
    }
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);
#endif
  }

  void enableStarter()
  {
#if STARTER_LOCK_ENABLE
    if (!securityStarterPermitted())
    {
      DEBUG_LOG("Starter enable refused: security PIN not entered.\r\n");
      return;
    }

#if STARTER_UNLOCK_DISABLE
    if (starterDisabled)
    {
      DEBUG_LOG("Engine was started, unable to unlock the starter.");
      //  Once an engine is started the only way to restart it is to cycle an ignition.
      return;
    }
#endif

    /* Keep the lock flag in sync with the relay so a later disableStarter()
     * call is not swallowed by the early-return above (Fixes #58). */
    starterDisabled = false;
    starterDisabledLogged = false;
    if (!starterEnabledLogged)
    {
      DEBUG_LOG("Starter is enabled.\r\n");
      starterEnabledLogged = true;
    }
#endif
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
  }

#ifdef __cplusplus
}
#endif