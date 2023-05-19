#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static bool engineStarted = false;

  void disableStarter()
  {
#if STARTER_LOCK_ENABLE
    engineStarted = true;
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);

#endif
  }

  void enableStarter()
  {
#if STARTER_LOCK_ENABLE

#if STARTER_UNLOCK_DISABLE
    if (engineStarted)
    {
      // DEBUG_LOG("Engine was started, unable to unlock the starter.");
      //  Once an engine is started the only way to restart it is to cycle an ignition.
      return;
    }
#endif

    // DEBUG_LOG("Starter is enabled.");
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
#endif
  }

#ifdef __cplusplus
}
#endif