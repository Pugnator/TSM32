#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif
  static bool starterDisabled = false;

  void disableStarter()
  {
#if STARTER_LOCK_ENABLE
    if (starterDisabled)
    {
      return;
    }

    static uint32_t crankingStartTime = HAL_GetTick();
    if (HAL_GetTick() - crankingStartTime < STARTER_DISABLE_THRESHOLD)
        return;

    starterDisabled = true;
    DEBUG_LOG("Starter disabled.\r\n");
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);

#endif
  }

  void enableStarter()
  {
#if STARTER_LOCK_ENABLE

#if STARTER_UNLOCK_DISABLE
    if (starterDisabled)
    {
      DEBUG_LOG("Engine was started, unable to unlock the starter.");
      //  Once an engine is started the only way to restart it is to cycle an ignition.
      return;
    }
#endif

    DEBUG_LOG("Starter is enabled.");
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
#endif
  }

#ifdef __cplusplus
}
#endif