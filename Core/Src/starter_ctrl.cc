#include "tsm.h"
#include "settings.h"

#ifdef __cplusplus
extern "C"
{
#endif
  void disableStarter()
  {
#if STARTER_LOCK_ON
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_RESET);
#endif
  }

  void enableStarter()
  {
#if STARTER_LOCK_ON
    HAL_GPIO_WritePin(STARTER_RELAY_GPIO_Port, STARTER_RELAY_Pin, GPIO_PIN_SET);
#endif
  }

#ifdef __cplusplus
}
#endif