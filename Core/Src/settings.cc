#include "tsm.h"

volatile bool settingsMode = false;
;

void startupSettingsHandler()
{
  if (LEFT_BUTTON == GPIO_PIN_SET && RIGHT_BUTTON == GPIO_PIN_SET)
  {
    return;
  }

  settingsMode = true;

  if (RIGHT_BUTTON == GPIO_PIN_RESET)
  {
  }

  settingsMode = false;
}