#include "tsm.h"
#include "settings.h"

void blinkerHandler()
{
#if BLINKER_ENABLED
  if (!hazardEnabled && !leftEnabled && !rightEnabled)
  {
    return;
  }

  if (overtakeMode && OVERTAKE_BLINK_COUNT <= blinkCounter)
  {
    blinkerOff();
    overtakeMode = false;
    leftEnabled = false;
    rightEnabled = false;
    hazardEnabled = false;
    blinkCounter = 0;
    return;
  }

  if (!blinkPause)
  {
    blinkCounter++;
    blinkerOn();
  }
  else
  {
    blinkerOff();
  }

  blinkPause = !blinkPause;
#endif
}

void memsHandler()
{
#if MEMS_ENABLED
  auto az = ahrs->getHeadingAngle();
  DEBUG_LOG("Turning at AZ=%.1f\r\n", az);
#endif
}

void j1850Handler()
{
#if J1850_ENABLED
  if (messageCollected)
  {
    printFrameJ1850();
    messageReset();
    messageCollected = false;
  }
#endif
}

void workerLoop()
{
  while (!stopAppExecuting)
  {
    adcHandler();
    memsHandler();
    j1850Handler();
    blinkerHandler();
  }
}