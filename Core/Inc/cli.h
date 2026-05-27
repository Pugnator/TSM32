#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Poll the RTT down-channel for a line of input and dispatch it.
     Cheap: returns immediately when no byte is pending, so it is safe to
     call from the main loop on every iteration. */
  void cliPoll(void);

  /* Live-state accessors implemented by the main application (tsm.cc) so
     that cli.cc stays decoupled from the AHRS templates.  Defaults return
     zero / a clear state until the application registers a real source. */
  float cliGetChipTemperatureC(void);
  void cliGetYprDeg(int16_t *yaw, int16_t *pitch, int16_t *roll);

#ifdef __cplusplus
}
#endif
