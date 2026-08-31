#include "tsm.h"
#include "settings.h"
#include "engine_state.h"
#include "j1850.h"

namespace Engine
{

static State     gState          = State::Unknown;
static bool      gStarterLocked  = false;
static uint32_t  gOffSince       = 0;   // tick when RPM+KPH first hit zero

// ── public API ──────────────────────────────────────────────────────────────

State getState()        { return gState; }
bool  isStarterLocked() { return gStarterLocked; }

static void lockStarterForIgnitionCycle()
{
  if (gStarterLocked)
  {
    return;
  }

  gStarterLocked = true;
  disableStarter();
  DEBUG_LOG("Engine: starter LOCKED (RPM=%u)\r\n", (unsigned)rpms);
}

// ── FSM ─────────────────────────────────────────────────────────────────────

void handler()
{
  const uint32_t now = HAL_GetTick();

  // Both inputs are required and expire independently.  Raw frameCounter
  // activity is deliberately ignored: unrelated traffic and malformed SOFs
  // say nothing about whether RPM/KPH are still current.
  const bool telemetryFresh =
      rpmSignalSeen && speedSignalSeen &&
      (now - rpmLastUpdateTick) <= J1850_SIGNAL_TIMEOUT_MS &&
      (now - speedLastUpdateTick) <= J1850_SIGNAL_TIMEOUT_MS;
  if (!telemetryFresh)
  {
    if (gState != State::Unknown)
    {
      DEBUG_LOG("Engine: RPM/KPH telemetry stale -> state Unknown (voltage FSM takes over)\r\n");
      gState = State::Unknown;
    }
    gOffSince = 0;
    return;
  }

  const bool engineOn  = (rpms >= ENGINE_RUNNING_RPM_MIN);
  const bool moving    = engineOn && (kph >= ENGINE_RUNNING_KPH_MIN);
  const bool stopped   = (rpms == 0) && (kph == 0);

  switch (gState)
  {
  case State::Unknown:
    if (engineOn)
    {
      gState = State::Running;
      gOffSince = 0;
      lockStarterForIgnitionCycle();
      DEBUG_LOG("Engine: fresh telemetry -> state Running (RPM=%u)\r\n",
                (unsigned)rpms);
    }
    else
    {
      gState = State::Off;
      gOffSince = stopped ? now : 0;
      DEBUG_LOG("Engine: fresh telemetry -> state Off\r\n");
    }
    break;

  case State::Off:
    if (engineOn)
    {
      gState = State::Running;
      gOffSince = 0;
      lockStarterForIgnitionCycle();
      DEBUG_LOG("Engine: state Running (RPM=%u)\r\n", (unsigned)rpms);
    }
    break;

  case State::Running:
    if (moving)
    {
      gState         = State::Moving;
      gOffSince      = 0;   // clear stale Off timestamp so the stop debounce
                            // in State::Moving starts fresh (Fixes #59)
      DEBUG_LOG("Engine: state Moving (RPM=%u KPH=%u)\r\n",
                (unsigned)rpms, (unsigned)kph);
    }
    else if (stopped)
    {
      /* A single zero frame at idle must not disable DRL or change starter
       * policy. Require the same continuous stop debounce used after Moving. */
      if (gOffSince == 0)
      {
        gOffSince = now;
      }
      if ((now - gOffSince) >= ENGINE_OFF_DEBOUNCE_MS)
      {
        gOffSince = 0;
        gState = State::Off;
        DEBUG_LOG("Engine: state Off after stop debounce\r\n");
      }
    }
    else
    {
      gOffSince = 0;
    }
    break;

  case State::Moving:
    if (stopped)
    {
      // Don't clear immediately — debounce to avoid brief GPS/sensor glitches.
      if (gOffSince == 0)
        gOffSince = now;

      if ((now - gOffSince) >= ENGINE_OFF_DEBOUNCE_MS)
      {
        gState = State::Off;
#if STARTER_UNLOCK_DISABLE
        /* The configured policy requires an ignition cycle to unlock.  Keep
         * software state aligned with the latched relay state. */
        gStarterLocked = true;
        DEBUG_LOG("Engine: state Off -> starter remains LOCKED until ignition cycle\r\n");
#else
        gStarterLocked = false;
        enableStarter();
        DEBUG_LOG("Engine: state Off -> starter UNLOCKED\r\n");
#endif
        gOffSince = 0;
      }
    }
    else
    {
      gOffSince = 0; // still moving, reset debounce
    }
    break;
  }
}

} // namespace Engine
