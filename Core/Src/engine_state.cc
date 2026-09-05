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

State getState()          { return gState; }
bool  isStarterLocked()   { return gStarterLocked; }
bool  telemetryEverSeen() { return rpmSignalSeen; }

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

  // RPM is the authoritative engine-running signal: the ECM broadcasts it
  // continuously (~15/s), including RPM=0 while the ignition is on and the
  // engine is off. Speed is broadcast only sporadically (a handful of frames
  // per ride), so it must NOT gate telemetry freshness - requiring it left the
  // state stuck at Unknown and handed the starter lock to the voltage
  // heuristic, which false-locks on a charged battery (>13.4 V for 15 s).
  // Speed is used only to distinguish Moving from Running. Raw frameCounter
  // activity is still ignored: unrelated traffic and malformed SOFs say
  // nothing about whether RPM is current.
  const bool rpmFresh =
      rpmSignalSeen && (now - rpmLastUpdateTick) <= J1850_SIGNAL_TIMEOUT_MS;
  const bool speedFresh =
      speedSignalSeen && (now - speedLastUpdateTick) <= J1850_SIGNAL_TIMEOUT_MS;
  if (!rpmFresh)
  {
    if (gState != State::Unknown)
    {
      DEBUG_LOG("Engine: RPM telemetry stale -> state Unknown (voltage FSM takes over)\r\n");
      gState = State::Unknown;
    }
    gOffSince = 0;
    return;
  }

  const bool engineOn  = (rpms >= ENGINE_RUNNING_RPM_MIN);
  // Only a fresh speed frame may upgrade Running -> Moving; a stale kph value
  // must never fabricate motion.
  const bool moving    = engineOn && speedFresh && (kph >= ENGINE_RUNNING_KPH_MIN);
  // RPM=0 alone is the authoritative "engine stopped" signal - speed frames
  // are too sparse to require kph==0 here.
  const bool stopped   = (rpms == 0);

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
