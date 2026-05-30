#include "tsm.h"
#include "settings.h"
#include "engine_state.h"
#include "j1850.h"

namespace Engine
{

static State     gState          = State::Unknown;
static bool      gStarterLocked  = false;
static uint32_t  gOffSince       = 0;   // tick when RPM+KPH first hit zero
static uint32_t  gLastFrameTick  = 0;   // tick of last live J1850 frame

// ── public API ──────────────────────────────────────────────────────────────

State getState()        { return gState; }
bool  isStarterLocked() { return gStarterLocked; }

// ── FSM ─────────────────────────────────────────────────────────────────────

void handler()
{
  const uint32_t now = HAL_GetTick();

  // Track whether the bus is producing frames.
  static uint32_t prevFrameCounter = 0;
  if (frameCounter != prevFrameCounter)
  {
    prevFrameCounter = frameCounter;
    gLastFrameTick   = now;
  }

  // If no frame has been seen yet, or the bus has been silent too long,
  // stay Unknown so the caller falls back to the voltage FSM.
  if (gLastFrameTick == 0 ||
      (now - gLastFrameTick) > J1850_BUS_TIMEOUT_MS)
  {
    if (gState != State::Unknown)
    {
      DEBUG_LOG("Engine: J1850 bus silent -> state Unknown (voltage FSM takes over)\r\n");
      gState = State::Unknown;
    }
    return;
  }

  const bool engineOn  = (rpms >= ENGINE_RUNNING_RPM_MIN);
  const bool moving    = engineOn && (kph >= ENGINE_RUNNING_KPH_MIN);
  const bool stopped   = (rpms == 0) && (kph == 0);

  switch (gState)
  {
  case State::Unknown:
    // First live frame — transition to Off as baseline.
    DEBUG_LOG("Engine: J1850 bus active -> state Off\r\n");
    gState   = State::Off;
    gOffSince = now;
    break;

  case State::Off:
    if (engineOn)
    {
      gState = State::Running;
      DEBUG_LOG("Engine: state Running (RPM=%u)\r\n", (unsigned)rpms);
    }
    break;

  case State::Running:
    if (moving)
    {
      gState         = State::Moving;
      gStarterLocked = true;
      disableStarter();
      DEBUG_LOG("Engine: state Moving -> starter LOCKED (RPM=%u KPH=%u)\r\n",
                (unsigned)rpms, (unsigned)kph);
    }
    else if (stopped)
    {
      // Revving at standstill then cutting throttle — treat as Off candidate.
      gOffSince = now;
      gState    = State::Off;
      DEBUG_LOG("Engine: state Off (RPM=0 KPH=0)\r\n");
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
        gState         = State::Off;
        gStarterLocked = false;
        enableStarter();
        gOffSince = 0;
        DEBUG_LOG("Engine: state Off -> starter UNLOCKED\r\n");
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
