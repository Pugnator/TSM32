#pragma once
#include <stdint.h>

namespace Engine
{
  // Engine/motion state derived from live J1850 telemetry.
  // Unknown = no valid J1850 frames yet (or bus timed out) → voltage FSM fallback.
  // Off      = RPM==0 and KPH==0 confirmed for ENGINE_OFF_DEBOUNCE_MS.
  // Running  = RPM >= ENGINE_RUNNING_RPM_MIN; starter latch engaged.
  // Moving   = RPM >= ENGINE_RUNNING_RPM_MIN AND KPH >= ENGINE_RUNNING_KPH_MIN
  //            (motion state; starter was already latched on Running).
  enum class State : uint8_t
  {
    Unknown = 0,
    Off,
    Running,
    Moving,
  };

  // Returns the current engine/motion state.
  State getState();

  // True once the Running threshold has been reached. With
  // STARTER_UNLOCK_DISABLE enabled it remains latched until ignition reset;
  // otherwise it clears after Off is confirmed.
  bool isStarterLocked();

  // Call once per main-loop iteration when J1850 is enabled.
  void handler();
} // namespace Engine
