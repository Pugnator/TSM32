#include "test_env.h"
#include "../Core/Inc/engine_state.h"
#include "../Core/Inc/watchdog.h"

#include <cstdio>

namespace
{
int failures = 0;
int disableCalls = 0;
int enableCalls = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

void publishTelemetry(uint16_t rpm, uint16_t speed)
{
    rpms = rpm;
    kph = speed;
    rpmLastUpdateTick = fakeTick;
    speedLastUpdateTick = fakeTick;
    rpmSignalSeen = true;
    speedSignalSeen = true;
}
}

extern "C" void disableStarter()
{
    ++disableCalls;
}

extern "C" void enableStarter()
{
    ++enableCalls;
}

int main()
{
    CHECK(watchdog_reset_allows_starter(WATCHDOG_RESET_CAUSE_POWER_ON));
    CHECK(!watchdog_reset_allows_starter(WATCHDOG_RESET_CAUSE_IWDG));
    CHECK(!watchdog_reset_allows_starter(WATCHDOG_RESET_CAUSE_SOFTWARE));
    CHECK(!watchdog_reset_allows_starter(WATCHDOG_RESET_CAUSE_PIN));
    CHECK(!watchdog_reset_allows_starter(WATCHDOG_RESET_CAUSE_UNKNOWN));

    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Unknown);

    fakeTick = 100;
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Off);

    // RPM below the configured threshold must not arm the lock, even when
    // speed is already high enough.
    ++fakeTick;
    publishTelemetry(ENGINE_RUNNING_RPM_MIN - 1, ENGINE_RUNNING_KPH_MIN);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Off);
    CHECK(disableCalls == 0);

    // The exact 700 RPM boundary enters Running and locks immediately. Speed
    // is only used to distinguish Running from Moving.
    ++fakeTick;
    publishTelemetry(ENGINE_RUNNING_RPM_MIN, ENGINE_RUNNING_KPH_MIN - 1);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);
    CHECK(Engine::isStarterLocked());
    CHECK(disableCalls == 1);

    // One or several isolated zero frames at stationary idle must not turn
    // DRL off or declare the engine stopped before the continuous debounce.
    ++fakeTick;
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);
    fakeTick += ENGINE_OFF_DEBOUNCE_MS - 1;
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);

    ++fakeTick;
    publishTelemetry(ENGINE_RUNNING_RPM_MIN, ENGINE_RUNNING_KPH_MIN - 1);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);

    ++fakeTick;
    publishTelemetry(ENGINE_RUNNING_RPM_MIN, ENGINE_RUNNING_KPH_MIN);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Moving);
    CHECK(Engine::isStarterLocked());
    CHECK(disableCalls == 1);

    ++fakeTick;
    publishTelemetry(0, 0);
    Engine::handler();
    fakeTick += ENGINE_OFF_DEBOUNCE_MS;
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Off);
#if STARTER_UNLOCK_DISABLE
    CHECK(Engine::isStarterLocked());
    CHECK(enableCalls == 0);
#else
    CHECK(!Engine::isStarterLocked());
    CHECK(enableCalls == 1);
#endif

    // Unrelated bus activity is not telemetry. Expiring either required
    // signal must hand control back to the voltage FSM.
    fakeTick += J1850_SIGNAL_TIMEOUT_MS + 1;
    rpmLastUpdateTick = fakeTick; // RPM is fresh, speed is deliberately stale.
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Unknown);

    std::printf("Engine-state tests: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
