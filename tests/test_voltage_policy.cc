#include "test_env.h"
#include "../Core/Inc/engine_state.h"
#include "../Core/Inc/voltage_policy.h"

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

void sampleAdc(uint32_t value)
{
    for (uint32_t &sample : adcDMAbuffer)
        sample = value;
    fakeTick += 500;
    adcDMAcompleted = true;
    adcHandler();
}

void testShortFilterLatency()
{
    VoltagePolicy::Controller controller;
    controller.update(ADC_11_1V_VALUE, 0);

    VoltagePolicy::Result result{};
    uint32_t detectedAt = 0;
    for (uint32_t now = 500; now <= 5000; now += 500)
    {
        result = controller.update(ADC_14_3V_VALUE, now);
        if (result.region == VoltagePolicy::Region::High)
        {
            detectedAt = now;
            break;
        }
    }
    CHECK(detectedAt != 0);
    CHECK(detectedAt <= 4000);
}

void testQualificationMustBeContinuous()
{
    VoltagePolicy::Controller controller;
    auto result = controller.update(ADC_14_3V_VALUE, 0);
    result = controller.update(ADC_14_3V_VALUE,
                               VOLTAGE_DETECTION_THRESHOLD - 1);
    CHECK(!result.highQualified);

    uint32_t now = VOLTAGE_DETECTION_THRESHOLD;
    for (uint32_t i = 0; i < VOLTAGE_FILTER_WINDOW_SIZE; ++i)
    {
        now += 500;
        result = controller.update((ADC_11_1V_VALUE + ADC_13_4V_VALUE) / 2,
                                   now);
    }
    CHECK(result.region == VoltagePolicy::Region::Middle);
    CHECK(!result.highQualified);

    uint32_t secondHighSince = 0;
    for (uint32_t i = 0; i < VOLTAGE_FILTER_WINDOW_SIZE; ++i)
    {
        now += 500;
        result = controller.update(ADC_14_3V_VALUE, now);
        if (result.region == VoltagePolicy::Region::High && secondHighSince == 0)
            secondHighSince = now;
    }
    CHECK(secondHighSince != 0);
    result = controller.update(ADC_14_3V_VALUE,
                               secondHighSince + VOLTAGE_DETECTION_THRESHOLD - 1);
    CHECK(!result.highQualified);
    result = controller.update(ADC_14_3V_VALUE,
                               secondHighSince + VOLTAGE_DETECTION_THRESHOLD);
    CHECK(result.highQualified);
}

void testIntermittentUndervoltageDoesNotAccumulate()
{
    VoltagePolicy::Controller controller;
    auto result = controller.update(ADC_10V_VALUE, 0);
    result = controller.update(ADC_10V_VALUE,
                               LOW_VOLTAGE_DETECTION_THRESHOLD - 1);
    CHECK(!result.lowQualified);

    uint32_t now = LOW_VOLTAGE_DETECTION_THRESHOLD;
    for (uint32_t i = 0; i < VOLTAGE_FILTER_WINDOW_SIZE; ++i)
    {
        now += 500;
        result = controller.update((ADC_11_1V_VALUE + ADC_13_4V_VALUE) / 2,
                                   now);
    }
    CHECK(result.region == VoltagePolicy::Region::Middle);

    uint32_t secondLowSince = 0;
    for (uint32_t i = 0; i < VOLTAGE_FILTER_WINDOW_SIZE; ++i)
    {
        now += 500;
        result = controller.update(ADC_10V_VALUE, now);
        if (result.region == VoltagePolicy::Region::Low && secondLowSince == 0)
            secondLowSince = now;
    }
    CHECK(secondLowSince != 0);
    result = controller.update(ADC_10V_VALUE,
                               secondLowSince + LOW_VOLTAGE_DETECTION_THRESHOLD - 1);
    CHECK(!result.lowQualified);
    result = controller.update(ADC_10V_VALUE,
                               secondLowSince + LOW_VOLTAGE_DETECTION_THRESHOLD);
    CHECK(result.lowQualified);
}

void testDrlAndStarterIntegration()
{
    fakeTick = 100;
    currentSidemarkBrightness = 0;
    leftEnabled = false;
    rightEnabled = false;
    hazardEnabled = false;
    fakeLeftPWM = 0;
    fakeRightPWM = 0;
    adcHandler(); // initialize the production handler's sample timestamp

    // Fresh running telemetry is authoritative for DRL, even at low voltage.
    publishTelemetry(ENGINE_RUNNING_RPM_MIN, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);
    CHECK(disableCalls == 1); // 700 RPM locks without waiting for road speed.
    sampleAdc(ADC_10V_VALUE);
    CHECK(currentSidemarkBrightness == DLR_BRIGHTNESS_VALUE);
    CHECK(fakeLeftPWM == DLR_BRIGHTNESS_VALUE);
    CHECK(fakeRightPWM == DLR_BRIGHTNESS_VALUE);

    // Hazard owns the physical PWM, but monitoring and desired DRL state run.
    hazardEnabled = true;
    fakeLeftPWM = 77;
    fakeRightPWM = 88;
    sampleAdc(ADC_10V_VALUE);
    CHECK(currentSidemarkBrightness == DLR_BRIGHTNESS_VALUE);
    CHECK(fakeLeftPWM == 77);
    CHECK(fakeRightPWM == 88);
    hazardEnabled = false;

    // A fresh confirmed engine-off state disables DRL.
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Running);
    fakeTick += ENGINE_OFF_DEBOUNCE_MS;
    publishTelemetry(0, 0);
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Off);
    sampleAdc(ADC_10V_VALUE);
    CHECK(currentSidemarkBrightness == 0);

    // With telemetry stale, sustained charging voltage is the DRL fallback.
    // The STARTER, however, stays RPM-driven: this bus has carried telemetry,
    // so a later J1850 fault must not let a charged battery fabricate a lock.
    fakeTick += J1850_SIGNAL_TIMEOUT_MS + 1;
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Unknown);
    CHECK(Engine::telemetryEverSeen());
    for (int i = 0; i < 50; ++i)
        sampleAdc(ADC_14_3V_VALUE);
    CHECK(currentSidemarkBrightness == DLR_BRIGHTNESS_VALUE);
    CHECK(disableCalls == 1); // Still only the 700 RPM lock; voltage added none.

    // More than five minutes of low voltage must not disable DRL while fresh
    // RPM proves the engine is still running (red-light/idle droop case).
    const int lowSamples =
        static_cast<int>(LOW_VOLTAGE_DETECTION_THRESHOLD / 500) +
        static_cast<int>(VOLTAGE_FILTER_WINDOW_SIZE) + 2;
    for (int i = 0; i < lowSamples; ++i)
    {
        publishTelemetry(ENGINE_RUNNING_RPM_MIN, 0);
        Engine::handler();
        sampleAdc(ADC_10V_VALUE);
    }
    CHECK(currentSidemarkBrightness == DLR_BRIGHTNESS_VALUE);
    CHECK(enableCalls == 0);

    // Once telemetry expires, the already sustained undervoltage fallback
    // applies immediately to DRL - but not to the starter, for the same
    // reason as above.
    fakeTick += J1850_SIGNAL_TIMEOUT_MS + 1;
    Engine::handler();
    sampleAdc(ADC_10V_VALUE);
    CHECK(currentSidemarkBrightness == 0);
    CHECK(enableCalls == 0);
}

// A bus that has NEVER carried telemetry is the one case where voltage still
// owns the starter relay: no J1850 install, or a bus dead from power-on.
// Runs after testDrlAndStarterIntegration and continues from its state (DRL
// fallback region Low, engine state Unknown), so it only has to clear the
// sticky telemetry flag and the call counters.
void testDeadBusVoltageStillDrivesStarter()
{
    rpmSignalSeen = false;
    speedSignalSeen = false;
    disableCalls = 0;
    enableCalls = 0;

    fakeTick += J1850_SIGNAL_TIMEOUT_MS + 1;
    Engine::handler();
    CHECK(Engine::getState() == Engine::State::Unknown);
    CHECK(!Engine::telemetryEverSeen());

    // Sustained charging voltage latches the relay when nothing else can.
    for (int i = 0; i < 50; ++i)
        sampleAdc(ADC_14_3V_VALUE);
    CHECK(disableCalls == 1);

    // And sustained undervoltage releases it again.
    const int lowSamples =
        static_cast<int>(LOW_VOLTAGE_DETECTION_THRESHOLD / 500) +
        static_cast<int>(VOLTAGE_FILTER_WINDOW_SIZE) + 2;
    for (int i = 0; i < lowSamples; ++i)
        sampleAdc(ADC_10V_VALUE);
    CHECK(enableCalls == 1);
}
} // namespace

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
    testShortFilterLatency();
    testQualificationMustBeContinuous();
    testIntermittentUndervoltageDoesNotAccumulate();
    testDrlAndStarterIntegration();
    testDeadBusVoltageStillDrivesStarter();

    std::printf("Voltage/DRL tests: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
