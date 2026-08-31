// ---------------------------------------------------------------------------
// Blinker unit tests
//
// Build and run all host tests from the repository root:
//   make -C tests test
// ---------------------------------------------------------------------------

// test_env.h must come first — it provides all stubs and extern declarations.
#include "test_env.h"

#include <cstdio>
#include <cstring>

// ── Constants ─────────────────────────────────────────────────────────────────
// Must match switch_ctrl.cc
#define BLINKER_TIMER_PERIOD_MS  110
#define LONG_PRESS_COUNT         (LONG_PRESS_TIME / BLINKER_TIMER_PERIOD_MS)  // 9

// ── Test framework ────────────────────────────────────────────────────────────────
static int g_passed = 0;
static int g_failed = 0;

#define ASSERT_EQ(a, b)                                                        \
    do {                                                                       \
        if ((a) != (b)) {                                                      \
            printf("  FAIL %s:%d  expected %d  got %d\n",                     \
                   __FILE__, __LINE__, (int)(b), (int)(a));                    \
            ++g_failed;                                                        \
        }                                                                      \
    } while (0)

#define ASSERT_TRUE(x)   ASSERT_EQ(!!(x), 1)
#define ASSERT_FALSE(x)  ASSERT_EQ(!!(x), 0)

#define TEST(name)                                                             \
    static void name();                                                        \
    struct _Reg_##name { _Reg_##name() { tests[nTests++] = {#name, name}; } } \
    _reg_##name;                                                               \
    static void name()

struct TestEntry { const char* name; void(*fn)(); };
static TestEntry tests[64];
static int nTests = 0;

// ── Simulation helpers ─────────────────────────────────────────────────────────────

// Time of the last BLINKER_TIMER_PERIOD_MS tick that was injected.
static uint32_t lastTimerTick = 0;

/// Reset all state to power-on defaults.
/// Drains any stale internal FSM state left over from the previous test
/// by advancing fakeTick past MAX_PRESS_WAIT_TIME and injecting one tick
/// to trigger the timeout guard (resetEvent) inside blinkerTimerFSM.
static void sim_reset()
{
    // ── Drain stale FSM state from the previous test ────────────────────────
    // Ensure buttons appear DEPRESSED for the drain run so processButtonEvents
    // doesn't start a new timer cycle.
    fakeLeftPin  = GPIO_PIN_SET;   // DEPRESSED
    fakeRightPin = GPIO_PIN_SET;
    // Advance time well past MAX_PRESS_WAIT_TIME so the timeout guard fires.
    fakeTick += MAX_PRESS_WAIT_TIME + BLINKER_TIMER_PERIOD_MS * 2;
    HAL_TIM_PeriodElapsedCallback(&htim4);  // sets blinkerTick
    blinkerHandler();                        // timeout path → resetEvent()

    // ── Reset all externally visible state ──────────────────────────────────
    fakeTick      = 0;
    lastTimerTick = 0;
    fakeLeftPin   = GPIO_PIN_SET;
    fakeRightPin  = GPIO_PIN_SET;
    _fakeTIM1.CCR3 = 0;   // fakeLeftPWM
    _fakeTIM1.CCR4 = 0;   // fakeRightPWM
    leftEnabled   = false;
    rightEnabled  = false;
    hazardEnabled = false;
    overtakeMode  = false;
    postTurnTailActive = false;
    blinkCounter  = 0;
    currentSidemarkBrightness = 0;

    // Force blinkerDoBlink() to restart the ramp on the next call.
    leftSideOff();
    rightSideOff();
}

/// Advance simulated time by `ms` milliseconds at 1 ms resolution.
/// - Injects a BLINKER_TIMER_PERIOD_MS tick every 110 ms (via lastTimerTick).
/// - Calls blinkerHandler() every millisecond (main-loop work).
/// - Calls blinkerDoBlink() when any blinker channel is active (PWM timing).
/// Returns the number of timer ticks injected.
static int sim_advance(uint32_t ms)
{
    int ticks = 0;
    uint32_t end = fakeTick + ms;
    while (fakeTick < end)
    {
        ++fakeTick;
        if (fakeTick - lastTimerTick >= BLINKER_TIMER_PERIOD_MS)
        {
            lastTimerTick = fakeTick;
            HAL_TIM_PeriodElapsedCallback(&htim4);
            ++ticks;
        }
        blinkerHandler();
        if (leftEnabled || rightEnabled || hazardEnabled)
            blinkerDoBlink();
    }
    return ticks;
}

/// Press a button (active-low: GPIO_PIN_RESET = pressed).
static void sim_press_left()
{
    fakeLeftPin = GPIO_PIN_RESET;
    HAL_GPIO_EXTI_Callback(LT_BUTTON_Pin);
    blinkerHandler();
}
static void sim_press_right()
{
    fakeRightPin = GPIO_PIN_RESET;
    HAL_GPIO_EXTI_Callback(RT_BUTTON_Pin);
    blinkerHandler();
}

/// Release a button (back to GPIO_PIN_SET = depressed; no EXTI fired).
static void sim_release_left()  { fakeLeftPin  = GPIO_PIN_SET; }
static void sim_release_right() { fakeRightPin = GPIO_PIN_SET; }

// ── Tests ─────────────────────────────────────────────────────────────────────────

// 1. Single right press (held through first tick) → rightEnabled
TEST(right_turn_on_held)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // tick: RIGHT_BUTTON held → toggle
    ASSERT_TRUE(rightEnabled);
    ASSERT_FALSE(leftEnabled);
}

// 2. Single left press held through the full waitLongPress window → NOT overtake
TEST(left_turn_on_long_press)
{
    sim_reset();
    sim_press_left();
    sim_advance(BLINKER_TIMER_PERIOD_MS);                          // tick 1: toggle, enter waitLongPress
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // wait full window, button still held
    ASSERT_TRUE(leftEnabled);
    ASSERT_FALSE(overtakeMode);
}

// 3. Right press, released before waitLongPress window expires → overtakeMode
TEST(right_turn_short_press_overtake)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);                          // tick 1: toggle, enter waitLongPress
    sim_release_right();                                           // released early
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // window expires, button up
    ASSERT_TRUE(rightEnabled);
    ASSERT_TRUE(overtakeMode);
}

// 4. Overtake blinks OVERTAKE_BLINK_COUNT times then self-cancels (tsm logic)
TEST(overtake_auto_cancel)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(overtakeMode);

    // Each blink cycle = ramp (~250 ms) + hold (200 ms) + pause (250 ms) = ~700 ms.
    sim_advance((OVERTAKE_BLINK_COUNT + 2) * 1200);
    ASSERT_TRUE(blinkCounter >= OVERTAKE_BLINK_COUNT);

    blinkerAutoCancelHandler();
    ASSERT_FALSE(overtakeMode);
    ASSERT_FALSE(rightEnabled);
}

// 5. Double-press same side (≥2 ticks apart) → reversal cancels the blinker
TEST(double_press_reversal_cancels)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // tick 1: toggle ON, enter waitLongPress (lPC=0)
    ASSERT_TRUE(rightEnabled);
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // waitLongPress tick: lPC → 1
    // Second press: lPC=1 ≥ 1 → reversal accepted
    sim_press_right();
    ASSERT_FALSE(rightEnabled);
}

// 6. Bounce rejection: second EXTI at lPC=0 (within first waitLongPress tick) is ignored
TEST(bounce_rejected_at_lpc_zero)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // tick 1: toggle ON, waitLongPress=true, lPC=0
    ASSERT_TRUE(rightEnabled);
    // Bounce: another EXTI fires while button is still physically held (lPC=0 → rejected)
    sim_press_right();
    ASSERT_TRUE(rightEnabled);              // reversal must NOT happen
}

// 7. Switch sides: right ON, then left ON → right turns off, left turns on
TEST(switch_sides)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // right = overtake
    ASSERT_TRUE(rightEnabled);

    sim_press_left();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // left toggle fires → right off, left on
    ASSERT_TRUE(leftEnabled);
    ASSERT_FALSE(rightEnabled);
}

// 8. Hazard ON: the second physical press resolves the chord immediately.
TEST(hazard_both_held)
{
    sim_reset();
    sim_press_left();
    sim_press_right();
    ASSERT_TRUE(hazardEnabled);
}

// 9. Hazard ON/OFF: both buttons quick-tap (released before the tick)
TEST(hazard_quick_tap)
{
    sim_reset();
    sim_press_left();
    sim_press_right();
    sim_release_left();
    sim_release_right();
    // Both events set but both pins released → quick-tap branch → hazardToggle()
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(hazardEnabled);

    // Second quick-tap turns it off
    sim_press_left();
    sim_press_right();
    sim_release_left();
    sim_release_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    ASSERT_FALSE(hazardEnabled);
}

// 10. Turn OFF via held press does NOT set overtakeMode
TEST(turn_off_no_overtake)
{
    sim_reset();
    // Turn on with LONG HELD press → normal signal (NOT overtake)
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);                           // toggle ON, enter waitLongPress
    // Keep button held through the entire window:
    // at evaluation the button is still PRESSED → long press = normal signal, no overtake
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // 10 waitLongPress ticks
    sim_release_right();                                            // release after evaluation
    ASSERT_TRUE(rightEnabled);
    ASSERT_FALSE(overtakeMode);                                     // sanity: long press ≠ overtake

    // Turn off: press right again and hold through one tick
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // toggle OFF → rightEnabled=false, resetEvent
    sim_release_right();
    ASSERT_FALSE(rightEnabled);

    // Advance past any possible waitLongPress window — overtakeMode must stay false
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS);
    ASSERT_FALSE(overtakeMode);
}

// 11. Blinker reset: mid-cycle off then back on starts a fresh PWM ramp from 0
TEST(blink_cycle_resets_on_retrigger)
{
    sim_reset();
    // Turn on (overtake)
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(rightEnabled);

    // Advance until the ramp reaches full-on, whatever the cycle alignment
    // after the long-press window (a fixed offset here breaks whenever the
    // blink cadence constants are retuned).
    uint32_t pwmMid = 0;
    for (int i = 0; i < 200 && pwmMid < 96; ++i)
    {
        sim_advance(PWM_DUTY_DELAY);
        pwmMid = fakeRightPWM;
    }
    ASSERT_TRUE(pwmMid >= 96);             // sanity: reached full brightness

    // Turn off mid-cycle
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    ASSERT_FALSE(rightEnabled);

    // Turn back on — must start a fresh ramp from period=0.  Within the
    // 110 ms toggle advance at most ~11 ramp steps run, so PWM must still
    // be well below full-on if the FSM restarted cleanly.
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    ASSERT_TRUE(rightEnabled);
    ASSERT_TRUE(fakeRightPWM < pwmMid);
}

// 12. Quick tap (<110 ms, released before the first tick) also arms overtake —
//     classification must not depend on timer phase (#67)
TEST(quick_tap_arms_overtake)
{
    sim_reset();
    sim_press_right();
    sim_release_right();                    // released before the first tick
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // tick: short-press branch → toggle + window
    ASSERT_TRUE(rightEnabled);
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // window expires, button up
    ASSERT_TRUE(overtakeMode);
}

// 13. Opposite-side press during the long-press window switches direction
//     instead of being swallowed (#67)
TEST(opposite_press_in_window_switches)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // toggle ON, enter waitLongPress (right)
    sim_release_right();
    ASSERT_TRUE(rightEnabled);
    sim_advance(BLINKER_TIMER_PERIOD_MS);   // one window tick
    sim_press_left();                       // opposite side during the window
    ASSERT_TRUE(leftEnabled);               // switched immediately
    ASSERT_FALSE(rightEnabled);
    sim_release_left();
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS); // left window expires
    ASSERT_TRUE(leftEnabled);
    ASSERT_TRUE(overtakeMode);              // released early → overtake for the left side
}

// 14. Both-button hazard chord wins even while a turn is being classified.
TEST(hazard_chord_during_classification)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS); // right ON, waitLongPress active
    ASSERT_TRUE(rightEnabled);

    // Keep right held and press left: both pins are now physically pressed.
    sim_press_left();
    ASSERT_TRUE(hazardEnabled);
    ASSERT_FALSE(leftEnabled);
    ASSERT_FALSE(rightEnabled);
    ASSERT_FALSE(overtakeMode);
}

// 15. Hazard starts with a clean counter and cannot be timed out.
TEST(hazard_does_not_inherit_overtake_countdown)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS);
    sim_release_right();
    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(overtakeMode);

    blinkCounter = OVERTAKE_BLINK_COUNT - 1;
    sim_press_left();
    sim_press_right();
    ASSERT_TRUE(hazardEnabled);
    ASSERT_FALSE(overtakeMode);
    ASSERT_EQ(blinkCounter, 0);

    blinkCounter = OVERTAKE_BLINK_COUNT + 10;
    blinkerAutoCancelHandler();
    ASSERT_TRUE(hazardEnabled);
}

// 16. Cancellation occurs at the configured completed-flash boundary.
TEST(timed_cancel_exact_boundary)
{
    sim_reset();
    rightEnabled = true;
    overtakeMode = true;
    blinkCounter = OVERTAKE_BLINK_COUNT - 1;
    blinkerAutoCancelHandler();
    ASSERT_TRUE(rightEnabled);

    blinkCounter = OVERTAKE_BLINK_COUNT;
    blinkerAutoCancelHandler();
    ASSERT_FALSE(rightEnabled);
    ASSERT_FALSE(overtakeMode);
}

// 17. A tick from the stopped timer cannot advance a newly armed press.
TEST(stale_timer_tick_is_discarded)
{
    sim_reset();
    fakeRightPin = GPIO_PIN_RESET;
    HAL_GPIO_EXTI_Callback(RT_BUTTON_Pin);      // raw press waiting for main loop
    HAL_TIM_PeriodElapsedCallback(&htim4);     // stale tick from old generation
    blinkerHandler();                          // starts timer and clears stale tick
    ASSERT_FALSE(rightEnabled);

    sim_advance(BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(rightEnabled);
}

// 18. Post-turn tail counts only flashes completed after it is armed.
TEST(post_turn_tail_uses_fresh_count)
{
    sim_reset();
    leftEnabled = true;
    blinkCounter = 42;
    startPostTurnTail();
    ASSERT_TRUE(postTurnTailActive);
    ASSERT_FALSE(overtakeMode);
    ASSERT_EQ(blinkCounter, 0);

    blinkCounter = POST_TURN_BLINK_COUNT - 1;
    blinkerAutoCancelHandler();
    ASSERT_TRUE(leftEnabled);
    blinkCounter = POST_TURN_BLINK_COUNT;
    blinkerAutoCancelHandler();
    ASSERT_FALSE(leftEnabled);
    ASSERT_FALSE(postTurnTailActive);
}

// 19. Late short/long classification cannot overwrite a detected turn tail.
TEST(post_turn_tail_wins_over_pending_classification)
{
    sim_reset();
    sim_press_right();
    sim_advance(BLINKER_TIMER_PERIOD_MS); // right ON, classification pending
    sim_release_right();
    startPostTurnTail();                  // IMU detects turn before deadline
    ASSERT_TRUE(postTurnTailActive);

    sim_advance((LONG_PRESS_COUNT + 1) * BLINKER_TIMER_PERIOD_MS);
    ASSERT_TRUE(postTurnTailActive);
    ASSERT_FALSE(overtakeMode);
    ASSERT_TRUE(rightEnabled);
}

// ── Main ─────────────────────────────────────────────────────────────────────────
int main()
{
    printf("Running %d blinker tests...\n\n", nTests);
    for (int i = 0; i < nTests; ++i)
    {
        int failBefore = g_failed;
        tests[i].fn();
        bool passed = (g_failed == failBefore);
        printf("  [%s] %s\n", passed ? "PASS" : "FAIL", tests[i].name);
        if (passed) ++g_passed;
    }
    printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
