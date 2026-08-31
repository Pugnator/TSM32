// Host tests for the security PIN / settings menu FSM (Core/Src/security.cc).
// Links the real security.cc and eeprom.cc (RAM flash simulation) plus
// turn_ctrl.cc for the lamp helpers; buttons/lamps/tick come from stubs.h.
#include "tsm.h"
#include "../Core/Inc/security.h"
#include "../Core/Inc/eeprom.h"

#include <cstdio>

static int failures = 0;
#define CHECK(cond)                                                        \
  do                                                                       \
  {                                                                        \
    if (!(cond))                                                           \
    {                                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
      failures++;                                                          \
    }                                                                      \
  } while (0)

// ── stubs security.cc needs that no linked module provides ─────────────────
extern "C"
{
  static bool starterEnabled = false;
  void enableStarter()
  {
    if (securityStarterPermitted())
      starterEnabled = true;
  }
  void disableStarter() { starterEnabled = false; }
  void discardButtonEvents() {}
  void adcHandler() {}
  uint32_t adcDMAbuffer[ADC_DMA_BUF_SIZE];
  volatile bool adcDMAcompleted = false;
  bool stopAppExecutingDummy; // silence unused

  // turn_ctrl externs normally defined elsewhere in firmware
  bool goOnDummy;
}
bool stopAppExecuting = false;
bool goOn = false;

// ── simulation helpers ──────────────────────────────────────────────────────
static uint32_t lampOnTime; // accumulated ms with any lamp lit (visibility)

static void sim_run(uint32_t ms)
{
  for (uint32_t i = 0; i < ms; ++i)
  {
    fakeTick++;
    securityHandler();
    if (fakeLeftPWM > 0 || fakeRightPWM > 0)
      lampOnTime++;
  }
}

static void sim_press_left()
{
  fakeLeftPin = GPIO_PIN_RESET; // PRESSED
  sim_run(SECURITY_DEBOUNCE_MS + 10);
  fakeLeftPin = GPIO_PIN_SET;
  sim_run(SECURITY_DEBOUNCE_MS + 10);
}

static void sim_press_right()
{
  fakeRightPin = GPIO_PIN_RESET;
  sim_run(SECURITY_DEBOUNCE_MS + 10);
  fakeRightPin = GPIO_PIN_SET;
  sim_run(SECURITY_DEBOUNCE_MS + 10);
}

static void sim_enter_digit(uint8_t digit)
{
  for (uint8_t i = 0; i < digit; ++i)
    sim_press_left();
  sim_press_right();
}

static void sim_enter_pin(uint32_t pin)
{
  uint8_t digits[SECURITY_PIN_LENGTH];
  for (int i = SECURITY_PIN_LENGTH - 1; i >= 0; --i)
  {
    digits[i] = (uint8_t)(pin % 10u);
    pin /= 10u;
  }
  for (int i = 0; i < SECURITY_PIN_LENGTH; ++i)
    sim_enter_digit(digits[i]);
  sim_run(2500); // let the success/error indication finish
}

static void sim_boot(bool settingsRequested, bool starterAllowed)
{
  fakeLeftPin = GPIO_PIN_SET;
  fakeRightPin = GPIO_PIN_SET;
  starterEnabled = false;
  lampOnTime = 0;
  CHECK(ee_init());
  securityInit(settingsRequested, starterAllowed);
  sim_run(1500); // welcome indication, settle
}

// ── tests ──────────────────────────────────────────────────────────────────

static void test_no_pin_boot_is_transparent()
{
  ee_test_reset();
  sim_boot(false, true);
  CHECK(!securityBusy());
  CHECK(starterEnabled); // no PIN + power-on reset -> starter on
  CHECK(securityStarterPermitted());
}

static void test_set_pin_via_menu_then_lock_and_unlock()
{
  ee_test_reset();
  sim_boot(true, true); // settings gesture at boot, no PIN yet
  CHECK(securityState() == SECURITY_STATE_MENU);
  CHECK(starterEnabled); // no PIN configured yet

  sim_enter_digit(1); // menu item 1 = set PIN
  sim_run(1200);
  CHECK(securityState() == SECURITY_STATE_SET_PIN);
  sim_enter_pin(2483);
  CHECK(securityLastIndication() == SECURITY_IND_SUCCESS);
  sim_run(3000);
  CHECK(!securityBusy());

  // "Reboot": PIN now gates the starter.
  sim_boot(false, true);
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  CHECK(!starterEnabled);
  CHECK(!securityStarterPermitted());

  sim_enter_pin(2483);
  CHECK(securityLastIndication() == SECURITY_IND_SUCCESS);
  CHECK(securityStarterPermitted());
  CHECK(starterEnabled);
  sim_run(3000);
  CHECK(!securityBusy());
}

static void test_wrong_pin_is_visually_indicated()
{
  // continues from previous state: PIN 2483 stored
  sim_boot(false, true);
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  lampOnTime = 0;
  sim_enter_pin(1111); // wrong
  CHECK(securityLastIndication() == SECURITY_IND_ERROR);
  CHECK(lampOnTime > 500); // the error flash actually lit the lamps
  CHECK(!starterEnabled);
  CHECK(securityState() == SECURITY_STATE_LOCKED); // retry allowed
}

static void test_lockout_after_max_attempts()
{
  sim_boot(false, true);
  for (int i = 0; i < SECURITY_MAX_ATTEMPTS; ++i)
    sim_enter_pin(9999);
  CHECK(securityState() == SECURITY_STATE_LOCKOUT);

  // Correct PIN is ignored during lockout (presses do nothing).
  sim_enter_pin(2483);
  CHECK(!starterEnabled);

  // After the lockout window the correct PIN unlocks again.
  sim_run(SECURITY_LOCKOUT_MS + 100);
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  sim_enter_pin(2483);
  CHECK(starterEnabled);
  sim_run(3000);
}

static void test_fail_lock_beats_pin()
{
  sim_boot(false, false); // non-power-on reset
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  sim_enter_pin(2483); // correct
  CHECK(securityLastIndication() == SECURITY_IND_SUCCESS);
  CHECK(!starterEnabled); // fail-lock holds even past a correct PIN
  sim_run(3000);
}

static void test_clear_pin()
{
  sim_boot(true, true); // settings gesture; PIN set -> unlock first
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  sim_enter_pin(2483);
  CHECK(securityState() == SECURITY_STATE_MENU); // settings after unlock
  sim_enter_digit(1);                            // set-PIN item
  sim_run(1200);
  sim_press_right(); // empty first digit = clear the PIN
  sim_run(3000);
  CHECK(securityLastIndication() == SECURITY_IND_SUCCESS);
  CHECK(!securityBusy());

  sim_boot(false, true);
  CHECK(!securityBusy()); // PIN gone -> transparent boot
  CHECK(starterEnabled);
}

static void test_hazard_chord_while_locked()
{
  ee_test_reset();
  sim_boot(true, true);
  sim_enter_digit(1);
  sim_run(1200);
  sim_enter_pin(5678); // set a PIN again
  sim_run(3000);

  sim_boot(false, true);
  CHECK(securityState() == SECURITY_STATE_LOCKED);
  hazardEnabled = false;
  fakeLeftPin = GPIO_PIN_RESET;
  fakeRightPin = GPIO_PIN_RESET;
  sim_run(SECURITY_CHORD_HAZARD_MS + SECURITY_DEBOUNCE_MS + 100);
  CHECK(hazardEnabled); // roadside hazard works while locked
  fakeLeftPin = GPIO_PIN_SET;
  fakeRightPin = GPIO_PIN_SET;
  sim_run(200);
  CHECK(!starterEnabled); // and it did not unlock anything
}

int main()
{
  test_no_pin_boot_is_transparent();
  test_set_pin_via_menu_then_lock_and_unlock();
  test_wrong_pin_is_visually_indicated();
  test_lockout_after_max_attempts();
  test_fail_lock_beats_pin();
  test_clear_pin();
  test_hazard_chord_while_locked();

  if (failures == 0)
  {
    std::printf("test_security: all tests passed\n");
    return 0;
  }
  std::printf("test_security: %d FAILURE(S)\n", failures);
  return 1;
}
