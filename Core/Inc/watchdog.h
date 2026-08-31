/**
 * @file watchdog.h
 * @brief Independent watchdog (IWDG) only.
 *
 * A single LSI-clocked deadman that resets the MCU if the firmware stops
 * refreshing it — a hang anywhere (a stuck fault handler, a wedged ISR, a
 * peripheral deadlock, an infinite loop in init or the main loop). It exists
 * to recover the emulated TSM after a lock-up so the instrument cluster does
 * not latch the security lamp (issue: intermittent SIL-on cleared by a reboot).
 *
 * Deliberately NOT the window watchdog: the WWDG's 65 ms hardware-maximum
 * timeout is far shorter than a legitimate main-loop iteration (bit-banged
 * J1850 TX) and previously caused a boot loop. The IWDG has no window, so one
 * refresh per loop iteration is enough — no legitimate iteration comes close
 * to the ~2 s timeout.
 *
 * Register-level (CMSIS), so it needs no HAL IWDG module and no CubeMX-
 * generated init call.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    WATCHDOG_RESET_CAUSE_UNKNOWN = 0,
    WATCHDOG_RESET_CAUSE_POWER_ON,
    WATCHDOG_RESET_CAUSE_IWDG,
    WATCHDOG_RESET_CAUSE_SOFTWARE,
    WATCHDOG_RESET_CAUSE_PIN,
  } watchdog_reset_cause_t;

  /* Only a real power cycle proves that the ignition-cycle starter latch may
   * be released. Recovery and unknown resets start fail-locked. */
  static inline bool watchdog_reset_allows_starter(watchdog_reset_cause_t cause)
  {
    return cause == WATCHDOG_RESET_CAUSE_POWER_ON;
  }

  /**
   * Configure and start the IWDG (~2 s nominal, 1.3..2.7 s across the LSI
   * spread). Irreversible. Call once, early — before the slow init steps — so
   * a hang during init is caught too; every init phase and the main loop must
   * then refresh within the timeout.
   */
  void watchdog_init(void);

  /** Reload the IWDG counter. Call once per main-loop iteration (and after
   *  each slow init phase). Legal at any time — the IWDG has no window. */
  void watchdog_refresh(void);

  /** Classify and log the previous reset cause, then clear the RCC flags.
   *  Only an unambiguous power-on reset permits the starter relay; recovery
   *  and unknown resets remain fail-locked. */
  watchdog_reset_cause_t watchdog_report_reset_cause(void);

#ifdef __cplusplus
}
#endif
