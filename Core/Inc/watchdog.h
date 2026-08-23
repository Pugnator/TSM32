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

  /** Decode and log the cause of the previous reset (RCC_CSR), then clear the
   *  flags. IWDG/software-fault resets are the interesting ones. Logs via
   *  PrintF, so it is silent in a release build. */
  void watchdog_report_reset_cause(void);

#ifdef __cplusplus
}
#endif
