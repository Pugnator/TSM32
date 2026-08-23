/**
 * @file watchdog.h
 * @brief IWDG + WWDG ownership, ported from the D937_Cruise firmware.
 *
 * Two watchdogs with different jobs:
 *
 *   WWDG  catches a wedged main loop fast (~65 ms). It is clocked from
 *         PCLK1, so it cannot detect a stopped APB1 - it dies with it.
 *   IWDG  runs from the independent LSI and is the backstop for exactly
 *         that: a stalled clock tree, or a WWDG that never gets serviced.
 *
 * Unlike D937's fixed 20 ms control frame, the TSM main loop free-runs
 * with legitimate long stretches (carrier-sense waits, bit-banged J1850
 * TX, SPI transfers), so the WWDG window is left fully OPEN (refresh
 * legal at any counter value): it polices only the maximum interval, not
 * the rate.  Long blocking waits kick the watchdogs inline via
 * watchdog_refresh(), which is legal at any time with an open window.
 *
 * Both are irreversible once started.  watchdog_start() is called right
 * before the main loop is entered, after all slow init (MEMS/DMP upload,
 * settings) has finished.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Timing (this board: SYSCLK 64 MHz, PCLK1 32 MHz, LSI 30..60 kHz)
 *
 * WWDG base tick = 32e6 / 4096 = 7812.5 Hz = 128 us; with prescaler /8 one
 * count is 1.024 ms and the usable span is 64 counts:
 *
 *   timeout = (0x7F - 0x3F) x 1.024 ms = 65.5 ms
 *   window  = 0x7F (fully open - any refresh interval below 65.5 ms is legal)
 *
 * Worst legitimate unkicked stretch in the main loop is ~30 ms (back-to-back
 * J1850 frame TX between carrier-sense kicks); a wedged SPI peripheral
 * (100 ms HAL timeouts) blows straight through 65.5 ms, which is the point.
 *
 * IWDG is sized to expire after WWDG under every LSI corner so the WWDG
 * early-wakeup interrupt records the cause first: /32 prescaler, reload
 * 1250 -> 1.0 s nominal, 0.67 s at the 60 kHz LSI corner - still far above
 * the 65.5 ms WWDG ceiling.
 * ------------------------------------------------------------------------- */
#define WATCHDOG_WWDG_COUNTER 0x7FU /**< Reload value; 65.5 ms to reset      */
#define WATCHDOG_WWDG_WINDOW 0x7FU  /**< Fully open: no minimum interval     */
#define WATCHDOG_IWDG_RELOAD 1250U  /**< 0.67..1.33 s across the LSI spread  */

  /**
   * Start both watchdogs.  Irreversible.  Call once, right before entering
   * the main loop (after all slow one-time init).
   */
  void watchdog_start(void);

  /** True once watchdog_start() has run. */
  bool watchdog_started(void);

  /**
   * Refresh both watchdogs.  Call once per main-loop iteration, and from
   * inside legitimate long blocking waits (carrier-sense).  Legal at any
   * time - the WWDG window is open.
   */
  void watchdog_refresh(void);

  /** WWDG early-wakeup ISR body; called from WWDG_IRQHandler. */
  void watchdog_wwdg_irq(void);

  /** Decode and log the cause of the last reset (RCC_CSR), then clear it. */
  void watchdog_report_reset_cause(void);

#ifdef __cplusplus
}
#endif
