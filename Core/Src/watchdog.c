/**
 * @file watchdog.c
 * @brief IWDG + WWDG ownership - implementation (ported from D937_Cruise).
 *
 * Programmed through CMSIS register definitions rather than the HAL, so the
 * module needs no HAL_IWDG/HAL_WWDG module enables and no CubeMX-generated
 * init calls.  Both peripherals are a handful of registers; see RM0008
 * sections 19 (IWDG) and 20 (WWDG).
 */
#include "watchdog.h"
#include "trace.h"
#include "stm32f1xx.h"

/* IWDG key register commands (RM0008 19.4.1). */
#define WATCHDOG_IWDG_KEY_RELOAD 0x0000AAAAU
#define WATCHDOG_IWDG_KEY_ENABLE 0x0000CCCCU
#define WATCHDOG_IWDG_KEY_UNLOCK 0x00005555U

/* IWDG_PR value for a /32 prescaler (RM0008 19.4.2) */
#define IWDG_PR_DIV32 3U

/* Bound on the shadow-register sync wait.  At 64 MHz this is well under a
 * millisecond, and far longer than the few LSI cycles the sync needs. */
#define WATCHDOG_IWDG_SYNC_GUARD 100000U

/* WWDG_CFR timebase field for a /8 prescaler (RM0008 20.4.2) */
#define WWDG_CFR_WDGTB_DIV8 (WWDG_CFR_WDGTB_0 | WWDG_CFR_WDGTB_1)

/* Escape hatch for bringing up a board or bisecting a reset: build with
 * WATCHDOG_ENABLED=0 to compile the arming out entirely.  Refresh and status
 * stay callable so nothing else needs conditional code. */
#ifndef WATCHDOG_ENABLED
#define WATCHDOG_ENABLED 1
#endif

static bool s_started; /**< start() has run, whatever the outcome */
static bool s_armed;   /**< hardware actually armed and needs refreshing */

void watchdog_report_reset_cause(void)
{
  const uint32_t csr = RCC->CSR;
  if (csr & RCC_CSR_IWDGRSTF)
    PrintF("RESET CAUSE: independent watchdog (IWDG) - firmware hang\r\n");
  if (csr & RCC_CSR_WWDGRSTF)
    PrintF("RESET CAUSE: window watchdog (WWDG) - main loop wedged\r\n");
  if (csr & RCC_CSR_SFTRSTF)
    PrintF("Reset cause: software reset\r\n");
  if (csr & RCC_CSR_PORRSTF)
    PrintF("Reset cause: power-on\r\n");
  else if (csr & RCC_CSR_PINRSTF)
    PrintF("Reset cause: NRST pin\r\n");
  RCC->CSR |= RCC_CSR_RMVF; /* clear all reset flags for the next boot */
}

void watchdog_start(void)
{
  if (s_started)
  {
    return;
  }
#if !WATCHDOG_ENABLED
  PrintF("WDG: disabled at build time (WATCHDOG_ENABLED=0)\r\n");
  s_started = true; /* stop re-entering; refresh becomes a no-op */
  return;
#else

#if DEBUG
  /* Freeze both watchdogs while the core is halted by a debugger, so
   * breakpoints do not reset the board.  RTT logging alone never halts. */
  DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP | DBGMCU_CR_DBG_WWDG_STOP;
#endif

  /* ---- IWDG: no window, safe to arm at any instant -------------------
   *
   * Order matters.  Starting the IWDG is what turns the LSI on, and the
   * PVU/RVU busy bits in IWDG_SR are clocked by the LSI - writing the
   * prescaler before starting leaves them set forever and the wait below
   * never returns.  Start, then unlock, then write (RM0008 19.3.2). */
  IWDG->KR = WATCHDOG_IWDG_KEY_ENABLE; /* start; also forces LSI on */
  IWDG->KR = WATCHDOG_IWDG_KEY_UNLOCK; /* enable writes to PR and RLR */
  IWDG->PR = IWDG_PR_DIV32;
  IWDG->RLR = WATCHDOG_IWDG_RELOAD;

  /* Wait for the shadow registers to take the new values, but never
   * indefinitely: a hang here would be a boot failure, and this runs
   * before anything can recover the device. */
  for (uint32_t guard = 0U; guard < WATCHDOG_IWDG_SYNC_GUARD; guard++)
  {
    if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0U)
    {
      break;
    }
  }
  IWDG->KR = WATCHDOG_IWDG_KEY_RELOAD;

  /* ---- WWDG: 65.5 ms deadman, window fully open ---------------------- */
  RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

  /* Window = 0x7F (open) and the early-wakeup interrupt enabled so the
   * cause is recorded one count before the reset. */
  WWDG->CFR = WWDG_CFR_WDGTB_DIV8 | WWDG_CFR_EWI | (WATCHDOG_WWDG_WINDOW & WWDG_CFR_W);

  /* Highest priority so the last gasp is recorded even while a
   * lower-priority handler is stuck. */
  NVIC_SetPriority(WWDG_IRQn, 0);
  NVIC_EnableIRQ(WWDG_IRQn);

  /* Arming and loading the counter are the same write: WDGA cannot be
   * cleared again by software. */
  WWDG->CR = WWDG_CR_WDGA | (WATCHDOG_WWDG_COUNTER & WWDG_CR_T);

  s_started = true;
  s_armed = true;
  PrintF("WDG: armed - WWDG %u ms deadman, IWDG ~%u ms backstop\r\n",
         (unsigned)((0x7FU - 0x3FU) * 1024U / 1000U),
         (unsigned)(WATCHDOG_IWDG_RELOAD * 32U / 40U));
#endif
}

bool watchdog_started(void)
{
  return s_started;
}

void watchdog_refresh(void)
{
  if (!s_armed)
  {
    return;
  }
  IWDG->KR = WATCHDOG_IWDG_KEY_RELOAD;
  /* WDGA is already set and reads back as such, so writing the whole
   * register keeps the watchdog armed.  With the window open this write is
   * legal at any counter value. */
  WWDG->CR = WWDG_CR_WDGA | (WATCHDOG_WWDG_COUNTER & WWDG_CR_T);
}

void watchdog_wwdg_irq(void)
{
  /* Bounded last gasp: one RTT line (a RAM write - may survive into the
   * host capture), then let the reset run.  The counter is deliberately
   * not reloaded. */
  PrintF("WDG: WWDG last gasp - resetting\r\n");
  WWDG->SR = ~WWDG_SR_EWIF; /* rc_w0: clear EWIF, leave reserved bits set */
}
