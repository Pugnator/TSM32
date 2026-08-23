/**
 * @file watchdog.c
 * @brief Independent watchdog (IWDG) — implementation.
 *
 * Programmed through CMSIS register definitions (RM0008 section 19). No HAL
 * IWDG module and no CubeMX-generated init call are required.
 */
#include "watchdog.h"
#include "trace.h"
#include "stm32f1xx.h"

/* IWDG key register commands (RM0008 19.4.1). */
#define WATCHDOG_IWDG_KEY_RELOAD 0x0000AAAAU
#define WATCHDOG_IWDG_KEY_ENABLE 0x0000CCCCU
#define WATCHDOG_IWDG_KEY_UNLOCK 0x00005555U

/* Prescaler /64 and a 12-bit reload of 1250:
 *   period = 1250 * 64 / LSI.  LSI is specified 30..60 kHz (40 kHz nominal),
 *   so the timeout is ~2.0 s nominal and 1.3..2.7 s across the spread — long
 *   enough to clear the slowest init phase, short enough to bring the TSM back
 *   before the cluster firmly latches the security lamp. */
#define WATCHDOG_IWDG_PR_DIV64 4U
#define WATCHDOG_IWDG_RELOAD 1250U

/* Bound on the shadow-register sync wait; a hang here would be a boot failure,
 * so it gives up rather than spinning forever. */
#define WATCHDOG_IWDG_SYNC_GUARD 100000U

/* Escape hatch: build with WATCHDOG_ENABLED=0 to compile the arming out for
 * board bring-up or bisecting a reset. Refresh/report stay callable. */
#ifndef WATCHDOG_ENABLED
#define WATCHDOG_ENABLED 1
#endif

static bool s_armed;

void watchdog_report_reset_cause(void)
{
  const uint32_t csr = RCC->CSR;
  if (csr & RCC_CSR_IWDGRSTF)
    PrintF("RESET: IWDG watchdog - firmware hang recovered\r\n");
  else if (csr & RCC_CSR_SFTRSTF)
    PrintF("RESET: software - recovered from a CPU fault\r\n");
  else if (csr & RCC_CSR_PORRSTF)
    PrintF("RESET: power-on\r\n");
  else if (csr & RCC_CSR_PINRSTF)
    PrintF("RESET: NRST pin\r\n");
  RCC->CSR |= RCC_CSR_RMVF; /* clear all reset flags for the next boot */
}

void watchdog_init(void)
{
#if !WATCHDOG_ENABLED
  return;
#else
#if DEBUG
  /* Freeze the IWDG while the core is halted at a breakpoint, so a debugging
   * session does not reset the board. RTT logging alone never halts. */
  DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
#endif

  /* Order matters. Enabling the IWDG turns the LSI on, and the PVU/RVU busy
   * bits in IWDG_SR are LSI-clocked: writing the prescaler before starting
   * leaves them set forever. Start, unlock, write (RM0008 19.3.2). */
  IWDG->KR = WATCHDOG_IWDG_KEY_ENABLE; /* start; also forces LSI on */
  IWDG->KR = WATCHDOG_IWDG_KEY_UNLOCK; /* enable writes to PR and RLR */
  IWDG->PR = WATCHDOG_IWDG_PR_DIV64;
  IWDG->RLR = WATCHDOG_IWDG_RELOAD;

  for (uint32_t guard = 0U; guard < WATCHDOG_IWDG_SYNC_GUARD; guard++)
  {
    if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) == 0U)
    {
      break;
    }
  }
  IWDG->KR = WATCHDOG_IWDG_KEY_RELOAD;
  s_armed = true;
#endif
}

void watchdog_refresh(void)
{
  if (s_armed)
  {
    IWDG->KR = WATCHDOG_IWDG_KEY_RELOAD;
  }
}
