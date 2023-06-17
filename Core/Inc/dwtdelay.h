/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  static inline void DWT_Init();
  static inline void DWT_Delay(uint32_t us);

  /**
   * Initialization routine.
   * You might need to enable access to DWT registers on Cortex-M7
   *   DWT->LAR = 0xC5ACCE55
   */
  static inline void DWT_Init()
  {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CYCCNT = 0;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
  }

  /**
   * Delay routine itself.
   * Time is in microseconds (1/1000000th of a second), not to be
   * confused with millisecond (1/1000th).
   *
   * No need to check an overflow. Let it just tick :)
   *
   * @param uint32_t us  Number of microseconds to delay for
   */
  static inline void DWT_Delay(uint32_t us) // microseconds
  {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while (DWT->CYCCNT - startTick < delayTicks)
      ;
  }
#ifdef __cplusplus
}
#endif
