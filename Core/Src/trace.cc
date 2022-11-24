#include "tsm.h"
#include <stdio.h>
#include "usart.h"
#include "trace.h"
#include "SEGGER_RTT.h"

#ifdef __cplusplus
extern "C"
{
#endif

  void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz)
  {
    uint32_t SWOSpeed = 64000;                              /* default 64k baud rate */
    uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */

    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;                                                     /* enable trace in core debug */
    *((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002;                                         /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
    *((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler;                                       /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
    *((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55;                                         /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */
    ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
    ITM->TPR = ITM_TPR_PRIVMASK_Msk;                                                                   /* ITM Trace Privilege Register */
    ITM->TER = portBits;                                                                               /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
    *((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE;                                         /* DWT_CTRL */
    *((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100;                                         /* Formatter and Flush Control Register */
  }

  void SWO_PrintChar(char c, uint8_t portNo)
  {
    volatile int timeout;

    /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0)
    {         /* check Trace Control Register if ITM trace is enabled*/
      return; /* not enabled? */
    }
    /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
    if ((ITM->TER & (1ul << portNo)) == 0)
    {         /* check Trace Enable Register if requested port is enabled */
      return; /* requested port not enabled? */
    }
    timeout = 5000; /* arbitrary timeout value */
    while (ITM->PORT[0].u32 == 0)
    {
      /* Wait until STIMx is ready, then send data */
      timeout--;
      if (timeout == 0)
      {
        return; /* not able to send */
      }
    }
    ITM->PORT[0].u16 = 0x08 | (c << 8);
  }

  void _putchar(char character)
  {
#if defined(USE_SEGGER_RTT_TRACE)
    SEGGER_RTT_PutChar(LOGGING_CHANNEL, character);
#elif defined(USE_SWO_TRACE)
#error SWO TRACE is not implemented yet.
#elif defined(USE_UART2_TRACE)
  HAL_UART_Transmit(&huart2, reinterpret_cast<uint8_t *>(&character), 1, 100);
#else
#error Define USE_SEGGER_RTT_TRACE USE_SWO_TRACE or USE_UART2_TRACE
#endif
  }

  // TO USE: addr2line -e ./bin/program.elf -a 0x8002327 [GDB: p/x pc when it hit for(;;)]
  void memory_dump(uint32_t *stackAddress)
  {
#ifdef MEMDMP
    Print("***Crash dump initiated***\n");

    /*
   These are volatile to try and prevent the compiler/linker optimising them
   away as the variables never actually get used.  If the debugger won't show the
   values of the variables, make them global my moving their declaration outside
   of this function.
   */
    volatile uint32_t r0 = stackAddress[0];
    volatile uint32_t r1 = stackAddress[1];
    volatile uint32_t r2 = stackAddress[2];
    volatile uint32_t r3 = stackAddress[3];
    volatile uint32_t r12 = stackAddress[4];
    /* Link register. */
    volatile uint32_t lr = stackAddress[5];
    /* Program counter. */
    volatile uint32_t pc = stackAddress[6];
    /* Program status register. */
    volatile uint32_t psr = stackAddress[7];

    PrintF("Dumping CPU registers...\r\n");

    char registers[128];
    uint32_t written;
    DEBUG_LOG("R0:  0x%08X\nR1:  0x%08X\nR2:  0x%08X\nR3:  0x%08X\nR12: 0x%08X\r\n",
              r0, r1, r2, r3, r12);

    DEBUG_LOG("LR:  0x%08X\nPC:  0x%08X\nPSR: 0x%08X\r\n", lr, pc, psr);
#endif
    die();
  }

  void die()
  {
    for (;;)
      ;
  }

#ifdef __cplusplus
}
#endif