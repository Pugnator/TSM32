#include "tsm.h"
#include <stdio.h>

#ifdef __has_include
  #if __has_include("usart.h")
    #include "usart.h"
  #endif
#endif

#include "trace.h"
#include "SEGGER_RTT.h"

#ifdef __cplusplus
extern "C"
{
#endif

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

#ifdef __cplusplus
}
#endif