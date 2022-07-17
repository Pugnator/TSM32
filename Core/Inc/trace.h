#pragma once
#include "printf.h"

#define LOGGING_CHANNEL 0
#define NDEBUG
#define USE_UART2_TRACE

#ifdef NDEBUG
#define DEBUG_LOG PrintF
#else
#define DEBUG_LOG(...)
#endif

#define CPU_CORE_FREQUENCY_HZ 100000000

#if defined(USE_SEGGER_RTT_TRACE)
#define Print(x) SEGGER_RTT_WriteString(LOGGING_CHANNEL, x)
#define PrintF(...) SEGGER_RTT_printf(LOGGING_CHANNEL, ##__VA_ARGS__)
#elif defined(USE_SWO_TRACE)
#define Print(x) SEGGER_RTT_WriteString(LOGGING_CHANNEL, x)
#define PrintF(...) SEGGER_RTT_printf(LOGGING_CHANNEL, ##__VA_ARGS__)
#elif defined(USE_UART2_TRACE)
#define Print printf_
#define PrintF printf_
#else
#error Define USE_SEGGER_RTT_TRACE USE_SWO_TRACE or USE_UART2_TRACE
#endif
#ifdef __cplusplus
extern "C"
{
#endif

void dump_registers(int r0, int r1, int r2, int r3);
void die();
#ifdef __cplusplus
}
#endif
