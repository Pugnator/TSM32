#pragma once
#include "printf.h"

#define LOGGING_CHANNEL 0
#define USE_UART2_TRACE

#ifdef NDEBUG
#define DEBUG_LOG PrintF
#else
#define DEBUG_LOG(...)
#endif

#if defined (USE_SEGGER_RTT_TRACE)
#define Print(x) SEGGER_RTT_WriteString(LOGGING_CHANNEL, x)
#define PrintF(...) SEGGER_RTT_printf(LOGGING_CHANNEL, ##__VA_ARGS__)
#elif defined (USE_SWO_TRACE)
#define Print(x) SEGGER_RTT_WriteString(LOGGING_CHANNEL, x)
#define PrintF(...) SEGGER_RTT_printf(LOGGING_CHANNEL, ##__VA_ARGS__)
#elif defined (USE_UART2_TRACE)
#define Print printf_
#define PrintF printf_
#else
#error Define USE_SEGGER_RTT_TRACE USE_SWO_TRACE or USE_UART1_TRACE
#endif
