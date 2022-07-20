#pragma once
#include "printf.h"

#define LOGGING_CHANNEL 0
#define NDEBUG

//#define USE_UART2_TRACE
#define USE_SEGGER_RTT_TRACE

#ifdef NDEBUG
#define DEBUG_LOG PrintF
#else
#define DEBUG_LOG(...)
#endif

#define CPU_CORE_FREQUENCY_HZ 100000000

#define Print printf_
#define PrintF printf_

#ifdef __cplusplus
extern "C"
{
#endif

void dump_registers(int r0, int r1, int r2, int r3);
void die();
#ifdef __cplusplus
}
#endif
