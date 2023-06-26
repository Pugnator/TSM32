#pragma once
#include "printf.h"

#define LOGGING_CHANNEL 0


//#define USE_UART2_TRACE
#define USE_SEGGER_RTT_TRACE
#define LOG_LEVEL_INFO
//#define LOG_LEVEL_DEBUG


#ifdef LOG_LEVEL_DEBUG
#define DEBUG_LOG PrintF
#else
#define DEBUG_LOG(...)
#endif

#ifdef LOG_LEVEL_INFO
#define INFO_LOG PrintF
#else
#define INFO_LOG(...)
#endif

#ifdef LOG_LEVEL_WARNING
#define WARN_LOG PrintF
#else
#define WARN_LOG(...)
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
