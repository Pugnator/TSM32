#pragma once
#include "printf.h"

#define LOGGING_CHANNEL 0

// #define USE_UART2_TRACE
#define USE_SEGGER_RTT_TRACE
#if DEBUG
#define LOG_LEVEL_INFO
#define LOG_LEVEL_DEBUG
// Uncomment to enable verbose bus-level tracing (J1850 pulse timings, voltage, etc.)
// #define LOG_LEVEL_TRACE
#endif

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

// TRACE_LOG: verbose output (J1850 frames, state changes).
// Active only when LOG_LEVEL_TRACE is defined.
#ifdef LOG_LEVEL_TRACE
#define TRACE_LOG PrintF
#else
#define TRACE_LOG(...)
#endif

#define Print printf_

// BANNER: the startup welcome line. Always emitted, even in a release build,
// which otherwise produces no RTT output.
#define BANNER printf_

#if DEBUG
#define PrintF printf_
#else
// Release build: every diagnostic PrintF compiles to a do-nothing call so the
// RTT channel carries only the BANNER welcome line. Arguments are still
// evaluated (side effects preserved) but nothing is printed, and the empty
// inline is optimised away.
static inline void PrintF(const char *fmt, ...) { (void)fmt; }
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
