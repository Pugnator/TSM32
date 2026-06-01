#pragma once
// Shadow: redirect the custom embedded printf to the system one.
#include <stdio.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef printf_
// Route printf_() to the system printf so test output is visible.
#define printf printf_
static inline int printf_(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int r = vprintf(fmt, args);
    va_end(args);
    return r;
}
static inline int sprintf_(char *buf, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int r = vsprintf(buf, fmt, args);
    va_end(args);
    return r;
}
static inline int snprintf_(char *buf, size_t n, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int r = vsnprintf(buf, n, fmt, args);
    va_end(args);
    return r;
}
#define sprintf  sprintf_
#define snprintf snprintf_
#endif // printf_

#ifdef __cplusplus
}
#endif
