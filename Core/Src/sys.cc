#include <sys/time.h>
#include "tsm.h"
#include "assert.h"

#ifdef __cplusplus
extern "C"
{
#endif
  int _gettimeofday(struct timeval *tv, void *tzvp)
  {
    return 0;
  }

  ssize_t _read_r(void *reent, int fd, void *buf, size_t nbytes)
  {
    // Dummy implementation that always returns -1 (indicating an error)
    return -1;
  }

  off_t _lseek_r(void *reent, int fd, off_t offset, int whence)
  {
    // Dummy implementation that always returns -1 (indicating an error)
    return -1;
  }

#ifdef __cplusplus
}
#endif

void __wrap___aeabi_unwind_cpp_pr0()
{
  for (;;)
    ;
}

/* The firmware has NO heap. Its single long-lived object (the AHRS instance
 * in tsm.cc) is a function-local static, and the vmmu pool allocator that
 * once backed operator new was removed with the last `new`. These overrides
 * stay so that any FUTURE accidental allocation fails loudly - log + reset -
 * instead of silently resolving to libstdc++'s default operator new, which
 * would call newlib malloc against the 512-byte _Min_Heap_Size and fault
 * unpredictably later. Unconditional trap, like the old OOM path (#46). */
[[noreturn]] static void noHeapTrap(const char *what, size_t n)
{
  PrintF("FATAL: %s(%u) - firmware has no heap\r\n", what, (unsigned)n);
  NVIC_SystemReset();
  for (;;)
  {
  }
}

void *operator new(size_t n)
{
  noHeapTrap("operator new", n);
}

void *operator new[](size_t n)
{
  noHeapTrap("operator new[]", n);
}

void operator delete(void *p)
{
  (void)p;
  noHeapTrap("operator delete", 0);
}

void operator delete(void *p, unsigned int n)
{
  (void)p;
  noHeapTrap("operator delete", n);
}

void operator delete[](void *p)
{
  (void)p;
  noHeapTrap("operator delete[]", 0);
}

void operator delete[](void *p, unsigned int n)
{
  (void)p;
  noHeapTrap("operator delete[]", n);
}
