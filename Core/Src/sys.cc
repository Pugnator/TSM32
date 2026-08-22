#include <sys/time.h>
#include "tsm.h"
#include "vmmu.h"
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

void *operator new(size_t n)
{
  void *const p = stalloc(n);
  if (!p)
  {
    /* assert() compiles out under NDEBUG, leaving callers to dereference
     * a nullptr return. Trap unconditionally so OOM is observable in
     * release builds (Fixes #46). */
    PrintF("FATAL: operator new(%u) OOM\r\n", (unsigned)n);
    NVIC_SystemReset();
  }
  return p;
}

void operator delete(void *p)
{
  stfree(p);
}

void operator delete(void *p, unsigned int)
{
  stfree(p);
}
