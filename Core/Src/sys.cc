#include <sys/time.h>
#include "tsm.h"
#include "vmmu.h"
#include "assert.h"

#ifdef __cplusplus
extern "C"
{
#endif
  int _getpid(void)
  {
    return 1;
  }

  void _kill(int pid)
  {
  }

  void _exit(int status)
  {
    _kill(status);
    while (1)
    {
    }
  }

  char *_sbrk(int delta)
  {
    return nullptr;
  }

  int _close(void)
  {
    return 1;
  }

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

  int _isatty(int /*fd*/)
  {
    // Dummy implementation
    // Return 1 (true) to simulate a terminal-like device
    // Return 0 (false) otherwise

    return 0; // Change to 1 to simulate a terminal-like device
  }

  int _fstat(int /*fd*/, struct stat * /*buf*/)
  {
    // Dummy implementation
    // Set the members of the 'buf' structure to simulate file status information

    return 0; // Return 0 to indicate success
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
  assert(p);
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
