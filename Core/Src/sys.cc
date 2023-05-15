#include <sys/time.h>
#include "tsm.h"
#include "vmmu.h"

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
    // handle p == 0
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
