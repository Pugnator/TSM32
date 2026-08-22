#pragma once
// Shadow: no-op assert for host-side tests.
// The real assert macro is already stubbed in stubs.h.
#ifndef assert
#define assert(x) ((void)(x))
#endif
