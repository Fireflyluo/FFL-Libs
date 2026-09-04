#ifndef FFL_ATOMIC_H
#define FFL_ATOMIC_H

#include <stdint.h>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

static inline int ffl_atomic_try_lock_u8(volatile uint8_t *value)
{
#if defined(_MSC_VER)
    return _InterlockedCompareExchange8((volatile char *)value, 1, 0) == 0;
#elif defined(__GNUC__) || defined(__clang__)
    return __sync_bool_compare_and_swap(value, 0U, 1U);
#else
#error "ffl_atomic requires MSVC, GCC, or Clang atomic intrinsics"
#endif
}

static inline void ffl_atomic_unlock_u8(volatile uint8_t *value)
{
#if defined(_MSC_VER)
    _InterlockedExchange8((volatile char *)value, 0);
#elif defined(__GNUC__) || defined(__clang__)
    __sync_lock_release(value);
#else
#error "ffl_atomic requires MSVC, GCC, or Clang atomic intrinsics"
#endif
}

#endif /* FFL_ATOMIC_H */
