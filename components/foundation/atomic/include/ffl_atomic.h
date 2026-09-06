#ifndef FFL_ATOMIC_H
#define FFL_ATOMIC_H

#ifdef __cplusplus
extern "C" {
#endif

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

static inline int ffl_atomic_compare_exchange_u8(volatile uint8_t *value,
                                                  uint8_t expected,
                                                  uint8_t desired)
{
#if defined(_MSC_VER)
    return _InterlockedCompareExchange8((volatile char *)value,
                                        (char)desired,
                                        (char)expected) == (char)expected;
#elif defined(__GNUC__) || defined(__clang__)
    return __sync_bool_compare_and_swap(value, expected, desired);
#else
#error "ffl_atomic requires MSVC, GCC, or Clang atomic intrinsics"
#endif
}

static inline uint8_t ffl_atomic_load_u8(volatile uint8_t *value)
{
#if defined(_MSC_VER)
    return (uint8_t)_InterlockedCompareExchange8((volatile char *)value, 0, 0);
#elif defined(__GNUC__) || defined(__clang__)
    return __sync_val_compare_and_swap(value, 0U, 0U);
#else
#error "ffl_atomic requires MSVC, GCC, or Clang atomic intrinsics"
#endif
}

static inline void ffl_atomic_store_u8(volatile uint8_t *value, uint8_t desired)
{
#if defined(_MSC_VER)
    _InterlockedExchange8((volatile char *)value, (char)desired);
#elif defined(__GNUC__) || defined(__clang__)
    __sync_lock_test_and_set(value, desired);
#else
#error "ffl_atomic requires MSVC, GCC, or Clang atomic intrinsics"
#endif
}

static inline uint8_t ffl_atomic_fetch_add_u8(volatile uint8_t *value, uint8_t amount)
{
    uint8_t current = ffl_atomic_load_u8(value);

    for (;;) {
        uint8_t desired = (uint8_t)(current + amount);
        if (ffl_atomic_compare_exchange_u8(value, current, desired)) {
            return current;
        }
        current = ffl_atomic_load_u8(value);
    }
}

static inline uint8_t ffl_atomic_fetch_sub_u8(volatile uint8_t *value, uint8_t amount)
{
    return ffl_atomic_fetch_add_u8(value, (uint8_t)(0u - amount));
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

#ifdef __cplusplus
}
#endif

#endif /* FFL_ATOMIC_H */
