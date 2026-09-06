#include "ffl_atomic.h"

#include <assert.h>
#include <stdint.h>

static void test_lock_and_access(void)
{
    volatile uint8_t value = 0u;

    assert(ffl_atomic_load_u8(&value) == 0u);
    assert(ffl_atomic_try_lock_u8(&value) != 0);
    assert(ffl_atomic_load_u8(&value) == 1u);
    assert(ffl_atomic_try_lock_u8(&value) == 0);

    ffl_atomic_unlock_u8(&value);
    assert(ffl_atomic_load_u8(&value) == 0u);

    ffl_atomic_store_u8(&value, 0x5au);
    assert(ffl_atomic_load_u8(&value) == 0x5au);
}

static void test_compare_exchange(void)
{
    volatile uint8_t value = 0x21u;

    assert(ffl_atomic_compare_exchange_u8(&value, 0x21u, 0x42u) != 0);
    assert(ffl_atomic_load_u8(&value) == 0x42u);
    assert(ffl_atomic_compare_exchange_u8(&value, 0x21u, 0x63u) == 0);
    assert(ffl_atomic_load_u8(&value) == 0x42u);
}

static void test_fetch_operations(void)
{
    volatile uint8_t value = 250u;

    assert(ffl_atomic_fetch_add_u8(&value, 10u) == 250u);
    assert(ffl_atomic_load_u8(&value) == 4u);
    assert(ffl_atomic_fetch_sub_u8(&value, 7u) == 4u);
    assert(ffl_atomic_load_u8(&value) == 253u);
}

int main(void)
{
    test_lock_and_access();
    test_compare_exchange();
    test_fetch_operations();
    return 0;
}
