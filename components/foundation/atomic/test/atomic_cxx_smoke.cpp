#include "ffl_atomic.h"

#include <cassert>
#include <cstdint>

static void test_cxx_atomic_api()
{
    volatile std::uint8_t value = 0u;

    assert(ffl_atomic_try_lock_u8(&value) != 0);
    assert(ffl_atomic_load_u8(&value) == 1u);
    ffl_atomic_unlock_u8(&value);

    ffl_atomic_store_u8(&value, 10u);
    assert(ffl_atomic_fetch_add_u8(&value, 2u) == 10u);
    assert(ffl_atomic_fetch_sub_u8(&value, 1u) == 12u);
    assert(ffl_atomic_load_u8(&value) == 11u);
    assert(ffl_atomic_compare_exchange_u8(&value, 11u, 0x7fu) != 0);
    assert(ffl_atomic_load_u8(&value) == 0x7fu);
}

int main()
{
    test_cxx_atomic_api();
    return 0;
}
