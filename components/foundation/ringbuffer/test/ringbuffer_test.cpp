/**
 * @file ringbuffer_test.cpp
 * @brief C++ 消费方对 ringbuffer C ABI 与缓冲区语义的 host 测试。
 */

#include <array>
#include <cassert>
#include <cstdint>

#include "ringbuffer.h"

static void expect_bytes(const uint8_t *actual, const uint8_t *expected, size_t length)
{
    for (size_t index = 0U; index < length; ++index) {
        assert(actual[index] == expected[index]);
    }
}

static void test_invalid_initialization()
{
    ringbuffer_t ringbuffer{};
    std::array<uint8_t, 1> byte{0U};

    ringbuffer_init(&ringbuffer, nullptr, 4U);

    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_ERROR);
    assert(ringbuffer_put(&ringbuffer, nullptr, 0U) == 0U);
    assert(ringbuffer_get(&ringbuffer, nullptr, 0U) == 0U);
    assert(ringbuffer_put(&ringbuffer, byte.data(), byte.size()) == 0U);
    assert(ringbuffer_get(&ringbuffer, byte.data(), byte.size()) == 0U);
}

static void test_exact_capacity_and_basic_read()
{
    ringbuffer_t ringbuffer{};
    std::array<uint8_t, 5> storage{};
    const std::array<uint8_t, 5> input{1U, 2U, 3U, 4U, 5U};
    std::array<uint8_t, 5> output{};

    ringbuffer_init(&ringbuffer, storage.data(), storage.size());

    assert(ringbuffer.buffer_size == storage.size());
    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_EMPTY);
    assert(ringbuffer_put(&ringbuffer, input.data(), input.size()) == input.size());
    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_FULL);
    assert(ringbuffer_get(&ringbuffer, output.data(), output.size()) == output.size());
    expect_bytes(output.data(), input.data(), input.size());
    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_EMPTY);
}

static void test_wrap_preserves_fifo_order()
{
    ringbuffer_t ringbuffer{};
    std::array<uint8_t, 4> storage{};
    const std::array<uint8_t, 3> first_input{1U, 2U, 3U};
    const std::array<uint8_t, 2> first_output{1U, 2U};
    const std::array<uint8_t, 3> second_input{4U, 5U, 6U};
    const std::array<uint8_t, 4> expected{3U, 4U, 5U, 6U};
    std::array<uint8_t, 4> output{};

    ringbuffer_init(&ringbuffer, storage.data(), storage.size());
    assert(ringbuffer_put(&ringbuffer, first_input.data(), first_input.size()) == first_input.size());
    assert(ringbuffer_get(&ringbuffer, output.data(), first_output.size()) == first_output.size());
    expect_bytes(output.data(), first_output.data(), first_output.size());
    assert(ringbuffer_put(&ringbuffer, second_input.data(), second_input.size()) == second_input.size());
    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_FULL);
    assert(ringbuffer_get(&ringbuffer, output.data(), output.size()) == output.size());
    expect_bytes(output.data(), expected.data(), expected.size());
}

static void test_force_write_keeps_latest_bytes()
{
    ringbuffer_t ringbuffer{};
    std::array<uint8_t, 4> storage{};
    const std::array<uint8_t, 6> input{1U, 2U, 3U, 4U, 5U, 6U};
    const std::array<uint8_t, 4> expected{3U, 4U, 5U, 6U};
    std::array<uint8_t, 4> output{};

    ringbuffer_init(&ringbuffer, storage.data(), storage.size());
    assert(ringbuffer_put_force(&ringbuffer, input.data(), input.size()) == storage.size());
    assert(ringbuffer_status(&ringbuffer) == RINGBUFFER_FULL);
    assert(ringbuffer_get(&ringbuffer, output.data(), output.size()) == output.size());
    expect_bytes(output.data(), expected.data(), expected.size());
}

int main()
{
    test_invalid_initialization();
    test_exact_capacity_and_basic_read();
    test_wrap_preserves_fifo_order();
    test_force_write_keeps_latest_bytes();
    return 0;
}
