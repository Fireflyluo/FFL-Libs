/**
 * @file sw_timer_test.cpp
 * @brief C++ 消费方对 sw_timer C ABI 与时间轮语义的 host 测试。
 */

#include <cassert>

#include "sw_timer.h"

struct CallbackState {
    int calls;
};

static void count_callback(void *arg)
{
    auto *state = static_cast<CallbackState *>(arg);
    ++state->calls;
}

static void test_requires_initialization()
{
    sw_timer_t timer{};
    CallbackState state{0};

    assert(sw_timer_start(&timer, 1U, 0U, count_callback, &state) == -2);
    assert(sw_timer_start(nullptr, 1U, 0U, count_callback, &state) == -1);
    assert(sw_timer_start(&timer, 1U, 0U, nullptr, &state) == -1);
}

static void test_one_shot_rounding()
{
    sw_timer_t timer{};
    CallbackState state{0};

    sw_timer_wheel_init(10U);
    assert(sw_timer_start(&timer, 25U, 0U, count_callback, &state) == 0);

    sw_timer_process();
    assert(state.calls == 0);
    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 0);
    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 0);
    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 1);
    sw_timer_process();
    assert(state.calls == 1);
}

static void test_exact_full_wheel_delay()
{
    sw_timer_t timer{};
    CallbackState state{0};

    sw_timer_wheel_init(1U);
    assert(sw_timer_start(&timer, 256U, 0U, count_callback, &state) == 0);

    for (int tick = 0; tick < 255; ++tick) {
        sw_timer_tick_isr();
        sw_timer_process();
    }
    assert(state.calls == 0);

    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 1);
}

static void test_stop_removes_expired_timer()
{
    sw_timer_t timer{};
    CallbackState state{0};

    sw_timer_wheel_init(1U);
    assert(sw_timer_start(&timer, 0U, 0U, count_callback, &state) == 0);
    sw_timer_stop(&timer);
    sw_timer_process();

    assert(state.calls == 0);
    assert(timer.active == 0U);
}

static void test_periodic_timer()
{
    sw_timer_t timer{};
    CallbackState state{0};

    sw_timer_wheel_init(10U);
    assert(sw_timer_start(&timer, 10U, 20U, count_callback, &state) == 0);

    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 1);
    assert(timer.active != 0U);

    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 1);
    sw_timer_tick_isr();
    sw_timer_process();
    assert(state.calls == 2);

    sw_timer_stop(&timer);
    assert(timer.active == 0U);
}

int main()
{
    test_requires_initialization();
    test_one_shot_rounding();
    test_exact_full_wheel_delay();
    test_stop_removes_expired_timer();
    test_periodic_timer();
    return 0;
}
