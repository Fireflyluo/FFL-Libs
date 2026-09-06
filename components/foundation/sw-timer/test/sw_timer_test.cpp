/**
 * @file sw_timer_test.cpp
 * @brief C++ 消费方对 ffl_sw_timer C ABI 与时间轮语义的 host 测试。
 */

#include <cassert>

#include "ffl/sw_timer.h"

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
    ffl_sw_timer_t timer{};
    CallbackState state{0};

    assert(ffl_sw_timer_start(&timer, 1U, 0U, count_callback, &state) == -2);
    assert(ffl_sw_timer_start(nullptr, 1U, 0U, count_callback, &state) == -1);
    assert(ffl_sw_timer_start(&timer, 1U, 0U, nullptr, &state) == -1);
}

static void test_one_shot_rounding()
{
    ffl_sw_timer_t timer{};
    CallbackState state{0};

    ffl_sw_timer_wheel_init(10U);
    assert(ffl_sw_timer_start(&timer, 25U, 0U, count_callback, &state) == 0);

    ffl_sw_timer_process();
    assert(state.calls == 0);
    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 0);
    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 0);
    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 1);
    ffl_sw_timer_process();
    assert(state.calls == 1);
}

static void test_exact_full_wheel_delay()
{
    ffl_sw_timer_t timer{};
    CallbackState state{0};

    ffl_sw_timer_wheel_init(1U);
    assert(ffl_sw_timer_start(&timer, 256U, 0U, count_callback, &state) == 0);

    for (int tick = 0; tick < 255; ++tick) {
        ffl_sw_timer_tick_isr();
        ffl_sw_timer_process();
    }
    assert(state.calls == 0);

    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 1);
}

static void test_stop_removes_expired_timer()
{
    ffl_sw_timer_t timer{};
    CallbackState state{0};

    ffl_sw_timer_wheel_init(1U);
    assert(ffl_sw_timer_start(&timer, 0U, 0U, count_callback, &state) == 0);
    ffl_sw_timer_stop(&timer);
    ffl_sw_timer_process();

    assert(state.calls == 0);
    assert(timer.active == 0U);
}

static void test_periodic_timer()
{
    ffl_sw_timer_t timer{};
    CallbackState state{0};

    ffl_sw_timer_wheel_init(10U);
    assert(ffl_sw_timer_start(&timer, 10U, 20U, count_callback, &state) == 0);

    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 1);
    assert(timer.active != 0U);

    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 1);
    ffl_sw_timer_tick_isr();
    ffl_sw_timer_process();
    assert(state.calls == 2);

    ffl_sw_timer_stop(&timer);
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
