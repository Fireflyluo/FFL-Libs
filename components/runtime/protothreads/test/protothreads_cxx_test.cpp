#include "protothreads.h"

struct cxx_wait_state {
    bool ready;
    bool busy;
    unsigned int steps;
};

static PT_THREAD(cxx_wait_thread(pt_t *pt, cxx_wait_state *state))
{
    PT_BEGIN(pt);
    state->steps = 1U;
    PT_WAIT_UNTIL(pt, state->ready);
    state->steps = 2U;
    PT_YIELD(pt);
    state->steps = 3U;
    PT_WAIT_WHILE(pt, state->busy);
    state->steps = 4U;
    PT_END(pt);
}

int main()
{
    pt_t pt{};
    cxx_wait_state state{false, false, 0U};

    PT_INIT(&pt);
    if (!PT_SCHEDULE(cxx_wait_thread(&pt, &state)) || state.steps != 1U) {
        return 1;
    }

    state.ready = true;
    if (!PT_SCHEDULE(cxx_wait_thread(&pt, &state)) || state.steps != 2U) {
        return 1;
    }

    state.busy = true;
    if (!PT_SCHEDULE(cxx_wait_thread(&pt, &state)) || state.steps != 3U) {
        return 1;
    }

    state.busy = false;
    return PT_SCHEDULE(cxx_wait_thread(&pt, &state)) || state.steps != 4U || pt.lc != 0U
               ? 1
               : 0;
}
