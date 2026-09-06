#include "protothreads.h"

typedef struct {
    int ready;
    int busy;
    unsigned int steps;
} wait_state_t;

static PT_THREAD(wait_thread(pt_t *pt, wait_state_t *state))
{
    PT_BEGIN(pt);
    state->steps = 1U;
    PT_WAIT_UNTIL(pt, state->ready != 0);
    state->steps = 2U;
    PT_YIELD(pt);
    state->steps = 3U;
    PT_WAIT_WHILE(pt, state->busy != 0);
    state->steps = 4U;
    PT_END(pt);
}

typedef struct {
    int child_ready;
    unsigned int child_steps;
    unsigned int parent_steps;
} spawn_state_t;

static PT_THREAD(child_thread(pt_t *pt, spawn_state_t *state))
{
    PT_BEGIN(pt);
    state->child_steps = 1U;
    PT_WAIT_UNTIL(pt, state->child_ready != 0);
    state->child_steps = 2U;
    PT_END(pt);
}

static PT_THREAD(parent_thread(pt_t *pt, pt_t *child, spawn_state_t *state))
{
    PT_BEGIN(pt);
    state->parent_steps = 1U;
    PT_SPAWN(pt, child, child_thread(child, state));
    state->parent_steps = 2U;
    PT_END(pt);
}

static PT_THREAD(exit_thread(pt_t *pt, unsigned int *steps))
{
    PT_BEGIN(pt);
    *steps = 1U;
    PT_EXIT(pt);
    PT_END(pt);
}

typedef struct {
    int restart;
    unsigned int runs;
} restart_state_t;

static PT_THREAD(restart_thread(pt_t *pt, restart_state_t *state))
{
    PT_BEGIN(pt);
    ++state->runs;
    if (state->restart != 0) {
        state->restart = 0;
        PT_RESTART(pt);
    }
    ++state->runs;
    PT_END(pt);
}

static int test_wait_and_yield(void)
{
    pt_t pt = {0};
    wait_state_t state = {0};

    PT_INIT(&pt);
    if (!PT_SCHEDULE(wait_thread(&pt, &state)) || state.steps != 1U || pt.lc == 0U) {
        return 1;
    }

    state.ready = 1;
    if (!PT_SCHEDULE(wait_thread(&pt, &state)) || state.steps != 2U) {
        return 1;
    }

    state.busy = 1;
    if (!PT_SCHEDULE(wait_thread(&pt, &state)) || state.steps != 3U) {
        return 1;
    }

    state.busy = 0;
    if (PT_SCHEDULE(wait_thread(&pt, &state)) || state.steps != 4U || pt.lc != 0U) {
        return 1;
    }
    return 0;
}

static int test_spawn(void)
{
    pt_t parent = {0};
    pt_t child = {0x55U};
    spawn_state_t state = {0};

    PT_INIT(&parent);
    if (!PT_SCHEDULE(parent_thread(&parent, &child, &state)) ||
        state.parent_steps != 1U || state.child_steps != 1U || child.lc == 0U) {
        return 1;
    }

    state.child_ready = 1;
    if (PT_SCHEDULE(parent_thread(&parent, &child, &state)) ||
        state.parent_steps != 2U || state.child_steps != 2U || parent.lc != 0U ||
        child.lc != 0U) {
        return 1;
    }
    return 0;
}

static int test_exit(void)
{
    pt_t pt = {0};
    unsigned int steps = 0U;
    PT_INIT(&pt);

    if (PT_SCHEDULE(exit_thread(&pt, &steps)) || steps != 1U || pt.lc != 0U) {
        return 1;
    }
    return 0;
}

static int test_restart(void)
{
    pt_t pt = {0};
    restart_state_t state = {1, 0U};
    PT_INIT(&pt);

    if (!PT_SCHEDULE(restart_thread(&pt, &state)) || state.runs != 1U || pt.lc != 0U) {
        return 1;
    }

    if (PT_SCHEDULE(restart_thread(&pt, &state)) || state.runs != 3U || pt.lc != 0U) {
        return 1;
    }
    return 0;
}

int main(void)
{
    return test_wait_and_yield() != 0 || test_spawn() != 0 || test_exit() != 0 ||
                   test_restart() != 0
               ? 1
               : 0;
}
