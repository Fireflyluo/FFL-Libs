#include "osal.h"
#include "osal_event.h"
#include "osal_memory.h"
#include "osal_msg.h"
#include "osal_pt.h"
#include "osal_timer.h"

#include <stdio.h>
#include <stdint.h>

#define EVENT_LOW 0x0001U
#define EVENT_HIGH 0x0002U
#define EVENT_RETAIN 0x0004U
#define EVENT_TIMER 0x0010U
#define EVENT_RELOAD 0x0020U
#define PT_EVENT 0x0040U

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            (void)fprintf(stderr, "check failed: %s:%d: %s\\n", __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

static uint32 critical_depth;
static uint32 critical_enter_calls;
static uint32 critical_exit_calls;
static uint8 critical_violation;
static uint32 tick_init_calls;
static uint32 tick_start_calls;
static uint32 tick_stop_calls;

static uint8 init_order[2];
static uint8 init_count;
static uint8 handler_calls;
static uint8 handler_task_id;
static uint16 handler_events;

static void fake_enter_critical(void)
{
    critical_depth++;
    critical_enter_calls++;
}

static void fake_exit_critical(void)
{
    if (critical_depth == 0U) {
        critical_violation = 1U;
    } else {
        critical_depth--;
    }
    critical_exit_calls++;
}

static void fake_tick_init(void)
{
    tick_init_calls++;
}

static void fake_tick_start(void)
{
    tick_start_calls++;
}

static void fake_tick_stop(void)
{
    tick_stop_calls++;
}

static void fake_task_init(uint8 task_id)
{
    if (init_count < 2U) {
        init_order[init_count] = task_id;
    }
    init_count++;
}

static uint16 fake_task_event_handler(uint8 task_id, uint16 task_event)
{
    handler_calls++;
    handler_task_id = task_id;
    handler_events = task_event;
    return (uint16)(task_event & EVENT_RETAIN);
}

static int test_memory_api(void)
{
    static const uint8 source[] = {1U, 2U, 3U, 4U};
    uint8 destination[sizeof(source)] = {0U};
    uint8 reversed[sizeof(source)] = {0U};
    char text[] = "osal";
    uint8 *duplicate;
    void *allocation;
    uint16 used_before;

    used_before = osal_heap_mem_used();
    allocation = osal_mem_alloc(13U);
    CHECK(allocation != NULL);
    CHECK(osal_heap_mem_used() > used_before);
    osal_mem_free(allocation);
    CHECK(osal_heap_mem_used() == used_before);

    CHECK(osal_memcpy(destination, source, sizeof(source)) == destination + sizeof(source));
    CHECK(osal_memcmp(destination, source, sizeof(source)) == TRUE);
    CHECK(osal_revmemcpy(reversed, source, sizeof(source)) == reversed + sizeof(source));
    CHECK(osal_memcmp(reversed, (uint8[]){4U, 3U, 2U, 1U}, sizeof(reversed)) == TRUE);
    CHECK(osal_memset(destination, 0xA5U, sizeof(destination)) == destination);
    CHECK(destination[0] == 0xA5U && destination[sizeof(destination) - 1U] == 0xA5U);
    CHECK(osal_strlen(text) == 4);

    duplicate = (uint8 *)osal_memdup(source, sizeof(source));
    CHECK(duplicate != NULL);
    CHECK(osal_memcmp(duplicate, source, sizeof(source)) == TRUE);
    osal_mem_free(duplicate);

    CHECK(osal_heap_block_cnt() >= 2U);
    CHECK(osal_heap_block_free() >= 1U);
    CHECK(osal_heap_block_max() >= osal_heap_block_cnt());
    CHECK(osal_heap_high_water() >= osal_heap_mem_used());
    CHECK(osal_heap_mem_usage_rate() <= 100U);
    return 0;
}

static int test_events_and_tasks(void)
{
    OsalTadkREC_t *high_task;
    OsalTadkREC_t *low_task;
    osal_add_Task(fake_task_init, fake_task_event_handler, 1U);
    osal_add_Task(fake_task_init, fake_task_event_handler, 2U);
    CHECK(tasksCnt == 2U);
    CHECK(osalFindTask(0U) != NULL && osalFindTask(1U) != NULL);
    CHECK(osalFindTask(2U) == NULL);

    osal_Task_init();
    CHECK(init_count == 2U && init_order[0] == 1U && init_order[1] == 0U);

    high_task = osalFindTask(1U);
    low_task = osalFindTask(0U);
    CHECK(high_task != NULL && low_task != NULL);
    CHECK(osal_set_event(1U, EVENT_HIGH | EVENT_RETAIN) == ZSUCCESS);
    CHECK(osal_set_event(2U, EVENT_LOW) == INVALID_TASK);
    CHECK(osalNextActiveTask() == high_task);

    osal_process_once();
    CHECK(handler_calls == 1U && handler_task_id == 1U);
    CHECK(handler_events == (EVENT_HIGH | EVENT_RETAIN));
    CHECK(high_task->events == EVENT_RETAIN);
    CHECK(osal_clear_event(1U, EVENT_RETAIN) == ZSUCCESS);
    CHECK(osalNextActiveTask() == NULL);

    CHECK(osal_set_event(0U, EVENT_LOW) == ZSUCCESS);
    CHECK(osalNextActiveTask() == low_task);
    CHECK(osal_clear_event(0U, EVENT_LOW) == ZSUCCESS);
    CHECK(osal_clear_event(2U, EVENT_LOW) == INVALID_TASK);
    CHECK(osalNextActiveTask() == NULL);
    return 0;
}

static int test_messages(void)
{
    osal_msg_q_t local_queue = NULL;
    uint8 *first_message;
    uint8 *second_message;
    uint8 *invalid_message;
    uint8 *queued_first;
    uint8 *queued_second;
    uint8 *queued_third;
    uint8 *received_message;

    CHECK(osal_clear_event(0U, 0xFFFFU) == ZSUCCESS);
    CHECK(osal_clear_event(1U, 0xFFFFU) == ZSUCCESS);

    first_message = osal_msg_allocate(3U);
    second_message = osal_msg_allocate(3U);
    CHECK(first_message != NULL && second_message != NULL);
    first_message[0] = 0x42U;
    first_message[1] = 0x01U;
    second_message[0] = 0x43U;
    second_message[1] = 0x02U;
    CHECK(osal_msg_send(1U, first_message) == SUCCESS);
    CHECK(osal_msg_send(1U, second_message) == SUCCESS);
    CHECK(osal_msg_deallocate(first_message) == MSG_BUFFER_NOT_AVAIL);
    CHECK(osal_msg_find(1U, 0x42U) == (osal_event_hdr_t *)first_message);

    received_message = osal_msg_receive(1U);
    CHECK(received_message == first_message);
    CHECK(OSAL_MSG_ID(received_message) == TASK_NO_TASK);
    CHECK(osal_msg_deallocate(received_message) == SUCCESS);
    CHECK((osalFindTask(1U)->events & 0x8000U) != 0U);

    received_message = osal_msg_receive(1U);
    CHECK(received_message == second_message);
    CHECK(osal_msg_deallocate(received_message) == SUCCESS);
    CHECK((osalFindTask(1U)->events & 0x8000U) == 0U);
    CHECK(osal_msg_receive(1U) == NULL);
    CHECK(osal_msg_deallocate(NULL) == INVALID_MSG_POINTER);

    invalid_message = osal_msg_allocate(1U);
    CHECK(invalid_message != NULL);
    CHECK(osal_msg_send(tasksCnt, invalid_message) == INVALID_TASK);

    queued_first = osal_msg_allocate(1U);
    queued_second = osal_msg_allocate(1U);
    queued_third = osal_msg_allocate(1U);
    CHECK(queued_first != NULL && queued_second != NULL && queued_third != NULL);
    osal_msg_enqueue(&local_queue, queued_first);
    CHECK(osal_msg_enqueue_max(&local_queue, queued_second, 2U) == TRUE);
    CHECK(osal_msg_enqueue_max(&local_queue, queued_third, 2U) == FALSE);
    CHECK(osal_msg_dequeue(&local_queue) == queued_first);
    CHECK(osal_msg_dequeue(&local_queue) == queued_second);
    CHECK(osal_msg_dequeue(&local_queue) == NULL);
    CHECK(osal_msg_deallocate(queued_first) == SUCCESS);
    CHECK(osal_msg_deallocate(queued_second) == SUCCESS);
    CHECK(osal_msg_deallocate(queued_third) == SUCCESS);
    return 0;
}

static int test_timers(void)
{
    CHECK(osal_timer_num_active() == 0U);
    CHECK(osal_start_timerEx(0U, EVENT_TIMER, 3U) == SUCCESS);
    CHECK(osal_timer_num_active() == 1U);
    CHECK(osal_get_timeoutEx(0U, EVENT_TIMER) == 3U);
    CHECK(tick_start_calls == 1U);

    osal_clear_event(0U, 0xFFFFU);
    osal_update_timers();
    CHECK(osal_GetSystemClock() == 1U);
    CHECK(osal_get_timeoutEx(0U, EVENT_TIMER) == 2U);
    osal_update_timers();
    CHECK(osal_get_timeoutEx(0U, EVENT_TIMER) == 1U);
    osal_update_timers();
    CHECK(osal_timer_num_active() == 0U);
    CHECK((osalFindTask(0U)->events & EVENT_TIMER) != 0U);

    osal_clear_event(0U, 0xFFFFU);
    CHECK(osal_start_reload_timer(0U, EVENT_RELOAD, 2U) == SUCCESS);
    osal_update_timers();
    CHECK(osal_get_timeoutEx(0U, EVENT_RELOAD) == 1U);
    osal_update_timers();
    CHECK(osal_get_timeoutEx(0U, EVENT_RELOAD) == 2U);
    CHECK((osalFindTask(0U)->events & EVENT_RELOAD) != 0U);
    osal_clear_event(0U, EVENT_RELOAD);
    osal_update_timers();
    osal_update_timers();
    CHECK((osalFindTask(0U)->events & EVENT_RELOAD) != 0U);
    CHECK(osal_stop_timerEx(0U, EVENT_RELOAD) == SUCCESS);
    CHECK(osal_stop_timerEx(0U, EVENT_RELOAD) == INVALID_EVENT_ID);
    osal_update_timers();
    CHECK(osal_timer_num_active() == 0U);
    CHECK(osal_stop_timerEx(0U, EVENT_RELOAD) == INVALID_EVENT_ID);
    return 0;
}

typedef struct {
    uint8 steps;
} pt_test_state_t;

static PT_THREAD(host_pt_entry(osal_pt_t *pt, void *arg))
{
    pt_test_state_t *state = (pt_test_state_t *)arg;

    PT_BEGIN_WRAPPER(pt);
    state->steps = 1U;
    PT_WAIT_EVENT(pt, PT_EVENT);
    state->steps = 2U;
    PT_DELAY(pt, 3U);
    state->steps = 3U;
    PT_END_WRAPPER(pt);
}

static int test_protothread_scheduler(void)
{
    osal_pt_scheduler_t scheduler;
    osal_pt_t *pt;
    pt_test_state_t state = {0U};
    uint8 pt_id;
    uint8 tick;

    osal_pt_scheduler_init(&scheduler);
    pt_id = osal_pt_create(&scheduler, host_pt_entry, &state, "host protothread name");
    CHECK(pt_id == 1U);
    CHECK(scheduler.pt_list != NULL);
    pt = scheduler.pt_list;
    CHECK(pt->id == pt_id && pt->state == PT_STATE_READY);
    CHECK(pt->name[sizeof(pt->name) - 1U] == '\0');

    osal_pt_schedule(&scheduler);
    CHECK(state.steps == 1U && pt->state == PT_STATE_WAITING);
    CHECK(pt->wait_events == PT_EVENT && pt->wakeup_time == 0U);

    osal_pt_set_event(pt, PT_EVENT);
    osal_pt_schedule(&scheduler);
    CHECK(state.steps == 2U && pt->state == PT_STATE_WAITING);
    CHECK(pt->wait_events == 0U && pt->wakeup_time != 0U);

    for (tick = 0U; tick < 2U; ++tick) {
        osal_update_timers();
        osal_pt_schedule(&scheduler);
        CHECK(state.steps == 2U && pt->state == PT_STATE_WAITING);
    }

    osal_update_timers();
    osal_pt_schedule(&scheduler);
    CHECK(state.steps == 3U && pt->state == PT_STATE_EXITED);
    osal_pt_schedule(&scheduler);
    CHECK(scheduler.pt_list == NULL);
    return 0;
}

int main(void)
{
    osal_port_set_critical_hooks(fake_enter_critical, fake_exit_critical);
    osal_port_set_tick_hooks(fake_tick_init, fake_tick_start, fake_tick_stop);
    CHECK(osal_init_system() == ZSUCCESS);
    CHECK(tick_init_calls == 1U && tick_stop_calls == 1U);
    CHECK(test_memory_api() == 0);
    CHECK(test_events_and_tasks() == 0);
    osal_mem_kick();
    CHECK(test_messages() == 0);
    CHECK(test_timers() == 0);
    CHECK(test_protothread_scheduler() == 0);
    CHECK(critical_depth == 0U && critical_violation == 0U);
    CHECK(critical_enter_calls == critical_exit_calls);
    return 0;
}
