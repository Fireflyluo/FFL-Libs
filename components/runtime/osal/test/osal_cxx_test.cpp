#include "osal.h"
#include "osal_event.h"
#include "osal_memory.h"
#include "osal_msg.h"
#include "osal_pt.h"
#include "osal_timer.h"
#include "timer.h"
#include "type.h"

static void cxx_enter_critical()
{
}

static void cxx_exit_critical()
{
}

static void cxx_tick_hook()
{
}

int main()
{
    static_assert(sizeof(uint8) == 1U, "uint8 must remain one byte");
    static_assert(sizeof(uint16) == 2U, "uint16 must remain two bytes");
    osal_pt_scheduler_t scheduler{};
    uint8 *message;
    void *allocation;

    osal_port_set_critical_hooks(cxx_enter_critical, cxx_exit_critical);
    osal_port_set_tick_hooks(cxx_tick_hook, cxx_tick_hook, cxx_tick_hook);
    if (osal_init_system() != ZSUCCESS) {
        return 1;
    }

    allocation = osal_mem_alloc(8U);
    if (allocation == nullptr) {
        return 1;
    }
    osal_mem_free(allocation);

    message = osal_msg_allocate(1U);
    if (message == nullptr || osal_msg_deallocate(message) != SUCCESS) {
        return 1;
    }

    osal_pt_scheduler_init(&scheduler);
    osal_pt_set_event(nullptr, 1U);
    osal_pt_delay(nullptr, 1U);
    osal_update_timers();
    return 0;
}
