#include "../inc/sc7a20_core.h"

#include <errno.h>
#include <string.h>

typedef struct {
    uint8_t regs[256];
    int force_error;
} sc7a20_mock_bus_t;

static int mock_xfer(void *ctx,
                     const sc7a20_comm_msg_t *msgs,
                     uint8_t cnt,
                     sc7a20_bus_done_cb_t cb,
                     void *user)
{
    sc7a20_mock_bus_t *bus;
    uint8_t reg;
    uint8_t i;

    if (ctx == NULL || msgs == NULL || cnt < 2u) {
        return -EINVAL;
    }

    bus = (sc7a20_mock_bus_t *)ctx;
    if (bus->force_error != 0) {
        return bus->force_error;
    }

    reg = (uint8_t)(msgs[0].buf[0] & 0x7Fu);

    for (i = 1u; i < cnt; ++i) {
        if ((msgs[i].flags & SC7A20_COMM_READ) != 0u) {
            memcpy(msgs[i].buf, &bus->regs[reg], msgs[i].len);
            reg = (uint8_t)(reg + msgs[i].len);
        } else {
            memcpy(&bus->regs[reg], msgs[i].buf, msgs[i].len);
            reg = (uint8_t)(reg + msgs[i].len);
        }
    }

    if (cb != NULL) {
        cb(user, 0);
    }
    return 0;
}

static int mock_cancel(void *ctx)
{
    (void)ctx;
    return 0;
}

const sc7a20_bus_ops_t g_sc7a20_mock_ops = {
    .xfer = mock_xfer,
    .cancel = mock_cancel,
};

void sc7a20_mock_bus_init(sc7a20_mock_bus_t *bus)
{
    if (bus == NULL) {
        return;
    }
    memset(bus, 0, sizeof(*bus));
    bus->regs[SC7A20_WHO_AM_I] = SC7A20_CHIP_ID;
}
