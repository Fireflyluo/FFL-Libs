#include "../inc/sht40.h"

#include <assert.h>
#include <string.h>

typedef struct {
    uint8_t mem[8];
} mock_bus_t;

static void mock_delay(void *ctx, uint32_t ms)
{
    (void)ctx;
    (void)ms;
}

static int mock_xfer(void *ctx,
                     const sht40_comm_msg_t *msgs,
                     uint8_t cnt,
                     sht40_bus_done_cb_t cb,
                     void *user)
{
    mock_bus_t *bus = (mock_bus_t *)ctx;
    (void)bus;

    if (cnt != 1u) {
        return -1;
    }

    if ((msgs[0].flags & SHT40_COMM_READ) != 0u && msgs[0].len == 6u) {
        uint8_t d[6] = {0x66, 0x66, 0, 0x80, 0x00, 0};
        memcpy(msgs[0].buf, d, 6);
    }

    if (cb != 0) {
        cb(user, 0);
    }
    return 0;
}

static const sht40_bus_ops_t g_ops = { .xfer = mock_xfer, .cancel = 0 };

int main(void)
{
    mock_bus_t bus;
    sht40_dev_t dev;
    sht40_sample_t s;

    memset(&bus, 0, sizeof(bus));
    memset(&dev, 0, sizeof(dev));

    dev.ops = &g_ops;
    dev.bus_ctx = &bus;
    dev.delay_ms = mock_delay;

    assert(sht40_init(&dev) == 0);
    assert(sht40_read_sample(&dev, SHT40_PRECISION_HIGH, &s) == 0);
    return 0;
}
