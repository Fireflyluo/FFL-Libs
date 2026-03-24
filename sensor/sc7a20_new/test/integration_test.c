#include "../inc/sc7a20.h"
#include "../adapters/mock_adapter.h"

#include <assert.h>

static int g_done;

static void on_done(void *user, const sc7a20_vec3i16_t *xyz, int status)
{
    (void)user;
    assert(status == 0);
    assert(xyz != 0);
    g_done = 1;
}

int main(void)
{
    sc7a20_mock_bus_t bus;
    sc7a20_dev_t dev;
    int rc;

    sc7a20_mock_bus_init(&bus);

    dev.ops = &g_sc7a20_mock_ops;
    dev.bus_ctx = &bus;
    dev.addr = SC7A20_I2C_ADDR_L;

    rc = sc7a20_init(&dev);
    assert(rc == 0);

    bus.regs[SC7A20_OUTX_L] = 0x10;
    bus.regs[SC7A20_OUTX_H] = 0x01;
    bus.regs[SC7A20_OUTY_L] = 0x20;
    bus.regs[SC7A20_OUTY_H] = 0x02;
    bus.regs[SC7A20_OUTZ_L] = 0x30;
    bus.regs[SC7A20_OUTZ_H] = 0x03;

    g_done = 0;
    rc = sc7a20_read_xyz_raw_async(&dev, on_done, 0);
    assert(rc == 0);
    assert(g_done == 1);

    return 0;
}
