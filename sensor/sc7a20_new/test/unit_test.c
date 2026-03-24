#include "../inc/sc7a20.h"
#include "../adapters/mock_adapter.h"

#include <assert.h>

int main(void)
{
    sc7a20_mock_bus_t bus;
    sc7a20_dev_t dev;
    sc7a20_vec3i16_t xyz;
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

    rc = sc7a20_read_xyz_raw(&dev, &xyz);
    assert(rc == 0);

    return 0;
}
