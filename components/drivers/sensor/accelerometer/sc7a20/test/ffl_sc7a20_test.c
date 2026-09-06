#include "ffl/sc7a20.h"

#include <assert.h>
#include <errno.h>
#include <math.h>
#include <string.h>

typedef struct {
    uint8_t regs[256];
    int force_status;
    int cancel_status;
    int defer_callback;
    ffl_xfer_done_fn pending_done;
    void *pending_user;
    ffl_endpoint_t last_endpoint;
    uint8_t last_flags[2];
} fake_sc7a20_bus_t;

static int fake_sc7a20_xfer(void *ctx,
                            const ffl_endpoint_t *endpoint,
                            const ffl_xfer_msg_t *msgs,
                            uint8_t count,
                            ffl_xfer_done_fn done,
                            void *user)
{
    fake_sc7a20_bus_t *bus = (fake_sc7a20_bus_t *)ctx;
    uint8_t reg;

    if (bus == NULL || endpoint == NULL || msgs == NULL || count != 2u) {
        return -EINVAL;
    }
    if (bus->force_status != 0) {
        return bus->force_status;
    }
    if (endpoint->kind != FFL_ENDPOINT_I2C_7BIT ||
        (msgs[0].flags & FFL_XFER_MSG_DIRECTION_MASK) != FFL_XFER_MSG_WRITE ||
        (msgs[1].flags & FFL_XFER_MSG_DIRECTION_MASK) == 0u) {
        return -EINVAL;
    }

    bus->last_endpoint = *endpoint;
    bus->last_flags[0] = msgs[0].flags;
    bus->last_flags[1] = msgs[1].flags;
    reg = (uint8_t)(msgs[0].buf[0] & 0x7Fu);
    if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
        memcpy(msgs[1].buf, &bus->regs[reg], msgs[1].len);
    } else {
        memcpy(&bus->regs[reg], msgs[1].buf, msgs[1].len);
    }

    if (done != NULL) {
        if (bus->defer_callback != 0) {
            bus->pending_done = done;
            bus->pending_user = user;
        } else {
            done(user, 0);
        }
    }
    return 0;
}

static int fake_sc7a20_cancel(void *ctx)
{
    fake_sc7a20_bus_t *bus = (fake_sc7a20_bus_t *)ctx;

    if (bus == NULL) {
        return -EINVAL;
    }
    if (bus->cancel_status != 0) {
        return bus->cancel_status;
    }
    bus->pending_done = NULL;
    bus->pending_user = NULL;
    return 0;
}

static const ffl_transport_ops_t g_fake_sc7a20_transport_ops = {
    .xfer = fake_sc7a20_xfer,
    .cancel = fake_sc7a20_cancel,
};

static void fake_sc7a20_complete(fake_sc7a20_bus_t *bus, int status)
{
    ffl_xfer_done_fn done = bus->pending_done;
    void *user = bus->pending_user;

    bus->pending_done = NULL;
    bus->pending_user = NULL;
    if (done != NULL) {
        done(user, status);
    }
}

static const ffl_time_ops_t g_fake_sc7a20_time_ops = {0};

#if FFL_SC7A20_ASYNC_ENABLED
typedef struct {
    int calls;
    int status;
    sc7a20_vec3i16_t raw;
    ffl_sc7a20_device_t *device;
    int cancel_from_callback;
    int cancel_status;
} async_state_t;

static void fake_sc7a20_read_done(void *user,
                                  const sc7a20_vec3i16_t *raw,
                                  int status)
{
    async_state_t *state = (async_state_t *)user;

    state->calls++;
    state->status = status;
    if (raw != NULL) {
        state->raw = *raw;
    }
    if (state->cancel_from_callback != 0 && state->device != NULL) {
        state->cancel_status = ffl_sc7a20_cancel_async(state->device);
    }
}
#endif

static void fake_sc7a20_setup(fake_sc7a20_bus_t *bus,
                              ffl_sc7a20_device_t *device,
                              ffl_transport_t *transport)
{
    memset(bus, 0, sizeof(*bus));
    memset(device, 0, sizeof(*device));
    bus->regs[SC7A20_WHO_AM_I] = SC7A20_CHIP_ID;
    transport->ops = &g_fake_sc7a20_transport_ops;
    transport->ctx = bus;
    transport->endpoint = ffl_endpoint_i2c7(FFL_SC7A20_DEFAULT_ADDR7_L);
    assert(ffl_sc7a20_bind(device, transport, &g_fake_sc7a20_time_ops, NULL) == 0);
}

int main(void)
{
    fake_sc7a20_bus_t bus;
    ffl_sc7a20_device_t device;
    ffl_transport_t transport;
    ffl_sc7a20_config_t config;
    ffl_sc7a20_raw_t raw;
    ffl_sc7a20_g_t sample;
    uint8_t who = 0u;

    fake_sc7a20_setup(&bus, &device, &transport);
    assert(ffl_sc7a20_set_i2c_addr(&device, FFL_SC7A20_DEFAULT_ADDR7_H) == 0);
    ffl_sc7a20_config_init(&config);
    config.range = FFL_SC7A20_RANGE_4G;
    config.odr = FFL_SC7A20_ODR_200HZ;
    config.high_resolution = true;

    assert(ffl_sc7a20_init(&device, &config) == 0);
    assert(bus.last_endpoint.value.i2c.addr7 == FFL_SC7A20_DEFAULT_ADDR7_H);
    assert(bus.last_flags[0] == FFL_XFER_MSG_WRITE);
    assert(bus.last_flags[1] == (FFL_XFER_MSG_READ | FFL_XFER_MSG_RESTART |
                                 FFL_XFER_MSG_STOP));
    assert(ffl_sc7a20_who_am_i(&device, &who) == 0);
    assert(who == SC7A20_CHIP_ID);

    bus.regs[SC7A20_OUTX_L] = 0x00u;
    bus.regs[SC7A20_OUTX_H] = 0x10u;
    bus.regs[SC7A20_OUTY_L] = 0x00u;
    bus.regs[SC7A20_OUTY_H] = 0xF0u;
    bus.regs[SC7A20_OUTZ_L] = 0x00u;
    bus.regs[SC7A20_OUTZ_H] = 0x08u;
    assert(ffl_sc7a20_read_raw(&device, &raw) == 0);
    assert(raw.x == 0x0100);
    assert(raw.y == -0x0100);
    assert(raw.z == 0x0080);
    assert(ffl_sc7a20_read_g(&device, &sample) == 0);
    assert(fabsf(sample.x - 0.5f) < 0.001f);
    assert(fabsf(sample.y + 0.5f) < 0.001f);

    assert(ffl_sc7a20_set_range(&device, FFL_SC7A20_RANGE_8G) == 0);
    assert((bus.regs[SC7A20_CTRL4] & SC7A20_CTRL4_FS_MASK) ==
           (uint8_t)(FFL_SC7A20_RANGE_8G << SC7A20_CTRL4_FS_SHIFT));
    assert(ffl_sc7a20_set_odr(&device, FFL_SC7A20_ODR_400HZ) == 0);
    assert((bus.regs[SC7A20_CTRL1] & SC7A20_CTRL1_ODR_MASK) ==
           (uint8_t)(FFL_SC7A20_ODR_400HZ << SC7A20_CTRL1_ODR_SHIFT));
    assert(ffl_sc7a20_set_axis_enable(&device, true, false, true) == 0);
    assert((bus.regs[SC7A20_CTRL1] & (SC7A20_CTRL1_XEN_MASK |
                                      SC7A20_CTRL1_YEN_MASK |
                                      SC7A20_CTRL1_ZEN_MASK)) ==
           (SC7A20_CTRL1_XEN_MASK | SC7A20_CTRL1_ZEN_MASK));
    assert(ffl_sc7a20_set_range(&device, (ffl_sc7a20_range_t)99) == -EINVAL);

    config = device.core.cfg;
    bus.force_status = -EIO;
    assert(ffl_sc7a20_set_odr(&device, FFL_SC7A20_ODR_50HZ) == -EIO);
    assert(device.core.cfg.odr == config.odr);
    bus.force_status = 0;

    bus.force_status = -EIO;
    assert(ffl_sc7a20_soft_reset(&device) == -EIO);
    assert(!device.core.initialized);
    assert(ffl_sc7a20_read_raw(&device, &raw) == -ENODEV);
    bus.force_status = 0;
    assert(ffl_sc7a20_init(&device, &config) == 0);

    bus.force_status = -EIO;
    assert(ffl_sc7a20_init(&device, &config) == -EIO);
    assert(!device.core.initialized);
    bus.force_status = 0;
    assert(ffl_sc7a20_init(&device, &config) == 0);

#if FFL_SC7A20_ASYNC_ENABLED
    {
        async_state_t state = {0};

        bus.defer_callback = 1;
        assert(ffl_sc7a20_read_raw_async(&device, fake_sc7a20_read_done, &state) == 0);
        assert(ffl_sc7a20_set_i2c_addr(&device, FFL_SC7A20_DEFAULT_ADDR7_L) == -EBUSY);
        assert(ffl_sc7a20_cancel_async(&device) == 0);
        assert(state.calls == 0);

        bus.cancel_status = -EIO;
        assert(ffl_sc7a20_read_raw_async(&device, fake_sc7a20_read_done, &state) == 0);
        assert(ffl_sc7a20_cancel_async(&device) == -EIO);
        assert(ffl_sc7a20_set_i2c_addr(&device, FFL_SC7A20_DEFAULT_ADDR7_L) == -EBUSY);
        bus.cancel_status = 0;
        fake_sc7a20_complete(&bus, 0);
        assert(state.calls == 1);

        bus.defer_callback = 0;
        state.device = &device;
        state.cancel_from_callback = 1;
        state.cancel_status = 0;
        assert(ffl_sc7a20_read_raw_async(&device, fake_sc7a20_read_done, &state) == 0);
        assert(state.calls == 2);
        assert(state.status == 0);
        assert(state.raw.x == 0x0100);
        assert(state.cancel_status == -EALREADY);
    }
#endif

    assert(ffl_sc7a20_soft_reset(&device) == 0);
    assert(ffl_sc7a20_deinit(&device) == 0);
    assert(!device.core.initialized);

    {
        ffl_sc7a20_device_t unsupported = {0};
        ffl_transport_t spi_transport = transport;

        spi_transport.endpoint = ffl_endpoint_spi();
        assert(ffl_sc7a20_bind(&unsupported,
                               &spi_transport,
                               &g_fake_sc7a20_time_ops,
                               NULL) == -ENOTSUP);
    }
    fake_sc7a20_complete(&bus, 0);
    return 0;
}
