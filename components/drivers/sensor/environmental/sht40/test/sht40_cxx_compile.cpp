#include "ffl/sht40.h"

struct fake_bus {
    unsigned writes;
};

static int fake_xfer(void *ctx,
                     const ffl_endpoint_t *endpoint,
                     const ffl_xfer_msg_t *msgs,
                     uint8_t count,
                     ffl_xfer_done_fn done,
                     void *user)
{
    fake_bus *bus = static_cast<fake_bus *>(ctx);

    (void)user;
    if (bus == nullptr || endpoint == nullptr || msgs == nullptr || count != 1u ||
        done != nullptr || endpoint->kind != FFL_ENDPOINT_I2C_7BIT) {
        return -1;
    }
    if ((msgs[0].flags & FFL_XFER_MSG_WRITE) != 0u) {
        ++bus->writes;
        return 0;
    }
    if ((msgs[0].flags & FFL_XFER_MSG_READ) != 0u && msgs[0].len == 6u) {
        const uint8_t response[6] = {0xBEu, 0xEFu, 0x92u, 0xBEu, 0xEFu, 0x92u};
        for (uint8_t index = 0u; index < 6u; ++index) {
            msgs[0].buf[index] = response[index];
        }
        return 0;
    }
    return -1;
}

static void fake_delay_ms(void *ctx, uint32_t ms)
{
    (void)ctx;
    (void)ms;
}

int main()
{
    static const ffl_transport_ops_t ops = {fake_xfer, nullptr};
    static const ffl_time_ops_t time_ops = {fake_delay_ms, nullptr, nullptr};
    fake_bus bus = {};
    ffl_sht40_device_t device = {};
    ffl_sht40_config_t config = {};
    ffl_sht40_sample_t sample = {};
    ffl_transport_t transport = {};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    transport.ops = &ops;
    transport.ctx = &bus;
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x44u;

    if (ffl_sht40_bind(&device, &transport, &time_ops, nullptr) != 0 ||
        ffl_sht40_init(&device, &config) != 0 ||
        ffl_sht40_read_sample(&device, FFL_SHT40_PRECISION_HIGH, &sample) != 0) {
        return 1;
    }
    return (bus.writes == 2u && sample.humidity_rh >= 0.0f) ? 0 : 1;
}
