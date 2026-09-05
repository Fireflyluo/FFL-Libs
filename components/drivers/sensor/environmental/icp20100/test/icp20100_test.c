#include "ffl/icp20100.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t registers[256];
    ffl_transport_t transport;
    uint8_t expected_addr7;
    uint8_t last_addr7;
    uint8_t transfer_count;
    uint32_t delay_total_us;
    int transfer_status;
    bool address_mismatch;
    ffl_icp20100_device_t *reentry_device;
    int reentry_status;
    bool change_address_on_xfer;
} fake_i2c_t;

static int fake_i2c_xfer(void *ctx,
                         const ffl_endpoint_t *endpoint,
                         const ffl_xfer_msg_t *msgs,
                         uint8_t count,
                         ffl_xfer_done_fn done,
                         void *user)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;
    uint8_t reg;
    uint16_t index;

    (void)user;

    if (bus == NULL || endpoint == NULL || msgs == NULL || done != NULL || count != 2u ||
        endpoint->kind != FFL_ENDPOINT_I2C_7BIT || msgs[0].buf == NULL ||
        msgs[0].len != 1u || msgs[0].flags != FFL_XFER_MSG_WRITE) {
        return -EINVAL;
    }
    if (bus->transfer_status != 0) {
        return bus->transfer_status;
    }

    bus->last_addr7 = endpoint->value.i2c.addr7;
    bus->transfer_count++;
    if (bus->last_addr7 != bus->expected_addr7) {
        bus->address_mismatch = true;
        return -EINVAL;
    }
    if (bus->change_address_on_xfer && bus->reentry_device != NULL) {
        bus->change_address_on_xfer = false;
        bus->reentry_status = ffl_icp20100_set_i2c_addr(
            bus->reentry_device,
            ICP20100_I2C_ADDR_AD0_HIGH);
    }

    reg = msgs[0].buf[0];
    if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
        if (msgs[1].flags != (FFL_XFER_MSG_READ | FFL_XFER_MSG_STOP) || msgs[1].buf == NULL) {
            return -EINVAL;
        }
        for (index = 0u; index < msgs[1].len; ++index) {
            msgs[1].buf[index] = bus->registers[(uint8_t)(reg + index)];
        }
        return 0;
    }
    if (msgs[1].flags != (FFL_XFER_MSG_WRITE | FFL_XFER_MSG_STOP) || msgs[1].buf == NULL) {
        return -EINVAL;
    }
    for (index = 0u; index < msgs[1].len; ++index) {
        bus->registers[(uint8_t)(reg + index)] = msgs[1].buf[index];
    }
    return 0;
}

static void fake_delay_us(void *ctx, uint32_t us)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;

    if (bus != NULL) {
        bus->delay_total_us += us;
    }
}

static const ffl_transport_ops_t g_transport_ops = {
    .xfer = fake_i2c_xfer,
    .cancel = NULL,
};

static const ffl_time_ops_t g_time_ops = {
    .delay_ms = NULL,
    .delay_us = fake_delay_us,
    .now_us = NULL,
};

static int bind_device(ffl_icp20100_device_t *device, fake_i2c_t *bus, uint8_t addr7)
{
    bus->transport = (ffl_transport_t){
        .ops = &g_transport_ops,
        .ctx = bus,
        .endpoint = {0},
    };

    bus->transport.endpoint = ffl_endpoint_i2c7(addr7);
    return ffl_icp20100_bind(device, &bus->transport, &g_time_ops, bus);
}

static int test_init_forwards_configured_address(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_HIGH,
    };
    ffl_icp20100_device_t device = {0};
    ffl_icp20100_config_t config;

    bus.registers[ICP20100_REG_DEVICE_ID] = ICP20100_DEVICE_ID_DEFAULT;
    bus.registers[ICP20100_REG_VERSION] = 0x11u;
    ffl_icp20100_config_init(&config);
    config.addr = ICP20100_I2C_ADDR_AD0_HIGH;

    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_LOW) != 0 ||
        ffl_icp20100_init(&device, &config) != 0) {
        return 1;
    }

    return (!bus.address_mismatch && bus.last_addr7 == ICP20100_I2C_ADDR_AD0_HIGH &&
            bus.transfer_count >= 5u && bus.delay_total_us >= 2050u && device.initialized &&
            device.chip_id == ICP20100_DEVICE_ID_DEFAULT && device.version == 0x11u &&
            bus.registers[ICP20100_REG_MODE_SELECT] == 0x08u) ? 0 : 1;
}

static int test_init_uses_bound_endpoint_without_config(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_HIGH,
    };
    ffl_icp20100_device_t device = {0};

    bus.registers[ICP20100_REG_DEVICE_ID] = ICP20100_DEVICE_ID_DEFAULT;
    bus.registers[ICP20100_REG_VERSION] = 0x02u;
    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_HIGH) != 0 ||
        ffl_icp20100_init(&device, NULL) != 0) {
        return 1;
    }

    return (!bus.address_mismatch && device.addr == ICP20100_I2C_ADDR_AD0_HIGH) ? 0 : 1;
}

static int test_rejects_unsupported_configurations(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_LOW,
    };
    ffl_icp20100_device_t device = {0};
    ffl_icp20100_config_t config;

    bus.registers[ICP20100_REG_DEVICE_ID] = ICP20100_DEVICE_ID_DEFAULT;
    bus.registers[ICP20100_REG_VERSION] = 0x01u;
    ffl_icp20100_config_init(&config);
    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_LOW) != 0 ||
        ffl_icp20100_init(&device, &config) != 0) {
        return 1;
    }

    config = device.cfg;
    config.fifo_mode = FFL_ICP20100_FIFO_TEMP_ONLY;
    if (ffl_icp20100_configure(&device, &config) != -EINVAL) {
        return 1;
    }
    config = device.cfg;
    config.meas_mode = FFL_ICP20100_MEAS_MODE_FORCED;
    if (ffl_icp20100_configure(&device, &config) != -EINVAL) {
        return 1;
    }
    config = device.cfg;
    config.addr = ICP20100_I2C_ADDR_AD0_HIGH;
    return ffl_icp20100_configure(&device, &config) == -EINVAL ? 0 : 1;
}

static int test_reads_signed_fifo_sample(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_LOW,
    };
    ffl_icp20100_device_t device = {0};
    ffl_icp20100_raw_sample_t raw;
    ffl_icp20100_sample_t sample;

    bus.registers[ICP20100_REG_DEVICE_ID] = ICP20100_DEVICE_ID_DEFAULT;
    bus.registers[ICP20100_REG_VERSION] = 0x01u;
    bus.registers[ICP20100_REG_FIFO_FILL] = 1u;
    bus.registers[ICP20100_REG_FIFO_BASE + 0u] = 0xFFu;
    bus.registers[ICP20100_REG_FIFO_BASE + 1u] = 0xFFu;
    bus.registers[ICP20100_REG_FIFO_BASE + 2u] = 0x0Fu;
    bus.registers[ICP20100_REG_FIFO_BASE + 3u] = 0x00u;
    bus.registers[ICP20100_REG_FIFO_BASE + 4u] = 0x00u;
    bus.registers[ICP20100_REG_FIFO_BASE + 5u] = 0x00u;

    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_LOW) != 0 ||
        ffl_icp20100_init(&device, NULL) != 0 ||
        ffl_icp20100_read_raw(&device, &raw) != 0 || raw.pressure_raw != -1 ||
        raw.temperature_raw != 0 || ffl_icp20100_read_sample(&device, &sample) != 0) {
        return 1;
    }

    return (sample.pressure_kpa > 69.99f && sample.pressure_kpa < 70.0f &&
            sample.temperature_c == 25.0f) ? 0 : 1;
}

static int test_maps_positive_transport_status(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_LOW,
        .transfer_status = 1,
    };
    ffl_icp20100_device_t device = {0};

    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_LOW) != 0) {
        return 1;
    }
    return ffl_icp20100_init(&device, NULL) == -EIO && !device.initialized ? 0 : 1;
}

static int test_serializes_init_against_address_change(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = ICP20100_I2C_ADDR_AD0_LOW,
        .change_address_on_xfer = true,
    };
    ffl_icp20100_device_t device = {0};

    bus.registers[ICP20100_REG_DEVICE_ID] = ICP20100_DEVICE_ID_DEFAULT;
    bus.registers[ICP20100_REG_VERSION] = 0x01u;
    bus.reentry_device = &device;
    if (bind_device(&device, &bus, ICP20100_I2C_ADDR_AD0_LOW) != 0 ||
        ffl_icp20100_init(&device, NULL) != 0) {
        return 1;
    }

    return (!bus.address_mismatch && bus.reentry_status == -EBUSY &&
            device.addr == ICP20100_I2C_ADDR_AD0_LOW) ? 0 : 1;
}

int main(void)
{
    const int failed = test_init_forwards_configured_address() +
                       test_init_uses_bound_endpoint_without_config() +
                       test_rejects_unsupported_configurations() +
                       test_reads_signed_fifo_sample() +
                       test_maps_positive_transport_status() +
                       test_serializes_init_against_address_change();

    if (failed != 0) {
        fprintf(stderr, "icp20100 tests failed: %d\n", failed);
    }
    return failed;
}
