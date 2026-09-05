#include "ffl/qmc5883p.h"

#include <errno.h>
#include <stdio.h>

typedef struct {
    uint8_t registers[256];
    ffl_transport_t transport;
    uint8_t expected_addr7;
    uint8_t last_addr7;
    uint8_t write_regs[16];
    uint8_t write_values[16];
    uint8_t write_count;
    int transfer_status;
    bool address_mismatch;
    bool change_address_on_xfer;
    ffl_qmc5883p_device_t *reentry_device;
    int reentry_status;
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
    if (bus->last_addr7 != bus->expected_addr7) {
        bus->address_mismatch = true;
        return -EINVAL;
    }
    if (bus->change_address_on_xfer && bus->reentry_device != NULL) {
        bus->change_address_on_xfer = false;
        bus->reentry_status = ffl_qmc5883p_set_i2c_addr(
            bus->reentry_device,
            FFL_QMC5883P_DEFAULT_ADDR7 + 1u);
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
    if (msgs[1].len == 1u && bus->write_count < (uint8_t)(sizeof(bus->write_regs))) {
        bus->write_regs[bus->write_count] = reg;
        bus->write_values[bus->write_count] = msgs[1].buf[0];
        bus->write_count++;
    }
    return 0;
}

static const ffl_transport_ops_t g_transport_ops = {
    .xfer = fake_i2c_xfer,
    .cancel = NULL,
};

static int bind_device(ffl_qmc5883p_device_t *device, fake_i2c_t *bus, uint8_t addr7)
{
    bus->transport = (ffl_transport_t){
        .ops = &g_transport_ops,
        .ctx = bus,
        .endpoint = {0},
    };
    bus->transport.endpoint = ffl_endpoint_i2c7(addr7);
    return ffl_qmc5883p_bind(device, &bus->transport);
}

static int init_device(ffl_qmc5883p_device_t *device, fake_i2c_t *bus, uint8_t addr7)
{
    bus->expected_addr7 = addr7;
    bus->registers[QMC5883P_REG_CHIP_ID] = QMC5883P_CHIP_ID_DEFAULT;
    return bind_device(device, bus, addr7) == 0 && ffl_qmc5883p_init(device, NULL) == 0 ? 0 : 1;
}

static int test_initializes_8g_with_bound_address(void)
{
    fake_i2c_t bus = {0};
    ffl_qmc5883p_device_t device = {0};

    if (init_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7 + 1u) != 0) {
        return 1;
    }
    return (!bus.address_mismatch && bus.last_addr7 == FFL_QMC5883P_DEFAULT_ADDR7 + 1u &&
            device.cfg.range == FFL_QMC5883P_RANGE_8G &&
            bus.registers[QMC5883P_REG_CONTROL_2] == 0x08u &&
            bus.registers[QMC5883P_REG_CONTROL_1] == 0x0Bu) ? 0 : 1;
}

static int test_rejects_no_data_and_overflow(void)
{
    fake_i2c_t bus = {0};
    ffl_qmc5883p_device_t device = {0};
    ffl_qmc5883p_vec3i16_t raw;

    if (init_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0) {
        return 1;
    }
    bus.registers[QMC5883P_REG_STATUS] = 0u;
    if (ffl_qmc5883p_read_raw(&device, &raw) != -EAGAIN) {
        return 1;
    }
    bus.registers[QMC5883P_REG_STATUS] = QMC5883P_STATUS_DRDY_MASK | QMC5883P_STATUS_OVFL_MASK;
    return ffl_qmc5883p_read_raw(&device, &raw) == -EIO ? 0 : 1;
}

static int test_converts_8g_data_to_microtesla(void)
{
    fake_i2c_t bus = {0};
    ffl_qmc5883p_device_t device = {0};
    ffl_qmc5883p_vec3f_t sample;

    if (init_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0) {
        return 1;
    }
    bus.registers[QMC5883P_REG_STATUS] = QMC5883P_STATUS_DRDY_MASK;
    bus.registers[QMC5883P_REG_XOUT_L + 0u] = 0xA6u;
    bus.registers[QMC5883P_REG_XOUT_L + 1u] = 0x0Eu;
    bus.registers[QMC5883P_REG_XOUT_L + 2u] = 0x2Du;
    bus.registers[QMC5883P_REG_XOUT_L + 3u] = 0xF8u;
    bus.registers[QMC5883P_REG_XOUT_L + 4u] = 0u;
    bus.registers[QMC5883P_REG_XOUT_L + 5u] = 0u;

    if (ffl_qmc5883p_read_ut(&device, &sample) != 0) {
        return 1;
    }
    return (sample.x > 99.9f && sample.x < 100.1f &&
            sample.y < -53.3f && sample.y > -53.5f && sample.z == 0.0f) ? 0 : 1;
}

static int test_suspends_before_runtime_reconfigure(void)
{
    fake_i2c_t bus = {0};
    ffl_qmc5883p_device_t device = {0};
    ffl_qmc5883p_config_t config;

    if (init_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0) {
        return 1;
    }
    bus.write_count = 0u;
    config = device.cfg;
    config.range = FFL_QMC5883P_RANGE_2G;
    if (ffl_qmc5883p_configure(&device, &config) != 0) {
        return 1;
    }
    return (bus.write_count == 3u && bus.write_regs[0] == QMC5883P_REG_CONTROL_1 &&
            bus.write_values[0] == QMC5883P_MODE_SUSPEND &&
            bus.write_regs[1] == QMC5883P_REG_CONTROL_2 && bus.write_values[1] == 0x0Cu &&
            bus.write_regs[2] == QMC5883P_REG_CONTROL_1 && bus.write_values[2] == 0x0Bu) ? 0 : 1;
}

static int test_requires_reinit_after_soft_reset(void)
{
    fake_i2c_t bus = {0};
    ffl_qmc5883p_device_t device = {0};
    ffl_qmc5883p_vec3i16_t raw;

    if (init_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0 ||
        ffl_qmc5883p_soft_reset(&device) != 0 || device.initialized ||
        ffl_qmc5883p_read_raw(&device, &raw) != -ENODEV ||
        ffl_qmc5883p_init(&device, NULL) != 0 || !device.initialized) {
        return 1;
    }
    return 0;
}

static int test_serializes_init_against_address_change(void)
{
    fake_i2c_t bus = {
        .change_address_on_xfer = true,
    };
    ffl_qmc5883p_device_t device = {0};

    bus.expected_addr7 = FFL_QMC5883P_DEFAULT_ADDR7;
    bus.registers[QMC5883P_REG_CHIP_ID] = QMC5883P_CHIP_ID_DEFAULT;
    bus.reentry_device = &device;
    if (bind_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0 ||
        ffl_qmc5883p_init(&device, NULL) != 0) {
        return 1;
    }
    return (!bus.address_mismatch && bus.reentry_status == -EBUSY &&
            device.addr == FFL_QMC5883P_DEFAULT_ADDR7) ? 0 : 1;
}

static int test_maps_positive_transport_status(void)
{
    fake_i2c_t bus = {
        .expected_addr7 = FFL_QMC5883P_DEFAULT_ADDR7,
        .transfer_status = 1,
    };
    ffl_qmc5883p_device_t device = {0};

    if (bind_device(&device, &bus, FFL_QMC5883P_DEFAULT_ADDR7) != 0) {
        return 1;
    }
    return ffl_qmc5883p_init(&device, NULL) == -EIO && !device.initialized ? 0 : 1;
}

int main(void)
{
    const int failed = test_initializes_8g_with_bound_address() +
                       test_rejects_no_data_and_overflow() +
                       test_converts_8g_data_to_microtesla() +
                       test_suspends_before_runtime_reconfigure() +
                       test_requires_reinit_after_soft_reset() +
                       test_serializes_init_against_address_change() +
                       test_maps_positive_transport_status();

    if (failed != 0) {
        fprintf(stderr, "qmc5883p tests failed: %d\n", failed);
    }
    return failed;
}
