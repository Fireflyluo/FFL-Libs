#include "ffl/qmi8658a.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t registers[256];
    ffl_transport_t transport;
    uint8_t expected_addr;
    uint8_t last_addr;
    uint8_t fail_reg;
    int transfer_status;
    uint32_t delay_ms;
    uint32_t delay_us;
    bool reenter_on_transfer;
    ffl_qmi8658a_device_t *reentry_device;
    int reentry_status;
} fake_i2c_t;

static int fake_xfer(void *ctx, const ffl_endpoint_t *endpoint,
                     const ffl_xfer_msg_t *msgs, uint8_t count,
                     ffl_xfer_done_fn done, void *user)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;
    uint8_t reg;

    (void)user;
    if (bus == NULL || endpoint == NULL || msgs == NULL || done != NULL ||
        count != 2u || endpoint->kind != FFL_ENDPOINT_I2C_7BIT ||
        msgs[0].len != 1u || msgs[0].buf == NULL ||
        msgs[0].flags != FFL_XFER_MSG_WRITE || msgs[1].buf == NULL) {
        return -EINVAL;
    }
    bus->last_addr = endpoint->value.i2c.addr7;
    if (bus->last_addr != bus->expected_addr) {
        return -EIO;
    }
    if (bus->transfer_status != 0) {
        return bus->transfer_status;
    }
    if (bus->reenter_on_transfer && bus->reentry_device != NULL) {
        bus->reenter_on_transfer = false;
        bus->reentry_status = ffl_qmi8658a_set_i2c_addr(
            bus->reentry_device, (uint8_t)(bus->expected_addr ^ 1u));
    }

    reg = msgs[0].buf[0];
    if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
        if (msgs[1].flags != (FFL_XFER_MSG_READ | FFL_XFER_MSG_RESTART |
                              FFL_XFER_MSG_STOP)) {
            return -EINVAL;
        }
        memcpy(msgs[1].buf, &bus->registers[reg], msgs[1].len);
    } else {
        if (msgs[1].flags != (FFL_XFER_MSG_WRITE | FFL_XFER_MSG_STOP)) {
            return -EINVAL;
        }
        if (bus->fail_reg != 0u && reg == bus->fail_reg) {
            return -EIO;
        }
        memcpy(&bus->registers[reg], msgs[1].buf, msgs[1].len);
        if (reg == QMI8658A_CTRL9 && msgs[1].buf[0] == QMI8658A_CTRL9_CMD_AHB_CLOCK_GATING) {
            bus->registers[QMI8658A_STATUSINT] |= QMI8658A_STATUSINT_CMD_DONE_MASK;
        }
        if (reg == QMI8658A_CTRL9 && msgs[1].buf[0] == QMI8658A_CTRL9_CMD_ACK) {
            bus->registers[QMI8658A_STATUSINT] &=
                (uint8_t)~QMI8658A_STATUSINT_CMD_DONE_MASK;
        }
    }
    return 0;
}

static const ffl_transport_ops_t fake_ops = {
    .xfer = fake_xfer,
    .cancel = NULL,
};

static void fake_delay_ms(void *ctx, uint32_t delay_ms)
{
    ((fake_i2c_t *)ctx)->delay_ms += delay_ms;
}

static void fake_delay_us(void *ctx, uint32_t delay_us)
{
    ((fake_i2c_t *)ctx)->delay_us += delay_us;
}

static const ffl_time_ops_t fake_time_ops = {
    .delay_ms = fake_delay_ms,
    .delay_us = fake_delay_us,
    .now_us = NULL,
};

static int prepare(ffl_qmi8658a_device_t *device, fake_i2c_t *bus,
                   uint8_t addr)
{
    bus->transport = (ffl_transport_t){
        .ops = &fake_ops,
        .ctx = bus,
        .endpoint = ffl_endpoint_i2c7(addr),
    };
    bus->expected_addr = addr;
    bus->registers[QMI8658A_WHO_AM_I] = QMI8658A_ID;
    bus->registers[QMI8658A_STATUSINT] =
        QMI8658A_STATUSINT_AVAIL_MASK | QMI8658A_STATUSINT_LOCKED_MASK |
        QMI8658A_STATUSINT_CMD_DONE_MASK;
    bus->registers[QMI8658A_STATUS0] =
        QMI8658A_STATUS0_ADA_MASK | QMI8658A_STATUS0_GDA_MASK;
    return ffl_qmi8658a_bind(device, &bus->transport, &fake_time_ops, bus);
}

static int setup(ffl_qmi8658a_device_t *device, fake_i2c_t *bus,
                 uint8_t addr)
{
    ffl_qmi8658a_config_t config;

    if (prepare(device, bus, addr) != 0) {
        return -1;
    }
    ffl_qmi8658a_config_init(&config);
    return ffl_qmi8658a_init(device, &config);
}

static int test_init_and_read(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_raw_sample_t raw;

    if (setup(&device, &bus, 0x6Bu) != 0 || !device.initialized ||
        bus.last_addr != 0x6Bu || bus.delay_ms != 20u ||
        bus.registers[QMI8658A_CTRL1] != 0x40u ||
        bus.registers[QMI8658A_CTRL7] != 0x83u) {
        return 1;
    }
    bus.registers[QMI8658A_OUT_TEMP_L + 0u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 1u] = 0x01u;
    bus.registers[QMI8658A_OUT_TEMP_L + 2u] = 0x10u;
    bus.registers[QMI8658A_OUT_TEMP_L + 3u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 4u] = 0x20u;
    bus.registers[QMI8658A_OUT_TEMP_L + 5u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 6u] = 0x30u;
    bus.registers[QMI8658A_OUT_TEMP_L + 7u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 8u] = 0x40u;
    bus.registers[QMI8658A_OUT_TEMP_L + 9u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 10u] = 0x50u;
    bus.registers[QMI8658A_OUT_TEMP_L + 11u] = 0x00u;
    bus.registers[QMI8658A_OUT_TEMP_L + 12u] = 0x60u;
    bus.registers[QMI8658A_OUT_TEMP_L + 13u] = 0x00u;
    return ffl_qmi8658a_read_raw(&device, &raw) == 0 && raw.temperature == 0x0100 &&
           raw.accel[0] == 0x0010 && raw.accel[2] == 0x0030 &&
           raw.gyro[0] == 0x0040 && raw.gyro[2] == 0x0060 ? 0 : 1;
}

static int test_status_and_config_validation(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_config_t config;
    ffl_qmi8658a_raw_sample_t raw;

    if (setup(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    bus.registers[QMI8658A_STATUSINT] = 0u;
    if (ffl_qmi8658a_read_raw(&device, &raw) != -EAGAIN) {
        return 1;
    }
    bus.registers[QMI8658A_STATUSINT] = QMI8658A_STATUSINT_AVAIL_MASK;
    bus.delay_us = 0u;
    if (ffl_qmi8658a_read_raw(&device, &raw) != -EAGAIN || bus.delay_us == 0u) {
        return 1;
    }
    bus.registers[QMI8658A_STATUSINT] =
        QMI8658A_STATUSINT_AVAIL_MASK | QMI8658A_STATUSINT_LOCKED_MASK;
    bus.registers[QMI8658A_STATUS0] = QMI8658A_STATUS0_ADA_MASK;
    if (ffl_qmi8658a_read_raw(&device, &raw) != -EAGAIN) {
        return 1;
    }
    ffl_qmi8658a_config_init(&config);
    config.enable_auto_increment = false;
    return ffl_qmi8658a_configure(&device, &config) == -EINVAL &&
                   device.initialized
               ? 0
               : 1;
}

static int test_rejects_non_sync_init(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_config_t config;

    if (prepare(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    ffl_qmi8658a_config_init(&config);
    config.enable_sync_sample = false;
    return ffl_qmi8658a_init(&device, &config) == -EINVAL &&
                   !device.initialized
               ? 0
               : 1;
}

static int test_configuration_failure_keeps_cache(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_config_t config;
    qmi8658a_accel_fs_t old_fs;

    if (setup(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    old_fs = device.cfg.accel_fs;
    config = device.cfg;
    config.accel_fs = FFL_QMI8658A_ACCEL_FS_16G;
    bus.fail_reg = QMI8658A_CTRL7;
    return ffl_qmi8658a_configure(&device, &config) == -EIO &&
           device.cfg.accel_fs == old_fs && !device.initialized ? 0 : 1;
}

static int test_configuration_preserves_ctrl1(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_config_t config;

    if (setup(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    config = device.cfg;
    config.accel_fs = FFL_QMI8658A_ACCEL_FS_16G;
    if (ffl_qmi8658a_configure(&device, &config) != 0) {
        return 1;
    }
    return device.initialized && device.cfg.accel_fs == FFL_QMI8658A_ACCEL_FS_16G &&
                   bus.registers[QMI8658A_CTRL1] == 0x40u
               ? 0
               : 1;
}

static int test_serializes_address_change(void)
{
    fake_i2c_t bus = {
        .reenter_on_transfer = true,
    };
    ffl_qmi8658a_device_t device = {0};

    if (prepare(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    bus.reentry_device = &device;
    if (ffl_qmi8658a_init(&device, NULL) != 0) {
        return 1;
    }
    return bus.reentry_status == -EBUSY && device.addr == 0x6Au ? 0 : 1;
}

static int test_maps_positive_transport_status(void)
{
    fake_i2c_t bus = {
        .transfer_status = 1,
    };
    ffl_qmi8658a_device_t device = {0};

    if (prepare(&device, &bus, 0x6Au) != 0) {
        return 1;
    }
    return ffl_qmi8658a_init(&device, NULL) == -EIO && !device.initialized ? 0 : 1;
}

int main(void)
{
    const int failed = test_init_and_read() +
                       test_status_and_config_validation() +
                       test_rejects_non_sync_init() +
                       test_configuration_failure_keeps_cache() +
                       test_configuration_preserves_ctrl1() +
                       test_serializes_address_change() +
                       test_maps_positive_transport_status();
    if (failed != 0) {
        fprintf(stderr, "qmi8658a tests failed: %d\n", failed);
    }
    return failed;
}
