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
} fake_i2c_t;

static int fake_xfer(void *ctx, const ffl_endpoint_t *endpoint,
                     const ffl_xfer_msg_t *msgs, uint8_t count,
                     ffl_xfer_done_fn done, void *user)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;
    uint8_t reg;

    (void)user;
    if (done != NULL || bus == NULL || endpoint == NULL || count != 2u ||
        endpoint->kind != FFL_ENDPOINT_I2C_7BIT) {
        return -EINVAL;
    }
    bus->last_addr = endpoint->value.i2c.addr7;
    if (bus->last_addr != bus->expected_addr || msgs[0].len != 1u ||
        msgs[0].buf == NULL || msgs[1].buf == NULL) {
        return -EIO;
    }
    reg = msgs[0].buf[0];
    if (bus->transfer_status != 0) {
        return bus->transfer_status;
    }
    if (bus->fail_reg != 0u && reg == bus->fail_reg) {
        return -EIO;
    }
    if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
        memcpy(msgs[1].buf, &bus->registers[reg], msgs[1].len);
    } else {
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

static int setup(ffl_qmi8658a_device_t *device, fake_i2c_t *bus,
                 uint8_t addr, bool sync_sample)
{
    bus->transport = (ffl_transport_t){
        .ops = &fake_ops,
        .ctx = bus,
        .endpoint = { .kind = FFL_ENDPOINT_I2C_7BIT,
                      .value.i2c = { .addr7 = addr } },
    };
    ffl_qmi8658a_config_t config;

    bus->expected_addr = addr;
    bus->registers[QMI8658A_WHO_AM_I] = QMI8658A_ID;
    bus->registers[QMI8658A_STATUSINT] =
        QMI8658A_STATUSINT_AVAIL_MASK | QMI8658A_STATUSINT_LOCKED_MASK |
        QMI8658A_STATUSINT_CMD_DONE_MASK;
    bus->registers[QMI8658A_STATUS0] =
        QMI8658A_STATUS0_ADA_MASK | QMI8658A_STATUS0_GDA_MASK;
    if (ffl_qmi8658a_bind(device, &bus->transport, &fake_time_ops, bus) != 0) {
        return -1;
    }
    ffl_qmi8658a_config_init(&config);
    config.enable_sync_sample = sync_sample;
    return ffl_qmi8658a_init(device, &config);
}

static int test_init_and_read(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_raw_sample_t raw;

    if (setup(&device, &bus, 0x6Bu, true) != 0 ||
        bus.last_addr != 0x6Bu || bus.delay_ms != 20u ||
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

    if (setup(&device, &bus, 0x6Au, false) != 0) {
        return 1;
    }
    bus.registers[QMI8658A_STATUSINT] = 0u;
    bus.registers[QMI8658A_STATUS0] = 0u;
    if (ffl_qmi8658a_read_raw(&device, &(ffl_qmi8658a_raw_sample_t){0}) != -EAGAIN) {
        return 1;
    }
    ffl_qmi8658a_config_init(&config);
    config.enable_auto_increment = false;
    return ffl_qmi8658a_configure(&device, &config) == -EINVAL ? 0 : 1;
}

static int test_configuration_failure_keeps_cache(void)
{
    fake_i2c_t bus = {0};
    ffl_qmi8658a_device_t device = {0};
    ffl_qmi8658a_config_t config;
    qmi8658a_accel_fs_t old_fs;

    if (setup(&device, &bus, 0x6Au, false) != 0) {
        return 1;
    }
    old_fs = device.cfg.accel_fs;
    config = device.cfg;
    config.accel_fs = FFL_QMI8658A_ACCEL_FS_16G;
    bus.fail_reg = QMI8658A_CTRL7;
    {
        int rc = ffl_qmi8658a_configure(&device, &config);
        return rc == -EIO &&
           device.cfg.accel_fs == old_fs && !device.initialized ? 0 : 1;
    }
}

int main(void)
{
    const int a = test_init_and_read();
    const int b = test_status_and_config_validation();
    const int c = test_configuration_failure_keeps_cache();
    const int failed = a + b + c;
    if (failed != 0) {
        fprintf(stderr, "qmi8658a tests failed: %d (%d, %d, %d)\n", failed, a, b, c);
    }
    return failed;
}
