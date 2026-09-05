#include "ffl/icm42688p.h"

#include <errno.h>
#include <stdio.h>

typedef struct {
    uint8_t registers[5][256];
    ffl_transport_t transport;
    uint8_t selected_bank;
    uint8_t expected_addr;
    uint8_t last_addr;
    uint8_t fail_bank;
    uint8_t fail_reg;
    int transfer_status;
    uint32_t delay_ms;
    bool reenter_on_transfer;
    ffl_icm42688p_device_t *reentry_device;
    int reentry_status;
} fake_i2c_t;

static int fake_xfer(void *ctx, const ffl_endpoint_t *endpoint,
                     const ffl_xfer_msg_t *msgs, uint8_t count,
                     ffl_xfer_done_fn done, void *user)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;
    uint8_t reg;
    uint16_t index;

    (void)user;
    if (bus == NULL || endpoint == NULL || msgs == NULL || done != NULL ||
        count != 2u || endpoint->kind != FFL_ENDPOINT_I2C_7BIT ||
        msgs[0].buf == NULL || msgs[0].len != 1u ||
        msgs[0].flags != FFL_XFER_MSG_WRITE || msgs[1].buf == NULL) {
        return -EINVAL;
    }
    bus->last_addr = endpoint->value.i2c.addr7;
    if (bus->last_addr != bus->expected_addr || bus->transfer_status != 0) {
        return bus->transfer_status != 0 ? bus->transfer_status : -EIO;
    }
    if (bus->reenter_on_transfer && bus->reentry_device != NULL) {
        bus->reenter_on_transfer = false;
        bus->reentry_status = ffl_icm42688p_set_i2c_addr(
            bus->reentry_device, (uint8_t)(bus->expected_addr ^ 1u));
    }

    reg = msgs[0].buf[0];
    if ((msgs[1].flags & FFL_XFER_MSG_READ) != 0u) {
        if (msgs[1].flags != (FFL_XFER_MSG_READ | FFL_XFER_MSG_RESTART |
                              FFL_XFER_MSG_STOP)) {
            return -EINVAL;
        }
        for (index = 0u; index < msgs[1].len; ++index) {
            msgs[1].buf[index] = bus->registers[bus->selected_bank]
                [(uint8_t)(reg + index)];
        }
        return 0;
    }
    if (msgs[1].flags != (FFL_XFER_MSG_WRITE | FFL_XFER_MSG_STOP)) {
        return -EINVAL;
    }
    if (bus->fail_reg != 0u && bus->selected_bank == bus->fail_bank &&
        reg == bus->fail_reg) {
        return -EIO;
    }
    if (reg == ICM42688_REG_BANK_SEL && msgs[1].len == 1u) {
        bus->selected_bank = (uint8_t)(msgs[1].buf[0] & 0x07u);
    } else {
        for (index = 0u; index < msgs[1].len; ++index) {
            bus->registers[bus->selected_bank][(uint8_t)(reg + index)] =
                msgs[1].buf[index];
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

static const ffl_time_ops_t fake_time_ops = {
    .delay_ms = fake_delay_ms,
    .delay_us = NULL,
    .now_us = NULL,
};

static int bind_device(ffl_icm42688p_device_t *device, fake_i2c_t *bus,
                       uint8_t addr)
{
    bus->transport = (ffl_transport_t){
        .ops = &fake_ops,
        .ctx = bus,
        .endpoint = ffl_endpoint_i2c7(addr),
    };
    return ffl_icm42688p_bind(device, &bus->transport, &fake_time_ops, bus);
}

static int init_device(ffl_icm42688p_device_t *device, fake_i2c_t *bus,
                       uint8_t addr)
{
    bus->expected_addr = addr;
    bus->selected_bank = ICM42688_BANK0;
    bus->registers[ICM42688_BANK0][ICM42688_REG_WHO_AM_I] =
        ICM42688P_WHO_AM_I_ID;
    if (bind_device(device, bus, addr) != 0) {
        return -1;
    }
    return ffl_icm42688p_init(device, NULL);
}

static int test_init_bank_and_sample(void)
{
    fake_i2c_t bus = {0};
    ffl_icm42688p_device_t device = {0};
    ffl_icm42688p_raw_sample_t raw;
    ffl_icm42688p_sample_t sample;
    uint8_t bank_value;

    if (init_device(&device, &bus, ICM42688P_I2C_ADDR_0) != 0 ||
        !device.initialized || bus.last_addr != ICM42688P_I2C_ADDR_0 ||
        bus.delay_ms != 57u || bus.selected_bank != ICM42688_BANK0 ||
        bus.registers[ICM42688_BANK0][ICM42688_REG_INTF_CONFIG1] != 0x01u ||
        bus.registers[ICM42688_BANK0][ICM42688_REG_PWR_MGMT0] != 0x0Fu ||
        bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_CONFIG0] != 0x48u ||
        bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_CONFIG0] != 0x48u) {
        return 1;
    }
    bus.registers[ICM42688_BANK1][ICM42688_REG_SENSOR_CONFIG0] = 0xA5u;
    if (imu_icm42688p_read_reg(&device, ICM42688_BANK1,
                               ICM42688_REG_SENSOR_CONFIG0, &bank_value, 1u) != 0 ||
        bank_value != 0xA5u || bus.selected_bank != ICM42688_BANK1) {
        return 1;
    }
    bus.registers[ICM42688_BANK0][ICM42688_REG_TEMP_DATA1] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_TEMP_DATA0] = 132u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_X1] = 0x20u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_X0] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_Y1] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_Y0] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_Z1] = 0xE0u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_ACCEL_DATA_Z0] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_X1] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_X0] = 0x42u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_Y1] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_Y0] = 0x00u;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_Z1] = 0xFFu;
    bus.registers[ICM42688_BANK0][ICM42688_REG_GYRO_DATA_Z0] = 0xBEu;
    if (ffl_icm42688p_read_raw(&device, &raw) != 0 || raw.temperature != 132 ||
        raw.accel[0] != 0x2000 || raw.accel[2] != -8192 ||
        raw.gyro[0] != 66 || raw.gyro[2] != -66 ||
        bus.selected_bank != ICM42688_BANK0 ||
        ffl_icm42688p_read_sample(&device, &sample) != 0 ||
        sample.accel_mps2[0] < 9.80f || sample.accel_mps2[0] > 9.81f ||
        sample.gyro_rads[0] < 0.0175f || sample.gyro_rads[0] > 0.0177f ||
        sample.temperature_c < 25.99f || sample.temperature_c > 26.01f) {
        return 1;
    }
    return 0;
}

static int test_address_and_serialization(void)
{
    fake_i2c_t bus = {0};
    ffl_icm42688p_device_t device = {0};

    bus.reenter_on_transfer = true;
    bus.reentry_device = &device;
    bus.expected_addr = ICM42688P_I2C_ADDR_1;
    bus.registers[ICM42688_BANK0][ICM42688_REG_WHO_AM_I] =
        ICM42688P_WHO_AM_I_ID;
    if (bind_device(&device, &bus, ICM42688P_I2C_ADDR_0) != 0 ||
        ffl_icm42688p_set_i2c_addr(&device, ICM42688P_I2C_ADDR_1) != 0 ||
        ffl_icm42688p_init(&device, NULL) != 0) {
        return 1;
    }
    return bus.reentry_status == -EBUSY && bus.last_addr == ICM42688P_I2C_ADDR_1 &&
           device.addr == ICM42688P_I2C_ADDR_1 &&
           ffl_icm42688p_set_i2c_addr(&device, ICM42688P_I2C_ADDR_0) == -EBUSY ? 0 : 1;
}

static int test_invalid_config_and_rollback(void)
{
    fake_i2c_t bus = {0};
    ffl_icm42688p_device_t device = {0};
    ffl_icm42688p_config_t config;
    icm42688_accel_fs_t old_fs;

    if (init_device(&device, &bus, ICM42688P_I2C_ADDR_0) != 0) {
        return 1;
    }
    config = device.cfg;
    config.gyro_mode = FFL_ICM42688P_MODE_LOW_POWER;
    if (ffl_icm42688p_configure(&device, &config) != -EINVAL) {
        return 1;
    }
    config = device.cfg;
    config.gyro_odr = (ffl_icm42688p_odr_t)12;
    if (ffl_icm42688p_configure(&device, &config) != -EINVAL) {
        return 1;
    }
    config = device.cfg;
    config.accel_mode = FFL_ICM42688P_MODE_LOW_POWER;
    config.accel_odr = FFL_ICM42688P_ODR_1000HZ;
    if (ffl_icm42688p_configure(&device, &config) != -EINVAL) {
        return 1;
    }
    old_fs = device.cfg.accel_fs;
    config = device.cfg;
    config.accel_fs = FFL_ICM42688P_ACCEL_FS_2G;
    bus.fail_bank = ICM42688_BANK0;
    bus.fail_reg = ICM42688_REG_ACCEL_CONFIG0;
    return ffl_icm42688p_configure(&device, &config) == -EIO &&
                   !device.initialized && device.cfg.accel_fs == old_fs
               ? 0
               : 1;
}

static int test_invalid_address_and_uninitialized_read(void)
{
    fake_i2c_t bus = {0};
    ffl_icm42688p_device_t device = {0};
    ffl_icm42688p_raw_sample_t raw;
    ffl_transport_t transport;

    transport.ops = &fake_ops;
    transport.ctx = &bus;
    transport.endpoint = ffl_endpoint_i2c7(0u);
    return ffl_icm42688p_bind(&device, &transport, &fake_time_ops, &bus) == -EINVAL &&
                   ffl_icm42688p_read_raw(&device, &raw) == -ENODEV
               ? 0
               : 1;
}

int main(void)
{
    const int init_bank = test_init_bank_and_sample();
    const int address = test_address_and_serialization();
    const int rollback = test_invalid_config_and_rollback();
    const int invalid = test_invalid_address_and_uninitialized_read();
    const int failed = init_bank + address + rollback + invalid;
    if (failed != 0) {
        fprintf(stderr, "icm42688p tests failed: %d\n", failed);
    }
    return failed;
}
