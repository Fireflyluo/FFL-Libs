#include "ffl/sht40.h"

#include <errno.h>
#include <stdio.h>

typedef struct {
    uint8_t last_command;
    uint8_t last_addr7;
    uint32_t delay_total_ms;
    int transfer_status;
    bool corrupt_response;
    bool complete_inline;
} fake_i2c_t;

typedef struct {
    uint8_t count;
    int status;
    bool has_sample;
} async_result_t;

static int fake_i2c_xfer(void *ctx,
                         const ffl_endpoint_t *endpoint,
                         const ffl_xfer_msg_t *msgs,
                         uint8_t count,
                         ffl_xfer_done_fn done,
                         void *user)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;
    const ffl_xfer_msg_t *message;

    (void)user;

    if (bus == NULL || endpoint == NULL || msgs == NULL || count != 1u ||
        endpoint->kind != FFL_ENDPOINT_I2C_7BIT) {
        return -EINVAL;
    }

    if (bus->transfer_status != 0) {
        return bus->transfer_status;
    }

    bus->last_addr7 = endpoint->value.i2c.addr7;
    message = &msgs[0];
    if ((message->flags & FFL_XFER_MSG_WRITE) != 0u && message->len == 1u) {
        bus->last_command = message->buf[0];
    } else if ((message->flags & FFL_XFER_MSG_READ) != 0u && message->len == 6u) {
        static const uint8_t response[6] = {0xBEu, 0xEFu, 0x92u, 0xBEu, 0xEFu, 0x92u};

        for (uint8_t index = 0u; index < 6u; ++index) {
            message->buf[index] = response[index];
        }
        if (bus->corrupt_response) {
            message->buf[5] ^= 0x01u;
        }
    } else {
        return -EINVAL;
    }

    if (done != NULL) {
        if (!bus->complete_inline) {
            return -ENOTSUP;
        }
        done(user, 0);
    }
    return 0;
}

static void fake_delay_ms(void *ctx, uint32_t ms)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;

    if (bus != NULL) {
        bus->delay_total_ms += ms;
    }
}

static void fake_sample_done(void *user, const ffl_sht40_sample_t *sample, int status)
{
    async_result_t *result = (async_result_t *)user;

    if (result != NULL) {
        result->count++;
        result->status = status;
        result->has_sample = sample != NULL;
    }
}

static int test_accepts_valid_frame(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    static const ffl_time_ops_t time_ops = {
        .delay_ms = fake_delay_ms,
        .now_us = NULL
    };
    fake_i2c_t bus = {0};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht40_config_t config;
    ffl_sht40_device_t device = {0};
    ffl_sht40_sample_t sample;

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x44u;

    if (ffl_sht40_bind(&device, &transport, &time_ops, &bus) != 0 ||
        ffl_sht40_init(&device, &config) != 0 ||
        ffl_sht40_read_sample(&device, FFL_SHT40_PRECISION_HIGH, &sample) != 0) {
        return 1;
    }

    return (bus.last_addr7 == 0x44u && bus.last_command == 0xFDu &&
            bus.delay_total_ms >= 12u && sample.humidity_rh >= 0.0f) ? 0 : 1;
}

static int test_rejects_invalid_crc(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {0};
    ffl_transport_t transport = {0};
    ffl_sht40_config_t config;
    ffl_sht40_device_t device = {0};
    ffl_sht40_sample_t sample;

    bus.corrupt_response = true;
    transport.ops = &transport_ops;
    transport.ctx = &bus;
    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x44u;

    if (ffl_sht40_bind(&device, &transport, NULL, NULL) != 0 ||
        ffl_sht40_init(&device, &config) != 0 ||
        ffl_sht40_read_sample(&device, FFL_SHT40_PRECISION_HIGH, &sample) == 0) {
        return 1;
    }
    return 0;
}

static int test_propagates_transport_error(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {.transfer_status = -EIO};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht40_config_t config;
    ffl_sht40_device_t device = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x44u;
    return (ffl_sht40_bind(&device, &transport, NULL, NULL) == 0 &&
            ffl_sht40_init(&device, &config) == -EIO) ? 0 : 1;
}

static int test_rejects_invalid_transport(void)
{
    ffl_transport_t invalid_transport = {0};
    ffl_sht40_device_t device = {0};

    invalid_transport.endpoint = ffl_endpoint_i2c7(0x44u);
    return ffl_sht40_bind(&device, &invalid_transport, NULL, NULL) != 0 ? 0 : 1;
}

static int test_accepts_inline_async_completion(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {.complete_inline = true};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht40_config_t config;
    ffl_sht40_device_t device = {0};
    ffl_sht40_sample_t sample;
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht40_config_init(&config);
    config.i2c_addr7 = 0x44u;

    if (ffl_sht40_bind(&device, &transport, NULL, NULL) != 0 ||
        ffl_sht40_init(&device, &config) != 0 ||
        ffl_sht40_read_sample_async(&device,
                                    FFL_SHT40_PRECISION_HIGH,
                                    fake_sample_done,
                                    &result) != 0 ||
        ffl_sht40_read_sample(&device, FFL_SHT40_PRECISION_HIGH, &sample) != 0) {
        return 1;
    }

    return (result.count == 1u && result.status == 0 && result.has_sample) ? 0 : 1;
}

int main(void)
{
    const int failed = test_accepts_valid_frame() + test_rejects_invalid_crc() +
                       test_propagates_transport_error() + test_rejects_invalid_transport() +
                       test_accepts_inline_async_completion();

    if (failed != 0) {
        fprintf(stderr, "sht40 CRC tests failed: %d\n", failed);
    }
    return failed;
}
