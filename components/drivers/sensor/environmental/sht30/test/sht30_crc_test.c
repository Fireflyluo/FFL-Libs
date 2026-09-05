#include "ffl/sht30.h"

#include <errno.h>
#include <stdio.h>

typedef struct {
    uint8_t last_command[2];
    uint8_t last_addr7;
    uint32_t delay_total_ms;
    int transfer_status;
    bool corrupt_response;
    bool complete_inline;
    bool defer_async;
    ffl_xfer_done_fn pending_done;
    void *pending_user;
    void *cancel_device;
    uint8_t cancel_calls;
    int nested_cancel_status;
    void *delay_cancel_device;
    int delay_cancel_status;
    bool complete_on_cancel;
    int cancel_status;
} fake_i2c_t;

typedef struct {
    uint8_t count;
    int status;
    bool has_sample;
    void *reentry_device;
    int reentry_status;
    bool reenter_sync;
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

    if (bus == NULL || endpoint == NULL || msgs == NULL || count != 1u ||
        endpoint->kind != FFL_ENDPOINT_I2C_7BIT) {
        return -EINVAL;
    }
    if (bus->transfer_status != 0) {
        return bus->transfer_status;
    }

    bus->last_addr7 = endpoint->value.i2c.addr7;
    message = &msgs[0];
    if ((message->flags & FFL_XFER_MSG_WRITE) != 0u && message->len == 2u) {
        bus->last_command[0] = message->buf[0];
        bus->last_command[1] = message->buf[1];
    } else if ((message->flags & FFL_XFER_MSG_READ) != 0u && message->len == 6u) {
        static const uint8_t response[6] = {0xBEu, 0xEFu, 0x92u, 0xBEu, 0xEFu, 0x92u};

        for (uint8_t index = 0u; index < sizeof(response); ++index) {
            message->buf[index] = response[index];
        }
        if (bus->corrupt_response) {
            message->buf[5] ^= 0x01u;
        }
    } else {
        return -EINVAL;
    }

    if (done != NULL) {
        if (bus->complete_inline) {
            done(user, 0);
        } else if (bus->defer_async && bus->pending_done == NULL) {
            bus->pending_done = done;
            bus->pending_user = user;
        } else {
            return -ENOTSUP;
        }
    }
    return 0;
}

static int fake_i2c_complete_next(fake_i2c_t *bus)
{
    ffl_xfer_done_fn done;
    void *user;

    if (bus == NULL || bus->pending_done == NULL) {
        return -ENOENT;
    }

    done = bus->pending_done;
    user = bus->pending_user;
    bus->pending_done = NULL;
    bus->pending_user = NULL;
    done(user, 0);
    return 0;
}

static int fake_i2c_cancel(void *ctx)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;

    if (bus == NULL) {
        return -EINVAL;
    }

    bus->cancel_calls++;
    if (bus->cancel_device != NULL) {
        ffl_sht30_device_t *device = (ffl_sht30_device_t *)bus->cancel_device;

        bus->cancel_device = NULL;
        bus->nested_cancel_status = ffl_sht30_cancel_async(device);
    }
    if (bus->complete_on_cancel && bus->pending_done != NULL) {
        ffl_xfer_done_fn done = bus->pending_done;
        void *user = bus->pending_user;

        bus->pending_done = NULL;
        bus->pending_user = NULL;
        done(user, 0);
    }
    bus->pending_done = NULL;
    bus->pending_user = NULL;
    return bus->cancel_status;
}

static void fake_delay_ms(void *ctx, uint32_t ms)
{
    fake_i2c_t *bus = (fake_i2c_t *)ctx;

    if (bus != NULL) {
        bus->delay_total_ms += ms;
        if (bus->delay_cancel_device != NULL) {
            ffl_sht30_device_t *device = (ffl_sht30_device_t *)bus->delay_cancel_device;

            bus->delay_cancel_device = NULL;
            bus->delay_cancel_status = ffl_sht30_cancel_async(device);
        }
    }
}

static const ffl_time_ops_t g_time_ops = {
    .delay_ms = fake_delay_ms,
    .now_us = NULL
};

static void fake_sample_done(void *user, const ffl_sht30_sample_t *sample, int status)
{
    async_result_t *result = (async_result_t *)user;

    if (result != NULL) {
        result->count++;
        result->status = status;
        result->has_sample = sample != NULL;
        if (result->reenter_sync) {
            ffl_sht30_sample_t reentry_sample;

            result->reentry_status = ffl_sht30_read_sample(
                (ffl_sht30_device_t *)result->reentry_device,
                FFL_SHT30_REPEATABILITY_HIGH,
                &reentry_sample);
        }
    }
}

static int test_accepts_valid_frame(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {0};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);
    config.i2c_addr7 = 0x45u;

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0 ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) {
        return 1;
    }

    return (bus.last_addr7 == 0x45u && bus.last_command[0] == 0x24u &&
            bus.last_command[1] == 0x00u && bus.delay_total_ms >= 18u &&
            sample.humidity_rh >= 0.0f) ? 0 : 1;
}

static int test_rejects_invalid_crc(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {.corrupt_response = true};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);

    return (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) == 0 &&
            ffl_sht30_init(&device, &config) == 0 &&
            ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) ? 0 : 1;
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
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);
    return (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) == 0 &&
            ffl_sht30_init(&device, &config) == -EIO) ? 0 : 1;
}

static int test_rejects_invalid_transport(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    ffl_transport_t invalid_transport = {
        .ops = &transport_ops,
        .endpoint = {0}
    };
    ffl_sht30_device_t device = {0};

    invalid_transport.endpoint = ffl_endpoint_i2c7(0x80u);
    return ffl_sht30_bind(&device, &invalid_transport, &g_time_ops, NULL) != 0 ? 0 : 1;
}

static int test_rejects_missing_delay(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {0};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_device_t device = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    return ffl_sht30_bind(&device, &transport, NULL, NULL) != 0 ? 0 : 1;
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
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);
    result.reentry_device = &device;
    result.reenter_sync = true;

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0 ||
        ffl_sht30_read_sample_async(&device,
                                    FFL_SHT30_REPEATABILITY_HIGH,
                                    fake_sample_done,
                                    &result) != 0 ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) {
        return 1;
    }

    return (result.count == 1u && result.status == 0 && result.has_sample &&
            result.reentry_status == -EBUSY) ? 0 : 1;
}

static int test_serializes_pending_async(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = NULL
    };
    fake_i2c_t bus = {.defer_async = true};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0 ||
        ffl_sht30_read_sample_async(&device,
                                    FFL_SHT30_REPEATABILITY_HIGH,
                                    fake_sample_done,
                                    &result) != 0 ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != -EBUSY ||
        ffl_sht30_set_i2c_addr(&device, 0x45u) != -EBUSY ||
        ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != -EBUSY ||
        fake_i2c_complete_next(&bus) != 0 || fake_i2c_complete_next(&bus) != 0 ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) {
        return 1;
    }

    return (result.count == 1u && result.status == 0 && result.has_sample) ? 0 : 1;
}

static int test_serializes_concurrent_cancel(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = fake_i2c_cancel
    };
    fake_i2c_t bus = {.defer_async = true};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0 ||
        ffl_sht30_read_sample_async(&device,
                                    FFL_SHT30_REPEATABILITY_HIGH,
                                    fake_sample_done,
                                    &result) != 0) {
        return 1;
    }

    bus.cancel_device = &device;
    if (ffl_sht30_cancel_async(&device) != 0 || bus.cancel_calls != 1u ||
        bus.nested_cancel_status != -EBUSY || result.count != 0u ||
        ffl_sht30_cancel_async(&device) != -ENOENT ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) {
        return 1;
    }

    return 0;
}

static int test_rejects_cancel_during_callback_processing(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = fake_i2c_cancel
    };
    fake_i2c_t bus = {.complete_inline = true};
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0) {
        return 1;
    }

    bus.delay_cancel_device = &device;
    if (ffl_sht30_read_sample_async(&device,
                                    FFL_SHT30_REPEATABILITY_HIGH,
                                    fake_sample_done,
                                    &result) != 0) {
        return 1;
    }

    return (bus.delay_cancel_status == -EBUSY && bus.cancel_calls == 0u &&
            result.count == 1u && result.status == 0 && result.has_sample) ? 0 : 1;
}

static int test_consumes_callback_that_races_cancel_failure(void)
{
    static const ffl_transport_ops_t transport_ops = {
        .xfer = fake_i2c_xfer,
        .cancel = fake_i2c_cancel
    };
    fake_i2c_t bus = {
        .defer_async = true,
        .complete_on_cancel = true,
        .cancel_status = -EIO
    };
    ffl_transport_t transport = {
        .ops = &transport_ops,
        .ctx = &bus,
        .endpoint = {0}
    };
    ffl_sht30_config_t config;
    ffl_sht30_device_t device = {0};
    ffl_sht30_sample_t sample;
    async_result_t result = {0};

    transport.endpoint = ffl_endpoint_i2c7(0x44u);
    ffl_sht30_config_init(&config);

    if (ffl_sht30_bind(&device, &transport, &g_time_ops, &bus) != 0 ||
        ffl_sht30_init(&device, &config) != 0 ||
        ffl_sht30_read_sample_async(&device,
                                    FFL_SHT30_REPEATABILITY_HIGH,
                                    fake_sample_done,
                                    &result) != 0 ||
        ffl_sht30_cancel_async(&device) != 0 || bus.cancel_calls != 1u ||
        result.count != 0u ||
        ffl_sht30_read_sample(&device, FFL_SHT30_REPEATABILITY_HIGH, &sample) != 0) {
        return 1;
    }

    return 0;
}

int main(void)
{
    const int failed = test_accepts_valid_frame() + test_rejects_invalid_crc() +
                       test_propagates_transport_error() + test_rejects_invalid_transport() +
                       test_rejects_missing_delay() + test_accepts_inline_async_completion() +
                       test_serializes_pending_async() + test_serializes_concurrent_cancel() +
                       test_rejects_cancel_during_callback_processing() +
                       test_consumes_callback_that_races_cancel_failure();

    if (failed != 0) {
        fprintf(stderr, "sht30 CRC tests failed: %d\n", failed);
    }
    return failed;
}
