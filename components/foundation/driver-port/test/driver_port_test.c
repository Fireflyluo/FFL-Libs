#include "ffl/driver_port.h"

#include <errno.h>

typedef struct {
    ffl_gpio_level_t level;
    uint32_t last_line;
    uint32_t delay_ms_total;
    uint32_t delay_us_total;
    uint8_t irq_calls;
} fake_port_t;

static int fake_xfer(void *ctx,
                     const ffl_endpoint_t *endpoint,
                     const ffl_xfer_msg_t *msgs,
                     uint8_t count,
                     ffl_xfer_done_fn done,
                     void *user)
{
    (void)ctx;
    (void)endpoint;
    (void)msgs;
    (void)count;
    (void)done;
    (void)user;
    return 0;
}

static int fake_gpio_write(void *ctx, uint32_t line, ffl_gpio_level_t level)
{
    fake_port_t *port = (fake_port_t *)ctx;

    port->last_line = line;
    port->level = level;
    return 0;
}

static int fake_gpio_read(void *ctx, uint32_t line, ffl_gpio_level_t *out_level)
{
    fake_port_t *port = (fake_port_t *)ctx;

    port->last_line = line;
    *out_level = port->level;
    return 0;
}

static int fake_irq_call(void *ctx, uint32_t line)
{
    fake_port_t *port = (fake_port_t *)ctx;

    port->last_line = line;
    port->irq_calls++;
    return 0;
}

static void fake_delay_ms(void *ctx, uint32_t ms)
{
    ((fake_port_t *)ctx)->delay_ms_total += ms;
}

static void fake_delay_us(void *ctx, uint32_t us)
{
    ((fake_port_t *)ctx)->delay_us_total += us;
}

int main(void)
{
    static const ffl_transport_ops_t transport_ops = {.xfer = fake_xfer, .cancel = 0};
    static const ffl_gpio_ops_t gpio_ops = {.write = fake_gpio_write, .read = fake_gpio_read};
    static const ffl_irq_ops_t irq_ops = {
        .enable = fake_irq_call,
        .disable = fake_irq_call,
        .ack = fake_irq_call,
    };
    static const ffl_time_ops_t time_ops = {
        .delay_ms = fake_delay_ms,
        .delay_us = fake_delay_us,
        .now_us = 0,
    };
    uint8_t byte = 0u;
    ffl_xfer_msg_t message = {.buf = &byte, .len = 1u, .flags = FFL_XFER_MSG_WRITE};
    ffl_transport_t transport = {.ops = &transport_ops, .ctx = 0, .endpoint = ffl_endpoint_spi()};
    ffl_gpio_t gpio;
    ffl_irq_t irq;
    ffl_gpio_level_t level;
    fake_port_t port = {0};

    gpio.ops = &gpio_ops;
    gpio.ctx = &port;
    gpio.line = 7u;
    irq.ops = &irq_ops;
    irq.ctx = &port;
    irq.line = 9u;

    if (!ffl_endpoint_is_valid(&transport.endpoint) ||
        ffl_transport_xfer(&transport, &transport.endpoint, &message, 1u, 0, 0) != 0 ||
        ffl_gpio_write(&gpio, FFL_GPIO_HIGH) != 0 || ffl_gpio_read(&gpio, &level) != 0 ||
        level != FFL_GPIO_HIGH || port.last_line != 7u ||
        ffl_irq_enable(&irq) != 0 || ffl_irq_disable(&irq) != 0 || ffl_irq_ack(&irq) != 0 ||
        port.irq_calls != 3u || ffl_time_delay_ms(&time_ops, &port, 2u) != 0 ||
        ffl_time_delay_us(&time_ops, &port, 3u) != 0 || port.delay_ms_total != 2u ||
        port.delay_us_total != 3u) {
        return 1;
    }

    message.flags = (uint8_t)(FFL_XFER_MSG_WRITE | FFL_XFER_MSG_READ);
    return ffl_transport_xfer(&transport, &transport.endpoint, &message, 1u, 0, 0) == -EINVAL ? 0 : 1;
}
