#ifndef FFL_DRIVER_PORT_H
#define FFL_DRIVER_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <errno.h>
#include <stdint.h>

#define FFL_XFER_MSG_WRITE   (1u << 0)
#define FFL_XFER_MSG_READ    (1u << 1)
#define FFL_XFER_MSG_STOP    (1u << 2)
#define FFL_XFER_MSG_RESTART (1u << 3)
#define FFL_XFER_MSG_DIRECTION_MASK (FFL_XFER_MSG_WRITE | FFL_XFER_MSG_READ)

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} ffl_xfer_msg_t;

typedef enum {
    FFL_ENDPOINT_NONE = 0,
    FFL_ENDPOINT_I2C_7BIT,
    FFL_ENDPOINT_SPI
} ffl_endpoint_kind_t;

typedef struct {
    ffl_endpoint_kind_t kind;
    union {
        struct {
            uint8_t addr7;
        } i2c;
        struct {
            uint8_t reserved;
        } spi;
    } value;
} ffl_endpoint_t;

typedef void (*ffl_xfer_done_fn)(void *user, int status);

typedef struct {
    int (*xfer)(void *ctx,
                const ffl_endpoint_t *endpoint,
                const ffl_xfer_msg_t *msgs,
                uint8_t count,
                ffl_xfer_done_fn done,
                void *user);
    int (*cancel)(void *ctx);
} ffl_transport_ops_t;

typedef struct {
    const ffl_transport_ops_t *ops;
    void *ctx;
    ffl_endpoint_t endpoint;
} ffl_transport_t;

typedef struct {
    void (*delay_ms)(void *ctx, uint32_t ms);
    void (*delay_us)(void *ctx, uint32_t us);
    uint32_t (*now_us)(void *ctx);
} ffl_time_ops_t;

typedef enum {
    FFL_GPIO_LOW = 0,
    FFL_GPIO_HIGH = 1
} ffl_gpio_level_t;

typedef struct {
    int (*write)(void *ctx, uint32_t line, ffl_gpio_level_t level);
    int (*read)(void *ctx, uint32_t line, ffl_gpio_level_t *out_level);
} ffl_gpio_ops_t;

typedef struct {
    const ffl_gpio_ops_t *ops;
    void *ctx;
    uint32_t line;
} ffl_gpio_t;

typedef struct {
    int (*enable)(void *ctx, uint32_t line);
    int (*disable)(void *ctx, uint32_t line);
    int (*ack)(void *ctx, uint32_t line);
} ffl_irq_ops_t;

typedef struct {
    const ffl_irq_ops_t *ops;
    void *ctx;
    uint32_t line;
} ffl_irq_t;

static inline ffl_endpoint_t ffl_endpoint_i2c7(uint8_t addr7)
{
    ffl_endpoint_t endpoint;

    endpoint.kind = FFL_ENDPOINT_I2C_7BIT;
    endpoint.value.i2c.addr7 = addr7;
    return endpoint;
}

static inline ffl_endpoint_t ffl_endpoint_spi(void)
{
    ffl_endpoint_t endpoint;

    endpoint.kind = FFL_ENDPOINT_SPI;
    endpoint.value.spi.reserved = 0u;
    return endpoint;
}

static inline bool ffl_endpoint_is_valid(const ffl_endpoint_t *endpoint)
{
    if (endpoint == 0) {
        return false;
    }

    if (endpoint->kind == FFL_ENDPOINT_I2C_7BIT) {
        return endpoint->value.i2c.addr7 <= 0x7Fu;
    }
    if (endpoint->kind == FFL_ENDPOINT_SPI) {
        return true;
    }
    return false;
}

static inline bool ffl_transport_is_valid(const ffl_transport_t *transport)
{
    return transport != 0 && transport->ops != 0 && transport->ops->xfer != 0;
}

static inline int ffl_transport_xfer(const ffl_transport_t *transport,
                                     const ffl_endpoint_t *endpoint,
                                     const ffl_xfer_msg_t *msgs,
                                     uint8_t count,
                                     ffl_xfer_done_fn done,
                                     void *user)
{
    if (!ffl_transport_is_valid(transport) || !ffl_endpoint_is_valid(endpoint) ||
        msgs == 0 || count == 0u) {
        return -EINVAL;
    }

    for (uint8_t index = 0u; index < count; ++index) {
        if (msgs[index].buf == 0 || msgs[index].len == 0u ||
            (msgs[index].flags & FFL_XFER_MSG_DIRECTION_MASK) == 0u ||
            (msgs[index].flags & FFL_XFER_MSG_DIRECTION_MASK) == FFL_XFER_MSG_DIRECTION_MASK) {
            return -EINVAL;
        }
    }

    return transport->ops->xfer(transport->ctx, endpoint, msgs, count, done, user);
}

static inline int ffl_transport_cancel(const ffl_transport_t *transport)
{
    if (!ffl_transport_is_valid(transport) || transport->ops->cancel == 0) {
        return -ENOTSUP;
    }

    return transport->ops->cancel(transport->ctx);
}

static inline int ffl_time_delay_ms(const ffl_time_ops_t *ops, void *ctx, uint32_t ms)
{
    if (ops == 0 || ops->delay_ms == 0) {
        return -ENOTSUP;
    }

    ops->delay_ms(ctx, ms);
    return 0;
}

static inline int ffl_time_delay_us(const ffl_time_ops_t *ops, void *ctx, uint32_t us)
{
    if (ops == 0 || ops->delay_us == 0) {
        return -ENOTSUP;
    }

    ops->delay_us(ctx, us);
    return 0;
}

static inline bool ffl_gpio_is_valid(const ffl_gpio_t *gpio)
{
    return gpio != 0 && gpio->ops != 0;
}

static inline int ffl_gpio_write(const ffl_gpio_t *gpio, ffl_gpio_level_t level)
{
    if (!ffl_gpio_is_valid(gpio) || gpio->ops->write == 0 ||
        (level != FFL_GPIO_LOW && level != FFL_GPIO_HIGH)) {
        return -EINVAL;
    }

    return gpio->ops->write(gpio->ctx, gpio->line, level);
}

static inline int ffl_gpio_read(const ffl_gpio_t *gpio, ffl_gpio_level_t *out_level)
{
    if (!ffl_gpio_is_valid(gpio) || gpio->ops->read == 0 || out_level == 0) {
        return -EINVAL;
    }

    return gpio->ops->read(gpio->ctx, gpio->line, out_level);
}

static inline bool ffl_irq_is_valid(const ffl_irq_t *irq)
{
    return irq != 0 && irq->ops != 0;
}

static inline int ffl_irq_enable(const ffl_irq_t *irq)
{
    if (!ffl_irq_is_valid(irq) || irq->ops->enable == 0) {
        return -ENOTSUP;
    }

    return irq->ops->enable(irq->ctx, irq->line);
}

static inline int ffl_irq_disable(const ffl_irq_t *irq)
{
    if (!ffl_irq_is_valid(irq) || irq->ops->disable == 0) {
        return -ENOTSUP;
    }

    return irq->ops->disable(irq->ctx, irq->line);
}

static inline int ffl_irq_ack(const ffl_irq_t *irq)
{
    if (!ffl_irq_is_valid(irq) || irq->ops->ack == 0) {
        return -ENOTSUP;
    }

    return irq->ops->ack(irq->ctx, irq->line);
}

#ifdef __cplusplus
}
#endif

#endif
