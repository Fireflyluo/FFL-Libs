#ifndef SHT30_CORE_H
#define SHT30_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#ifndef SHT30_COMM_WRITE
#define SHT30_COMM_WRITE (1u << 0)
#endif
#ifndef SHT30_COMM_READ
#define SHT30_COMM_READ  (1u << 1)
#endif
#ifndef SHT30_COMM_STOP
#define SHT30_COMM_STOP  (1u << 2)
#endif

#ifndef SHT30_I2C_ADDR
#define SHT30_I2C_ADDR 0x44u
#endif

typedef enum {
    SHT30_PRECISION_HIGH = 0,
    SHT30_PRECISION_MEDIUM,
    SHT30_PRECISION_LOW
} sht30_precision_t;

typedef sht30_precision_t sht30_repeatability_t;

typedef enum {
    SHT30_HEATER_ENABLE  = 0x306D,
    SHT30_HEATER_DISABLE = 0x3066
} sht30_heater_cmd_t;

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} sht30_comm_msg_t;

typedef void (*sht30_bus_done_cb_t)(void *user, int status);

typedef struct {
    int (*xfer)(void *ctx,
                const sht30_comm_msg_t *msgs,
                uint8_t cnt,
                sht30_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} sht30_bus_ops_t;

typedef void (*sht30_delay_ms_fn)(void *ctx, uint32_t ms);

typedef struct {
    float temperature_c;
    float humidity_rh;
} sht30_sample_t;

typedef void (*sht30_done_cb_t)(void *user, int status);
typedef void (*sht30_sample_cb_t)(void *user, const sht30_sample_t *sample, int status);

typedef enum {
    SHT30_ASYNC_NONE = 0,
    SHT30_ASYNC_READ_SAMPLE,
    SHT30_ASYNC_SOFT_RESET
} sht30_async_op_t;

typedef struct {
    sht30_async_op_t op;
    uint8_t cmd[2];
    uint8_t rx[6];
    sht30_sample_cb_t sample_cb;
    sht30_done_cb_t done_cb;
    void *user;
} sht30_async_ctx_t;

typedef struct {
    const sht30_bus_ops_t *ops;
    void *bus_ctx;
    uint8_t addr;

    sht30_delay_ms_fn delay_ms;
    void *delay_ctx;

    bool initialized;
    volatile uint8_t in_use;
    sht30_async_ctx_t async;
} sht30_dev_t;

int sht30_core_try_lock(sht30_dev_t *dev);
void sht30_core_unlock(sht30_dev_t *dev);
int sht30_core_validate_dev(const sht30_dev_t *dev);
int sht30_core_map_bus_status(int status);

void sht30_core_precision_cmd(uint8_t cmd[2], sht30_precision_t precision);
uint32_t sht30_core_measure_delay_ms(const uint8_t cmd[2]);

int sht30_core_xfer_sync(sht30_dev_t *dev, uint8_t *buf, uint16_t len, bool read);
int sht30_core_read_sample_parse(const uint8_t rx[6], sht30_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif