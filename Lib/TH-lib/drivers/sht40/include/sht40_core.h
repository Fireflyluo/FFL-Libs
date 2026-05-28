#ifndef SHT40_NEW_CORE_H
#define SHT40_NEW_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#ifndef SHT40_COMM_WRITE
#define SHT40_COMM_WRITE (1u << 0)
#endif
#ifndef SHT40_COMM_READ
#define SHT40_COMM_READ  (1u << 1)
#endif
#ifndef SHT40_COMM_STOP
#define SHT40_COMM_STOP  (1u << 2)
#endif

#ifndef SHT40_I2C_ADDR
#define SHT40_I2C_ADDR 0x46u
#endif

typedef enum {
    SHT40_PRECISION_HIGH = 0,
    SHT40_PRECISION_MEDIUM,
    SHT40_PRECISION_LOW
} sht40_precision_t;

typedef enum {
    SHT40_HEATER_200MW_1S = 0x39,
    SHT40_HEATER_200MW_100MS = 0x32,
    SHT40_HEATER_110MW_1S = 0x2F,
    SHT40_HEATER_110MW_100MS = 0x24,
    SHT40_HEATER_20MW_1S = 0x1E,
    SHT40_HEATER_20MW_100MS = 0x15
} sht40_heater_cmd_t;

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} sht40_comm_msg_t;

typedef void (*sht40_bus_done_cb_t)(void *user, int status);

typedef struct {
    int (*xfer)(void *ctx,
                const sht40_comm_msg_t *msgs,
                uint8_t cnt,
                sht40_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} sht40_bus_ops_t;

typedef void (*sht40_delay_ms_fn)(void *ctx, uint32_t ms);

typedef struct {
    float temperature_c;
    float humidity_rh;
} sht40_sample_t;

typedef void (*sht40_done_cb_t)(void *user, int status);
typedef void (*sht40_sample_cb_t)(void *user, const sht40_sample_t *sample, int status);

typedef enum {
    SHT40_ASYNC_NONE = 0,
    SHT40_ASYNC_READ_SAMPLE,
    SHT40_ASYNC_SOFT_RESET
} sht40_async_op_t;

typedef struct {
    sht40_async_op_t op;
    uint8_t cmd;
    uint8_t rx[6];
    sht40_sample_cb_t sample_cb;
    sht40_done_cb_t done_cb;
    void *user;
} sht40_async_ctx_t;

typedef struct {
    const sht40_bus_ops_t *ops;
    void *bus_ctx;
    uint8_t addr;

    sht40_delay_ms_fn delay_ms;
    void *delay_ctx;

    bool initialized;
    volatile uint8_t in_use;
    sht40_async_ctx_t async;
} sht40_dev_t;

int sht40_core_try_lock(sht40_dev_t *dev);
void sht40_core_unlock(sht40_dev_t *dev);
int sht40_core_validate_dev(const sht40_dev_t *dev);
int sht40_core_map_bus_status(int status);

uint8_t sht40_core_precision_cmd(sht40_precision_t precision);
uint32_t sht40_core_measure_delay_ms(uint8_t cmd);

int sht40_core_xfer_sync(sht40_dev_t *dev, uint8_t *buf, uint16_t len, bool read);
int sht40_core_read_sample_parse(const uint8_t rx[6], sht40_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif
