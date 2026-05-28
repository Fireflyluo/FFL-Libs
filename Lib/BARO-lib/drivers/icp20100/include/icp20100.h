#ifndef ICP20100_H
#define ICP20100_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "icp20100_reg.h"

#ifndef ICP20100_COMM_WRITE
#define ICP20100_COMM_WRITE (1u << 0)
#endif
#ifndef ICP20100_COMM_READ
#define ICP20100_COMM_READ  (1u << 1)
#endif
#ifndef ICP20100_COMM_STOP
#define ICP20100_COMM_STOP  (1u << 2)
#endif

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} icp20100_comm_msg_t;

typedef struct {
    int (*xfer)(void *ctx,
                const icp20100_comm_msg_t *msgs,
                uint8_t cnt,
                void *done_cb,
                void *user);
} icp20100_bus_ops_t;

typedef struct {
    float pressure_kpa;
    float temperature_c;
} icp20100_sample_t;

typedef struct {
    int32_t pressure_raw;
    int32_t temperature_raw;
} icp20100_raw_sample_t;

typedef struct {
    uint8_t addr;
    uint8_t expected_chip_id;
    icp20100_op_mode_t op_mode;
    icp20100_meas_mode_t meas_mode;
    icp20100_power_mode_t power_mode;
    icp20100_fifo_mode_t fifo_mode;
} icp20100_cfg_t;

typedef struct {
    const icp20100_bus_ops_t *ops;
    void *bus_ctx;
    void (*delay_us)(void *ctx, uint32_t us);
    void *delay_ctx;

    uint8_t addr;
    uint8_t chip_id;
    uint8_t version;
    bool initialized;
    icp20100_cfg_t cfg;
} icp20100_dev_t;

extern const icp20100_cfg_t g_icp20100_default_cfg;

int icp20100_init(icp20100_dev_t *dev, const icp20100_cfg_t *cfg);
int icp20100_probe(icp20100_dev_t *dev, uint8_t *chip_id, uint8_t *version);
int icp20100_soft_reset(icp20100_dev_t *dev);
int icp20100_set_config(icp20100_dev_t *dev, const icp20100_cfg_t *cfg);

int icp20100_read_reg(icp20100_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int icp20100_write_reg(icp20100_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

int icp20100_read_raw(icp20100_dev_t *dev, icp20100_raw_sample_t *raw);
int icp20100_read_sample(icp20100_dev_t *dev, icp20100_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
