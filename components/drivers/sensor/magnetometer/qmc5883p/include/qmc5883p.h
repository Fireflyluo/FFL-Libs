#ifndef QMC5883P_H
#define QMC5883P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "qmc5883p_reg.h"

#ifndef QMC5883P_COMM_WRITE
#define QMC5883P_COMM_WRITE (1u << 0)
#endif
#ifndef QMC5883P_COMM_READ
#define QMC5883P_COMM_READ  (1u << 1)
#endif
#ifndef QMC5883P_COMM_STOP
#define QMC5883P_COMM_STOP  (1u << 2)
#endif

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} qmc5883p_comm_msg_t;

typedef struct {
    int (*xfer)(void *ctx,
                const qmc5883p_comm_msg_t *msgs,
                uint8_t cnt,
                void *done_cb,
                void *user);
} qmc5883p_bus_ops_t;

typedef struct {
    float x;
    float y;
    float z;
} qmc5883p_vec3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883p_vec3i16_t;

typedef struct {
    uint8_t addr;
    uint8_t expected_chip_id;
    qmc5883p_mode_t mode;
    qmc5883p_odr_t odr;
    qmc5883p_osr1_t osr1;
    qmc5883p_osr2_t osr2;
    qmc5883p_range_t range;
    qmc5883p_set_reset_mode_t set_reset_mode;
} qmc5883p_cfg_t;

typedef struct {
    const qmc5883p_bus_ops_t *ops;
    void *bus_ctx;
    uint8_t addr;
    uint8_t chip_id;
    bool initialized;
    qmc5883p_cfg_t cfg;
} qmc5883p_dev_t;

extern const qmc5883p_cfg_t g_qmc5883p_default_cfg;

int qmc5883p_init(qmc5883p_dev_t *dev, const qmc5883p_cfg_t *cfg);
int qmc5883p_probe(qmc5883p_dev_t *dev, uint8_t *chip_id);
int qmc5883p_soft_reset(qmc5883p_dev_t *dev);
int qmc5883p_set_config(qmc5883p_dev_t *dev, const qmc5883p_cfg_t *cfg);

int qmc5883p_read_reg(qmc5883p_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int qmc5883p_write_reg(qmc5883p_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

int qmc5883p_read_raw(qmc5883p_dev_t *dev, qmc5883p_vec3i16_t *out);
int qmc5883p_read_ut(qmc5883p_dev_t *dev, qmc5883p_vec3f_t *out);

#ifdef __cplusplus
}
#endif

#endif
