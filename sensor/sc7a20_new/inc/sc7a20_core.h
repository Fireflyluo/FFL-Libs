#ifndef SC7A20_NEW_CORE_H
#define SC7A20_NEW_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../sc7a20_reg.h"

#ifndef SC7A20_COMM_WRITE
#define SC7A20_COMM_WRITE (1u << 0)
#endif
#ifndef SC7A20_COMM_READ
#define SC7A20_COMM_READ  (1u << 1)
#endif
#ifndef SC7A20_COMM_STOP
#define SC7A20_COMM_STOP  (1u << 2)
#endif

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} sc7a20_comm_msg_t;

typedef void (*sc7a20_bus_done_cb_t)(void *user, int status);

typedef struct {
    int (*xfer)(void *ctx,
                const sc7a20_comm_msg_t *msgs,
                uint8_t cnt,
                sc7a20_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} sc7a20_bus_ops_t;

typedef struct {
    sc7a20_accel_fs_t range;
    sc7a20_accel_odr_t odr;
    bool axis_x_en;
    bool axis_y_en;
    bool axis_z_en;
    bool block_data_update;
    bool high_resolution;
    bool low_power;
} sc7a20_cfg_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sc7a20_vec3i16_t;

typedef struct {
    float x;
    float y;
    float z;
} sc7a20_vec3f_t;

typedef void (*sc7a20_done_cb_t)(void *user, int status);
typedef void (*sc7a20_read_xyz_cb_t)(void *user, const sc7a20_vec3i16_t *xyz, int status);

typedef enum {
    SC7A20_ASYNC_OP_NONE = 0,
    SC7A20_ASYNC_OP_READ_REG,
    SC7A20_ASYNC_OP_WRITE_REG,
    SC7A20_ASYNC_OP_READ_XYZ
} sc7a20_async_op_t;

typedef struct {
    sc7a20_async_op_t op;
    uint8_t reg_addr;
    uint8_t *read_buf;
    uint16_t read_len;
    const uint8_t *write_buf;
    uint16_t write_len;
    uint8_t raw_xyz[6];
    sc7a20_done_cb_t done_cb;
    sc7a20_read_xyz_cb_t xyz_cb;
    void *user;
} sc7a20_async_ctx_t;

typedef struct {
    const sc7a20_bus_ops_t *ops;
    void *bus_ctx;
    uint8_t addr;

    sc7a20_cfg_t cfg;
    bool initialized;
    uint8_t who_am_i;
    uint8_t endian_ble;
    float sensitivity_g_per_lsb;

    volatile uint8_t in_use;
    sc7a20_async_ctx_t async;
} sc7a20_dev_t;

extern const sc7a20_cfg_t g_sc7a20_default_cfg;

int sc7a20_core_try_lock(sc7a20_dev_t *dev);
void sc7a20_core_unlock(sc7a20_dev_t *dev);
int sc7a20_core_map_bus_status(int status);
int sc7a20_core_validate_dev(const sc7a20_dev_t *dev);

int sc7a20_core_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int sc7a20_core_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
int sc7a20_core_update_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t value);

int sc7a20_core_decode_xyz(const sc7a20_dev_t *dev, const uint8_t raw[6], sc7a20_vec3i16_t *out);
int sc7a20_core_apply_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
