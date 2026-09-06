#ifndef FFL_SC7A20_H
#define FFL_SC7A20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "../sc7a20_core.h"

#ifndef FFL_SC7A20_ASYNC_ENABLED
#define FFL_SC7A20_ASYNC_ENABLED 1
#endif

#define FFL_SC7A20_DEFAULT_ADDR7_L SC7A20_I2C_ADDR_L
#define FFL_SC7A20_DEFAULT_ADDR7_H SC7A20_I2C_ADDR_H

typedef struct {
    sc7a20_dev_t core;
    const ffl_transport_t *transport;
    const ffl_time_ops_t *time_ops;
    void *time_ctx;
} ffl_sc7a20_device_t;

typedef sc7a20_cfg_t ffl_sc7a20_config_t;
typedef sc7a20_accel_fs_t ffl_sc7a20_range_t;
typedef sc7a20_accel_odr_t ffl_sc7a20_odr_t;
typedef sc7a20_vec3i16_t ffl_sc7a20_raw_t;
typedef sc7a20_vec3f_t ffl_sc7a20_g_t;

#define FFL_SC7A20_RANGE_2G  SC7A20_ACCEL_FS_2G
#define FFL_SC7A20_RANGE_4G  SC7A20_ACCEL_FS_4G
#define FFL_SC7A20_RANGE_8G  SC7A20_ACCEL_FS_8G
#define FFL_SC7A20_RANGE_16G SC7A20_ACCEL_FS_16G

#define FFL_SC7A20_ODR_POWER_DOWN SC7A20_ACCEL_ODR_POWER_DOWN
#define FFL_SC7A20_ODR_1_56HZ     SC7A20_ACCEL_ODR_1_56HZ
#define FFL_SC7A20_ODR_12_5HZ     SC7A20_ACCEL_ODR_12_5HZ
#define FFL_SC7A20_ODR_25HZ       SC7A20_ACCEL_ODR_25HZ
#define FFL_SC7A20_ODR_50HZ       SC7A20_ACCEL_ODR_50HZ
#define FFL_SC7A20_ODR_100HZ      SC7A20_ACCEL_ODR_100HZ
#define FFL_SC7A20_ODR_200HZ      SC7A20_ACCEL_ODR_200HZ
#define FFL_SC7A20_ODR_400HZ      SC7A20_ACCEL_ODR_400HZ
#define FFL_SC7A20_ODR_800HZ      SC7A20_ACCEL_ODR_800HZ
#define FFL_SC7A20_ODR_1_48KHZ    SC7A20_ACCEL_ODR_1_48KHZ
#define FFL_SC7A20_ODR_2_66KHZ    SC7A20_ACCEL_ODR_2_66KHZ
#define FFL_SC7A20_ODR_4_434KHZ   SC7A20_ACCEL_ODR_4_434KHZ

typedef sc7a20_done_cb_t ffl_sc7a20_done_fn;
typedef sc7a20_read_xyz_cb_t ffl_sc7a20_read_raw_fn;

void ffl_sc7a20_config_init(ffl_sc7a20_config_t *config);
int ffl_sc7a20_bind(ffl_sc7a20_device_t *device,
                    const ffl_transport_t *transport,
                    const ffl_time_ops_t *time_ops,
                    void *time_ctx);
int ffl_sc7a20_set_i2c_addr(ffl_sc7a20_device_t *device, uint8_t addr7);
int ffl_sc7a20_init(ffl_sc7a20_device_t *device,
                    const ffl_sc7a20_config_t *config);
int ffl_sc7a20_deinit(ffl_sc7a20_device_t *device);
int ffl_sc7a20_soft_reset(ffl_sc7a20_device_t *device);
int ffl_sc7a20_who_am_i(ffl_sc7a20_device_t *device, uint8_t *who_am_i);
int ffl_sc7a20_read_raw(ffl_sc7a20_device_t *device,
                        ffl_sc7a20_raw_t *raw);
int ffl_sc7a20_read_g(ffl_sc7a20_device_t *device,
                      ffl_sc7a20_g_t *sample);
int ffl_sc7a20_set_range(ffl_sc7a20_device_t *device,
                         ffl_sc7a20_range_t range);
int ffl_sc7a20_set_odr(ffl_sc7a20_device_t *device,
                       ffl_sc7a20_odr_t odr);
int ffl_sc7a20_set_axis_enable(ffl_sc7a20_device_t *device,
                               bool x_en,
                               bool y_en,
                               bool z_en);

#if FFL_SC7A20_ASYNC_ENABLED
int ffl_sc7a20_read_reg_async(ffl_sc7a20_device_t *device,
                              uint8_t reg,
                              uint8_t *data,
                              uint16_t len,
                              ffl_sc7a20_done_fn callback,
                              void *user);
int ffl_sc7a20_write_reg_async(ffl_sc7a20_device_t *device,
                               uint8_t reg,
                               const uint8_t *data,
                               uint16_t len,
                               ffl_sc7a20_done_fn callback,
                               void *user);
int ffl_sc7a20_read_raw_async(ffl_sc7a20_device_t *device,
                              ffl_sc7a20_read_raw_fn callback,
                              void *user);
int ffl_sc7a20_cancel_async(ffl_sc7a20_device_t *device);
#endif

#ifdef __cplusplus
}
#endif

#endif
