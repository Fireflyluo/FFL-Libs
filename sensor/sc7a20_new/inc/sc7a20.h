#ifndef SC7A20_NEW_H
#define SC7A20_NEW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sc7a20_core.h"

int sc7a20_init(sc7a20_dev_t *dev);
int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg);
int sc7a20_deinit(sc7a20_dev_t *dev);
int sc7a20_soft_reset(sc7a20_dev_t *dev);
int sc7a20_get_who_am_i(sc7a20_dev_t *dev, uint8_t *who_am_i);

int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);

int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out);
int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out);

int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range);
int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr);
int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en);

int sc7a20_read_reg_async(sc7a20_dev_t *dev,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len,
                          sc7a20_done_cb_t cb,
                          void *user);
int sc7a20_write_reg_async(sc7a20_dev_t *dev,
                           uint8_t reg,
                           const uint8_t *data,
                           uint16_t len,
                           sc7a20_done_cb_t cb,
                           void *user);
int sc7a20_read_xyz_raw_async(sc7a20_dev_t *dev,
                              sc7a20_read_xyz_cb_t cb,
                              void *user);
int sc7a20_cancel_async(sc7a20_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif
