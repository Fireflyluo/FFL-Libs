# API 参考（中文）

> English version: [API_REFERENCE.md](./API_REFERENCE.md)

## 设备生命周期

- `int sc7a20_init(sc7a20_dev_t *dev)`
- `int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg)`
- `int sc7a20_deinit(sc7a20_dev_t *dev)`
- `int sc7a20_soft_reset(sc7a20_dev_t *dev)`
- `int sc7a20_get_who_am_i(sc7a20_dev_t *dev, uint8_t *who_am_i)`

## 寄存器访问

- `int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)`
- `int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)`

## 数据读取

- `int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out)`
- `int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out)`

## 配置接口

- `int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range)`
- `int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr)`
- `int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en)`

## 异步接口

- `int sc7a20_read_reg_async(..., sc7a20_done_cb_t cb, void *user)`
- `int sc7a20_write_reg_async(..., sc7a20_done_cb_t cb, void *user)`
- `int sc7a20_read_xyz_raw_async(..., sc7a20_read_xyz_cb_t cb, void *user)`
- `int sc7a20_cancel_async(sc7a20_dev_t *dev)`

## 返回值约定

- 成功返回 `0`
- 失败返回负的 POSIX 风格错误码
  - 例如：`-EINVAL`、`-EBUSY`、`-ENODEV`、`-EIO`、`-ETIMEDOUT`
