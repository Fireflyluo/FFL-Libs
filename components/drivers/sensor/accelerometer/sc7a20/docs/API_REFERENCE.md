# API 参考

portable facade 的公共前缀为 `ffl_sc7a20_`，core API 的公共前缀为
`sc7a20_`。应用层优先使用 facade；需要源码级裁剪时通过
`sc7a20_async` Xmake option 移除异步源文件。

## Portable facade

- `void ffl_sc7a20_config_init(ffl_sc7a20_config_t *config)`
- `int ffl_sc7a20_bind(ffl_sc7a20_device_t *device, const ffl_transport_t *transport, const ffl_time_ops_t *time_ops, void *time_ctx)`
- `int ffl_sc7a20_set_i2c_addr(ffl_sc7a20_device_t *device, uint8_t addr7)`
- `int ffl_sc7a20_init(ffl_sc7a20_device_t *device, const ffl_sc7a20_config_t *config)`
- `int ffl_sc7a20_deinit(ffl_sc7a20_device_t *device)`
- `int ffl_sc7a20_soft_reset(ffl_sc7a20_device_t *device)`
- `int ffl_sc7a20_who_am_i(ffl_sc7a20_device_t *device, uint8_t *who_am_i)`
- `int ffl_sc7a20_read_raw(ffl_sc7a20_device_t *device, ffl_sc7a20_raw_t *raw)`
- `int ffl_sc7a20_read_g(ffl_sc7a20_device_t *device, ffl_sc7a20_g_t *sample)`
- `int ffl_sc7a20_set_range(ffl_sc7a20_device_t *device, ffl_sc7a20_range_t range)`
- `int ffl_sc7a20_set_odr(ffl_sc7a20_device_t *device, ffl_sc7a20_odr_t odr)`
- `int ffl_sc7a20_set_axis_enable(ffl_sc7a20_device_t *device, bool x_en, bool y_en, bool z_en)`

## Core 生命周期

- `int sc7a20_init(sc7a20_dev_t *dev)`
- `int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg)`
- `int sc7a20_deinit(sc7a20_dev_t *dev)`
- `int sc7a20_soft_reset(sc7a20_dev_t *dev)`
- `int sc7a20_get_who_am_i(sc7a20_dev_t *dev, uint8_t *who_am_i)`

## Core 寄存器访问

- `int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)`
- `int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len)`

## Core 数据与配置

- `int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out)`
- `int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out)`
- `int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range)`
- `int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr)`
- `int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en)`

## 异步接口

`ffl.sc7a20.full` 与兼容目标 `ffl-sc7a20` 始终提供以下异步入口；
`ffl.sc7a20` object 仅在 `sc7a20_async=true` 时提供：

- `int ffl_sc7a20_read_reg_async(..., ffl_sc7a20_done_fn callback, void *user)`
- `int ffl_sc7a20_write_reg_async(..., ffl_sc7a20_done_fn callback, void *user)`
- `int ffl_sc7a20_read_raw_async(..., ffl_sc7a20_read_raw_fn callback, void *user)`
- `int ffl_sc7a20_cancel_async(ffl_sc7a20_device_t *device)`

对应 core 入口为 `sc7a20_read_reg_async()`、`sc7a20_write_reg_async()`、
`sc7a20_read_xyz_raw_async()` 和 `sc7a20_cancel_async()`。关闭
`sc7a20_async` 后，`ffl.sc7a20` object 不编译异步源文件，且该 object
的 facade 异步声明隐藏；完整 static 入口不受该裁剪选项影响。

完成回调必须遵守 `ffl.driver_port` 契约，在任务或线程上下文执行；异步
传输接受后，调用者提供的缓冲区必须保持有效直到完成回调。transport 未
提供 cancel 时，取消返回 `-ENOTSUP`，不会默默降级为同步等待。

## 错误码

- 成功返回 `0`。
- 失败返回负的 POSIX 风格错误码，例如 `-EINVAL`、`-EBUSY`、`-ENODEV`、
  `-EIO` 和 `-ENOTSUP`。
- facade 绑定 SPI 或其它非 I2C endpoint 返回 `-ENOTSUP`。
- 地址必须是非零 7-bit I2C 地址；非法地址返回 `-EINVAL`。
