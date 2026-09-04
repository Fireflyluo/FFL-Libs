# QMI8658A Driver

`ffl.qmi8658a` 使用 `ffl.imu_common` 的通用总线契约，不包含固定板级资源。

当前完成代码迁移与 host 编译检查。板端回补应覆盖实际 I2C/SPI 接口、WHO_AM_I、量程、FIFO 和数据连续性。
