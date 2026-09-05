# QMI8658A Driver

`ffl.qmi8658a` 是 QMI8658A 的同步 I2C facade，统一使用 `ffl.driver_port` 的
`ffl_transport_t`、`ffl_time_ops_t` 和 endpoint。公开入口位于
`include/ffl/qmi8658a.h`，目标为 `ffl.qmi8658a` object。

## 当前交付边界

- 仅支持 I2C 7-bit endpoint，地址来自 `ffl_endpoint_i2c7()`；默认地址为 `0x6A`，SA0 拉低时使用 `0x6B`。
- 仅支持加速度计与陀螺仪同时启用、自动地址递增和 14 字节 6DOF burst 读取；`enable_sync_sample` 可选。
- 同步模式下 `read_raw()` 检查 `STATUSINT.Avail/Locked`，非同步模式跳过该门控；两种模式都会检查 `STATUS0.aDA/gDA`，未就绪返回 `-EAGAIN`。
- 同步模式初始化会按 I2C 要求配置 `CTRL8/CTRL9` AHB clock gating，并轮询 `STATUSINT.CmdDone`，写入 `CTRL9=0x00` 完成 ACK；命令未完成或 ACK 未清除则初始化/配置失败。
- 运行中的同步配置会先清除 `CTRL7.syncSmpl`，等待并读取最后一个陀螺仪高字节释放旧锁定，再修改量程/ODR；切换到非同步模式时会恢复 AHB clock gating。
- 配置写入全部成功后才更新软件缓存；运行中的 facade 调用通过 `in_use` 原子字节锁串行化。
- FIFO、IRQ、SPI、异步传输和单传感器模式当前明确不属于支持范围，不应由文档或 API 推断为已实现。

## 用法

```c
ffl_qmi8658a_device_t device = {0};
ffl_qmi8658a_config_t config;
ffl_transport_t transport = {
    .ops = &board_i2c_ops,
    .ctx = &board_i2c,
    .endpoint = ffl_endpoint_i2c7(0x6A),
};

ffl_qmi8658a_config_init(&config);
ffl_qmi8658a_bind(&device, &transport, &board_time_ops, &board_time);
ffl_qmi8658a_init(&device, &config);
```

`ffl_time_ops_t` 必须提供 `delay_ms` 和 `delay_us`。`transport` 与 `time_ops` 结构体由设备借用，必须在设备整个生命周期内保持有效且不可并发修改。板级 I2C port 必须保留寄存器地址写入与后续读取之间的 repeated-start，并按
`ffl_xfer_msg_t.flags` 处理 `READ | RESTART | STOP`。

`imu_qmi8658a_*()` 是 facade 使用的内部核心接口；同一个设备对象不要与
`ffl_qmi8658a_*()` 混用，以免绕过 facade 的并发锁。

## 验证与硬件 backlog

`ffl.qmi8658a.test` 使用 fake-register I2C 验证地址传播、事务 flags、状态门控、burst 解析、CTRL9 命令 ACK、配置错误传播和缓存一致性；
`ffl.qmi8658a.cxx-test` 验证 C++ 外部头兼容。两者都不构成真实器件验证。

板端仍需验证 WHO_AM_I、SA0 两种地址、复位时序、量程/ODR 实际输出、STATUSINT 锁定延迟、I2C 异常恢复和连续采样。
