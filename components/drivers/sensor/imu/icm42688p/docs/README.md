# ICM42688P Driver

`ffl.icm42688p` 是 ICM-42688-P 的可移植 I2C 7-bit facade，使用
`ffl.driver_port` 注入 transport 和 delay，不包含固定板级 HAL、GPIO 或外设句柄。

## 交付形态

- `ffl.icm42688p` 是 object target；应用只编译需要的源码组件。
- 公共入口为 `include/ffl/icm42688p.h`，实现为 `src/ffl_icm42688p.c`。
- `bind()` 要求 I2C 7-bit endpoint、有效地址和 `delay_ms`；transport endpoint 的地址是初始地址，后续事务每次使用 `device->addr`。
- `transport` 与 `time_ops` 结构体由 device 借用，必须在整个 device 生命周期内保持有效且不可并发修改；不要把局部变量地址传入后再离开其作用域。
- I2C 寄存器读保持“寄存器地址写 + repeated-start + 读 + stop”；User Bank 切换通过 `BANK_SEL (0x76)` 完成，并在 bank 切换事务成功后更新软件 bank 状态。

## 识别与配置

- `WHO_AM_I (Bank 0, 0x75)` 的 ICM-42688-P 值为 `0x47`。
- `0x68/0x69` 是 I2C 7-bit 地址，由 SDO/AD0 硬件连接决定，不是 WHO_AM_I 值。
- 当前 facade 仅覆盖同步 I2C、加速度计和陀螺仪配置、14-byte 温度/6DOF burst raw 读取及单位换算。
- 配置只接受有效的 ICM42688P ODR 编码；加速度低功耗模式仅接受 200 Hz 及以下 ODR，陀螺仪拒绝保留 ODR 编码。
- `configure()` 在所有目标寄存器成功写入后才提交配置缓存；写失败时缓存回滚，设备标记为未初始化，需重新 `init()`。
- `soft_reset()` 成功或失败后都不保留 initialized 状态；未初始化读取返回 `-ENODEV`。
- `imu_icm42688p_*()` 是 facade 使用的内部 core 接口，不应与 `ffl_icm42688p_*()` 混用，否则会绕过 facade 的并发锁。

## 未交付与硬件 backlog

本批不伪装支持 SPI、FIFO、IRQ、APEX、DMA、异步 transport。现有寄存器定义保留这些硬件区域供后续专项实现，但未纳入 facade API。

Host fake-register 测试覆盖 bank select、地址传播、repeated-start 标志、初始化配置、raw 解析、单位换算、重入锁、错误回滚和未初始化错误。仍需在目标板核对真实 I2C 波形、WHO_AM_I、复位延迟、bank 切换、量程/ODR 与连续采样稳定性。
