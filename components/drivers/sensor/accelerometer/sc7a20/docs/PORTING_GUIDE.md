# 移植指南

## Portable facade

正式应用建议注入 `ffl_transport_t`，不要在组件内包含 MCU HAL：

```c
ffl_sc7a20_device_t device = {0};
ffl_sc7a20_config_t config;

ffl_sc7a20_config_init(&config);
if (ffl_sc7a20_bind(&device, &i2c_transport, &time_ops, time_ctx) == 0) {
    (void)ffl_sc7a20_init(&device, &config);
}
```

`transport`、`time_ops`、`time_ctx` 和异步传输使用的缓冲区都是借用的，
必须持续到设备解绑/销毁或异步回调完成。`bind()` 只接受 I2C 7-bit
endpoint；SPI endpoint 明确返回 `-ENOTSUP`。设备已初始化或有异步事务时，
修改地址/重新绑定返回 `-EBUSY`。

## Core 总线适配

直接使用 core 时，需要实现：

- `xfer(ctx, msgs, cnt, cb, user)`
- `cancel(ctx)`（异步取消可选）

`msgs` 表示一次原子事务，驱动层不会将它拆成互不相关的操作。典型寄存器
读取由一条写寄存器地址消息和一条读数据消息组成；写入由一条写寄存器地址
消息和一条写载荷消息组成。

portable facade 会将 core 的 `SC7A20_COMM_*` 标志转换为 `FFL_XFER_MSG_*`。
寄存器读的第二条消息额外带 `FFL_XFER_MSG_RESTART`，以保留 I2C
repeated-start；写事务不伪造 repeated-start。

## 同步与异步

`ffl.sc7a20.full` 始终包含异步入口。`ffl.sc7a20` object 由
`sc7a20_async` 控制是否编译 `src/sc7a20_async.c`；关闭后异步声明和实现
都不进入目标，适合 Flash/RAM 受限 MCU。transport 不支持取消时，
`cancel_async` 明确返回 `-ENOTSUP`，不降级为同步等待。

异步传输接受后，core 保存的寄存器地址、读写缓冲区和用户上下文必须在
完成回调前保持有效。底层回调如果由 ISR 触发，必须先切换到任务/线程上下文
再执行 `ffl.driver_port` 完成回调；不能直接在 ISR 中执行可能继续发起传输
的驱动回调。

## CH32 接入边界

CH32 I2C、GPIO、时钟和中断适配应放在 `ports/ch32/`，由其实现
`ffl_transport_ops_t`。组件 core 和 facade 不依赖固定 I2C 句柄、`main.h`
或厂商 HAL。当前正式 SC7A20 组件仅交付同步/异步 I2C 事务和三轴读取，
FIFO/IRQ 工作区仍保留在 `experimental/sc7a20htr/`，不进入正式 target。

## 验证边界

fake-register 测试只能验证寄存器事务、地址、标志、解析和错误传播；它不
等价于真实器件验证。尚未完成真实 SC7A20 硬件上的 WHO_AM_I、量程/ODR、
连续采样、异步总线时序和异常恢复验证。
