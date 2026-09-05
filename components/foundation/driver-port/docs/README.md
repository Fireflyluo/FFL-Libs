# `ffl.driver_port`

`ffl.driver_port` 提供组件 core 使用的最小南向传输契约。它不绑定具体
MCU、板卡或厂商 HAL，正式端口和最终工程都可以实现同一组回调。

- `ffl_transport_t`：携带 transport 回调、上下文和 endpoint。
- `ffl_xfer_msg_t`：描述一次 I2C/SPI 消息，支持读、写、STOP 和 repeated
  start 标志。
- `ffl_time_ops_t`：提供可选的毫秒/微秒延时和微秒时间源。
- `ffl_gpio_t`：通过 `write` / `read` 接入 CS、CE、复位等板级信号。
- `ffl_irq_t`：通过 `enable` / `disable` / `ack` 接入 IRQ 控制；实际 ISR
  中应由最终工程显式调用组件的 `on_irq` API，而非由 core 注册厂商中断。

SPI endpoint 不管理片选。SPI 设备的 CS/CE 必须由各组件配置中的
`ffl_gpio_t` 管理，避免 transport 与 GPIO 对同一硬件信号产生双重所有权。

同步传输直接返回结果；当调用方传入 `done` 时，适配层在传输完成后调用
一次回调。适配层可以在返回前完成回调，也可以排队后异步完成。若提交
阶段直接失败，应返回负 errno，并且不要再调用 `done`。

SHT40 的使用示例见 `components/drivers/sensor/environmental/sht40/docs/`。
