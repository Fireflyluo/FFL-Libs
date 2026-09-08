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

SHT40 的使用示例见 [`sht40/docs/`](../../../drivers/sensor/environmental/sht40/docs/)。

## 接入步骤

### 1. 实现 transport

在板级 port 中实现 `ffl_transport_ops_t.xfer`，将消息数组转换为 MCU HAL 的 I2C/SPI 调用：

- I2C endpoint 使用 `ffl_endpoint_i2c7(addr7)`，只传 7 位地址，不要把读写位拼入地址。
- SPI endpoint 使用 `ffl_endpoint_spi()`；片选、器件使能和复位线由独立的 `ffl_gpio_t` 控制。
- `FFL_XFER_MSG_WRITE` 和 `FFL_XFER_MSG_READ` 只能二选一；`STOP`、`RESTART` 是事务边界标志。
- `done == NULL` 时必须同步完成并返回最终状态；`done != NULL` 时，成功提交必须回调一次，提交失败不得回调。

如果 port 会排队异步请求，必须复制 endpoint 和消息描述；调用方持有的消息 buffer 要一直有效到同步返回或异步回调完成。异步回调不能直接在 ISR 中执行，应该投递到任务或线程上下文。实现 `cancel` 时，成功取消意味着回调不会再运行，也不能仍在运行。

### 2. 组装 transport

```c
static const ffl_transport_ops_t i2c_ops = {
  .xfer = board_i2c_xfer,
  .cancel = board_i2c_cancel,
};

static ffl_transport_t i2c1_transport = {
  .ops = &i2c_ops,
  .ctx = &board_i2c1,
  .endpoint = {0},
};

void board_sht40_transport_init(void)
{
  i2c1_transport.endpoint = ffl_endpoint_i2c7(0x46u);
}
```

`ctx` 由 port 自己定义，可以指向 HAL 句柄、总线实例或板级状态；core 不会解释它。不要让 transport 同时拥有 SPI CS，否则 GPIO 和 transport 可能重复操作同一根线。

### 3. 提供时间、GPIO 和 IRQ

按驱动的公开 `bind()` 参数和配置结构提供能力：

```c
static const ffl_time_ops_t board_time_ops = {
  .delay_ms = board_delay_ms,
  .delay_us = board_delay_us,
  .now_us = board_now_us,
};

static const ffl_gpio_ops_t board_gpio_ops = {
  .write = board_gpio_write,
  .read = board_gpio_read,
};
```

GPIO 的 `line` 是 port 自己解释的标识，可以是 GPIO 编号、端口和引脚的编码，或板级查表索引。IRQ 的 enable/disable/ack 只负责中断线控制；真正的数据处理仍由驱动的公开 `on_irq` 入口完成。

### 4. 绑定驱动并初始化

设备对象通常需要先清零，再按具体驱动的顺序调用 `*_bind()`、`*_config_init()` 和 `*_init()`。以 SHT40 为例：

```c
ffl_sht40_device_t device = {0};
ffl_sht40_config_t config;

ffl_sht40_config_init(&config);
config.i2c_addr7 = 0x46u;

int status = ffl_sht40_bind(&device, &i2c1_transport,
              &board_time_ops, 0);
if (status == 0) {
  status = ffl_sht40_init(&device, &config);
}
```

不同驱动需要的能力不完全相同。先查看对应公共头文件的 `*_bind()` 签名；不要为了复用一个 port 而填充驱动没有使用的能力。

## 验证清单

在 host/mock 阶段至少覆盖：无效 endpoint、空消息、读写方向、同步返回、异步 callback 次数、提交失败和取消路径。上板后再验证真实总线波形、地址、时序、IRQ、异常恢复和长时间运行。编译通过不能替代这些硬件证据。
