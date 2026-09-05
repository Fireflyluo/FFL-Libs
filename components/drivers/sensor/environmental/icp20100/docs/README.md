# ICP20100 Driver

`ffl.icp20100` 是不绑定 MCU 的 ICP20100 气压传感器 core。使用
`#include "ffl/icp20100.h"`，调用 `ffl_icp20100_bind()` 注入 I2C
`ffl_transport_t` 与必须提供 `delay_us` 的 `ffl_time_ops_t`；应用不需要等待官方
MCU port，也不应向 core 传入 HAL 句柄。

设备对象必须零初始化。transport、time ops 和其 ctx 在设备使用期间必须保持有效；bind 后的
I2C endpoint 地址会用于每一笔传输，配置中的 `addr` 仅能在初始化前选择目标地址。
同一设备的 bind、初始化、配置、读取、停测和改地址会串行化；若另一操作正在执行则返回
`-EBUSY`。

## 当前支持边界

- 仅支持 I2C 7-bit endpoint。
- 仅支持连续测量和 pressure/temperature 成对 FIFO 格式。
- forced mode、单通道 FIFO 和温度优先 FIFO 会返回 `-EINVAL`，避免以固定的 6-byte
  pressure-first 解析器产生错位数据。
- `ffl_icp20100_stop_measurement()` 仅停止当前测量，不是芯片软复位或完整异常恢复流程。

host mock 覆盖 I2C 地址传播、初始化寄存器访问、错误码、FIFO 20-bit 符号扩展和物理量换算；
这不是硬件验证。板端仍需确认上电/OTP 校准流程、芯片识别、启动延迟、压力温度读数及异常恢复。
