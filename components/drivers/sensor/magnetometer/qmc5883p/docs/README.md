# QMC5883P Driver

`ffl.qmc5883p` 是不绑定 MCU 的 QMC5883P 磁力计 core。使用
`#include "ffl/qmc5883p.h"`，调用 `ffl_qmc5883p_bind()` 注入 I2C
`ffl_transport_t`；应用不需要向 core 传入 HAL 句柄。

设备对象必须零初始化，transport 及其 ctx 在设备使用期间必须保持有效。bind 确定 I2C
transport 和默认地址；初始化配置中的 `addr` 可在初始化前选择实际 7-bit 地址，并会实际用于
每笔 I2C 事务。同一设备的 bind、初始化、配置、读取、复位和改地址会串行化，并发调用返回
`-EBUSY`。

## 当前支持边界

- 仅支持 I2C 7-bit endpoint，默认地址为 `0x2C`。
- 读取数据前必须检测 `DRDY`；无新数据返回 `-EAGAIN`，溢出返回 `-EIO`。
- 量程寄存器编码与物理单位换算按芯片定义处理：`30G/12G/8G/2G` 对应编码
  `0/1/2/3`，默认 8G 为 `37.5 LSB/µT`。
- 运行中调用 `ffl_qmc5883p_configure()` 会先写入 Suspend，再更新量程和测量参数。
- `ffl_qmc5883p_soft_reset()` 成功后设备回到未初始化状态，调用方必须重新执行 init。

host mock 覆盖地址传播、初始化寄存器访问、量程编码、DRDY/溢出、三轴解析、µT 换算和
运行时重配顺序；这不是硬件验证。板端仍需验证芯片识别、连续读数、量程设置、数据就绪
时序和异常恢复。
