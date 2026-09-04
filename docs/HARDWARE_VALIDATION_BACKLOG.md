# 硬件验证回补清单

**更新日期：** 2026-09-04  
**原则：** 本清单中的“已编译”或“mock 通过”不等于硬件验证。完成板端测试后，应在对应组件文档和本表记录目标板、总线、频率、固件提交和结果。

## 已有自动验证

| 组件 | 验证方式 | 结论 |
|---|---|---|
| `ffl.ringbuffer` | C++ host test | 可自动验证。 |
| `ffl.sw_timer` | C++ host test | 可自动验证。 |
| `ffl.sc7a20.unit` | mock I2C 单元测试 | 不访问真实器件。 |
| `ffl.sc7a20.integration` | mock I2C 集成测试 | 不访问真实器件。 |

## 需要板端回补

| 组件 / Port | 当前状态 | 板端验收项 |
|---|---|---|
| `ffl.sc7a20` + `ports/ch32/sc7a20/` | core 与 mock 测试可构建 | WHO_AM_I、寄存器读写、量程/ODR、连续采样、FIFO、异步或中断路径、I2C 异常恢复。 |
| `ffl.sht40` + `ports/ch32/sht40/` | core 可构建 | 序列号、单次测量、CRC、repeatability、I2C NACK / timeout 恢复。 |
| `ffl.sht30` | core 可构建 | 初始化、测温湿、CRC、clock stretching 或平台总线时序。 |
| `ffl.icp20100` | core 可构建 | 芯片识别、启动延迟、压力温度读数、异常复位恢复。 |
| `ffl.qmc5883p` | core 可构建 | 芯片识别、连续磁场读数、量程与数据就绪。 |
| `ffl.qmi8658a` | core 可构建 | I2C/SPI 任选实际接口、WHO_AM_I、加速度/陀螺仪量程与 FIFO。 |
| `ffl.icm42688p` | core 可构建 | SPI/I2C 实际接口、WHO_AM_I、加速度/陀螺仪、FIFO 与中断。 |
| `ffl.osal` + `ports/py32/osal/` | 通用 core 可编译 | tick、临界区保护、任务消息、定时器、内存分配与长时间调度。 |
| `ffl.adhoc` + `ports/ch32/adhoc/` | 协议 core 可编译 | CH32 链路收发、时钟驱动、丢包/重传、长时间稳定性。 |

## 已整理、待解耦的板级驱动

这些目录已经从旧根目录迁入 `ports/ch32/legacy/`，但源码仍直接依赖 CH32 HAL、`main.h`、固定 GPIO 或全局 SPI 句柄。因此它们 **不是** 可复用 `components/` core，也没有自动 host 测试：

| Port 目录 | 后续整理目标 | 板端验收项 |
|---|---|---|
| `ports/ch32/legacy/display/oled/` | 抽出显示数据/命令传输、延时与复位 port | 上电初始化、全屏刷新、字体/图形、长时间刷新。 |
| `ports/ch32/legacy/radio/si24r1/` | 抽出 SPI、CE、IRQ 与延时 port | SPI 读写、收发配对、IRQ、丢包与重发。 |
| `ports/ch32/legacy/radio/xn297l/` | 抽出 SPI、CE/CSN、IRQ 与延时 port | 寄存器读写、收发配对、频道/速率切换。 |
| `ports/ch32/legacy/radio/xl2400p/` | 抽出 SPI、NSS、IRQ 与延时 port | SPI self-test、收发配对、状态机与异常恢复。 |

`SC7A20HTR` FIFO 工作区保留在 `components/drivers/sensor/accelerometer/sc7a20/experimental/sc7a20htr/`；在确定是否吸收其接口前，不应将其结果当作正式 SC7A20 驱动的验证结论。
