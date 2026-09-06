# 嵌入式库重组记录

**状态：** 旧树已清理，正式组件与端口树已落地。  
**日期：** 2026-09-04

## 已执行的架构决策

1. `components/` 是唯一正式源码来源；分类目录不再产生 `ACC-lib`、`IMU-lib` 或 `embedded-sensor-drivers` 之类的大类聚合 target。
2. `ports/` 承担 MCU、板卡、HAL、SPI、I2C、GPIO、DMA、IRQ、时钟与临界区适配；通用 core 不得包含这些依赖。
3. 默认使用 `object` 源码组件，保证资源受限单片机能够按功能裁剪；稳定驱动可额外提供同源的单设备 static package。
4. 根工程采用 C11 + C++17；C core 使用 C-compatible public API。
5. `experimental/` 是明确隔离的未整合工作区，不进入默认构建。

## 当前迁移结果

| 领域 | 正式组件 | 适配 / 测试位置 | 状态 |
|---|---|---|---|
| 基础设施 | `ffl.atomic`、`ffl.driver_port`、`ffl.ringbuffer`、`ffl.sw_timer` | 各组件 `test/` | driver-port 为通用南向契约；atomic、ringbuffer 与 sw-timer 均有 GCC host 测试。ringbuffer 与 sw-timer 的公开头分别位于 `include/ffl/ringbuffer.h`、`include/ffl/sw_timer.h`。 |
| 算法 | `ffl.impact_displacement` | `components/algorithms/impact-displacement/test/` | 纯算法 core，无 HAL、RTOS、总线或板级依赖；提供 C/C++ GCC host 测试。源项目的 `impact_episode_v02*` 业务策略与事件生命周期未迁移，仍由应用层负责。 |
| 运行时 | `ffl.protothreads`、`ffl.osal` | `ports/py32/osal/` | OSAL 已编译；PY32 硬件回补待做。 |
| 协议 | `ffl.adhoc` | `ports/ch32/adhoc/`、`tests/protocols/adhoc/` | core 已迁移并可编译。 |
| 加速度计 | `ffl.sc7a20`、`ffl-sc7a20` | `ports/ch32/sc7a20/`、组件 mock test | 保留源码裁剪与完整 static 双入口。 |
| 环境传感器 | `ffl.sht30`、`ffl.sht40`、`ffl.icp20100` | `ports/ch32/sht40/` | SHT40 已提供 `ffl_transport_t` 门面与 CH32 适配；板端 I2C 验证待做。 |
| IMU / 磁力计 | `ffl.qmi8658a`、`ffl.icm42688p`、`ffl.qmc5883p` | 由最终工程提供 bus port | core 已迁移；板端验证待做。 |
| 显示 / 射频 | 无正式 core target | `ports/ch32/legacy/` | 已从旧根目录隔离，但仍待拆出通用 bus/GPIO port。 |

## 旧目录清理结果

旧 `Lib/`、`sensor/`、`RF/`、`OSAL/`、`utils/` 与 `demo/` 已从工作区清除。重复的 SC7A20、SHT40、QMI8658A、ICM42688P 实现已收敛到一个正式 core；SC7A20HTR FIFO 工作区以及旧 HAL 直接实现被保留在相应组件的 `experimental/` 子目录，避免覆盖正在开发的内容。

`ports/ch32/legacy/radio/xl2400p/xn297L.*` 的重复副本已删除，只保留独立的 `ports/ch32/legacy/radio/xn297l/` 实现。

## 验收标准

- 每个正式叶子组件只有一个 core、一套公开头和一个主 `ffl.*` target。
- core 可在不引入目标 MCU HAL 的情况下配置和编译。
- host / mock 测试、编译检查和真实板端验证明确区分。
- 需要裁剪的驱动始终保留源码入口；package 只作为同源的快速入口。
- 板端测试未完成时，必须登记在 `docs/HARDWARE_VALIDATION_BACKLOG.md`，不以构建成功替代硬件结论。
