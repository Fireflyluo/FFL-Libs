# CH32 Ports

本目录集中放置 CH32 系列 MCU 的组件适配层。适配层负责把最终工程的 I2C、SPI、GPIO、tick、DMA 和 IRQ 资源接入通用组件，不能反向让 component core 依赖 `board.h` 或 HAL。

- `adhoc/`：Ad-Hoc 协议链路适配。
- `sc7a20/`：SC7A20 事务总线适配；需要真实 I2C / FIFO / IRQ 验证。
- `sht40/`：SHT40 事务总线适配；需要真实 I2C / CRC / timeout 验证。
- `legacy/`：OLED 与射频的既有 HAL 直连实现，仅作整理保留，尚待抽取通用 core。

端口完成度和板端验收标准见 `docs/HARDWARE_VALIDATION_BACKLOG.md`。
