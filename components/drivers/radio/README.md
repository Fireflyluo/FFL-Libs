# Radio Drivers

此分类暂未声明通用射频 core。SI24R1、XN297L 和 XL2400P 的既有 CH32 HAL 实现已整理到 `ports/ch32/legacy/radio/`。

后续迁移需要先抽出 SPI、CS/CE、IRQ 和 delay port，随后每颗芯片建立独立叶子组件。板端回补项见 `docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md`。
