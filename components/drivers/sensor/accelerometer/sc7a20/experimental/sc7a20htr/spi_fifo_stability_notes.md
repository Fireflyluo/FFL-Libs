# SC7A20 SPI FIFO 稳定性记录

## 背景

在 CH58x 3 线 SPI + FIFO 流模式场景中，应用侧曾出现以下异常：

- `FIFO_SRC` 间歇性读到 `0xFF`
- FIFO 中断安静，样本路径报错后需要重配
- `WHO_AM_I` 仍然能读通，说明不是整条总线完全失效

这类现象容易被误判为 CPU 占用、DMA 或中断调度问题，但本轮定位结果不是这个方向。

## 根因结论

本轮确认的关键点是 FIFO 取样访问路径：

- 直接读取 `0x69 (SC7A20_FIFO_DATA)` 时，SPI 侧依赖 `SC7A20_SPI_CTRL.ADR_SPI_AD6` 先切到 `0x40~0x7F` bank
- 3 线 SPI 下频繁走 `0x69 + bank 切换`，链路更容易进入异常状态
- 按手册改为从 `0x27` 起做 7 字节连续读取，再用后 6 字节解码 XYZ，链路明显稳定

因此，本轮的主修复不是“换 DMA”或“降低频率”，而是“改正确的 FIFO 读法”。

## 本次库内沉淀

新增 API：

- `sc7a20_get_fifo_src()`
- `sc7a20_read_fifo_raw_data()`
- `sc7a20_read_fifo_acceleration()`

新增约束说明：

- `inc/sc7a20_reg.h` 中补充了 `0x69`/`ADR_SPI_AD6` 的 SPI 注意事项
- README 同步补充 SPI FIFO 使用建议

## 集成建议

1. 如果项目通过 SPI 使用 FIFO，优先调用上述 FIFO API，不要在应用层直接拼 `0x69` 读取序列。
2. 如果必须访问 `0x69`，需要保证 `ADR_SPI_AD6` bank 管理完全正确，并避免被其他事务打断。
3. 若 SPI 线上挂了多个器件，必须确保片选和总线仲裁严格独占，不能把“共享总线”当作默认安全前提。
4. 遇到 `FIFO_SRC=0xFF`、空满位矛盾、INT1 长时间静默等异常，建议在应用层保留一次轻量探测和重初始化兜底。

## 适用范围

该结论首先来自 CH58x 3 线 SPI 场景，但“优先使用 `0x27` burst 读取 FIFO 样本”的建议对后续 SPI 集成都成立。
