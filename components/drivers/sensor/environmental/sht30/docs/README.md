# SHT30 Driver

`ffl.sht30` 是不绑定 MCU 的 SHT30 温湿度驱动 core。最终工程需要按公开接口注入 I2C 读写、延时和错误处理策略。

当前只完成代码迁移与 host 编译检查；真实器件上的 CRC、时序、NACK、timeout 和测量结果仍需按 `docs/HARDWARE_VALIDATION_BACKLOG.md` 回补。
