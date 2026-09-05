# SHT30 Driver

`ffl.sht30` 是不绑定 MCU 的 SHT30 温湿度驱动 core。使用
`#include "ffl/sht30.h"`，通过 `ffl_sht30_bind()` 注入
`ffl_transport_t` 与必须提供 `delay_ms` 的 `ffl_time_ops_t`；最终工程也可直接实现这些公开
回调，不需要等待官方 MCU port。

设备对象应零初始化，且每个生命周期只调用一次 `ffl_sht30_bind()`；异步传输未完成时，其它
设备操作和地址修改会返回 `-EBUSY`。

最小适配器和同步/异步完成语义见 [PORTING_GUIDE.md](PORTING_GUIDE.md)。

当前只完成代码迁移与 host 编译检查；真实器件上的 CRC、时序、NACK、timeout 和测量结果仍需按 `docs/HARDWARE_VALIDATION_BACKLOG.md` 回补。
