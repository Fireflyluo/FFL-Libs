# SHT30 Driver

`ffl.sht30` 是不绑定 MCU 的 SHT30 温湿度驱动 core。使用
`#include "ffl/sht30.h"`，通过 `ffl_sht30_bind()` 注入
`ffl_transport_t` 与必须提供 `delay_ms` 的 `ffl_time_ops_t`；最终工程也可直接实现这些公开
回调，不需要等待官方 MCU port。

设备对象应零初始化，并在首次 `ffl_sht30_init()` 前完成绑定；初始化后重新绑定会返回
`-EBUSY`。异步传输未完成时，其它设备操作和地址修改也会返回 `-EBUSY`。
异步传输正在提交或回调正在处理测量命令/读回时，`ffl_sht30_cancel_async()` 同样返回 `-EBUSY`；
调用方应在回调结束后按需重试取消。
若 transport 在 `xfer()` 内联完成回调，驱动会持有设备直到最外层 `xfer()` 返回；因此用户完成
回调中不能对同一设备发起其它操作，相关调用会返回 `-EBUSY`。

最小适配器和同步/异步完成语义见 [PORTING_GUIDE.md](PORTING_GUIDE.md)。

当前只完成代码迁移与 host 编译检查；真实器件上的 CRC、时序、NACK、timeout 和测量结果仍需按 `docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md` 回补。
