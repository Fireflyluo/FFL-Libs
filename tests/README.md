# 测试与验证

仓库中的测试分为三类，结论范围不同，不能互相替代。

## Host / mock 测试

在 MinGW-w64 GCC 上运行，不需要 MCU 或传感器实物：

```powershell
xmake f -P . -p mingw -a x86_64 -m release
xmake test -P .
```

这类测试适合检查纯算法、编解码、寄存器解析、错误码、C/C++ 头文件兼容和 fake transport 行为。

## 交叉编译检查

交叉编译只能说明源码、工具链和目标架构参数可以通过编译和链接检查，不能说明 GPIO、总线时序或 IRQ 在板上正确。ARM 和 WCH 命令见 [toolchains/README.md](../toolchains/README.md)。

## 真实硬件验证

硬件验证需要记录目标板、芯片、时钟、总线频率、固件版本、测量方法和日志。重点包括 WHO_AM_I、实际总线波形、复位、连续采样、异常恢复、中断和长时间稳定性。

当前哪些项目已经有自动验证、哪些仍待板端回补，见 [docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md](../docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md)。
