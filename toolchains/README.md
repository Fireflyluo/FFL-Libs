# 工具链选择

仓库使用 Xmake 管理构建。Host 验证使用 MinGW-w64 GCC；固件构建根据 MCU 选择 GNU 交叉工具链。

## Host

```powershell
xmake f -P . -p mingw -a x86_64 -m release
xmake -P .
xmake test -P .
```

## ARM GNU Toolchain

适用于使用 `arm-none-eabi-*` 工具链的 Cortex-M 工程。推荐安装 `arm-gnu-toolchain-15.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip`，并将 `bin` 加入 PATH。

```powershell
xmake f -P . -p cross -a arm --toolchain=arm-none-eabi
xmake -P . ffl.sht40
```

工具链只提供编译器、链接器和基本 ABI；`-mcpu`、FPU、启动文件、链接脚本和 MCU 头文件由具体固件工程提供。需要固定安装位置时可增加 `--sdk="D:/path/to/arm-toolchain"`。

## WCH RISC-V Embedded GCC

适用于 WCH RISC-V 工程。推荐使用 `RISC-V Embedded GCC15.7z`，当前工具链 target 为 `riscv32-wch-elf`。

```powershell
xmake f -P . -p cross -a riscv --toolchain=wch-riscv --sdk="E:/path/to/RISC-V Embedded GCC15"
```

当前配置默认使用 `rv32imacxw` 和 `ilp32`，并兼容 `riscv32-wch-elf-` 与 `riscv-wch-elf-` 前缀。它不是完整的芯片 BSP；启动文件、链接脚本、芯片头文件、HAL 和板级初始化仍需由最终工程提供。

## 配置失败时

先确认 `xmake show -l`、编译器前缀和 SDK 路径，再检查目标工程自己的启动文件和链接脚本。不要把某个厂商 BSP 的启动代码放进通用组件目录。
