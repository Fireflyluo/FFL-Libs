# GNU GCC Toolchains

仓库的宿主编译基线为 MinGW-w64 GCC，不使用 MSVC 作为组件质量基线。组件仍保持 C11 / C++17；实际 MCU 工程需要根据目标芯片显式选择 GNU 交叉工具链。

## Host GCC validation

```bash
xmake f -P . -p mingw -a x86_64 -m release
xmake -P . ffl.sht30.test
xmake test -P .
```

根目录默认目标同样选择 `mingw`，因此通常直接执行 `xmake f -P .` 即可。

## Arm GNU Toolchain

`arm-none-eabi.lua` 优先使用 `PATH` 中的 `arm-none-eabi-*` 工具。当前开发机可解析到 `D:/APP/path/arm-gcc-15.2`，但仓库不固化该本地绝对路径。

```bash
xmake f -P . -p cross -a arm --toolchain=arm-none-eabi
xmake -P . ffl.sht30
```

若需要固定 SDK 版本，显式传入工具链根目录：

```bash
xmake f -P . -p cross -a arm --toolchain=arm-none-eabi --sdk="D:/APP/path/arm-gcc-15.2"
```

该 toolchain 仅绑定 GNU 工具程序；`-mcpu`、FPU、浮点 ABI、链接脚本与启动文件必须由具体 MCU/板级工程提供。

## WCH RISC-V Embedded GCC

`wch-riscv.lua` 是给 WCH RISC-V Embedded GCC 准备的可复用 `xmake` toolchain 定义。

最小接入示例：

```lua
includes("path/to/toolchains/wch-riscv.lua")

set_plat("cross")
set_arch("riscv")
set_toolchains("wch-riscv", {configs = {sdkdir = "E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15"}})
```

也可以走命令行配置：

```bash
xmake f -P . -p cross -a riscv --toolchain=wch-riscv --sdk="E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15"
```

说明：

- 自动兼容 `riscv32-wch-elf-` 与 `riscv-wch-elf-` 两种前缀。
- toolchain 只负责 ABI / 指令集约束，不固化具体协议库或板级移植逻辑。
- 如果工程通过 `xrepo` 引入裸机协议库，建议至少同时关闭 `PIC`；若包安装阶段无法稳定复用自定义 toolchain，可退回 `add_requireconfs(...)` 直接传 ABI flags。
- 对于当前这个 `2.4G_RF` 工程，`adhoc-lib` 包安装阶段仍通过 `add_requireconfs` 直接传 ABI flags；这是为了绕开 xmake 在包安装阶段对工程内自定义 toolchain 复用不稳定的问题。
