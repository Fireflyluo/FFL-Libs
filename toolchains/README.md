# 工具链选择

仓库使用 Xmake 管理构建。Host 验证使用 MinGW-w64 GCC；固件构建根据 MCU 选择 GNU 交叉工具链。

> **工具链安装包（`.zip`/`.7z`）不入仓库**，请按本文档下载并安装到本机。
> 仓库内只保留工具链定义（`arm-none-eabi.lua`、`wch-riscv.lua`）与本文档。

## Host

```powershell
xmake f -P . -p mingw -a x86_64 -m release
xmake -P .
xmake test -P .
```

## 交叉工具链：总览

| 工具链                  | 适用目标                         | 前缀                                  | 下载来源                 |
| ----------------------- | -------------------------------- | ------------------------------------- | ------------------------ |
| ARM GNU Toolchain       | Cortex-M（本仓库示例用 STM32F1） | `arm-none-eabi-`                      | ARM 官网（见下）         |
| WCH RISC-V Embedded GCC | WCH RISC-V（CH32V 等）           | `riscv32-wch-elf-` / `riscv-wch-elf-` | 沁恒 / MounRiver（见下） |

安装包解压后通常把 `<root>/bin` 加入 `PATH`，或用 Xmake 的 `--sdk=<root>` 显式指向，两种方式仓库的 toolchain 定义都支持。

## ARM GNU Toolchain

### 获取与安装

1. 打开官方下载页：<https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>
2. 选择 Windows 平台的 `x86_64` 包，当前推荐（本仓库实编译验证过 GCC 15.2/15.3）：
   `arm-gnu-toolchain-15.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip`
3. 解压，例如到 `D:/toolchains/arm-gnu-toolchain-15.3.rel1-mingw-w64-x86_64-arm-none-eabi/`
4. 把解压目录下的 `bin` 加入 `PATH`（或见下方 `--sdk` 方式）。

验证：

```powershell
arm-none-eabi-gcc --version
```

### Xmake 使用

工具链在 PATH 中时直接：

```powershell
xmake f -P . -p cross -a arm --toolchain=arm-none-eabi
xmake -P . ffl.sht40
```

未加入 PATH 时用 `--sdk` 指向解压根目录（toolchain 定义会取其 `bin`）：

```powershell
xmake f -P . -p cross -a arm --toolchain=arm-none-eabi --sdk="D:/toolchains/arm-gnu-toolchain-15.3.rel1-mingw-w64-x86_64-arm-none-eabi"
```

> 参考：本仓库 `examples/stm32-base-driver` 即用该工具链交叉编译 Cortex-M3 固件，见其 `readme.md`。

工具链只提供编译器、链接器和基本 ABI；`-mcpu`、FPU、启动文件、链接脚本和 MCU 头文件由具体固件工程提供。

## WCH RISC-V Embedded GCC

### 获取与安装

WCH 的 RISC-V 工具链随 **MounRiver Studio**（IDE）一起发布，也可单独获取：

- MounRiver Studio：<https://www.mounriver.com>（下载后工具链位于安装目录，见下方路径）
- 沁恒官网：<https://www.wch.cn>（MounRiver / 工具链相关下载入口）

MounRiver 内置工具链的典型目录（开发机实测路径示例）：

```text
<MounRiver安装目录>/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15/
```

也可以使用独立下载的 `RISC-V Embedded GCC15.7z`（GCC 15.2，target `riscv32-wch-elf`）解压后自行放置。

### Xmake 使用

```powershell
xmake f -P . -p cross -a riscv --toolchain=wch-riscv --sdk="E:/path/to/RISC-V Embedded GCC15"
```

如果随 MounRiver 使用，`--sdk` 指向上面那个 `.../RISC-V Embedded GCC15` 目录即可。

当前配置默认使用 `rv32imacxw` 和 `ilp32`，并兼容 `riscv32-wch-elf-` 与 `riscv-wch-elf-` 前缀。它不是完整的芯片 BSP；启动文件、链接脚本、芯片头文件、HAL 和板级初始化仍需由最终工程提供。

## 烧录 / 调试（可选）

需要烧录或在线调试时，可使用 OpenOCD（本仓库此前仅在本机以压缩包形式留档，不入库）：

- 获取：<https://openocd.org/> 或芯片厂商/IDE 随附版本
- 例（ST-Link + STM32F1）：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program build/cross/arm/release/<target>.bin 0x08000000 verify reset exit"
```

具体命令以你的调试器与目标板为准。

## 配置失败时

先确认 `xmake show -l`、编译器前缀和 SDK 路径，再检查目标工程自己的启动文件和链接脚本。不要把某个厂商 BSP 的启动代码放进通用组件目录。

