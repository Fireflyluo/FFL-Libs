# WCH RISC-V Toolchain

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
xmake f --sdk="E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15"
```

说明：

- 自动兼容 `riscv32-wch-elf-` 与 `riscv-wch-elf-` 两种前缀。
- toolchain 只负责 ABI / 指令集约束，不固化具体协议库或板级移植逻辑。
- 如果工程通过 `xrepo` 引入裸机协议库，建议至少同时关闭 `PIC`；若包安装阶段无法稳定复用自定义 toolchain，可退回 `add_requireconfs(...)` 直接传 ABI flags。
- 对于当前这个 `2.4G_RF` 工程，`adhoc-lib` 包安装阶段仍通过 `add_requireconfs` 直接传 ABI flags；这是为了绕开 xmake 在包安装阶段对工程内自定义 toolchain 复用不稳定的问题。
