# CherryUSB

CherryUSB 是上游 USB Host/Device 协议栈。本目录用于保存 CherryUSB 的上游源码或 Git submodule，不声明本仓库自己的 `ffl.*` 组件。

- 上游仓库：https://github.com/cherry-embedded/CherryUSB
- 官方文档：https://cherryusb.cherry-embedded.org/
- 许可证：Apache-2.0，以随上游源码提供的 `LICENSE` 为准
- 当前状态：已纳入上游源码，固定为 `v1.6.1`；尚未接入根构建，也尚未完成目标板验证

## 纳入源码

源码以固定 release tag 的 Git submodule 纳入：

```powershell
git submodule add https://github.com/cherry-embedded/CherryUSB.git third_party/usb/cherryusb/upstream
Set-Location third_party/usb/cherryusb/upstream
git checkout v1.6.1
Set-Location ../../../..
```

当前 submodule 提交：`c9625ffa773ad10b8824d1b5361bca2ccc1f3d1e`。

上游源码放在 `upstream/`，本目录的说明文件不参与编译。后续若需要 Xmake 入口，应在 `examples/cherryusb/` 或对应板级适配目录中显式引用，不加入根 `xmake.lua`。

## 本仓库适配边界

```text
third_party/usb/cherryusb/upstream/   # CherryUSB 原始源码
ports/cherryusb/<mcu>/<board>/        # MCU、USB IP、时钟、IRQ、DMA、GPIO 适配
examples/cherryusb/<board>/           # 最小可运行示例和验证工程
```

升级 CherryUSB 时只更新 `upstream/` 的版本，并重新验证相关 port 和 example。不要直接修改上游源码来适配本仓库；确需补丁时，应单独记录补丁原因和上游版本。
