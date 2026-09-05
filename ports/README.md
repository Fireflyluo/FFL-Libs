# Ports

`ports/` 放置与 MCU、开发板和外设资源绑定的适配代码。它可以包含厂商 HAL、`board.h`、DMA/IRQ 设置与固定管脚，但这些内容不得进入 `components/` 的通用 core。

```text
ports/
├── ch32/
│   ├── adhoc/                 # Ad-Hoc CH32 链路适配
│   ├── sc7a20/                # SC7A20 CH32 I2C 适配
│   ├── sht40/                  # SHT40 CH32 ffl_transport 适配
│   └── legacy/
│       ├── display/oled/       # 尚未解耦的 HAL 直连 OLED
│       └── radio/              # 尚未解耦的 HAL 直连射频实现
└── py32/
    └── osal/                   # PY32 OSAL 临界区 / 类型适配
```

`legacy/` 表示“已从旧仓库根目录整理出来、但尚未形成可复用 core”的过渡区域。它不是正式驱动接口，也不会由根 `xmake.lua` 默认构建；后续重构必须先建立 bus、GPIO、delay、IRQ 等显式 port 契约，再创建对应 `components/drivers/...` 叶子组件。

`ch32/sht40/` 是已经具备显式 `ffl_transport_t` 入口的正式适配层，
通过 `ffl_sht40_ch32_transport_init()` 和
`ffl_sht40_ch32_time_init()` 接入 `ffl_sht40_bind()`；它仍需要最终 CH32
工程提供 `drv_i2c.h`、`board.h` 和实际 I2C 初始化。

真实硬件测试项见 `docs/HARDWARE_VALIDATION_BACKLOG.md`。
