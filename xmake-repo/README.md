# Local Xmake Repository

本地 package 仓库只提供已经明确需要“快速完整库入口”的单设备驱动。它不再提供 `embedded-sensor-drivers`、`acc-lib`、`imu-lib` 等聚合包。

当前 package：

- `ffl-sc7a20`：SC7A20 完整静态 driver core；不携带 I2C、GPIO、IRQ、DMA 或具体 HAL。

```lua
add_repositories("fireflyluo path/to/fireflyluo-Embedded-Libs/xmake-repo")
add_requires("ffl-sc7a20")

target("firmware")
    set_kind("binary")
    add_files("src/*.c", "board/sc7a20_port.c")
    add_packages("ffl-sc7a20")
```

需要关闭异步 API、检查源代码或按工程裁剪时，请改用源码组件：

```lua
includes("path/to/fireflyluo-Embedded-Libs/components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    add_deps("ffl.sc7a20")
```
