# 选择组件

`components/` 提供可独立接入的源码组件。应用只引入需要的叶子 target，不会因为选择一个领域而自动编译全部驱动。

## 按需求选择

| 需求                           | target                                         | 入口                                               |
| ------------------------------ | ---------------------------------------------- | -------------------------------------------------- |
| 原子操作、环形缓冲、软件定时器 | `ffl.atomic`、`ffl.ringbuffer`、`ffl.sw_timer` | `components/foundation/`                           |
| 总线、时间、GPIO、IRQ 能力     | `ffl.driver_port`                              | `components/foundation/driver-port/`               |
| 协作式状态机                   | `ffl.protothreads`                             | `components/runtime/protothreads/`                 |
| 事件、消息、定时器和内存抽象   | `ffl.osal`                                     | `components/runtime/osal/`                         |
| Ad-Hoc 协议 core               | `ffl.adhoc`                                    | `components/protocols/adhoc/`                      |
| 冲击位移算法                   | `ffl.impact_displacement`                      | `components/algorithms/impact-displacement/`       |
| 加速度计                       | `ffl.sc7a20`                                   | `components/drivers/sensor/accelerometer/sc7a20/`  |
| 温湿度 / 气压                  | `ffl.sht30`、`ffl.sht40`、`ffl.icp20100`       | `components/drivers/sensor/environmental/`         |
| 磁力计                         | `ffl.qmc5883p`                                 | `components/drivers/sensor/magnetometer/qmc5883p/` |
| IMU                            | `ffl.qmi8658a`、`ffl.icm42688p`                | `components/drivers/sensor/imu/`                   |

## 接入步骤

1. 在应用的 `xmake.lua` 中 `includes()` 目标组件目录。
2. 在固件 target 中使用 `add_deps("ffl.<component>")`。
3. 为驱动提供需要的 `ffl.driver_port` 能力，或使用 `ports/` 中已有适配。
4. 阅读组件目录下的 `docs/` 和公开头文件，确认配置、资源和错误码。

```lua
includes("path/to/components/foundation/ringbuffer")
includes("path/to/components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    add_files("src/*.c", "board/*.c")
    add_deps("ffl.ringbuffer", "ffl.sc7a20")
```

## 源码入口与完整 package

源码 target 适合需要裁剪 Flash/RAM、检查实现或修改编译选项的工程。SC7A20 另有 `ffl.sc7a20.full`，用于不需要裁剪的完整静态入口；本地 package 名称为 `ffl-sc7a20`。两者都不包含固定 MCU HAL。

## 边界

组件 core 不包含 `main.h`、厂商 HAL、固定 GPIO、全局 I2C/SPI 句柄或板级业务逻辑。未在真实硬件上验证的组件，只能视为 host/mock 或编译验证通过；维护者记录见 [docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md](../docs/maintainer/HARDWARE_VALIDATION_BACKLOG.md)。
