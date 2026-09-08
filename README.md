# fireflyluo Embedded Libraries

面向资源受限 MCU 的本地 C/C++ 组件仓库。仓库以 **源码组件优先** 的方式组织：应用只引入需要的叶子组件；不再提供按传感器、射频或中间件大类聚合的默认库。

## 当前状态

- 构建基线：C11 + C++17；C API 的公开头文件可直接被 C++ 工程使用。
- 正式组件：基础设施、OSAL、Protothreads、Ad-Hoc 协议 core，以及 SC7A20、SHT30、SHT40、ICP20100、QMC5883P、QMI8658A、ICM42688P 驱动 core。
- `ports/`：CH32 / PY32 适配以及尚未解耦的 OLED、SI24R1、XN297L、XL2400P 板级实现。
- `third_party/`：按技术领域分类保存的上游第三方库；当前预留 `third_party/usb/cherryusb/`。
- 验证状态以 `docs/HARDWARE_VALIDATION_BACKLOG.md` 为准；没有板端实测的内容仅表示 host 或编译验证通过。

## 现有库概述

当前正式库均位于 `components/`，按领域拆分为可独立引入的叶子组件。下面的名称是正式 Xmake target；其中 `object` 和 `headeronly` 入口适合源码复制与资源裁剪，`static` 入口仅针对明确支持双入口的单个设备。

| 领域       | 现有库                                   | 主要职责与入口                                                                                 |
| ---------- | ---------------------------------------- | ---------------------------------------------------------------------------------------------- |
| 基础设施   | `ffl.atomic`                             | 纯头文件的字节级原子操作和短临界区辅助，不绑定 RTOS 或 MCU HAL。                               |
| 基础设施   | `ffl.driver_port`                        | 统一 transport、I2C/SPI 消息、时间、GPIO 与 IRQ 的南向契约，供驱动 core 注入平台能力。         |
| 基础设施   | `ffl.ringbuffer`                         | 固定容量字节环形缓冲区。                                                                       |
| 基础设施   | `ffl.sw_timer`                           | 不绑定 MCU 的软件定时器组件，通过调用方注入临界区和 tick。                                     |
| 算法       | `ffl.impact_displacement`                | 面向加速度采样流的冲击位移算法 core；应用层事件生命周期、RF 帧和业务策略不包含在内。           |
| 运行时     | `ffl.protothreads`                       | 纯头文件协作式状态机宏，不创建线程、栈或时钟。                                                 |
| 运行时     | `ffl.osal`                               | 通用事件、消息、定时器、内存和 OSAL protothread 调度 core，通过 port hooks 接入临界区与 tick。 |
| 协议       | `ffl.adhoc`                              | Ad-Hoc 协议 core，包含帧语义、编解码和协议状态相关的通用实现；射频链路与硬件时序由端口层负责。 |
| 加速度计   | `ffl.sc7a20`                             | SC7A20 源码/object 入口，可按配置裁剪异步能力；另有同源的 `ffl.sc7a20.full` 完整 static 入口。 |
| 环境传感器 | `ffl.sht30`、`ffl.sht40`、`ffl.icp20100` | 温湿度与气压传感器 core；SHT40 额外提供通用 transport facade 和 CH32 适配。                    |
| 磁力计     | `ffl.qmc5883p`                           | QMC5883P 磁力计 core，平台总线由最终工程提供。                                                 |
| IMU        | `ffl.qmi8658a`、`ffl.icm42688p`          | QMI8658A 与 ICM42688P 同步 I2C 6 轴驱动 facade；`ffl.imu_common` 是共享底层支持组件。          |

### 当前不属于正式库的实现

- `ports/ch32/legacy/display/oled/` 中的 OLED 实现仍依赖固定 CH32 HAL 和管脚配置，尚未声明通用显示 target。
- `ports/ch32/legacy/radio/` 中的 SI24R1、XN297L、XL2400P 实现仍是板级射频代码，尚未拆出通用 SPI、CS/CE、IRQ 和 delay port。
- `experimental/` 下的工作区只用于评审或后续吸收，不进入默认组件入口；例如 SC7A20HTR FIFO 工作区仍保持隔离。

host/mock 测试只证明纯函数、接口和 fake-platform 行为；驱动的真实总线、IRQ、FIFO、射频链路、功耗与时序结果，仍以目标板验证记录为准。

## 目录
```
├── build/
├── components/                         # 唯一正式源码来源
│      ├── algorithms                   # 算法库
│      ├── drivers                      # 驱动库
│      ├── foundation
│      ├── protocols                    # 协议库
│      └── runtime                      # 运行时组件库
├── docs/                               # 相关文档
├── examples/
├── ports/
├── third_party/                        # 上游第三方库，保持原目录和许可证
├── tests/
├── xmake-repo/
└── Readme.md         --文档
```



```text
components/                         # 唯一正式源码来源
  foundation/atomic|ringbuffer|sw-timer/
  runtime/osal|protothreads/
  protocols/adhoc/
  drivers/sensor/
    accelerometer/sc7a20/
    environmental/sht30|sht40|icp20100/
    magnetometer/qmc5883p/
    imu/common|qmi8658a|icm42688p/
ports/                              # MCU、板卡和总线适配
  ch32/adhoc|sc7a20|sht40/
  ch32/legacy/display/oled/
  ch32/legacy/radio/si24r1|xn297l|xl2400p/
  py32/osal/
examples/                           # 最小使用示例
third_party/                         # 按领域分类的上游第三方库
  usb/cherryusb/                     # CherryUSB 源码归档与接入说明
tests/                              # 独立 host / 协议测试
xmake-repo/                         # 叶子驱动的本地 xrepo recipe
docs/                               # 重组决策和硬件回补清单
```

`experimental/` 下的实现仅保留作评审或后续吸收，不属于默认构建入口。例如 SC7A20HTR FIFO 工作区被完整保留在 `components/drivers/sensor/accelerometer/sc7a20/experimental/sc7a20htr/`。

## 源码组件：默认入口

需要裁剪功能、检查源码或适配资源受限单片机时，在最终工程显式引入叶子目录：

```lua
includes("third_party/embedded-libs/components/drivers/sensor/accelerometer/sc7a20")

target("firmware")
    set_kind("binary")
    set_languages("c11", "cxx17")
    add_files("src/*.c", "board/*.c")
    add_deps("ffl.sc7a20")
```

组件 core 不包含 `main.h`、MCU HAL 或固定外设句柄。I2C、SPI、GPIO、DMA、IRQ、时钟和临界区由调用工程或 `ports/` 适配层提供。SC7A20 的源码入口保留 `sc7a20_async` option；关闭异步能力可减少编译进来的代码。

## 完整驱动包：快速入口

对于已提供 package 的单设备驱动，可以使用本地 xrepo recipe 获取同源完整静态库：

```lua
add_repositories("fireflyluo path/to/fireflyluo-Embedded-Libs/xmake-repo")
add_requires("ffl-sc7a20")

target("firmware")
    set_kind("binary")
    add_files("src/*.c", "board/sc7a20_port.c")
    add_packages("ffl-sc7a20")
```

“完整”只表示该 **单个设备** 的已支持 core 功能集，不携带固定 MCU HAL。需要关闭 FIFO、异步、缓存或诊断功能时，仍使用源码组件入口。

## 本仓库验证

```powershell
xmake f -P .
xmake test -P .

# 显式构建一个驱动 core 或完整静态库
xmake -P . ffl.sht40
xmake -P . ffl-sc7a20
```

`ffl.sc7a20.unit` 与 `ffl.sc7a20.integration` 使用 mock 总线，不能替代连接真实 SC7A20 后的 CH32 I2C / IRQ / FIFO 验证。

## 组织约定

- `components/` 是源码真相；一个正式叶子组件只保留一份 core 实现。
- 分类目录只做 `includes()` 索引，绝不生成“大类库”。
- `third_party/` 只保存上游库及其版本、许可证和来源说明，不声明 `ffl.*` target。
- `ports/` 允许包含 MCU HAL 和板级资源，但不得反向混入通用组件。
- 第三方库的 MCU、USB IP 和开发板适配放到 `ports/`，可运行接入验证放到 `examples/`。
- `tests/` 放 host 或 mock 验证；板端验证步骤和结果写入硬件回补清单。
- 新驱动先建立独立组件和总线契约，再考虑提供完整 static package。

详细约定见 `components/README.md`，迁移记录见 `docs/REORGANIZATION.md`。
