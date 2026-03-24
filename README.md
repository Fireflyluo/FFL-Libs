# fireflyluo-Embedded-Libs

## 项目概述

这是一个**嵌入式单片机常用库集合**，包含传感器驱动、射频驱动、OSAL 框架以及通用工具模块（utils），用于复用、模块化管理硬件与基础组件代码。

### 核心目标
- **模块化设计**：每个模块独立封装，便于移植和复用
- **统一接口**：提供一致的 API 风格和使用方式
- **轻量高效**：适用于资源受限的 MCU 环境
- **文档完善**：每个模块都包含使用说明和示例

## 仓库结构

```text
fireflyluo-Embedded-Libs/
├── sensor/                 # 传感器驱动
│   ├── ICM42688P/          # 6轴IMU（加速度计+陀螺仪）
│   ├── QMI8658A/           # 6轴IMU（加速度计+陀螺仪）
│   ├── sc7a20htr/          # 三轴加速度计
│   ├── sc7a20_new/         # 三轴加速度计（新版）
│   ├── sht40/              # 温湿度传感器
│   ├── sht40_new/          # 温湿度传感器（新版）
│   └── oled/               # OLED 显示驱动
├── RF/                     # 射频芯片驱动
│   ├── SI24R1/             # 2.4G射频芯片
│   ├── xl2400p/            # 2.4G射频芯片
│   └── xn297l/             # 2.4G射频芯片
├── OSAL/                   # OSAL 事件驱动框架库
├── utils/                  # 通用工具模块（ringbuffer/sw_timer）
├── demo/                   # 示例工程
│   └── xmake/              # xmake 构建示例
└── xmake-repo/             # xrepo 本地仓库配置
```

## 模块详情

### 传感器驱动 (sensor/)

| 驱动 | 器件类型 | 主要特性 |
|------|----------|----------|
| **ICM42688P** | 6轴IMU | ±2/±4/±8/±16g加速度计，±250/±500/±1000/±2000dps陀螺仪，I2C/SPI接口 |
| **QMI8658A** | 6轴IMU | ±2/±4/±8/±16g加速度计，±32/±64/±128/±256/±512/±1024/±2048dps陀螺仪，I2C接口 |
| **SC7A20HTR** | 三轴加速度计 | ±2/±4/±8/±16g量程，最高4.434kHz输出率，支持同步/异步操作 |
| **SC7A20 New** | 三轴加速度计 | 可移植重构版本，统一事务总线契约，支持同步/异步操作 |
| **SHT40** | 温湿度传感器 | 温度精度±0.1°C，湿度精度±1.8%RH，I2C接口 |
| **SHT40 New** | 温湿度传感器 | 可移植重构版本，统一总线抽象，支持同步/异步操作 |
| **OLED** | 显示屏驱动 | 常见 0.96 寸 OLED 屏驱动，支持基础绘制与显示 |

### 射频驱动 (RF/)

| 驱动 | 芯片类型 | 主要特性 |
|------|----------|----------|
| **SI24R1** | 2.4G射频 | 2Mbps数据率，-80dBm接收灵敏度，SPI接口 |
| **XL2400P** | 2.4G射频 | 兼容 NRF24L01+ 协议，256 个通道，SPI接口 |
| **XN297L** | 2.4G射频 | 低功耗 2.4G 收发器，多种工作模式，SPI接口 |

### 系统框架 (OSAL/)

- **事件驱动架构**：基于 TI OSAL 的轻量级事件驱动框架
- **任务管理**：支持多任务优先级调度
- **消息队列**：任务间通信机制
- **软件定时器**：单次和周期定时器支持
- **内存管理**：动态内存分配与统计
- **协程支持**：基于 Protothreads 的轻量级协程（可选）

### 工具模块 (utils/)

| 模块 | 说明 | 备注 |
|------|------|------|
| **ringbuffer** | 通用环形缓冲区 | 适合串口、DMA、数据流缓存 |
| **sw_timer** | 时间轮软件定时器 | 已移除 CH32 头文件强依赖，支持平台锁钩子适配 |

## 使用方法

### 1. 模块集成

每个模块目录都包含文档（README.md）和示例，推荐按以下步骤集成：

```bash
# 1. 复制模块文件到您的项目
cp -r sensor/sc7a20htr/ your_project/drivers/
cp -r utils/ your_project/components/

# 2. 查看模块文档
cat sensor/sc7a20htr/README.md
cat utils/README.md
```

### 2. xmake 构建系统

项目支持 xmake 构建，可按需启用/禁用传感器和 utils 模块。

#### 2.1 安装 xmake

如未安装，请访问 [xmake.io](https://xmake.io) 下载并安装。

#### 2.2 快速开始

```bash
# 1. 克隆项目
git clone https://github.com/Fireflyluo/fireflyluo-Embedded-Libs.git

# 2. 进入项目目录
cd fireflyluo-Embedded-Libs

# 3. 编译全部库（sensor + utils）
xmake

# 4. 运行示例程序
cd demo/xmake
xmake
xmake run
```

#### 2.3 在您的项目中使用（xrepo 本地包）

在您的项目 `xmake.lua` 中添加：

```lua
add_repositories("embedded-sensors path/to/fireflyluo-Embedded-Libs/xmake-repo")
add_requires("embedded-sensor-drivers", {
    configs = {
        sensor_icm42688p = true,
        sensor_qmi8658a = true,
        sensor_sc7a20htr = true,
        sensor_sc7a20_new = true,
        sensor_sht40 = true,
        sensor_sht40_new = true,
        sensor_oled = false,
        utils_ringbuffer = true,
        utils_sw_timer = true
    }
})

target("my_app")
    set_kind("binary")
    add_files("src/*.c")
    add_packages("embedded-sensor-drivers")
```

#### 2.4 构建选项

可通过以下选项控制编译模块：

- `--sensor_icm42688p=y/n`：启用/禁用 ICM42688P
- `--sensor_qmi8658a=y/n`：启用/禁用 QMI8658A
- `--sensor_sc7a20htr=y/n`：启用/禁用 SC7A20HTR
- `--sensor_sc7a20_new=y/n`：启用/禁用 SC7A20 New
- `--sensor_sht40=y/n`：启用/禁用 SHT40
- `--sensor_sht40_new=y/n`：启用/禁用 SHT40 New
- `--sensor_oled=y/n`：启用/禁用 OLED
- `--utils_ringbuffer=y/n`：启用/禁用 ringbuffer
- `--utils_sw_timer=y/n`：启用/禁用 sw_timer

示例：仅启用 ICM42688P + SHT40 + ringbuffer

```bash
xmake f --sensor_icm42688p=y --sensor_qmi8658a=n --sensor_sc7a20htr=n --sensor_sc7a20_new=n --sensor_sht40=y --sensor_sht40_new=n --sensor_oled=n --utils_ringbuffer=y --utils_sw_timer=n
xmake
```

### 3. 平台适配

大多数驱动采用分层架构：

- **核心层**：驱动核心逻辑
- **HAL层**：硬件抽象接口
- **平台层**：具体 MCU 平台实现

您需要根据目标平台实现相应的 HAL 接口。

#### 3.1 sw_timer 适配说明

`sw_timer` 支持通过钩子函数接入平台临界区：

```c
#include "sw_timer.h"

extern void __disable_irq(void);
extern void __enable_irq(void);

void app_timer_init(void)
{
    sw_timer_set_lock_hooks(__disable_irq, __enable_irq);
    sw_timer_wheel_init(1); /* 1ms tick */
}
```

如果是单线程且无中断竞争场景，可不设置锁钩子。

## 文档规范

各模块文档建议包含：

- **README.md**：模块概述、特性、配置选项、API说明
- **使用指南**：详细的集成和使用步骤
- **示例代码**：完整应用示例
- **平台适配说明**：移植到不同 MCU 平台的方法

## 免责声明

本库是个人项目，仅供学习参考，请勿用于商业用途。

## 更新记录

#### 2026.1.24
- 增加传感器 -> 加速度计 -> SC7A20 驱动
- 添加传感器 -> 陀螺仪 -> QMI8658A 驱动

#### 2026.1.28
- 添加 RF 芯片 -> 2.4G 射频 -> xn297L 驱动
- 添加 RF 芯片 -> 2.4G 射频 -> xl2400p 驱动
- 添加 RF 芯片 -> 2.4G 射频 -> si24r1 驱动
- 添加传感器 -> 陀螺仪 -> icm42688 驱动

#### 2026.2.28
- 添加传感器 -> 加速度计 -> sc7a20htr 驱动
- 添加传感器 -> 温湿度传感器 -> sht40 驱动
- 添加裸机框架 -> 事件驱动 -> OSAL

#### 2026.3.24
- 添加 `utils` 工具模块构建支持（ringbuffer/sw_timer）
- `sw_timer` 去除 `ch32v20x.h` 强依赖，新增平台临界区钩子适配
