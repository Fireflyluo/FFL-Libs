# fireflyluo-Embedded-Libs

## 项目概述

这是一个**嵌入式单片机常用驱动库集合**，旨在为自己提供复用、模块化的硬件驱动代码，简化传感器、射频芯片等外设。

### 核心目标
- **模块化设计**：每个驱动独立封装，便于移植和复用
- **统一接口**：提供一致的API风格和使用方式
- **轻量高效**：适用于资源受限的MCU环境
- **文档完善**：每个驱动都包含详细的使用说明和示例

## 仓库结构

```
fireflyluo-Embedded-Libs/
├── sensor/                 # 传感器驱动
│   ├── ICM42688P/         # 6轴IMU（加速度计+陀螺仪）
│   ├── QMI8658A/          # 6轴IMU（加速度计+陀螺仪）  
│   ├── sc7a20htr/         # 三轴加速度计
│   └── sht40/             # 温湿度传感器
├── RF/                     # 射频芯片驱动
│   ├── SI24R1/            # 2.4G射频芯片
│   ├── xl2400p/           # 2.4G射频芯片
│   └── xn297l/            # 2.4G射频芯片
├── OSAL/                   # OSAL事件驱动框架库
└── demo/                   # 示例工程
    └── xmake/             # xmake构建示例
```

## 驱动模块详情

### 传感器驱动 (sensor/)

| 驱动 | 器件类型 | 主要特性 |
|------|----------|----------|
| **ICM42688P** | 6轴IMU | ±2/±4/±8/±16g加速度计，±250/±500/±1000/±2000dps陀螺仪，I2C/SPI接口 |
| **QMI8658A** | 6轴IMU | ±2/±4/±8/±16g加速度计，±32/±64/±128/±256/±512/±1024/±2048dps陀螺仪，I2C接口 |
| **SC7A20HTR** | 三轴加速度计 | ±2/±4/±8/±16g量程，最高4.434kHz输出率，支持同步/异步操作 |
| **SHT40** | 温湿度传感器 | 温度精度±0.1°C，湿度精度±1.8%RH，I2C接口 |

### 射频驱动 (RF/)

| 驱动 | 芯片类型 | 主要特性 |
|------|----------|----------|
| **SI24R1** | 2.4G射频 | 2Mbps数据率，-80dBm接收灵敏度，SPI接口 |
| **XL2400P** | 2.4G射频 | 兼容NRF24L01+协议，256个通道，SPI接口 |
| **XN297L** | 2.4G射频 | 低功耗2.4G收发器，多种工作模式，SPI接口 |

### 系统框架 (OSAL/)

- **事件驱动架构**：基于TI OSAL的轻量级事件驱动框架
- **任务管理**：支持多任务优先级调度
- **消息队列**：任务间通信机制
- **软件定时器**：单次和周期定时器支持
- **内存管理**：动态内存分配与统计
- **协程支持**：基于Protothreads的轻量级协程（可选）

## 使用方法

### 1. 驱动集成
每个驱动目录都包含完整的文档（README.md）和使用指南，按照以下步骤集成：

```bash
# 1. 复制驱动文件到您的项目
cp -r sensor/sc7a20htr/ your_project/drivers/

# 2. 查看驱动文档
cat sensor/sc7a20htr/README.md

# 3. 实现平台抽象层（如需要）
# 参考 platform/ 目录下的示例
```

### 2. xmake构建系统
本项目现已支持 **xmake** 构建系统，您可以轻松地将传感器驱动库集成到您的项目中。

#### 2.1 安装xmake
如果您还没有安装xmake，请访问 [xmake.io](https://xmake.io) 下载并安装。

#### 2.2 快速开始
```bash
# 1. 克隆项目
git clone https://github.com/Fireflyluo/fireflyluo-Embedded-Libs.git

# 2. 进入项目目录
cd fireflyluo-Embedded-Libs

# 3. 编译所有传感器驱动库
xmake

# 4. 运行示例程序
cd demo/xmake
xmake
xmake run
```

#### 2.3 在您的项目中使用
在您的项目xmake.lua中添加以下内容：

```lua
-- 添加对传感器驱动库的依赖
add_requires("embedded-sensor-drivers")

target("my_app")
    set_kind("binary")
    add_files("src/*.c")
    add_packages("embedded-sensor-drivers")
    
    -- 可选择性地启用特定传感器驱动
    add_defines("USE_ICM42688P")
    add_defines("USE_QMI8658A")
    add_defines("USE_SC7A20HTR")
    add_defines("USE_SHT40")
    add_defines("USE_OLED")
```

#### 2.4 构建选项
项目提供了以下构建选项，可以在编译时启用或禁用特定的传感器驱动：
- `--enable-sensor_icm42688p=true/false` : 启用/禁用ICM42688P驱动
- `--enable-sensor_qmi8658a=true/false` : 启用/禁用QMI8658A驱动
- `--enable-sensor_sc7a20htr=true/false` : 启用/禁用SC7A20HTR驱动
- `--enable-sensor_sht40=true/false` : 启用/禁用SHT40驱动
- `--enable-sensor_oled=true/false` : 启用/禁用OLED驱动

例如，只启用ICM42688P和SHT40驱动：
```bash
xmake f --enable-sensor_icm42688p=true --enable-sensor_qmi8658a=false --enable-sensor_sc7a20htr=false --enable-sensor_sht40=true --enable-sensor_oled=false
xmake
```

### 3. 平台适配
大多数驱动采用分层架构：
- **核心层**：驱动核心逻辑
- **HAL层**：硬件抽象接口
- **平台层**：具体MCU平台实现

您需要根据目标平台实现相应的HAL接口。


## 文档规范

每个驱动模块都遵循统一的文档规范，包含：
- **README.md**：驱动概述、特性、配置选项、API说明
- **使用指南**：详细的集成和使用步骤
- **示例代码**：完整的应用示例
- **平台适配说明**：如何移植到不同MCU平台

## 免责声明
本库是个人项目，仅供学习参考，请勿用于商业用途。


## 更新记录

#### 2026.1.24  
- 增加传感器 -> 加速度计 -> SC7A20 驱动
- 添加传感器 -> 陀螺仪 -> QMI8658A 驱动


#### 2026.1.28  
- 添加RF芯片 -> 2.4G射频 -> xn297L驱动
- 添加RF芯片 -> 2.4G射频 -> xl2400p驱动
- 添加RF芯片 -> 2.4G射频 -> si24r1驱动
- 添加传感器 -> 陀螺仪 -> icm42688 驱动

#### 2026.2.28  
- 添加传感器 -> 加速度计 -> sc7a20htr 驱动
- 添加传感器 -> 温湿度传感器 -> sht40 驱动
- 添加裸机框架 -> 事件驱动 -> OSAL
