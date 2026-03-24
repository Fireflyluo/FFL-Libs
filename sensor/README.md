# 嵌入式传感器驱动库

本目录包含多种嵌入式传感器驱动的 xmake 构建配置，支持多平台编译（Windows、ARM、RISC-V）。

## 支持的传感器

| 传感器型号 | 类型 | 接口 | 说明 |
|-----------|------|------|------|
| ICM42688P | 6 轴 IMU | SPI | TDK 高性能运动跟踪传感器 |
| QMI8658A | 6 轴 IMU | I2C | 高通 6 轴惯性测量单元 |
| SC7A20HTR | 3 轴加速度计 | I2C | 芯科低功耗加速度计 |
| SHT40 | 温湿度传感器 | I2C | Sensirion 高精度温湿度传感器 |
| OLED | 显示屏 | I2C/SPI | 0.96 寸 OLED 显示屏驱动 |

## 目录结构

```
sensor/
├── xmake.lua              # 主构建配置（支持 Windows 原生编译）
├── xmake_arm.lua          # ARM 交叉编译配置
├── xmake_riscv.lua        # RISC-V 交叉编译配置
├── ICM42688P/             # ICM42688P 驱动
├── QMI8658A/              # QMI8658A 驱动
├── sc7a20htr/             # SC7A20HTR 驱动
├── sht40/                 # SHT40 驱动
└── oled/                  # OLED 显示屏驱动
```

## 使用方法

### 1. Windows 平台编译（用于 PC 测试）

```bash
cd sensor
xmake
```

### 2. ARM 平台编译（使用 ARM GCC）

```bash
cd sensor
xmake f -p cross -a arm --toolchain=gnu-rm -F xmake_arm.lua
xmake -F xmake_arm.lua
```

编译输出位于 `build/cross/arm/release/` 目录。

### 3. RISC-V 平台编译（使用沁恒 RISC-V 工具链）

```bash
cd sensor
xmake f -p riscv --toolchain=riscv-wch-gcc -F xmake_riscv.lua
xmake -F xmake_riscv.lua
```

编译输出位于 `build/riscv/x64/release/` 目录。

## 在其他 xmake 项目中使用

### 方式 1：添加子目录

在你的 xmake 项目的 `xmake.lua` 中添加：

```lua
-- 添加传感器驱动库
includes("path/to/sensor")

target("my_project")
    set_kind("binary")
    add_deps("icm42688p")      -- 使用 ICM42688P 驱动
    add_deps("qmi8658a")       -- 使用 QMI8658A 驱动
    add_deps("sc7a20htr")      -- 使用 SC7A20HTR 驱动
    add_deps("sht40")          -- 使用 SHT40 驱动
    add_deps("oled")           -- 使用 OLED 驱动
```

### 方式 2：使用 xrepo（推荐）

`lua
add_repositories(\"embedded-sensors path/to/fireflyluo-Embedded-Libs/xmake-repo\")
add_requires(\"embedded-sensor-drivers\", {
    configs = {
        sensor_icm42688p = true,
        sensor_qmi8658a = true,
        sensor_sc7a20htr = true,
        sensor_sht40 = true,
        sensor_oled = false
    }
})
`

Then link it to your target:

`lua
target(\"my_project\")
    set_kind(\"binary\")
    add_files(\"src/*.c\")
    add_packages(\"embedded-sensor-drivers\")
`
## 硬件抽象层（HAL）接口

所有驱动都使用弱函数定义硬件抽象层接口，用户需要在自己的项目中实现这些接口。

### I2C 设备（SHT40、QMI8658A、SC7A20HTR）

```c
// I2C 写入函数
uint8_t SHT40_I2C_Write(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);
uint8_t QMI8658A_I2C_Write(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);

// I2C 读取函数
uint8_t SHT40_I2C_Read(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);
uint8_t QMI8658A_I2C_Read(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len);

// 延时函数
void SHT40_Delay(uint32_t ms);
void HAL_Delay(uint32_t ms);
```

### SPI 设备（ICM42688P、OLED）

```c
// SPI 片选控制
void ICM42688P_CS_Low(void);
void ICM42688P_CS_High(void);

// SPI 读写函数
uint8_t ICM42688P_SPI_Write_Read(uint8_t tx_data);
void ICM42688P_Write_Reg(uint8_t reg, uint8_t data);
uint8_t ICM42688P_Read_Reg(uint8_t reg);

// 延时函数
void ICM42688P_Delay(uint32_t ms);
```

### OLED 特定接口

```c
// 命令/数据选择
void OLED_CMD_Data(uint8_t value);
void OLED_Write_Data(uint8_t data);

// 复位控制
void OLED_RST_Low(void);
void OLED_RST_High(void);
```

## 编译输出

编译成功后，将在 `build/` 目录下生成静态库文件：

- ARM: `build/cross/arm/release/lib<driver>.a`
- RISC-V: `build/riscv/x64/release/lib<driver>.a`
- Windows: `build/windows/x64/release/<driver>.lib`

## 注意事项

1. 所有驱动都移除了对特定 MCU 厂商 HAL 库的依赖，使用弱函数提供硬件抽象接口
2. 用户需要根据自己的使用的 MCU 平台实现相应的 HAL 函数
3. 编译前请确保已安装对应的工具链并正确配置环境变量
4. RISC-V 工具链路径可在 `xmake_riscv.lua` 中修改

## 许可证

MIT License
