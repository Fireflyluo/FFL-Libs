# ICM42688P 驱动说明文档

## 1. 芯片与驱动定位

**ICM42688P** 是 InvenSense（TDK）的一款 6 轴惯性测量单元（IMU），集成 3 轴加速度计和 3 轴陀螺仪。

**驱动定位**：
- 位于 `Lib/IMU-lib/drivers/icm42688p`
- 提供基础传感器配置和数据读取功能
- 通过抽象总线接口（`imu_bus_ops_t`）实现平台无关设计
- 支持 I2C/SPI 通信（由底层总线实现决定）

## 2. 当前目录与文件职责

```
Lib/IMU-lib/
├── include/
│   ├── imu_icm42688p.h    # 驱动对外接口和数据结构定义
│   ├── imu_bus.h          # 总线抽象层接口
│   └── imu_types.h        # 通用数据类型定义
└── drivers/icm42688p/
    ├── include/
    │   └── icm42688_reg.h # 寄存器地址和位域定义（Bank0-4）
    └── src/
        └── imu_icm42688p.c # 驱动实现
```

**文件职责**：
- `icm42688_reg.h`：芯片寄存器真值源，定义所有 Bank 的寄存器地址和位域结构
- `imu_icm42688p.c`：驱动核心实现，包括初始化、配置和数据读取
- `imu_icm42688p.h`：对外 API 接口和配置数据结构
- `imu_bus.h`：总线操作抽象，支持异步传输和回调
- `imu_types.h`：原始数据和转换后样本的通用结构体

## 3. 当前支持的能力

### 已实现功能

1. **设备探测**：通过 WHO_AM_I 寄存器验证芯片 ID
2. **软复位**：通过 DEVICE_CONFIG 寄存器触发芯片复位
3. **时钟配置**：设置 PLL 或内部 RC 振荡器
4. **电源管理**：独立控制加速度计和陀螺仪的工作模式
5. **传感器配置**：
   - 加速度计：满量程范围（±2g/4g/8g/16g）、输出数据率（ODR）
   - 陀螺仪：满量程范围（±15.625dps 至 ±2000dps）、输出数据率（ODR）
6. **数据读取**：
   - 原始数据读取（14 字节连续读取：温度 + 加速度计 + 陀螺仪）
   - 转换后的样本读取（国际单位制：m/s²、rad/s、°C）
7. **Bank 切换**：支持访问 Bank0-4 的寄存器
8. **寄存器读写**：通用寄存器读写接口

### 未实现/未覆盖功能

1. **FIFO 操作**：FIFO 配置、读取、水位中断等
2. **中断配置**：INT1/INT2 引脚配置、中断源映射
3. **APEX 功能**：计步器、倾斜检测、抬手唤醒、敲击检测、运动唤醒
4. **自检功能**：陀螺仪和加速度计的自检测试
5. **时间戳功能**：内部时间戳计数器配置和读取
6. **高级滤波器配置**：UI 滤波器阶数、带宽详细配置
7. **低功耗优化**：低功耗模式下的详细配置
8. **FSYNC 配置**：外部帧同步功能

## 4. 初始化路径、Bank 切换与关键寄存器

### 初始化路径

`imu_icm42688p_init()` 函数执行以下初始化序列：

1. **参数验证**：检查设备指针和配置有效性
2. **设备探测**：读取 WHO_AM_I 寄存器（0x75）验证芯片 ID
3. **软复位**：写入 DEVICE_CONFIG 寄存器（0x11）的 SOFT_RESET 位，等待 2ms
4. **时钟配置**：写入 INTF_CONFIG1 寄存器（0x4D），设置 CLKSEL=1（优先使用 PLL）
5. **电源管理**：写入 PWR_MGMT0 寄存器（0x4E），设置加速度计和陀螺仪模式
6. **等待稳定**：延迟 10ms 等待传感器启动
7. **陀螺仪配置**：写入 GYRO_CONFIG0 寄存器（0x4F）
8. **加速度计配置**：写入 ACCEL_CONFIG0 寄存器（0x50）

### Bank 切换机制

ICM42688P 使用 Bank 选择寄存器（0x76）切换寄存器 Bank：
- Bank0：传感器数据、配置、电源管理等主要寄存器
- Bank1：陀螺仪静态配置、自测数据
- Bank2：加速度计静态配置、自测数据
- Bank4：APEX 配置、WOM 阈值、中断源扩展

驱动通过 `imu_icm42688p_switch_bank()` 函数自动管理 Bank 切换，记录当前 Bank 状态避免重复切换。

### 关键寄存器

| 寄存器 | 地址 | Bank | 功能 |
|--------|------|------|------|
| WHO_AM_I | 0x75 | 0 | 芯片 ID 读取 |
| DEVICE_CONFIG | 0x11 | 0 | 软复位、SPI 模式 |
| PWR_MGMT0 | 0x4E | 0 | 传感器电源模式 |
| GYRO_CONFIG0 | 0x4F | 0 | 陀螺仪 ODR 和满量程 |
| ACCEL_CONFIG0 | 0x50 | 0 | 加速度计 ODR 和满量程 |
| INTF_CONFIG1 | 0x4D | 0 | 时钟源选择 |
| BANK_SEL | 0x76 | 0 | Bank 选择 |

## 5. 采样读取路径

### 原始数据读取

`imu_icm42688p_read_raw()` 函数：
1. 从 TEMP_DATA1（0x1D）开始连续读取 14 字节
2. 解析为：
   - 温度：2 字节有符号整数
   - 加速度计：X/Y/Z 各 2 字节有符号整数
   - 陀螺仪：X/Y/Z 各 2 字节有符号整数

### 数据转换

`imu_icm42688p_read_sample()` 函数将原始数据转换为国际单位：
1. **加速度计转换**：原始值 × (1/LSB_per_g) × 9.80665 → m/s²
2. **陀螺仪转换**：原始值 × (1/LSB_per_dps) × (π/180) → rad/s
3. **温度转换**：原始值 / 132.48 + 25.0 → °C

**满量程对应的 LSB 值**：

| 加速度计满量程 | LSB/g | 陀螺仪满量程 | LSB/dps |
|----------------|-------|--------------|---------|
| ±16g | 2048 | ±2000dps | 16.384 |
| ±8g | 4096 | ±1000dps | 32.768 |
| ±4g | 8192 | ±500dps | 65.536 |
| ±2g | 16384 | ±250dps | 131.072 |
| - | - | ±125dps | 262.144 |
| - | - | ±62.5dps | 524.288 |
| - | - | ±31.25dps | 1048.576 |
| - | - | ±15.625dps | 2097.152 |

## 6. 对外接口摘要

### 核心函数

```c
// 初始化驱动
int imu_icm42688p_init(imu_icm42688p_t *dev, const imu_icm42688p_cfg_t *cfg);

// 探测设备（读取 WHO_AM_I）
int imu_icm42688p_probe(imu_icm42688p_t *dev, uint8_t *who_am_i);

// 软复位
int imu_icm42688p_soft_reset(imu_icm42688p_t *dev);

// 读取原始数据
int imu_icm42688p_read_raw(imu_icm42688p_t *dev, imu_raw_sample_t *raw);

// 读取转换后的样本
int imu_icm42688p_read_sample(imu_icm42688p_t *dev, imu_sample_t *sample);

// 配置陀螺仪
int imu_icm42688p_set_gyro_config(imu_icm42688p_t *dev,
                                 icm42688_gyro_fs_t fs,
                                 icm42688_odr_t odr);

// 配置加速度计
int imu_icm42688p_set_accel_config(imu_icm42688p_t *dev,
                                  icm42688_accel_fs_t fs,
                                  icm42688_odr_t odr);

// 通用寄存器读写
int imu_icm42688p_read_reg(imu_icm42688p_t *dev,
                          icm42688_bank_t bank,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len);

int imu_icm42688p_write_reg(imu_icm42688p_t *dev,
                           icm42688_bank_t bank,
                           uint8_t reg,
                           const uint8_t *data,
                           uint16_t len);
```

### 配置结构体

```c
typedef struct {
    uint8_t addr;                    // I2C 地址或 SPI 设备选择
    icm42688_sensor_mode_t accel_mode; // 加速度计模式
    icm42688_sensor_mode_t gyro_mode;  // 陀螺仪模式
    icm42688_accel_fs_t accel_fs;      // 加速度计满量程
    icm42688_odr_t accel_odr;          // 加速度计 ODR
    icm42688_gyro_fs_t gyro_fs;        // 陀螺仪满量程
    icm42688_odr_t gyro_odr;           // 陀螺仪 ODR
} imu_icm42688p_cfg_t;
```

### 默认配置

驱动提供默认配置 `g_imu_icm42688p_default_cfg`：
- 地址：0x69（SDO=VDDIO）
- 加速度计：低噪声模式，±4g，100Hz
- 陀螺仪：低噪声模式，±500dps，100Hz

## 7. 已知限制与注意事项

### 重要：WHO_AM_I 期望值不一致

**当前状态**：
- 寄存器头文件 `icm42688_reg.h` 定义：`ICM42688P_DEVICE_ID 0x68U`（第 40 行）
- 驱动代码 `imu_icm42688p.c` 期望：`IMU_ICM42688P_EXPECTED_ID 0x47u`（第 20 行）

**影响**：
- 使用 `imu_icm42688p_probe()` 函数时，如果芯片实际返回 0x68（符合数据手册），驱动会返回 `-ENODEV` 错误
- 这是一个已知的不一致，需要根据实际芯片型号确认正确的 ID 值

**建议**：
1. 查阅所用 ICM42688P 芯片的数据手册确认 WHO_AM_I 默认值
2. 如果实际芯片返回 0x68，需要修改驱动中的期望值
3. 如果实际芯片返回 0x47，需要修改寄存器头文件的定义

### 其他限制

1. **时间戳功能未实现**：虽然寄存器定义完整，但驱动未提供时间戳配置和读取接口
2. **FIFO 未支持**：FIFO 相关寄存器已定义，但驱动未实现 FIFO 操作
3. **中断未配置**：INT1/INT2 引脚配置、中断源映射等未实现
4. **APEX 功能缺失**：计步器、倾斜检测等高级功能未实现
5. **自检功能缺失**：虽然自检配置寄存器已定义，但驱动未提供自检接口
6. **滤波器配置有限**：仅支持基本 ODR 和满量程配置，高级滤波器参数未暴露
7. **无 FIFO 水位中断**：不支持基于 FIFO 水位的数据批量读取
8. **无多设备支持**：驱动结构体设计支持多实例，但需确保总线操作线程安全

### 使用建议

1. **初始化顺序**：必须先调用 `imu_icm42688p_init()` 完成初始化后再读取数据
2. **错误处理**：所有函数返回 0 表示成功，负数表示错误（参考 errno.h）
3. **总线实现**：需实现 `imu_bus_ops_t` 接口，确保 `xfer` 函数支持异步传输
4. **延迟函数**：需提供 `delay_ms` 回调函数，驱动内部使用它进行必要的等待
5. **Bank 访问**：访问非 Bank0 寄存器时，驱动会自动切换 Bank，无需手动管理

### 验证要点

1. **芯片 ID 验证**：初始化前建议先调用 `imu_icm42688p_probe()` 验证芯片连接
2. **数据就绪检查**：虽然驱动未实现中断，但可通过轮询 INT_STATUS 寄存器检查数据就绪
3. **配置验证**：配置后可通过读取对应配置寄存器验证设置是否生效

---

**文档版本**：1.0  
**基于代码版本**：当前仓库 HEAD  
**最后更新**：2026 年 5 月 28 日