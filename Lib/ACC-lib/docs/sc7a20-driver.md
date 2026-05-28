# SC7A20 加速度计驱动说明文档

## 1. 驱动定位与目录

本驱动为 SC7A20HTR 三轴加速度计提供硬件抽象层，支持同步与异步数据读取，适用于嵌入式实时系统。

### 目录结构
```
Lib/ACC-lib/
├── drivers/sc7a20/
│   ├── include/
│   │   ├── sc7a20.h          # 用户接口头文件
│   │   ├── sc7a20_core.h     # 核心数据结构定义
│   │   └── sc7a20_reg.h      # 寄存器定义与位域结构
│   └── src/
│       ├── sc7a20_core.c     # 核心功能实现
│       ├── sc7a20_sync.c     # 同步接口实现
│       └── sc7a20_async.c    # 异步接口实现
├── include/                   # ACC-lib 公共头文件
├── xmake.lua                 # 构建配置
└── docs/                     # 文档目录
```

### 构建配置
在 `xmake.lua` 中启用 SC7A20 驱动：
```lua
option("acc_sc7a20")
    set_default(true)  -- 设置为 true 启用驱动
```

## 2. 关键数据结构与总线抽象

### 设备结构体 (`sc7a20_dev_t`)
```c
typedef struct {
    const sc7a20_bus_ops_t *ops;      // 总线操作函数指针
    void *bus_ctx;                    // 总线上下文
    uint8_t addr;                     // 设备地址

    sc7a20_cfg_t cfg;                 // 当前配置
    bool initialized;                 // 初始化标志
    uint8_t who_am_i;                 // 设备ID
    uint8_t endian_ble;               // 字节序
    float sensitivity_g_per_lsb;      // 灵敏度系数

    volatile uint8_t in_use;          // 异步操作锁
    sc7a20_async_ctx_t async;         // 异步上下文
} sc7a20_dev_t;
```

### 总线操作接口 (`sc7a20_bus_ops_t`)
移植时必须实现的总线回调：
```c
typedef struct {
    int (*xfer)(void *ctx,
                const sc7a20_comm_msg_t *msgs,
                uint8_t cnt,
                sc7a20_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} sc7a20_bus_ops_t;
```

#### 总线消息结构 (`sc7a20_comm_msg_t`)
```c
typedef struct {
    uint8_t *buf;    // 数据缓冲区
    uint16_t len;    // 数据长度
    uint8_t flags;   // 操作标志
} sc7a20_comm_msg_t;
```

#### 操作标志定义
- `SC7A20_COMM_WRITE` (0x01): 写操作
- `SC7A20_COMM_READ`  (0x02): 读操作  
- `SC7A20_COMM_STOP`  (0x04): 传输结束后发送停止条件

#### 总线回调约定
1. **同步模式**：`xfer()` 应阻塞等待传输完成，返回 0 表示成功
2. **异步模式**：`xfer()` 应立即返回 0，传输完成后调用 `cb(user, status)`
3. **错误处理**：返回负数错误码（-EIO, -EINVAL 等）
4. **取消操作**：`cancel()` 应取消当前进行的异步操作

### 配置结构体 (`sc7a20_cfg_t`)
```c
typedef struct {
    sc7a20_accel_fs_t range;      // 量程 (±2g/4g/8g/16g)
    sc7a20_accel_odr_t odr;       // 数据输出率
    bool axis_x_en;               // X轴使能
    bool axis_y_en;               // Y轴使能
    bool axis_z_en;               // Z轴使能
    bool block_data_update;       // 块数据更新
    bool high_resolution;         // 高分辨率模式
    bool low_power;               // 低功耗模式
} sc7a20_cfg_t;
```

### 默认配置
```c
const sc7a20_cfg_t g_sc7a20_default_cfg = {
    .range = SC7A20_ACCEL_FS_2G,
    .odr = SC7A20_ACCEL_ODR_100HZ,
    .axis_x_en = true,
    .axis_y_en = true,
    .axis_z_en = true,
    .block_data_update = true,
    .high_resolution = false,
    .low_power = false,
};
```

## 3. 初始化与配置路径

### 初始化流程
1. **基本初始化**：使用默认配置
```c
int sc7a20_init(sc7a20_dev_t *dev);
```

2. **自定义配置初始化**
```c
int sc7a20_init_with_config(sc7a20_dev_t *dev, const sc7a20_cfg_t *cfg);
```

#### 初始化步骤
1. 验证设备结构体有效性
2. 获取设备锁（防止并发访问）
3. 发送软复位命令 (0xA5 到 0x68 寄存器)
4. 读取 WHO_AM_I 寄存器验证设备ID (预期值 0x11)
5. 应用配置到控制寄存器
6. 设置初始化完成标志

### 配置修改
```c
int sc7a20_set_range(sc7a20_dev_t *dev, sc7a20_accel_fs_t range);
int sc7a20_set_odr(sc7a20_dev_t *dev, sc7a20_accel_odr_t odr);
int sc7a20_set_axis_enable(sc7a20_dev_t *dev, bool x_en, bool y_en, bool z_en);
```

### 设备反初始化
```c
int sc7a20_deinit(sc7a20_dev_t *dev);
```
将设备置于关断模式 (ODR = 0x00)。

## 4. 数据读取路径

### 同步数据读取
1. **原始数据读取**（16位有符号整数）
```c
int sc7a20_read_xyz_raw(sc7a20_dev_t *dev, sc7a20_vec3i16_t *out);
```

2. **重力加速度读取**（浮点数，单位：g）
```c
int sc7a20_read_xyz_g(sc7a20_dev_t *dev, sc7a20_vec3f_t *out);
```

#### 数据解码流程
1. 从 OUTX_L (0x28) 开始连续读取6字节原始数据
2. 根据字节序配置（BLE位）解析16位有符号整数
3. 右移4位（12位有效数据）
4. 应用灵敏度系数转换为重力加速度

#### 灵敏度系数
| 量程 | 灵敏度 (g/LSB) |
|------|----------------|
| ±2g  | 0.0009765625   |
| ±4g  | 0.001953125    |
| ±8g  | 0.00390625     |
| ±16g | 0.0078125      |

### 寄存器直接访问
```c
int sc7a20_read_reg(sc7a20_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int sc7a20_write_reg(sc7a20_dev_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
```

## 5. 异步接口模型

### 异步操作类型
```c
typedef enum {
    SC7A20_ASYNC_OP_NONE = 0,      // 无操作
    SC7A20_ASYNC_OP_READ_REG,      // 寄存器读取
    SC7A20_ASYNC_OP_WRITE_REG,     // 寄存器写入
    SC7A20_ASYNC_OP_READ_XYZ       // XYZ数据读取
} sc7a20_async_op_t;
```

### 异步接口函数
1. **异步寄存器读取**
```c
int sc7a20_read_reg_async(sc7a20_dev_t *dev,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len,
                          sc7a20_done_cb_t cb,
                          void *user);
```

2. **异步寄存器写入**
```c
int sc7a20_write_reg_async(sc7a20_dev_t *dev,
                           uint8_t reg,
                           const uint8_t *data,
                           uint16_t len,
                           sc7a20_done_cb_t cb,
                           void *user);
```

3. **异步XYZ数据读取**
```c
int sc7a20_read_xyz_raw_async(sc7a20_dev_t *dev,
                              sc7a20_read_xyz_cb_t cb,
                              void *user);
```

4. **取消异步操作**
```c
int sc7a20_cancel_async(sc7a20_dev_t *dev);
```

### 异步操作流程
1. 获取设备锁（`in_use` 标志）
2. 设置异步上下文（操作类型、缓冲区、回调等）
3. 调用总线 `xfer()` 启动异步传输
4. 传输完成后，总线层调用 `sc7a20_async_on_bus_done()`
5. 解码数据（如果是XYZ读取）
6. 调用用户回调
7. 释放设备锁

### 回调函数类型
```c
typedef void (*sc7a20_done_cb_t)(void *user, int status);
typedef void (*sc7a20_read_xyz_cb_t)(void *user, const sc7a20_vec3i16_t *xyz, int status);
```

## 6. 已知限制与注意事项

### 已实现能力
1. **完整的寄存器读写**：支持单字节和多字节读写
2. **同步/异步数据读取**：提供阻塞和非阻塞两种数据获取方式
3. **设备配置**：量程、ODR、轴使能等参数可动态配置
4. **并发保护**：使用原子操作实现设备锁，防止多任务冲突
5. **错误处理**：完整的错误码体系（EINVAL, ENODEV, EBUSY, EIO 等）
6. **字节序处理**：自动识别并处理大端/小端数据格式

### 未实现/未覆盖能力
1. **FIFO 操作**：未实现 FIFO 模式配置和批量数据读取
2. **中断配置**：未提供中断使能、阈值设置等接口
3. **敲击检测**：未实现单击/双击检测功能
4. **自测试**：未实现硬件自测试功能
5. **低功耗模式细节**：未提供详细的低功耗配置选项
6. **SPI 接口**：当前仅支持 I2C 接口，SPI 接口未实现
7. **校准功能**：未提供偏移校准和灵敏度校准接口
8. **温度补偿**：未实现温度传感器读取和补偿
9. **运动检测**：未实现6D/4D方向检测功能
10. **高通滤波器配置**：未提供滤波器参数设置接口

### 移植注意事项
1. **总线回调实现**：必须正确实现 `xfer()` 和 `cancel()` 函数
2. **并发安全**：确保总线操作在多任务环境下安全
3. **错误码映射**：总线层错误码应正确映射到标准错误码
4. **内存管理**：驱动不管理内存，所有缓冲区由调用者提供
5. **中断环境**：异步回调可能在中断上下文中调用，需注意重入问题

### 构建验证
```bash
# 启用 SC7A20 驱动构建
xmake f --acc_sc7a20=y
xmake
```

### 已验证平台
- 当前代码未包含具体平台验证信息，需要用户在实际硬件上测试验证