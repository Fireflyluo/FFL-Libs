# IMU-lib

`Lib/IMU-lib` 是本仓库统一的 IMU（惯性测量单元）驱动目录。

## 支持器件

| 器件 | 类型 | 总线 | 默认地址 |
|------|------|------|----------|
| QMI8658A | 6 轴 IMU（加速度计 + 陀螺仪） | I2C / SPI | `0x6A` |
| ICM42688P | 6 轴 IMU（加速度计 + 陀螺仪） | I2C / SPI | `0x69` |

## 目录结构

```
Lib/IMU-lib/
├── include/                     # 跨器件公共接口
│   ├── imu_bus.h                # 总线抽象（imu_bus_ops_t、imu_bus_msg_t）
│   ├── imu_types.h              # 通用数据结构（imu_raw_sample_t、imu_sample_t）
│   ├── imu_qmi8658a.h           # QMI8658A 对外 API
│   └── imu_icm42688p.h          # ICM42688P 对外 API
├── drivers/
│   ├── qmi8658a/
│   │   ├── include/qmi8658a_reg.h  # 寄存器地址 + 位域定义
│   │   └── src/imu_qmi8658a.c     # 驱动实现
│   └── icm42688p/
│       ├── include/icm42688_reg.h
│       └── src/imu_icm42688p.c
├── port/                        # 平台适配示例（PY32 / CH32）
├── docs/                        # 文档
├── xmake.lua                    # 包构建脚本
└── README.md
```

## xmake 选项开关

默认不启用任何驱动，必须在引入方显式开启：

```lua
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("imu-lib", {
    configs = {
        qmi8658a = true,   -- 启用 QMI8658A
        icm42688p = false  -- 禁用 ICM42688P
    }
})
add_packages("imu-lib")
```

若两个都未启用，构建会报错提示配置。

## 条件编译宏

| 宏 | 含义 |
|----|------|
| `IMU_LIB_AVAILABLE` | imu-lib 包已接入 |
| `IMU_DRIVER_QMI8658A` | QMI8658A 驱动已启用 |
| `IMU_DRIVER_ICM42688P` | ICM42688P 驱动已启用 |

## 最小移植接口

驱动核心不依赖任何平台 HAL，需由使用者实现：

### 总线操作 `imu_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const imu_bus_msg_t *msgs, uint8_t cnt,
                imu_bus_done_cb_t cb, void *user);
    int (*cancel)(void *ctx);
} imu_bus_ops_t;
```

- `xfer`：执行 I2C/SPI 传输。`cnt==2` 时为标准寄存器读写模式（`msgs[0]` 写寄存器地址，`msgs[1]` 读/写数据）
- `cancel`：当前未使用，可返回 0 占位

### 延时回调 `imu_delay_ms_fn`

```c
typedef void (*imu_delay_ms_fn)(void *ctx, uint32_t ms);
```

封装平台毫秒延时（如 `HAL_Delay`）。

### 设备实例填充

```c
imu_qmi8658a_t dev = {
    .bus_ops  = &my_bus_ops,
    .bus_ctx  = &my_i2c_handle,
    .delay_ms = my_delay_ms,
    .delay_ctx = NULL,
};
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [imu-lib-api.md](docs/imu-lib-api.md) | 公共接口、数据结构、API 列表 |
| [imu-lib-porting-guide.md](docs/imu-lib-porting-guide.md) | 移植指南（平台适配步骤） |
| [qmi8658a-driver.md](docs/qmi8658a-driver.md) | QMI8658A 驱动说明 |
| [icm42688p-driver.md](docs/icm42688p-driver.md) | ICM42688P 驱动说明 |

## 常见问题与排障

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `init` 返回 `-ENODEV` | WHO_AM_I 校验失败 | 用 `probe` 读 chip_id，确认硬件地址和连线 |
| 读数全为 0 或 0xFF | I2C 地址左移问题 | 检查 HAL 是否需要 7-bit 地址左移 1 位 |
| `init` 报 "no driver enabled" | 未在 xmake configs 中启用器件 | 在 `add_requires` 中设置对应选项为 `true` |
| 编译报 `EIO` 未定义 | 工具链缺 POSIX errno | 在适配层自行定义或包含 `<errno.h>` |
| ICM42688P probe 返回非预期值 | WHO_AM_I 期望值不一致 | 参考 `icm42688p-driver.md` 第 7 节 |
