# BARO-lib

`Lib/BARO-lib` 是本仓库统一的气压计 / 压力传感器驱动目录。

## 支持器件

| 器件 | 类型 | 总线 | 默认地址 |
|------|------|------|----------|
| ICP20100 | 气压 / 温度传感器 | I2C | `0x63`（AD0 接 GND）/ `0x64`（AD0 接 VCC） |

## 目录结构

```
Lib/BARO-lib/
├── include/
│   └── baro_icp20100.h          # 聚合头文件，直接 include icp20100.h
├── drivers/icp20100/
│   ├── include/
│   │   ├── icp20100.h           # 对外 API 声明、数据结构
│   │   └── icp20100_reg.h       # 寄存器地址 + 位域定义
│   └── src/
│       └── icp20100.c           # 驱动实现
├── docs/                        # 文档
├── xmake.lua
└── README.md
```

## xmake 选项开关

默认不启用，必须在引入方显式开启：

```lua
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("baro-lib", {
    configs = {
        icp20100 = true
    }
})
add_packages("baro-lib")
```

## 条件编译宏

| 宏 | 含义 |
|----|------|
| `BARO_LIB_AVAILABLE` | baro-lib 包已接入 |
| `BARO_DRIVER_ICP20100` | ICP20100 驱动已启用 |

## 最小移植接口

驱动核心不依赖任何平台 HAL，需由使用者实现：

### 总线操作 `icp20100_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const icp20100_comm_msg_t *msgs, uint8_t cnt,
                void *done_cb, void *user);
} icp20100_bus_ops_t;
```

- `xfer`：执行 I2C 传输。`cnt==2` 时为标准寄存器读写模式

### 延时回调

```c
typedef void (*delay_us_fn)(void *ctx, uint32_t us);
```

ICP20100 驱动需要微秒级延时（`delay_us`），用于初始化和测量等待。

### 设备实例填充

```c
icp20100_dev_t dev = {
    .ops      = &my_bus_ops,
    .bus_ctx  = &my_i2c_handle,
    .delay_us = my_delay_us,
    .delay_ctx = NULL,
    .addr     = 0x63,
};
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [icp20100-driver.md](docs/icp20100-driver.md) | ICP20100 驱动说明 |

## 常见问题与排障

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `init` 返回 `-ENODEV` | WHO_AM_I 不匹配 | 用 `icp20100_probe` 读 chip_id，期望 `0x63` |
| `init` 报 "no driver enabled" | 未在 xmake configs 中启用 | 在 `add_requires` 中设置 `icp20100 = true` |
| 读数异常 | 未正确配置 op_mode | 检查 `icp20100_cfg_t` 中的 `op_mode` 和 `meas_mode` |
