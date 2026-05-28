# TH-lib

`Lib/TH-lib` 是本仓库统一的温湿度传感器驱动目录。

## 支持器件

| 器件 | 类型 | 总线 | 默认地址 |
|------|------|------|----------|
| SHT40 | 温湿度传感器 | I2C | `0x44`（默认）/ `0x45` / `0x46` |
| SHT30 | 温湿度传感器 | I2C | `0x44`（默认）/ `0x45` |

## 目录结构

```
Lib/TH-lib/
├── include/                     # 公共聚合头文件
│   └── th_sht40.h               # 聚合头文件
├── drivers/
│   ├── sht40/
│   │   ├── include/
│   │   │   ├── sht40.h          # 对外 API 声明
│   │   │   └── sht40_core.h     # 核心类型、总线抽象
│   │   └── src/
│   │       ├── sht40_core.c     # 基础工具函数
│   │       ├── sht40_sync.c     # 同步接口实现
│   │       └── sht40_async.c    # 异步接口实现
│   └── sht30/
│       ├── include/
│       │   ├── sht30.h          # 对外 API 声明
│       │   └── sht30_core.h     # 核心类型、总线抽象
│       └── src/
│           ├── sht30_core.c
│           ├── sht30_sync.c
│           └── sht30_async.c
├── docs/                        # 文档
├── xmake.lua
└── README.md
```

## xmake 选项开关

默认不启用，必须在引入方显式开启：

```lua
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")

-- 仅启用 SHT40
add_requires("th-lib", {configs = {sht40 = true, sht30 = false}})
add_packages("th-lib")

-- 仅启用 SHT30
add_requires("th-lib", {configs = {sht40 = false, sht30 = true}})
add_packages("th-lib")

-- 同时启用
add_requires("th-lib", {configs = {sht40 = true, sht30 = true}})
add_packages("th-lib")
```

## 条件编译宏

| 宏 | 含义 |
|----|------|
| `TH_LIB_AVAILABLE` | th-lib 包已接入 |
| `TH_DRIVER_SHT40` | SHT40 驱动已启用 |
| `TH_DRIVER_SHT30` | SHT30 驱动已启用 |

## 最小移植接口

驱动核心不依赖任何平台 HAL，需由使用者实现：

### 总线操作 `sht40_bus_ops_t` / `sht30_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const sht40_comm_msg_t *msgs, uint8_t cnt,
                sht40_bus_done_cb_t cb, void *user);
    int (*cancel)(void *ctx);
} sht40_bus_ops_t;
```

- `xfer`：同步模式下 `cb` 传 `NULL`；异步模式下传完成回调
- `cancel`：取消异步操作（同步模式可返回 0 占位）

### 延时回调 `sht40_delay_ms_fn` / `sht30_delay_ms_fn`

```c
typedef void (*sht40_delay_ms_fn)(void *ctx, uint32_t ms);
```

封装平台毫秒延时。若为 `NULL`，同步接口的测量等待将被跳过，可能导致读取失败。

### 设备实例填充（SHT40）

```c
sht40_dev_t dev = {
    .ops      = &my_bus_ops,
    .bus_ctx  = &my_i2c_handle,
    .addr     = 0,           // 0 = 使用默认地址 0x46
    .delay_ms = my_delay_ms,
    .delay_ctx = NULL,
};
```

### 设备实例填充（SHT30）

```c
sht30_dev_t dev = {
    .ops      = &my_bus_ops,
    .bus_ctx  = &my_i2c_handle,
    .addr     = 0,           // 0 = 使用默认地址 0x44
    .delay_ms = my_delay_ms,
    .delay_ctx = NULL,
};
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [th-lib-api.md](docs/th-lib-api.md) | SHT40 公共接口、数据结构、API 列表 |
| [sht40-driver.md](docs/sht40-driver.md) | SHT40 驱动说明 |
| [sht30-driver.md](docs/sht30-driver.md) | SHT30 驱动说明 |

## 常见问题与排障

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `init` 返回 `-EIO` | I2C 通信失败 | 检查连线和地址（SHT40 默认 `0x44`，SHT30 默认 `0x44`） |
| 读数异常（温度 0、湿度 0） | 未实现 `delay_ms` | 确保 `delay_ms` 回调非 NULL |
| 异步接口返回 `-EBUSY` | 设备正忙 | 等待上次异步操作完成后再调用 |
| `init` 报 "no driver enabled" | 未在 xmake configs 中启用 | 在 `add_requires` 中设置对应选项为 `true` |
| CRC 校验失败 | 数据传输错误 | 检查 I2C 总线质量，降低速率 |
