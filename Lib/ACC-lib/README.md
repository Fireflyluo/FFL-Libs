# ACC-lib

`Lib/ACC-lib` 是本仓库统一的纯加速度计驱动目录。

## 支持器件

| 器件 | 类型 | 总线 | 默认地址 |
|------|------|------|----------|
| SC7A20 | 三轴加速度计 | I2C | `0x18`（SDO 接 GND）/ `0x19`（SDO 悬空或接 VCC） |

## 目录结构

```
Lib/ACC-lib/
├── include/                     # 公共聚合头文件
│   └── acc_sc7a20.h             # 聚合头文件，直接 include sc7a20.h
├── drivers/sc7a20/
│   ├── include/
│   │   ├── sc7a20.h             # 对外 API 声明
│   │   ├── sc7a20_core.h        # 核心类型（设备结构体、总线回调、配置类型）
│   │   └── sc7a20_reg.h         # 寄存器地址 + 位域定义
│   └── src/
│       ├── sc7a20_core.c        # 核心功能实现
│       ├── sc7a20_sync.c        # 同步接口实现
│       └── sc7a20_async.c       # 异步接口实现
├── docs/                        # 文档
├── xmake.lua
└── README.md
```

## xmake 选项开关

默认不启用，必须在引入方显式开启：

```lua
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("acc-lib", {
    configs = {
        sc7a20 = true
    }
})
add_packages("acc-lib")
```

## 条件编译宏

| 宏 | 含义 |
|----|------|
| `ACC_LIB_AVAILABLE` | acc-lib 包已接入 |
| `ACC_DRIVER_SC7A20` | SC7A20 驱动已启用 |

## 最小移植接口

驱动核心不依赖任何平台 HAL，需由使用者实现：

### 总线操作 `sc7a20_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const sc7a20_comm_msg_t *msgs, uint8_t cnt,
                sc7a20_bus_done_cb_t cb, void *user);
    int (*cancel)(void *ctx);
} sc7a20_bus_ops_t;
```

- `xfer`：同步模式下 `cb` 传 `NULL`，阻塞返回；异步模式下传完成回调
- `cancel`：取消异步操作（同步模式可返回 0 占位）

### 设备实例填充

```c
sc7a20_dev_t dev = {
    .ops     = &my_bus_ops,
    .bus_ctx = &my_i2c_handle,
    .addr    = SC7A20_I2C_ADDR_L,  // 0x18
};
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [acc-lib-api.md](docs/acc-lib-api.md) | 公共接口、数据结构、API 列表 |
| [sc7a20-driver.md](docs/sc7a20-driver.md) | SC7A20 驱动说明 |

## 常见问题与排障

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `init` 返回 `-ENODEV` | WHO_AM_I 不匹配 | 用 `sc7a20_get_who_am_i` 读 ID，期望 `0x11` |
| 读数全为 0 | I2C 地址错误 | 确认 SDO 引脚电平，选 `0x18` 或 `0x19` |
| 异步接口返回 `-EBUSY` | 设备正忙 | 等待上次异步操作完成后再调用 |
| `init` 报 "no driver enabled" | 未在 xmake configs 中启用 | 在 `add_requires` 中设置 `sc7a20 = true` |
