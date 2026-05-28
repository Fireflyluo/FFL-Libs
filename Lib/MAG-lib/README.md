# MAG-lib

`Lib/MAG-lib` 是本仓库统一的纯磁力计驱动目录。

## 支持器件

| 器件 | 类型 | 总线 | 默认地址 |
|------|------|------|----------|
| QMC5883P | 三轴磁力计 | I2C | `0x2C` |

## 目录结构

```
Lib/MAG-lib/
├── include/
│   └── mag_qmc5883p.h           # 聚合头文件，直接 include qmc5883p.h
├── drivers/qmc5883p/
│   ├── include/
│   │   ├── qmc5883p.h           # 对外 API 声明、数据结构
│   │   └── qmc5883p_reg.h       # 寄存器地址 + 位域定义
│   └── src/
│       └── qmc5883p.c           # 驱动实现
├── docs/                        # 文档
├── xmake.lua
└── README.md
```

## xmake 选项开关

默认不启用，必须在引入方显式开启：

```lua
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")
add_requires("mag-lib", {
    configs = {
        qmc5883p = true
    }
})
add_packages("mag-lib")
```

## 条件编译宏

| 宏 | 含义 |
|----|------|
| `MAG_LIB_AVAILABLE` | mag-lib 包已接入 |
| `MAG_DRIVER_QMC5883P` | QMC5883P 驱动已启用 |

## 最小移植接口

驱动核心不依赖任何平台 HAL，需由使用者实现：

### 总线操作 `qmc5883p_bus_ops_t`

```c
typedef struct {
    int (*xfer)(void *ctx, const qmc5883p_comm_msg_t *msgs, uint8_t cnt,
                void *done_cb, void *user);
} qmc5883p_bus_ops_t;
```

- `xfer`：执行 I2C 传输。`cnt==2` 时为标准寄存器读写模式

### 设备实例填充

```c
qmc5883p_dev_t dev = {
    .ops     = &my_bus_ops,
    .bus_ctx = &my_i2c_handle,
    .addr    = 0x2C,
};
```

## 文档索引

| 文档 | 说明 |
|------|------|
| [qmc5883p-driver.md](docs/qmc5883p-driver.md) | QMC5883P 驱动说明 |

## 常见问题与排障

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| `init` 返回 `-ENODEV` | WHO_AM_I 不匹配 | 用 `qmc5883p_probe` 读 chip_id，期望 `0x80` |
| 磁场数据为 0 | 未等待数据就绪 | 检查 STATUS 寄存器 DRDY 位 |
| `init` 报 "no driver enabled" | 未在 xmake configs 中启用 | 在 `add_requires` 中设置 `qmc5883p = true` |
