# SC7A20

本目录提供不依赖 MCU HAL 的 SC7A20 加速度计 core，以及基于统一
`ffl.driver_port` 的 I2C 7-bit portable facade。正式实现不吸收
`experimental/sc7a20htr/` 的 FIFO 工作区。

## 目录

```text
sc7a20/
├── include/
│   ├── ffl/sc7a20.h       # portable facade
│   ├── sc7a20.h           # core 双入口 API
│   ├── sc7a20_core.h      # core 类型与事务契约
│   └── sc7a20_reg.h       # 寄存器与枚举
├── src/
│   ├── ffl_sc7a20.c       # ffl.driver_port 适配器
│   ├── sc7a20_core.c      # 事务、解析、配置
│   ├── sc7a20_sync.c      # 同步 API
│   └── sc7a20_async.c     # 可选异步 API
├── test/                  # fake-register C/C++ host tests
├── docs/
└── experimental/sc7a20htr/ # 保留但不进入正式 target
```

## 两种交付方式

### 快速使用：完整静态库

`ffl.sc7a20.full` 编译 core、portable facade 和异步入口，适合不需要
按功能裁剪的工程：

```lua
includes("components/drivers/sensor/accelerometer/sc7a20")
target("firmware")
    set_kind("binary")
    add_deps("ffl.sc7a20.full")
```

仓库还保留 `ffl-sc7a20` 作为现有 xmake-repo recipe 的兼容静态 target；
它不是驱动集合，内容与 `ffl.sc7a20.full` 相同。

### 资源受限：object + 源码裁剪

`ffl.sc7a20` 是主要的可裁剪入口：

```lua
includes("components/drivers/sensor/accelerometer/sc7a20")
set_config("sc7a20_async", false)
target("firmware")
    set_kind("binary")
    add_deps("ffl.sc7a20")
```

默认 `sc7a20_async=true` 时包含异步源文件和 API；关闭后不编译
`src/sc7a20_async.c`，facade 头中的异步声明也随之隐藏。同步 core、
I2C facade、量程/ODR/轴使能和三轴读取仍保留。

## Portable facade 用法

```c
#include "ffl/sc7a20.h"

static ffl_sc7a20_device_t sensor = {0};
static ffl_sc7a20_config_t config;

ffl_sc7a20_config_init(&config);
ffl_sc7a20_bind(&sensor, &i2c_transport, &time_ops, time_ctx);
ffl_sc7a20_set_i2c_addr(&sensor, FFL_SC7A20_DEFAULT_ADDR7_L);
ffl_sc7a20_init(&sensor, &config);
ffl_sc7a20_read_g(&sensor, &accel_g);
```

`i2c_transport` 的 endpoint 必须是 `FFL_ENDPOINT_I2C_7BIT`，地址必须是
非零 7-bit 地址。每次事务都使用 device 当前地址，并将寄存器读事务
映射为 write-register + repeated-start read + stop。transport、time_ops、
time_ctx 和异步读写缓冲区均为借用对象，必须在设备使用期间保持有效。

SC7A20 当前 core 没有延时回调需求，`time_ops/time_ctx` 为统一 bind 形态
保留并记录生命周期；不会被伪装成硬件时序保证。异步 transport 必须遵守
`ffl.driver_port` 的 callback/cancel 契约，缺少 cancel 时
`ffl_sc7a20_cancel_async()` 返回 `-ENOTSUP`。

## 验证状态

当前提供 fake-register C 测试和 C++ 头兼容测试，覆盖地址传播、I2C 标志、
寄存器读写、三轴解析、g 换算、配置失败回滚、异步完成与取消。尚未进行
真实 SC7A20 硬件验证；FIFO、IRQ 和 `experimental/sc7a20htr/` 路径不属于
本次 portable facade 的正式支持范围。

更多信息见 [API_REFERENCE.md](./API_REFERENCE.md)、
[PORTING_GUIDE.md](./PORTING_GUIDE.md) 和 [regs_map.md](./regs_map.md)。
