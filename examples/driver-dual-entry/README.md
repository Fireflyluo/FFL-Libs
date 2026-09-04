# 驱动双入口最小示例

本示例用一个无硬件依赖的 mock 加速度计演示同一份驱动源码的两种使用方式：

- **完整库入口：** `ffl-demo-accel-full` 是一个本地 Xmake package，默认编译 core 和 FIFO；适合快速接入。
- **源码裁剪入口：** `ffl-demo-accel-source` 是 `object` target；最终工程选择是否编译 FIFO 源文件。

这不是正式 SC7A20 实现，也不代表任何板端验证；它只验证目录、构建图、配置传播和 package/source 双入口模型。

## 1. 快速使用完整库

```powershell
cd examples/driver-dual-entry/quick-package
xmake f -y
xmake
xmake run quick-package-app
```

预期输出：

```text
profile=full, sample=981 mg, fifo=32
```

`quick-package/xmake.lua` 的关键部分是：

```lua
add_repositories("driver-dual-entry " .. path.join(os.scriptdir(), "..", "xmake-repo"))
add_requires("ffl-demo-accel-full")

target("quick-package-app")
    add_packages("ffl-demo-accel-full")
```

## 2. 使用裁剪源码

默认关闭 FIFO：

```powershell
cd examples/driver-dual-entry
xmake f --demo_accel_fifo=n
xmake source-trimmed-app
xmake run source-trimmed-app
```

预期输出：

```text
profile=trimmed, sample=981 mg
fifo=excluded at compile time
```

开启 FIFO：

```powershell
xmake f --demo_accel_fifo=y
xmake source-trimmed-app
xmake run source-trimmed-app
```

此时输出包含 `profile=full` 和 `fifo=32`。裁剪 target 通过条件 `add_files("src/demo_accel_fifo.c")` 真正排除或纳入 FIFO 实现，而不是只对 API 加宏。

## 3. 结构对应关系

```text
component/                       # 未来 components/drivers/.../sc7a20 的形态
  include/                       # 公共 API
  src/                           # 唯一 driver core 源码
  xmake.lua                      # full static target + cropped object target
xmake-repo/packages/.../         # 仅完整库入口的 package recipe
quick-package/                   # add_requires() 消费者
source-trimmed/                  # includes() + add_deps() 消费者
```

真实驱动替换 mock 代码时，`component/` 内必须保持无 HAL；SPI/I2C、CS、CE、IRQ、DMA、时钟和引脚等实现都留在最终工程或 `ports/`。
