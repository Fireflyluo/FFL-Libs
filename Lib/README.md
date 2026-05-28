# Lib 传感器与协议库目录

本目录包含本仓库所有传感器驱动库和协议库，按传感器类别分类组织。

---

## 1. 分类原则

| 包名 | 类别 | 说明 |
|------|------|------|
| `IMU-lib` | 惯性测量单元 | 6 轴 / 9 轴 IMU 驱动（加速度计 + 陀螺仪，可含磁力计） |
| `ACC-lib` | 加速度计 | 纯加速度计驱动（无陀螺仪） |
| `MAG-lib` | 磁力计 | 纯磁力计驱动（三轴地磁） |
| `BARO-lib` | 气压计 | 气压 / 温度传感器驱动 |
| `TH-lib` | 温湿度 | 温度 + 湿度传感器驱动 |
| `Ad-Hoc-lib` | 协议库 | 自组网协议（非传感器类） |

**分类规则**：
- 一个包内只放同一类别的芯片驱动，不混装
- 每个包内各芯片驱动位于 `drivers/<chip>/include|src`
- 平台相关代码不进驱动核心，放在 `port/` 或应用层
- 包选项默认关闭，由引入方工程显式启用具体器件

---

## 2. 统一接入流程

以下以 `IMU-lib` 为例，其他传感器包流程一致。

### 2.1 添加本地仓库并引入包

在工程 `xmake.lua` 中：

```lua
-- 添加本地包仓库（路径按实际修改）
add_repositories("embedded-libs D:/path/to/0.fireflyluo-Embedded-Libs-main/xmake-repo")

-- 引入包，显式启用需要的器件驱动
add_requires("imu-lib", {configs = {qmi8658a = true, icm42688p = false}})

-- 在 target 中挂载
target("my-app")
    add_packages("imu-lib")
```

### 2.2 编写移植层（平台侧）

驱动核心不依赖任何平台 HAL，需由使用者实现以下移植接口：

| 接口 | 说明 |
|------|------|
| 总线传输回调 (`xfer`) | I2C / SPI 读写，同步阻塞返回 |
| 总线取消回调 (`cancel`) | 可占位返回 0 |
| 延时函数 (`delay_ms` / `delay_us`) | 毫秒 / 微秒级阻塞延时 |

详见各包 `docs/porting-guide.md`。

### 2.3 初始化与读取

```c
#include "imu_qmi8658a.h"

imu_qmi8658a_t dev = {
    .bus_ops  = &my_bus_ops,
    .bus_ctx  = &my_i2c_handle,
    .delay_ms = my_delay_ms,
};

int rc = imu_qmi8658a_init(&dev, NULL);  // NULL 使用默认配置
if (rc != 0) { /* 错误处理 */ }

imu_sample_t sample;
rc = imu_qmi8658a_read_sample(&dev, &sample);
```

### 2.4 错误处理约定

所有传感器库统一约定：
- 返回值 `int` 类型：**0 成功，负数失败**
- 常见错误码：`-EIO`（总线错误）、`-EINVAL`（参数无效）、`-ENODEV`（设备未识别）、`-EBUSY`（设备忙）
- 使用者应在每次 API 调用后检查返回值

---

## 3. 目录结构约定

```
Lib/<category>-lib/
├── include/                  # 跨器件公共接口与聚合头文件
├── drivers/<chip>/
│   ├── include/<chip>_reg.h  # 寄存器地址 + 位域定义
│   └── src/<chip>_*.c        # 驱动实现
├── port/<platform>/          # 平台适配示例（可选）
├── docs/                     # 驱动说明与移植文档
├── xmake.lua                 # 包构建脚本
└── README.md
```

---

## 4. xmake 选项开关约定

每个包的器件驱动默认**不启用**（`set_default(false)`），必须由引入方工程显式开启。

| 包 | 选项名 | 说明 |
|----|--------|------|
| `imu-lib` | `imu_qmi8658a` | 启用 QMI8658A 驱动 |
| `imu-lib` | `imu_icm42688p` | 启用 ICM42688P 驱动 |
| `acc-lib` | `acc_sc7a20` | 启用 SC7A20 驱动 |
| `mag-lib` | `mag_qmc5883p` | 启用 QMC5883P 驱动 |
| `baro-lib` | `baro_icp20100` | 启用 ICP20100 驱动 |
| `th-lib` | `th_sht40` | 启用 SHT40 驱动 |
| `th-lib` | `th_sht30` | 启用 SHT30 驱动 |

未启用的驱动源文件不会编译进库，节省 Flash 空间。

---

## 5. 条件编译宏

xmake 会在启用驱动时自动定义以下宏（`public` 作用域，应用层可见）：

| 宏模式 | 示例 | 含义 |
|--------|------|------|
| `<CATEGORY>_LIB_AVAILABLE` | `IMU_LIB_AVAILABLE` | 该类别包已接入 |
| `<CATEGORY>_DRIVER_<CHIP>` | `IMU_DRIVER_QMI8658A` | 该器件驱动已启用 |

可在应用层用 `#ifdef` 做条件编译。

---

## 6. 外部工程决定目标平台与 toolchain

本仓库所有传感器库均为**平台无关的纯 C 静态库**，不指定编译器、不指定目标架构。

- 目标平台（ARM Cortex-M / RISC-V / x86 等）由**外部引入工程**决定
- toolchain（GCC / ARMCC / IAR / Clang 等）由**外部引入工程**决定
- 本仓库 `xmake.lua` 仅设置 C 语言标准（C11）和警告选项，不做平台判断
- 若需要交叉编译，在外部工程的 `xmake.lua` 中配置 toolchain 即可

---

## 7. 各包文档索引

| 包 | README | API 文档 | 驱动文档 | 移植指南 |
|----|--------|----------|----------|----------|
| IMU-lib | [README](IMU-lib/README.md) | [imu-lib-api.md](IMU-lib/docs/imu-lib-api.md) | [qmi8658a-driver.md](IMU-lib/docs/qmi8658a-driver.md), [icm42688p-driver.md](IMU-lib/docs/icm42688p-driver.md) | [imu-lib-porting-guide.md](IMU-lib/docs/imu-lib-porting-guide.md) |
| ACC-lib | [README](ACC-lib/README.md) | [acc-lib-api.md](ACC-lib/docs/acc-lib-api.md) | [sc7a20-driver.md](ACC-lib/docs/sc7a20-driver.md) | — |
| MAG-lib | [README](MAG-lib/README.md) | — | [qmc5883p-driver.md](MAG-lib/docs/qmc5883p-driver.md) | — |
| BARO-lib | [README](BARO-lib/README.md) | — | [icp20100-driver.md](BARO-lib/docs/icp20100-driver.md) | — |
| TH-lib | [README](TH-lib/README.md) | [th-lib-api.md](TH-lib/docs/th-lib-api.md) | [sht40-driver.md](TH-lib/docs/sht40-driver.md), [sht30-driver.md](TH-lib/docs/sht30-driver.md) | — |
| Ad-Hoc-lib | [README](Ad-Hoc-lib/README.md) | [USAGE.md](Ad-Hoc-lib/docs/USAGE.md) | — | [porting-guide.md](Ad-Hoc-lib/docs/porting-guide.md) |
