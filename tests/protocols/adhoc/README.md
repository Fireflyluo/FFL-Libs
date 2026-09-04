# Ad-Hoc-lib 仿真测试

## 目录结构

```
test/
├── main.c                       # 仿真主程序入口
├── build_and_run.bat            # 一键编译运行脚本 (GCC/MSVC)
├── .gitignore                   # 排除二进制和日志文件
├── README.md                    # 本文件
├── 验证与回归清单.md             # 8 个场景的验证记录
│
├── channel_win32/               # Windows 仿真适配层
│   ├── README.md                # 模块详细文档
│   ├── adhoc_virtual_time.h     # 虚拟时间 (QPC 高精度时钟)
│   ├── adhoc_channel.h/c        # 空域信道 (拓扑矩阵 + FIFO + 临界区)
│   ├── adhoc_link_win32.h/c     # 链路适配 (实现 adhoc_link_ops_t)
│   └── adhoc_logger.h/c         # 日志系统 (仿串口日志 → 文件)
│
├── logs/                        # 运行时日志输出
│   └── README.md                # 日志命名规则说明
│
└── scenarios/                   # 测试场景配置
    ├── README.md                # 场景格式说明
    └── basic_topology.conf      # 基本拓扑场景 (含 3 个示例)
```

## 快速开始

**方式一: 一键运行**
```batch
build_and_run.bat
```
自动检测 MinGW-GCC 或 MSVC, 编译并运行。

**方式二: 手动 GCC**
```batch
gcc -Wall -O2 -std=gnu11 -I"..\include" -I"." ^
    main.c channel_win32\adhoc_channel.c channel_win32\adhoc_link_win32.c ^
    channel_win32\adhoc_logger.c ..\src\adhoc_*.c -o adhoc_sim.exe
```

**方式三: 手动 MSVC**
```batch
cl /nologo /W3 /O2 /MT /D_CRT_SECURE_NO_WARNINGS /utf-8 ^
   /I"..\include" /I"." main.c channel_win32\*.c ..\src\*.c ^
   /Fe:adhoc_sim.exe
```

## 场景切换

编辑 `main.c` 顶部宏:

```c
#define SCENARIO_MODE  0u   // 0..6, 见下表
```

| 模式 | 网关 | 信标 | 拓扑 |
|------|------|------|------|
| 0 | 1 | 2 | 全连通 (RSSI=-40) |
| 1 | 2 | 7 | 3 级树形 + 网状交叉 |
| 2 | 1 | 2 | 全连通弱信号 (RSSI=-90) |
| 3 | 1 | 2 | 全连通 + 高频数据 (60ms) |
| 4 | 1 | 2 | 全连通 + 重组网超时 |
| 5 | 1 | 15 | 全连通 16 节点压力 |
| 6 | 1 | 2 | 全连通 10 分钟稳定性 |

**添加新场景**：在 `sim_init_scenario()` 中添加 `#elif SCENARIO_MODE == N` 分支, 以矩阵形式定义拓扑。

## 设计原理

### 分层架构

```
  主线程 (定时打印状态, 注入故障)
    │
    ├─ 节点线程0 ── adhoc_node ── link_ops ──╮
    ├─ 节点线程1 ── adhoc_node ── link_ops ──┤
    │     ...                                ├── 空域信道 (channel)
    └─ 节点线程N ── adhoc_node ── link_ops ──╯   拓扑矩阵 + FIFO队列
                                                    │
                                               CriticalSection
```

### 关键设计

| 机制 | 实现 |
|------|------|
| 拓扑矩阵 | `topo[from][to]` = RSSI, 0 = 不可达 |
| 虚拟时间 | 全仿真共享 `QueryPerformanceCounter` |
| 日志 | 每节点独立文件, slot 索引隔离 |
| 线程安全 | 信道操作在 CriticalSection 内 |

## 文档导航

| 文档 | 内容 |
|------|------|
| `验证与回归清单.md` | 8 个场景的步骤预期、实际结果、回归记录 |
| `channel_win32/README.md` | 四模块 API 与数据流详解 |
| `scenarios/README.md` | 场景配置文件格式与 RSSI 等级说明 |
| `logs/README.md` | 日志文件命名规则与事件类型 |

## 版本记录

| 日期 | 变更 |
|------|------|
| 2026-05-11 | 初始版本: 4 模块 + main.c + 8 场景验证 |
| 2026-05-11 | 重构: `SCENARIO_MODE` 宏切换, 程序化 N 节点生成 |
| 2026-05-11 | 补齐: `.gitignore`、子目录 README、日志规则文档 |
