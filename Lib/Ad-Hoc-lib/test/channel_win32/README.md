# channel_win32 — Windows 仿真适配层

本目录为协议库在 Windows 上的仿真适配实现，将嵌入式 RF 链路抽象替换为软件模拟。

## 模块一览

| 模块 | 文件 | 职责 |
|------|------|------|
| **虚拟时间** | `adhoc_virtual_time.h` | 基于 `QueryPerformanceCounter` 的单调微秒时钟 |
| **空域信道** | `adhoc_channel.h/c` | 拓扑矩阵 + 环形 FIFO + 临界区, 模拟无线广播 |
| **日志系统** | `adhoc_logger.h/c` | 仿嵌入式串口日志, 每节点独立文件输出 |
| **链路适配** | `adhoc_link_win32.h/c` | 实现 `adhoc_link_ops_t` 六函数, 桥接协议层与信道 |

## 分层架构

```
  main.c (多线程仿真)
    │
    ├─ 节点线程 ── adhoc_node ── adhoc_link_win32 ── adhoc_channel
    │                  (协议层)      (链路适配)         (空域信道)
    │
    └─ 主线程 : 定时打印状态, 管理仿真生命周期
```

## 各模块详解

### 1. adhoc_virtual_time.h — 虚拟时间

```c
adhoc_virtual_time_t vt;
adhoc_virtual_time_init(&vt);
uint32_t now = adhoc_virtual_time_now_us(&vt);
```

- 全仿真共享同一个 `vt` 实例
- 基于 `QueryPerformanceCounter`, 精度 < 1μs
- 函数为 `static`, 直接内联于头文件

### 2. adhoc_channel.h/c — 空域信道

核心数据结构:

```c
typedef struct {
    uint8_t  topo[MAX][MAX];        // 拓扑矩阵: topo[from][to] = RSSI (0=不可达)
    node_rx_t nodes[MAX];           // 每节点独立的环形 FIFO 接收队列
    CRITICAL_SECTION lock;          // 多线程安全
} adhoc_channel_t;
```

发送流程:
1. 查 `topo[from][*]` 获得所有可到达的目标节点
2. 将帧拷贝到每个目标节点的 RX FIFO (携带 RSSI)
3. FIFO 满时丢弃 (不阻塞)

关键常量:
- `ADHOC_CHANNEL_MAX_NODES` = 64
- `ADHOC_CHANNEL_RX_FIFO_SIZE` = 256

### 3. adhoc_logger.h/c — 日志系统

- 全局单例, 基于 slot 索引管理文件
- 最大 64 个日志槽 (`ADHOC_LOGGER_MAX_SLOTS`)
- 线程安全 (CriticalSection 保护)
- 输出格式: `[HH:MM:SS.ms] 消息内容`

```c
adhoc_logger_init(logs_dir);       // 初始化, 指定日志目录
adhoc_logger_open_slot(i, name);   // 为 slot i 打开日志文件
adhoc_logger_log(i, fmt, ...);     // 向 slot i 写入格式化日志
adhoc_logger_shutdown();           // 关闭所有文件
```

### 4. adhoc_link_win32.h/c — 链路适配

实现 `adhoc_link_ops_t` 六个函数:

| 函数 | 实现 |
|------|------|
| `init` | 记录链路初始化日志 |
| `start_rx` | 无操作 (信道始终就绪) |
| `tx` | 调用 `adhoc_channel_tx`, 记录 TX 日志 |
| `poll_rx` | 调用 `adhoc_channel_poll_rx`, 记录 RX 日志 |
| `now_us` | 代理到 `adhoc_virtual_time_now_us` |
| `rand_u16` | 基于 `rand()` + 线性同余混合 |

上下文结构 `adhoc_link_win32_ctx_t`:

| 字段 | 用途 |
|------|------|
| `channel` | 指向共享信道 |
| `node_idx` | 本节点在信道中的索引 |
| `log_slot` | 日志槽索引 |
| `vt` | 虚拟时间实例 |
| `node_id` | 协议层节点 ID |

## 编译依赖

- Windows SDK (`windows.h`)
- C11 标准库
- `../../include/` (协议库头文件)
