# Task 5：组件功能点演示

代码：[`app/features_demo.c`](../../app/features_demo.c)  
上电后 COM4 打印 `[feat] ...`，与采样业务并行、互不依赖。

## 1. ffl.ringbuffer

| API | 演示 |
|-----|------|
| `init` + `put` / `get` | 基本 FIFO |
| `put` 超过剩余空间 | 按组件语义处理（丢弃/返回长度，以头文件为准） |
| `get` 批量读出 | 与 `drained_bytes` 统计同型 |

## 2. ffl.sw_timer

| API | 演示 |
|-----|------|
| 周期定时 `period != 0` | 主业务 500ms 采样 |
| 单次定时 `period == 0` | 蜂鸣器脉宽 80ms 后关断 |
| `stop` / 再 `start` | 重复触发单次定时器 |

单次示例：

```c
ffl_sw_timer_start(&s_beep_timer, 80u, 0u, sw_beep_off, NULL);
```

## 3. ffl.atomic（纯软件，无 port）

| API 思路 | 演示 |
|----------|------|
| 加载/存储 | 统计计数在任务里读写 |
| fetch_add / CAS | `features_demo` 里做一次自检 |

具体符号名以 `components/foundation/atomic/include` 为准；本示例只调用公开头，不复制实现。

## 4. 组合读法（应用层模式）

```text
sw_timer 到期
  → osal_set_event
    → 任务里 read_raw
      → ringbuffer_put
        → STATS 时 ringbuffer_get + printf
```

这是仓库推荐的**应用编排**方式：组件只提供原语，编排在 `app/`。

## 5. 不在本示例中的组件

可在同板或 host 测试中自行接入，路径见 `components/README.md`：

| 组件 | 说明 |
|------|------|
| `ffl.protothreads` | 协作式状态机；OSAL 已依赖其头 |
| `ffl.sht40` / `qmc5883p` 等 | 同样走 `ffl.driver_port` |
| `ffl.impact_displacement` | 纯算法，可 host 测 |

返回：[README 索引](README.md) · [总览](../readme.md)
