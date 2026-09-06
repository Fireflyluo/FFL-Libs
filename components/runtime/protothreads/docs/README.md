# Protothreads Component

`ffl.protothreads` 是纯头文件协作式状态机宏。它不创建线程、栈或时钟；调用方负责调度频率、并发边界和任何与 OSAL 的组合方式。

公开头位于 `include/protothreads.h`，可被遵守下述宏约束的 C11 或 C++17 目标直接包含。每个协程函数使用 `PT_THREAD` 声明返回类型，并在函数体中成对使用 `PT_BEGIN` 与 `PT_END`：

```c
static PT_THREAD(worker(pt_t *pt, int *ready))
{
    PT_BEGIN(pt);
    PT_WAIT_UNTIL(pt, *ready != 0);
    PT_END(pt);
}
```

使用约定：

- 首次调用前使用 `PT_INIT`；`PT_SPAWN` 会在首次进入该调用点时初始化子协程。
- `PT_WAIT_UNTIL`、`PT_WAIT_WHILE`、`PT_YIELD` 和 `PT_SPAWN` 在函数内使用 `__LINE__` 保存恢复点，同一函数中的这些宏调用必须位于不同源代码行。
- `PT_BEGIN` 到 `PT_END` 之间不能跨等待点声明需要初始化或析构的局部变量，也不要在其中使用会截断恢复流程的嵌套 `switch`；需要保存的状态应放入调用方结构体。
- 等待、让出和重启返回 `0`；`PT_END` 与 `PT_EXIT` 返回 `2`，因此可用 `PT_SCHEDULE` 判断协程是否仍需调度。
- `PT_RESTART` 清零状态并立即返回，下一次调度从 `PT_BEGIN` 的入口重新执行。
