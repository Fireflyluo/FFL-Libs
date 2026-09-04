# Protothreads Component

`ffl.protothreads` 是纯头文件协作式状态机宏。它不创建线程、栈或时钟；调用方负责调度频率、并发边界和任何与 OSAL 的组合方式。

公开头位于 `include/protothreads.h`，可被 C11 或 C++17 目标直接包含。
