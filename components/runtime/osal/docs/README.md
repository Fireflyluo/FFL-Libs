# OSAL Component

`ffl.osal` 提供事件、任务、消息、定时器与内存管理 core。通用实现通过 `osal_port.c` 的 hook 接入临界区与时基，不能直接依赖具体 MCU HAL。

`ports/py32/osal/` 保留 PY32 类型和 HAL 相关适配；最终工程应根据自己的中断模型、tick 来源和内存策略实现或替换 port hook。当前状态为 host 编译检查，尚未完成目标板的调度、定时器和长时间稳定性验证。
