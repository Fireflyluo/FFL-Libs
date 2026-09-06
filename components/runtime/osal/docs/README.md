# OSAL Component

`ffl.osal` 提供事件、任务、消息、定时器与内存管理 core。通用实现通过 `osal_port.c` 的 hook 接入临界区与时基，不能直接依赖具体 MCU HAL。

`ports/py32/osal/` 保留 PY32 类型和 HAL 相关适配；最终工程应根据自己的中断模型、tick 来源和内存策略实现或替换 port hook。当前状态为 host 编译检查，尚未完成目标板的调度、定时器和长时间稳定性验证。

## Host 验证

`ffl.osal.test` 使用 fake critical-section 和 tick hooks，在 GCC host 上覆盖事件优先级与清除、消息分配/发送/接收/队列、单次与重载定时器、OSAL protothread 事件/延时调度，以及内存分配和内存工具 API。

`ffl.osal.cxx-test` 链接调用公开 C API，覆盖 OSAL 公共头的 C++ 外部链接兼容性。两个 target 都使用 `-Wall -Wextra -Werror`，只证明 host/mock 行为与编译边界，不替代真实 MCU 调度和定时器验证。

```text
xmake build -P . ffl.osal.test
xmake build -P . ffl.osal.cxx-test
xmake run -P . ffl.osal.test
xmake run -P . ffl.osal.cxx-test
xmake test -P .
```
