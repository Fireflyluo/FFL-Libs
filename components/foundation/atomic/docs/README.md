# Atomic Helpers

`ffl.atomic` 提供跨 MSVC、GCC 和 Clang 的最小 `uint8_t` 自旋锁封装，供组件内部保护短临界区。它是纯头文件组件，不引入 RTOS、MCU HAL 或具体中断控制。

没有原子内建支持的编译器会使用普通读写回退，因此该回退只适用于调用方已在其他层面保证互斥的场景。对 MCU 中断并发的正式保护仍应由组件的临界区 hook 或 port 实现负责。
