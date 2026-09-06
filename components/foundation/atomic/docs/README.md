# Atomic Helpers

`ffl.atomic` 提供跨 MSVC、GCC 和 Clang 的最小 `uint8_t` 原子操作封装，供组件内部保护短临界区。它是纯头文件组件，不引入 RTOS、MCU HAL 或具体中断控制。

实现依赖编译器原子内建：MSVC 使用 `_Interlocked*`，GCC/Clang 使用 `__sync_*`。如果编译器不提供这些内建，头文件会直接触发 `#error`，不会退化为普通读写。因此，使用不支持这些内建的工具链前，必须先提供等价的原子适配；组件本身不接受无同步保证的回退实现。

`ffl_atomic_try_lock_u8()` 执行 `0 -> 1` 的原子尝试加锁，成功返回非零；`ffl_atomic_unlock_u8()` 写回 `0`。`ffl_atomic_load_u8()`、`ffl_atomic_store_u8()`、`ffl_atomic_fetch_add_u8()`、`ffl_atomic_fetch_sub_u8()` 和 `ffl_atomic_compare_exchange_u8()` 分别提供原子读、写、返回旧值的加减以及比较交换。`uint8_t` 运算按 8 位无符号值回绕。

该封装只负责原子字节访问，不替代 MCU 中断并发场景所需的临界区 hook 或 port 实现。

## Host Smoke

两个 smoke target 使用 MinGW GCC/G++，并以 C11/C++17 和 `-Wall -Wextra -Werror` 编译：

```powershell
xmake build -P . ffl.atomic.test
xmake build -P . ffl.atomic.cxx-test
xmake run -P . ffl.atomic.test
xmake run -P . ffl.atomic.cxx-test
```
