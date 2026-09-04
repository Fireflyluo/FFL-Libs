# Ringbuffer

`ffl.ringbuffer` 是一个固定容量的字节环形缓冲区。它不依赖 MCU HAL、RTOS 或板级资源，适合放在串口、DMA 回调和协议流缓存的上层。

## API 语义

- `ringbuffer_init()` 使用传入的完整 `size` 作为容量，不会因内部对齐缩减容量；`pool == NULL` 或 `size == 0` 会得到不可用实例。
- `ringbuffer_put()` 只写入当前空闲空间能容纳的数据，超出的输入被丢弃。
- `ringbuffer_put_force()` 永远保留最新数据；输入长度超过容量时，仅保留输入末尾的 `buffer_size` 字节。
- `ringbuffer_get()` 只读取已写入的数据，保持 FIFO 顺序。
- 组件不做锁或原子操作；并发访问必须由调用方的临界区或访问模型保证。

## C 与 C++ 使用

公开头文件为 C API，并以 `extern "C"` 防止 C++ 名字改编。因此 `.c` 实现可直接被 C 或 C++ 目标链接：

```lua
includes("third_party/embedded-libs/components/foundation/ringbuffer")

target("firmware")
    set_kind("binary")
    set_languages("c11", "cxx17")
    add_deps("ffl.ringbuffer")
```

## 验证

仓库根目录执行：

```powershell
xmake f -P .
xmake test -P . ffl.ringbuffer.test
```

测试用 C++ 编译并调用 C 实现，用于验证 C++ 消费方的头文件和链接兼容性。
