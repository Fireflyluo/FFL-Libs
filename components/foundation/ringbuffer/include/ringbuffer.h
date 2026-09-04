#ifndef FFL_RINGBUFFER_H
#define FFL_RINGBUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @file ringbuffer.h
 * @brief 固定容量字节环形缓冲区的 C/C++ 兼容公开接口。
 *
 * `ringbuffer_t` 不负责并发保护。中断与线程、DMA 回调与主循环等并发访问场景，
 * 调用方必须通过平台临界区或单生产者/单消费者约束完成同步。
 */

typedef enum {
    RINGBUFFER_EMPTY,
    RINGBUFFER_FULL,
    RINGBUFFER_HALFFULL,
    RINGBUFFER_ERROR
} ringbuffer_state;

typedef struct {
    uint8_t *buffer_ptr;
    size_t buffer_size;
    size_t read_index;
    size_t write_index;
    uint8_t read_mirror;
    uint8_t write_mirror;
} ringbuffer_t;

/** 初始化缓冲区；`pool == NULL` 或 `size == 0` 会重置为不可用状态。 */
void ringbuffer_init(ringbuffer_t *rb, uint8_t *pool, size_t size);
/** 写入数据；空间不足时丢弃尾部超出的输入，并返回实际写入字节数。 */
size_t ringbuffer_put(ringbuffer_t *rb, const uint8_t *ptr, size_t length);
/** 强制写入数据；空间不足时丢弃最旧数据，并返回实际保留的字节数。 */
size_t ringbuffer_put_force(ringbuffer_t *rb, const uint8_t *ptr, size_t length);
/** 读取最多 `length` 个已存字节，并返回实际读取字节数。 */
size_t ringbuffer_get(ringbuffer_t *rb, uint8_t *ptr, size_t length);
/** 返回空、满、非空非满或无效状态。 */
ringbuffer_state ringbuffer_status(ringbuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* FFL_RINGBUFFER_H */
