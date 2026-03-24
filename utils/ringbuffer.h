#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// 定义环形缓冲区的状态枚举
typedef enum {
    RINGBUFFER_EMPTY,    // 环形缓冲区状态码：空
    RINGBUFFER_FULL,     // 环形缓冲区状态码：满
    RINGBUFFER_HALFFULL, // 环形缓冲区状态码：半满
    RINGBUFFER_ERROR     // 环形缓冲区状态码：错误
} ringbuffer_state;

// 定义环形缓冲区结构体
typedef struct {
    uint8_t *buffer_ptr;  // 缓冲区指针
    size_t buffer_size;   // 缓冲区大小
    size_t read_index;    // 读索引
    size_t write_index;   // 写索引
    uint8_t read_mirror;  // 读镜像标志
    uint8_t write_mirror; // 写镜像标志
} ringbuffer_t;

// 初始化环形缓冲区
void ringbuffer_init(ringbuffer_t *rb, uint8_t *pool, size_t size);
// 环形缓冲区写操作，容量不足时丢弃超出范围数据
size_t ringbuffer_put(ringbuffer_t *rb, const uint8_t *ptr, size_t length);
// 形缓冲区写操作，容量不足时覆盖已有数据
size_t ringbuffer_put_force(ringbuffer_t *rb, const uint8_t *ptr, size_t length);
// 从环形缓冲区中获取数据
size_t ringbuffer_get(ringbuffer_t *rb, uint8_t *ptr, size_t length);
// 获取环形缓冲区的状态
ringbuffer_state ringbuffer_status(ringbuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H */