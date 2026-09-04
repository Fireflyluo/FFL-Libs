#include "ringbuffer.h"

/**
 * @brief 初始化环形缓冲区
 *
 * @param rb            指向环形缓冲区对象的指针
 * @param pool          指向缓冲区池的指针
 * @param size          缓冲区大小
 */
void ringbuffer_init(ringbuffer_t *rb, uint8_t *pool, size_t size)
{
    if (rb == NULL || size == 0)
    {
        return;
    }

    // 初始化读写索引
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    // 设置缓冲区池和大小
    rb->buffer_ptr = pool;
    rb->buffer_size = size - (size % 4); // 假设对齐大小为4
}

/**
 * @brief 将一块数据放入环形缓冲区。如果容量不足，将丢弃超出范围的数据。
 *
 * @param rb            指向环形缓冲区对象的指针
 * @param ptr           指向数据缓冲区的指针
 * @param length        数据大小（字节）
 *
 * @return 返回放入环形缓冲区的数据大小
 */
size_t ringbuffer_put(ringbuffer_t *rb, const uint8_t *ptr, size_t length)
{
    size_t size;

    if (rb == NULL)
    {
        return 0;
    }

    // 是否有足够的空间
    size = rb->buffer_size - (rb->write_index - rb->read_index);
    if (rb->write_index < rb->read_index)
    {
        size = rb->read_index - rb->write_index;
    }

    // 没有空间
    if (size == 0)
    {
        return 0;
    }

    // 丢弃超出范围的数据
    if (size < length)
    {
        length = size;
    }

    if (rb->buffer_size - rb->write_index > length)
    {
        // read_index - write_index = 空闲空间
        memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        // 这不会导致溢出，因为在当前镜像中有足够的空间
        rb->write_index += length;
        return length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index], &ptr[0], rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer_ptr[0], &ptr[rb->buffer_size - rb->write_index], length - (rb->buffer_size - rb->write_index));

    // 进入镜像区间
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    return length;
}

/**
 * @brief 将一块数据放入环形缓冲区。如果容量不足，将覆盖环形缓冲区中的现有数据。
 *
 * @param rb            指向环形缓冲区对象的指针
 * @param ptr           指向数据缓冲区的指针
 * @param length        数据大小（字节）
 *
 * @return 返回放入环形缓冲区的数据大小
 */
size_t ringbuffer_put_force(ringbuffer_t *rb, const uint8_t *ptr, size_t length)
{
    size_t space_length;

    if (rb == NULL)
    {
        return 0;
    }

    space_length = rb->buffer_size - (rb->write_index - rb->read_index);
    if (rb->write_index < rb->read_index)
    {
        space_length = rb->read_index - rb->write_index;
    }

    if (length > rb->buffer_size)
    {
        ptr = &ptr[length - rb->buffer_size];
        length = rb->buffer_size;
    }

    if (rb->buffer_size - rb->write_index > length)
    {
        // read_index - write_index = 空闲空间
        memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        // 这不会导致溢出，因为在当前镜像中有足够的空间
        rb->write_index += length;
        if (length > space_length)
        {
            rb->read_index = rb->write_index;
        }

        return length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index], &ptr[0], rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer_ptr[0], &ptr[rb->buffer_size - rb->write_index], length - (rb->buffer_size - rb->write_index));

    // 进入镜像区间
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    if (length > space_length)
    {
        if (rb->write_index <= rb->read_index)
        {
            rb->read_mirror = ~rb->read_mirror;
        }
        rb->read_index = rb->write_index;
    }

    return length;
}

/**
 * @brief 从环形缓冲区中获取数据。
 *
 * @param rb            指向环形缓冲区的指针
 * @param ptr           指向数据缓冲区的指针
 * @param length        想要从环形缓冲区中读取的数据大小
 *
 * @return 返回从环形缓冲区中读取的数据大小
 */
size_t ringbuffer_get(ringbuffer_t *rb, uint8_t *ptr, size_t length)
{
    size_t size;

    if (rb == NULL)
    {
        return 0;
    }

    // 是否有足够的数据
    size = rb->buffer_size - (rb->write_index - rb->read_index);
    if (rb->write_index < rb->read_index)
    {
        size = rb->read_index - rb->write_index;
    }

    // 没有数据
    if (size == 0)
    {
        return 0;
    }

    // 数据不足
    if (size < length)
    {
        length = size;
    }

    if (rb->buffer_size - rb->read_index > length)
    {
        // 复制所有数据
        memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        // 这不会导致溢出，因为在当前镜像中有足够的空间
        rb->read_index += length;
        return length;
    }

    memcpy(&ptr[0], &rb->buffer_ptr[rb->read_index], rb->buffer_size - rb->read_index);
    memcpy(&ptr[rb->buffer_size - rb->read_index], &rb->buffer_ptr[0], length - (rb->buffer_size - rb->read_index));

    // 进入镜像区间
    rb->read_mirror = ~rb->read_mirror;
    rb->read_index = length - (rb->buffer_size - rb->read_index);

    return length;
}

/**
 * @brief 获取环形缓冲区的状态
 * @param rb  指向环形缓冲区对象的指针
 * @return 返回环形缓冲区的状态
 * 在读写指针的值相同情况下，如果二者的指示位相同，说明缓冲区为空；如果二者的指示位不同，说明缓冲区为满。
 */
inline ringbuffer_state ringbuffer_status(ringbuffer_t *rb)
{
    if (rb == NULL)
    {
        return RINGBUFFER_ERROR;
    }

    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
        {
            return RINGBUFFER_EMPTY;
        }
        else
        {
            return RINGBUFFER_FULL;
        }
    }
    return RINGBUFFER_HALFFULL;
}