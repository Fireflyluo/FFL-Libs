#include "ringbuffer.h"

#include <string.h>

/**
 * @file ringbuffer.c
 * @brief 基于镜像索引的固定容量字节环形缓冲区实现。
 */

static int ringbuffer_is_valid(const ringbuffer_t *rb)
{
    return rb != NULL && rb->buffer_ptr != NULL && rb->buffer_size != 0U &&
           rb->read_index < rb->buffer_size && rb->write_index < rb->buffer_size;
}

static size_t ringbuffer_data_length(const ringbuffer_t *rb)
{
    if (rb->read_index == rb->write_index) {
        return rb->read_mirror == rb->write_mirror ? 0U : rb->buffer_size;
    }

    if (rb->write_index > rb->read_index) {
        return rb->write_index - rb->read_index;
    }

    return rb->buffer_size - (rb->read_index - rb->write_index);
}

static size_t ringbuffer_space_length(const ringbuffer_t *rb)
{
    return rb->buffer_size - ringbuffer_data_length(rb);
}

static void ringbuffer_copy_in(ringbuffer_t *rb, const uint8_t *source, size_t length)
{
    size_t first_length = rb->buffer_size - rb->write_index;

    if (first_length > length) {
        first_length = length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index], source, first_length);
    memcpy(rb->buffer_ptr, source + first_length, length - first_length);
}

static void ringbuffer_copy_out(ringbuffer_t *rb, uint8_t *destination, size_t length)
{
    size_t first_length = rb->buffer_size - rb->read_index;

    if (first_length > length) {
        first_length = length;
    }

    memcpy(destination, &rb->buffer_ptr[rb->read_index], first_length);
    memcpy(destination + first_length, rb->buffer_ptr, length - first_length);
}

static void ringbuffer_advance_read(ringbuffer_t *rb, size_t length)
{
    size_t next_index = rb->read_index + length;

    if (next_index >= rb->buffer_size) {
        rb->read_mirror ^= 1U;
    }

    rb->read_index = next_index % rb->buffer_size;
}

static void ringbuffer_advance_write(ringbuffer_t *rb, size_t length)
{
    size_t next_index = rb->write_index + length;

    if (next_index >= rb->buffer_size) {
        rb->write_mirror ^= 1U;
    }

    rb->write_index = next_index % rb->buffer_size;
}

void ringbuffer_init(ringbuffer_t *rb, uint8_t *pool, size_t size)
{
    if (rb == NULL) {
        return;
    }

    rb->buffer_ptr = pool;
    rb->buffer_size = pool == NULL ? 0U : size;
    rb->read_index = 0U;
    rb->write_index = 0U;
    rb->read_mirror = 0U;
    rb->write_mirror = 0U;
}

size_t ringbuffer_put(ringbuffer_t *rb, const uint8_t *ptr, size_t length)
{
    if (!ringbuffer_is_valid(rb) || (ptr == NULL && length != 0U)) {
        return 0U;
    }

    if (length == 0U) {
        return 0U;
    }

    if (length > ringbuffer_space_length(rb)) {
        length = ringbuffer_space_length(rb);
    }

    ringbuffer_copy_in(rb, ptr, length);
    ringbuffer_advance_write(rb, length);
    return length;
}

size_t ringbuffer_put_force(ringbuffer_t *rb, const uint8_t *ptr, size_t length)
{
    size_t space_length;

    if (!ringbuffer_is_valid(rb) || (ptr == NULL && length != 0U)) {
        return 0U;
    }

    if (length == 0U) {
        return 0U;
    }

    if (length > rb->buffer_size) {
        ptr += length - rb->buffer_size;
        length = rb->buffer_size;
    }

    space_length = ringbuffer_space_length(rb);
    if (length > space_length) {
        ringbuffer_advance_read(rb, length - space_length);
    }

    ringbuffer_copy_in(rb, ptr, length);
    ringbuffer_advance_write(rb, length);
    return length;
}

size_t ringbuffer_get(ringbuffer_t *rb, uint8_t *ptr, size_t length)
{
    if (!ringbuffer_is_valid(rb) || (ptr == NULL && length != 0U)) {
        return 0U;
    }

    if (length == 0U) {
        return 0U;
    }

    if (length > ringbuffer_data_length(rb)) {
        length = ringbuffer_data_length(rb);
    }

    ringbuffer_copy_out(rb, ptr, length);
    ringbuffer_advance_read(rb, length);
    return length;
}

ringbuffer_state ringbuffer_status(ringbuffer_t *rb)
{
    if (!ringbuffer_is_valid(rb)) {
        return RINGBUFFER_ERROR;
    }

    if (rb->read_index != rb->write_index) {
        return RINGBUFFER_HALFFULL;
    }

    return rb->read_mirror == rb->write_mirror ? RINGBUFFER_EMPTY : RINGBUFFER_FULL;
}
