#include "sc7a20_core.h"

static void sc7a20_fifo_decode_axes(sc7a20_handle_t handle,
                                    const uint8_t *buffer,
                                    int16_t *x,
                                    int16_t *y,
                                    int16_t *z)
{
    if (handle->endian == 0)
    {
        *x = ((int16_t)((buffer[1] << 8) | buffer[0])) >> 4;
        *y = ((int16_t)((buffer[3] << 8) | buffer[2])) >> 4;
        *z = ((int16_t)((buffer[5] << 8) | buffer[4])) >> 4;
    }
    else
    {
        *x = ((int16_t)((buffer[0] << 8) | buffer[1])) >> 4;
        *y = ((int16_t)((buffer[2] << 8) | buffer[3])) >> 4;
        *z = ((int16_t)((buffer[4] << 8) | buffer[5])) >> 4;
    }
}

static void sc7a20_fifo_fill_accel(sc7a20_handle_t handle, const uint8_t *buffer, sc7a20_accel_data_t *data)
{
    sc7a20_fifo_decode_axes(handle, buffer, &data->x, &data->y, &data->z);
    data->x_g = data->x * handle->sensitivity;
    data->y_g = data->y * handle->sensitivity;
    data->z_g = data->z * handle->sensitivity;
}

sc7a20_status_t sc7a20_get_fifo_src(sc7a20_handle_t handle, sc7a20_fifo_src_t *fifo_src)
{
    sc7a20_status_t status;

    if (handle == NULL || fifo_src == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    status = handle->ops.read(SC7A20_FIFO_SRC, &fifo_src->reg, 1);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_read_fifo_raw_data(sc7a20_handle_t handle, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[7];
    sc7a20_status_t status;

    if (handle == NULL || x == NULL || y == NULL || z == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    /* Prefer a 0x27 burst for FIFO sample reads. This path works on I2C and
     * avoids SPI_CTRL.ADR_SPI_AD6 bank switching when the transport is SPI. */
    status = handle->ops.read(SC7A20_DRDY_STATUS | 0x80, buffer, 7);
    if (status != SC7A20_OK)
    {
        handle->error_count++;
        handle->last_status = status;
        return status;
    }

    sc7a20_fifo_decode_axes(handle, &buffer[1], x, y, z);
    handle->read_count++;
    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_read_fifo_acceleration(sc7a20_handle_t handle, sc7a20_accel_data_t *data)
{
    uint8_t buffer[7];
    sc7a20_status_t status;

    if (handle == NULL || data == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    /* Prefer a 0x27 burst for FIFO sample reads. This path works on I2C and
     * avoids SPI_CTRL.ADR_SPI_AD6 bank switching when the transport is SPI. */
    status = handle->ops.read(SC7A20_DRDY_STATUS | 0x80, buffer, 7);
    if (status != SC7A20_OK)
    {
        handle->error_count++;
        handle->last_status = status;
        return status;
    }

    sc7a20_fifo_fill_accel(handle, &buffer[1], data);
    handle->read_count++;
    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}
