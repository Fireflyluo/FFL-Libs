/**
 ******************************************************************************
 * @file    SC7A20_core_async.c
 * @brief   SC7A20HTR 核心驱动实现（异步版本）
 ******************************************************************************
 * @note    本文件实现传感器的核心驱动，避免动态内存分配
 *
 ******************************************************************************
 */

#include "sc7a20_core.h"
#include "sc7a20_async.h"

#if SC7A20_ASYNC_SUPPORT

#include <string.h>

/* ========================== 内部辅助函数 ========================== */


/**
 * @brief 转换原始数据到加速度数据结构
 */
void sc7a20_convert_raw_to_accel_data(sc7a20_handle_t handle, 
                                    const uint8_t *raw_data, 
                                    sc7a20_accel_data_t *accel_data)
{
    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    
    // 组合数据（考虑字节序）
    if (handle->endian == 0)
    {
        // 小端模式：低字节在前
        raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
        raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
        raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    }
    else
    {
        // 大端模式：高字节在前
        raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        raw_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        raw_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    }
    
    accel_data->x = raw_x >> 4; // 右移4位，提取12位有效数据
    accel_data->y = raw_y >> 4;
    accel_data->z = raw_z >> 4;
    
    // 转换为g值
    accel_data->x_g = accel_data->x * handle->sensitivity;
    accel_data->y_g = accel_data->y * handle->sensitivity;
    accel_data->z_g = accel_data->z * handle->sensitivity;
}

/**
 * @brief 异步操作状态机处理函数
 */
void sc7a20_process_async_operation(sc7a20_handle_t handle)
{
    // 当前实现中，状态机主要由I2C回调驱动
    // 这里可以添加额外的状态检查逻辑
}

/**
 * @brief 检查异步操作是否超时
 */
bool sc7a20_check_async_timeout(sc7a20_handle_t handle)
{
    if (handle->async_ctx.state == SC7A20_ASYNC_IDLE) {
        return false;
    }
    
    // 这里假设有一个获取当前时间的函数
    // 在实际TMOS环境中，可以使用tmos_clock_get()
    // 为了通用性，这里暂时返回false
    // 用户可以在应用层实现具体的超时检测
    return false;
}

/* ========================== 内部回调函数 ========================== */

/**
 * @brief 内部读完成回调函数
 */
void sc7a20_internal_read_callback(void *user_data, sc7a20_status_t status)
{
    sc7a20_handle_t handle = (sc7a20_handle_t)user_data;
    
    if (status == SC7A20_OK) {
        // 数据转换（从内部缓冲区到用户数据结构）
        if (handle->async_ctx.read_data != NULL) {
            // 根据操作类型进行不同的处理
            if (handle->async_ctx.is_read_operation) {
                // 加速度数据读取
                sc7a20_convert_raw_to_accel_data(handle, 
                                               handle->internal_buffer, 
                                               (sc7a20_accel_data_t*)handle->async_ctx.read_data);
            }
            // 其他类型的读取操作可以在这里扩展
        }
    }
    
    // 调用用户回调
    if (handle->async_ctx.callback) {
        handle->async_ctx.callback(handle, status);
    }
    
    // 重置状态
    handle->async_ctx.state = SC7A20_ASYNC_IDLE;
}

/**
 * @brief 内部写完成回调函数
 */
void sc7a20_internal_write_callback(void *user_data, sc7a20_status_t status)
{
    sc7a20_handle_t handle = (sc7a20_handle_t)user_data;
    
    // 调用用户回调
    if (handle->async_ctx.callback) {
        handle->async_ctx.callback(handle, status);
    }
    
    // 重置状态
    handle->async_ctx.state = SC7A20_ASYNC_IDLE;
}

/* ========================== 异步API实现 ========================== */

/**
 * @brief 异步写寄存器
 */
sc7a20_status_t sc7a20_write_register_async(sc7a20_handle_t handle, 
                                           uint8_t reg, 
                                           const uint8_t *data, 
                                           uint16_t len,
                                           void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || data == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 检查是否有正在进行的异步操作
    if (handle->async_ctx.state != SC7A20_ASYNC_IDLE)
    {
        return SC7A20_ERROR; // 或者加入队列
    }
    
    // 检查硬件操作接口是否支持异步写
    if (handle->ops.write_async == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 设置异步上下文
    handle->async_ctx.state = SC7A20_ASYNC_WRITE_DATA;
    handle->async_ctx.reg_addr = reg;
    handle->async_ctx.write_data = data;
    handle->async_ctx.data_len = len;
    handle->async_ctx.is_read_operation = false;
    handle->async_ctx.callback = callback;
    handle->async_ctx.start_time = 0; // 需要实际的时间获取函数
    handle->async_ctx.timeout_ms = SC7A20_ASYNC_DEFAULT_TIMEOUT_MS;
    
    // 调用底层异步写函数
    sc7a20_status_t status = handle->ops.write_async(reg, data, len, sc7a20_internal_write_callback);
    
    if (status != SC7A20_OK) {
        handle->async_ctx.state = SC7A20_ASYNC_IDLE;
        return status;
    }
    
    return SC7A20_OK;
}

/**
 * @brief 异步读寄存器
 */
sc7a20_status_t sc7a20_read_register_async(sc7a20_handle_t handle, 
                                          uint8_t reg, 
                                          uint8_t *data, 
                                          uint16_t len,
                                          void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || data == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 检查是否有正在进行的异步操作
    if (handle->async_ctx.state != SC7A20_ASYNC_IDLE)
    {
        return SC7A20_ERROR;
    }
    
    // 检查硬件操作接口是否支持异步读
    if (handle->ops.read_async == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 设置异步上下文
    handle->async_ctx.state = SC7A20_ASYNC_READ_DATA;
    handle->async_ctx.reg_addr = reg;
    handle->async_ctx.read_data = data;
    handle->async_ctx.data_len = len;
    handle->async_ctx.is_read_operation = false; // 普通寄存器读取
    handle->async_ctx.callback = callback;
    handle->async_ctx.start_time = 0;
    handle->async_ctx.timeout_ms = SC7A20_ASYNC_DEFAULT_TIMEOUT_MS;
    
    // 调用底层异步读函数
    sc7a20_status_t status = handle->ops.read_async(reg, data, len, sc7a20_internal_read_callback);
    
    if (status != SC7A20_OK) {
        handle->async_ctx.state = SC7A20_ASYNC_IDLE;
        return status;
    }
    
    return SC7A20_OK;
}

/**
 * @brief 异步读取加速度数据
 */
sc7a20_status_t sc7a20_read_acceleration_async(sc7a20_handle_t handle,
                                              sc7a20_accel_data_t *data,
                                              void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || data == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 检查是否有正在进行的异步操作
    if (handle->async_ctx.state != SC7A20_ASYNC_IDLE)
    {
        return SC7A20_ERROR;
    }
    
    // 检查硬件操作接口是否支持异步读
    if (handle->ops.read_async == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 设置异步上下文
    handle->async_ctx.state = SC7A20_ASYNC_READ_DATA;
    handle->async_ctx.reg_addr = SC7A20_OUTX_L | 0x80;
    handle->async_ctx.read_data = (uint8_t*)data; // 临时存储，实际使用内部缓冲区
    handle->async_ctx.data_len = 6; // 6字节加速度数据
    handle->async_ctx.is_read_operation = true; // 标记为加速度数据读取
    handle->async_ctx.callback = callback;
    handle->async_ctx.start_time = 0;
    handle->async_ctx.timeout_ms = SC7A20_ASYNC_DEFAULT_TIMEOUT_MS;
    
    // 调用底层异步读函数，读取到内部缓冲区
    sc7a20_status_t status = handle->ops.read_async(SC7A20_OUTX_L | 0x80, 
                                                  handle->internal_buffer, 
                                                  6,
                                                  sc7a20_internal_read_callback);
    
    if (status != SC7A20_OK) {
        handle->async_ctx.state = SC7A20_ASYNC_IDLE;
        return status;
    }
    
    return SC7A20_OK;
}

/**
 * @brief 异步读取原始数据
 */
sc7a20_status_t sc7a20_read_raw_data_async(sc7a20_handle_t handle,
                                          int16_t *x, int16_t *y, int16_t *z,
                                          void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || x == NULL || y == NULL || z == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 检查是否有正在进行的异步操作
    if (handle->async_ctx.state != SC7A20_ASYNC_IDLE)
    {
        return SC7A20_ERROR;
    }
    
    // 检查硬件操作接口是否支持异步读
    if (handle->ops.read_async == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 创建临时加速度数据结构用于内部处理
    static sc7a20_accel_data_t temp_data;
    temp_data.x = *x; // 这些值会被覆盖，只是用于传递指针
    temp_data.y = *y;
    temp_data.z = *z;
    
    // 设置异步上下文
    handle->async_ctx.state = SC7A20_ASYNC_READ_DATA;
    handle->async_ctx.reg_addr = SC7A20_OUTX_L | 0x80;
    handle->async_ctx.read_data = (uint8_t*)&temp_data;
    handle->async_ctx.data_len = 6;
    handle->async_ctx.is_read_operation = true;
    handle->async_ctx.callback = callback;
    handle->async_ctx.start_time = 0;
    handle->async_ctx.timeout_ms = SC7A20_ASYNC_DEFAULT_TIMEOUT_MS;
    
    // 调用底层异步读函数
    sc7a20_status_t status = handle->ops.read_async(SC7A20_OUTX_L | 0x80, 
                                                  handle->internal_buffer, 
                                                  6,
                                                  sc7a20_internal_read_callback);
    
    if (status != SC7A20_OK) {
        handle->async_ctx.state = SC7A20_ASYNC_IDLE;
        return status;
    }
    
    return SC7A20_OK;
}

/**
 * @brief 异步设置加速度计量程
 */
sc7a20_status_t sc7a20_set_range_async(sc7a20_handle_t handle, 
                                      sc7a20_accel_fs_t range,
                                      void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    if (range > SC7A20_ACCEL_FS_16G)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 读取当前CTRL4寄存器（同步方式，因为配置读取通常很快）
    sc7a20_ctrl4_t ctrl4;
    sc7a20_status_t status = read_register(handle, SC7A20_CTRL4, &ctrl4.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    
    // 更新量程设置
    ctrl4.bit.fs = range;
    
    // 异步写回寄存器
    return sc7a20_write_register_async(handle, SC7A20_CTRL4, &ctrl4.reg, 1, callback);
}

/**
 * @brief 异步设置输出数据率
 */
sc7a20_status_t sc7a20_set_output_data_rate_async(sc7a20_handle_t handle, 
                                                 sc7a20_accel_odr_t odr,
                                                 void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    if (odr > SC7A20_ACCEL_ODR_4_434KHZ)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    // 读取当前CTRL1寄存器（同步方式）
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    
    // 更新数据率设置
    ctrl1.bit.ODR = odr;
    
    // 异步写回寄存器
    return sc7a20_write_register_async(handle, SC7A20_CTRL1, &ctrl1.reg, 1, callback);
}

/**
 * @brief 异步设置轴使能状态
 */
sc7a20_status_t sc7a20_set_axis_enable_async(sc7a20_handle_t handle, 
                                            bool x_enable, bool y_enable, bool z_enable,
                                            void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 读取当前CTRL1寄存器（同步方式）
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    
    // 更新轴使能设置
    ctrl1.bit.Xen = x_enable ? 1 : 0;
    ctrl1.bit.Yen = y_enable ? 1 : 0;
    ctrl1.bit.Zen = z_enable ? 1 : 0;
    
    // 异步写回寄存器
    return sc7a20_write_register_async(handle, SC7A20_CTRL1, &ctrl1.reg, 1, callback);
}

/**
 * @brief 异步设置电源模式
 */
sc7a20_status_t sc7a20_set_power_mode_async(sc7a20_handle_t handle, 
                                           sc7a20_power_mode_t mode,
                                           void (*callback)(sc7a20_handle_t handle, sc7a20_status_t status))
{
    if (handle == NULL || callback == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }
    
    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }
    
    // 读取当前CTRL0和CTRL1寄存器（同步方式）
    sc7a20_ctrl0_t ctrl0;
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status;
    
    status = read_register(handle, SC7A20_CTRL0, &ctrl0.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    
    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    
    // 根据模式配置寄存器
    switch (mode)
    {
    case SC7A20_POWER_DOWN:
        ctrl1.bit.ODR = SC7A20_ACCEL_ODR_POWER_DOWN;
        break;
        
    case SC7A20_LOW_POWER:
        ctrl1.bit.LPen = 1;
        ctrl0.bit.HR   = 0;
        break;
        
    case SC7A20_NORMAL:
        ctrl1.bit.LPen = 0;
        ctrl0.bit.HR   = 0;
        break;
        
    case SC7A20_HIGH_RESOLUTION:
        ctrl1.bit.LPen = 0;
        ctrl0.bit.HR   = 1;
        break;
        
    default:
        return SC7A20_INVALID_PARAM;
    }
    
    // 异步写回CTRL0和CTRL1寄存器
    // 注意：这里需要分两次写入，或者使用特殊的批量写入机制
    // 为了简化，我们先写CTRL0，然后在回调中写CTRL1
    
    // 创建临时上下文保存CTRL1数据和最终回调
    static struct {
        sc7a20_ctrl1_t ctrl1;
        void (*final_callback)(sc7a20_handle_t, sc7a20_status_t);
        sc7a20_handle_t handle;
    } power_mode_ctx;
    
    power_mode_ctx.ctrl1 = ctrl1;
    power_mode_ctx.final_callback = callback;
    power_mode_ctx.handle = handle;
    
    // 定义中间回调函数
    static void power_mode_step2_callback(sc7a20_handle_t h, sc7a20_status_t s) {
        if (s == SC7A20_OK) {
            // 第一步成功，继续写CTRL1
            sc7a20_write_register_async(power_mode_ctx.handle, SC7A20_CTRL1, 
                                       &power_mode_ctx.ctrl1.reg, 1, 
                                       power_mode_ctx.final_callback);
        } else {
            // 第一步失败，直接调用最终回调
            power_mode_ctx.final_callback(h, s);
        }
    }
    
    // 先写CTRL0
    status = sc7a20_write_register_async(handle, SC7A20_CTRL0, &ctrl0.reg, 1, power_mode_step2_callback);
    return status;
}

#endif // SC7A20_ASYNC_SUPPORT