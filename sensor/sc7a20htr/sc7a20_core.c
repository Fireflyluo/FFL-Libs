/**
 ******************************************************************************
 * @file    SC7A20_core.c
 * @brief   SC7A20HTR 核心驱动实现
 ******************************************************************************
 * @note    本文件实现传感器的核心驱动，避免动态内存分配
 *
 ******************************************************************************
 */
#include "sc7a20_core.h"
#include <string.h>

/* ========================== 内部常量定义 ========================== */

// 灵敏度查找表（根据量程，单位：mg/LSB）
static const float sensitivity_table[] = {
    [SC7A20_ACCEL_FS_2G] = 0.9765625f, // ±2g
    [SC7A20_ACCEL_FS_4G] = 1.953125f,  // ±4g
    [SC7A20_ACCEL_FS_8G] = 3.90625f,   // ±8g
    [SC7A20_ACCEL_FS_16G] = 7.8125f    // ±16g
};

// 默认配置
static const sc7a20_config_t default_config = {
    .i2c_addr = SC7A20_I2C_ADDR_L,
    .range = SC7A20_ACCEL_FS_2G,
    .odr = SC7A20_ACCEL_ODR_100HZ,
    .enable_axis = {1, 1, 1},
    .block_data_update = true,
    .high_resolution_mode = false,
    .low_power_mode = false};

/* ========================== 设备上下文结构 ========================== */

/* ========================== 内部辅助函数 ========================== */

/**
 * @brief 读取单个寄存器
 */
static sc7a20_status_t read_register(sc7a20_handle_t handle, uint8_t reg, uint8_t *value)
{
    if (handle == NULL || value == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    return handle->ops.read(reg, value, 1);
}

/**
 * @brief 写入单个寄存器
 */
static sc7a20_status_t write_register(sc7a20_handle_t handle, uint8_t reg, uint8_t value)
{
    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    return handle->ops.write(reg, &value, 1);
}

/**
 * @brief 读取多个寄存器
 */
static sc7a20_status_t read_registers(sc7a20_handle_t handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (handle == NULL || data == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    return handle->ops.read(reg, data, len);
}

/**
 * @brief 验证设备通信
 */
static sc7a20_status_t verify_communication(sc7a20_handle_t handle)
{
    uint8_t chip_id = 0;
    sc7a20_status_t status;

    status = read_register(handle, SC7A20_WHO_AM_I, &chip_id);
    if (status != SC7A20_OK)
    {
        return status;
    }

    if (chip_id != SC7A20_CHIP_ID)
    {
        return SC7A20_DEVICE_NOT_FOUND;
    }

    handle->chip_id = chip_id;
    return SC7A20_OK;
}

/**
 * @brief 应用配置到寄存器
 */
static sc7a20_status_t apply_configuration(sc7a20_handle_t handle)
{
    sc7a20_status_t status = SC7A20_OK;
    sc7a20_ctrl0_t ctrl0 = {0};
    sc7a20_ctrl1_t ctrl1 = {0};
    sc7a20_ctrl2_t ctrl2 = {0};
    sc7a20_ctrl3_t ctrl3 = {0};
    sc7a20_ctrl4_t ctrl4 = {0};

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    // 配置CTRL0寄存器（高分辨率模式）
    ctrl0.bit.HR = handle->config.high_resolution_mode ? 1 : 0;
    // OSR位根据数据手册说明设置，这里使用默认值000（ODR）
    ctrl0.bit.OSR = 0; // 000: ODR

    status = write_register(handle, SC7A20_CTRL0, ctrl0.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }

    // 配置CTRL1寄存器（数据率、轴使能、功耗模式）
    ctrl1.bit.Xen = handle->config.enable_axis[0] ? 1 : 0;
    ctrl1.bit.Yen = handle->config.enable_axis[1] ? 1 : 0;
    ctrl1.bit.Zen = handle->config.enable_axis[2] ? 1 : 0;
    ctrl1.bit.LPen = handle->config.low_power_mode ? 1 : 0;
    ctrl1.bit.ODR = handle->config.odr;

    // ctrl2.reg = 0x47;//50HZ 正常模式 xyz使能
    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    ctrl2.reg = 0x00; // 关闭滤波器
    status = write_register(handle, SC7A20_CTRL2, ctrl2.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    ctrl3.reg = 0x00; // 关闭中断
    status = write_register(handle, SC7A20_CTRL3, ctrl3.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }
    // 配置CTRL4寄存器（量程、块数据更新、字节序等）
    ctrl4.reg = 0x08; // 读取完更新 小端 2g 高精度
    ctrl4.bit.fs = handle->config.range;

    status = write_register(handle, SC7A20_CTRL4, ctrl4.reg);
    if (status != SC7A20_OK)
    {
        return status;
    }

    // 更新灵敏度
    if (handle->config.range < sizeof(sensitivity_table) / sizeof(sensitivity_table[0]))
    {
        handle->sensitivity = sensitivity_table[handle->config.range] / 1000.0f; // 转换为g/LSB
    }

    return SC7A20_OK;
}

/* ========================== 公共接口实现 ========================== */

sc7a20_status_t sc7a20_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops,
                            const sc7a20_config_t *config)
{
    sc7a20_status_t status = SC7A20_OK;

    // 参数检查
    if (handle == NULL || ops == NULL || ops->write == NULL ||
        ops->read == NULL || ops->delay_ms == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    // 初始化设备结构
    memset(handle, 0, sizeof(sc7a20_dev_t));
    handle->ops = *ops;

    // 应用配置（使用默认配置或传入配置）
    if (config != NULL)
    {
        handle->config = *config;
    }
    else
    {
        handle->config = default_config;
    }

    // 软件复位
    status = write_register(handle, SC7A20_SOFT_RESET, 0xA5);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    handle->ops.delay_ms(10); // 等待复位完成

    // 验证设备通信
    status = verify_communication(handle);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 应用设备配置
    status = apply_configuration(handle);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 标记为已初始化
    handle->flags.is_initialized = 1;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_deinit(sc7a20_handle_t handle)
{
    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    // 进入电源关断模式
    sc7a20_ctrl1_t ctrl1 = {0};
    ctrl1.bit.ODR = SC7A20_ACCEL_ODR_POWER_DOWN;
    write_register(handle, SC7A20_CTRL1, ctrl1.reg);

    // 清除初始化标志
    handle->flags.is_initialized = 0;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_read_acceleration(sc7a20_handle_t handle, sc7a20_accel_data_t *data)
{
    uint8_t buffer[6];
    sc7a20_status_t status;

    if (handle == NULL || data == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取6字节加速度数据
    status = read_registers(handle, SC7A20_OUTX_L | 0x80, buffer, 6);
    if (status != SC7A20_OK)
    {
        handle->error_count++;
        handle->last_status = status;
        return status;
    }

    // 根据CTRL4.BLE位决定字节序（需要读取CTRL4寄存器）
    sc7a20_ctrl4_t ctrl4;
    status = read_register(handle, SC7A20_CTRL4, &ctrl4.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }
    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    // 组合数据（考虑字节序）
    if (ctrl4.bit.BLE == 0)
    {
        // 小端模式：低字节在前
        raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
    else
    {
        // 大端模式：高字节在前
        raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
        raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    }
    data->x = raw_x >> 4; // 右移4位，提取12位有效数据
    data->y = raw_y >> 4; 
    data->z = raw_z >> 4; 
    // 转换为g值
    data->x_g = data->x * handle->sensitivity;
    data->y_g = data->y * handle->sensitivity;
    data->z_g = data->z * handle->sensitivity;

    handle->read_count++;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_read_raw_data(sc7a20_handle_t handle, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    sc7a20_status_t status;

    if (handle == NULL || x == NULL || y == NULL || z == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取6字节原始数据
    status = read_registers(handle, SC7A20_OUTX_L | 0x80, buffer, 6);
    if (status != SC7A20_OK)
    {
        handle->error_count++;
        handle->last_status = status;
        return status;
    }
    // 根据CTRL4.BLE位决定字节序（需要读取CTRL4寄存器）
    sc7a20_ctrl4_t ctrl4;
    status = read_register(handle, SC7A20_CTRL4, &ctrl4.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }
    if (ctrl4.bit.BLE == 0)
    {
        // 组合数据（小端序）
        *x = ((int16_t)((buffer[1] << 8) | buffer[0])) >> 4;
        *y = ((int16_t)((buffer[3] << 8) | buffer[2])) >> 4;
        *z = ((int16_t)((buffer[5] << 8) | buffer[4])) >> 4;
    }
    else
    {
        // 组合数据（大端序）
        *x = ((int16_t)((buffer[0] << 8) | buffer[1])) >> 4;
        *y = ((int16_t)((buffer[2] << 8) | buffer[3])) >> 4;
        *z = ((int16_t)((buffer[4] << 8) | buffer[5])) >> 4;
    }
    handle->read_count++;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_is_data_ready(sc7a20_handle_t handle, bool *ready)
{
    sc7a20_drdy_status_t status_reg;
    sc7a20_status_t status;

    if (handle == NULL || ready == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    status = read_register(handle, SC7A20_DRDY_STATUS, &status_reg.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    *ready = (status_reg.bit.ZYXDA != 0);
    handle->flags.data_ready = *ready;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_read_new_data(sc7a20_handle_t handle, sc7a20_accel_data_t *data)
{
    uint8_t buffer[6];
    sc7a20_status_t status;

    if (handle == NULL || data == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 从新数据寄存器读取（避免数据覆盖）
    status = read_registers(handle, SC7A20_OUTX_New_L | 0x80, buffer, 6);
    if (status != SC7A20_OK)
    {
        handle->error_count++;
        handle->last_status = status;
        return status;
    }
    // 根据CTRL4.BLE位决定字节序（需要读取CTRL4寄存器）
    sc7a20_ctrl4_t ctrl4;
    status = read_register(handle, SC7A20_CTRL4, &ctrl4.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }
    if (ctrl4.bit.BLE == 0)
    {
        // 组合数据 (小端序)
        data->x = ((int16_t)((buffer[1] << 8) | buffer[0])) >> 4;
        data->y = ((int16_t)((buffer[3] << 8) | buffer[2])) >> 4;
        data->z = ((int16_t)((buffer[5] << 8) | buffer[4])) >> 4;
    }
    else
    {
        // 组合数据 (大端序)
        data->x = ((int16_t)((buffer[0] << 8) | buffer[1])) >> 4;
        data->y = ((int16_t)((buffer[2] << 8) | buffer[3])) >> 4;
        data->z = ((int16_t)((buffer[4] << 8) | buffer[5])) >> 4;
    }
    // 转换为g值
    data->x_g = data->x * handle->sensitivity;
    data->y_g = data->y * handle->sensitivity;
    data->z_g = data->z * handle->sensitivity;

    handle->read_count++;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_set_range(sc7a20_handle_t handle, sc7a20_accel_fs_t range)
{
    sc7a20_ctrl4_t ctrl4;
    sc7a20_status_t status;

    if (handle == NULL)
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

    // 读取当前CTRL4寄存器
    status = read_register(handle, SC7A20_CTRL4, &ctrl4.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新量程设置
    ctrl4.bit.fs = range;

    // 写回寄存器
    status = write_register(handle, SC7A20_CTRL4, ctrl4.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新配置和灵敏度
    handle->config.range = range;
    if (range < sizeof(sensitivity_table) / sizeof(sensitivity_table[0]))
    {
        handle->sensitivity = sensitivity_table[range] / 1000.0f; // 转换为g/LSB
    }

    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_set_output_data_rate(sc7a20_handle_t handle, sc7a20_accel_odr_t odr)
{
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status;

    if (handle == NULL)
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

    // 读取当前CTRL1寄存器
    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新数据率设置
    ctrl1.bit.ODR = odr;

    // 写回寄存器
    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新配置
    handle->config.odr = odr;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_set_axis_enable(sc7a20_handle_t handle, bool x_enable, bool y_enable, bool z_enable)
{
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取当前CTRL1寄存器
    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新轴使能设置
    ctrl1.bit.Xen = x_enable ? 1 : 0;
    ctrl1.bit.Yen = y_enable ? 1 : 0;
    ctrl1.bit.Zen = z_enable ? 1 : 0;

    // 写回寄存器
    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新配置
    handle->config.enable_axis[0] = x_enable;
    handle->config.enable_axis[1] = y_enable;
    handle->config.enable_axis[2] = z_enable;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_get_config(sc7a20_handle_t handle, sc7a20_config_t *config)
{
    if (handle == NULL || config == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    *config = handle->config;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_enter_sleep(sc7a20_handle_t handle)
{
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取当前CTRL1寄存器
    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 设置低功耗模式
    ctrl1.bit.LPen = 1;

    // 写回寄存器
    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    handle->flags.is_sleeping = 1;
    handle->config.low_power_mode = true;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_wakeup(sc7a20_handle_t handle)
{
    sc7a20_ctrl1_t ctrl1;
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取当前CTRL1寄存器
    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 清除低功耗模式
    ctrl1.bit.LPen = 0;

    // 写回寄存器
    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    handle->flags.is_sleeping = 0;
    handle->config.low_power_mode = false;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}
sc7a20_status_t sc7a20_set_power_mode(sc7a20_handle_t handle, sc7a20_power_mode_t mode)
{
    sc7a20_ctrl1_t ctrl1;
    sc7a20_ctrl0_t ctrl0; // 改为使用CTRL0寄存器
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 读取当前CTRL0和CTRL1寄存器
    status = read_register(handle, SC7A20_CTRL0, &ctrl0.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    status = read_register(handle, SC7A20_CTRL1, &ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
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
        ctrl0.bit.HR = 0; // 低功耗模式，HR=0
        break;

    case SC7A20_NORMAL:
        ctrl1.bit.LPen = 0;
        ctrl0.bit.HR = 0; // 正常模式，HR=0
        break;

    case SC7A20_HIGH_RESOLUTION:
        ctrl1.bit.LPen = 0;
        ctrl0.bit.HR = 1; // 高分辨率模式，HR=1
        break;

    default:
        return SC7A20_INVALID_PARAM;
    }

    // 写回CTRL0和CTRL1寄存器
    status = write_register(handle, SC7A20_CTRL0, ctrl0.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    status = write_register(handle, SC7A20_CTRL1, ctrl1.reg);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    // 更新配置
    handle->config.low_power_mode = (mode == SC7A20_LOW_POWER);
    handle->config.high_resolution_mode = (mode == SC7A20_HIGH_RESOLUTION);
    handle->flags.is_sleeping = (mode == SC7A20_POWER_DOWN) ? 1 : 0;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_get_chip_id(sc7a20_handle_t handle, uint8_t *chip_id)
{
    if (handle == NULL || chip_id == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    *chip_id = handle->chip_id;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_get_status(sc7a20_handle_t handle, uint8_t *status)
{
    sc7a20_drdy_status_t drdy_status;
    sc7a20_status_t ret;

    if (handle == NULL || status == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    ret = read_register(handle, SC7A20_DRDY_STATUS, &drdy_status.reg);
    if (ret != SC7A20_OK)
    {
        handle->last_status = ret;
        return ret;
    }

    *status = drdy_status.reg;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_reset(sc7a20_handle_t handle)
{
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    // 软件复位
    status = write_register(handle, SC7A20_SOFT_RESET, 0xA5);
    if (status != SC7A20_OK)
    {
        handle->last_status = status;
        return status;
    }

    handle->ops.delay_ms(10); // 等待复位完成

    // 重新初始化配置
    if (handle->flags.is_initialized)
    {
        status = apply_configuration(handle);
        if (status != SC7A20_OK)
        {
            handle->last_status = status;
            return status;
        }
    }

    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_is_ready(sc7a20_handle_t handle, bool *ready)
{
    if (handle == NULL || ready == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    *ready = handle->flags.is_initialized;
    handle->last_status = SC7A20_OK;

    return SC7A20_OK;
}

sc7a20_status_t sc7a20_config_high_performance(sc7a20_handle_t handle)
{
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 设置高分辨率模式
    status = sc7a20_set_power_mode(handle, SC7A20_HIGH_RESOLUTION);
    if (status != SC7A20_OK)
    {
        return status;
    }

    // 设置较高的数据率
    status = sc7a20_set_output_data_rate(handle, SC7A20_ACCEL_ODR_400HZ);
    if (status != SC7A20_OK)
    {
        return status;
    }

    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_config_low_power(sc7a20_handle_t handle)
{
    sc7a20_status_t status;

    if (handle == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    if (!handle->flags.is_initialized)
    {
        return SC7A20_NOT_INIT;
    }

    // 设置低功耗模式
    status = sc7a20_set_power_mode(handle, SC7A20_LOW_POWER);
    if (status != SC7A20_OK)
    {
        return status;
    }

    // 设置较低的数据率
    status = sc7a20_set_output_data_rate(handle, SC7A20_ACCEL_ODR_12_5HZ);
    if (status != SC7A20_OK)
    {
        return status;
    }

    handle->last_status = SC7A20_OK;
    return SC7A20_OK;
}

sc7a20_status_t sc7a20_quick_init(sc7a20_handle_t handle, const sc7a20_ops_t *ops, uint8_t i2c_addr)
{
    sc7a20_config_t config = default_config;

    if (handle == NULL || ops == NULL)
    {
        return SC7A20_INVALID_PARAM;
    }

    config.i2c_addr = i2c_addr;

    return sc7a20_init(handle, ops, &config);
}