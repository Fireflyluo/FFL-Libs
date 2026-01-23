/*-----------------------------------------------File Info------------------------------------------------
** File Name:               qmi8658a_reg.c  
** Last modified date:      2025.8.16
** Last version:            V0.1
** Description:             QMI8658A寄存器硬件抽象层
**
**--------------------------------------------------------------------------------------------------------            
** Created date:            2025.7.1
** author:                  Fireflyluo
** Version:                 V0.1
** Descriptions:            提供最底层的寄存器访问能力
**                          实现物理通信接口（I2C）
**                          定义所有寄存器地址和位域结构
**--------------------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif
  
#include "qmi8658a_reg.h"
#include "py32f4xx_hal.h"

/* 私有宏定义 */
#define QMI8658A_I2C_ADDR    0x6A  // 默认I2C地址（SA0=0时）



/**
  * @brief  读取寄存器（I2C实现 南向接口）
  * @param  handle: 设备句柄（I2C_HandleTypeDef指针）
  * @param  reg: 寄存器地址
  * @param  data: 读取数据缓冲区
  * @param  len: 读取数据长度
  * @retval 0: 成功；非0: 失败
  */
 int32_t qmi8658a_i2c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c, 
        QMI8658A_I2C_ADDR << 1, // I2C设备地址（左移1位，保留R/W位）
        reg,                    // 目标寄存器地址
        I2C_MEMADD_SIZE_8BIT,   // 寄存器地址长度（8位）
        data,                   // 数据缓冲区
        len,                    // 读取长度
        100                     // 超时时间（ms）
    );
    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  写入寄存器（I2C实现 南向接口）
  * @param  handle: 设备句柄（I2C_HandleTypeDef指针）
  * @param  reg: 寄存器地址
  * @param  data: 待写入数据缓冲区
  * @param  len: 写入数据长度
  * @retval 0: 成功；非0: 失败
  */
 int32_t qmi8658a_i2c_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        hi2c,  
        QMI8658A_I2C_ADDR << 1, // I2C设备地址（左移1位，保留R/W位）
        reg,                    // 目标寄存器地址
        I2C_MEMADD_SIZE_8BIT,   // 寄存器地址长度（8位）
        data,                   // 数据缓冲区
        len,                    // 读取长度
        100                     // 超时时间（ms）
    );
    return (status == HAL_OK) ? 0 : -1;
}


/* 基础寄存器操作 */
int32_t qmi8658a_read_reg(qmi8658a_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (ctx == NULL || ctx->read_reg == NULL) return -1;
    return ctx->read_reg(ctx->handle, reg, data, len);
}

int32_t qmi8658a_write_reg(qmi8658a_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (ctx == NULL || ctx->write_reg == NULL) return -1;
    return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  执行软复位
  * @param  ctx: 驱动上下文指针
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_soft_reset(qmi8658a_ctx_t *ctx)
{
    uint8_t reset_cmd = 0xB0;
    int32_t ret = qmi8658a_write_reg(ctx, QMI8658A_RESET, &reset_cmd, 1);
    
    /* 等待复位完成 */
    HAL_Delay(15); // 复位时间约15ms
    
    return ret;
}

/**
  * @brief  读取设备ID（验证设备连接）
  * @param  ctx: 驱动上下文指针
  * @param  buff: 存储设备ID的缓冲区（至少1字节）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_device_id_get(qmi8658a_ctx_t *ctx, uint8_t *buff)
{
    return qmi8658a_read_reg(ctx, QMI8658A_WHO_AM_I, buff, 1);
}

/**
  * @brief  设置加速度计输出数据率（ODR）
  * @param  ctx: 驱动上下文指针
  * @param  val: 目标ODR（qmi8658a_accel_odr_t枚举值）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_accel_data_rate_set(qmi8658a_ctx_t *ctx, qmi8658a_accel_odr_t val)
{
    qmi8658a_ctrl2_t ctrl2;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
    if (ret != 0) return ret;
    ctrl2.bit.aODR = val;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
}

/**
  * @brief  获取加速度计输出数据率（ODR）
  * @param  ctx: 驱动上下文指针
  * @param  val: 存储当前ODR的变量（qmi8658a_accel_odr_t指针）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_accel_data_rate_get(qmi8658a_ctx_t *ctx, qmi8658a_accel_odr_t *val)
{
    qmi8658a_ctrl2_t ctrl2;
    int32_t ret;

    ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
    if (ret != 0) {
        return ret;  // 读取失败
    }

    *val = (qmi8658a_accel_odr_t)ctrl2.bit.aODR;
    return 0;
}

/**
  * @brief  设置加速度计量程（Full Scale）
  * @param  ctx: 驱动上下文指针
  * @param  val: 目标量程（qmi8658a_accel_fs_t枚举值）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_accel_full_scale_set(qmi8658a_ctx_t *ctx, qmi8658a_accel_fs_t val)
{
    qmi8658a_ctrl2_t ctrl2;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
    if (ret != 0) return ret;
    ctrl2.bit.aFS = val;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
}

/**
  * @brief  获取加速度计量程（Full Scale）
  * @param  ctx: 驱动上下文指针
  * @param  val: 存储当前量程的变量（qmi8658a_accel_fs_t指针）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_accel_full_scale_get(qmi8658a_ctx_t *ctx, qmi8658a_accel_fs_t *val)
{
    qmi8658a_ctrl2_t ctrl2;
    int32_t ret;

    ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL2, (uint8_t *)&ctrl2, 1);
    if (ret != 0) {
        return ret;
    }

    *val = (qmi8658a_accel_fs_t)ctrl2.bit.aFS;
    return 0;
}

/**
  * @brief  设置陀螺仪输出数据率（ODR）
  * @param  ctx: 驱动上下文指针
  * @param  val: 目标ODR（qmi8658a_gyro_odr_t枚举值）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_gyro_data_rate_set(qmi8658a_ctx_t *ctx, qmi8658a_gyro_odr_t val)
{
    qmi8658a_ctrl3_t ctrl3;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0) return ret;
    ctrl3.bit.gODR = val;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
}
/**
  * @brief  获取陀螺仪输出数据率（ODR）
  * @param  ctx: 驱动上下文指针
  * @param  val: 存储当前ODR的变量（qmi8658a_gyro_odr_t指针）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_gyro_data_rate_get(qmi8658a_ctx_t *ctx, qmi8658a_gyro_odr_t *val)
{
    qmi8658a_ctrl3_t ctrl3;
    int32_t ret;

    ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0) {
        return ret;
    }

    *val = (qmi8658a_gyro_odr_t)ctrl3.bit.gODR;
    return 0;
}

/**
  * @brief  设置陀螺仪量程（Full Scale）
  * @param  ctx: 驱动上下文指针
  * @param  val: 目标量程（qmi8658a_gyro_fs_t枚举值）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_gyro_full_scale_set(qmi8658a_ctx_t *ctx, qmi8658a_gyro_fs_t val)
{
    qmi8658a_ctrl3_t ctrl3;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0) return ret;
    ctrl3.bit.gFS = val;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
}

/**
  * @brief  获取陀螺仪量程（Full Scale）
  * @param  ctx: 驱动上下文指针
  * @param  val: 存储当前量程的变量（qmi8658a_gyro_fs_t指针）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_gyro_full_scale_get(qmi8658a_ctx_t *ctx, qmi8658a_gyro_fs_t *val)
{
    qmi8658a_ctrl3_t ctrl3;
    int32_t ret;

    ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL3, (uint8_t *)&ctrl3, 1);
    if (ret != 0) {
        return ret;
    }

    *val = (qmi8658a_gyro_fs_t)ctrl3.bit.gFS;
    return 0;
}

/**
  * @brief  配置传感器滤波器
  * @param  ctx: 驱动上下文指针
  * @param  accel_filter: 加速度计滤波器配置
  * @param  gyro_filter: 陀螺仪滤波器配置
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_filter_config(qmi8658a_ctx_t *ctx, 
                              uint8_t accel_filter, 
                              uint8_t gyro_filter)
{
    qmi8658a_ctrl5_t ctrl5 = {0};
    
    /* 配置滤波器参数 */
    ctrl5.bit.aLPF_EN = (accel_filter & 0x01) ? 1 : 0;
    ctrl5.bit.aLPF_MODE = (accel_filter >> 1) & 0x03;
    ctrl5.bit.gLPF_EN = (gyro_filter & 0x01) ? 1 : 0;
    ctrl5.bit.gLPF_MODE = (gyro_filter >> 1) & 0x03;
    
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL5, (uint8_t*)&ctrl5, 1);
}

/**
  * @brief  使能/禁用加速度计
  * @param  ctx: 驱动上下文指针
  * @param  enable: 1=使能，0=禁用
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_accel_enable(qmi8658a_ctx_t *ctx, uint8_t enable)
{
    qmi8658a_ctrl7_t ctrl7;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL7, (uint8_t *)&ctrl7, 1);
    if (ret != 0) return ret;
    ctrl7.bit.aEN = enable;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL7, (uint8_t *)&ctrl7, 1);
}
/**
  * @brief  使能/禁用陀螺仪
  * @param  ctx: 驱动上下文指针
  * @param  enable: 1=使能，0=禁用
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_gyro_enable(qmi8658a_ctx_t *ctx, uint8_t enable)
{
    qmi8658a_ctrl7_t ctrl7;
    int32_t ret = qmi8658a_read_reg(ctx, QMI8658A_CTRL7, (uint8_t *)&ctrl7, 1);
    if (ret != 0) return ret;
    ctrl7.bit.gEN = enable;
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL7, (uint8_t *)&ctrl7, 1);
}
/**
  * @brief  读取原始温度数据（2字节）
  * @param  ctx: 驱动上下文指针
  * @param  buff: 存储温度数据的缓冲区（至少2字节）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_temperature_raw(qmi8658a_ctx_t *ctx, uint8_t *buff)
{
    return qmi8658a_read_reg(ctx, QMI8658A_OUT_TEMP_L, buff, 2);
}

/**
  * @brief  获取温度值（摄氏度）
  * @param  ctx: 驱动上下文指针
  * @param  temp: 存储温度值的指针
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_temperature_get(qmi8658a_ctx_t *ctx, float *temp)
{
    uint8_t data[2];
    int32_t ret;
    
    /* 读取温度原始数据 */
    ret = qmi8658a_read_reg(ctx, QMI8658A_OUT_TEMP_L, data, 2);
    if (ret != 0) return ret;
    
    /* 转换为摄氏度 */
    int16_t raw_temp = (int16_t)((data[1] << 8) | data[0]);
    *temp = raw_temp / 256.0f;  // 分辨率 1/256°C
    
    return 0;
}
/**
  * @brief  读取时间戳值
  * @param  ctx: 设备上下文指针
  * @param  timestamp: 存储时间戳的指针（24位值）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_timestamp_get(qmi8658a_ctx_t *ctx, uint32_t *timestamp)
{
    uint8_t data[3];
    int32_t ret;
    
    /* 读取时间戳寄存器（3字节） */
    ret = qmi8658a_read_reg(ctx, QMI8658A_TIMESTAMP_LOW, data, 3);
    if (ret != 0) return ret;
    
    /* 组合24位时间戳（小端格式） */
    *timestamp = (uint32_t)(data[2] << 16) | (data[1] << 8) | data[0];
    
    return 0;
}
/**
  * @brief  读取原始加速度计数据（6字节，X/Y/Z各2字节）
  * @param  ctx: 驱动上下文指针
  * @param  buff: 存储加速度数据的缓冲区（至少6字节）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_acceleration_raw(qmi8658a_ctx_t *ctx, uint8_t *buff)
{
    return qmi8658a_read_reg(ctx, QMI8658A_OUTX_L_A, buff, 6);
}
/**
  * @brief  读取原始陀螺仪数据（6字节，X/Y/Z各2字节）
  * @param  ctx: 驱动上下文指针
  * @param  buff: 存储陀螺仪数据的缓冲区（至少6字节）
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_angular_rate_raw(qmi8658a_ctx_t *ctx, uint8_t *buff)
{
    return qmi8658a_read_reg(ctx, QMI8658A_OUTX_L_G, buff, 6);
}


/**
  * @brief  配置FIFO
  * @param  ctx: 驱动上下文指针
  * @param  mode: FIFO工作模式
  * @param  size: FIFO大小
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_fifo_config(qmi8658a_ctx_t *ctx, 
                            qmi8658a_fifo_mode_t mode,
                            qmi8658a_fifo_size_t size)
{
    qmi8658a_fifo_ctrl_t ctrl = {0};
    
    /* 配置FIFO参数 */
    ctrl.bit.FIFO_MODE = mode;
    ctrl.bit.FIFO_SIZE = size;

    
    return qmi8658a_write_reg(ctx, QMI8658A_FIFO_CTRL, (uint8_t*)&ctrl, 1);
}
/**
  * @brief  发送CTRL9命令
  * @param  ctx: 驱动上下文指针
  * @param  cmd: 命令值
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_ctrl9_command(qmi8658a_ctx_t *ctx, uint8_t cmd)
{
    /* 发送命令 */
    int32_t ret = qmi8658a_write_reg(ctx, QMI8658A_CTRL9, &cmd, 1);
    if (ret != 0) return ret;
    
    /* 检查命令完成状态 */
    qmi8658a_statusint_t status;
    uint32_t timeout = 100; // 超时计数器
    
    do {
        qmi8658a_read_reg(ctx, QMI8658A_STATUSINT, (uint8_t*)&status, 1);
        if (status.bit.CmdDone) break;
        HAL_Delay(1);
    } while (timeout-- > 0);
    
    if (!status.bit.CmdDone) return -1; // 超时
    
    /* 发送确认命令 */
    uint8_t ack = 0x00; // CTRL_CMD_ACK
    return qmi8658a_write_reg(ctx, QMI8658A_CTRL9, &ack, 1);
}
/**
  * @brief  读取FIFO数据
  * @param  ctx: 驱动上下文指针
  * @param  buffer: 存储FIFO数据的缓冲区
  * @param  buffer_size: 缓冲区大小
  * @retval 实际读取的字节数（成功），负值表示错误
  * 
  * 读取流程：
  * 1. 获取FIFO样本计数
  * 2. 计算需要读取的字节数
  * 3. 发送CTRL_CMD_REQ_FIFO命令进入读模式
  * 4. 从FIFO_DATA寄存器读取数据
  * 5. 关闭FIFO读模式
  */
int32_t qmi8658a_fifo_read(qmi8658a_ctx_t *ctx, uint8_t *buffer, uint16_t buffer_size)
{
    uint8_t fifo_status;
    uint8_t fifo_smpl_cnt;
    uint16_t sample_count;
    uint16_t bytes_to_read;
    int32_t ret;
    
    /* 步骤1：读取FIFO状态和样本计数 */
    ret = qmi8658a_read_reg(ctx, QMI8658A_FIFO_STATUS, &fifo_status, 1);
    if (ret != 0) return -1;
    
    ret = qmi8658a_read_reg(ctx, QMI8658A_FIFO_SMPL_CNT, &fifo_smpl_cnt, 1);
    if (ret != 0) return -2;
    
    /* 组合10位样本计数 (FIFO_STATUS[1:0] + FIFO_SMPL_CNT) */
    sample_count = ((fifo_status & 0x03) << 8) | fifo_smpl_cnt;
    
    /* 步骤2：计算需要读取的字节数 */
    bytes_to_read = sample_count * 2;  // 每个样本2字节
    
    /* 检查缓冲区是否足够 */
    if (bytes_to_read > buffer_size) {
        return -3;  // 缓冲区不足
    }
    
    /* 步骤3：发送FIFO读模式命令 */
    ret = qmi8658a_ctrl9_command(ctx, CTRL_CMD_REQ_FIFO);
    if (ret != 0) return -4;
    
    /* 步骤4：读取FIFO数据 */
    ret = qmi8658a_read_reg(ctx, QMI8658A_FIFO_DATA, buffer, bytes_to_read);
    if (ret != 0) {
        // 即使出错也要尝试关闭读模式
        qmi8658a_fifo_read_mode_disable(ctx);
        return -5;
    }
    
    /* 步骤5：关闭FIFO读模式 */
    ret = qmi8658a_fifo_read_mode_disable(ctx);
    if (ret != 0) return -6;
    
    return bytes_to_read;  // 返回实际读取的字节数
}

/**
  * @brief  禁用FIFO读模式
  * @param  ctx: 驱动上下文指针
  * @retval 0: 成功；非0: 失败
  */
int32_t qmi8658a_fifo_read_mode_disable(qmi8658a_ctx_t *ctx)
{
    qmi8658a_fifo_ctrl_t ctrl;
    int32_t ret;
    
    /* 读取当前FIFO控制寄存器 */
    ret = qmi8658a_read_reg(ctx, QMI8658A_FIFO_CTRL, (uint8_t *)&ctrl, 1);
    if (ret != 0) return ret;
    
    /* 关闭读模式 */
    ctrl.bit.FIFO_RD_MODE = 0;
    
    /* 写回寄存器 */
    return qmi8658a_write_reg(ctx, QMI8658A_FIFO_CTRL, (uint8_t *)&ctrl, 1);
}



#ifdef __cplusplus
}
#endif
