// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied。
// See the License for the specific language governing permissions and
// limitations under the License。

#include <stdio.h>
#include "spi.h"
#include "xl2400.h"

extern SPI_HandleTypeDef hspi1;

/**
 * @brief  通过SPI发送并接收一个字节的数据
 * @param  data: 要发送的字节
 * @retval 接收到的字节
 */
uint8_t SPI_TxRxByte(uint8_t data)
{
    return SPI3_ReadWriteByte(data);
}

// 全局变量定义
uint8_t xl2400_state, cbuf[2], xbuf[XL2400_PL_WIDTH_MAX + 1];

/**
 * @brief  向指定寄存器写入一个字节
 * @param  reg: 寄存器地址
 * @param  value: 要写入的值
 * @retval 实际读取到的值（部分芯片返回状态）
 */
uint8_t XL2400_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t reg_val;
    XL2400_NSS_LOW();               // 拉低片选信号
    SPI_TxRxByte(reg);              // 发送寄存器地址
    reg_val = SPI_TxRxByte(value);  // 写入数据
    XL2400_NSS_HIGH();              // 拉高片选信号
    return reg_val;
}

/**
 * @brief  从指定寄存器读取一个字节
 * @param  reg: 寄存器地址
 * @retval 读取到的值
 */
uint8_t XL2400_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    XL2400_NSS_LOW();               // 拉低片选信号
    SPI_TxRxByte(reg);              // 发送寄存器地址
    reg_val = SPI_TxRxByte(XL2400_CMD_NOP);  // 读取数据
    XL2400_NSS_HIGH();              // 拉高片选信号
    return reg_val;
}

/**
 * @brief  向指定寄存器写入多个字节
 * @param  reg: 寄存器地址
 * @param  pBuf: 数据指针
 * @param  len: 数据长度
 */
void XL2400_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++) {
        SPI_TxRxByte(*pBuf++);
    }
    XL2400_NSS_HIGH();
}

/**
 * @brief  从指定寄存器读取多个字节
 * @param  reg: 寄存器地址
 * @param  pBuf: 缓冲区指针
 * @param  len: 读取长度
 */
void XL2400_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XL2400_NSS_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++) {
        pBuf[ctr] = SPI_TxRxByte(XL2400_CMD_NOP);
    }
    XL2400_NSS_HIGH();
}

/**
 * @brief  设置CE引脚为低电平（进入待机模式）
 */
void XL2400_CE_Low(void)
{
#ifdef USE_XL2400P
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0xEE);
#else
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    *(cbuf + 1) &= 0xBF;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
#endif
}

/**
 * @brief  设置CE引脚为高电平（启动发射/接收）
 */
void XL2400_CE_High(void)
{
#ifdef USE_XL2400P
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0xEF);
#else
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    *(cbuf + 1) |= 0x40;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
#endif
}

/**
 * @brief  测试SPI通信是否正常
 * @retval SUCCESS 成功，ERROR 失败
 */
ErrorStatus XL2400_SPI_Test(void)
{
    uint8_t i;
    const uint8_t *ptr = (const uint8_t *)XL2400_TEST_ADDR;

#ifdef USE_XL2400P
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x02);
    HAL_Delay(2);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x3E);
    HAL_Delay(2);
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, xbuf, 6);
    xbuf[5] = (xbuf[5] | 0x6d);
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, xbuf, 6);
#endif

    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_TX_ADDR, ptr, 5);
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_TX_ADDR, xbuf, 5);

    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
        if (*(xbuf + i) != *ptr++) return ERROR;
    }
    return SUCCESS;
}

/**
 * @brief  初始化XL2400模块
 */
/**
 * @brief  初始化XL2400模块
 *
 * 该函数根据不同的芯片型号（XL2400P 或 XL2400）对无线模块进行初始化配置。
 * 包括电源管理、模拟寄存器配置、自动应答设置、地址宽度设定、重传次数、射频速率等。
 * 最终将模块设置为可发送和接收数据的状态。
 */
void XL2400_Init(void)
{
#ifdef USE_XL2400P
    // Reset EN_PM, POWER_UP
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x02);
    HAL_Delay(2);
    // Set EN_PM, POWER_UP
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x3E);
    HAL_Delay(2);
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, xbuf, 6);
    xbuf[5] = ((xbuf[5]&0xff) | 0x6d);
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, xbuf, 6);
#else
    // Analog config
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    *(xbuf + 4) &= ~0x04;
    *(xbuf + 12) |= 0x40;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    // Switch to software CE control, wake up RF
    *(xbuf + 0) = 0x7E;
    *(xbuf + 1) = 0x82;
    *(xbuf + 2) = 0x0B;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
    XL2400_CE_Low();
    XL2400_ClearStatus();
#endif

    // Enable Auto ACK on all pipes
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_AA, 0x3F);
    // Enable all pipes
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_RXADDR, 0x3F);
    // Address Width, 5 bytes
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_SETUP_AW, 0xAF);
    // Retries and interval
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_SETUP_RETR, 0x33);
    // RF Data Rate: 08:2Mbps, 00:1Mbps, 20:250Kbps, 28:125Kbps
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_RF_SETUP, XL2400_RF_DR_250K);
    // Number of bytes in RX payload, pipe 0 and pipe 1
    *(cbuf + 0) = XL2400_PLOAD_WIDTH;
    *(cbuf + 1) = XL2400_PLOAD_WIDTH;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_PW_PX, cbuf, 2);
    // Dynamic payload width: off
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_DYNPD, 0x00);
    // Other features
    //bit7&6=00 return status when send register address
    //bit5=0 long data pack off
    //bit4=1 FEC off
    //bit3=1 FEATURE on
    //bit2=0 Dynamic length off
    //bit1=0 ACK without payload
    //bit0=0 W_TX_PAYLOAD_NOACK off
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_FEATURE, 0x18);

		// Enable RSSI
    *(cbuf + 0) = 0x10;
    *(cbuf + 1) = 0x00;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RSSI, cbuf, 2);
		

	XL2400_Reset();
    XL2400_ClearStatus();
}


/**
 * @brief  设置射频频道
 * @param  channel: 频道号（最大80）
 */
void XL2400_SetChannel(uint8_t channel)
{
    if (channel > 80) channel = 80;
#ifdef USE_XL2400P
    *cbuf = XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_EN_AA);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_AA, *cbuf & ~0x40);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, 0x60 + channel);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_EN_AA, *cbuf | 0x40);
#else
    // AFC reset
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, 0x06);
    // AFC on
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, 0x0E);
    // 设置频率
    *(cbuf + 0) = 0x60 + channel;
    *(cbuf + 1) = 0x09;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, cbuf, 2);
    // AFC锁定
    *(cbuf + 1) |= 0x20;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, cbuf, 2);
#endif
}

/**
 * @brief  设置发送地址
 * @param  address: 地址数组（5字节）
 */
void XL2400_SetTxAddress(const uint8_t *address)
{
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_TX_ADDR, address, 5);
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_ADDR_P0, address, 5);
}

/**
 * @brief  设置接收地址（管道1）
 * @param  address: 地址数组（5字节）
 */
void XL2400_SetRxAddress(const uint8_t *address)
{
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RX_ADDR_P1, address, 5);
}

/**
 * @brief  设置发射功率
 * @param  power: 功率配置值
 */
void XL2400_SetPower(uint8_t power)
{
#ifdef USE_XL2400P
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
	*(xbuf + 1) = power;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
#else
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
    *(xbuf + 2) = power;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
#endif
}

/**
 * @brief  将模块设置为休眠模式，降低功耗
 * @note   XL2400P 特有功能
 */
void XL2400_Sleep(void)
{
#ifdef USE_XL2400P
    // 写入配置寄存器，关闭发射/接收功能
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x00);
#else
    // XL240：CE 引脚拉低并清空状态
    XL2400_CE_Low();
    XL2400_ClearStatus();

    *(xbuf + 0) = 0x7C;
    *(xbuf + 1) = 0x82;
    *(xbuf + 2) = 0x03;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
#endif
}


/**
 * @brief  唤醒模块（仅用于 XL2400）
 * @note   XL2400P 使用 Sleep/WakeUp 组合唤醒
 */
void XL2400_WakeUp(void)
{
#ifdef USE_XL2400
    // 配置进入待机模式
    *(xbuf + 0) = 0x7E;
    *(xbuf + 1) = 0x82;
    *(xbuf + 2) = 0x0B;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
    XL2400_CE_Low();          // CE 引脚拉低
    XL2400_ClearStatus();      // 清除状态标志
#endif
}
/**
 * @brief  复位模块（仅用于 XL2400P）
 * @note   XL2400 使用 Sleep + WakeUp 实现复位
 */
void XL2400_Reset(void)
{
#ifdef USE_XL2400P
    // 发送复位命令
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0xEA);
    HAL_Delay(0);  // 短暂延时
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0xEE);
    HAL_Delay(1);  // 等待复位完成
#endif
}

/**
 * @brief  对接收通道进行校准，确保接收灵敏度
 * @retval SUCCESS 校准成功；ERROR 校准失败
 */
ErrorStatus XL2400_RxCalibrate(void)
{
    uint8_t i, j;

    for (i = 0; i < 10; i++) {
        HAL_Delay(2);
        // 读取模拟配置寄存器
        XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);

        // 修改配置字节以启用校准
        *(cbuf + 1) |= 0x90;
        *(cbuf + 1) &= ~0x20;
        XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);

        *(cbuf + 1) |= 0x40;
        XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);

        HAL_Delay(1);

        // 检查 FIFO 是否准备好
        XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_FIFO_STATUS, cbuf, 2);

        if (*(cbuf + 1) & 0x20) {
            j = *(cbuf + 1) << 3;
            XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
            *(cbuf + 1) &= 0x8F;
            *(cbuf + 1) |= 0x20;
            *(cbuf + 0) &= 0x07;
            *(cbuf + 0) |= j;
            XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG3, cbuf, 2);
            return SUCCESS;
        }
    }

    return ERROR;
}

/**
 * @brief  设置模块为发送模式
 */
void XL2400_SetTxMode(void)
{
#ifdef USE_XL2400P
    // 设置 TX 模式
    cbuf[0] = 0xee;
    cbuf[1] = 0x80;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    XL2400_ClearStatus();
#else
    // XL2400 的初始化流程
    XL2400_CE_Low();
    XL2400_ClearStatus();
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x7E);
    XL2400_RxCalibrate();  // 校准接收电路
#endif
    HAL_Delay(1);
}

/**
 * @brief  设置模块为接收模式
 */
void XL2400_SetRxMode(void)
{
#ifdef USE_XL2400P
    // 设置 RX 模式
    cbuf[0] = 0xee;
    cbuf[1] = 0xc0;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, cbuf, 2);
    XL2400_ClearStatus();
#else
    // XL2400 初始化流程
    XL2400_CE_Low();
    XL2400_ClearStatus();
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_CFG_TOP, 0x7F);
    // XL2400_RxCalibrate();  // 可选校准
#endif

		
    XL2400_CE_High();  // 启动接收
    HAL_Delay(1);
}

/**
 * @brief  快速发送数据（阻塞等待直到 FIFO 可写）
 * @param  ucPayload: 数据指针
 * @param  length: 数据长度
 * @retval SUCCESS 成功发送；ERROR 发送失败或达到最大重试次数
 */
ErrorStatus XL2400_TxFast(const uint8_t *ucPayload, uint8_t length)
{
    // 如果 FIFO 已满，则阻塞等待
    while ((XL2400_ReadStatus() & XL2400_FLAG_TX_FULL)) {
        if (xl2400_state & XL2400_FLAG_MAX_RT) {
            return ERROR;
        }
    }
    // 写入数据并启动发送
    XL2400_WriteFromBuf(XL2400_CMD_W_TX_PAYLOAD, ucPayload, length);
    XL2400_CE_High();  // 触发发送
    return SUCCESS;
}

/**
 * @brief  重复使用上一次发送的数据包重新发送
 */
void XL2400_ReuseTX(void)
{
    // 清除最大重试标志
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, XL2400_FLAG_MAX_RT);
    XL2400_CE_Low();
    XL2400_CE_High();  // 重新触发发送
}

/**
 * @brief  发送指定长度的数据包
 * @param  ucPayload: 数据指针
 * @param  length: 数据长度
 * @retval 返回状态字节（包含发送结果）
 */
uint8_t XL2400_Tx(const uint8_t *ucPayload, uint8_t length)
{
    uint8_t y = 16, status = 0;
    XL2400_ClearStatus();  // 清除状态
    XL2400_WriteFromBuf(XL2400_CMD_W_TX_PAYLOAD, ucPayload, length);  // 写入数据
    XL2400_CE_High();     // 启动发送
    printf(".");           // 打印进度点

    // 超时重试机制
    while (y--) {
        status = XL2400_ReadStatus();  // 读取状态
        if ((status & (XL2400_FLAG_MAX_RT | XL2400_FLAG_TX_DS)) != 0) {
            break;  // 发送成功或超限退出
        }
        HAL_Delay(0);  // 等待 1ms
    }
    return status;
}

/**
 * @brief  接收一个数据包
 * @retval 返回状态字节（包含接收是否就绪）
 */
uint8_t XL2400_Rx(void)
{
    uint8_t status = XL2400_ReadStatus();
    // 直接读取RSSI值
    int8_t rssi = XL2400_ReadRSSI();
    if (status & XL2400_FLAG_RX_DR) {
        
        printf("RSSI: %d dBm\n", rssi);
        
        // 清除标志
        XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, status);
        
        // 读取数据
        uint8_t rxplWidth = XL2400_ReadReg(XL2400_CMD_R_RX_PL_WID);
        XL2400_ReadToBuf(XL2400_CMD_R_RX_PAYLOAD, xbuf, rxplWidth);
		printf("\r\n[Status]");
    printf("%02X", status);
    printf("\r\n\r\n");
    }

    return status;
}

/**
 * @brief  读取模块当前状态
 * @retval 当前状态寄存器值
 */
uint8_t XL2400_ReadStatus(void)
{
    xl2400_state = XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_STATUS);
    return xl2400_state;
}

/**
 * @brief  清除所有状态标志
 */
void XL2400_ClearStatus(void)
{
    XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);  // 清空 TX FIFO
    XL2400_WriteReg(XL2400_CMD_FLUSH_RX, XL2400_CMD_NOP);  // 清空 RX FIFO
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_STATUS, 0x70);  // 清除中断标志
}

/**
 * @brief  清空 TX/RX FIFO 缓冲区
 */
void XL2400_FlushRxTX(void)
{
    XL2400_WriteReg(XL2400_CMD_FLUSH_TX, XL2400_CMD_NOP);
    XL2400_WriteReg(XL2400_CMD_FLUSH_RX, XL2400_CMD_NOP);
}

/**
 * @brief  启动载波检测测试（用于调试 RF 模块）
 */
void XL2400_CarrierTest(void)
{
    XL2400_CE_Low();
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    *(xbuf + 12) |= 0x40;
    *(xbuf + 4) &= ~0x04;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_ANALOG_CFG0, xbuf, 13);
    XL2400_WriteReg(XL2400_CMD_W_REGISTER | XL2400_REG_TXPROC_CFG, 0x00);
    *(xbuf + 0) = 0x01;
    *(xbuf + 1) = 0x00;
    XL2400_WriteFromBuf(XL2400_CMD_W_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
    XL2400_ClearStatus();
}
/**
 * @brief 读取 RSSI（dBm）计算结果
 * @retval RSSI（dBm）值
 */
int8_t XL2400_ReadRSSI(void)
{
    // 直接读取0x09寄存器的RSSI值
    uint8_t reg09[2];
	uint32_t rssi_reg;
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSI, reg09, 2);
    uint8_t rssi_buf[4];
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSIREC, rssi_buf, 4);
	// 提取RSSI_DBM (bits 29:22)
	 rssi_reg = ((uint32_t)rssi_buf[3] << 24) | 
               ((uint32_t)rssi_buf[2] << 16) |
               ((uint32_t)rssi_buf[1] << 8)  |
                (uint32_t)rssi_buf[0];
    int8_t rssi_dbm = (int8_t)((rssi_reg >> 22) & 0xFF);
    // RSSI值在reg09[0]的低6位
    int8_t rssi = (int8_t)(reg09[0] & 0x3F);
    
    // 调试信息
//	 // 调试信息
//    printf("RSSIREC: %02X %02X %02X %02X\n", 
//           rssi_buf[0], rssi_buf[1], rssi_buf[2], rssi_buf[3]);
//    printf("RSSI_DBM: %d\n", rssi_dbm);
    printf("REG09: %02X %02X (RSSI=%d)\n", reg09[0], reg09[1], rssi);
    return rssi;
}
/**
 * @brief 打印XL2400芯片的状态信息
 * 
 * 该函数通过读取XL2400芯片的各个寄存器来获取并打印芯片的当前状态信息，包括配置寄存器、使能自动重传、
 * 使能接收地址、地址宽度、重试设置、RF通道、RF设置、发送观察、RSSI值、发送地址、接收地址、接收负载宽度、
 * FIFO状态、动态负载宽度和特性寄存器等信息。最后打印当前的状态寄存器值，并返回该值。
 * 
 * @return uint8_t 当前状态寄存器的值
 */
uint8_t XL2400_PrintStatus(void)
{
    uint8_t i, status;

    // 打印配置寄存器的值
    printf("Bytes from low to high: 0,1,2,3,...\r\n[Config]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_CFG_TOP, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印使能自动重传寄存器的值
    printf("  [EN_AA]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_EN_AA));

    // 打印使能接收地址寄存器的值
    printf("  [EN_RxAddr]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_EN_RXADDR));

    // 打印地址宽度寄存器的值
    printf("  [AddrWidth]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_SETUP_AW));

    // 打印重试设置寄存器的值
    printf("  [Retry]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_SETUP_RETR, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印RF通道寄存器的值
    printf("\r\n[RF_Channel]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_CH, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印RF设置寄存器的值
    printf("  [RF_Setup]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RF_SETUP, xbuf, 2);
    for (i = 0; i < 2; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印发送观察寄存器的值
    printf("  [Observe_Tx]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_OBSERVE_TX, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }
		printf("  [RSSI]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSI, xbuf, 2);
    for (i = 0; i < 2; i++) {
        printf("%02X", *(xbuf + i));
    }
		
//		uint8_t reg09[2], reg18[4];
//    // 打印RSSI值寄存器的值
//    printf("  [RSSI]");
//    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSI, reg09, 2);
//     printf("REG09: %02X %02X (EN_RSSI=%d)\n", 
//           reg09[0], reg09[1], (reg09[0] >> 4) & 1);

//    // 打印RSSI_DBM寄存器的值
//    printf("\r\n[RSSI_DBM]  ");
////    printf("%x", XL2400_ReadRSSI());
//    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RSSIREC, reg18, 4);
//    printf("REG18: %02X %02X %02X %02X\n", 
//           reg18[0], reg18[1], reg18[2], reg18[3]);

    // 打印发送地址寄存器的值
    printf("\r\n[TxAddr]  ");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印接收地址寄存器的值
    printf("\r\n[RxAddrP0]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P0, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [RxAddrP1]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P1, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [RxAddrP2-P5]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_ADDR_P2_P5, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印接收负载宽度寄存器的值
    printf("\r\n[RxPloadWidth_P0-P5]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_RX_PW_PX, xbuf, 6);
    for (i = 0; i < 6; i++) {
        printf("%02X", *(xbuf + i));
    }

    // 打印FIFO状态寄存器的值
    printf("\r\n[FIFO_Status]");
    XL2400_ReadToBuf(XL2400_CMD_R_REGISTER | XL2400_REG_FIFO_STATUS, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf("  [DynPloadWidth]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_DYNPD));
    printf("  [Feature]");
    printf("%02X", XL2400_ReadReg(XL2400_CMD_R_REGISTER | XL2400_REG_FEATURE));

    // 打印当前状态寄存器的值
    status = XL2400_ReadStatus();
    printf("\r\n[Status]");
    printf("%02X", status);
    printf("\r\n\r\n");
    return status;
}

