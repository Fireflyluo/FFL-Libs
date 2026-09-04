#ifndef __FW_XL2400_H__
#define __FW_XL2400_H__

#include "main.h"
#include "string.h"

/**
 * 芯片型号选择：
 * - USE_XL2400：使用 XL2400 芯片
 * - USE_XL2400P：使用 XL2400P 芯片（推荐）
 */
#define USE_XL2400P

/**
 * 默认数据载荷宽度
 * 可根据实际需求调整为最大支持的 64 字节
 */
#define XL2400_PLOAD_WIDTH       32   // Payload width

/**
 * 片选引脚控制宏
 * NSS 用于 SPI 通信中选择芯片
 */
#define XL2400_NSS_LOW()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define XL2400_NSS_HIGH()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

/**
 * ======================================
 * ==      SPI 命令定义                ==
 * ======================================
 */
/****************** SPI 操作命令 **********************/
#define XL2400_CMD_R_REGISTER    0x00 // [000A AAAA] 寄存器读操作
#define XL2400_CMD_W_REGISTER    0x20 // [001A AAAA] 寄存器写操作
#define XL2400_CMD_R_RX_PAYLOAD  0x61 // 读取接收缓冲区中的有效数据
#define XL2400_CMD_W_TX_PAYLOAD  0xA0 // 写入发送缓冲区
#define XL2400_CMD_FLUSH_TX      0xE1 // 清空 TX FIFO 缓冲区
#define XL2400_CMD_FLUSH_RX      0xE2 // 清空 RX FIFO 缓冲区
#define XL2400_CMD_REUSE_TX_PL   0xE3 // 重用上一次发送的数据
#define XL2400_CMD_ACTIVATE      0x50 // 激活特定功能
#define XL2400_CMD_DEACTIVATE    0x50 // 停用特定功能
#define XL2400_CMD_RST_FSPI      0x53 // 复位 SPI 接口
#define XL2400_CMD_R_RX_PL_WID   0x60 // 读取当前接收数据长度
#define XL2400_CMD_W_ACK_PAYLOAD 0xA8 // 发送带确认的数据
#define XL2400_CMD_W_TX_PAYLOAD_NOACK 0xB0 // 发送不请求确认的数据
#define XL2400_CMD_NOP           0xFF // 空操作（用于读状态寄存器）

/**
 * ======================================
 * ==      控制寄存器地址定义          ==
 * ======================================
 */
/****************** 控制寄存器地址 ******************/
#define XL2400_REG_CFG_TOP       0x00 // 配置寄存器（20 bits）
#define XL2400_REG_EN_AA         0x01 // 自动应答使能
#define XL2400_REG_EN_RXADDR     0x02 // 接收地址使能
#define XL2400_REG_SETUP_AW      0x03 // 地址宽度设置
#define XL2400_REG_SETUP_RETR    0x04 // 自动重传设置（30 bits）
#define XL2400_REG_RF_CH         0x05 // 射频通道设置（22 bits）
#define XL2400_REG_RF_SETUP      0x06 // RF 设置（16 bits）
#define XL2400_REG_STATUS        0x07 // 状态寄存器
#define XL2400_REG_OBSERVE_TX    0x08 // 发送观察寄存器（32 bits）
#define XL2400_REG_RSSI          0x09 // RSSI（信号强度）值（14 bits）
#define XL2400_REG_RX_ADDR_P0    0x0A // 接收地址管道 0（40 bits）
#define XL2400_REG_RX_ADDR_P1    0x0B // 接收地址管道 1（40 bits）
#define XL2400_REG_RX_ADDR_P2_P5 0x0C // 接收地址管道 2~5（32 bits）
#define XL2400_REG_BER_RESULT    0x0D // BER 测试结果（64 bits）
#define XL2400_REG_AGC_SETTING   0x0E // AGC 设置（32 bits）
#define XL2400_REG_PGA_SETTING   0x0F // PGA 设置（39 bits）
#define XL2400_REG_TX_ADDR       0x10 // 发送地址（40 bits）
#define XL2400_REG_RX_PW_PX      0x11 // 接收缓冲区大小（48 bits）
#define XL2400_REG_ANALOG_CFG0   0x12 // 模拟配置寄存器 0（128 bits）
#define XL2400_REG_ANALOG_CFG1   0x13 // 模拟配置寄存器 1（128 bits）
#define XL2400_REG_ANALOG_CFG2   0x14 // 模拟配置寄存器 2（128 bits）
#define XL2400_REG_ANALOG_CFG3   0x15 // 模拟配置寄存器 3（128 bits）
#define XL2400_REG_FIFO_STATUS   0x17 // FIFO 状态（20 bits）
#define XL2400_REG_RSSIREC       0x18 // RSSI 记录功能（32 bits）
#define XL2400_REG_TXPROC_CFG    0x19 // 发送处理配置（29 bits）
#define XL2400_REG_RXPROC_CFG    0x1A // 接收处理配置（40 bits）
#define XL2400_REG_DYNPD         0x1C // 动态负载长度使能
#define XL2400_REG_FEATURE       0x1D // 特性配置
#define XL2400_REG_RAMP_CFG      0x1E // PA 斜坡配置（88 bits）

/**
 * ======================================
 * ==      配置常量定义                ==
 * ======================================
 */

/***************************** 最大载荷宽度 ************************************/
#define XL2400_PL_WIDTH_MAX      64   // 最大数据载荷宽度

/***************************** 功率等级配置 ************************************/
#define XL2400_RF_10DB           0x3F
#define XL2400_RF_9DB            0x38
#define XL2400_RF_8DB            0x34
#define XL2400_RF_7DB            0x30
#define XL2400_RF_6DB            0x2C // 最高支持 250Kbps
#define XL2400_RF_5DB            0x28
#define XL2400_RF_4DB            0x24
#define XL2400_RF_3DB            0x20
#define XL2400_RF_2DB            0x14
#define XL2400_RF_0DB            0x10 // 最高支持 1Mbps
#define XL2400_RF__2DB           0x0C
#define XL2400_RF__6DB           0x08
#define XL2400_RF__12DB          0x04
#define XL2400_RF__18DB          0x02
#define XL2400_RF__24DB          0x01

/***************************** 数据速率配置 ************************************/
#define XL2400_RF_DR_2M          0x08 // 2Mbps
#define XL2400_RF_DR_1M          0x00 // 1Mbps
#define XL2400_RF_DR_250K        0x20 // 250Kbps
#define XL2400_RF_DR_125K        0x28 // 125Kbps

/***************************** 状态标志 ************************************/
#define XL2400_FLAG_RX_DR        0X40   // 接收数据就绪
#define XL2400_FLAG_TX_DS        0X20   // 数据已发送
#define XL2400_FLAG_RX_TX_CMP    0X60   // 数据发送并被确认
#define XL2400_FLAG_MAX_RT       0X10   // 达到最大重传次数
#define XL2400_FLAG_TX_FULL      0x01   // 发送缓冲区满

/***************************** 测试地址 ************************************/
#define XL2400_TEST_ADDR         "XL240"  // 默认测试地址

/**
 * ======================================
 * ==      函数声明                    ==
 * ======================================
 */

/**************************** 寄存器操作 ******************************/
uint8_t XL2400_WriteReg(uint8_t reg, uint8_t value);
uint8_t XL2400_ReadReg(uint8_t reg);

void XL2400_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len);
void XL2400_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);

/**************************** CE 引脚控制 ******************************/
void XL2400_CE_Low(void);
void XL2400_CE_High(void);

/**************************** SPI 测试 ******************************/
ErrorStatus XL2400_SPI_Test(void);

/**************************** 初始化与配置 ******************************/
void XL2400_Init(void);
void XL2400_SetChannel(uint8_t channel);

void XL2400_SetTxAddress(const uint8_t *address);
void XL2400_SetRxAddress(const uint8_t *address);
void XL2400_SetPower(uint8_t power);

/**************************** 电源管理 ******************************/
void XL2400_Sleep(void);
void XL2400_WakeUp(void);
void XL2400_Reset(void);

/**************************** 校准与模式设置 ******************************/
ErrorStatus XL2400_RxCalibrate(void);

void XL2400_SetTxMode(void);
void XL2400_SetRxMode(void);

/**************************** 收发操作 ******************************/
uint8_t XL2400_Tx(const uint8_t *ucPayload, uint8_t length);
ErrorStatus XL2400_TxFast(const uint8_t *ucPayload, uint8_t length);
void XL2400_ReuseTX(void);
uint8_t XL2400_Rx(void);

/**************************** 状态管理 ******************************/
int8_t XL2400_ReadRSSI(void);
uint8_t XL2400_ReadStatus(void);
void XL2400_ClearStatus(void);
void XL2400_FlushRxTX(void);
void XL2400_CarrierTest(void);
uint8_t XL2400_PrintStatus(void);

#endif /* __FW_XL2400_H__ */
