// xn297L.h
#ifndef __XN297L_H__
#define __XN297L_H__

#include "main.h"
#include "string.h"
#include "userConfig.h"
#include "bsp_core.h"

/**
 * 默认数据载荷宽度
 * 可根据实际需求调整为最大支持的 64 字节
 */
#define XN297L_PLOAD_WIDTH       32   // Payload width

/**
 * 片选引脚控制宏
 * NSS 用于 SPI 通信中选择芯片
 */
#define XN297L_CSN_LOW()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define XN297L_CSN_HIGH()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define XN297L_CE_LOW()          XN297L_WriteReg(XN297L_CMD_CE_FSPI_OFF, 0)
#define XN297L_CE_HIGH()         XN297L_WriteReg(XN297L_CMD_CE_FSPI_ON, 0)
/**
 * ======================================
 * ==      SPI 命令定义                ==
 * ======================================
 */
/****************** SPI 操作命令 **********************/
#define XN297L_CMD_R_REGISTER    0x00 // [000A AAAA] 寄存器读操作
#define XN297L_CMD_W_REGISTER    0x20 // [001A AAAA] 寄存器写操作
#define XN297L_CMD_R_RX_PAYLOAD  0x61 // 读取接收缓冲区中的有效数据
#define XN297L_CMD_W_TX_PAYLOAD  0xA0 // 写入发送缓冲区
#define XN297L_CMD_FLUSH_TX      0xE1 // 清空 TX FIFO 缓冲区
#define XN297L_CMD_FLUSH_RX      0xE2 // 清空 RX FIFO 缓冲区
#define XN297L_CMD_REUSE_TX_PL   0xE3 // 重用上一次发送的数据
#define XN297L_CMD_ACTIVATE      0x50 // 激活特定功能
#define XN297L_CMD_DEACTIVATE    0x50 // 停用特定功能
#define XN297L_CMD_RST_FSPI      0x53 // 复位 SPI 接口
#define XN297L_CMD_R_RX_PL_WID   0x60 // 读取当前接收数据长度
#define XN297L_CMD_W_ACK_PAYLOAD 0xA8 // 发送带确认的数据
#define XN297L_CMD_W_TX_PAYLOAD_NOACK 0xB0 // 发送不请求确认的数据
#define XN297L_CMD_CE_FSPI_ON    0xFD // SPI 命令使 CE 内部逻辑置 1，用该命令后跟数据 0x00。
#define XN297L_CMD_CE_FSPI_OFF   0xFC // SPI 命令使 CE 内部逻辑置 0，用该命令后跟数据 0x00。
#define XN297L_CMD_NOP           0xFF // 空操作（用于读状态寄存器）

/**
 * ======================================
 * ==      控制寄存器地址定义          ==
 * ======================================
 */
/****************** 控制寄存器地址 ******************/
#define XN297L_REG_CONFIG       0x00 // 配置寄存器（20 bits）
#define XN297L_REG_EN_AA         0x01 // 自动应答使能
#define XN297L_REG_EN_RXADDR     0x02 // 接收地址使能
#define XN297L_REG_SETUP_AW      0x03 // 地址宽度设置
#define XN297L_REG_SETUP_RETR    0x04 // 自动重传设置（30 bits）
#define XN297L_REG_RF_CH         0x05 // 射频通道设置（22 bits）
#define XN297L_REG_RF_SETUP      0x06 // RF 设置（16 bits）
#define XN297L_REG_STATUS        0x07 // 状态寄存器
#define XN297L_REG_OBSERVE_TX    0x08 // 发送观察寄存器（32 bits）
#define XN297L_REG_RSSI          0x09 // RSSI（信号强度）值（14 bits）
#define XN297L_REG_RX_ADDR_P0    0x0A // 接收地址管道 0（40 bits）
#define XN297L_REG_RX_ADDR_P1    0x0B // 接收地址管道 1（40 bits）
#define XN297L_REG_RX_ADDR_P2    0x0C // 接收地址管道 1（40 bits）
#define XN297L_REG_RX_ADDR_P3    0x0D // 接收地址管道 1（40 bits）
#define XN297L_REG_RX_ADDR_P4   0x0E  // 接收地址管道 1（40 bits）
#define XN297L_REG_RX_ADDR_P5   0x0F  // 接收地址管道 1（40 bits）

#define XN297L_REG_TX_ADDR       0x10 // 发送地址（40 bits）
#define XN297L_REG_RX_PW_P0     0x11 //  data pipe 0 中的RX payload 的数据长度
#define XN297L_REG_RX_PW_P1     0x12 //  data pipe 1 中的RX payload 的数据长度
#define XN297L_REG_RX_PW_P2     0x13 //  data pipe 2 中的RX payload 的数据长度
#define XN297L_REG_RX_PW_P3     0x14 //  data pipe 3 中的RX payload 的数据长度
#define XN297L_REG_RX_PW_P4     0x15 //  data pipe 4 中的RX payload 的数据长度
#define XN297L_REG_RX_PW_P5     0x16 //  data pipe 5 中的RX payload 的数据长度

#define XN297L_REG_FIFO_STATUS   0x17 // FIFO状态（20 bits）
#define XN297L_REG_DEM_CAL      0x19 // 8bit
#define XN297L_REG_RF_CAL2      0x1A // 48bit
#define XN297L_REG_DEM_CAL2     0x1B // 24bit
#define XN297L_REG_DYNPD        0x1C // Enable dynamic payload length
#define XN297L_REG_FEATURE      0x1D // Feature config
#define XN297L_REG_RF_CAL       0x1E // 24bit
#define XN297L_REG_BB_CAL       0x1F // 40bit

/**
 * ======================================
 * ==      寄存器配置位定义说明          ==
 * ======================================
 */
/**
 * CONFIG 寄存器（地址：0x00）
 * 00 CONFIG BIT[7:0]
*/
#define XN297L_CONFIG_EN_PM         0x80  // STB3模式(当PWR_UP=1时)，切换前需等待50us以上
#define XN297L_CONFIG_MASK_RX_DR    0x40  // 屏蔽IRQ上的RX_DR中断
#define XN297L_CONFIG_MASK_TX_DS    0x20  // 屏蔽IRQ上的TX_DS中断
#define XN297L_CONFIG_MASK_MAX_RT   0x10  // 屏蔽IRQ上的MAX_RT中断
#define XN297L_CONFIG_EN_CRC        0x08  // 启用CRC (需额外2字节)
#define XN297L_CONFIG_PWR_UP        0x02  // 芯片上电
#define XN297L_CONFIG_PRIM_RX       0x01  // RX模式
/**
 * EN_AA 自动应答通道使能（地址：0x01）
 * 01 EN_AA BIT[5:0]
 *   每个比特代表一个通道
 * 
 * EN_RXADDR 接收地址通道使能（地址：0x02）
 * 02 EN_RXADDR BIT[5:0]
 *   每个比特代表一个通道
 *
 * SETUP_AW 地址宽度设定（地址：0x03）
 * 03 SETUP_AW BIT[1:0]
*/
#define XN297L_SETUP_AW_3BYTE    0x01  // 地址长度3字节
#define XN297L_SETUP_AW_4BYTE    0x10  // 地址长度4字节
#define XN297L_SETUP_AW_5BYTE    0x11  // 地址长度5字节
/**
 * SETUP_RETR 自动重传设定（地址：0x04）
 * 04 SETUP_RETR BIT[7:0]
 *   ARD BIT[7:4] 0x00:250us, 0x10:500us, ... 0xF0:4000us
 *   ARC BIT[3:0] 0x00:无重试, 0x01:1次重试, 0x0F:15次重试
 * 
 * 射频通道设置（地址：0x05）
 * 05 RF_CH BIT[6:0]
 *   信道号: 0x00 ~ 0x7F, 频率(MHz) = 2400 + channel
 * 
 * 射频参数设置（地址：0x06）
 * 06 RF_SETUP BIT[5:0]
*/
/***************************** 功率等级配置 ************************************/
#define XN297L_RF_POWER_P_11     0x27  // 100 111: 11dbm
#define XN297L_RF_POWER_P_10     0x26  // 100 110: 10dbm
#define XN297L_RF_POWER_P_9      0x15  // 010 101: 9dbm
#define XN297L_RF_POWER_P_7      0x0D  // 001 101: 7dbm, 不兼容250Kbps。推荐用于安规测试(在1Mbps下使用)。
#define XN297L_RF_POWER_P_6      0x06  // 000 110: 6dbm, 不兼容250Kbps。推荐用于安规测试(在1Mbps下使用)。
#define XN297L_RF_POWER_P_5      0x2C  // 101 100: 5dbm
#define XN297L_RF_POWER_P_5L     0x05  // 000 101: 5dbm, 不兼容250Kbps。推荐用于安规测试(在1Mbps下使用)。
#define XN297L_RF_POWER_P_4      0x14  // 010 100: 4dbm
#define XN297L_RF_POWER_P_3      0x0C  // 001 100: 3dbm, 不兼容250Kbps。推荐用于安规测试(在1Mbps下使用)。
#define XN297L_RF_POWER_N_1      0x2A  // 101 010: -1dbm
#define XN297L_RF_POWER_N_9      0x29  // 101 001: -9dbm
#define XN297L_RF_POWER_N_10     0x19  // 011 001: -10dbm
#define XN297L_RF_POWER_N_23     0x30  // 110 000: -23dbm
/**
 * 06 RF_SETUP BIT[7:6]
*/
/***************************** 数据速率配置 ************************************/
#define XN297L_RF_DR_2M          0x40  // 2Mbps
#define XN297L_RF_DR_1M          0x00  // 1Mbps
#define XN297L_RF_DR_250K        0xC0  // 250Kbps (需要配合XN297L_RF_POWER_P_9使用)
/**
 * STATUS 状态寄存器（地址：0x07）
 * 07 STATUS BIT[6:0]
*/
#define XN297L_FLAG_RX_DR        0x40  // RX FIFO 数据就绪
#define XN297L_FLAG_TX_DS        0x20  // TX FIFO 数据已发送完成
#define XN297L_FLAG_MAX_RT       0x10  // 达到最大重试次数，发送失败
#define XN297L_FLAG_RX_P_NO      0x0E  // RX FIFO 数据就绪管道掩码
#define XN297L_FLAG_TX_FULL      0x01  // TX FIFO 已满
/**
 * OBSERVE_TX 传输状态寄存器（地址：0x08）
 * 08 OBSERVE_TX
 *   PLOS_CNT BIT[7:4]: 包丢失计数器，最大值为15
 *   ARC_CNT BIT[3:0]: 重试计数器，达到SETUP_RETR.ARC后，PLOS_CNT+1。写入TX FIFO会复位此值
 * 
 * DATAOUT 数据读取寄存器 （地址：0x09）      （前提 DATAOUT_SEL=0） 
 * 09 DATAOUT BIT[7:0]
 *   BIT[7:4]: 实时RSSI
 *   BIT[3:0]: 接收到包的RSSI
 * 
 * 0A RX_ADDR_P0 BIT[39:0]      RX payload的数据长度
 * 0B RX_ADDR_P1 BIT[39:0]      RX payload的接收地址
 * 0C RX_ADDR_P2 BIT[7:0]       RX payload的接收地址
 * 0D RX_ADDR_P3 BIT[7:0]       RX payload的接收地址
 * 0E RX_ADDR_P4 BIT[7:0]       RX payload的接收地址
 * 0F RX_ADDR_P5 BIT[7:0]       RX payload的接收地址
 * 
 * TX_ADDR发送端地址（由低字节开始写） （地址：0x10）
 * 只能在配置为 PTX 模式的芯片中使用，需要设置RX_ADDR_P0 等于该地址以便接收 ACK 自动应答。
 * 10 TX_ADDR    BIT[39:0]
 * 
 * 11 RX_PW_P0   BIT[6:0]       RX pipe0负载长度
 * 12 RX_PW_P1   BIT[6:0]       RX pipe1负载长度
 * 13 RX_PW_P2   BIT[6:0]       RX pipe2负载长度
 * 14 RX_PW_P3   BIT[6:0]       RX pipe3负载长度
 * 15 RX_PW_P4   BIT[6:0]       RX pipe4负载长度
 * 16 RX_PW_P5   BIT[6:0]       RX pipe5负载长度
 * 
 * FIFO 状态寄存器 （地址：0x17）
 * 17 FIFO_STATUS BIT[6:0]
*/
#define XN297L_FIFO_STATUS_TX_REUSE     0x40
#define XN297L_FIFO_STATUS_TX_FULL     0x20
#define XN297L_FIFO_STATUS_TX_EMPTY   0x10
#define XN297L_FIFO_STATUS_RX_FULL    0x02
#define XN297L_FIFO_STATUS_RX_EMPTY   0x01

/**
 * DEMOD_CAL 调制解调参数寄存器  (地址：0x19）
 * 19 DEMOD_CAL BIT[7:0]        * 0x01 *
 *   CHIP BIT[7]:               0, Debug模式, 1:ON, 0:OFF
 *   CARR BIT[6:5]:             00, 载波测试模式, 11:ON且CHIP=1, 00:OFF
 *   GAUS_CAL BIT[4:1]:         0111, DAC信号高斯滤波, 1111:低, 0000:高
 *   SCRAMBLE_EN BIT[0]:        1, 扰码启用, 1:ON, 0:OFF
 * 
 * RF_CAL2 补充射频寄存器 （地址：0x1A）
 * 1A RF_CAL2 BIT[45:0]         * 0x45, 0x21, 0xEF, 0x2C, 0x5A, 0x40 *
 *   BW_500K BIT[45]:           滤波器带宽, 0:窄, 1:宽
 *   GC_500K BIT[44]:           滤波器增益控制, 0:低, 1:高
 * DEM_CAL2 补充解调参数寄存器 （地址：0x1B）
 * 1B DEM_CAL2 BIT[23:0]        * 0x0B, 0xDF, 0x02 *
 *   
 * DYNPD 动态 PAYLOAD 长度使能  (地址：0x1C）)
 * 1C DYNPD BIT[5:0]
 *   启用动态负载长度，每个比特对应一个通道
 * 
 * FEATURE 特征寄存器 （地址：0x1D）
 * 1D FEATURE BIT[6:0]
*/
#define XN297L_FEATURE_BIT6_MUX_PA          0x40
#define XN297L_FEATURE_BIT6_MUX_IRQ         0x00
#define XN297L_FEATURE_BIT5_CE_SOFT         0x20
#define XN297L_FEATURE_BIT5_CE_HARD         0x00
#define XN297L_FEATURE_BIT43_DATA_64BYTE    0x18
#define XN297L_FEATURE_BIT43_DATA_32BYTE    0x00
#define XN297L_FEATURE_BIT2_EN_DPL_ON       0x04
#define XN297L_FEATURE_BIT2_EN_DPL_OFF      0x00
#define XN297L_FEATURE_BIT1_EN_ACK_PAY_ON   0x02
#define XN297L_FEATURE_BIT1_EN_ACK_PAY_OFF  0x00
#define XN297L_FEATURE_BIT0_EN_NOACK_ON     0x01
#define XN297L_FEATURE_BIT0_EN_NOACK_OFF    0x00
/**
 * RF_CAL 射频参数寄存器 （地址：0x1E）
 * 1E RF_CAL BIT[23:0]          * 0xF6(0x06 安规测试), 0x3F, 0x5D *
 *   EN_CLK_OUT BIT[23]:        OSC输出到CLK_OUT, 1:ON, 0:OFF
 *   DA_VREF_MB BIT[22:20]:     DAC vref+, 111:高, 000:低
 *   DA_VREF_LB BIT[19:17]:     DAC vref-, 111:低, 000:高
 *   DA_LPF_CTRL BIT[16]:       DAC输出放大倍数, 1:0.8, 0:0.5
 *   RSSI_EN BIT[15]:           RSSI启用, 1:ON, 0:OFF
 *   RSSI_Gain_CTR BIT[14:13]:  RSSI增益控制, 00:None, 01:-6dB, 10:-12dB, 11:-18dB
 *   MIXL_GC BIT[12]:           RX混频器增益控制, 1:14dB, 0:8dB
 *   PA_BC BIT[11:10]:          功放电流, 00: ×1, 01: ×2, 10: ×3, 11: ×4
 *   LNA_GC BIT[9:8]:           LNA增益控制, 11:17dB, 10:11dB, 01:5.4dB, 00:-0.4dB
 *   VCO_BIAS BIT[7:5]:         VCO电流, 000:900uA, 001:1050uA, 010:1200uA, 011:1350uA, 100:1500uA, 101:1650uA, 110:1800uA, 111:1950uA
 *   RES_SEL BIT[4:3]:          偏置电流电阻, 00:26kR, 01:24kR, 10:22kR, 11:20kR
 *   LNA_HCURR BIT[2]:          LNA高电流启用, 1:ON, 0:OFF
 *   MIXL_BC BIT[1]:            RX混频器电流, 1:×1, 0:×0.5
 *   IB_BPF_TRIM BIT[0]:        RX带通滤波器电流, 1:×1, 0:×0.5
 * 
 * BB_CAL 射频参数寄存器 （地址：0x1F）
 * 1F BB_CAL BIT[39:0]          * 0x12, 0xED, 0x67, 0x9C, 0x46 *
 *   Reserved BIT[39:32]:       
 *   INVERTER BIT[31]:          是否反转进入RX模块的数据, 1:反转, 0:保持不变
 *   DAC_MODE BIT[30]:          DAC输入格式, 1:dac_out[5:0]<= [0:5], 0: dac_out[5:0]<= [5:0]
 *   DAC_BASAL BIT[29:24]:      DAC初始偏移量
 *   TRX_TIME BIT[23:21]:       发射载波到发包时间 = TRX_TIME*8 + 7.5
 *   EX_PA_TIME BIT[20:16]      TX PLL开启到PA开启的时间 = EX_PA_TIME*16, 单位us
 *   TX_SETUP_TIME BIT[15:11]:  PA开启到TX PLL打开的时间 = TX_SETUP_TIME*16, 单位us
 *   RX_SETUP_TIME BIT[10:6]:   RX PLL开启到RX使能的时间 = RX_SETUP_TIME*16, 单位us
 *   RX_ACK_TIME BIT[5:0]:      进入RX模式后等ACK时间 = RX_ACK_TIME*16(2Mbps), RX_ACK_TIME*32(1Mbps), RX_ACK_TIME*128(250Kbps)
 */

#define XN297L_TEST_ADDR         "XN297"
/**
 * ======================================
 * ==      函数声明                    ==
 * ======================================
 */
 /**************************** 寄存器操作 *****************************/
uint8_t XN297L_WriteReg(uint8_t reg, uint8_t value);
uint8_t XN297L_ReadReg(uint8_t reg);

void XN297L_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len);
void XN297L_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);

/**************************** SPI 测试 *****************************/
ErrorStatus XN297L_SPI_Test(void);
/**************************** 初始化与配置 **************************/
void XN297L_Init(void);
void XN297L_SetChannel(uint8_t channel);

void XN297L_SetTxAddress(const uint8_t *address);
void XN297L_SetRxAddress(const uint8_t *address);

void XN297L_SetPower(uint8_t power);
void XN297L_EnableRSSI(void);
uint8_t XN297L_GetRSSI(void);
/**************************** 电源管理 ******************************/
void XN297L_Sleep(void);
void XN297L_Reset(void);
/**************************** 模式设置与校准测试 *********************/
void XN297L_SetTxMode(void);
void XN297L_SetRxMode(void);

void XN297L_Carrier(uint8_t ucChannel_Set);
/**************************** 状态管理 ******************************/
uint8_t XN297L_ReadStatus(void);
void XN297L_ClearStatus(void);
void XL2400_FlushRxTX(void);
uint8_t XN297L_ClearFIFO(void);

/**************************** 收发操作 ******************************/
uint8_t XN297L_TxData(uint8_t *ucPayload, uint8_t length);
uint8_t XN297L_DumpRxData(void);

ErrorStatus XN297L_TxFast(const uint8_t *ucPayload, uint8_t length);
void XN297L_ReuseTX(void);


uint8_t XN297L_PrintStatus(void);

#endif /* __XN297L_H__ */
