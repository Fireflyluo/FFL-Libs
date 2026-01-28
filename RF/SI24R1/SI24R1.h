#ifndef _SI24R1_H_
#define _SI24R1_H_

#include "py32f4xx_hal.h"
#include "main.h"
#include "userConfig.h"
#include "RF_Config.h"
#include "bsp_core.h"

typedef enum {TX_MODE, RX_MODE} SI24R1_MODE;

typedef enum SpeedType
{
        SPEED_250K = 0,
        SPEED_1M,
        SPEED_2M
}SI24R1SpeedType;

typedef enum PowerType
{
        POWER_F12DBM = 0,
        POWER_F6DBM,
        POWER_F4DBM,
        POWER_0DBM,
        POWER_1DBM,
        POWER_3DBM,
        POWER_4DBM,
        POWER_7DBM  //SI24R1最大支持7DB
}SI24R1PowerType;

/************************************************函数定义****************************************************************************/
//寄存器操作
uint8_t SI24R1_Read_Reg( uint8_t RegAddr );				//SI24R1读寄存器										//SI24R1读寄存器
void SI24R1_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );	//SI24R1读指定长度的数据
void SI24R1_Write_Reg( uint8_t RegAddr, uint8_t data );		        //SI24R1写寄存器
void SI24R1_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );	//SI24R1写多个字节

//射频操作
uint8_t SI24R1_ReadStatus(void);		//读取STATUS寄存器的值
uint8_t SI24R1_ReadIRQStatus(void);		//读取SI24R1+的中断状态
uint8_t SI24R1_ClearIRQ(uint8_t irqMask);          //清除SI24R1+引起的中断 

void SI24R1_FlushRX(void);//清空RX缓冲区
void SI24R1_FlushTX(void);//清空TX缓冲区

uint8_t SI24R1_ReadRxPayloadWidth(void);	//读取RX FIFO中第一个数据包的长度
uint8_t SI24R1_ReadAddressWidth(void); 		//读取SETUP_AW寄存器，了解当前地址宽度
uint8_t SI24R1_ReadRetransmitConfig(void);	//读取SETUP_RETR寄存器，了解自动重发延迟和次数
uint8_t SI24R1_ReadRxPayload(uint8_t *buffer);  //读取RX FIFO中第一个数据包的有效数据

void ST24R1_SetStandby(void);                          //设置为待机模式              
void SI24R1_SetShutdown(void);                         //设置为关断模式

void SI24R1_SetAddressWidth(uint8_t width);	        //设置SETUP_AW寄存器，配置地址宽度
void SI24R1_WriteTxAddress(uint8_t *addrBuffer, uint8_t addr_size);//写入TX地址
void SI24R1_WriteRxAddress(uint8_t pipeNum, uint8_t *addrBuffer, uint8_t addr_size);//写入RX地址
void SI24R1_SetFreq(uint8_t freq);		        //设置频率(信道)   
void SI24R1_Set_Power(SI24R1PowerType Power);	        //设置功率
void SI24R1_SetDataRate(SI24R1SpeedType drate);	        //设置数据速率 
void SI24R1_SetMode(SI24R1_MODE mode);                  //设置模式
void SI24R1_SetRetransmitConfig(uint8_t ard, uint8_t arc);	//设置SETUP_RETR寄存器，配置自动重发延迟和次数
void SI24R1_WriteTxPayload_NOACK(uint8_t *buffer, uint8_t size);//写入TX有效载荷，不带ACK
void SI24R1_WriteTxPayload_ACK(uint8_t *buffer, uint8_t size);  //写入TX有效载荷，带ACK
void SI24R1_WriteRxPayload_InAck(uint8_t *buffer, uint8_t size);//写入RX有效载荷，用于ACK模式接收数据包
//收发函数
uint8_t SI24R1_TxPacket(uint8_t *txbuf,uint8_t len, uint16_t timeout_ms); //发送数据包,阻塞等待发送完毕
uint8_t SI24R1_RxPacket(uint8_t *rxbuf); //非阻塞接收数据 同时处理ACK PAYLOAD数据包
uint8_t SI24R1_TxRxPacket(uint8_t *txbuf, uint8_t* rxbuf); //此函数支持双向通信 可以接收ACK PAYLOAD数据包
uint8_t SI24R1_RxPacket_ack(uint8_t *rxbuf); //将接收到的数据读到指定缓冲区,ACK模式
uint8_t SI24R1_Txfast( uint8_t *tx_buffer,uint8_t len); //SI24R1以最高效率发送数据包 无应答
uint8_t SI24R1_Rxfast(uint8_t *rxbuf); //SI24R1以最高效率接收数据包 无应答


void SI24R1_Init(void);				//初始化
void SI24R12_Init(void);				//初始化
uint8_t SI24R1_Check(void);			//自检


//接口函数
#define L01_CSN_LOW()      HAL_GPIO_WritePin(RF1_CSN_GPIO_Port, RF1_CSN_Pin,GPIO_PIN_RESET)  // CSN 置低
#define L01_CSN_HIGH()     HAL_GPIO_WritePin(RF1_CSN_GPIO_Port, RF1_CSN_Pin,GPIO_PIN_SET)      // CSN 置高
#define L01_CE_LOW()       HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin,GPIO_PIN_RESET)    // CE 置低
#define L01_CE_HIGH()      HAL_GPIO_WritePin(RF_CE_GPIO_Port, RF_CE_Pin,GPIO_PIN_SET)      // CE 置高
#define GET_L01_IRQ()      HAL_GPIO_ReadPin(RF_IRQ_GPIO_Port, RF_IRQ_Pin)  // 获取中断引脚状态
//#define SPI_ExchangeByte(data)      SPI3_ReadWriteByte(data) 
#define RF_Delay(Delay)					HAL_Delay(Delay)


/****************************************************************************************************************************/
#define MODEL_RX            1           //普通接收
#define MODEL_TX            2           //普通发射
#define MODEL_RX2           3           //接收模式2，用于双向传输
#define MODEL_TX2           4           //发送模式2，用于双向传输

#define TX_ADR_WIDTH        5           //5字节的地址宽度
#define RX_ADR_WIDTH        5           //5字节的地址宽度
#define TX_PLOAD_WIDTH      32          //32字节的用户数据宽度
#define RX_PLOAD_WIDTH      32          //32字节的用户数据宽度

/** SI24R1定义 */
/*************************************************寄存器操作命令**************************************************************/
#define SPI_READ_REG        0x00        //读配置寄存器，低5位为寄存器地址
#define SPI_WRITE_REG       0x20        //写配置寄存器，低5位为寄存器地址
#define R_RX_PL_WID         0x60        //读取收到的数据字节数
#define RD_RX_PLOAD         0x61        //读RX有效数据，1~32字节
#define WR_TX_PLOAD         0xA0        //写TX有效数据，1~32字节
#define WR_TX_PLOAD_NACK    0xB0        //写TX入不带 ACK 的发送有效载荷
#define FLUSH_TX            0xE1        //清除TX FIFO寄存器，发射模式下使用
#define FLUSH_RX            0xE2        //清除RX FIFO寄存器，接收模式下使用
#define REUSE_TX_PL         0xE3        //重新使用上一包数据，CE为高，数据包被不断发送

#define W_ACK_PLOAD         0xA8        //接收方，将数据通过ACK形式发送，最多允许3帧存放在FIFO中
#define NOP                 0xFF        //空操作，可以用来读状态寄存器

/*************************************************寄存器地址**************************************************************/

#define CONFIG          0x00        //配置寄存器地址，bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                                    //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能        
#define EN_AA           0x01        //使能自动应答功能 bit0~5 对应通道0~5
#define EN_RXADDR       0x02        //接收地址允许 bit0~5 对应通道0~5
#define SETUP_AW        0x03        //设置地址宽度(所有数据通道) bit0~1: 00,3字节 01,4字节, 02,5字节
#define SETUP_RETR      0x04        //建立自动重发;bit0~3:自动重发计数器;bit4~7:自动重发延时 250*x+86us
#define RF_CH           0x05        //RF通道,bit0~6工作通道频率
#define RF_SETUP        0x06        //RF寄存器，bit3:传输速率( 0:1M 1:2M);bit1~2:发射功率;bit0:噪声放大器增益

//状态
#define STATUS          0x07        //状态寄存器;bit0:TX FIFO满标志;bit1~3:接收数据通道号(最大:6);bit4:达到最多次重发次数
                                    //bit5:数据发送完成中断;bit6:接收数据中断
#define MAX_TX          0x10        //达到最大发送次数中断
#define TX_OK           0x20        //TX发送完成中断
#define RX_OK           0x40        //接收到数据中断

#define OBSERVE_TX      0x08        //发送检测寄存器,bit7~4:数据包丢失计数器;bit3~0:重发计数器
#define CD              0x09        //载波检测寄存器,bit0:载波检测
#define RX_ADDR_P0      0x0A        //数据通道0接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P1      0x0B        //数据通道1接收地址，最大长度5个字节，低字节在前
#define RX_ADDR_P2      0x0C        //数据通道2接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P3      0x0D        //数据通道3接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P4      0x0E        //数据通道4接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define RX_ADDR_P5      0x0F        //数据通道5接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define TX_ADDR         0x10        //发送地址(低字节在前),ShockBurstTM模式下，RX_ADDR_P0与地址相等
#define RX_PW_P0        0x11        //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12        //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13        //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14        //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15        //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16        //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS 		0x17        //FIFO状态寄存器;bit0:RX FIFO寄存器空标志;bit1:RX FIFO满标志;bit2~3保留
                                    //bit4:TX FIFO 空标志;bit5:TX FIFO满标志;bit6:1,循环发送上一数据包.0,不循环  

#define DYNPD           0x1C        //动态数据包长度寄存器；bit0~5:对应通道0~5的动态数据包长度使能
#define FEATRUE         0x1D        //动态特征寄存器；bit1:使能ACK负载（带负载数据的ACK包）;bit1:使能动态负载长度


/*************************************************位定义**************************************************************/

#define MASK_RX_DR           6 
#define MASK_TX_DS           5 
#define MASK_MAX_RT          4 
#define EN_CRC               3 
#define CRCO                 2 
#define PWR_UP               1 
#define PRIM_RX              0 

#define ENAA_P5              5 
#define ENAA_P4              4 
#define ENAA_P3              3 
#define ENAA_P2              2 
#define ENAA_P1              1 
#define ENAA_P0              0 

#define ERX_P5               5 
#define ERX_P4               4 
#define ERX_P3               3 
#define ERX_P2              2 
#define ERX_P1               1 
#define ERX_P0               0 

#define AW_RERSERVED         0x0 
#define AW_3BYTES            0x1
#define AW_4BYTES            0x2
#define AW_5BYTES            0x3

#define ARD_250US            (0x00<<4)
#define ARD_500US            (0x01<<4)
#define ARD_750US            (0x02<<4)
#define ARD_1000US           (0x03<<4)
#define ARD_2000US           (0x07<<4)
#define ARD_4000US           (0x0F<<4)
#define ARC_DISABLE           0x00
#define ARC_15                0x0F

#define CONT_WAVE             7 
#define RF_DR_LOW             5 
#define PLL_LOCK              4 
#define RF_DR_HIGH            3 
//bit2-bit1:
#define PWR_F12DB             (0x00<<1)
#define PWR_F6DB              (0x01<<1)
#define PWR_F4DB              (0x02<<1)
#define PWR_0DB               (0x03<<1)
#define PWR_1DB               (0x04<<1)
#define PWR_3DB               (0x05<<1)
#define PWR_4DB               (0x06<<1)
#define PWR_7DB               (0x07<<1)

#define RX_DR                 6 
#define TX_DS                 5 
#define MAX_RT                4 
//for bit3-bit1, 
#define TX_FULL_0             0 

#define RPD                   0 

#define TX_REUSE              6 
#define TX_FULL_1             5 
#define TX_EMPTY              4 
//bit3-bit2, reserved, only '00'
#define RX_FULL               1 
#define RX_EMPTY              0 

#define DPL_P5                5 
#define DPL_P4                4 
#define DPL_P3                3 
#define DPL_P2                2 
#define DPL_P1                1 
#define DPL_P0                0 

#define EN_DPL                2 
#define EN_ACK_PAY            1 
#define EN_DYN_ACK            0 
#define IRQ_ALL  ( (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) )

#endif
