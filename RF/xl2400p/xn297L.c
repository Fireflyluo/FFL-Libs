#include <stdio.h>
#include "xn297l.h"
/**
 * @brief 校准数据数组（5字节）
 * 基带校准数据数组（5字节）
 * 射频校准数据数组1、2
 * 解调校准数据数组1、2
 */
const uint8_t 
    BB_cal_data[]    = {0x12,0xEC,0x6F,0xA1,0x46},
    RF_cal_data[]    = {0xF6,0xBF,0x5D},
    RF_cal2_data[]   = {0xD5,0x21,0xEB,0x2C,0x5A,0x42},
    Dem_cal_data[]   = {0x1F},
    Dem_cal2_data[]  = {0x0B,0xDF,0x02};
// 全局变量定义
uint8_t xn297l_state, cbuf[2], xbuf[XN297L_PLOAD_WIDTH + 1];

 /************************ 寄存器操作 ************************************************/
extern SPI_HandleTypeDef hspi1;

/**
 * @brief  通过SPI发送并接收一个字节的数据
 * @param  data: 要发送的字节
 * @retval 接收到的字节
 */
uint8_t SPI_TxRxByte(uint8_t data)
{
    uint8_t rxdata = 0;
	if( BSP_SPI_TransmitReceive(SPI_NUM_3,&data,&rxdata,1,1000)!= HAL_OK)
	{
    rxdata=0XFF;
	}
  return rxdata;
}

/**
 * @brief  向指定寄存器写入一个字节
 * @param  reg: 寄存器地址
 * @param  value: 要写入的值
 * @retval 实际读取到的值（部分芯片返回状态）
 */
uint8_t XN297L_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t reg_val;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(value);
    XN297L_CSN_HIGH();
    return reg_val;
}

/**
 * @brief  从指定寄存器读取一个字节
 * @param  reg: 寄存器地址
 * @retval 读取到的值
 */
uint8_t XN297L_ReadReg(uint8_t reg)
{
    uint8_t reg_val;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    reg_val = SPI_TxRxByte(XN297L_CMD_NOP);
    XN297L_CSN_HIGH();
    return reg_val;
}

/**
 * @brief  向指定寄存器写入多个字节
 * @param  reg: 寄存器地址
 * @param  pBuf: 数据指针
 * @param  len: 数据长度
 */
void XN297L_WriteFromBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        SPI_TxRxByte(*pBuf++);
    }
    XN297L_CSN_HIGH();
}

/**
 * @brief  从指定寄存器读取多个字节
 * @param  reg: 寄存器地址
 * @param  pBuf: 缓冲区指针
 * @param  len: 读取长度
 */
void XN297L_ReadToBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t ctr;
    XN297L_CSN_LOW();
    SPI_TxRxByte(reg);
    for (ctr = 0; ctr < len; ctr++)
    {
        pBuf[ctr] = SPI_TxRxByte(XN297L_CMD_NOP);
    }
    XN297L_CSN_HIGH();
}
/**************************** 初始化与配置 ******************************/
/**
 * @brief 初始化 XN297L 模块，配置寄存器参数
 *
 * 包括软复位、清空 FIFO、设置自动应答、地址宽度、信道、载荷宽度等。
 * 最后根据载荷大小设置特性寄存器。
 */
void XN297L_Init(void)
{
    // 发送软复位命令（0x5A 和 0xA5），模拟上电过程
    XN297L_Reset();

    // 清空 TX 和 RX FIFO 缓冲区
    XN297L_ReuseTX();

    // 清除状态寄存器中的所有标志位
   XN297L_ClearStatus();

    // 启用管道0（P0~P5）
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_RXADDR, 0x01);
    // 设置地址宽度为 5 字节
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_AW, XN297L_SETUP_AW_5BYTE); // Address width  
    // 设置通信频道为 78（对应频率：2478MHz）
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, 78);          // Channel 78, 2478M HZ
    // 设置各接收管道的数据负载长度（均为 XN297L_PLOAD_WIDTH）
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P0, XN297L_PLOAD_WIDTH); // Payload width of P0  
  
    // 写入校准数据到各个寄存器
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));   // 基带校准数据
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data)); // 射频校准数据 2
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data)); // 解调器校准数据
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));   // 射频校准数据 1
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));// 解调器校准数据 2

    // 关闭动态负载长度功能
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_DYNPD, 0x00); 

    // 配置发射功率为 +9dBm，传输速率为 1Mbps
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP,  XN297L_RF_POWER_P_9|XN297L_RF_DR_1M); // 9dbm 1Mbps

    // 激活特定功能（如增强型 CRC 等）
    XN297L_WriteReg(XN297L_CMD_ACTIVATE, 0x73);

    // 关闭自动重传
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_RETR, 0x00); 
    // 关闭自动应答功能
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_AA, 0x00);   
    
    // 启用 RSSI 功能
    XN297L_EnableRSSI();

    // 根据负载大小选择是否启用 64 字节模式
    if(XN297L_PLOAD_WIDTH > 32)
    {
        // 启用 CE 引脚控制并支持 64 字节数据包
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE,
                        XN297L_FEATURE_BIT5_CE_SOFT | XN297L_FEATURE_BIT43_DATA_64BYTE);
    }
    else
    {
        // 仅启用 CE 软件控制
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT);
    }
}

/**
 * @brief 设置发送地址和接收管道 P0 的地址为相同值
 *
 * @param address 地址指针（5字节）
 */
void XN297L_SetTxAddress(const uint8_t *address)
{
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, address, 5);
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P0, address, 5);
}
/**
 * @brief 设置接收管道 P1 的地址
 *
 * @param address 地址指针（5字节）
 */
void XN297L_SetRxAddress(const uint8_t *address)
{
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P1, address, 5);
}
/**
 * @brief 设置无线通信频道
 *
 * @param channel 频道号（建议范围：0~80）
 */
void XN297L_SetChannel(uint8_t channel)
{    
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, channel);
}

/**
 * @brief  设置发射功率
 * @param  power: 功率配置值
 */
void XN297L_SetPower(uint8_t power)
{
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RF_SETUP, xbuf, 2);
	*(xbuf + 1) = power;
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP, xbuf, 2);

}
/**
 * @brief 启用 RSSI 功能
 *
 * 通过设置 RF_CAL 寄存器（0x1E）的最高位为 1 来启用 RSSI 模块。
 */
void XN297L_EnableRSSI(void)
{
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RF_CAL, xbuf, 3);
	*(xbuf + 1) |= (1 << 7);

    // 将更新后的值写回寄存器
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL, xbuf, 3);
}
/**
 * @brief 读取最近一次成功接收包的 RSSI 值
 *
 * 从 DATAOUT 寄存器中读取 ANADATA[3:0] 字段作为 RSSI 值
 *
 * @return uint8_t RSSI 值（范围：0 ~ 15）
 */
uint8_t XN297L_GetRSSI(void)
{
    uint8_t dataout;

    // 读取 DATAOUT 寄存器（即 RF_CH）
    dataout = XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RSSI);

    // 提取 ANADATA[3:0] 作为 RSSI 值
    return dataout & 0xF0;
}

/**************************** 电源管理 ******************************/
/**
 * @brief  将模块设置为休眠模式，降低功耗
 * @note    
 */
void XN297L_Sleep(void)
{
    // 写入配置寄存器，关闭发射/接收功能
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG, 0x0c);
    XN297L_CE_LOW();
}

/**
 * @brief 软件复位 XN297L 模块
 *
 * 向芯片发送软复位指令 (0x5A 和 0xA5)，模拟上电过程。
 */
void XN297L_Reset(void)
{
    // 发送软复位命令
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0x5A); 
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0xA5); 
    // 可选：延时等待复位完成
    HAL_Delay(1);
}
/**************************** 模式设置与校准测试 ************************************************/
/**
 * @brief 设置模块为发送模式
 */
void XN297L_SetTxMode(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8E);
    XN297L_CE_LOW(); 
}
/**
 * @brief 设置模块为接收模式
 */
void XN297L_SetRxMode(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8F);
    HAL_Delay(10);   
    XN297L_CE_HIGH();	
}
/**
 * @brief 测试 SPI 通信是否正常
 *
 * @return SUCCESS 成功，ERROR 失败
 */
ErrorStatus XN297L_SPI_Test(void)
{
    uint8_t i;
    const uint8_t *ptr = (const uint8_t *)XN297L_TEST_ADDR;
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, ptr, 5);
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
        if (*(xbuf + i) != *ptr++) return ERROR;
    }
    return SUCCESS;
}

/**
 * @brief 进行载波测试，用于调试RF模块
 *
 * @param ucChannel_Set 设置载波频率点
 */
void XN297L_Carrier(uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]   = {0x12, 0xED, 0x67, 0x9C, 0x46};
    uint8_t RF_cal_data[]   = {0xF6, 0x3F, 0x5D};
    uint8_t RF_cal2_data[]  = {0x45, 0x21, 0xEF, 0x2C, 0x5A, 0x42};
    uint8_t Dem_cal_data[]  = {0xE1};
    uint8_t Dem_cal2_data[] = {0x0B, 0xDF, 0x02};

    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0x5A); // Soft reset
    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0XA5);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT);

    XN297L_CE_LOW();
    HAL_Delay(0);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG, 0X8E);         // tx mode
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, ucChannel_Set); // carrier frequency point
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP, XN297L_RF_POWER_P_9|XN297L_RF_DR_1M);
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_BB_CAL, BB_cal_data, sizeof(BB_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_RF_CAL2, RF_cal2_data, sizeof(RF_cal2_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_DEM_CAL, Dem_cal_data, sizeof(Dem_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_RF_CAL, RF_cal_data, sizeof(RF_cal_data));
    XN297L_WriteFromBuf(XN297L_CMD_W_REGISTER + XN297L_REG_DEM_CAL2, Dem_cal2_data, sizeof(Dem_cal2_data));
    HAL_Delay(0);
}

/**************************** 状态管理 ******************************/

/**
 * @brief 读取模块状态寄存器
 *
 * @return 状态寄存器的值
 */
uint8_t XN297L_ReadStatus(void)
{
    xn297l_state = XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_STATUS);
    return xn297l_state;
}

/**
 * @brief 清除模块状态寄存器的所有标志位
 */
void XN297L_ClearStatus(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);
}

/**
 * @brief  清空 TX/RX FIFO 缓冲区
 */
void XN297L_FlushRxTX(void)
{
    XN297L_WriteReg(XN297L_CMD_FLUSH_TX, XN297L_CMD_NOP);
    XN297L_WriteReg(XN297L_CMD_FLUSH_RX, XN297L_CMD_NOP);
}
/**************************** 收发操作 ****************************************/
/**
 * @brief 发送数据
 *
 * @param ucPayload 数据指针
 * @param length 数据长度
 *
 * @return SUCCESS 成功，ERROR 失败
 */
ErrorStatus XN297L_TxFast(const uint8_t *ucPayload, uint8_t length)
{
    //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
    while ((XN297L_ReadStatus() & XN297L_FLAG_TX_FULL)) {
        if (xn297l_state & XN297L_FLAG_MAX_RT) {
            return ERROR;
        }
    }
    XN297L_WriteFromBuf(XN297L_CMD_W_TX_PAYLOAD, ucPayload, length);
    XN297L_CE_HIGH();
    return SUCCESS;
}

/**
 * @brief 重新发送数据
 */
void XN297L_ReuseTX(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, XN297L_FLAG_MAX_RT); //Clear max retry flag
    XN297L_CE_LOW();
    XN297L_CE_HIGH();
}

/**
 * @brief 发送数据
 *
 * @param ucPayload 数据指针
 * @param length 数据长度
 *
 * @return 状态寄存器的值
 */
uint8_t XN297L_TxData(uint8_t *ucPayload, uint8_t length)
{
    uint8_t y = 100, status = 0;
    XN297L_CE_HIGH();
    __NOP();
    XN297L_WriteFromBuf(XN297L_CMD_W_TX_PAYLOAD, ucPayload, length);
    // Retry until timeout
    while (y--)
    {
        HAL_Delay(1);
        status = XN297L_ReadStatus();
        // If TX successful or retry timeout, exit
        if ((status & (XN297L_FLAG_MAX_RT | XN297L_FLAG_TX_DS)) != 0)
        {
            //printf(" %d %02x\r\n", y, status);
            break;
        }
    }
    XN297L_WriteReg(XN297L_CMD_FLUSH_TX, 0);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);
    XN297L_CE_LOW();
    return status;
}

/**
 * @brief 接收数据
 *
 * @return 状态寄存器的值
 */
uint8_t XN297L_DumpRxData(void)
{
    uint8_t status, rxplWidth;
    status = XN297L_ReadStatus();
    XN297L_GetRSSI();
    if (status & XN297L_FLAG_RX_DR)
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, status);
        rxplWidth = XN297L_ReadReg(XN297L_CMD_R_RX_PL_WID);
        XN297L_ReadToBuf(XN297L_CMD_R_RX_PAYLOAD, xbuf, rxplWidth);
    }
    return status;
}


/**
 * @brief 打印 XN297L 芯片的所有关键寄存器状态信息
 *
 * 包括配置、使能、地址、负载宽度、FIFO状态、功能寄存器等。
 *
 * @return 当前状态寄存器值
 */
uint8_t XN297L_PrintStatus(void)
{
    uint8_t i, status;

    printf("\r\n[Config]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_CONFIG));
    printf("  [EN_AA]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_EN_AA));
    printf("  [EN_RxAddr]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_EN_RXADDR));
    printf("  [AddrWidth]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_SETUP_AW));
    printf("  [Retry]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_SETUP_RETR));

    printf("\r\n[RF_Channel]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RF_CH));
    printf("  [RF_Setup]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RF_SETUP));
    printf("  [Observe_Tx]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_OBSERVE_TX, xbuf, 4);
    for (i = 0; i < 4; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[TxAddr] ");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_TX_ADDR, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxAddrP0]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P0, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P1]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P1, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P2]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P2, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P3]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P3, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P4]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P4, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf(" [P5]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_RX_ADDR_P5, xbuf, 5);
    for (i = 0; i < 5; i++) {
        printf("%02X", *(xbuf + i));
    }

    printf("\r\n[RxPloadWidth_P0-P5]");
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P0));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P1));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P2));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P3));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P4));
    printf("%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_RX_PW_P5));

    printf("\r\n[FIFO_Status]");
    XN297L_ReadToBuf(XN297L_CMD_R_REGISTER | XN297L_REG_FIFO_STATUS, xbuf, 3);
    for (i = 0; i < 3; i++) {
        printf("%02X", *(xbuf + i));
    }
    printf("  [DynPloadWidth]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_DYNPD));
    printf("  [Feature]0x%02X", XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_FEATURE));

    status = XN297L_ReadStatus();
    printf("\r\n[Status]0x%02X\r\n\r\n", status);
    return status;
}