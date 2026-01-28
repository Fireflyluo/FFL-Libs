#include "SI24R1.h"

uint8_t enable_ack_payload;//是否允许数据回传
uint8_t si24r1_ack_payload_buffer[32];
uint16_t ack_payload_count;//已经发送的ACK PAYLOAD数据包个数
uint8_t irq_state=0;

static uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01};//{0xA0,0x01,0x07,0x0E,0x01};//发送地址
static uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xAA,0xBB,0xCC,0x00,0x01};//{0xA0,0x01,0x07,0x0E,0x01};//接收地址


uint8_t SPI_ExchangeByte(uint8_t data)
{
    uint8_t rxdata = 0;
	if( BSP_SPI_TransmitReceive(SPI_NUM_3,&data,&rxdata,1,1000)!= HAL_OK)
	{
    rxdata=0XFF;
	}
  return rxdata;
}

 /**********************************************************寄存器操作函数***************************************************************/

 /**
  * @brief :SI24R1读寄存器
  * @param :
           @Addr:寄存器地址
  * @note  :地址在设备中有效
  * @retval:读取的数据
  */
 uint8_t SI24R1_Read_Reg( uint8_t RegAddr )
 {
     uint8_t btmp;
         
     L01_CSN_LOW( );       //片选
         
     SPI_ExchangeByte( SPI_READ_REG | RegAddr );    //读命令 地址
     btmp = SPI_ExchangeByte(0xFF);               //读数据
         
     L01_CSN_HIGH( );       //取消片选
         
     return btmp;
 }
 
 /**
   * @brief :SI24R1读指定长度的数据
   * @param :
   *          @reg:地址
   *          @pBuf:数据存放地址
   *          @len:数据长度
   * @note  :数据长度不超过255，地址在设备中有效
   * @retval:读取状态
   */
 void SI24R1_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
 {
    uint8_t btmp;
        
    L01_CSN_LOW();                        //片选
    SPI_ExchangeByte( SPI_READ_REG | RegAddr );        //读命令 地址
    for( btmp = 0; btmp < len; btmp ++ )
    {
        *( pBuf + btmp ) = SPI_ExchangeByte( 0xFF );   //读数据
    }
    L01_CSN_HIGH( );                //取消片选
 }
 
 /**
   * @brief :SI24R1写寄存器
   * @param :无
   * @note  :地址在设备中有效
   * @retval:读写状态
   */
void SI24R1_Write_Reg( uint8_t RegAddr, uint8_t data )
 {
   
     L01_CSN_LOW();                //片选
     SPI_ExchangeByte( SPI_WRITE_REG | RegAddr ); //写命令 地址
     SPI_ExchangeByte( data );                   //写数据
    
     L01_CSN_HIGH();                //取消片选
 }
 
 /**
   * @brief :SI24R1写多个字节
   * @param :
   * @reg:地址
   * @pBuf:写入的数据地址
   * @len:数据长度
   * @note  :数据长度不超过255，地址在设备中有效
   * @retval:写状态
   */
 void SI24R1_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
 {
     uint8_t i;
         
     L01_CSN_LOW();                //片选 
     SPI_ExchangeByte( SPI_WRITE_REG | RegAddr );        //写命令 地址
     for( i = 0; i < len; i ++ )
     {
      SPI_ExchangeByte( *( pBuf + i ) );                //写数据
     }
         
     L01_CSN_HIGH();                //取消片选
 }
 /**********************************************************设置函数***************************************************************/
/*!
 *  @brief        设置为关断模式          
 *  @param        无
 *  @return       无  
 *  @note         该模式下，Si24R1所有收发功能模块关闭，芯片停止工作，消耗电流最小
*/
void SI24R1_SetShutdown(void)
{
    uint8_t controlreg = SI24R1_Read_Reg(CONFIG);
    SI24R1_Write_Reg(CONFIG, controlreg & (~(1 << PWR_UP)));
}

/*!
*  @brief        设置为待机模式       
 *  @param       无
 *  @return      无  
 *  @note        该模式下，只有晶体振荡器电路工作，保证了芯片在消耗较少电流的同时能够快速启动。
*/
void ST24R1_SetStandby(void)
{
  uint8_t controlreg = SI24R1_Read_Reg(CONFIG);
  SI24R1_Write_Reg(CONFIG, controlreg | (1 << PWR_UP));
}
 /*!
 *  @brief        设置nRF24L01+为TX/RX模式
 *  @param        mode: 模式 (TX/RX)
 *  @return       无
 *  @note         TX工作模式的条件为：
 *								TX FIFO 中有数据， CONFIG寄存器的PWR_UP位的值为1，PRIM_RX位的值为0，
 *								同时要求引脚CE上有一个至少持续10us的高脉冲。当发送端TX FIFO寄存器为空，并且引脚CE=1时，芯片进入到
 *								Idle-TX 模式。在该模式下，如果有新的数据包被送到TX FIFO中，芯片内部的电路将立即启动，切换到TX模式将数据包发送。
									RX工作模式的条件为：
									设置寄存器CONFIG的PWR_UP位的值为1，PRIM_RX位的值为1，并且引脚CE=1。
 */
void SI24R1_SetMode(SI24R1_MODE mode)
{
    uint8_t mask = SI24R1_Read_Reg(CONFIG);
    if (mode == TX_MODE)
    {
        mask &= ~(1 << PRIM_RX);
    }
    else if (mode == RX_MODE)
    {
        mask |= (1 << PRIM_RX);
    }
    SI24R1_Write_Reg(CONFIG, mask);
}

/*!
 *  @brief        清空TX缓冲区             
 *  @param        无 
 *  @return       无  
 *  @note          
*/
void SI24R1_FlushTX(void)
{
    SI24R1_Write_Reg(FLUSH_TX, 0xFF);
}
/*!
 *  @brief        清空RX缓冲区           
 *  @param        无
 *  @return       无  
 *  @note          
*/
void SI24R1_FlushRX(void)
{
    SI24R1_Write_Reg(FLUSH_RX, 0xFF);
}
 /*!
 *  @brief        重用最后一个发送的有效载荷           
 *  @param        无
 *  @return       无  
 *  @note          重新发送TX FIFO中的最后一个数据包
 */
void SI24R1_ReuseTxPayload(void)
{
    SI24R1_Write_Reg(REUSE_TX_PL, 0xFF);
}

 /*!
 *  @brief        设置频率(信道)          
 *  @param        freq: 频率跳变点，范围:0-125,2400Mhz-2525Mhz
 *  @return       无  
 *  @note          
*/
void SI24R1_SetFreq(uint8_t freq)
{
  if (freq < 126 && freq > 0) {
    SI24R1_Write_Reg(SPI_WRITE_REG + RF_CH, freq);
}
} 
/*!
 *  @brief        设置功率          
 *  @param        power: f12dB,f6dB,f4dB,0dB,1dB,3dB,4dB,7dB
 *  @return       无  
 *  @note          
*/
// void SI24R1_SetPower(uint8_t power)
// {
//     uint8_t reg = SI24R1_Read_Reg(RF_SETUP);
//     reg = (reg & 0xF9) | ((power << 1) & 0x06); // 保留其他位，设置PWR[2:1]
//     SI24R1_Write_Reg(SPI_WRITE_REG + RF_SETUP, reg);
// }
void SI24R1_Set_Power( SI24R1PowerType Power )
{
    uint8_t btmp;
        
        btmp = SI24R1_Read_Reg( RF_SETUP ) & ~0x07;
    switch( Power )
    {
         case POWER_F12DBM:
           btmp |= PWR_F12DB;
           break;
         case POWER_F6DBM:
           btmp |= PWR_F6DB;
           break;
         case POWER_F4DBM:
           btmp |= PWR_F4DB;
           break;
         case POWER_0DBM:
           btmp |= PWR_0DB;
           break;
         case POWER_1DBM:
           btmp |= PWR_1DB;
           break;
         case POWER_3DBM:
           btmp |= PWR_3DB;
           break;
         case POWER_4DBM:
           btmp |= PWR_4DB;
           break;
         case POWER_7DBM:
           btmp |= PWR_7DB;
           break;
        // default:
        //     break;
    }
    SI24R1_Write_Reg( RF_SETUP, btmp );
}

/*!
 *  @brief        设置数据速率          
 *  @param        drate: 250K,1M,2M
 *  @return       无  
 *  @note          
*/
void SI24R1_SetDataRate(SI24R1SpeedType drate)
{
    uint8_t reg = SI24R1_Read_Reg(RF_SETUP);
    reg = (reg & 0xD7); // 清除速率位
    if(drate == SPEED_250K)       // 250Kbps
        reg |= 0x20;
    else if(drate == SPEED_1M)  // 1Mbps
        reg &= 0xD7;     // 默认值
    else if(drate == SPEED_2M)  // 2Mbps
        reg |= 0x08;
    SI24R1_Write_Reg(SPI_WRITE_REG + RF_SETUP, reg);
}



/*!
 *  @brief        读取SI24R1+的接收数据长度           
 *  @param        无
 *  @return       接收数据长度（1-32字节）  
 *  @note          读取RX FIFO中第一个数据包的长度
 */
uint8_t SI24R1_ReadRxPayloadWidth(void)
{
    return SI24R1_Read_Reg(R_RX_PL_WID);
}

/*!
 *  @brief        读取SI24R1+的地址宽度           
 *  @param        无
 *  @return       地址宽度（3-5字节）  
 *  @note          读取SETUP_AW寄存器，了解当前地址宽度
 */
uint8_t SI24R1_ReadAddressWidth(void)
{
    return (SI24R1_Read_Reg(SETUP_AW) & 0x03) + 2; // 0x00: 非法, 0x01: 3字节, 0x02: 4字节, 0x03: 5字节
}

/*!
 *  @brief        设置SI24R1+的地址宽度           
 *  @param        width: 地址宽度（3-5字节）
 *  @return       无  
 *  @note          设置SETUP_AW寄存器，配置地址宽度
 */
void SI24R1_SetAddressWidth(uint8_t width)
{
    if (width >= 3 && width <= 5) {
        SI24R1_Write_Reg(SETUP_AW, width - 2); // 0x01: 3字节, 0x02: 4字节, 0x03: 5字节
    }
}

/*!
 *  @brief        将传输地址写入TX_ADDR寄存器
 *  @param        addrBuffer: 存储地址的缓冲区
 *  @param        addr_size: 地址字节数
 *  @return       无
 *  @note         仅用于PTX设备
 */
void SI24R1_WriteTxAddress(uint8_t *addrBuffer, uint8_t addr_size)
{
 uint8_t w_size = (addr_size > 5) ? 5 : addr_size;
 SI24R1_Write_Buf(SPI_WRITE_REG + TX_ADDR, addrBuffer, w_size);
}

/*!
*  @brief        为RX管道写入地址
*  @param        pipeNum: 数据管道编号
*  @param        addrBuffer: 存储地址的缓冲区
*  @param        addr_size: 地址字节数
*  @return       无
*  @note          
*/
void SI24R1_WriteRxAddress(uint8_t pipeNum, uint8_t *addrBuffer, uint8_t addr_size)
{
 uint8_t w_size = (addr_size > 5) ? 5 : addr_size;
 SI24R1_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0 + pipeNum, addrBuffer, w_size);
}

/*!
 *  @brief        读取SI24R1+的自动重发配置           
 *  @param        无
 *  @return       自动重发配置寄存器的值  
 *  @note          读取SETUP_RETR寄存器，了解自动重发延迟和次数
 */
uint8_t SI24R1_ReadRetransmitConfig(void)
{
    return SI24R1_Read_Reg(SETUP_RETR);
}

/*!
 *  @brief        设置SI24R1+的自动重发配置           
 *  @param        ard: 自动重发延迟（0-15，单位250us）
 *  @param        arc: 自动重发次数（0-15）
 *  @return       无  
 *  @note          设置SETUP_RETR寄存器，配置自动重发延迟和次数
 */
void SI24R1_SetRetransmitConfig(uint8_t ard, uint8_t arc)
{
    SI24R1_Write_Reg(SETUP_RETR, (uint8_t)(((ard & 0x0F) << 4) | (arc & 0x0F)));
}


 /**********************************************************功能函数***************************************************************/

/*!
 *  @brief        读取SI24R1+的状态寄存器           
 *  @param        无
 *  @return       状态寄存器的值  
 *  @note          读取STATUS寄存器的值
 */
uint8_t SI24R1_ReadStatus(void)
{
    return SI24R1_Read_Reg(STATUS);
}

/*!
 *  @brief        读取SI24R1+的中断状态           
 *  @param        无
 *  @return       irqMask: RX_DR(位[6]),TX_DS(位[5]),MAX_RT(位[4]) 
 *  @note          读取STATUS寄存器的中断标志位
 */
uint8_t SI24R1_ReadIRQStatus(void)
{
    return SI24R1_Read_Reg(STATUS) & 0x70; // 只保留RX_DR, TX_DS, MAX_RT位
}

/*!
 *  @brief        清除SI24R1+引起的中断           
 *  @param        irqMask: RX_DR(位[6]),TX_DS(位[5]),MAX_RT(位[4])
 *  @return       无  
 *  @note          清除STATUS寄存器的指定中断标志位
 */
uint8_t SI24R1_ClearIRQ(uint8_t IRQ_Source)
{
  uint8_t btmp = 0;
  IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
  btmp = SI24R1_ReadStatus();			      //读状态寄存器

  L01_CSN_LOW();	
  SPI_ExchangeByte( SPI_WRITE_REG + STATUS );	//写状态寄存器命令
  SPI_ExchangeByte( IRQ_Source | btmp );		//清相应中断标志
  L01_CSN_HIGH();	

  return (SI24R1_ReadStatus() );			    //返回状态寄存器状态
}


/*!
 *  @brief        从FIFO读取RX有效载荷并存储到缓冲区
 *  @param        buffer: 存储数据的缓冲区
 *  @return       要读取的长度
 *  @note          
 */
uint8_t SI24R1_ReadRxPayload(uint8_t *buffer)
{
    uint8_t width = SI24R1_ReadRxPayloadWidth();
    if (width > 32)
    {
        L01_CSN_HIGH();
        SI24R1_FlushRX();
        return 0;
    }
    SI24R1_Read_Buf(RD_RX_PLOAD, buffer, width);
    SI24R1_FlushRX();//清空RF的RX FIFO
    return width;
}
/*!
 *  @brief        写入ACK PAYLOAD，在RX模式下向数据管道写入TX有效载荷
 *  @param        buffer: 存储数据的缓冲区
 *  @param        size: 要写入的大小
 *  @return       无
 *  @note          
 */
 void SI24R1_WriteRxPayload_InAck(uint8_t *buffer, uint8_t size)
 {
  uint8_t w_size = (size > 32) ? 32 : size;
  SI24R1_Write_Buf(W_ACK_PLOAD,buffer,w_size);
 }

/*!
 *  @brief        向数据管道写入TX有效载荷，RX将返回ACK
 *  @param        buffer: 存储数据的缓冲区
 *  @param        size: 要写入的大小
 *  @return       无
 *  @note          
 */
 void SI24R1_WriteTxPayload_ACK(uint8_t *buffer, uint8_t size)
 {
  uint8_t i;
  uint8_t w_size = (size > 32) ? 32 : size;
  SI24R1_FlushTX();
  SI24R1_Write_Buf(WR_TX_PLOAD,buffer,w_size);
 }
 /*!
 *  @brief        向数据管道写入TX有效载荷，RX不会返回ACK
 *  @param        buffer: 存储数据的缓冲区
 *  @param        size: 要写入的大小
 *  @return       无
 *  @note          
 */
void SI24R1_WriteTxPayload_NOACK(uint8_t *buffer, uint8_t size)
{
 uint8_t i;
 uint8_t w_size = (size > 32) ? 32 : size;
 SI24R1_FlushTX();
 SI24R1_Write_Buf(WR_TX_PLOAD_NACK,buffer,w_size);
}
 /**
 * @brief :检查SI24R1模块是否在位。
 * @return 返回一个uint8_t类型的值，表示SI24R1模块是否在位。
 *         0表示SI24R1在位，1表示SI24R1不在位。
 */
 uint8_t SI24R1_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;   	 
	SI24R1_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	SI24R1_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址  	
	for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;					   
	if(i!=5)return 1;                               //SI24R1不在位	
	return 0;		                                    //SI24R1在位
}	

/**
  * @brief :初始化SI24R1模块
  * @param :无
  * @note  :配置SI24R1模块的基本参数
  * @retval:无
  */
void SI24R1_Init(void)
{  
	uint8_t status;
//	spi_Init();
	RF_Delay(10);
  status = SI24R1_Check(); // 检查模块是否在位
	if(status != 0){
		printf("SI24R1 ERR!");
	}else{
		printf("SI24R1 OK!");
	}
    L01_CE_LOW();// 设置CE为低电平，进入待机模式

    // 进入掉电模式
    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, 0x00); // 掉电模式
    // 清除所有中断标志
    SI24R1_Write_Reg(SPI_WRITE_REG + STATUS, 0x70); // 清除RX_DR, TX_DS, MAX_RT中断
		//清空FIFO
		SI24R1_FlushRX();
		SI24R1_FlushTX();
			
#if DYNAMIC_PACKET == 1
    // 动态有效载荷长度
    SI24R1_Write_Reg(SPI_WRITE_REG + DYNPD, (1 << DPL_P0)); // 启用管道 0 动态有效载荷长度
//    SI24R1_Write_Reg(SPI_WRITE_REG + FEATRUE, 0x07);//使能动态负载长度 使能ACK负载(带负载数据的ACK包) 使能命令W_TX_PAYLOAD_NOACK
																										//(1 << EN_DPL) | (1 << EN_ACK_PAY) | (1 << EN_DYN_ACK )
	 SI24R1_Write_Reg(SPI_WRITE_REG + FEATRUE, (1 << EN_DPL) | (1 << EN_ACK_PAY));
#elif DYNAMIC_PACKET == 0
    // 固定有效载荷长度
    SI24R1_Write_Reg(SPI_WRITE_REG + RX_PW_P0, RF_PACKET_SIZE);
#endif

//    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, (1 << EN_CRC));                  // 启用CRC，2字节校验
    SI24R1_Write_Reg(SPI_WRITE_REG + EN_AA, (1 << ENAA_P0));                  // 自动应答在管道 0
    SI24R1_Write_Reg(SPI_WRITE_REG + EN_RXADDR, (1 << ERX_P0));               // 启用RX管道 0
    SI24R1_Write_Reg(SPI_WRITE_REG + SETUP_AW, AW_5BYTES);                    // 设置地址宽度为5字节
		SI24R1_SetRetransmitConfig(0,5);																					// 设置自动重发延迟250Us和5次数
    SI24R1_Write_Reg(SPI_WRITE_REG + RF_SETUP, (1 << RF_DR_LOW) | PWR_F12DB);   // 设置数据速率和发射功率


    SI24R1_Write_Reg(SPI_WRITE_REG +RF_CH,45);        //设置信道45
		
    SI24R1_Write_Buf(SPI_WRITE_REG +TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    // 设置TX地址
    SI24R1_Write_Buf(SPI_WRITE_REG +RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); // 设置RX地址
    
   

    // 设置信道
    SI24R1_SetFreq(45);
    // 上电
    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, (1 << PWR_UP)); 

		SI24R1_SetMode(RX_MODE);// 设置为接收模式
		

    SI24R1_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);            						// 数据传输率2Mbps，发射功率7dBm
    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0f);              						// CRC使能，16位CRC校验，上电，接收模式
    SI24R1_Write_Reg(SPI_WRITE_REG + STATUS, 0xff);  

    L01_CE_HIGH( ); 
}
void SI24R12_Init(void)
{  
	uint8_t status;
//	spi_Init();
	RF_Delay(10);
  status = SI24R1_Check(); // 检查模块是否在位
	if(status != 0){
		printf("SI24R1 ERR!");
	}else{
		printf("SI24R1 OK!");
	}
    L01_CE_LOW();// 设置CE为低电平，进入待机模式

    SI24R1_Write_Buf(SPI_WRITE_REG +RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); // 设置RX地址
	  SI24R1_Write_Buf(SPI_WRITE_REG +TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    // 设置TX地址
    SI24R1_Write_Reg(SPI_WRITE_REG + EN_AA, (1 << ENAA_P0));                  // 自动应答在管道 0
    SI24R1_Write_Reg(SPI_WRITE_REG + EN_RXADDR, (1 << ERX_P0));               // 启用RX管道 0
		SI24R1_SetRetransmitConfig(2,10);																					// 设置自动重发延迟500Us和10次数
    SI24R1_Write_Reg(SPI_WRITE_REG +RF_CH,45);        //设置信道45
    SI24R1_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);            						// 数据传输率2Mbps，发射功率7dBm
	
	
    // 清除所有中断标志
    SI24R1_Write_Reg(SPI_WRITE_REG + STATUS, 0x70); // 清除RX_DR, TX_DS, MAX_RT中断
		//清空FIFO
		SI24R1_FlushRX();
		SI24R1_FlushTX();
			
#if DYNAMIC_PACKET == 1
    // 动态有效载荷长度
    SI24R1_Write_Reg(SPI_WRITE_REG + DYNPD, (1 << DPL_P0)); // 启用管道 0 动态有效载荷长度
//    SI24R1_Write_Reg(SPI_WRITE_REG + FEATRUE, 0x07);//使能动态负载长度 使能ACK负载(带负载数据的ACK包) 使能命令W_TX_PAYLOAD_NOACK
																										//(1 << EN_DPL) | (1 << EN_ACK_PAY) | (1 << EN_DYN_ACK )
	 SI24R1_Write_Reg(SPI_WRITE_REG + FEATRUE, (1 << EN_DPL) | (1 << EN_ACK_PAY));
#elif DYNAMIC_PACKET == 0
    // 固定有效载荷长度
    SI24R1_Write_Reg(SPI_WRITE_REG + RX_PW_P0, RF_PACKET_SIZE);
#endif

    SI24R1_Write_Reg(SPI_WRITE_REG + SETUP_AW, AW_5BYTES);                    // 设置地址宽度为5字节

//    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0f);              						// CRC使能，16位CRC校验，上电，接收模式
    SI24R1_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0e);              						// CRC使能，16位CRC校验，上电，发送模式

		SI24R1_SetMode(RX_MODE);// 设置为接收模式
		
//    SI24R1_Write_Reg(SPI_WRITE_REG + STATUS, 0xff);  

    L01_CE_HIGH( ); 
}

 /**********************************************************收发函数示例***************************************************************/
 
  /**
  * @brief :发送数据包,阻塞等待发送完毕
  * @param :
  *         @txbuff:要发送的数据包
  *         @len:数据包长度
  *         @timeout_ms: 最大等待时间
  * @note  :一次不超过32个字节
  * @retval: MAX_TX：达到最大重发次数  TX_OK：发送完成  0xFF:其他原因
  *         0x10:达到最大重发次数，发送失败 
  *         0x20:发送成功            
  *         0xff:发送失败
  */
 uint8_t SI24R1_TxPacket(uint8_t *txbuf,uint8_t len, uint16_t timeout_ms)
 {
  uint8_t _status = 0;
	uint16_t _1ms_count = 0;
//  SI24R1_FlushTX();
  L01_CE_LOW();	
   SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, len);
  L01_CE_HIGH();	//启动发送  
  while(GET_L01_IRQ() == 1)
  {
    RF_Delay(1);
    _1ms_count ++;
    if(_1ms_count >= timeout_ms)break;
  }
  if(_1ms_count >= timeout_ms){
    printf("si24r1: tx packet timeout!\r\n");
  }else{
    printf("si24r1: tx packet done! wait time: %dms\r\n", _1ms_count);
  }
  _status = SI24R1_ReadStatus();  											  //读取状态寄存器的值	 
  SI24R1_ClearIRQ(_status);
  if(_status & MAX_TX){
		SI24R1_FlushTX();	//清除TX FIFO寄存器
    printf("si24r1: tx failed: MAX TX\r\n");
		return MAX_TX; 
	}
	if(_status & TX_OK){
    printf("si24r1: tx success\r\n");
		return TX_OK;
	}
	printf("si24r1: tx failed: other error\r\n");
	return 0xFF;	//其他原因发送失败 
 }

  /**
  * @brief :非阻塞接收数据 同时处理ACK PAYLOAD数据包
  * @param :
  *         @txbuff:要接收的数据包
  * @note  :一次不超过32个字节
  * @retval:0x10:达到最大重发次数，发送失败 
            0x20:发送成功            
            0xff:发送失败
  */
uint8_t SI24R1_RxPacket(uint8_t *rxbuf)
{
  uint8_t _status = 0, _rxlength = 0;
  _status = SI24R1_ReadStatus();
  SI24R1_ClearIRQ(_status);//清除中断
  if(_status & RX_OK)	//接收到数据
  {
      _rxlength = SI24R1_ReadRxPayloadWidth();
      SI24R1_ReadRxPayload(rxbuf);
      return _rxlength; //返回接收到的数据字节数
  }
  if((_status & TX_OK) && (enable_ack_payload)){ //作为接收端收到TXDS表示ACK PAYLOAD发送完成
      // TX FIFO中ACK PAYLOAD已经自动清除
      // 这里再次进行手动清除
      SI24R1_FlushTX();
      // 再次填充TX FIFO 准备下一次返回数据
      SI24R1_WriteRxPayload_InAck( si24r1_ack_payload_buffer, 32);//写入ACK PAYLOAD
      ack_payload_count ++;//返回的数据包计数
  }
  return 0;//没有收到数据	
}
 
  /**
  * @brief :此函数支持双向通信 可以接收ACK PAYLOAD数据包
  * @param :
  *         @txbuff:要发送的数据包
  *         @rxbuff:要接收的数据包
  * @note  :一次不超过32个字节
  * @retval:0:发送成功
  *         1:接收到ACK PAYLOAD           
  *         2:发送超时
  *         3:其他原因
  */   
 uint8_t SI24R1_TxRxPacket(uint8_t *txbuf, uint8_t* rxbuf)
 {
     uint8_t rx_length = 0;//接收到的数据长度
     uint8_t ret = 3;
     uint8_t _status = 0;
     static uint16_t timeout_count = 0;//超时计数器
     _status = SI24R1_ReadStatus();		//读状态寄存器
     if(_status & (TX_DS | RX_DR | MAX_RT)){//存在中断
      SI24R1_ClearIRQ(_status);//清除对应中断
     }
     if(_status & TX_OK){//发送成功 重新装填
         ret=0;
         SI24R1_FlushTX();//清空之前要发送的数据
         L01_CE_LOW();	
         SI24R1_WriteTxPayload_ACK(txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
         L01_CE_HIGH();	//启动发送   
     }
     if(_status & RX_OK){//接收到ACK PAYLOAD
         // 读取ACK PAYLOAD数据
         SI24R1_ReadRxPayload(rxbuf);
         ret=1;
     }
     if(_status & MAX_TX){//发送超时 重新装填
         SI24R1_FlushTX();//清除TX FIFO寄存器
         ret=2;
         L01_CE_LOW();	
         SI24R1_WriteTxPayload_ACK(txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
         L01_CE_HIGH();//启动发送
     }
     if(ret==3){//无事情发生 或许应该强制发送以启动循环
         timeout_count ++;
         if(timeout_count >= 100){
             timeout_count = 0;
             // 强制发送
             SI24R1_FlushTX();	//清除TX FIFO寄存器
             L01_CE_LOW();	
             SI24R1_WriteTxPayload_ACK(txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
             L01_CE_HIGH();//启动发送
         }
     }else{
         timeout_count = 0;
     }
     return ret;
 }
 

 /**
  * @brief :将接收到的数据读到指定缓冲区,ACK模式
  * @param :
  *         @rxbuff:接收数据存放地址
  * @note  :一次不超过32个字节
  * @retval:0，未收到；len,收到数据长度
  */
 uint8_t SI24R1_RxPacket_ack(uint8_t *rxbuf)
 {
  uint8_t state,len;
	state = SI24R1_Read_Reg(STATUS);  			                 //读取状态寄存器的值    	  
	SI24R1_Write_Reg(SPI_WRITE_REG+STATUS,state);               //清除RX_DS中断标志

	if(state & RX_OK)								                           //接收到数据
	{		 
    if((len = SI24R1_ReadRxPayload(rxbuf)) != 0)
    {
      SI24R1_FlushTX(); // 清空RF的TX FIFO
      L01_CE_LOW();																							//CE拉低，使能SI24R1配置
      SI24R1_Write_Buf(W_ACK_PLOAD, rxbuf, len);	              // 写入ACK应答数据
      L01_CE_HIGH();																						//CE置高，使能发送	 
    }   
    SI24R1_FlushRX();    //清除RX FIFO寄存器
		return len; 
	}	   
	return 0;                                                   //没收到任何数据
 }

 /**
  * @brief :快速发送模式，无应答
  * @param :
  *         @rxbuff:接收数据存放地址
  *         @len:数据包长度
  * @note  :一次不超过32个字节
  * @retval:0:成功
  */
uint8_t SI24R1_Txfast( uint8_t *tx_buffer,uint8_t len)
{
  uint8_t size = (len > RF_PACKET_SIZE) ? RF_PACKET_SIZE:len;

//  L01_CE_LOW();																							//CE拉低，使能SI24R1配置
  SI24R1_Write_Buf(WR_TX_PLOAD, tx_buffer, size);	    //写数据到TX FIFO,32个字节
  L01_CE_HIGH();	

  return 0;
}
 /**
  * @brief :快速接收模式
  * @param :
  *         @rxbuff:接收数据存放地址
  * @note  :一次不超过32个字节
  * @retval:0，收到；len，收到数据长度
  */
uint8_t SI24R1_Rxfast(uint8_t *rxbuf)
{
 uint8_t state,len;
 state = SI24R1_Read_Reg(STATUS);  			                 //读取状态寄存器的值    	  
 SI24R1_Write_Reg(SPI_WRITE_REG+STATUS,state);               //清除RX_DS中断标志

 if(state & RX_OK)								                           //接收到数据
 {		 
   len = SI24R1_ReadRxPayload(rxbuf); 
   SI24R1_FlushRX();    //清除RX FIFO寄存器
   return len; 
 }	   
 return 0;                                                   //没收到任何数据
}

