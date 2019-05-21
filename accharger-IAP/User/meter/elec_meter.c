
#include "./meter/elec_meter.h"
static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;  
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

 /**
  * @brief  METER SPI模块初始化
  * @param 	None
  * @retval None
  */
void SPI_METER_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 使能SPI时钟 */
	METER_SPI_APBxClock_FUN ( METER_SPI_CLK, ENABLE );
	
	/* 使能SPI引脚相关的时钟 */
 	METER_SPI_CS_APBxClock_FUN ( METER_SPI_CS_CLK|METER_SPI_SCK_CLK|
																	METER_SPI_MISO_PIN|METER_SPI_MOSI_PIN, ENABLE );
	
  /* 配置SPI的 CS引脚，普通IO即可 */
  GPIO_InitStructure.GPIO_Pin = METER_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(METER_SPI_CS_PORT, &GPIO_InitStructure);
	
  /* 配置SPI的 SCK引脚*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(METER_SPI_SCK_PORT, &GPIO_InitStructure);

  /* 配置SPI的 MISO引脚*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_MISO_PIN;
  GPIO_Init(METER_SPI_MISO_PORT, &GPIO_InitStructure);

  /* 配置SPI的 MOSI引脚*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_MOSI_PIN;
  GPIO_Init(METER_SPI_MOSI_PORT, &GPIO_InitStructure);

  /* 停止信号 FLASH: CS引脚高电平*/
  SPI_METER_CS_HIGH();

  /* SPI 模式配置 */
  // FLASH芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

  // 上升沿输出，下降沿采样
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;    // SCK空闲状态时为低电平
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  // SCK的偶数边沿采样
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;     // 使用专用NSS引脚
	
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  // 64分频
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(METER_SPIx , &SPI_InitStructure);

  /* 使能 SPI  */
  SPI_Cmd(METER_SPIx , ENABLE);	
}



 /**
  * @brief  读取METER数据
  * @param 	address： 待读取的地址
  * @retval 返回读取的数据
  */
u32 MeterRead(u8 address)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	SPI_WRPROTECT_OPEN(address);
  SPI_METER_CS_LOW();
  SPI_METER_SendByte(address);
  Temp0 = SPI_METER_SendByte(Dummy_Byte);
  Temp1 = SPI_METER_SendByte(Dummy_Byte);
  Temp2 = SPI_METER_SendByte(Dummy_Byte);
  SPI_METER_CS_HIGH();
	
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}

/**
  * @brief  向METER写入数据
  * @param 	address： 待写入的地址
  * @retval 写入成功返回1
  */
u8 MeterWrite(u8 address ,u32 data)
{
	u32 temp=0;
	SPI_WRPROTECT_OPEN(address);
  SPI_METER_CS_LOW();
  SPI_METER_SendByte(address|0x80);
  SPI_METER_SendByte((data>>16)&0xff) ;
  SPI_METER_SendByte((data>>8)&0xff);
  SPI_METER_SendByte(data&0xff);
  SPI_METER_CS_HIGH();	
	temp = MeterRead(BACKUPDATA);
	if(temp==data)
		return 1;
	else
		return 0;
} 


/**
  * @brief  校表参数寄存器写保护打开
  * @param 	通过判断要操作的寄存器地址，写入相应的数据
  * @retval None
  */
void SPI_WRPROTECT_OPEN(u8 address)
{
	if(address>=0x50)
	{
		SPI_WRPROTECT1_OPEN();
	}
	else if(address>=0x40)
	{
		SPI_WRPROTECT2_OPEN();
	}
}

/**
  * @brief  校表参数寄存器写保护打开
  * @param 	只能操作50H~71H的寄存器
  * @retval None
  */
void SPI_WRPROTECT1_OPEN(void)
{	
	SPI_METER_CS_LOW();
  SPI_METER_SendByte(WPREG|0x80);
  SPI_METER_SendByte(0x00) ;
  SPI_METER_SendByte(0x00);
  SPI_METER_SendByte(0xa6);
  SPI_METER_CS_HIGH();
}


/**
  * @brief  校表参数寄存器写保护打开
  * @param 	只能操作40H~45H的寄存器
  * @retval None
  */
void SPI_WRPROTECT2_OPEN(void)
{	
	SPI_METER_CS_LOW();
  SPI_METER_SendByte(WPREG|0x80);
  SPI_METER_SendByte(0x00) ;
  SPI_METER_SendByte(0x00);
  SPI_METER_SendByte(0xbc);
  SPI_METER_CS_HIGH();
}


/**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
u8 SPI_METER_SendByte(u8 byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待发送缓冲区为空，TXE事件 */
  while (SPI_I2S_GetFlagStatus(METER_SPIx , SPI_I2S_FLAG_TXE) == RESET)
	{
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
   }

  /* 写入数据寄存器，把要写入的数据写入发送缓冲区 */
  SPI_I2S_SendData(METER_SPIx , byte);

	SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 等待接收缓冲区非空，RXNE事件 */
  while (SPI_I2S_GetFlagStatus(METER_SPIx , SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
   }

  /* 读取数据寄存器，获取接收缓冲区数据 */
  return SPI_I2S_ReceiveData(METER_SPIx );
}

/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
  printf("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}
