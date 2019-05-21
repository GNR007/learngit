
#include "./meter/elec_meter.h"
static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;  
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

 /**
  * @brief  METER SPIģ���ʼ��
  * @param 	None
  * @retval None
  */
void SPI_METER_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ʹ��SPIʱ�� */
	METER_SPI_APBxClock_FUN ( METER_SPI_CLK, ENABLE );
	
	/* ʹ��SPI������ص�ʱ�� */
 	METER_SPI_CS_APBxClock_FUN ( METER_SPI_CS_CLK|METER_SPI_SCK_CLK|
																	METER_SPI_MISO_PIN|METER_SPI_MOSI_PIN, ENABLE );
	
  /* ����SPI�� CS���ţ���ͨIO���� */
  GPIO_InitStructure.GPIO_Pin = METER_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(METER_SPI_CS_PORT, &GPIO_InitStructure);
	
  /* ����SPI�� SCK����*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(METER_SPI_SCK_PORT, &GPIO_InitStructure);

  /* ����SPI�� MISO����*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_MISO_PIN;
  GPIO_Init(METER_SPI_MISO_PORT, &GPIO_InitStructure);

  /* ����SPI�� MOSI����*/
  GPIO_InitStructure.GPIO_Pin = METER_SPI_MOSI_PIN;
  GPIO_Init(METER_SPI_MOSI_PORT, &GPIO_InitStructure);

  /* ֹͣ�ź� FLASH: CS���Ÿߵ�ƽ*/
  SPI_METER_CS_HIGH();

  /* SPI ģʽ���� */
  // FLASHоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

  // ������������½��ز���
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;    // SCK����״̬ʱΪ�͵�ƽ
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  // SCK��ż�����ز���
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;     // ʹ��ר��NSS����
	
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  // 64��Ƶ
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(METER_SPIx , &SPI_InitStructure);

  /* ʹ�� SPI  */
  SPI_Cmd(METER_SPIx , ENABLE);	
}



 /**
  * @brief  ��ȡMETER����
  * @param 	address�� ����ȡ�ĵ�ַ
  * @retval ���ض�ȡ������
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
  * @brief  ��METERд������
  * @param 	address�� ��д��ĵ�ַ
  * @retval д��ɹ�����1
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
  * @brief  У������Ĵ���д������
  * @param 	ͨ���ж�Ҫ�����ļĴ�����ַ��д����Ӧ������
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
  * @brief  У������Ĵ���д������
  * @param 	ֻ�ܲ���50H~71H�ļĴ���
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
  * @brief  У������Ĵ���д������
  * @param 	ֻ�ܲ���40H~45H�ļĴ���
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
  * @brief  ʹ��SPI����һ���ֽڵ�����
  * @param  byte��Ҫ���͵�����
  * @retval ���ؽ��յ�������
  */
u8 SPI_METER_SendByte(u8 byte)
{
	 SPITimeout = SPIT_FLAG_TIMEOUT;
  /* �ȴ����ͻ�����Ϊ�գ�TXE�¼� */
  while (SPI_I2S_GetFlagStatus(METER_SPIx , SPI_I2S_FLAG_TXE) == RESET)
	{
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
   }

  /* д�����ݼĴ�������Ҫд�������д�뷢�ͻ����� */
  SPI_I2S_SendData(METER_SPIx , byte);

	SPITimeout = SPIT_FLAG_TIMEOUT;
  /* �ȴ����ջ������ǿգ�RXNE�¼� */
  while (SPI_I2S_GetFlagStatus(METER_SPIx , SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
   }

  /* ��ȡ���ݼĴ�������ȡ���ջ��������� */
  return SPI_I2S_ReceiveData(METER_SPIx );
}

/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* �ȴ���ʱ��Ĵ���,���������Ϣ */
  printf("SPI �ȴ���ʱ!errorCode = %d",errorCode);
  return 0;
}
