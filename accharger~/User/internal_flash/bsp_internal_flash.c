/**
  ******************************************************************************
  * @file    bsp_internalFlash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   �ڲ�FLASH��д���Է���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32 �Ե� ������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./internal_flash/bsp_internal_flash.h"   
#include "./usart/bsp_usart.h"






/**
  * @brief  InternalFlash_Test,���ڲ�FLASH���ж�д����
  * @param  None
  * @retval None
  */
	
	#if 0
int InternalFlash_Test(void){
	uint32_t EraseCounter = 0x00; 	//��¼Ҫ��������ҳ
	uint32_t Address = 0x00;				//��¼д��ĵ�ַ
	uint32_t Data = 0x3210ABCD;			//��¼д�������
	uint32_t NbrOfPage = 0x00;			//��¼д�����ҳ
	
	FLASH_Status FLASHStatus = FLASH_COMPLETE; //��¼ÿ�β����Ľ��	
	TestStatus MemoryProgramStatus = PASSED;//��¼�������Խ��
	

  /* ���� */
  FLASH_Unlock();

  /* ����Ҫ��������ҳ */
  NbrOfPage = (WRITE_END_ADDR - WRITE_START_ADDR) / FLASH_PAGE_SIZE;

  /* ������б�־λ */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /* ��ҳ����*/
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
  
	}
  
  /* ���ڲ�FLASHд������ */
  Address = WRITE_START_ADDR;

  while((Address < WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  {
    FLASHStatus = FLASH_ProgramWord(Address, Data);
    Address = Address + 4;
  }

  FLASH_Lock();
  
  /* ���д��������Ƿ���ȷ */
  Address = WRITE_START_ADDR;

  while((Address < WRITE_END_ADDR) && (MemoryProgramStatus != FAILED))
  {
    if((*(__IO uint32_t*) Address) != Data)
    {
      MemoryProgramStatus = FAILED;
    }
    Address += 4;
  }
	return MemoryProgramStatus;
}


#endif

void writeFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){
	uint32_t i = 0; 
uint32_t NbrOfPage = 0;	//Ҫ������ҳ������ 
uint32_t NbrOfPage4byte = 0; //Ҫ�����ĵ�λ��������ÿ4���ֽ�Ϊһ����λ
FLASH_Status FLASHStatus = FLASH_COMPLETE; /*��¼ÿ�β�����״̬*/


	__IO  uint32_t * const pFlashData32 = (__IO uint32_t *)WRITE_FLAG_START_ADDR;//��4�ֽڵ���ʽ����FLASH
	 
	uint8_t buf[BYTESIZE_USEDDATAFLASH];//���ݻ���
	uint32_t* const pBuf32 = (uint32_t*)buf;//��4�ֽڵ���ʽ���ʻ���
	if((labelStart >= 1024) || (dataLen >= BYTESIZE_USEDDATAFLASH) || !data)return;

 /*����*/
FLASH_Unlock();

	//ȡ��FLASH��ԭ�е�����
	NbrOfPage4byte = BYTESIZE_USEDDATAFLASH/4; //��4�ֽ�Ϊ��λʱ����ʹ�õ��������Ĵ�С

	for(i = 0 ; i< NbrOfPage4byte;i++){
		pBuf32[i] = pFlashData32[i];
	}
//�޸�����
for(i = 0 ; i < dataLen; i++)
{buf[labelStart + i] = data[i];}
	
  /*����Ҫ������ҳ��*/
NbrOfPage = (WRITE_FLAG_END_ADDR - WRITE_FLAG_START_ADDR) / FLASH_PAGE_SIZE;

  /*������б�־λ*/
FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /*ҳ����*/
for(i = 0; (i < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); i++)
{
  FLASHStatus = FLASH_ErasePage(WRITE_FLAG_START_ADDR + (FLASH_PAGE_SIZE * i));
}

//����д��FLASH
    for(i=0;(i<NbrOfPage4byte)&&(FLASHStatus == FLASH_COMPLETE);i++){
                FLASHStatus = FLASH_ProgramWord( (uint32_t)(pFlashData32+i), pBuf32[i]);  
           }

 FLASH_Lock();
  
	return;
}

void readFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){
	int i ;
	if((labelStart >= 1024) || (dataLen >= BYTESIZE_USEDDATAFLASH) || !data)return;
	for(i = 0 ; i < dataLen; i++)
	{*(data + i) = (*(__IO uint8_t *)(UPGRADEFLAGADRESS + labelStart + i));}
}




void writeBaudRate(uint32_t BR){
	uint8_t DATA[4] ;
	*(uint32_t *)DATA= BR ;
	writeFlashByte(LABEL_BAUD_RATE_Flag,DATA,4);
	return;
}


void writeIAPAddress(SettingList * settingList){
	writeFlashByte(LABEL_ADDRESS_Flag,(uint8_t *)settingList->PileAddress,6);
	return;
}


void UpdateSettingList(SettingList * settingList){

uint32_t sizeSettingList ;
uint8_t * pData ;
if(NULL == settingList){
	return;
}
sizeSettingList = sizeof(SettingList);
pData = (uint8_t * )settingList;
 writeFlashByte(LABEL_settingList,pData,sizeSettingList);
 writeBaudRate(settingList->BaudRate);
 writeIAPAddress(settingList);

}

void GetSetting(SettingList * settingList){
	//int i;
	uint8_t * pData;
	uint16_t sizeSettingList ;
	
	if(NULL == settingList){
		return;
	}
	
	pData = (uint8_t * )settingList;
	
    sizeSettingList = sizeof(SettingList);
 
    readFlashByte(LABEL_settingList , pData , sizeSettingList);
	
}


void writeIAPUpgradeFlag(uint8_t data){//д���־λ
    uint8_t DATA[1] ;
	DATA[0] = data ;
	writeFlashByte(LABEL_IAPUpgradeFlag,DATA,1);
	return;
}

void readWorkingMode(uint8_t * data){
	uint8_t mode = *(__IO uint8_t *)(WRITE_FLAG_START_ADDR + LABEL_workingMode );
	if(mode == 0xff){
		mode = 1;
	}
	*data = mode;
}
void writeWorkingMode(uint8_t data){
	uint8_t DATA[1] ;
	DATA[0] = data ;
	writeFlashByte(LABEL_workingMode,DATA,1);
	return;
}


