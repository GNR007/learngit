/**
  ******************************************************************************
  * @file    bsp_internalFlash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   内部FLASH读写测试范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32 霸道 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./internal_flash/bsp_internal_flash.h"   
#include "./usart/bsp_usart.h"






/**
  * @brief  InternalFlash_Test,对内部FLASH进行读写测试
  * @param  None
  * @retval None
  */
	
	#if 0
int InternalFlash_Test(void){
	uint32_t EraseCounter = 0x00; 	//记录要擦除多少页
	uint32_t Address = 0x00;				//记录写入的地址
	uint32_t Data = 0x3210ABCD;			//记录写入的数据
	uint32_t NbrOfPage = 0x00;			//记录写入多少页
	
	FLASH_Status FLASHStatus = FLASH_COMPLETE; //记录每次擦除的结果	
	TestStatus MemoryProgramStatus = PASSED;//记录整个测试结果
	

  /* 解锁 */
  FLASH_Unlock();

  /* 计算要擦除多少页 */
  NbrOfPage = (WRITE_END_ADDR - WRITE_START_ADDR) / FLASH_PAGE_SIZE;

  /* 清空所有标志位 */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /* 按页擦除*/
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
  
	}
  
  /* 向内部FLASH写入数据 */
  Address = WRITE_START_ADDR;

  while((Address < WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
  {
    FLASHStatus = FLASH_ProgramWord(Address, Data);
    Address = Address + 4;
  }

  FLASH_Lock();
  
  /* 检查写入的数据是否正确 */
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
uint32_t NbrOfPage = 0;	//要操作的页的总数 
uint32_t NbrOfPage4byte = 0; //要操作的单位的总数，每4个字节为一个单位
FLASH_Status FLASHStatus = FLASH_COMPLETE; /*记录每次操作的状态*/


	__IO  uint32_t * const pFlashData32 = (__IO uint32_t *)WRITE_FLAG_START_ADDR;//以4字节的形式访问FLASH
	 
	uint8_t buf[BYTESIZE_USEDDATAFLASH];//数据缓冲
	uint32_t* const pBuf32 = (uint32_t*)buf;//以4字节的形式访问缓冲
	if((labelStart >= 1024) || (dataLen >= BYTESIZE_USEDDATAFLASH) || !data)return;

 /*解锁*/
FLASH_Unlock();

	//取出FLASH中原有的数据
	NbrOfPage4byte = BYTESIZE_USEDDATAFLASH/4; //以4字节为单位时，已使用的数据区的大小

	for(i = 0 ; i< NbrOfPage4byte;i++){
		pBuf32[i] = pFlashData32[i];
	}
//修改数据
for(i = 0 ; i < dataLen; i++)
{buf[labelStart + i] = data[i];}
	
  /*计算要擦出的页数*/
NbrOfPage = (WRITE_FLAG_END_ADDR - WRITE_FLAG_START_ADDR) / FLASH_PAGE_SIZE;

  /*清空所有标志位*/
FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /*页擦除*/
for(i = 0; (i < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); i++)
{
  FLASHStatus = FLASH_ErasePage(WRITE_FLAG_START_ADDR + (FLASH_PAGE_SIZE * i));
}

//数据写入FLASH
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


void writeIAPUpgradeFlag(uint8_t data){//写入标志位
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


