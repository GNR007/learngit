#include "./internal_flash/bsp_internal_flash.h"   
#include "./usart/bsp_usart.h"




void writeFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){//往数据区中一段连续的内存中写入数据
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
int readFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){//读取数据区中一段连续的内存
	int i ;
	if((labelStart >= 1024) || (dataLen >= BYTESIZE_USEDDATAFLASH) || !data)return 0;
	for(i = 0 ; i < dataLen; i++)
	{*(data + i) = (*(__IO uint8_t *)(WRITE_FLAG_START_ADDR + labelStart + i));}
    return 1;
}





uint8_t getIAPUpgradeFlag(void){//获取升级标志
uint8_t state = UPGRADE_REQUEST;
if(readFlashByte(LABEL_IAPUpgradeFlag,&state,1)){
	if(state == 0xff){
	state = UPGRADE_FREE;
    }
}
return state;
}
void writeIAPUpgradeFlag(uint8_t data){//写入升级标志
    uint8_t DATA[1] ;
	DATA[0] = data ;
	writeFlashByte(LABEL_IAPUpgradeFlag,DATA,1);
	return;
}



uint8_t getAPPState(void){//获取APP状态标志
	uint8_t AppState = APP_NOT_AVAILABLE;
	if(readFlashByte(LABEL_APP_STATE_Flag,&AppState,1)){
		/*下载IAP时会擦除整个FLASH，接下来可能会下载APP也可能不会下载APP，程序要能区分出这两种情况，逻辑如下：*/
		if(AppState == 0xff){
			if( 0x20000000 == ((*((__IO uint32_t*)APPLICATIONADDRESS)) & 0x2FFE0000) ){//通过检测其栈顶地址是否合法来判断是否下载APP
				AppState = APP_AVAILABLE;
			}else{
				AppState = APP_NOT_AVAILABLE;
			}
		
	    }
	}
	return AppState;
}
void writeAPPStateFlag(uint8_t data){//写入APP状态标志
    uint8_t DATA[1] ;
	DATA[0] = data ;
	writeFlashByte(LABEL_APP_STATE_Flag,DATA,1);
	return;
}

PileAddress getAddress(void){
	PileAddress pileAddress;
	readFlashByte(LABEL_ADDRESS_Flag,(uint8_t *)&pileAddress,6);
	return pileAddress;
}



void writeFlash(uint32_t numFrame,uint32_t sumFrame,uint8_t * pstart){//收到的帧数据写入缓冲区
	int i; //临时变量
	uint32_t startAddress; //要写入帧数据的起始地址
	uint32_t endAddress; //要写入帧数据的结束地址
	uint32_t EraseCounter = 0x00; 
  uint32_t NbrOfPage = 0x00;	
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	uint32_t Address = 0x00;
	uint32_t * pData;
	//解锁
						FLASH_Unlock();
						//清空所有的标志
						FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
						//计算需要擦除的页数
						if(0 == numFrame){
							//需要擦除的页数
							NbrOfPage = sumFrame/8;
							if(sumFrame%8 > 0 ){
							   NbrOfPage ++ ;
							}
							//擦除
              for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
                {
                  FLASHStatus = FLASH_ErasePage(APP_CACHE_ADDRESS + (FLASH_PAGE_SIZE * EraseCounter));
                }	
						}
						
						//计算起始和结束地址
						startAddress = APP_CACHE_ADDRESS + numFrame * UPGRADEDATASIZE;
						endAddress = startAddress + UPGRADEDATASIZE;
						
						Address = startAddress;
 
            pData = (uint32_t * )pstart;
        
            for(i=0;(i<32)&&(Address < endAddress)&&(FLASHStatus == FLASH_COMPLETE);i++){
                FLASHStatus = FLASH_ProgramWord(Address, pData[i]);
                Address = Address + 4;	
           }

            FLASH_Lock();
						
	return;
}
void CopyFromBufToRunArea(void){//从缓冲到app区的拷贝函数。
uint32_t i = 0; 
uint32_t NbrOfPage4byte = (APP_SIZE_PAGES * FLASH_PAGE_SIZE)/COPY_CELL_SIZE; //要操作的单位的总数，每4个字节为一个单位
FLASH_Status FLASHStatus = FLASH_COMPLETE; /*记录每次操作的状态*/

	__IO  uint32_t * const pFlashData32 = (__IO uint32_t *)APPLICATIONADDRESS;//以4字节的形式访问APP区
	__IO  uint32_t * const pBuf32 = (__IO uint32_t *)APP_CACHE_ADDRESS;//以4字节的形式访问缓存
	
 /*解锁*/
FLASH_Unlock();

	
  /*清空所有标志位*/
FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /*页擦除*/
for(i = 0; (i < APP_SIZE_PAGES) && (FLASHStatus == FLASH_COMPLETE); i++)
{
  FLASHStatus = FLASH_ErasePage(APPLICATIONADDRESS + (FLASH_PAGE_SIZE * i));
}


//数据写入FLASH
    for(i=0;(i<NbrOfPage4byte)&&(FLASHStatus == FLASH_COMPLETE);i++){
                FLASHStatus = FLASH_ProgramWord( (uint32_t)(pFlashData32+i), pBuf32[i]);  
           }

 FLASH_Lock();
  
	return;
}

uint32_t getBaudRate(void){
	uint32_t baudRate = DefaultBaudRate;
	if(readFlashByte(LABEL_BAUD_RATE_Flag,(uint8_t *)&baudRate,4)){
		if(baudRate == 0xff){
			baudRate = DefaultBaudRate;
		}
	}
	if((baudRate != 19200) && (baudRate != 9600) && (baudRate != 4800) && (baudRate != 2400) &&(baudRate != 1200)){
			
			baudRate = DefaultBaudRate;
	}
return baudRate;
}
