#include "./internal_flash/bsp_internal_flash.h"   
#include "./usart/bsp_usart.h"




void writeFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){//����������һ���������ڴ���д������
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
int readFlashByte(uint16_t labelStart , uint8_t * data , uint16_t dataLen){//��ȡ��������һ���������ڴ�
	int i ;
	if((labelStart >= 1024) || (dataLen >= BYTESIZE_USEDDATAFLASH) || !data)return 0;
	for(i = 0 ; i < dataLen; i++)
	{*(data + i) = (*(__IO uint8_t *)(WRITE_FLAG_START_ADDR + labelStart + i));}
    return 1;
}





uint8_t getIAPUpgradeFlag(void){//��ȡ������־
uint8_t state = UPGRADE_REQUEST;
if(readFlashByte(LABEL_IAPUpgradeFlag,&state,1)){
	if(state == 0xff){
	state = UPGRADE_FREE;
    }
}
return state;
}
void writeIAPUpgradeFlag(uint8_t data){//д��������־
    uint8_t DATA[1] ;
	DATA[0] = data ;
	writeFlashByte(LABEL_IAPUpgradeFlag,DATA,1);
	return;
}



uint8_t getAPPState(void){//��ȡAPP״̬��־
	uint8_t AppState = APP_NOT_AVAILABLE;
	if(readFlashByte(LABEL_APP_STATE_Flag,&AppState,1)){
		/*����IAPʱ���������FLASH�����������ܻ�����APPҲ���ܲ�������APP������Ҫ�����ֳ�������������߼����£�*/
		if(AppState == 0xff){
			if( 0x20000000 == ((*((__IO uint32_t*)APPLICATIONADDRESS)) & 0x2FFE0000) ){//ͨ�������ջ����ַ�Ƿ�Ϸ����ж��Ƿ�����APP
				AppState = APP_AVAILABLE;
			}else{
				AppState = APP_NOT_AVAILABLE;
			}
		
	    }
	}
	return AppState;
}
void writeAPPStateFlag(uint8_t data){//д��APP״̬��־
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



void writeFlash(uint32_t numFrame,uint32_t sumFrame,uint8_t * pstart){//�յ���֡����д�뻺����
	int i; //��ʱ����
	uint32_t startAddress; //Ҫд��֡���ݵ���ʼ��ַ
	uint32_t endAddress; //Ҫд��֡���ݵĽ�����ַ
	uint32_t EraseCounter = 0x00; 
  uint32_t NbrOfPage = 0x00;	
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	uint32_t Address = 0x00;
	uint32_t * pData;
	//����
						FLASH_Unlock();
						//������еı�־
						FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
						//������Ҫ������ҳ��
						if(0 == numFrame){
							//��Ҫ������ҳ��
							NbrOfPage = sumFrame/8;
							if(sumFrame%8 > 0 ){
							   NbrOfPage ++ ;
							}
							//����
              for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
                {
                  FLASHStatus = FLASH_ErasePage(APP_CACHE_ADDRESS + (FLASH_PAGE_SIZE * EraseCounter));
                }	
						}
						
						//������ʼ�ͽ�����ַ
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
void CopyFromBufToRunArea(void){//�ӻ��嵽app���Ŀ���������
uint32_t i = 0; 
uint32_t NbrOfPage4byte = (APP_SIZE_PAGES * FLASH_PAGE_SIZE)/COPY_CELL_SIZE; //Ҫ�����ĵ�λ��������ÿ4���ֽ�Ϊһ����λ
FLASH_Status FLASHStatus = FLASH_COMPLETE; /*��¼ÿ�β�����״̬*/

	__IO  uint32_t * const pFlashData32 = (__IO uint32_t *)APPLICATIONADDRESS;//��4�ֽڵ���ʽ����APP��
	__IO  uint32_t * const pBuf32 = (__IO uint32_t *)APP_CACHE_ADDRESS;//��4�ֽڵ���ʽ���ʻ���
	
 /*����*/
FLASH_Unlock();

	
  /*������б�־λ*/
FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  /*ҳ����*/
for(i = 0; (i < APP_SIZE_PAGES) && (FLASHStatus == FLASH_COMPLETE); i++)
{
  FLASHStatus = FLASH_ErasePage(APPLICATIONADDRESS + (FLASH_PAGE_SIZE * i));
}


//����д��FLASH
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
