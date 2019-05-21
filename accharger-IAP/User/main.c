/**
  ******************************************************************************
  * @file    main.c
  * @author  YunQing
  * @version V1.0
  * @date    2019-02-20
  * @brief   ����IAP
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
 
#include "./meter/elec_meter.h"
#include "./usart/bsp_usart.h"
#include "./internal_flash/bsp_internal_flash.h" 
#include "./485/bsp_485.h"

//#include "./SysTick/bsp_SysTick.h"
#include "core_cm3.h"
#include "misc.h"
#include "./dlt645_07/dlt645_07_api.h"
#include "./crc16/crc16.h"

typedef uint8_t bool;
#define true 1
#define false 0



#define WDG_CESHI 0 //���Ź�����
#define IAP_iap 1 //IAP���ԣ�iap��


typedef struct {
		uint32_t dataIdentification;
		uint16_t frameNumber;
		uint16_t frameToal;
}Determine;//�ж�һ֡


#define ID_REBOOT (0x00000005)
#define ID_EXECUTION_UPGRADE (0x00000009) //ִ�����������ݱ�ʶ
#define ID_UPGRADE_SWITCHMODE (0x00000007) //�л�����ģʽ��֡���ݱ�ʶ
#define ID_UPGRADEDATA 8 //��������֡�����ݱ�ʶ
//#define ID_UPGRADEREQUEST 9 //ִ�����������ݱ�ʶ


#define ENDWORK_REBOOT (1)
#define ENDWORK_IAP2APP (2)

//ָʾ�Ƶ����Ŷ���
#define PB_LED_LINK	GPIO_Pin_4  //���ӵ�
#define PB_LED_ERR		GPIO_Pin_5  //����ָʾ��


#define __DEBUG___
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("File: "__FILE__", Line: %d: "format"\n", __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif

typedef struct{char a[6];} *pAdress6 ;

uint8_t iapUpgradeFlag = 0;
uint8_t appStateFlag = 0; //0��ʾAPP���ĳ��򲻿��ã�1��ʾ����



#if IAP_iap

#define FRAMEBUFSIZE 216 //֡�������Ĵ�С

#define CTRL_UPGRADEDATA_LASTONR 0x14 //�����������ݵ�֡�Ŀ����룬���һ֡�Ŀ�����
#define CTRL_UPGRADEDATA_NOTLAST 0x34 //�����������ݵ�֡�Ŀ����룬�����һ֡�Ŀ�����
#define ID_UPGRADEDATA 8 //��������֡�����ݱ�ʶ
#define INDEX_UPGRADEFRAMENUM 4 //���������Ƕ�֡����ģ�֡�б�ʶ֡��ŵ������ֽڵ���ʼ�ֽڵ�λ��
#define INDEX_UPGRADEFRAMETOTALNUM 6 //��������ڻ�����λ��
#define INDEX_UPGRADEFRAMECRC 136 //CRCУ������λ��
#define LEN_CRCDATA 132 //CRCУ�����ݵ��ֽ���
#define INDEX_UPGRADEDATA 8 //����������ʼλ��
#define CTRL_NORMALRESPONSE_UPGRADEFRAME 0x94 //����֡��վ����Ӧ��Ŀ�����
#define CTRL_ABNORMALRESPONSE_UPGRADEFRAME 0xD4 //����֡��վ�쳣Ӧ��Ŀ�����
typedef void (*pFunction)(void); 
char benjiAddress[6]={0}; 
char BroadcastAddress[6]={0x99,0x99,0x99,0x99,0x99,0x99};



#endif


void DelayUs(uint32_t us){
	volatile uint32_t i;
#if 1
	for (i = 0; i < us; i++) {
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
#else
	//i = 8 * us;
	//while(i--);
#endif
}


bool cmpStr(const char * str1 ,const char * str2){//�Ƚ��ַ�����ǰADDRESSLEN���ַ��Ƿ����
	int i;
	for(i=0;i<ADDRESSLEN;i++){
		if(str1[i] != str2[i]){
			return false;
		}
	}
	return true;
}


/**/void SoftReset(void){//����
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}



#if IAP_iap

void iap_2_app(uint32_t appxAddr){//��IAP��ת��APP
    pFunction Jump_To_App;
    uint32_t JumpAddress;
	if( 0x20000000 == ((*((__IO uint32_t*)appxAddr)) & 0x2FFE0000) ){//���ջ��ָ���Ƿ���ȷ
		
		//��ȡ��λ�жϷ������ĵ�ַ
		JumpAddress = * ((__IO uint32_t * ) (appxAddr + 4));
		
		//�ѵ�ַǿת�ɺ���ָ��
		Jump_To_App = (pFunction)JumpAddress;
		
		//��������ջָ��
		__set_MSP(*(__IO uint32_t*) appxAddr);
		
		//ִ�и�λ����
		Jump_To_App();
		
	}
}


bool frameEqualsTheOperation(const Determine*frame1,const Determine*frame2){
		if((frame1->dataIdentification == frame2->dataIdentification) \
	    && (frame1->frameNumber == frame2->frameNumber) \
	    && (frame1->frameToal == frame2->frameToal)){
		    return true;
	    }else{
			return false;
		}
		
	}


void listen(S_D07_UNPACK * pFrame){
	
    S_D07_PACK_FRAME framePackBuf;//��װ�ظ�֡�õ�����Ϣ�洢������󣬵��÷�װ������֡
	
    uint8_t frameBuf[FRAMEBUFSIZE]={0}; //֡�Ļ�����
	uint32_t frameLen = 0; //֡�ĳ���
	uint16_t crc=0; //����֡��CRCУ��
	uint16_t sumCRC = 0; //�Լ����CRC

    //����֡����������
	static Determine oldFrame = {ID_UPGRADE_SWITCHMODE , 0 , 0};//��ʼ��ʱ��һ֡Ϊ�л�����ģʽ֡
    static Determine waitingFrame = {ID_UPGRADEDATA , 0 , 0};//�ȴ���֡Ϊ����֡�ĵ�һ֡
    Determine receivedFrame = {0,0,0}; //�յ���֡
	//����֡�ĳ���
	const Determine rebootFrame = {ID_REBOOT,0,0};
	
	bool executingResults; //ָ���ִ�н��
	uint8_t errorNumber; //������
	uint8_t endWork = 0; //����ʱҪ���еĲ���
	
	if(!pFrame){
		DEBUG("����ָ��Ϊ��,��������");
		return;
	}
	
//��ַУ��
	if((cmpStr (pFrame->address,benjiAddress) == false) && (cmpStr( pFrame->address,BroadcastAddress) == false)){
		DEBUG("\n��ַ����");
		return;
	}
//������У��
	if(  ( (CTRL_UPGRADEDATA_NOTLAST != pFrame->ctrl_c)  ) && (CTRL_UPGRADEDATA_LASTONR != pFrame->ctrl_c) ){//����֡�Ŀ����룬���һ֡Ϊ0x14������Ϊ0x34
		DEBUG("\n���������");
		//�ظ��쳣����goto
		executingResults = false;
		errorNumber = 1;
		goto END;
	}

{
//ˢ���յ���֡
receivedFrame.dataIdentification = pFrame->ruler_id ; //�յ���֡�����ݱ�ʶ
if(ID_UPGRADEDATA == receivedFrame.dataIdentification){//����յ���֡Ϊ����֡
	receivedFrame.frameNumber = (*((uint16_t * )(&(pFrame->aucDataTmp[INDEX_UPGRADEFRAMENUM]))));
	receivedFrame.frameToal = (*((uint16_t * )(&(pFrame->aucDataTmp[INDEX_UPGRADEFRAMETOTALNUM]))));
	if( (receivedFrame.frameNumber == 0) && (waitingFrame.dataIdentification == ID_UPGRADEDATA) && (waitingFrame.frameToal == 0) ){//����ȴ���֡������֡�ĵ�һ֡�����µȴ�֡��֡����
		waitingFrame.frameToal = receivedFrame.frameToal;
	}
}



	if(frameEqualsTheOperation(&receivedFrame,&waitingFrame)){ //����յ���֡Ϊ���ȴ���֡
		//����()��
		{	
		   if(receivedFrame.dataIdentification == ID_UPGRADEDATA ){//����֡
		         //д��FLASH����
				crc = (*(uint16_t * )(&pFrame->aucDataTmp[INDEX_UPGRADEFRAMECRC]));
			 
				sumCRC = crc16((char *)(pFrame->aucDataTmp + INDEX_UPGRADEFRAMENUM),LEN_CRCDATA);

					if(crc != sumCRC){
						DEBUG("\n crc error\n");
						//�ظ��쳣����goto
						executingResults = false;
		                errorNumber = 2;
						goto END;
					}	
					
				writeFlash(receivedFrame.frameNumber,receivedFrame.frameToal,&(pFrame->aucDataTmp[INDEX_UPGRADEDATA]));
				if(receivedFrame.frameNumber == receivedFrame.frameToal){
					iapUpgradeFlag = UPGRADE_RECEIVE_COMPLETE;
					writeIAPUpgradeFlag(iapUpgradeFlag);//�������ݽ�����ɣ�����������־
				}
				//�ظ���������
				executingResults = true;
		    }else if(receivedFrame.dataIdentification == ID_EXECUTION_UPGRADE) {//ִ�����������֡
					//�ظ���������****
					iapUpgradeFlag = UPGRADE_COPYING;
					writeIAPUpgradeFlag(iapUpgradeFlag); //���±�־λΪ �� ��������ִ����
					appStateFlag = APP_NOT_AVAILABLE;
					writeAPPStateFlag(appStateFlag); //���±�־λΪ �� APP��������
					
					CopyFromBufToRunArea();
					
					iapUpgradeFlag = UPGRADE_FREE;
					writeIAPUpgradeFlag(iapUpgradeFlag); //���±�־λΪ ������
					appStateFlag = APP_AVAILABLE;
					writeAPPStateFlag(appStateFlag); //���±�־λΪ ��APP������
					
					executingResults = true;
					endWork = ENDWORK_IAP2APP;
					//goto END;
		    }
			
		}
		//����,�ȴ�����һ֡()��
		//��һ֡ = �ȴ���֡

		oldFrame = waitingFrame;
		//�ȴ���֡ = ��һ֡
		if(receivedFrame.dataIdentification == ID_UPGRADEDATA ){//����֡
			if(receivedFrame.frameNumber == receivedFrame.frameToal && (receivedFrame.frameToal != 0)){//���Ϊ���һ֡
				waitingFrame.dataIdentification = ID_EXECUTION_UPGRADE;
				waitingFrame.frameNumber = 0;
				waitingFrame.frameToal = 0;
			}else if(receivedFrame.frameNumber < receivedFrame.frameToal){//�������һ֡
				waitingFrame.frameNumber++;//���µȴ���֡Ϊ��һ������֡
			}

		}
	}
	else if( frameEqualsTheOperation(&receivedFrame,&oldFrame) ){ //����յ���֡Ϊ��һ֡
		//�ظ�����
		executingResults = true;
	}
	else if ( frameEqualsTheOperation(&receivedFrame,&rebootFrame) ){ //����յ���֡Ϊ����֡
		//�ظ�����*******
		executingResults = true;
		endWork = ENDWORK_REBOOT;
		//goto END;
	}
	else{
		//�ظ��쳣����
		executingResults = false;
		errorNumber = 1;
		
	}
}
//����ظ�
{
END:
	if(executingResults == true){//����
		framePackBuf.ctrl_C = 0x94;
        framePackBuf.data_L = 0x0;
	}
    else{//�쳣
		framePackBuf.ctrl_C = 0xD4;
        framePackBuf.data_L = 0x1;
        framePackBuf.data[0]= errorNumber;
	}

	*(pAdress6)framePackBuf.Address_receiver = *(pAdress6)benjiAddress;
	//���
	if(cmpStr( pFrame->address,BroadcastAddress) == false){//���ǹ㲥��ַʱ����ظ�
	    if(0 == pack_d07_frame_by_data(&framePackBuf, frameBuf, &frameLen)){
	//����
		   RS485_SendStr_length(frameBuf,frameLen);	
	    }else{
		   printf("\n���ʧ��\n");
	    }
	}

	switch(endWork){
		case ENDWORK_REBOOT:
		      SoftReset();
		break;
		case ENDWORK_IAP2APP:
		      iap_2_app(APPLICATIONADDRESS);//��ת��APPִ���µĳ���
		break;
		default : break;
	}
	
}


}

void iap_write_appbin(void){//�ѽ��յ�������д��FLASH��
  	uint8_t *pbuf;
	uint16_t len = 0,len1 = 1;		
    uint8_t *  pframe;//���ú����ڻ��������õ�������֡��֡��֡��ʼ��ַ��֡�ֽ�������ʽ���ߺ����������أ�����Ϊ��¼֡��ʼ��ַ�ı���
    uint16_t frameLen = 0;//��¼֡�ֽ����ı���
    S_D07_UNPACK  outpFrame;//����֡��õ������ݴ洢������
	
	
	iapUpgradeFlag = UPGRADE_RECEIVING; //����״̬Ϊ���ݽ�����
	writeIAPUpgradeFlag(iapUpgradeFlag);//��������״̬��־Ϊ������

	while(1){
		{//�ȴ�����֡
			
            pbuf = get_rebuff(&len);
            DelayUs(10000);//�յ���֡��֡�ļ��Ӧ�ô���2���ĸ���ʱʱ��
            pbuf = get_rebuff(&len1);
            if((len == len1) && (len>=12))
            {
				DEBUG("�յ����ݣ�");
               if(0 == get_d07_first_valid_frame(pbuf,len,&pframe,&frameLen))
                {
                  if( 0 ==  unpack_d07_frame(pframe,frameLen,&outpFrame))
            				{
            				  //�õ�֡��Ϣ��Ĵ���
							  DEBUG("����ɹ���ʼ����");
                               listen(&outpFrame);
            				}
							#ifdef __DEBUG__
							else{
								DEBUG("���ʧ��");
							}
							#endif
                }
				#ifdef __DEBUG__
				else{
					DEBUG("û���ҵ�������֡");
				}
				#endif
                clean_rebuff();
            }
        }
    }
	
}


int NeedToUpgrade(){ //�Ƿ���Ҫ�ȴ������µĳ���0����Ҫ��1��Ҫ��
		iapUpgradeFlag = getIAPUpgradeFlag();
		appStateFlag = getAPPState();
		#ifdef __DEBUG__
		DEBUG("��һ���������̽��е���");
		switch(iapUpgradeFlag){
			case  UPGRADE_FREE :
			DEBUG("����");
			break;
			case  UPGRADE_REQUEST :
			DEBUG("����������");
			break;
			case  UPGRADE_RECEIVING :
			DEBUG("������������");
			break;
			case  UPGRADE_RECEIVE_COMPLETE :
			DEBUG("�������������");
			break;
			case   UPGRADE_COPYING :
			DEBUG("������������");
			break;
			default : DEBUG("δ֪״̬��0x%02X",iapUpgradeFlag);break;
		}
		DEBUG("APP�Ŀ������Ϊ��");
			switch(appStateFlag){
			case  APP_NOT_AVAILABLE :
			DEBUG("APP��������");
			break;
			case  APP_AVAILABLE :
			DEBUG("APP������");
			break;
			default : DEBUG("δ֪״̬��0x%02X",iapUpgradeFlag);break;
		}
		#endif
		if( (iapUpgradeFlag == UPGRADE_REQUEST)||(appStateFlag == APP_NOT_AVAILABLE) ){
			return 1;
		}else{
			return 0;
		}
	}


//��ʼ�����
void GpioConfig(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = PB_LED_LINK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = PB_LED_ERR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

    GPIO_ResetBits(GPIOB, PB_LED_LINK);//�ϵ�ʱĬ���Ǹߵ�ƽ��ʵ����Ҫ�͵�ƽ
}

//������ָʾ��
void UpgradeCueLight(void){
	GPIO_SetBits(GPIOB, PB_LED_ERR);
}

#endif



int main(void){
	#if 0 //��ʱ������ԣ�1ms��ʱ����0.3us�����
	while(1){
		
		DelayUs(1000);
		
	}
	#endif
#if IAP_iap
    GpioConfig();
	RS485_Config(getBaudRate());
	*(PileAddress *)benjiAddress = getAddress();
	#ifdef __DEBUG__
	USART_Config();
	DEBUG("\nthis is iap\n");
	#endif
	
  if(NeedToUpgrade()){
	    UpgradeCueLight();//��������ָʾ�� //��˸�����ֹ���״̬
		iap_write_appbin();//��ʼ��������
	}else{
		iap_2_app(APPLICATIONADDRESS);//��ת��ҵ�����
  }
#endif

#if WDG_CESHI

#if 0
int i=0;
	USART_Config();
printf("\nopen IWDG\n");
// IWDG 1s ��ʱ���,���㷽����.c�ļ���
IWDG_Config(IWDG_Prescaler_64 ,625);
while(1){
if(i >= 1){
printf("\ni>=1\n");
while(1){}
}
DelayUs(500000);
printf("\nfeed\n");
IWDG_Feed();
i++;
}
#else
int i =0;
USART_Config();
	// ��ʼ��WWDG�����ü�������ʼֵ�������ϴ���ֵ������WWDG��ʹ����ǰ�����ж�
	WWDG_Config(0X7F, 0X5F, WWDG_Prescaler_8);
printf("\n open wwdg \n");
while(1){
	i++;
if(i<=5){
DelayUs(30000);
printf("\nwei wwdg\n");
WWDG_Feed();
}
else{
	i=6;
printf("\nnot wei wwdg\n");
DelayUs(50000);

}
}
#endif
#endif

}

/*********************************************END OF FILE**********************/


