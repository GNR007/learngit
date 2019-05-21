/**
  ******************************************************************************
  * @file    main.c
  * @author  YunQing
  * @version V1.0
  * @date    2019-02-20
  * @brief   测试IAP
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



#define WDG_CESHI 0 //看门狗测试
#define IAP_iap 1 //IAP测试，iap端


typedef struct {
		uint32_t dataIdentification;
		uint16_t frameNumber;
		uint16_t frameToal;
}Determine;//判定一帧


#define ID_REBOOT (0x00000005)
#define ID_EXECUTION_UPGRADE (0x00000009) //执行升级的数据标识
#define ID_UPGRADE_SWITCHMODE (0x00000007) //切换升级模式的帧数据标识
#define ID_UPGRADEDATA 8 //升级数据帧的数据标识
//#define ID_UPGRADEREQUEST 9 //执行升级的数据标识


#define ENDWORK_REBOOT (1)
#define ENDWORK_IAP2APP (2)

//指示灯的引脚定义
#define PB_LED_LINK	GPIO_Pin_4  //连接灯
#define PB_LED_ERR		GPIO_Pin_5  //升级指示灯


#define __DEBUG___
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("File: "__FILE__", Line: %d: "format"\n", __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif

typedef struct{char a[6];} *pAdress6 ;

uint8_t iapUpgradeFlag = 0;
uint8_t appStateFlag = 0; //0表示APP区的程序不可用，1表示可用



#if IAP_iap

#define FRAMEBUFSIZE 216 //帧缓冲区的大小

#define CTRL_UPGRADEDATA_LASTONR 0x14 //带有升级数据的帧的控制码，最后一帧的控制码
#define CTRL_UPGRADEDATA_NOTLAST 0x34 //带有升级数据的帧的控制码，非最后一帧的控制码
#define ID_UPGRADEDATA 8 //升级数据帧的数据标识
#define INDEX_UPGRADEFRAMENUM 4 //升级数据是多帧传输的，帧中标识帧序号的俩个字节的起始字节的位置
#define INDEX_UPGRADEFRAMETOTALNUM 6 //总序号所在缓冲区位置
#define INDEX_UPGRADEFRAMECRC 136 //CRC校验所在位置
#define LEN_CRCDATA 132 //CRC校验数据的字节数
#define INDEX_UPGRADEDATA 8 //升级数据起始位置
#define CTRL_NORMALRESPONSE_UPGRADEFRAME 0x94 //升级帧从站正常应答的控制码
#define CTRL_ABNORMALRESPONSE_UPGRADEFRAME 0xD4 //升级帧从站异常应答的控制码
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


bool cmpStr(const char * str1 ,const char * str2){//比较字符串的前ADDRESSLEN个字符是否相等
	int i;
	for(i=0;i<ADDRESSLEN;i++){
		if(str1[i] != str2[i]){
			return false;
		}
	}
	return true;
}


/**/void SoftReset(void){//重启
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}



#if IAP_iap

void iap_2_app(uint32_t appxAddr){//从IAP跳转到APP
    pFunction Jump_To_App;
    uint32_t JumpAddress;
	if( 0x20000000 == ((*((__IO uint32_t*)appxAddr)) & 0x2FFE0000) ){//检查栈顶指针是否正确
		
		//获取复位中断服务函数的地址
		JumpAddress = * ((__IO uint32_t * ) (appxAddr + 4));
		
		//把地址强转成函数指针
		Jump_To_App = (pFunction)JumpAddress;
		
		//设置主堆栈指针
		__set_MSP(*(__IO uint32_t*) appxAddr);
		
		//执行复位函数
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
	
    S_D07_PACK_FRAME framePackBuf;//封装回复帧用到的信息存储在这里后，调用封装函数封帧
	
    uint8_t frameBuf[FRAMEBUFSIZE]={0}; //帧的缓冲区
	uint32_t frameLen = 0; //帧的长度
	uint16_t crc=0; //升级帧的CRC校验
	uint16_t sumCRC = 0; //自己求的CRC

    //关于帧的三个变量
	static Determine oldFrame = {ID_UPGRADE_SWITCHMODE , 0 , 0};//初始化时上一帧为切换升级模式帧
    static Determine waitingFrame = {ID_UPGRADEDATA , 0 , 0};//等待的帧为数据帧的第一帧
    Determine receivedFrame = {0,0,0}; //收到的帧
	//关于帧的常量
	const Determine rebootFrame = {ID_REBOOT,0,0};
	
	bool executingResults; //指令的执行结果
	uint8_t errorNumber; //错误编号
	uint8_t endWork = 0; //结束时要进行的操作
	
	if(!pFrame){
		DEBUG("参数指针为空,结束处理");
		return;
	}
	
//地址校验
	if((cmpStr (pFrame->address,benjiAddress) == false) && (cmpStr( pFrame->address,BroadcastAddress) == false)){
		DEBUG("\n地址错误");
		return;
	}
//控制码校验
	if(  ( (CTRL_UPGRADEDATA_NOTLAST != pFrame->ctrl_c)  ) && (CTRL_UPGRADEDATA_LASTONR != pFrame->ctrl_c) ){//升级帧的控制码，最后一帧为0x14，其他为0x34
		DEBUG("\n控制码错误");
		//回复异常（）goto
		executingResults = false;
		errorNumber = 1;
		goto END;
	}

{
//刷新收到的帧
receivedFrame.dataIdentification = pFrame->ruler_id ; //收到的帧的数据标识
if(ID_UPGRADEDATA == receivedFrame.dataIdentification){//如果收到的帧为数据帧
	receivedFrame.frameNumber = (*((uint16_t * )(&(pFrame->aucDataTmp[INDEX_UPGRADEFRAMENUM]))));
	receivedFrame.frameToal = (*((uint16_t * )(&(pFrame->aucDataTmp[INDEX_UPGRADEFRAMETOTALNUM]))));
	if( (receivedFrame.frameNumber == 0) && (waitingFrame.dataIdentification == ID_UPGRADEDATA) && (waitingFrame.frameToal == 0) ){//如果等待的帧是数据帧的第一帧，更新等待帧的帧总数
		waitingFrame.frameToal = receivedFrame.frameToal;
	}
}



	if(frameEqualsTheOperation(&receivedFrame,&waitingFrame)){ //如果收到的帧为所等待的帧
		//处理()；
		{	
		   if(receivedFrame.dataIdentification == ID_UPGRADEDATA ){//数据帧
		         //写入FLASH（）
				crc = (*(uint16_t * )(&pFrame->aucDataTmp[INDEX_UPGRADEFRAMECRC]));
			 
				sumCRC = crc16((char *)(pFrame->aucDataTmp + INDEX_UPGRADEFRAMENUM),LEN_CRCDATA);

					if(crc != sumCRC){
						DEBUG("\n crc error\n");
						//回复异常（）goto
						executingResults = false;
		                errorNumber = 2;
						goto END;
					}	
					
				writeFlash(receivedFrame.frameNumber,receivedFrame.frameToal,&(pFrame->aucDataTmp[INDEX_UPGRADEDATA]));
				if(receivedFrame.frameNumber == receivedFrame.frameToal){
					iapUpgradeFlag = UPGRADE_RECEIVE_COMPLETE;
					writeIAPUpgradeFlag(iapUpgradeFlag);//升级数据接收完成，更新升级标志
				}
				//回复正常（）
				executingResults = true;
		    }else if(receivedFrame.dataIdentification == ID_EXECUTION_UPGRADE) {//执行命令的命令帧
					//回复正常（）****
					iapUpgradeFlag = UPGRADE_COPYING;
					writeIAPUpgradeFlag(iapUpgradeFlag); //更新标志位为 ： 拷贝程序到执行区
					appStateFlag = APP_NOT_AVAILABLE;
					writeAPPStateFlag(appStateFlag); //更新标志位为 ： APP区不可用
					
					CopyFromBufToRunArea();
					
					iapUpgradeFlag = UPGRADE_FREE;
					writeIAPUpgradeFlag(iapUpgradeFlag); //更新标志位为 ：空闲
					appStateFlag = APP_AVAILABLE;
					writeAPPStateFlag(appStateFlag); //更新标志位为 ：APP区可用
					
					executingResults = true;
					endWork = ENDWORK_IAP2APP;
					//goto END;
		    }
			
		}
		//更新,等待和上一帧()；
		//上一帧 = 等待的帧

		oldFrame = waitingFrame;
		//等待的帧 = 下一帧
		if(receivedFrame.dataIdentification == ID_UPGRADEDATA ){//数据帧
			if(receivedFrame.frameNumber == receivedFrame.frameToal && (receivedFrame.frameToal != 0)){//如果为最后一帧
				waitingFrame.dataIdentification = ID_EXECUTION_UPGRADE;
				waitingFrame.frameNumber = 0;
				waitingFrame.frameToal = 0;
			}else if(receivedFrame.frameNumber < receivedFrame.frameToal){//不是最后一帧
				waitingFrame.frameNumber++;//更新等待的帧为下一条数据帧
			}

		}
	}
	else if( frameEqualsTheOperation(&receivedFrame,&oldFrame) ){ //如果收到的帧为上一帧
		//回复正常
		executingResults = true;
	}
	else if ( frameEqualsTheOperation(&receivedFrame,&rebootFrame) ){ //如果收到的帧为重启帧
		//回复正常*******
		executingResults = true;
		endWork = ENDWORK_REBOOT;
		//goto END;
	}
	else{
		//回复异常（）
		executingResults = false;
		errorNumber = 1;
		
	}
}
//处理回复
{
END:
	if(executingResults == true){//正常
		framePackBuf.ctrl_C = 0x94;
        framePackBuf.data_L = 0x0;
	}
    else{//异常
		framePackBuf.ctrl_C = 0xD4;
        framePackBuf.data_L = 0x1;
        framePackBuf.data[0]= errorNumber;
	}

	*(pAdress6)framePackBuf.Address_receiver = *(pAdress6)benjiAddress;
	//打包
	if(cmpStr( pFrame->address,BroadcastAddress) == false){//不是广播地址时给予回复
	    if(0 == pack_d07_frame_by_data(&framePackBuf, frameBuf, &frameLen)){
	//发送
		   RS485_SendStr_length(frameBuf,frameLen);	
	    }else{
		   printf("\n打包失败\n");
	    }
	}

	switch(endWork){
		case ENDWORK_REBOOT:
		      SoftReset();
		break;
		case ENDWORK_IAP2APP:
		      iap_2_app(APPLICATIONADDRESS);//跳转到APP执行新的程序
		break;
		default : break;
	}
	
}


}

void iap_write_appbin(void){//把接收到的数据写到FLASH中
  	uint8_t *pbuf;
	uint16_t len = 0,len1 = 1;		
    uint8_t *  pframe;//调用函数在缓冲区中拿到完整的帧，帧以帧起始地址和帧字节数的形式，走函数参数返回，这里为记录帧起始地址的变量
    uint16_t frameLen = 0;//记录帧字节数的变量
    S_D07_UNPACK  outpFrame;//解析帧后得到的数据存储在这里
	
	
	iapUpgradeFlag = UPGRADE_RECEIVING; //升级状态为数据接收中
	writeIAPUpgradeFlag(iapUpgradeFlag);//更新升级状态标志为升级中

	while(1){
		{//等待数据帧
			
            pbuf = get_rebuff(&len);
            DelayUs(10000);//收到的帧与帧的间隔应该大于2倍的该延时时间
            pbuf = get_rebuff(&len1);
            if((len == len1) && (len>=12))
            {
				DEBUG("收到数据：");
               if(0 == get_d07_first_valid_frame(pbuf,len,&pframe,&frameLen))
                {
                  if( 0 ==  unpack_d07_frame(pframe,frameLen,&outpFrame))
            				{
            				  //拿到帧信息后的处理
							  DEBUG("解包成功开始处理");
                               listen(&outpFrame);
            				}
							#ifdef __DEBUG__
							else{
								DEBUG("解包失败");
							}
							#endif
                }
				#ifdef __DEBUG__
				else{
					DEBUG("没有找到完整的帧");
				}
				#endif
                clean_rebuff();
            }
        }
    }
	
}


int NeedToUpgrade(){ //是否需要等待接收新的程序，0不需要，1需要。
		iapUpgradeFlag = getIAPUpgradeFlag();
		appStateFlag = getAPPState();
		#ifdef __DEBUG__
		DEBUG("上一次升级过程进行到：");
		switch(iapUpgradeFlag){
			case  UPGRADE_FREE :
			DEBUG("空闲");
			break;
			case  UPGRADE_REQUEST :
			DEBUG("有升级请求");
			break;
			case  UPGRADE_RECEIVING :
			DEBUG("升级包接收中");
			break;
			case  UPGRADE_RECEIVE_COMPLETE :
			DEBUG("升级包接收完成");
			break;
			case   UPGRADE_COPYING :
			DEBUG("升级包拷贝中");
			break;
			default : DEBUG("未知状态：0x%02X",iapUpgradeFlag);break;
		}
		DEBUG("APP的可用情况为：");
			switch(appStateFlag){
			case  APP_NOT_AVAILABLE :
			DEBUG("APP区不可用");
			break;
			case  APP_AVAILABLE :
			DEBUG("APP区可用");
			break;
			default : DEBUG("未知状态：0x%02X",iapUpgradeFlag);break;
		}
		#endif
		if( (iapUpgradeFlag == UPGRADE_REQUEST)||(appStateFlag == APP_NOT_AVAILABLE) ){
			return 1;
		}else{
			return 0;
		}
	}


//初始化语句
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

    GPIO_ResetBits(GPIOB, PB_LED_LINK);//上电时默认是高电平，实际需要低电平
}

//亮升级指示灯
void UpgradeCueLight(void){
	GPIO_SetBits(GPIOB, PB_LED_ERR);
}

#endif



int main(void){
	#if 0 //延时程序测试，1ms延时，有0.3us的误差
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
	    UpgradeCueLight();//开启升级指示灯 //闪烁，区分故障状态
		iap_write_appbin();//开始升级引导
	}else{
		iap_2_app(APPLICATIONADDRESS);//跳转到业务程序
  }
#endif

#if WDG_CESHI

#if 0
int i=0;
	USART_Config();
printf("\nopen IWDG\n");
// IWDG 1s 超时溢出,计算方法见.c文件。
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
	// 初始化WWDG：配置计数器初始值，配置上窗口值，启动WWDG，使能提前唤醒中断
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


