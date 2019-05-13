#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_GeneralTim.h"  
#include "./meter/elec_meter.h"
#include "./usart/bsp_usart.h"
#include "./internal_flash/bsp_internal_flash.h" 
#include "./485/bsp_485.h"
#include "./EXTI/bsp_exti.h"
#include "./iwdg/bsp_iwdg.h"
#include "./wwdg/bsp_wwdg.h"
#include "./SysTick/bsp_SysTick.h"
#include "core_cm3.h"
#include "misc.h"
#include "./dlt645_07/dlt645_07_api.h"
#include <string.h>
#include "./b232/bsp_232.h"

typedef uint8_t bool;
#define true 1
#define false 0

#define WAITING_FOR_PLUGIN 2000	// 单位：毫秒
#define DEFAULT_PWM_RATIO 52	// 设置53，出了55，单位：百分之一 占空比
//#define DEFAULT_RATIO 100 //单位：千分之一 默认占空比 初始化参数列表中的参数用的

#define PB_LED_RUN		GPIO_Pin_3    //GPIO_Pin_12
#define PA_AC_CONTACTOR	GPIO_Pin_1
#define PA_CC			GPIO_Pin_4
#define PA_CP_PWM		GPIO_Pin_7
#define PA_CP_DET0		GPIO_Pin_5
#define PB_CP_DET1		GPIO_Pin_0

#define PB_LED_LINK	GPIO_Pin_4  //连接灯
#define PB_LED_ERR		GPIO_Pin_5  //故障灯
#define PB_LED_CHARGING	GPIO_Pin_6  //充电灯
#define PB_STOP		GPIO_Pin_7  //急停检测引脚

#define CHARGING 1
#define FREE 2
#define WAITDRAWGUN 3
#define ERRORS_STOP 4
#define ERRORS_VOL_CUR_FREQ 5
#define UPGRADE 6

#define       IAPLICATIONADDRESS       ((uint32_t)0x08000000) //IAP的起始地址


#define ZFZ 1 //修改中的程序开关宏
#define BAN_CESHI 1 //屏蔽掉电压，电流，频率的保护


#define __DEBUG___
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("File: "__FILE__", Line: %d: "format"\n", __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif

#define VN_LEN 4 //软件版本的字节数

//通信的错误码
#define POWER_EXCEEDS_LIMIT (1) //功率不在允许的范围
#define REFUSE_UPGRADE (2) //拒绝升级请求

#define POWER_MAX (32*220) //7040 占空比53%
#define POWER_MIN (6*220) //1320 占空比10%

#define CUT_PWM (1)
#define CUT_CONTACTOR (2)
#define CUT_CONTACTOR_PWM (3)

#define END_WORK_REBOOT (1)
#define END_WORK_IAP2APP (2)

extern uint32_t timeRecord;//


typedef enum {normalVol,overVol,underVol}VolStatus; //正常，过压，欠压
typedef enum {normalCur,overCur,overCurTimeout}CurStatus; //正常，过流，过流超时
typedef enum {normalFreq,overFreq,underFreq}FreqStatus; //正常，频率过高，频率过低
typedef enum {delay_free,delay_start,delay_flow}FlagDelay; //异常锁止状态标志
typedef enum {canStartCharging,stopCharging}RemotePermissibleChargingStatus;// 命令状态
typedef struct{char a[6];} *pAdress6 ;
typedef void (*pFunction)(void);

typedef struct backstageMessage{
           bool messageStatus;//是否有新的消息
		   uint16_t messageLen;//消息的长度
		   uint8_t * pMessage;//消息的指针
}BackstageMessage;

typedef struct List StatusList; //此句的目的是为了接下来这两个结构体的相互引用
typedef struct {
		uint32_t curRms;  /*电流有效值*/
	    CurStatus   curStatus; //电流的状态
		float        cur2RmsGain;  /*求电流有效值时用到的增益Cur2RmsGain =0.000736207;*/
		int          curMax; /*当前允许的最大电流*/
		void (* GetCurValue)(StatusList * pSystemStatusList); //获取电流的方法
	    void (* currentMonitoring)(StatusList *pSystemStatusList); //工作电流的监控方法
}CURRENT;
struct List{
           uint8_t      cpValue;//CP的采样值
		   uint8_t      workingState_flag;/*4种工作状态，充电中，空闲，待拔枪，故障*/
           bool         scramStatus;/*急停状态*/
           u32          volRms;/*电压有效值*/
		   float        volRmsGain;/*电压值增益VolRmsGain = 0.0009737;*/
           float        volFreq; /*电压频率*/
           RemotePermissibleChargingStatus         remoteStartOrStopFlag;
		       int          vMax; /*过压点*/
		       int          vMin; /*欠压点*/
		       int          curMin; /**/
		       int          freqMax; /*最高允许频率*/
		       int          freqMin; /*最低允许频率*/
		       VolStatus   volStatus; //电压的状态
		       FreqStatus  freqStatus; //频率的状态
			   SettingList parameterList; //FLASH中的记录的参数
					 
					 char zhuzhanAddress[6]; //主控的地址
					 char benjiAddress[6]; //本机的地址
					 char broadcastAddress[6]; //广播地址
					 char developersAddress[6]; //开发者地址
					 //通信有关
					 BackstageMessage messageNotice;


					 uint32_t powerSetInBackground;//后台设置的功率
					 unsigned int percentPWM ; //PWM的百分比 分辨率千分之一
					 uint8_t workingMode ; //充电模式，0 表示自动充电，充电过程自动进行无需干涉。 1 表示命令充电，充电时需要等待后台的充电命令
					 uint32_t powerMax;
					 const uint32_t * pTimeRecord;
					 
					 CURRENT operatingCurrent; //工作时的电流。
					 
} ;
unsigned int * pRatio;//PWM开关函数用的，指向参数列表中的percentPWM
extern uint8_t  scramStatusFlag; 



void DelayUs(uint32_t us){
	volatile uint32_t i;

	for (i = 0; i < us; i++) {
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();//__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}

}

void DelayMs(uint32_t ms){
	DelayUs((1000 * ms));
}

void GpioConfig(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = PB_LED_LINK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	

	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = PB_LED_ERR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = PB_LED_CHARGING;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = PB_STOP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PA_AC_CONTACTOR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = PA_CP_DET0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = PB_CP_DET1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PA_CP_PWM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	GPIO_ResetBits(GPIOB, PB_LED_LINK);//灭连接灯，该引脚PB4为JTAG复用脚的JNTRST,复位后默认置于上拉模式。所以需要灭一下
}


/* ----------------   PWM信号 周期和占空比的计算--------------- */
// ARR ：自动重装载寄存器的值
// CLK_cnt：计数器的时钟，等于 Fck_int / (psc+1) = 72M/(psc+1)
// PWM 信号的周期 T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// 占空比P=CCR/(ARR+1)

void PwmConfig(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR2_Val = 1000;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  PA_CP_PWM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 开启定时器时钟,即内部时钟CK_INT=72M
	GENERAL_TIM_APBxClock_FUN(GENERAL_TIM_CLK,ENABLE);
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period=GENERAL_TIM_Period;	
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= GENERAL_TIM_Prescaler;	
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);

	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// 输出比较通道 2
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OC2Init(GENERAL_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);

	// 使能计数器
	TIM_Cmd(GENERAL_TIM, ENABLE);
}

void PwmRatio(uint8_t ratio){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	if (ratio > 100)
		ratio = 100;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = ratio * 10;
	
	TIM_Cmd(GENERAL_TIM, DISABLE);
	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Disable);
	TIM_OC2Init(GENERAL_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	TIM_Cmd(GENERAL_TIM, ENABLE);
	DelayMs(10);
}

void PwmCtrl(bool enable){
	if (enable) {
		PwmRatio(*pRatio);
	} else {//*pRatio
		PwmRatio(100);
	}
}


void SetLedRun(bool on){
	if (!on) {
		GPIO_SetBits(GPIOB, PB_LED_RUN);
	} else {
		GPIO_ResetBits(GPIOB, PB_LED_RUN);
	}
}

void SetAcContactor(bool on){
	if (on) {
		GPIO_SetBits(GPIOA, PA_AC_CONTACTOR);
	} else {
		GPIO_ResetBits(GPIOA, PA_AC_CONTACTOR);
	}
}




uint8_t GetCpStatus(void){
	uint8_t ret = 0;
	
	uint8_t bit0 = GPIO_ReadInputDataBit(GPIOA, PA_CP_DET0);
	uint8_t bit1 = GPIO_ReadInputDataBit(GPIOB, PB_CP_DET1);
	
	if (bit0 == 0) {
		ret = 6;
	} else if (bit1 == 0) {
		ret = 9;
	} else {
		ret = 12;
	}
	
	return ret;
}
uint8_t GetCpStatusFiltering(void){
	int i;
	uint8_t ret = 0;
	uint8_t bit0;
	uint8_t bit1;
	int bit0TrueValueCnt = 0;
	int bit1TrueValueCnt = 0;
	int samplingNum = 1000;
	
	for (i = 0; i < samplingNum; i++) {
		if (GPIO_ReadInputDataBit(GPIOA, PA_CP_DET0)) {
			bit0TrueValueCnt++;
		}
		if (GPIO_ReadInputDataBit(GPIOB, PB_CP_DET1)) {
			bit1TrueValueCnt++;
		}
		DelayUs(10);
	}
	
	bit0 = (bit0TrueValueCnt > samplingNum*90/100)?1:0;
	bit1 = (bit1TrueValueCnt > samplingNum*50/100)?1:0;
	
	if (bit0 == 0) {
		ret = 6;
	} else if (bit1 == 0) {
		ret = 9;
	} else {
		ret = 12;
	}
	
	return ret;
}





/*###-----------------###*/
/**定时任务**/
typedef enum {  //定时任务的状态
    STOP,            // 停用
    GET_TIMESTAMP,             // 获取时间戳
    DETERMINE,             // 判断，若指定的时间到了，则执行指定的任务
} TIMING_STATUS;

//定时任务的相关信息
typedef struct{
	//时间戳
	uint32_t timeStamp;

	//定时时间
	uint32_t TimingTime;
	//定时状态
	TIMING_STATUS timingStatus;
	
	//时间到后要执行的任务
	void (* TaskHook)(StatusList *);
} TIMED_TASKS;

typedef enum _TASK_LIST  {  
    TASK1,            // 任务1
    TASK2,            // 任务2
    TASK3,            //任务3
	TASK4,            //任务4
     // 添加别的任务
     TASKS_MAX         // 总的可供分配的定时任务的总数
} TASK_LIST; 

void task1(StatusList *pSystemStatusList){
	
}
void task2(StatusList *pSystemStatusList){ //发出停充命令后3秒若仍处在充电状态则断开接触器，跳到等待拔枪状态。
	if(pSystemStatusList->workingState_flag == CHARGING){
		 SetAcContactor(false);
         pSystemStatusList->workingState_flag = WAITDRAWGUN;
	}

}
void task3(StatusList *pSystemStatusList){ //
if(pSystemStatusList->workingState_flag == ERRORS_VOL_CUR_FREQ){
	pSystemStatusList->workingState_flag = WAITDRAWGUN;
}
}
void task4(StatusList *pSystemStatusList){
	pSystemStatusList->operatingCurrent.curStatus = overCurTimeout;
}
TIMED_TASKS timed_task[] = {{0,20,STOP,task1}, \
	{0,3000,STOP,task2}, \
		{0,5000,STOP,task3}, \
		{0,5000,STOP,task4}
};
void PerformTimedTask(TIMED_TASKS * task,StatusList *pSystemStatusList){
	uint32_t timeBuf;
	if(task->timingStatus == STOP){
		return;
	}
	
	timeBuf = *(pSystemStatusList->pTimeRecord);
	switch(task->timingStatus){
		case GET_TIMESTAMP:
		task->timeStamp = timeBuf;
		task->timingStatus = DETERMINE;
		break;
		case DETERMINE:
		if((timeBuf = timeBuf - task->timeStamp) >= task->TimingTime){
			task->TaskHook(pSystemStatusList);
			task->timingStatus = STOP;
		}
		break;
		default :break;
	}
}
void time_proc(StatusList *pSystemStatusList){
	int i = 0;
    for(i = 0;i<TASKS_MAX;i++){
    	PerformTimedTask(timed_task+i,pSystemStatusList);
    }
}


/****/



bool GetScramStatues(void){
	if (0 == GPIO_ReadInputDataBit(GPIOB, PB_STOP) ){
		DelayMs(10);
		if (0 == GPIO_ReadInputDataBit(GPIOB, PB_STOP) ){
			return true;
		}
	}
	return false;
}
void TimConfig(void){
	SysTick_Init();
	return;
}



void SerialConfig(void){
	return;
}

void GetCurrentValue(StatusList * pSystemStatusList){
	u32 cur = 0;
	cur = U24TRANS(MeterRead(RMS_I1));
	//cur = U24TRANS (MeterRead(RMS_I2));  //另外一个通道的电流
    pSystemStatusList->operatingCurrent.curRms  =(0.1 *cur * pSystemStatusList->operatingCurrent.cur2RmsGain);
}
void CurrentMonitoringMethod(StatusList *pSystemStatusList){
	//获取电流值
	
		pSystemStatusList->operatingCurrent.GetCurValue(pSystemStatusList);
		//pSystemStatusList->operatingCurrent.curRms = 100;
	//获取当前允许的最大电流值
		pSystemStatusList->operatingCurrent.curMax = 10 * ((uint32_t) pSystemStatusList->powerSetInBackground /220);
		//pSystemStatusList->operatingCurrent.curMax = 20;
if(CHARGING == pSystemStatusList->workingState_flag){
		if(pSystemStatusList->operatingCurrent.curMax <= 200){//代码可以缩减一下
			if(pSystemStatusList->operatingCurrent.curRms >= (pSystemStatusList->operatingCurrent.curMax  + 20) ){
				
				//第一次进，开启定时任务，5秒后置电流状态为过流超时
				if(pSystemStatusList->operatingCurrent.curStatus == normalCur)
				{
					pSystemStatusList->operatingCurrent.curStatus = overCur;
				    //开启定时任务，你去等着，5秒中后不给你发信号，你就把状态改成过流超时。
				    timed_task[TASK4].timingStatus = GET_TIMESTAMP;
				}
			}else{
				pSystemStatusList->operatingCurrent.curStatus = normalCur;
				//如果定时任务不等于STOP。置为STOP
				if(timed_task[TASK4].timingStatus != STOP){
					timed_task[TASK4].timingStatus = STOP;
				}
			}
		}else if(pSystemStatusList->operatingCurrent.curMax >200 ){
			if(pSystemStatusList->operatingCurrent.curRms >= (pSystemStatusList->operatingCurrent.curMax *1.1) ){

				//第一次进，开启定时任务，5秒后置电流状态为过流超时
				if(pSystemStatusList->operatingCurrent.curStatus == normalCur)
				{
					pSystemStatusList->operatingCurrent.curStatus = overCur;
				    //开启定时任务
				    timed_task[TASK4].timingStatus = GET_TIMESTAMP;
				}
			}else{
				pSystemStatusList->operatingCurrent.curStatus = normalCur;
				//如果定时任务不等于STOP。置为STOP
				if(timed_task[TASK4].timingStatus != STOP){
					timed_task[TASK4].timingStatus = STOP;
				}
			}
		}
}else{
		pSystemStatusList->operatingCurrent.curStatus = normalCur;
	}
}


void SysInit(StatusList * pSystemStatusList){

pSystemStatusList->pTimeRecord = &timeRecord;//参数列表中一个指向时间基准的指针，该基准只读。
/*----底层模块初始化----------*/
	  GpioConfig();
	  PwmConfig();
	  EXTI_Key_Config();
	  TimConfig();
	  SPI_METER_Init();
	  
	//默认配置封装成函数  
GetSetting(&(pSystemStatusList->parameterList));
if(0xff == pSystemStatusList->parameterList.flag_default){//写入默认参数
	  strcpy(pSystemStatusList->parameterList.SN,"kj01");
	  //软件版本号的初始化
	  pSystemStatusList->parameterList.VN[0] = 0x01;//默认硬件版本
	  pSystemStatusList->parameterList.VN[1] = 0x00;
	  pSystemStatusList->parameterList.VN[2] = 0x00;
	  pSystemStatusList->parameterList.VN[3] = 0x00;
	  #if 1
	  //vxn4f0
	  pSystemStatusList->parameterList.PileAddress[0] = '0';//默认桩地址
	  pSystemStatusList->parameterList.PileAddress[1] = 'f';
	  pSystemStatusList->parameterList.PileAddress[2] = '4';
	  pSystemStatusList->parameterList.PileAddress[3] = 'n';
	  pSystemStatusList->parameterList.PileAddress[4] = 'x';
	  pSystemStatusList->parameterList.PileAddress[5] = 'v';
	  #endif

	  pSystemStatusList->parameterList.BaudRate = 9600;//默认波特率
	  pSystemStatusList->parameterList.maxPower = 7040;//默认最大功率
	  
    pSystemStatusList->parameterList.flag_default = 0;
    pSystemStatusList->parameterList.vMax = 2640;/*过压点，正常值的20%*/
    pSystemStatusList->parameterList.vMin = 1870;/*欠压点，正常值的15%*/
    pSystemStatusList->parameterList.curMax = 3520;/*过流点，正常值的1.1倍*/
    pSystemStatusList->parameterList.freqMax = 65;/*最大允许频率*/
    pSystemStatusList->parameterList.freqMin = 45;/*最低允许频率*/
    pSystemStatusList->parameterList.volRmsGain = 0.0009737;
    pSystemStatusList->parameterList.cur2RmsGain =0.000736207;
	
	UpdateSettingList(&(pSystemStatusList->parameterList));
}
USART_Config();	 
RS485_Config(pSystemStatusList->parameterList.BaudRate);//初始化485,入口参数为通信速率。

pSystemStatusList->broadcastAddress[0] = 0x99;
pSystemStatusList->broadcastAddress[1] = 0x99;
pSystemStatusList->broadcastAddress[2] = 0x99;
pSystemStatusList->broadcastAddress[3] = 0x99;
pSystemStatusList->broadcastAddress[4] = 0x99;
pSystemStatusList->broadcastAddress[5] = 0x99;



pSystemStatusList->developersAddress[0] = 0x80;
pSystemStatusList->developersAddress[1] = 0x80;
pSystemStatusList->developersAddress[2] = 0x80;
pSystemStatusList->developersAddress[3] = 0x80;
pSystemStatusList->developersAddress[4] = 0x80;
pSystemStatusList->developersAddress[5] = 0x80;

/*-------------指示灯，接触器，pwm输出初始化-----------------*/
    SetAcContactor(false);/*断开接触器*/
    PwmCtrl(false);/*关闭PWM输出*/
/*-----------------------------------------------------------*/


/*---------------默认配置参数------------------------------*/
  
	pSystemStatusList->vMax = pSystemStatusList->parameterList.vMax;
    pSystemStatusList->vMin = pSystemStatusList->parameterList.vMin;
    pSystemStatusList->operatingCurrent.curMax = pSystemStatusList->parameterList.curMax;
    pSystemStatusList->freqMax = pSystemStatusList->parameterList.freqMax;
    pSystemStatusList->freqMin = pSystemStatusList->parameterList.freqMin;
    
   
    MeterWrite(ANAEN,0x3f);  /*打开电表I2测量通道*/
    pSystemStatusList->volRmsGain = pSystemStatusList->parameterList.volRmsGain;
    MeterWrite(URMSOFFSET,0x0A);  /*电压有效值偏置*/
    pSystemStatusList->operatingCurrent.cur2RmsGain = pSystemStatusList->parameterList.cur2RmsGain;
    MeterWrite(I2RMSOFFSET,0x02); /*电流2有效值偏置*/
  
/************与异常有关的********************/
    scramStatusFlag = GetScramStatues(); /*急停状态初始化*/
/********************************************/

/***********与状态切换有关的*****************/
    pSystemStatusList->workingState_flag = FREE;
/********************************************/


#if BAN_CESHI
/*电压电流频率初始化为正常*/
    pSystemStatusList->volStatus = normalVol;
    //pSystemStatusList->curStatus = normalCur;
    pSystemStatusList->freqStatus = normalFreq;

#endif

/*--------与后台的通信有关的------------*/

		
        pSystemStatusList->remoteStartOrStopFlag = stopCharging;

		*(pAdress6)pSystemStatusList->benjiAddress = *(pAdress6)pSystemStatusList->parameterList.PileAddress;
		*(pAdress6)pSystemStatusList->zhuzhanAddress = *(pAdress6)pSystemStatusList->benjiAddress;
		
		
		//获取工作模式
		readWorkingMode(&(pSystemStatusList->workingMode));
		//初始化PWM的占空比
		pSystemStatusList->powerMax = pSystemStatusList->parameterList.maxPower;
		
		pSystemStatusList->percentPWM = DEFAULT_PWM_RATIO;
		pSystemStatusList->powerSetInBackground = pSystemStatusList->powerMax;
		pRatio = & (pSystemStatusList->percentPWM);
		
		pSystemStatusList->operatingCurrent.GetCurValue =  GetCurrentValue;
		pSystemStatusList->operatingCurrent.currentMonitoring = CurrentMonitoringMethod;
		pSystemStatusList->operatingCurrent.curStatus = normalCur;
		
/*------------------------------*/

}





void GetVoltageValue(StatusList * pSystemStatusList){
	u32 vol = 0;
	vol = U24TRANS(MeterRead(RMS_U));
	pSystemStatusList->volRms =  (vol * pSystemStatusList->volRmsGain);
}



int GetQuencyValue(void){
	//pSystemStatusList->Reg = MeterRead(MODULEEN);
	return 921000/2/((float)(MeterRead(FREQ_U)));
}



bool GetBackstageStatues(void){
	return true;
}






void GetVolStatus(StatusList * pSystemStatusList){
if(CHARGING == pSystemStatusList->workingState_flag){
	if(pSystemStatusList->volRms >= pSystemStatusList->vMax){
		pSystemStatusList->volStatus = overVol;
	}
	else if(pSystemStatusList->volRms <= pSystemStatusList->vMin){
		pSystemStatusList->volStatus = underVol;
	}
	else{
		pSystemStatusList->volStatus = normalVol;
	}
}
else{
	pSystemStatusList->volStatus = normalVol;
}
}


void GetFreqStatus(StatusList * pSystemStatusList){
if(CHARGING == pSystemStatusList->workingState_flag){
	if(pSystemStatusList->volFreq >= pSystemStatusList->freqMax){
		pSystemStatusList->freqStatus = overFreq;
	}
	else if(pSystemStatusList->volFreq <= pSystemStatusList->freqMin){
		pSystemStatusList->freqStatus = underFreq;
	}
	else{
		pSystemStatusList->freqStatus = normalFreq;
	}
}
else{
	pSystemStatusList->freqStatus = normalFreq;
}
}
void GetMessageStatus(StatusList * pSystemStatusList){
	uint16_t length = 0 ;//
	static uint16_t oldLength = 0 ;//
	uint8_t * pbuf ;//
	static uint8_t flag_first = 0 ;//
	if(!pSystemStatusList){//
		pSystemStatusList->messageNotice.messageStatus = false;
		return;
	}
		if( 0 == flag_first ){
			flag_first = 1 ;
			pbuf = get_rebuff(&length);
			oldLength = length ;
			//开启定时任务
			timed_task[TASK1].timingStatus = GET_TIMESTAMP;
			//printf("开启定时任务\n");
		}else{
			if( timed_task[TASK1].timingStatus == STOP){
				//printf("时间到\n");
				pbuf = get_rebuff(&length);
				if((length == oldLength) &&(oldLength >= 12)){
					pSystemStatusList->messageNotice.messageStatus = true ;
					pSystemStatusList->messageNotice.messageLen = length ;
					pSystemStatusList->messageNotice.pMessage = pbuf ;
				}else{
					oldLength = length ;
					pSystemStatusList->messageNotice.messageStatus = false;
				}
				flag_first = 0;
				return;
			}else{
				//printf("时间没到\n");
				pSystemStatusList->messageNotice.messageStatus = false;
				return ;
			}
		}
	
}


void GetSystemStatus(StatusList * pSystemStatusList){
	
	pSystemStatusList->cpValue = GetCpStatusFiltering();
	pSystemStatusList->scramStatus = scramStatusFlag;

	pSystemStatusList->volFreq = GetQuencyValue();
	GetVoltageValue(pSystemStatusList);

	GetVolStatus(pSystemStatusList);
	GetFreqStatus(pSystemStatusList);
	
	pSystemStatusList->operatingCurrent.currentMonitoring(pSystemStatusList);
	
	time_proc(pSystemStatusList);
	//获取通信状态
	GetMessageStatus(pSystemStatusList);
}


void ExceptionHanding(StatusList * pSystemStatusList){
/*急停按下的处理*/
    if( true == pSystemStatusList->scramStatus ){/*急停按下*/
		SetAcContactor(false);/*断开接触器*/
		PwmCtrl(false);/*关掉PWM输出*/
		pSystemStatusList->workingState_flag = ERRORS_STOP;		
	}

} 



bool ChargeCondition(StatusList * pSystemStatusList){//空闲跳转到充电状态的条件在这里判断
	if(6 == pSystemStatusList->cpValue || 9 == pSystemStatusList->cpValue ){/*桩处在连接状态*/
		if(false == pSystemStatusList->scramStatus){/*急停没有按下*/
			
			if(pSystemStatusList->workingMode == 0){//工作模式为自动模式
				    return true;
			}else{//工作模式为命令模式

			    if(canStartCharging == pSystemStatusList->remoteStartOrStopFlag){/*后台允许充电*/
			    	return true;/*可以开始充电*/
			    }
			}
			
		}
	}
	return false;	
}

void StartCharging(StatusList * pSystemStatusList){
	if(ChargeCondition(pSystemStatusList)== true ){
		PwmCtrl(true);/*启动PWM*/
		if(6 == pSystemStatusList->cpValue){
					SetAcContactor(true);/*闭合接触器*/
					pSystemStatusList->workingState_flag = CHARGING;
		}
	}else{
		PwmCtrl(false);
		SetAcContactor(false);/*断开接触器*/
		pSystemStatusList->workingState_flag = FREE;
	}
}



bool StopChargeCondition(StatusList * pSystemStatusList){//充电状态跳转到别的状态的条件在这里判断
	 if(pSystemStatusList->cpValue == 6){/*连接状态*/
		#if BAN_CESHI
		 if(overCurTimeout != pSystemStatusList->operatingCurrent.curStatus ){/*过流点*/
		#endif
		
		          if(pSystemStatusList->workingMode == 1 ){//命令模式
					if(canStartCharging == pSystemStatusList->remoteStartOrStopFlag){/*后台允许充电*/
						return 0;
					}else{
						return CUT_PWM;
					}	
				  }else{//自动模式
					    return 0;
				  }
					
	#if BAN_CESHI	
			}else{
				return CUT_CONTACTOR_PWM;
			}
	#endif
	 }else{
		 return CUT_CONTACTOR;
	 }
	 
	 
	
}

int setPercent(unsigned int num) {
		static unsigned int oldPercent;
		unsigned int newPercent;
		newPercent =  num;
		//参数安全检查
		if(newPercent > 100){
			return -1;
		}
	
		if(newPercent != oldPercent){
			oldPercent = newPercent;
			PwmRatio(newPercent);
		}
		return 0 ;	
}

void StopCharging (StatusList * pSystemStatusList){
	setPercent(pSystemStatusList->percentPWM);//调节PWM
	switch(StopChargeCondition(pSystemStatusList)){
		case  CUT_CONTACTOR :
		SetAcContactor(false);
		pSystemStatusList->workingState_flag = WAITDRAWGUN;
		break;
		case  CUT_PWM :
		PwmCtrl(false);/*关掉PWM输出*/
		//打开定时器
		//若时间到则切断接触器，跳转到待拔枪。
		if(timed_task[TASK2].timingStatus == STOP){
			timed_task[TASK2].timingStatus = GET_TIMESTAMP;
		}
		break;
		case  CUT_CONTACTOR_PWM :
		SetAcContactor(false);/*断开接触器*/
		PwmCtrl(false);/*关掉PWM输出*/
		pSystemStatusList->workingState_flag = ERRORS_VOL_CUR_FREQ;
		break;
		default : break;
	}
}


void WaitDrawGun(StatusList * pSystemStatusList){
	static uint8_t waitflag=0;
	switch (waitflag){
		case 0 :
		pSystemStatusList->remoteStartOrStopFlag = stopCharging;
		waitflag = 1;
		break;
		case 1 :
			if(12 == pSystemStatusList->cpValue){
		        pSystemStatusList->workingState_flag = FREE;
		
		        pSystemStatusList->remoteStartOrStopFlag = stopCharging;
				waitflag=0;
	        }
	        if(pSystemStatusList->remoteStartOrStopFlag == canStartCharging){
		        pSystemStatusList->workingState_flag = FREE;
				waitflag=0;
	        }
			
	    break;
	
	}

}

void WaitErrorsEnd(StatusList * pSystemStatusList){
	if( false == pSystemStatusList->scramStatus ){/*急停松开*/
		if(timed_task[TASK3].timingStatus == STOP){
			pSystemStatusList->workingState_flag = WAITDRAWGUN;
		}else{
			pSystemStatusList->workingState_flag = ERRORS_VOL_CUR_FREQ;
		}			
	}
	return;
}


void errorsVolCurFreqHandle(StatusList * pSystemStatusList){
		if(timed_task[TASK3].timingStatus == STOP){
			timed_task[TASK3].timingStatus = GET_TIMESTAMP;
		}
	return;
}

void WorkingStateChange(StatusList * pSystemStatusList){
switch(pSystemStatusList->workingState_flag)//状态条件放函数外
{
  case CHARGING:
             StopCharging(pSystemStatusList);
            break;
  case FREE:
             StartCharging(pSystemStatusList);
            break;
  case WAITDRAWGUN:
			 WaitDrawGun(pSystemStatusList);
			break;
  case ERRORS_STOP:
       WaitErrorsEnd(pSystemStatusList);
			break;
	#if BAN_CESHI
  case ERRORS_VOL_CUR_FREQ:
       errorsVolCurFreqHandle(pSystemStatusList);
	    break;
	#endif
  default: break;
}	
}




void DisplayProcessing(StatusList * pSystemStatusList){
	if((6 == pSystemStatusList->cpValue )||(9 == pSystemStatusList->cpValue)){
		
		GPIO_SetBits(GPIOB, PB_LED_LINK); /*亮连接灯*/
		//printf("\r\ncpValue:%d\r\n",pSystemStatusList->cpValue);//请不要删除或屏蔽这条语句
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_LINK); /*灭*/
		//printf("\r\ncpValue:%d\r\n",pSystemStatusList->cpValue);//请不要删除或屏蔽这条语句
	}
	
	if(true == pSystemStatusList->scramStatus || ERRORS_VOL_CUR_FREQ ==  pSystemStatusList->workingState_flag){
		
		GPIO_SetBits(GPIOB, PB_LED_ERR);   /*亮故障灯*/
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_ERR); /*灭*/
	}
	
	if( CHARGING == pSystemStatusList->workingState_flag ){
		
		GPIO_SetBits(GPIOB, PB_LED_CHARGING);  /*亮充电灯*/
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_CHARGING);  /*灭*/
	}
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

void SoftReset(void){
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}


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

	// GB T 18487.1-2015 表A.1 充电设施产生的占空比与充电电流映射关系
	// 10% <= D <= 85%   Imax = D*100*0.6
	// 85% < D <= 90%    Imax = (D*100-64)，且 Imax <= 63
int setCurrent(unsigned int maxCur) {
		int percent = 0;
		
		if (maxCur >= 6 && maxCur <= 51) {
			percent = (int)(maxCur/0.6);
		} else if (maxCur > 52.5 && maxCur <= 63) {
			percent = (int)(maxCur/2.5+64);
		} else {
			//
			//printf("cur = %d not valid\n", maxCur);
			return -1;
		}

		//printf("AC charge set pwm %d A, percent %d\n", maxCur, percent);
		return percent;
	}
	
//功率转换成百分比
uint32_t power2percent(uint32_t power_ls){
		unsigned int cur;
		//int percent_ls;
		//获取电流
		 cur= (int)(power_ls /220);
		//获取百分比
		 return setCurrent(cur);
		 
}


void listen(S_D07_UNPACK * pFrame, StatusList * pSystemStatusList){
	
	S_D07_PACK_FRAME framePackBuf;//打包用的信息结构体
	
  uint8_t frameBuf[D07_DATA_MAX + D07_FRAME_LEN_MIN]={0}; //帧的容器
	
	uint32_t frameLen = 0;
	
	uint32_t power_ls;//临时存储功率
	bool executingResults; //指令的执行结果
	uint8_t errorNumber; //错误编号
    uint8_t endWork = 0;
	uint32_t baudRate_buf = 0;
	uint32_t maxPower_buf = 0;
	if((cmpStr( pFrame->address,pSystemStatusList->benjiAddress) == true) || \
	((cmpStr( pFrame->address,pSystemStatusList->broadcastAddress) == true) && ((pFrame->ruler_id == 7)||(pFrame->ruler_id == 0xf)) ) || \
	(cmpStr( pFrame->address,pSystemStatusList->developersAddress) == true)){//地址为本机地址 或者广播地址且控制码为15 或 7
		//printf("%02x\n",pFrame->ctrl_c);
		//printf("%02x\n",pFrame->ruler_id);
		switch(pFrame->ctrl_c){
			
	        case 0x11://读
	            switch(pFrame->ruler_id){//根据数据标识如何打包
	        		      case 1://获取充电桩状态
								{//封装成函数
						          
							        framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 12;
                                    *(uint32_t*)framePackBuf.data= 0x00000001;
                                    				        
					                
									 
									 
									 
									 
								       //封装数据域
							         //data[0]
							         if((6 == pSystemStatusList->cpValue )||(9 == pSystemStatusList->cpValue)){//bit7置1//bit7=1表示插枪状态
							             //framePackBuf.data[0+ 4] |= (0x1<<7);
								           framePackBuf.data[0 + 4] |= 0x80;
							         }else{//bit7置0//bit7 =0 表示未插枪
							             //framePackBuf.data[0+ 4] &= (~(0x01<<7));
								           framePackBuf.data[0+ 4] &= 0x7f;
							         }
							         framePackBuf.data[2+ 4] = 0;//故障为0 表示无故障
							         framePackBuf.data[0+ 4] &= 0xf0;//清零bit0~3
							         switch(pSystemStatusList->workingState_flag)
                                           {
                                             case CHARGING:    //bit0~3赋值为0x01
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x01
								       				                          framePackBuf.data[0+ 4]  |= 0x01;
                                                       break;
                                             case FREE:    //bit0~3赋值为0x00
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x00
								       				                         framePackBuf.data[0+ 4] &= 0xf0;
                                                       break;
                                             case WAITDRAWGUN:    //bit0~3赋值为0x02
                                           			       //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x02
								       				                         framePackBuf.data[0+ 4]  |= 0x02;
                                           			       break;
                                             case ERRORS_STOP:    //bit0~3赋值为0x03
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x03
								       				                          framePackBuf.data[0+ 4]  |= 0x03;
								       				                          framePackBuf.data[2+ 4] = 1;
                                           			        break;
                                           	#if BAN_CESHI
                                             case ERRORS_VOL_CUR_FREQ:    //bit0~3赋值为0x03
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x03
								       				                          framePackBuf.data[0+4]  |= 0x03;
								       				                          framePackBuf.data[2+4] = 2;
                                           	            break;
                                           	#endif
								       	                    case UPGRADE:    //bit0~3赋值为0x04
								       	                                //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x04
								       				                          framePackBuf.data[0+ 4]  |= 0x04;
								       	                                break;
                                             default: break;
                                           }	
							         
                                    //data[1]升级状态
							         framePackBuf.data[5] = 0;
							         
							         
							         if(6 == pSystemStatusList->cpValue){
									//	printf("v:%d\nc:%d\n",pSystemStatusList->volRms,pSystemStatusList->curRms);
							               *((uint16_t *)(&framePackBuf.data[4+4])) = pSystemStatusList->volRms;
							               *((uint16_t *)(&framePackBuf.data[6+4])) = pSystemStatusList->operatingCurrent.curRms;
											//	 printf("v:%d\nc:%d\n",*((uint16_t *)(&framePackBuf.data[4+4])),*((uint16_t *)(&framePackBuf.data[6+4])));
							         }else{
							               *((uint16_t *)(&framePackBuf.data[4+4])) = 0;
							               *((uint16_t *)(&framePackBuf.data[6+4])) = 0;
							         }
                       
					              
	        		  break;
											 }
	        	          case 2://读取软件版本号
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000002;
					                
									*(uint32_t*)(framePackBuf.data+4) = *(uint32_t*)pSystemStatusList->parameterList.VN;
	        	                break;
								}
					      case 4://获取充电功率
								{

									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000004;
									*((uint32_t *)(framePackBuf.data + 4)) = (uint32_t)((pSystemStatusList->operatingCurrent.curRms/10) * (pSystemStatusList->volRms/10));
					            
							  
	        		             break;
								}
					      case 6://获取工作模式---OK
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000006;
									*((uint32_t *)(framePackBuf.data + 4)) = pSystemStatusList->workingMode;
					                break;
								}
					      case 15://获取配置参数
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x18;
                                    *(uint32_t*)framePackBuf.data= 0x0000000f;
									*((uint32_t *)(framePackBuf.data + 4)) = pSystemStatusList->parameterList.BaudRate;//通讯速率
									*((uint32_t *)(framePackBuf.data + 8)) = pSystemStatusList->parameterList.maxPower;//最大充电功率
									*((uint32_t *)(framePackBuf.data + 12)) = *(uint32_t*)pSystemStatusList->parameterList.VN;//版本号
									*((pAdress6)(framePackBuf.data + 16)) = *(pAdress6)pSystemStatusList->parameterList.PileAddress;//桩地址
									framePackBuf.data[22]=0xff;
									framePackBuf.data[23]=0xff;
					              break;
								}
					      default : break;
	        	  }
				  
				  
				  {//回复 //换成拷贝函数
				   *(pAdress6)framePackBuf.Address_receiver = *(pAdress6)pSystemStatusList->zhuzhanAddress;
				  }
				  
				  
			break;
	        case 0x14://写
	            switch(pFrame->ruler_id){
					
	        		      case 3://启动/停止充电 ------OK
								{
								
									switch(pFrame->aucDataTmp[4]){/*除数据标识外的数据域的最低字节*/
										case 0 :
											  pSystemStatusList->remoteStartOrStopFlag = stopCharging;
											  executingResults = true;
											break;
										case 1:
											  
											  {//获取充电功率
											  
											  
											  if(6 == pSystemStatusList->cpValue || 9 == pSystemStatusList->cpValue ){/*桩处在连接状态*/
		                                         if(false == pSystemStatusList->scramStatus){/*急停没有按下*/
												   pSystemStatusList->remoteStartOrStopFlag = canStartCharging; 
											  }}
									
									
									
									if(pFrame->data_len == 12){
									            pSystemStatusList->powerSetInBackground = (*((uint32_t * )(&(pFrame->aucDataTmp[8]))));
		                                        power_ls = *(uint32_t *)(pFrame->aucDataTmp+8);
									            //功率转换为百分比存到参数列表中
												//如果你要更改这里的话，记得把设置功率命令里的也改了
										        if((power_ls < pSystemStatusList->powerMax) && (power_ls > POWER_MIN)){//如果功率在合法的范围 32*220
                                                   pSystemStatusList->percentPWM = power2percent(power_ls);
                                                   executingResults = true;
												}else{
                                                   	//回复异常
													executingResults = false;
									                errorNumber = POWER_EXCEEDS_LIMIT;
                                                }
												
									}
												
												
												
									          }									
											break;
                                        default : 
										   	
										break;
									}
								
	        		      break;
								} 
	        		      case 4://设置充电功率 ------
								{
									pSystemStatusList->powerSetInBackground = (*((uint32_t * )(&(pFrame->aucDataTmp[4]))));
									/*数组下标的4表示除数据标识外的数据域的最低字节*/
								//	printf("%08x\n",pSystemStatusList->powerSetInBackground);
									power_ls = pSystemStatusList->powerSetInBackground;
									//功率转换为百分比存到参数列表中
									//如果你要更改这里的话，记得把启动命令里的也改了
								    if((power_ls < pSystemStatusList->powerMax) && (power_ls > POWER_MIN)){//如果功率在合法的范围 32*220
                                         pSystemStatusList->percentPWM = power2percent(power_ls);
                                         executingResults = true;
									}else{
                                                   	//回复异常
													executingResults = false;
									                errorNumber = POWER_EXCEEDS_LIMIT;
                                    }
			            
	        		      break;
								}
					      case 5://重启充电桩 ------OK---有插枪不允许重启
								{
									if((6 == pSystemStatusList->cpValue )||(9 == pSystemStatusList->cpValue)){//插枪，不允许重启
										executingResults = false;
									    errorNumber = 0;
									}
									else{
								        executingResults = true;
                                        endWork = END_WORK_REBOOT;								
									}									
							        break;
								}
					      case 6://设置工作模式------OK---自动和命令模式
								{
									if(CHARGING == pSystemStatusList->workingState_flag){
										pSystemStatusList->remoteStartOrStopFlag = canStartCharging ;
									}
								   pSystemStatusList->workingMode = pFrame->aucDataTmp[4];
								   //写入FLASH
								   writeWorkingMode(pSystemStatusList->workingMode);
			                       executingResults = true;
					               break;
								}
						  case 7://切换升级模式 ----OK-----只有空闲状态才允许升级
								{
									
									printf("zhixing shengji1 \n");
									if(FREE == pSystemStatusList->workingState_flag){//空闲状态才允许升级
									    //改写升级标志为有升级请求
										printf("kong xian1 \n");
										writeIAPUpgradeFlag(UPGRADE_REQUEST);
										printf("kong xian 2\n");
										endWork = END_WORK_REBOOT;
										executingResults = true;
										printf("kong xian \n");
									}else{//回复异常
									
								        executingResults = false;
									    errorNumber = REFUSE_UPGRADE;
										printf("buyunxu shengji \n");
                                    }		
                                    									
					                break;
								}
					      case 15://配置参数
								{
								 baudRate_buf = 	*(uint32_t*)(pFrame->aucDataTmp + 4);
								 if(baudRate_buf == 9600 || \
								    baudRate_buf == 0 || \
								    baudRate_buf == 4800 || \
									baudRate_buf == 2400 || \
									baudRate_buf == 19200 || \
									baudRate_buf == 115200 ){
										if(baudRate_buf != 0){
											pSystemStatusList->parameterList.BaudRate = baudRate_buf;//通信速率
										}
										
										
                                        maxPower_buf = *(uint32_t*)(pFrame->aucDataTmp + 8);//最大充电功率
										if (maxPower_buf == POWER_MAX || maxPower_buf == 0 ||maxPower_buf == 3520){
											if(maxPower_buf != 0){
												pSystemStatusList->parameterList.maxPower = maxPower_buf;
											}
											if(pFrame->aucDataTmp[12] != '0' || \
											pFrame->aucDataTmp[13] != '0' || \
											pFrame->aucDataTmp[14] != '0' || \
											pFrame->aucDataTmp[15] != '0' ){
											   *(uint32_t*)pSystemStatusList->parameterList.VN = *(uint32_t*)(pFrame->aucDataTmp + 12);//版本号

											}
										
										
										    if(pFrame->aucDataTmp[16] != '0' || \
											pFrame->aucDataTmp[17] != '0' || \
											pFrame->aucDataTmp[18] != '0' || \
											pFrame->aucDataTmp[19] != '0' || \
											pFrame->aucDataTmp[20] != '0' || \
											pFrame->aucDataTmp[21] != '0' ){
									            *(pAdress6)pSystemStatusList->parameterList.PileAddress = *(pAdress6)(pFrame->aucDataTmp + 16);//写入桩地址

											}
								            UpdateSettingList(&pSystemStatusList->parameterList);
								            executingResults = true;
										}else{
											executingResults = false;
										    errorNumber = 1;
										}
				
								 
									}else {
										executingResults = false;
										errorNumber = 1;
									}
								 

					             break;
								}
	        		  default : break;
	        	  }	
				  
		
		{//回复
				      if(executingResults == true){//正常
					  printf("zhengchang chuli \n");
									framePackBuf.ctrl_C = 0x94;
                                    framePackBuf.data_L = 0x0;
						  }
					  else{//异常
					  printf("yichang chuli \n");
								    framePackBuf.ctrl_C = 0xD4;
                                    framePackBuf.data_L = 0x1;
                                    framePackBuf.data[0]= errorNumber;
						  }
				
		       *(pAdress6)framePackBuf.Address_receiver = *(pAdress6)pSystemStatusList->zhuzhanAddress;
		}
		
	        break;
			
	        default : break;
        }	


		//打包
		if(0 == pack_d07_frame_by_data(&framePackBuf, frameBuf, &frameLen)){
		//发送
			RS485_SendStr_length(frameBuf,frameLen);	
			printf("fa song \n");
		}else{
		printf("\n打包失败\n");
		}
		
		switch(endWork){
		case END_WORK_REBOOT:
		printf("chong qi \n");
		      SoftReset();
		break;
		default : break;
	}
	}
}












void communication(StatusList * pSystemStatusList){
	uint8_t *  pframe;
	uint16_t frameLen = 0;
	S_D07_UNPACK  outpFrame;
	if( true == pSystemStatusList->messageNotice.messageStatus ){
		printf("shoudao\n");
		if(0 == get_d07_first_valid_frame(pSystemStatusList->messageNotice.pMessage,pSystemStatusList->messageNotice.messageLen,&pframe,&frameLen)){
			if( 0 ==  unpack_d07_frame(pframe,frameLen,&outpFrame)){
				listen(&outpFrame, pSystemStatusList);//处理解析后的数据
			}
			else{
			printf("JieXi ShiBai\n");
			}
		}else{
		printf("MeiYou ZhaoDao zhen\n");
		}
		clean_rebuff();
	}else{
	
	}
}


 
int main(void){
//改到IAP中
SCB->VTOR = FLASH_BASE | 0x4000;//重定义中断向量表,该程序在FLASH中的起始地址为：0x8004000。
#if ZFZ 
{
	StatusList sysStatusList;
	StatusList * pSysStatusList = &sysStatusList ;
	SysInit(pSysStatusList);/*初始化函数*/

	while(1){
	  GetSystemStatus(pSysStatusList);/*刷新信息*/
		communication(pSysStatusList);/*通信处理*/
		ExceptionHanding(pSysStatusList);/*异常处理*/
		WorkingStateChange(pSysStatusList);/*工作状态切换*/
		DisplayProcessing(pSysStatusList);/*显示处理*/
	//加看门狗
	//串口中断对FLASH的影响？
	}

}
#endif	
}

/*********************************************END OF FILE**********************/
