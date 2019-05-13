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

#define WAITING_FOR_PLUGIN 2000	// ��λ������
#define DEFAULT_PWM_RATIO 52	// ����53������55����λ���ٷ�֮һ ռ�ձ�
//#define DEFAULT_RATIO 100 //��λ��ǧ��֮һ Ĭ��ռ�ձ� ��ʼ�������б��еĲ����õ�

#define PB_LED_RUN		GPIO_Pin_3    //GPIO_Pin_12
#define PA_AC_CONTACTOR	GPIO_Pin_1
#define PA_CC			GPIO_Pin_4
#define PA_CP_PWM		GPIO_Pin_7
#define PA_CP_DET0		GPIO_Pin_5
#define PB_CP_DET1		GPIO_Pin_0

#define PB_LED_LINK	GPIO_Pin_4  //���ӵ�
#define PB_LED_ERR		GPIO_Pin_5  //���ϵ�
#define PB_LED_CHARGING	GPIO_Pin_6  //����
#define PB_STOP		GPIO_Pin_7  //��ͣ�������

#define CHARGING 1
#define FREE 2
#define WAITDRAWGUN 3
#define ERRORS_STOP 4
#define ERRORS_VOL_CUR_FREQ 5
#define UPGRADE 6

#define       IAPLICATIONADDRESS       ((uint32_t)0x08000000) //IAP����ʼ��ַ


#define ZFZ 1 //�޸��еĳ��򿪹غ�
#define BAN_CESHI 1 //���ε���ѹ��������Ƶ�ʵı���


#define __DEBUG___
#ifdef __DEBUG__  
#define DEBUG(format,...) printf("File: "__FILE__", Line: %d: "format"\n", __LINE__, ##__VA_ARGS__)  
#else  
#define DEBUG(format,...)  
#endif

#define VN_LEN 4 //����汾���ֽ���

//ͨ�ŵĴ�����
#define POWER_EXCEEDS_LIMIT (1) //���ʲ�������ķ�Χ
#define REFUSE_UPGRADE (2) //�ܾ���������

#define POWER_MAX (32*220) //7040 ռ�ձ�53%
#define POWER_MIN (6*220) //1320 ռ�ձ�10%

#define CUT_PWM (1)
#define CUT_CONTACTOR (2)
#define CUT_CONTACTOR_PWM (3)

#define END_WORK_REBOOT (1)
#define END_WORK_IAP2APP (2)

extern uint32_t timeRecord;//


typedef enum {normalVol,overVol,underVol}VolStatus; //��������ѹ��Ƿѹ
typedef enum {normalCur,overCur,overCurTimeout}CurStatus; //������������������ʱ
typedef enum {normalFreq,overFreq,underFreq}FreqStatus; //������Ƶ�ʹ��ߣ�Ƶ�ʹ���
typedef enum {delay_free,delay_start,delay_flow}FlagDelay; //�쳣��ֹ״̬��־
typedef enum {canStartCharging,stopCharging}RemotePermissibleChargingStatus;// ����״̬
typedef struct{char a[6];} *pAdress6 ;
typedef void (*pFunction)(void);

typedef struct backstageMessage{
           bool messageStatus;//�Ƿ����µ���Ϣ
		   uint16_t messageLen;//��Ϣ�ĳ���
		   uint8_t * pMessage;//��Ϣ��ָ��
}BackstageMessage;

typedef struct List StatusList; //�˾��Ŀ����Ϊ�˽������������ṹ����໥����
typedef struct {
		uint32_t curRms;  /*������Чֵ*/
	    CurStatus   curStatus; //������״̬
		float        cur2RmsGain;  /*�������Чֵʱ�õ�������Cur2RmsGain =0.000736207;*/
		int          curMax; /*��ǰ�����������*/
		void (* GetCurValue)(StatusList * pSystemStatusList); //��ȡ�����ķ���
	    void (* currentMonitoring)(StatusList *pSystemStatusList); //���������ļ�ط���
}CURRENT;
struct List{
           uint8_t      cpValue;//CP�Ĳ���ֵ
		   uint8_t      workingState_flag;/*4�ֹ���״̬������У����У�����ǹ������*/
           bool         scramStatus;/*��ͣ״̬*/
           u32          volRms;/*��ѹ��Чֵ*/
		   float        volRmsGain;/*��ѹֵ����VolRmsGain = 0.0009737;*/
           float        volFreq; /*��ѹƵ��*/
           RemotePermissibleChargingStatus         remoteStartOrStopFlag;
		       int          vMax; /*��ѹ��*/
		       int          vMin; /*Ƿѹ��*/
		       int          curMin; /**/
		       int          freqMax; /*�������Ƶ��*/
		       int          freqMin; /*�������Ƶ��*/
		       VolStatus   volStatus; //��ѹ��״̬
		       FreqStatus  freqStatus; //Ƶ�ʵ�״̬
			   SettingList parameterList; //FLASH�еļ�¼�Ĳ���
					 
					 char zhuzhanAddress[6]; //���صĵ�ַ
					 char benjiAddress[6]; //�����ĵ�ַ
					 char broadcastAddress[6]; //�㲥��ַ
					 char developersAddress[6]; //�����ߵ�ַ
					 //ͨ���й�
					 BackstageMessage messageNotice;


					 uint32_t powerSetInBackground;//��̨���õĹ���
					 unsigned int percentPWM ; //PWM�İٷֱ� �ֱ���ǧ��֮һ
					 uint8_t workingMode ; //���ģʽ��0 ��ʾ�Զ���磬�������Զ�����������档 1 ��ʾ�����磬���ʱ��Ҫ�ȴ���̨�ĳ������
					 uint32_t powerMax;
					 const uint32_t * pTimeRecord;
					 
					 CURRENT operatingCurrent; //����ʱ�ĵ�����
					 
} ;
unsigned int * pRatio;//PWM���غ����õģ�ָ������б��е�percentPWM
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
	

	GPIO_ResetBits(GPIOB, PB_LED_LINK);//�����ӵƣ�������PB4ΪJTAG���ýŵ�JNTRST,��λ��Ĭ����������ģʽ��������Ҫ��һ��
}


/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
// ARR ���Զ���װ�ؼĴ�����ֵ
// CLK_cnt����������ʱ�ӣ����� Fck_int / (psc+1) = 72M/(psc+1)
// PWM �źŵ����� T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// ռ�ձ�P=CCR/(ARR+1)

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

	// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	GENERAL_TIM_APBxClock_FUN(GENERAL_TIM_CLK,ENABLE);
	// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=GENERAL_TIM_Period;	
	// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= GENERAL_TIM_Prescaler;	
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(GENERAL_TIM, &TIM_TimeBaseStructure);

	// ����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// ���ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// ���ͨ����ƽ��������	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// ����Ƚ�ͨ�� 2
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	TIM_OC2Init(GENERAL_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);

	// ʹ�ܼ�����
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
/**��ʱ����**/
typedef enum {  //��ʱ�����״̬
    STOP,            // ͣ��
    GET_TIMESTAMP,             // ��ȡʱ���
    DETERMINE,             // �жϣ���ָ����ʱ�䵽�ˣ���ִ��ָ��������
} TIMING_STATUS;

//��ʱ����������Ϣ
typedef struct{
	//ʱ���
	uint32_t timeStamp;

	//��ʱʱ��
	uint32_t TimingTime;
	//��ʱ״̬
	TIMING_STATUS timingStatus;
	
	//ʱ�䵽��Ҫִ�е�����
	void (* TaskHook)(StatusList *);
} TIMED_TASKS;

typedef enum _TASK_LIST  {  
    TASK1,            // ����1
    TASK2,            // ����2
    TASK3,            //����3
	TASK4,            //����4
     // ��ӱ������
     TASKS_MAX         // �ܵĿɹ�����Ķ�ʱ���������
} TASK_LIST; 

void task1(StatusList *pSystemStatusList){
	
}
void task2(StatusList *pSystemStatusList){ //����ͣ�������3�����Դ��ڳ��״̬��Ͽ��Ӵ����������ȴ���ǹ״̬��
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
	//cur = U24TRANS (MeterRead(RMS_I2));  //����һ��ͨ���ĵ���
    pSystemStatusList->operatingCurrent.curRms  =(0.1 *cur * pSystemStatusList->operatingCurrent.cur2RmsGain);
}
void CurrentMonitoringMethod(StatusList *pSystemStatusList){
	//��ȡ����ֵ
	
		pSystemStatusList->operatingCurrent.GetCurValue(pSystemStatusList);
		//pSystemStatusList->operatingCurrent.curRms = 100;
	//��ȡ��ǰ�����������ֵ
		pSystemStatusList->operatingCurrent.curMax = 10 * ((uint32_t) pSystemStatusList->powerSetInBackground /220);
		//pSystemStatusList->operatingCurrent.curMax = 20;
if(CHARGING == pSystemStatusList->workingState_flag){
		if(pSystemStatusList->operatingCurrent.curMax <= 200){//�����������һ��
			if(pSystemStatusList->operatingCurrent.curRms >= (pSystemStatusList->operatingCurrent.curMax  + 20) ){
				
				//��һ�ν���������ʱ����5����õ���״̬Ϊ������ʱ
				if(pSystemStatusList->operatingCurrent.curStatus == normalCur)
				{
					pSystemStatusList->operatingCurrent.curStatus = overCur;
				    //������ʱ������ȥ���ţ�5���к󲻸��㷢�źţ���Ͱ�״̬�ĳɹ�����ʱ��
				    timed_task[TASK4].timingStatus = GET_TIMESTAMP;
				}
			}else{
				pSystemStatusList->operatingCurrent.curStatus = normalCur;
				//�����ʱ���񲻵���STOP����ΪSTOP
				if(timed_task[TASK4].timingStatus != STOP){
					timed_task[TASK4].timingStatus = STOP;
				}
			}
		}else if(pSystemStatusList->operatingCurrent.curMax >200 ){
			if(pSystemStatusList->operatingCurrent.curRms >= (pSystemStatusList->operatingCurrent.curMax *1.1) ){

				//��һ�ν���������ʱ����5����õ���״̬Ϊ������ʱ
				if(pSystemStatusList->operatingCurrent.curStatus == normalCur)
				{
					pSystemStatusList->operatingCurrent.curStatus = overCur;
				    //������ʱ����
				    timed_task[TASK4].timingStatus = GET_TIMESTAMP;
				}
			}else{
				pSystemStatusList->operatingCurrent.curStatus = normalCur;
				//�����ʱ���񲻵���STOP����ΪSTOP
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

pSystemStatusList->pTimeRecord = &timeRecord;//�����б���һ��ָ��ʱ���׼��ָ�룬�û�׼ֻ����
/*----�ײ�ģ���ʼ��----------*/
	  GpioConfig();
	  PwmConfig();
	  EXTI_Key_Config();
	  TimConfig();
	  SPI_METER_Init();
	  
	//Ĭ�����÷�װ�ɺ���  
GetSetting(&(pSystemStatusList->parameterList));
if(0xff == pSystemStatusList->parameterList.flag_default){//д��Ĭ�ϲ���
	  strcpy(pSystemStatusList->parameterList.SN,"kj01");
	  //����汾�ŵĳ�ʼ��
	  pSystemStatusList->parameterList.VN[0] = 0x01;//Ĭ��Ӳ���汾
	  pSystemStatusList->parameterList.VN[1] = 0x00;
	  pSystemStatusList->parameterList.VN[2] = 0x00;
	  pSystemStatusList->parameterList.VN[3] = 0x00;
	  #if 1
	  //vxn4f0
	  pSystemStatusList->parameterList.PileAddress[0] = '0';//Ĭ��׮��ַ
	  pSystemStatusList->parameterList.PileAddress[1] = 'f';
	  pSystemStatusList->parameterList.PileAddress[2] = '4';
	  pSystemStatusList->parameterList.PileAddress[3] = 'n';
	  pSystemStatusList->parameterList.PileAddress[4] = 'x';
	  pSystemStatusList->parameterList.PileAddress[5] = 'v';
	  #endif

	  pSystemStatusList->parameterList.BaudRate = 9600;//Ĭ�ϲ�����
	  pSystemStatusList->parameterList.maxPower = 7040;//Ĭ�������
	  
    pSystemStatusList->parameterList.flag_default = 0;
    pSystemStatusList->parameterList.vMax = 2640;/*��ѹ�㣬����ֵ��20%*/
    pSystemStatusList->parameterList.vMin = 1870;/*Ƿѹ�㣬����ֵ��15%*/
    pSystemStatusList->parameterList.curMax = 3520;/*�����㣬����ֵ��1.1��*/
    pSystemStatusList->parameterList.freqMax = 65;/*�������Ƶ��*/
    pSystemStatusList->parameterList.freqMin = 45;/*�������Ƶ��*/
    pSystemStatusList->parameterList.volRmsGain = 0.0009737;
    pSystemStatusList->parameterList.cur2RmsGain =0.000736207;
	
	UpdateSettingList(&(pSystemStatusList->parameterList));
}
USART_Config();	 
RS485_Config(pSystemStatusList->parameterList.BaudRate);//��ʼ��485,��ڲ���Ϊͨ�����ʡ�

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

/*-------------ָʾ�ƣ��Ӵ�����pwm�����ʼ��-----------------*/
    SetAcContactor(false);/*�Ͽ��Ӵ���*/
    PwmCtrl(false);/*�ر�PWM���*/
/*-----------------------------------------------------------*/


/*---------------Ĭ�����ò���------------------------------*/
  
	pSystemStatusList->vMax = pSystemStatusList->parameterList.vMax;
    pSystemStatusList->vMin = pSystemStatusList->parameterList.vMin;
    pSystemStatusList->operatingCurrent.curMax = pSystemStatusList->parameterList.curMax;
    pSystemStatusList->freqMax = pSystemStatusList->parameterList.freqMax;
    pSystemStatusList->freqMin = pSystemStatusList->parameterList.freqMin;
    
   
    MeterWrite(ANAEN,0x3f);  /*�򿪵��I2����ͨ��*/
    pSystemStatusList->volRmsGain = pSystemStatusList->parameterList.volRmsGain;
    MeterWrite(URMSOFFSET,0x0A);  /*��ѹ��Чֵƫ��*/
    pSystemStatusList->operatingCurrent.cur2RmsGain = pSystemStatusList->parameterList.cur2RmsGain;
    MeterWrite(I2RMSOFFSET,0x02); /*����2��Чֵƫ��*/
  
/************���쳣�йص�********************/
    scramStatusFlag = GetScramStatues(); /*��ͣ״̬��ʼ��*/
/********************************************/

/***********��״̬�л��йص�*****************/
    pSystemStatusList->workingState_flag = FREE;
/********************************************/


#if BAN_CESHI
/*��ѹ����Ƶ�ʳ�ʼ��Ϊ����*/
    pSystemStatusList->volStatus = normalVol;
    //pSystemStatusList->curStatus = normalCur;
    pSystemStatusList->freqStatus = normalFreq;

#endif

/*--------���̨��ͨ���йص�------------*/

		
        pSystemStatusList->remoteStartOrStopFlag = stopCharging;

		*(pAdress6)pSystemStatusList->benjiAddress = *(pAdress6)pSystemStatusList->parameterList.PileAddress;
		*(pAdress6)pSystemStatusList->zhuzhanAddress = *(pAdress6)pSystemStatusList->benjiAddress;
		
		
		//��ȡ����ģʽ
		readWorkingMode(&(pSystemStatusList->workingMode));
		//��ʼ��PWM��ռ�ձ�
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
			//������ʱ����
			timed_task[TASK1].timingStatus = GET_TIMESTAMP;
			//printf("������ʱ����\n");
		}else{
			if( timed_task[TASK1].timingStatus == STOP){
				//printf("ʱ�䵽\n");
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
				//printf("ʱ��û��\n");
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
	//��ȡͨ��״̬
	GetMessageStatus(pSystemStatusList);
}


void ExceptionHanding(StatusList * pSystemStatusList){
/*��ͣ���µĴ���*/
    if( true == pSystemStatusList->scramStatus ){/*��ͣ����*/
		SetAcContactor(false);/*�Ͽ��Ӵ���*/
		PwmCtrl(false);/*�ص�PWM���*/
		pSystemStatusList->workingState_flag = ERRORS_STOP;		
	}

} 



bool ChargeCondition(StatusList * pSystemStatusList){//������ת�����״̬�������������ж�
	if(6 == pSystemStatusList->cpValue || 9 == pSystemStatusList->cpValue ){/*׮��������״̬*/
		if(false == pSystemStatusList->scramStatus){/*��ͣû�а���*/
			
			if(pSystemStatusList->workingMode == 0){//����ģʽΪ�Զ�ģʽ
				    return true;
			}else{//����ģʽΪ����ģʽ

			    if(canStartCharging == pSystemStatusList->remoteStartOrStopFlag){/*��̨������*/
			    	return true;/*���Կ�ʼ���*/
			    }
			}
			
		}
	}
	return false;	
}

void StartCharging(StatusList * pSystemStatusList){
	if(ChargeCondition(pSystemStatusList)== true ){
		PwmCtrl(true);/*����PWM*/
		if(6 == pSystemStatusList->cpValue){
					SetAcContactor(true);/*�պϽӴ���*/
					pSystemStatusList->workingState_flag = CHARGING;
		}
	}else{
		PwmCtrl(false);
		SetAcContactor(false);/*�Ͽ��Ӵ���*/
		pSystemStatusList->workingState_flag = FREE;
	}
}



bool StopChargeCondition(StatusList * pSystemStatusList){//���״̬��ת�����״̬�������������ж�
	 if(pSystemStatusList->cpValue == 6){/*����״̬*/
		#if BAN_CESHI
		 if(overCurTimeout != pSystemStatusList->operatingCurrent.curStatus ){/*������*/
		#endif
		
		          if(pSystemStatusList->workingMode == 1 ){//����ģʽ
					if(canStartCharging == pSystemStatusList->remoteStartOrStopFlag){/*��̨������*/
						return 0;
					}else{
						return CUT_PWM;
					}	
				  }else{//�Զ�ģʽ
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
		//������ȫ���
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
	setPercent(pSystemStatusList->percentPWM);//����PWM
	switch(StopChargeCondition(pSystemStatusList)){
		case  CUT_CONTACTOR :
		SetAcContactor(false);
		pSystemStatusList->workingState_flag = WAITDRAWGUN;
		break;
		case  CUT_PWM :
		PwmCtrl(false);/*�ص�PWM���*/
		//�򿪶�ʱ��
		//��ʱ�䵽���жϽӴ�������ת������ǹ��
		if(timed_task[TASK2].timingStatus == STOP){
			timed_task[TASK2].timingStatus = GET_TIMESTAMP;
		}
		break;
		case  CUT_CONTACTOR_PWM :
		SetAcContactor(false);/*�Ͽ��Ӵ���*/
		PwmCtrl(false);/*�ص�PWM���*/
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
	if( false == pSystemStatusList->scramStatus ){/*��ͣ�ɿ�*/
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
switch(pSystemStatusList->workingState_flag)//״̬�����ź�����
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
		
		GPIO_SetBits(GPIOB, PB_LED_LINK); /*�����ӵ�*/
		//printf("\r\ncpValue:%d\r\n",pSystemStatusList->cpValue);//�벻Ҫɾ���������������
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_LINK); /*��*/
		//printf("\r\ncpValue:%d\r\n",pSystemStatusList->cpValue);//�벻Ҫɾ���������������
	}
	
	if(true == pSystemStatusList->scramStatus || ERRORS_VOL_CUR_FREQ ==  pSystemStatusList->workingState_flag){
		
		GPIO_SetBits(GPIOB, PB_LED_ERR);   /*�����ϵ�*/
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_ERR); /*��*/
	}
	
	if( CHARGING == pSystemStatusList->workingState_flag ){
		
		GPIO_SetBits(GPIOB, PB_LED_CHARGING);  /*������*/
	}else{
		GPIO_ResetBits(GPIOB, PB_LED_CHARGING);  /*��*/
	}
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

void SoftReset(void){
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}


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

	// GB T 18487.1-2015 ��A.1 �����ʩ������ռ�ձ��������ӳ���ϵ
	// 10% <= D <= 85%   Imax = D*100*0.6
	// 85% < D <= 90%    Imax = (D*100-64)���� Imax <= 63
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
	
//����ת���ɰٷֱ�
uint32_t power2percent(uint32_t power_ls){
		unsigned int cur;
		//int percent_ls;
		//��ȡ����
		 cur= (int)(power_ls /220);
		//��ȡ�ٷֱ�
		 return setCurrent(cur);
		 
}


void listen(S_D07_UNPACK * pFrame, StatusList * pSystemStatusList){
	
	S_D07_PACK_FRAME framePackBuf;//����õ���Ϣ�ṹ��
	
  uint8_t frameBuf[D07_DATA_MAX + D07_FRAME_LEN_MIN]={0}; //֡������
	
	uint32_t frameLen = 0;
	
	uint32_t power_ls;//��ʱ�洢����
	bool executingResults; //ָ���ִ�н��
	uint8_t errorNumber; //������
    uint8_t endWork = 0;
	uint32_t baudRate_buf = 0;
	uint32_t maxPower_buf = 0;
	if((cmpStr( pFrame->address,pSystemStatusList->benjiAddress) == true) || \
	((cmpStr( pFrame->address,pSystemStatusList->broadcastAddress) == true) && ((pFrame->ruler_id == 7)||(pFrame->ruler_id == 0xf)) ) || \
	(cmpStr( pFrame->address,pSystemStatusList->developersAddress) == true)){//��ַΪ������ַ ���߹㲥��ַ�ҿ�����Ϊ15 �� 7
		//printf("%02x\n",pFrame->ctrl_c);
		//printf("%02x\n",pFrame->ruler_id);
		switch(pFrame->ctrl_c){
			
	        case 0x11://��
	            switch(pFrame->ruler_id){//�������ݱ�ʶ��δ��
	        		      case 1://��ȡ���׮״̬
								{//��װ�ɺ���
						          
							        framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 12;
                                    *(uint32_t*)framePackBuf.data= 0x00000001;
                                    				        
					                
									 
									 
									 
									 
								       //��װ������
							         //data[0]
							         if((6 == pSystemStatusList->cpValue )||(9 == pSystemStatusList->cpValue)){//bit7��1//bit7=1��ʾ��ǹ״̬
							             //framePackBuf.data[0+ 4] |= (0x1<<7);
								           framePackBuf.data[0 + 4] |= 0x80;
							         }else{//bit7��0//bit7 =0 ��ʾδ��ǹ
							             //framePackBuf.data[0+ 4] &= (~(0x01<<7));
								           framePackBuf.data[0+ 4] &= 0x7f;
							         }
							         framePackBuf.data[2+ 4] = 0;//����Ϊ0 ��ʾ�޹���
							         framePackBuf.data[0+ 4] &= 0xf0;//����bit0~3
							         switch(pSystemStatusList->workingState_flag)
                                           {
                                             case CHARGING:    //bit0~3��ֵΪ0x01
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x01
								       				                          framePackBuf.data[0+ 4]  |= 0x01;
                                                       break;
                                             case FREE:    //bit0~3��ֵΪ0x00
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x00
								       				                         framePackBuf.data[0+ 4] &= 0xf0;
                                                       break;
                                             case WAITDRAWGUN:    //bit0~3��ֵΪ0x02
                                           			       //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x02
								       				                         framePackBuf.data[0+ 4]  |= 0x02;
                                           			       break;
                                             case ERRORS_STOP:    //bit0~3��ֵΪ0x03
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x03
								       				                          framePackBuf.data[0+ 4]  |= 0x03;
								       				                          framePackBuf.data[2+ 4] = 1;
                                           			        break;
                                           	#if BAN_CESHI
                                             case ERRORS_VOL_CUR_FREQ:    //bit0~3��ֵΪ0x03
                                                        //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x03
								       				                          framePackBuf.data[0+4]  |= 0x03;
								       				                          framePackBuf.data[2+4] = 2;
                                           	            break;
                                           	#endif
								       	                    case UPGRADE:    //bit0~3��ֵΪ0x04
								       	                                //framePackBuf.data[0] = & (~(0x0f<<0)) | 0x04
								       				                          framePackBuf.data[0+ 4]  |= 0x04;
								       	                                break;
                                             default: break;
                                           }	
							         
                                    //data[1]����״̬
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
	        	          case 2://��ȡ����汾��
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000002;
					                
									*(uint32_t*)(framePackBuf.data+4) = *(uint32_t*)pSystemStatusList->parameterList.VN;
	        	                break;
								}
					      case 4://��ȡ��繦��
								{

									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000004;
									*((uint32_t *)(framePackBuf.data + 4)) = (uint32_t)((pSystemStatusList->operatingCurrent.curRms/10) * (pSystemStatusList->volRms/10));
					            
							  
	        		             break;
								}
					      case 6://��ȡ����ģʽ---OK
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x8;
                                    *(uint32_t*)framePackBuf.data= 0x00000006;
									*((uint32_t *)(framePackBuf.data + 4)) = pSystemStatusList->workingMode;
					                break;
								}
					      case 15://��ȡ���ò���
								{
									framePackBuf.ctrl_C = 0x91;
                                    framePackBuf.data_L = 0x18;
                                    *(uint32_t*)framePackBuf.data= 0x0000000f;
									*((uint32_t *)(framePackBuf.data + 4)) = pSystemStatusList->parameterList.BaudRate;//ͨѶ����
									*((uint32_t *)(framePackBuf.data + 8)) = pSystemStatusList->parameterList.maxPower;//����繦��
									*((uint32_t *)(framePackBuf.data + 12)) = *(uint32_t*)pSystemStatusList->parameterList.VN;//�汾��
									*((pAdress6)(framePackBuf.data + 16)) = *(pAdress6)pSystemStatusList->parameterList.PileAddress;//׮��ַ
									framePackBuf.data[22]=0xff;
									framePackBuf.data[23]=0xff;
					              break;
								}
					      default : break;
	        	  }
				  
				  
				  {//�ظ� //���ɿ�������
				   *(pAdress6)framePackBuf.Address_receiver = *(pAdress6)pSystemStatusList->zhuzhanAddress;
				  }
				  
				  
			break;
	        case 0x14://д
	            switch(pFrame->ruler_id){
					
	        		      case 3://����/ֹͣ��� ------OK
								{
								
									switch(pFrame->aucDataTmp[4]){/*�����ݱ�ʶ��������������ֽ�*/
										case 0 :
											  pSystemStatusList->remoteStartOrStopFlag = stopCharging;
											  executingResults = true;
											break;
										case 1:
											  
											  {//��ȡ��繦��
											  
											  
											  if(6 == pSystemStatusList->cpValue || 9 == pSystemStatusList->cpValue ){/*׮��������״̬*/
		                                         if(false == pSystemStatusList->scramStatus){/*��ͣû�а���*/
												   pSystemStatusList->remoteStartOrStopFlag = canStartCharging; 
											  }}
									
									
									
									if(pFrame->data_len == 12){
									            pSystemStatusList->powerSetInBackground = (*((uint32_t * )(&(pFrame->aucDataTmp[8]))));
		                                        power_ls = *(uint32_t *)(pFrame->aucDataTmp+8);
									            //����ת��Ϊ�ٷֱȴ浽�����б���
												//�����Ҫ��������Ļ����ǵð����ù����������Ҳ����
										        if((power_ls < pSystemStatusList->powerMax) && (power_ls > POWER_MIN)){//��������ںϷ��ķ�Χ 32*220
                                                   pSystemStatusList->percentPWM = power2percent(power_ls);
                                                   executingResults = true;
												}else{
                                                   	//�ظ��쳣
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
	        		      case 4://���ó�繦�� ------
								{
									pSystemStatusList->powerSetInBackground = (*((uint32_t * )(&(pFrame->aucDataTmp[4]))));
									/*�����±��4��ʾ�����ݱ�ʶ��������������ֽ�*/
								//	printf("%08x\n",pSystemStatusList->powerSetInBackground);
									power_ls = pSystemStatusList->powerSetInBackground;
									//����ת��Ϊ�ٷֱȴ浽�����б���
									//�����Ҫ��������Ļ����ǵð������������Ҳ����
								    if((power_ls < pSystemStatusList->powerMax) && (power_ls > POWER_MIN)){//��������ںϷ��ķ�Χ 32*220
                                         pSystemStatusList->percentPWM = power2percent(power_ls);
                                         executingResults = true;
									}else{
                                                   	//�ظ��쳣
													executingResults = false;
									                errorNumber = POWER_EXCEEDS_LIMIT;
                                    }
			            
	        		      break;
								}
					      case 5://�������׮ ------OK---�в�ǹ����������
								{
									if((6 == pSystemStatusList->cpValue )||(9 == pSystemStatusList->cpValue)){//��ǹ������������
										executingResults = false;
									    errorNumber = 0;
									}
									else{
								        executingResults = true;
                                        endWork = END_WORK_REBOOT;								
									}									
							        break;
								}
					      case 6://���ù���ģʽ------OK---�Զ�������ģʽ
								{
									if(CHARGING == pSystemStatusList->workingState_flag){
										pSystemStatusList->remoteStartOrStopFlag = canStartCharging ;
									}
								   pSystemStatusList->workingMode = pFrame->aucDataTmp[4];
								   //д��FLASH
								   writeWorkingMode(pSystemStatusList->workingMode);
			                       executingResults = true;
					               break;
								}
						  case 7://�л�����ģʽ ----OK-----ֻ�п���״̬����������
								{
									
									printf("zhixing shengji1 \n");
									if(FREE == pSystemStatusList->workingState_flag){//����״̬����������
									    //��д������־Ϊ����������
										printf("kong xian1 \n");
										writeIAPUpgradeFlag(UPGRADE_REQUEST);
										printf("kong xian 2\n");
										endWork = END_WORK_REBOOT;
										executingResults = true;
										printf("kong xian \n");
									}else{//�ظ��쳣
									
								        executingResults = false;
									    errorNumber = REFUSE_UPGRADE;
										printf("buyunxu shengji \n");
                                    }		
                                    									
					                break;
								}
					      case 15://���ò���
								{
								 baudRate_buf = 	*(uint32_t*)(pFrame->aucDataTmp + 4);
								 if(baudRate_buf == 9600 || \
								    baudRate_buf == 0 || \
								    baudRate_buf == 4800 || \
									baudRate_buf == 2400 || \
									baudRate_buf == 19200 || \
									baudRate_buf == 115200 ){
										if(baudRate_buf != 0){
											pSystemStatusList->parameterList.BaudRate = baudRate_buf;//ͨ������
										}
										
										
                                        maxPower_buf = *(uint32_t*)(pFrame->aucDataTmp + 8);//����繦��
										if (maxPower_buf == POWER_MAX || maxPower_buf == 0 ||maxPower_buf == 3520){
											if(maxPower_buf != 0){
												pSystemStatusList->parameterList.maxPower = maxPower_buf;
											}
											if(pFrame->aucDataTmp[12] != '0' || \
											pFrame->aucDataTmp[13] != '0' || \
											pFrame->aucDataTmp[14] != '0' || \
											pFrame->aucDataTmp[15] != '0' ){
											   *(uint32_t*)pSystemStatusList->parameterList.VN = *(uint32_t*)(pFrame->aucDataTmp + 12);//�汾��

											}
										
										
										    if(pFrame->aucDataTmp[16] != '0' || \
											pFrame->aucDataTmp[17] != '0' || \
											pFrame->aucDataTmp[18] != '0' || \
											pFrame->aucDataTmp[19] != '0' || \
											pFrame->aucDataTmp[20] != '0' || \
											pFrame->aucDataTmp[21] != '0' ){
									            *(pAdress6)pSystemStatusList->parameterList.PileAddress = *(pAdress6)(pFrame->aucDataTmp + 16);//д��׮��ַ

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
				  
		
		{//�ظ�
				      if(executingResults == true){//����
					  printf("zhengchang chuli \n");
									framePackBuf.ctrl_C = 0x94;
                                    framePackBuf.data_L = 0x0;
						  }
					  else{//�쳣
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


		//���
		if(0 == pack_d07_frame_by_data(&framePackBuf, frameBuf, &frameLen)){
		//����
			RS485_SendStr_length(frameBuf,frameLen);	
			printf("fa song \n");
		}else{
		printf("\n���ʧ��\n");
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
				listen(&outpFrame, pSystemStatusList);//��������������
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
//�ĵ�IAP��
SCB->VTOR = FLASH_BASE | 0x4000;//�ض����ж�������,�ó�����FLASH�е���ʼ��ַΪ��0x8004000��
#if ZFZ 
{
	StatusList sysStatusList;
	StatusList * pSysStatusList = &sysStatusList ;
	SysInit(pSysStatusList);/*��ʼ������*/

	while(1){
	  GetSystemStatus(pSysStatusList);/*ˢ����Ϣ*/
		communication(pSysStatusList);/*ͨ�Ŵ���*/
		ExceptionHanding(pSysStatusList);/*�쳣����*/
		WorkingStateChange(pSysStatusList);/*����״̬�л�*/
		DisplayProcessing(pSysStatusList);/*��ʾ����*/
	//�ӿ��Ź�
	//�����ж϶�FLASH��Ӱ�죿
	}

}
#endif	
}

/*********************************************END OF FILE**********************/
