/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ����led
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-ָ���� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "stm32f10x.h"
#include "bsp_led.h"
#include "bsp_GeneralTim.h"  

typedef uint8_t bool;
#define true 1
#define false 0

#define WAITING_FOR_PLUGIN 2000	// ��λ������
#define DEFAULT_PWM_RATIO 53	// ��λ��ǧ��֮һ

#define PB_LED_RUN		GPIO_Pin_12
#define PA_AC_CONTACTOR	GPIO_Pin_1
#define PA_CC			GPIO_Pin_4
#define PA_CP_PWM		GPIO_Pin_7
#define PA_CP_DET0		GPIO_Pin_5
#define PB_CP_DET1		GPIO_Pin_0
 
void DelayUs(uint32_t us)
{
	uint32_t i;
#if 1
	for (i = 0; i < us; i++) {
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();
	}
#else
	i = 8 * us;
	while(i--);
#endif
}

void DelayMs(uint32_t ms)
{
	uint32_t i = 1000 * ms;
	DelayUs(i);
}

void GpioConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = PB_LED_RUN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
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
}


/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
// ARR ���Զ���װ�ؼĴ�����ֵ
// CLK_cnt����������ʱ�ӣ����� Fck_int / (psc+1) = 72M/(psc+1)
// PWM �źŵ����� T = ARR * (1/CLK_cnt) = ARR*(PSC+1) / 72M
// ռ�ձ�P=CCR/(ARR+1)

void PwmConfig(void)
{
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

void PwmRatio(uint8_t ratio)
{
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

void PwmCtrl(bool enable)
{
	if (enable) {
		PwmRatio(DEFAULT_PWM_RATIO);
	} else {
		PwmRatio(100);
	}
}


void SetLedRun(bool on)
{
	if (!on) {
		GPIO_SetBits(GPIOB, PB_LED_RUN);
	} else {
		GPIO_ResetBits(GPIOB, PB_LED_RUN);
	}
}

void SetAcContactor(bool on)
{
	if (on) {
		GPIO_SetBits(GPIOA, PA_AC_CONTACTOR);
	} else {
		GPIO_ResetBits(GPIOA, PA_AC_CONTACTOR);
	}
}

uint8_t GetCpStatus(void)
{
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

void LedRunFlicker(uint32_t periodMs)
{
	uint32_t period = periodMs*500;
	SetLedRun(true);
	DelayUs(period);
	SetLedRun(false);
	DelayUs(period);
}

int main(void)
{
	uint8_t cnt = 0;
	GpioConfig();
	PwmConfig();
	
	SetAcContactor(false);
	PwmCtrl(false);

	while(1) {
		cnt = 0;
		if (GetCpStatus() == 12) { //����
			SetAcContactor(false);
			LedRunFlicker(1000);
			continue;
		}
		
		SetLedRun(true);
		
		if (GetCpStatus() == 9) { // ��S2���أ���ǹ
			DelayMs(WAITING_FOR_PLUGIN);
			if (GetCpStatus() == 12) continue;
			
			PwmCtrl(true); // ����PWM

			while (GetCpStatus() == 9) DelayMs(50);
			if (GetCpStatus() == 12) continue;

			SetAcContactor(true);			
			
			while (cnt < 5) {
				if (GetCpStatus() != 6) {
					cnt++;
					DelayMs(1);
					continue;
				}
				
				cnt = 0;
				LedRunFlicker(25);
			}

			SetAcContactor(false);
			PwmCtrl(false);
			SetLedRun(false);
			while (GetCpStatus() != 12) DelayMs(50);
		}
		else if (GetCpStatus() == 6) { // ��S2���ز�ǹ
			DelayMs(WAITING_FOR_PLUGIN);
			if (GetCpStatus() != 6) continue;
			
			PwmCtrl(true); // ����PWM

			SetAcContactor(true);
			
			while (1) { // �����
				if (GetCpStatus() != 6) {
					cnt++;
				} else {
					cnt = 0;
				}
				
				if (cnt > 5) {
					break;
				}
				
				LedRunFlicker(5);
			}
			
			SetAcContactor(false);
			PwmCtrl(false);
			SetLedRun(false);
			while (GetCpStatus() != 12) DelayMs(50);
		}
	}
}

/*********************************************END OF FILE**********************/
