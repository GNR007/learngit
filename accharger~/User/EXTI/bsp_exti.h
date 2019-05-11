#ifndef __EXTI_H
#define	__EXTI_H


#include "stm32f10x.h"
#include "./EXTI/bsp_exti.h"

//Òý½Å¶¨Òå
#define KEY1_INT_GPIO_PORT         GPIOB
#define KEY1_INT_GPIO_CLK          (RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO)
#define KEY1_INT_GPIO_PIN          GPIO_Pin_7
#define KEY1_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOB
#define KEY1_INT_EXTI_PINSOURCE    GPIO_PinSource7
#define KEY1_INT_EXTI_LINE         EXTI_Line7
#define KEY1_INT_EXTI_IRQ          EXTI9_5_IRQn

#define KEY1_IRQHandler            EXTI9_5_IRQHandler

#if 0
#define KEY2_INT_GPIO_PORT         GPIOC
#define KEY2_INT_GPIO_CLK          (RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO)
#define KEY2_INT_GPIO_PIN          GPIO_Pin_13
#define KEY2_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOC
#define KEY2_INT_EXTI_PINSOURCE    GPIO_PinSource13
#define KEY2_INT_EXTI_LINE         EXTI_Line13
#define KEY2_INT_EXTI_IRQ          EXTI15_10_IRQn

#define KEY2_IRQHandler            EXTI15_10_IRQHandler
#endif

void EXTI_Key_Config(void);


#endif /* __EXTI_H */
