#ifndef __232_USART_H
#define	__232_USART_H


#include "stm32f10x.h"
#include <stdio.h>

/** 
  * ���ں궨�壬��ͬ�Ĵ��ڹ��ص����ߺ�IO��һ������ֲʱ��Ҫ�޸��⼸����
	* 1-�޸�����ʱ�ӵĺ꣬uart1���ص�apb2���ߣ�����uart���ص�apb1����
	* 2-�޸�GPIO�ĺ�
  */
	
// ����1-USART1
#define  DEBUG_232_USARTx                   USART1
#define  DEBUG_232_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_232_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_232_USART_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  DEBUG_232_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DEBUG_232_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_232_USART_TX_GPIO_PORT       GPIOA   
#define  DEBUG_232_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_232_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_232_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_232_USART_IRQ                USART1_IRQn
#define  DEBUG_232_USART_IRQHandler         USART1_IRQHandler


void USART_232_Config(void);

void Usart_232_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);

void Usart_232_SendString( USART_TypeDef * pUSARTx, char *str);

void Usart_232_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif /* __USART_H */
