#ifndef _RS485_H
#define	_RS485_H

#include "stm32f10x.h"
#include <stdio.h>


/*USART�š�ʱ�ӡ�������*/
#define RS485_USART                             USART2
#define RS485_USART_CLK                         RCC_APB1Periph_USART2
//#define RS485_USART_BAUDRATE                    115200

/*RX����*/
#define RS485_USART_RX_GPIO_PORT                GPIOA
#define RS485_USART_RX_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define RS485_USART_RX_PIN                      GPIO_Pin_3

/*TX����*/
#define RS485_USART_TX_GPIO_PORT                GPIOA
#define RS485_USART_TX_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define RS485_USART_TX_PIN                      GPIO_Pin_2

/*�ж����*/
#define RS485_INT_IRQ                 						USART2_IRQn
#define RS485_IRQHandler                         USART2_IRQHandler


	/// ����ȷ����ʱ
static void RS485_delay(__IO u32 nCount)
{
	for(; nCount != 0; nCount--);
} 


/*debug*/

#define RS485_DEBUG_ON          1
#define RS485_DEBUG_ARRAY_ON   1
#define RS485_DEBUG_FUNC_ON    1
   
   
// Log define
#define RS485_INFO(fmt,arg...)           printf("<<-RS485-INFO->> "fmt"\n",##arg)
#define RS485_ERROR(fmt,arg...)          printf("<<-RS485-ERROR->> "fmt"\n",##arg)
#define RS485_DEBUG(fmt,arg...)          do{\
																					 if(RS485_DEBUG_ON)\
																					 printf("<<-RS485-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
																				 }while(0)

#define RS485_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(RS485_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-RS485-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define RS485_DEBUG_FUNC()               do{\
                                         if(RS485_DEBUG_FUNC_ON)\
                                         printf("<<-RS485-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)


void RS485_Config(uint32_t baudRate);
void RS485_SendByte(  uint8_t ch );
void RS485_SendStr_length( uint8_t *str,uint32_t strlen );
void RS485_SendString(  uint8_t *str);



void bsp_RS485_IRQHandler(void);
																			 
uint8_t *get_rebuff(uint16_t *len);
void clean_rebuff(void);
#endif /* _RS485_H */