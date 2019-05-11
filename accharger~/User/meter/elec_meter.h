#ifndef __ELEC_METER_H
#define __ELEC_METER_H

#include "stm32f10x.h"
#include <stdio.h>


/*�Ĵ�������-��ͷ*******************************/
//���������Ĵ��� READ ONLY
#define SPL_I1		      0x00      //����ͨ��1 ��ADC ��������
#define SPL_I2		      0x01      //����ͨ��2 ��ADC ��������
#define SPL_U   		    0x02      //��ѹͨ����ADC ��������
#define SPL_P   		    0x03      //�й����ʲ�������
#define SPL_Q		        0x04      //�޹����ʲ�������
#define SPL_S		        0x05      //���ڹ��ʲ�������
#define RMS_I1		      0x06      //����ͨ��1 ����Чֵ
#define RMS_I2		      0x07      //����ͨ��2 ����Чֵ
#define RMS_U			      0x08      //��ѹͨ������Чֵ
#define FREQ_U		      0x09      //��ѹƵ��
#define POWERP1		      0x0A      //��һͨ���й�����
#define POWERQ1			    0x0B      //��һͨ���޹�����
#define POWER_S	        0x0C      //���ڹ���
#define ENERGY_P			  0x0D      //�й�����
#define ENERGY_Q      	0x0E      //�޹�����
#define ENERGY_S   	    0x0F      //��������
#define POWERP2		      0x10      //�ڶ�ͨ���й�����
#define POWERQ2		      0x11      //�ڶ�ͨ���޹�����
#define I1ANGLE		      0x12      //����ͨ��1 ���ѹͨ���н�
#define I2ANGLE		      0x13      //����ͨ��2 ���ѹͨ���н�
#define TEMPDATA		    0x14      //�¶Ȳ�������

#define BACKUPDATA      0x16      //ͨѶ���ݱ��ݼĴ���
#define COMCHECKSUM     0x17      //ͨѶУ��ͼĴ���
#define SUMCHECKSUM     0x18      //У�����У��ͼĴ���
#define EMUSR           0x19      //EMU ״̬�Ĵ���

//У������Ĵ���
#define EMUIE           0x30      //EMU�ж�ʹ�ܼĴ���
#define EMUIF           0x31      //EMU�жϱ�־�Ĵ���
#define WPREG           0x32      //д�����Ĵ���
#define SRSTREG         0x33      //�����λ�Ĵ���

#define EMUCFG          0x40      //EMU���üĴ���
#define FREQCFG         0x41      //ʱ��/����Ƶ�����üĴ���
#define MODULEEN        0x42      //EMUģ��ʹ�ܼĴ���
#define ANAEN           0x43      //ģ��ģ��ʹ�ܼĴ���
#define IOCFG           0x45      //IO������üĴ���

#define GP1             0x50      //ͨ��1 ���й�����У��
#define GQ1             0x51      //ͨ��1 ���޹�����У��
#define GS1             0x52      //ͨ��1 �����ڹ���У��
#define PHASE1          0x53      //ͨ��1 ����λУ�����Ʋ����㷽ʽ��
#define GP2             0x54      //ͨ��2 ���й�����У��
#define GQ2             0x55      //ͨ��2 ���޹�����У��
#define GS2             0x56      //ͨ��2 �����ڹ���У��
#define PHASE2          0x57      //ͨ��2 ����λУ�����Ʋ����㷽ʽ��
#define QPHSCAL         0x58      //�޹���λ����
#define ADCCON          0x59      //ADC ͨ������ѡ��
#define ALLGAIN         0x5A      //3 ��ADC ͨ����������Ĵ�������Ҫ���Vref�仯�����ADC �����̱仯
#define I2GAIN          0x5B      //����ͨ��2 ���油��
#define I1OFF           0x5C      //����ͨ��1 ��ƫ��У��
#define I2OFF           0x5D      //����ͨ��2 ��ƫ��У��
#define UOFF            0x5E      //��ѹͨ����ƫ��У��
#define PQSTART         0x5F      //�𶯹�������
#define RMSSTART        0x60      //��Чֵ����ֵ���üĴ���
#define HFCONST         0x61      //�������Ƶ������
#define CHK             0x62      //�Ե���ֵ����
#define IPTAMP          0x63      //�Ե��������ֵ
#define UCONST          0x64      //ʧѹ����²�������ĵ�ѹ��������Ե�
#define P1OFFSET				0x65      //ͨ��1 �й�����ƫִУ��������Ϊ8bit ����
#define P2OFFSET				0x66      //ͨ��2 �й�����ƫִУ��������Ϊ8bit ����
#define Q1OFFSET				0x67      //ͨ��1 �޹�����ƫִУ��������Ϊ8bit ����
#define Q2OFFSET				0x68      //ͨ��2 �޹�����ƫִУ��������Ϊ8bit ����
#define I1RMSOFFSET     0x69      //ͨ��1 ��Чֵ�����Ĵ�����Ϊ8bit �޷�����
#define I2RMSOFFSET     0x6A      //ͨ��2 ��Чֵ�����Ĵ�����Ϊ8bit �޷�����
#define URMSOFFSET      0x6B      //��ѹͨ����ͨ��3����Чֵ�����Ĵ�����Ϊ8bit�޷�����
#define ZCROSSCURRENT   0x6C      //����������ֵ���üĴ���
#define GPHS1           0x6D      //ͨ��1 ����λУ����PQ ��ʽ��
#define GPHS2           0x6E      //ͨ��1 ����λУ����PQ ��ʽ��
#define PFCNT           0x6F      //�����й��������
#define QFCNT           0x70      //�����޹��������
#define SFCNT           0x71      //�����޹��������



#define U19TRANS(x)        (x&0x7ffff )
#define W19TRANS(x)        (x<262144? x:((x&0x7ffff)-524288) )          
#define U24TRANS(x)        (x&0xffffff )
#define W24TRANS(x)        (x<8388608? x:((x&0xffffff)-16777216))



/* WIP(busy)��־��FLASH�ڲ�����д�� */
#define WIP_Flag                  0x01
#define Dummy_Byte                0xFF
/*�����-��β*******************************/


/*SPI�ӿڶ���-��ͷ****************************/
#define      METER_SPIx                        SPI2
#define      METER_SPI_APBxClock_FUN          RCC_APB1PeriphClockCmd
#define      METER_SPI_CLK                     RCC_APB1Periph_SPI2

//CS(NSS)���� Ƭѡѡ��ͨGPIO����
#define      METER_SPI_CS_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      METER_SPI_CS_CLK                  RCC_APB2Periph_GPIOB    
#define      METER_SPI_CS_PORT                 GPIOB
#define      METER_SPI_CS_PIN                  GPIO_Pin_12

//SCK����
#define      METER_SPI_SCK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      METER_SPI_SCK_CLK                 RCC_APB2Periph_GPIOB   
#define      METER_SPI_SCK_PORT                GPIOB   
#define      METER_SPI_SCK_PIN                 GPIO_Pin_13
//MISO����
#define      METER_SPI_MISO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      METER_SPI_MISO_CLK                RCC_APB2Periph_GPIOB    
#define      METER_SPI_MISO_PORT               GPIOB 
#define      METER_SPI_MISO_PIN                GPIO_Pin_14
//MOSI����
#define      METER_SPI_MOSI_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      METER_SPI_MOSI_CLK                RCC_APB2Periph_GPIOB    
#define      METER_SPI_MOSI_PORT               GPIOB 
#define      METER_SPI_MOSI_PIN                GPIO_Pin_15

#define  		SPI_METER_CS_LOW()     						GPIO_ResetBits( METER_SPI_CS_PORT, METER_SPI_CS_PIN )
#define  		SPI_METER_CS_HIGH()    						GPIO_SetBits( METER_SPI_CS_PORT, METER_SPI_CS_PIN )

/*SPI�ӿڶ���-��β****************************/

/*�ȴ���ʱʱ��*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))



void SPI_METER_Init(void);
u32 MeterRead(u8 address);
u8 MeterWrite(u8 address ,u32 data);
void SPI_WRPROTECT_OPEN(u8 address);
void SPI_WRPROTECT1_OPEN(void);
void SPI_WRPROTECT2_OPEN(void);
u8 SPI_METER_SendByte(u8 byte);





#endif /* __SPI_FLASH_H */
