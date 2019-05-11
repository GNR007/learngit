#ifndef __ELEC_METER_H
#define __ELEC_METER_H

#include "stm32f10x.h"
#include <stdio.h>


/*寄存器定义-开头*******************************/
//计量参数寄存器 READ ONLY
#define SPL_I1		      0x00      //电流通道1 的ADC 采样数据
#define SPL_I2		      0x01      //电流通道2 的ADC 采样数据
#define SPL_U   		    0x02      //电压通道的ADC 采样数据
#define SPL_P   		    0x03      //有功功率波形数据
#define SPL_Q		        0x04      //无功功率波形数据
#define SPL_S		        0x05      //视在功率波形数据
#define RMS_I1		      0x06      //电流通道1 的有效值
#define RMS_I2		      0x07      //电流通道2 的有效值
#define RMS_U			      0x08      //电压通道的有效值
#define FREQ_U		      0x09      //电压频率
#define POWERP1		      0x0A      //第一通道有功功率
#define POWERQ1			    0x0B      //第一通道无功功率
#define POWER_S	        0x0C      //视在功率
#define ENERGY_P			  0x0D      //有功能量
#define ENERGY_Q      	0x0E      //无功能量
#define ENERGY_S   	    0x0F      //视在能量
#define POWERP2		      0x10      //第二通道有功功率
#define POWERQ2		      0x11      //第二通道无功功率
#define I1ANGLE		      0x12      //电流通道1 与电压通道夹角
#define I2ANGLE		      0x13      //电流通道2 与电压通道夹角
#define TEMPDATA		    0x14      //温度测试数据

#define BACKUPDATA      0x16      //通讯数据备份寄存器
#define COMCHECKSUM     0x17      //通讯校验和寄存器
#define SUMCHECKSUM     0x18      //校表参数校验和寄存器
#define EMUSR           0x19      //EMU 状态寄存器

//校表参数寄存器
#define EMUIE           0x30      //EMU中断使能寄存器
#define EMUIF           0x31      //EMU中断标志寄存器
#define WPREG           0x32      //写保护寄存器
#define SRSTREG         0x33      //软件复位寄存器

#define EMUCFG          0x40      //EMU配置寄存器
#define FREQCFG         0x41      //时钟/更新频率配置寄存器
#define MODULEEN        0x42      //EMU模块使能寄存器
#define ANAEN           0x43      //模拟模块使能寄存器
#define IOCFG           0x45      //IO输出配置寄存器

#define GP1             0x50      //通道1 的有功功率校正
#define GQ1             0x51      //通道1 的无功功率校正
#define GS1             0x52      //通道1 的视在功率校正
#define PHASE1          0x53      //通道1 的相位校正（移采样点方式）
#define GP2             0x54      //通道2 的有功功率校正
#define GQ2             0x55      //通道2 的无功功率校正
#define GS2             0x56      //通道2 的视在功率校正
#define PHASE2          0x57      //通道2 的相位校正（移采样点方式）
#define QPHSCAL         0x58      //无功相位补偿
#define ADCCON          0x59      //ADC 通道增益选择
#define ALLGAIN         0x5A      //3 个ADC 通道整体增益寄存器，主要针对Vref变化引起的ADC 满量程变化
#define I2GAIN          0x5B      //电流通道2 增益补偿
#define I1OFF           0x5C      //电流通道1 的偏置校正
#define I2OFF           0x5D      //电流通道2 的偏置校正
#define UOFF            0x5E      //电压通道的偏置校正
#define PQSTART         0x5F      //起动功率设置
#define RMSSTART        0x60      //有效值启动值设置寄存器
#define HFCONST         0x61      //输出脉冲频率设置
#define CHK             0x62      //窃电阈值设置
#define IPTAMP          0x63      //窃电检测电流域值
#define UCONST          0x64      //失压情况下参与计量的电压，断相仿窃电
#define P1OFFSET				0x65      //通道1 有功功率偏执校正参数，为8bit 补码
#define P2OFFSET				0x66      //通道2 有功功率偏执校正参数，为8bit 补码
#define Q1OFFSET				0x67      //通道1 无功功率偏执校正参数，为8bit 补码
#define Q2OFFSET				0x68      //通道2 无功功率偏执校正参数，为8bit 补码
#define I1RMSOFFSET     0x69      //通道1 有效值补偿寄存器，为8bit 无符号数
#define I2RMSOFFSET     0x6A      //通道2 有效值补偿寄存器，为8bit 无符号数
#define URMSOFFSET      0x6B      //电压通道（通道3）有效值补偿寄存器，为8bit无符号数
#define ZCROSSCURRENT   0x6C      //电流过零阈值设置寄存器
#define GPHS1           0x6D      //通道1 的相位校正（PQ 方式）
#define GPHS2           0x6E      //通道1 的相位校正（PQ 方式）
#define PFCNT           0x6F      //快速有功脉冲计数
#define QFCNT           0x70      //快速无功脉冲计数
#define SFCNT           0x71      //快速无功脉冲计数



#define U19TRANS(x)        (x&0x7ffff )
#define W19TRANS(x)        (x<262144? x:((x&0x7ffff)-524288) )          
#define U24TRANS(x)        (x&0xffffff )
#define W24TRANS(x)        (x<8388608? x:((x&0xffffff)-16777216))



/* WIP(busy)标志，FLASH内部正在写入 */
#define WIP_Flag                  0x01
#define Dummy_Byte                0xFF
/*命令定义-结尾*******************************/


/*SPI接口定义-开头****************************/
#define      METER_SPIx                        SPI2
#define      METER_SPI_APBxClock_FUN          RCC_APB1PeriphClockCmd
#define      METER_SPI_CLK                     RCC_APB1Periph_SPI2

//CS(NSS)引脚 片选选普通GPIO即可
#define      METER_SPI_CS_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      METER_SPI_CS_CLK                  RCC_APB2Periph_GPIOB    
#define      METER_SPI_CS_PORT                 GPIOB
#define      METER_SPI_CS_PIN                  GPIO_Pin_12

//SCK引脚
#define      METER_SPI_SCK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      METER_SPI_SCK_CLK                 RCC_APB2Periph_GPIOB   
#define      METER_SPI_SCK_PORT                GPIOB   
#define      METER_SPI_SCK_PIN                 GPIO_Pin_13
//MISO引脚
#define      METER_SPI_MISO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      METER_SPI_MISO_CLK                RCC_APB2Periph_GPIOB    
#define      METER_SPI_MISO_PORT               GPIOB 
#define      METER_SPI_MISO_PIN                GPIO_Pin_14
//MOSI引脚
#define      METER_SPI_MOSI_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      METER_SPI_MOSI_CLK                RCC_APB2Periph_GPIOB    
#define      METER_SPI_MOSI_PORT               GPIOB 
#define      METER_SPI_MOSI_PIN                GPIO_Pin_15

#define  		SPI_METER_CS_LOW()     						GPIO_ResetBits( METER_SPI_CS_PORT, METER_SPI_CS_PIN )
#define  		SPI_METER_CS_HIGH()    						GPIO_SetBits( METER_SPI_CS_PORT, METER_SPI_CS_PIN )

/*SPI接口定义-结尾****************************/

/*等待超时时间*/
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
