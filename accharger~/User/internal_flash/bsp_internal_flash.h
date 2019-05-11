#ifndef __INTERNAL_FLASH_H
#define	__INTERNAL_FLASH_H
#include <stdint.h>
#include "stm32f10x.h"

/* STM32大容量产品每页大小2KByte，中、小容量产品每页大小1KByte */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)	//2048
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)	//1024
#endif


#define UPGRADE_FREE 1 //升级状态为空闲，无升级请求
#define UPGRADEING 2 //升级中
#define UPGRADE_REQUEST 0 //有升级请求
#define UPGRADEFLAGADRESS ((uint32_t)0x08003000) //升级标志所在的页的起始地址
#define WRITE_FLAG_START_ADDR  ((uint32_t)0x08003000) //存储在FLASH中的数据所在的区域的起始地址
#define WRITE_FLAG_END_ADDR    ((uint32_t)0x08003400) //存储在FLASH中的数据所在的区域的结束地址
#define UPGRADEDATASIZE 128 //升级帧中有效数据的字节数
#define BYTESIZE_USEDDATAFLASH 128 //FLASH中存储数据的区域已使用的大小

//flash中1024个存储数据的字节是这样使用的：
#define LABEL_IAPUpgradeFlag  (0) //这个标志位在存储页中的字节标号
#define LABEL_BAUD_RATE_Flag  (2)																			   
#define LABEL_ADDRESS_Flag  (6) //地址占用6个字节 [6,11]													   

#define LABEL_workingMode (16) //工作模式的字节标号
#define LABEL_settingList (20) //这个结构体在存储页中的字节起始标号

//字节标号2不可用
typedef enum 
{
	FAILED = 0, 
  PASSED = !FAILED
} TestStatus;

#if 1
typedef struct List_set{
char SN[5]; //资产码
char PileAddress[6]; //桩地址
uint8_t flag_default ; //是否赋值默认参数的标志，0xff 表示赋值默认参数
char VN[4];//版本号
int vMax;//过压点
int vMin;//欠压点
int curMax;//过流点
int freqMax;//最高允许频率
int freqMin;//最低允许频率
float volRmsGain;//电压增益
float cur2RmsGain;//电流增益
uint32_t BaudRate; //通讯速率，波特率
uint32_t maxPower; //最大充电功率
}SettingList;
#endif


int InternalFlash_Test(void);
void UpdateSettingList(SettingList * settingList);
void GetSetting(SettingList * settingList);
void writeIAPUpgradeFlag(uint8_t data);
void readWorkingMode(uint8_t * data);
void writeWorkingMode(uint8_t data);
//void writeBaudRate(uint32_t BR);
//void writeIAPAddress(SettingList * settingList);
#endif /* __INTERNAL_FLASH_H */

