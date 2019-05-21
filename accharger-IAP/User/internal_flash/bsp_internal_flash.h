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

typedef enum 
{
  FAILED = 0, 
  PASSED = !FAILED
} TestStatus;

typedef struct{char a[6];} PileAddress ;


#define       APPLICATIONADDRESS       ((uint32_t)0x08004000) //APP的起始地址 页16到页39
#define       APP_CACHE_ADDRESS       ((uint32_t)0x0800A000) //APP的bin文件缓冲起始地址 页40到页64
#define UPGRADEDATASIZE 128 //升级帧中有效数据的字节数
#define APP_SIZE_PAGES 24 //APP区所占页的页数
#define COPY_CELL_SIZE 4 //拷贝程序时，每次读写存储单元的大小
#define APP_NOT_AVAILABLE 0 //app区不可用
#define APP_AVAILABLE 1 //app区可用



#define UPGRADE_FREE 1 //升级状态为空闲，无升级请求
#define UPGRADE_REQUEST 0 //有升级请求
#define UPGRADE_RECEIVING 3 //数据接收中
#define UPGRADE_RECEIVE_COMPLETE 4 //数据接收完成
#define UPGRADE_COPYING 5 //程序拷贝中




#define WRITE_FLAG_START_ADDR  ((uint32_t)0x08003000) //数据区的起始地址
#define WRITE_FLAG_END_ADDR    ((uint32_t)0x08004000) //数据区的结束地址
#define BYTESIZE_USEDDATAFLASH 128 //数据区已使用的大小
//前8个字节是留给IAP的
#define LABEL_IAPUpgradeFlag  (0) //这个标志位在存储页中的字节标号
#define LABEL_APP_STATE_Flag  (1)
#define LABEL_BAUD_RATE_Flag  (2) //波特率的起始地址标号 一共占用4个字节 [2,5]
#define LABEL_ADDRESS_Flag  (6) //地址占用6个字节 [6,11]
//#define LABEL_workingMode (16) //工作模式的字节标号
//#define LABEL_settingList (20) //这个结构体在存储页中的字节起始标号

#define DefaultBaudRate  (9600)

//函数
uint8_t getIAPUpgradeFlag(void);
void writeIAPUpgradeFlag(uint8_t data);
uint8_t getAPPState(void);
void writeAPPStateFlag(uint8_t data);
void writeFlash(uint32_t numFrame,uint32_t sumFrame,uint8_t * pstart);
void CopyFromBufToRunArea(void);
uint32_t getBaudRate(void);
PileAddress getAddress(void);
#endif /* __INTERNAL_FLASH_H */

