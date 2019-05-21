#ifndef __INTERNAL_FLASH_H
#define	__INTERNAL_FLASH_H
#include <stdint.h>
#include "stm32f10x.h"

/* STM32��������Ʒÿҳ��С2KByte���С�С������Ʒÿҳ��С1KByte */
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


#define       APPLICATIONADDRESS       ((uint32_t)0x08004000) //APP����ʼ��ַ ҳ16��ҳ39
#define       APP_CACHE_ADDRESS       ((uint32_t)0x0800A000) //APP��bin�ļ�������ʼ��ַ ҳ40��ҳ64
#define UPGRADEDATASIZE 128 //����֡����Ч���ݵ��ֽ���
#define APP_SIZE_PAGES 24 //APP����ռҳ��ҳ��
#define COPY_CELL_SIZE 4 //��������ʱ��ÿ�ζ�д�洢��Ԫ�Ĵ�С
#define APP_NOT_AVAILABLE 0 //app��������
#define APP_AVAILABLE 1 //app������



#define UPGRADE_FREE 1 //����״̬Ϊ���У�����������
#define UPGRADE_REQUEST 0 //����������
#define UPGRADE_RECEIVING 3 //���ݽ�����
#define UPGRADE_RECEIVE_COMPLETE 4 //���ݽ������
#define UPGRADE_COPYING 5 //���򿽱���




#define WRITE_FLAG_START_ADDR  ((uint32_t)0x08003000) //����������ʼ��ַ
#define WRITE_FLAG_END_ADDR    ((uint32_t)0x08004000) //�������Ľ�����ַ
#define BYTESIZE_USEDDATAFLASH 128 //��������ʹ�õĴ�С
//ǰ8���ֽ�������IAP��
#define LABEL_IAPUpgradeFlag  (0) //�����־λ�ڴ洢ҳ�е��ֽڱ��
#define LABEL_APP_STATE_Flag  (1)
#define LABEL_BAUD_RATE_Flag  (2) //�����ʵ���ʼ��ַ��� һ��ռ��4���ֽ� [2,5]
#define LABEL_ADDRESS_Flag  (6) //��ַռ��6���ֽ� [6,11]
//#define LABEL_workingMode (16) //����ģʽ���ֽڱ��
//#define LABEL_settingList (20) //����ṹ���ڴ洢ҳ�е��ֽ���ʼ���

#define DefaultBaudRate  (9600)

//����
uint8_t getIAPUpgradeFlag(void);
void writeIAPUpgradeFlag(uint8_t data);
uint8_t getAPPState(void);
void writeAPPStateFlag(uint8_t data);
void writeFlash(uint32_t numFrame,uint32_t sumFrame,uint8_t * pstart);
void CopyFromBufToRunArea(void);
uint32_t getBaudRate(void);
PileAddress getAddress(void);
#endif /* __INTERNAL_FLASH_H */

