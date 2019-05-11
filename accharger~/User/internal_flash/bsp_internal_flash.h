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


#define UPGRADE_FREE 1 //����״̬Ϊ���У�����������
#define UPGRADEING 2 //������
#define UPGRADE_REQUEST 0 //����������
#define UPGRADEFLAGADRESS ((uint32_t)0x08003000) //������־���ڵ�ҳ����ʼ��ַ
#define WRITE_FLAG_START_ADDR  ((uint32_t)0x08003000) //�洢��FLASH�е��������ڵ��������ʼ��ַ
#define WRITE_FLAG_END_ADDR    ((uint32_t)0x08003400) //�洢��FLASH�е��������ڵ�����Ľ�����ַ
#define UPGRADEDATASIZE 128 //����֡����Ч���ݵ��ֽ���
#define BYTESIZE_USEDDATAFLASH 128 //FLASH�д洢���ݵ�������ʹ�õĴ�С

//flash��1024���洢���ݵ��ֽ�������ʹ�õģ�
#define LABEL_IAPUpgradeFlag  (0) //�����־λ�ڴ洢ҳ�е��ֽڱ��
#define LABEL_BAUD_RATE_Flag  (2)																			   
#define LABEL_ADDRESS_Flag  (6) //��ַռ��6���ֽ� [6,11]													   

#define LABEL_workingMode (16) //����ģʽ���ֽڱ��
#define LABEL_settingList (20) //����ṹ���ڴ洢ҳ�е��ֽ���ʼ���

//�ֽڱ��2������
typedef enum 
{
	FAILED = 0, 
  PASSED = !FAILED
} TestStatus;

#if 1
typedef struct List_set{
char SN[5]; //�ʲ���
char PileAddress[6]; //׮��ַ
uint8_t flag_default ; //�Ƿ�ֵĬ�ϲ����ı�־��0xff ��ʾ��ֵĬ�ϲ���
char VN[4];//�汾��
int vMax;//��ѹ��
int vMin;//Ƿѹ��
int curMax;//������
int freqMax;//�������Ƶ��
int freqMin;//�������Ƶ��
float volRmsGain;//��ѹ����
float cur2RmsGain;//��������
uint32_t BaudRate; //ͨѶ���ʣ�������
uint32_t maxPower; //����繦��
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

