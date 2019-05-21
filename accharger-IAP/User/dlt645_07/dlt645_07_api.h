#ifndef __BSP_DLT_H
#define	__BSP_DLT_H


#include "stm32f10x.h"
//校验和检验的是要发送和接收的数据
#define D07_FRAME_LEN_MIN    12  /* DLT645 2007 最小帧字节数 */
#define ADDRESSLEN 6  //地址域的长度
#define FRAME_DATA_ID 4 //数据域标识的长度为4
#define D07_DATA_MAX         256   /* dlt645 2007 最大数据域字节数 */

#define CUR_FRAME_START1 0 //帧起始符1所在位置
#define CUR_FRAME_START2 7 //帧起始符2所在位置
#define CUR_FRAME_ADDRESS 1 //帧地址域起始位置
#define CUR_FRAME_C 8 //帧控制码位置
#define CUR_FRAME_L 9 //帧数据域长度字节位置
#define CUR_FRAME_DATA 10 //帧数据域开始的位置
#define CUR_FRAME_CS(x) (ADDRESSLEN + 4 + x) //帧CS域位置，x为数据域的长度
#define CUR_FRAME_END(x) ( ADDRESSLEN +5 +x ) //帧结束字符位置

typedef uint8_t bool;
#define true 1
#define false 0

//帧
typedef struct{
	uint8_t frame[ D07_DATA_MAX + D07_FRAME_LEN_MIN ];
	uint8_t frameLen;
}S_D07_FRAME;


//打包用的帧的结构体
typedef struct
{
    uint8_t Address_receiver[6];     /* 接收方的地址 */
    uint8_t  ctrl_C;    /* 控制码 */
    uint8_t  data_L;     /* 数据域总的字节数 */
    uint8_t  data[D07_DATA_MAX];          /* 数据域起始地址 */
}S_D07_PACK_FRAME;


/* 解包用的帧信息数据结构 */
typedef struct{
    uint8_t                 ctrl_c;           /* 控制码*/
    uint32_t                ruler_id;         /* 规约ID */
    unsigned short          data_len;         /* 数据域长 */
    unsigned short          frame_len;         /* 整个帧长*/
    char                 address[6];     //6位地址	
	uint8_t                 aucDataTmp[D07_DATA_MAX];/*整个数据域的数据*/
}S_D07_UNPACK;

//从缓冲区拿到完整的帧
int get_d07_first_valid_frame(uint8_t *pBuf,         // 要解析的buf
                              uint16_t usLenBuf,           // 要解析的buf字节数
                              uint8_t  **ppFirstPos,       // 输出帧buffer
                              uint16_t *pusLenFrame);      // 输出Frame字节数

//
uint32_t pack_d07_frame_by_data(S_D07_PACK_FRAME *inPara, uint8_t *outBuffer, uint32_t *outLength);

uint32_t unpack_d07_frame(void *inpBuffer, uint32_t inBufferLength, S_D07_UNPACK *outpFrame);
#endif
