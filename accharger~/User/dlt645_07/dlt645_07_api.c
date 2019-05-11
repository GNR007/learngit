#include "./dlt645_07/dlt645_07_api.h"

/*****************************************************************************
 函 数 名  : get_d07_first_valid_frame
 功能描述  : 从一段数据中找出第一个有效dlt645 2007帧的位置及长度 
 输入参数  : const uint8_t *pBuf    // 要解析的buf
             UINT16 usLenBuf      // 要解析的buf字节数
             uint8_t  **ppFirstPos  // 输出帧buffer
             UINT16 *pusLenFrame  // 输出Frame字节数
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2013年1月25日
    作    者   ：云青
    原    创   : liming
    修改内容   : 
*****************************************************************************/
/* 返回值类型 列表*/
typedef enum{
    E_D07_OK = 0,            /* 返回OK */
    E_D07_ERRO_NULL,        /* 指针为空 */
    E_D07_ERRO_UNKOWN_ID,    // 未知规约ID
    E_D07_ERRO_PARA_NONE,   /* 不存在 */
    E_D07_ERRO_PARA_OUT_RANGE,
    
    //check frame
    E_D07_ERRO_FRAME_UNCOMP,        /* 不完整的帧数据 */
    E_D07_ERRO_FRAME_0x68,          /* 测试帧帧起始符 0x68 的位置不对 */
    E_D07_ERRO_FRAME_CHECK_SUM,     /* 测试帧检验和不对 */
    E_D07_ERRO_FRAME_END_0x16,      /* 测试帧结束符 0x16 不对 */
    E_D07_ERRO_NONE_FRAME,          // 没有找到帧

}eD07Err;


//从缓冲区拿到完整的帧，输入帧所在的缓冲区，以及要解析的字节数，返回帧的起始地址以及整个帧的长度
int get_d07_first_valid_frame(uint8_t *pBuf,         // 要解析的buf
                              uint16_t usLenBuf,           // 要解析的buf字节数
                              uint8_t  **ppFirstPos,       // 输出帧buffer
                              uint16_t *pusLenFrame)      // 输出Frame字节数
{

    int i = 0, j = 0;
    uint16_t usLenLeft = 0; //剩余的没有遍历的字节
    uint8_t ucDataLen = 0;
    uint8_t ucCheckSum = 0;
    int nLen = 0;
    uint32_t nCheckSumPosStart, nCheckSumPos, nEndPos;
    
    if(!pBuf || !ppFirstPos || !pusLenFrame)//对参数进行安全检查
    {
        return E_D07_ERRO_NULL;
    }
    
    for(i = 0; i < usLenBuf; i++)//
    {
        usLenLeft = usLenBuf - i;

			if(usLenLeft < D07_FRAME_LEN_MIN)//在没有找到0x68之前若所剩字节数不足12，则认为这段内存中没有完整的帧
        {
            return E_D07_ERRO_FRAME_UNCOMP;//不完整的帧数据
        }

        if(pBuf[i] == 0x68)
        {
            
          if (0x68 == pBuf[i+CUR_FRAME_START2])
          {
           
             nCheckSumPosStart = i;//校验开始处，也就是帧的起始处。
             ucDataLen = pBuf[i+CUR_FRAME_L];//数据长度
						
						//检验数据长度的准确性
             nLen = usLenLeft - D07_FRAME_LEN_MIN  - ucDataLen;
             if(nLen < 0)
             {
                return E_D07_ERRO_FRAME_UNCOMP;//不完整的帧数据
             }

             nCheckSumPos = nCheckSumPosStart + ucDataLen + CUR_FRAME_DATA;//获取校验的下标
             nEndPos = nCheckSumPos+1;//最后一个字节下标

             // 计算校验和
             //查检checksum
            for(j = nCheckSumPosStart; j < nCheckSumPos; j++)
            {
                ucCheckSum +=  pBuf[j];
            }
            if(ucCheckSum != pBuf[nCheckSumPos])
            {
                //return E_D07_ERRO_FRAME_CHECK_SUM;
                continue;
            }

            //结束符
            if(0x16 != pBuf[nEndPos])
            {
                //return E_D07_ERRO_FRAME_END_0x16;
                continue;
            }
            
            *ppFirstPos = (uint8_t*)(pBuf + nCheckSumPosStart);
            *pusLenFrame = (uint16_t)(ucDataLen + 12);
            return E_D07_OK;

          }
          
        }
    }
    
    return E_D07_ERRO_NONE_FRAME;// 没有找到帧
}



//将数据打包成帧的函数
/*************************************************
Function:       pack_d07_frame_by_data
Description:    内部函数，仅完成结构体的填充

Author:         yunQin

Original：      liming

Calls:          
Called By:      
Input:          S_D07_PACK_FRAME *inPara 用于封帧数据
                
Output:         char *outBuffer 封装后的帧buffer
                uint32_t  *outLength 该帧总长
Return:         正确返回0

Others:         
  
*************************************************/
//打包帧的函数
//S_D07_PACK_FRAME *inPara   帧的结构体
//char *outBuffer    包装好的帧
//uint32_t *outLength   帧的长度

uint32_t pack_d07_frame_by_data(S_D07_PACK_FRAME *inPara, uint8_t *outBuffer, uint32_t *outLength){
    	int i;
    uint8_t ucCheckSum = 0;
    
    //参数安全检查
    if(!outBuffer || !inPara ||  !outLength )
    {
        return E_D07_ERRO_NULL;
    }
	
	outBuffer[0] = 0x68;
	for( i = 0; i < 6; i++ ){
		outBuffer[ 1 + i ] = inPara->Address_receiver[ i ] ;
	}
	outBuffer[7] = 0x68;
	outBuffer[8] = inPara->ctrl_C;
	outBuffer[9] = inPara->data_L;
	for(i = 0 ; i < inPara->data_L; i++){
		outBuffer[10 + i] = inPara->data[i] + 0x33;
	}
	for(i = 0 ; i< (10 + inPara->data_L); i++){
		ucCheckSum += outBuffer[i];
	}
	//校验和
	outBuffer[10 + inPara->data_L] = ucCheckSum;
	//结束符
	outBuffer[11 + inPara->data_L] = 0x16;
	
	*outLength = (12 + inPara->data_L);
    return E_D07_OK;
}




/*************************************************
Function:       unpack_d07_frame
Description:    解析DLT645 2007帧功能函数

Author:         yunQing

Original：      liming


Calls:          
Called By:      
Input:          inpBuffer      传入包有帧buffer指针
                inBufferLength 该buffer长度
                
Output:         outpFrame 该帧的各类详细信息

Return:         正确返回0

Others:         重要的功能函数
  
*************************************************/
//void *inpBuffer    帧起始地址
//uint32_t inBufferLength 帧的字节长度
//S_D07_UNPACK *outpFrame    解析后的帧信息
uint32_t unpack_d07_frame(void *inpBuffer, uint32_t inBufferLength, S_D07_UNPACK *outpFrame){
    uint32_t i = 0;
    uint32_t  nCheckSumPos;
    uint8_t ucDataLen = 0;
    uint8_t ucCheckSum = 0;
    //uint32_t ulRulerID = 0;
    
    uint8_t *buffer = (uint8_t *)inpBuffer;//确定访问帧的方式，一个字节一个字节的访问
  
	//对参数进行安全检查
    if(!inpBuffer || !outpFrame){//对指针判空
        return E_D07_ERRO_NULL;
    }
    if(inBufferLength < D07_FRAME_LEN_MIN){//对帧的长度进行安全检查
        return E_D07_ERRO_FRAME_UNCOMP;
    }


    // 检查前导字符 0x68
    if(0x68 != buffer[CUR_FRAME_START1] || 0x68 != buffer[CUR_FRAME_START2]){
        return E_D07_ERRO_FRAME_0x68;
    }


   //获取地址
    for(i = 0; i < ADDRESSLEN; i++){
        outpFrame->address[i] = buffer[CUR_FRAME_ADDRESS + i];
    }
   
		
  
    outpFrame->ctrl_c = buffer[CUR_FRAME_C];//获取控制码
  
   //数据域长度
    ucDataLen = buffer[CUR_FRAME_L];//获取数据域的长度
		
    nCheckSumPos = CUR_FRAME_CS(ucDataLen);//CS字节的光标

if(ucDataLen > 0){	
    //数据域-33处理	获取整个数据域的数据
    for(i = 0; i < ucDataLen; i++){
        outpFrame->aucDataTmp[i] = (buffer[CUR_FRAME_DATA + i] - 0x33); 
    }
}
    //查检checksum
    for(i = CUR_FRAME_START1; i < nCheckSumPos; i++){
        ucCheckSum +=  buffer[i];
    }
    if(ucCheckSum != buffer[nCheckSumPos]) {
    
        return E_D07_ERRO_FRAME_CHECK_SUM;
    }

    //结束符
    if(0x16 != buffer[nCheckSumPos+1]){
        return E_D07_ERRO_FRAME_END_0x16;
    }

//获取数据标识
	//ulRulerID = *((uint32_t * )(outpFrame->aucDataTmp))；
    outpFrame->ruler_id = ((outpFrame->aucDataTmp[0] & 0xFF) | 
            ((outpFrame->aucDataTmp[1] << 8) & 0xFF00) |
            ((outpFrame->aucDataTmp[2] << 16) & 0xFF0000) |
            ((outpFrame->aucDataTmp[3] << 24) & 0xFF000000));
		
    outpFrame->data_len = ucDataLen;
    outpFrame->frame_len = ucDataLen + 12;//帧的长度
	
    return E_D07_OK;
}


