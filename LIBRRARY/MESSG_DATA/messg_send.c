/*
 * File      : messg_send.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-01-8     Bernard      the first version
 */
 #include "messg_send.h"
 
 
#define X25_INIT_CRC 0xffff
/*******************************************************************************
函 数 名：void crc_init(void)
功能描述：校验初始
入口参数：	crcAccum：校验初始值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_init(uint16_t* crcAccum)
{
        *crcAccum = X25_INIT_CRC;
}
/*******************************************************************************
函 数 名：void crc_accumulate(void)
功能描述：校验计算
入口参数：	data:校验数据   crcAccum:校验输出值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
   /*Accumulate one byte of data into the CRC*/
   uint8_t tmp;
   tmp = data ^ (uint8_t)(*crcAccum &0xff);
   tmp ^= (tmp<<4);
   *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}
/*******************************************************************************
函 数 名：void crc_calculate(void)
功能描述：校验计算
入口参数：	pBuffer:校验数据   length:校验长度
返回参数：校验值CRC16
创建时间: 2018-1-05 by zam
********************************************************************************/
static uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
    uint16_t crcTmp;
    crc_init(&crcTmp);
		while (length--)
		{
       crc_accumulate(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}
/*******************************************************************************
函 数 名：void crc_calculate(void)
功能描述：校验计算
入口参数：	pBuffer:校验数据   length:校验长度 crcAccum:校验输出值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) 
	{
		crc_accumulate(*p++, crcAccum);
  }
}
/*******************************************************************************
函 数 名：void Message_Chan_Send(void)
功能描述：数据发送
入口参数：	USARTx:串口号   msgid:数据id  length:数据长度 bith+eart:心跳数据包
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Message_Chan_Send(USART_TypeDef* USARTx,uint8_t msgid, const char *packet,uint16_t length,uint8_t bitheart)
{
		uint16_t i;
		uint16_t checksum;
		uint8_t buf[7];
		uint8_t ck[2];
		buf[0]=MESSG_STX;
		buf[1]=(uint8_t)(length>>8);
		buf[2]=(uint8_t)(length & 0xff);
		buf[3]=bitheart;
		buf[4]=SYSID;
		buf[5]=COMPID;
		buf[6]=msgid;
		checksum=crc_calculate((const uint8_t*)&buf[1],6);
		crc_accumulate_buffer(&checksum,(const char*)packet,length);
		ck[1]=(uint8_t)(checksum>>8);
		ck[0]=(uint8_t)(checksum & 0xff);
		for(i=0;i<7;i++)
		{
			USART_SendData(USARTx,buf[i]);
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		}
		for(i=0;i<length;i++)
		{
			USART_SendData(USARTx,packet[i]);
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		}
		for(i=0;i<2;i++)
		{
			USART_SendData(USARTx,ck[i]);
			while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		}
}

