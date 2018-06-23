/*
 * File      : messg_send.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 
 
 ****************************************************************************************************
 *     |    -    |    --    |    -    |    -    |    -    |    -    |    ......    |    --    |
 *       起始位    数据长度    心跳包   系统版本  系统组件  信号标号     数据位         CRC
  ****************************************************************************************************
 
 
 * Change Logs:
 * Date           Author       ZAM
 * 2018-01-8     Bernard      the first version
 */
 
 #ifndef __MESSG_SEND_H
 #define __MESSG_SEND_H
 
 #include "stm32f10x.h"
 
 #define MESSG_STX 254		//数据起始标志号
 #define SYSID 1					//系统版本号
 #define COMPID 1					//系统组件号
 
 #define MESSAGE_USART USART2			//发送串口
 
 #define DHT11_MAVLINK_ID 12		
 
 
 void Message_Chan_Send(USART_TypeDef* USARTx,uint8_t msgid, const char *packet,uint16_t length,uint8_t bitheart);
 #endif
 



