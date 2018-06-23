/*
 * File      : wifi_esp.h
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-1-5     Bernard      the first version
 */
#ifndef __WIFI_ESP_H
#define __WIFI_ESP_H

#include "stm32f10x.h"
#include "usart.h"

/******SIM卡接口配置********/
#define WIFI_UART_N USART3
#define WIFI_Uart USE_USART3
#define Wif_Send(fmt,args...) {USE_USART=WIFI_Uart;printf(fmt,  ##args);}
#define Wifi_Mess USART3_MESSG
/***************************/

/*********片选控制端******************/
#define WIFI_CH_PORT GPIOA
#define WIFI_CH_RCC RCC_APB2Periph_GPIOA
#define WIFI_CH_GPIO GPIO_Pin_0
#define WIFI_CH_ON GPIO_SetBits(WIFI_CH_PORT,WIFI_CH_GPIO)
#define WIFI_CH_OFF GPIO_ResetBits(WIFI_CH_PORT,WIFI_CH_GPIO)
/***************************/

/*********复位端******************/
#define WIFI_RST_PORT GPIOA
#define WIFI_RST_RCC RCC_APB2Periph_GPIOA
#define WIFI_RST_GPIO GPIO_Pin_1
#define WIFI_RST_ON GPIO_SetBits(WIFI_RST_PORT,WIFI_RST_GPIO)
#define WIFI_RST_OFF GPIO_ResetBits(WIFI_RST_PORT,WIFI_RST_GPIO)
/***************************/
#define Pssid "TP-ZAM"
#define PassWord "!~1s5(^15)>?dD6r"

#define WIFI_OP_SMARTCNFIG "AT+CWSTARTSMART=1"      //打开smartconfig模式
#define WIFI_CL_SMARTCNFIG "AT+CWSTOPSMART"         //关闭smartconfig模式

typedef enum
{
    STA = 0,
    AP,
    STA_AP
} ENUM_Net_ModeTypeDef;

typedef struct
{
    char *hostName;
    short port;
} wifi_soc_address; //连接地址

extern wifi_soc_address MY_COMP;

void Wifi_Io_Init(void);
unsigned char Wifi_JoinAP(char * pSSID, char * pPassWord);
void Wifi_Init(void);
unsigned char Wifi_Connet(wifi_soc_address addrss);
unsigned char Wifi_Send(char *send_buffer);

#endif




