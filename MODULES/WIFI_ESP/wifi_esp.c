/*
 * File      : wifi_esp.c
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
#include "wifi_esp.h"
#include "delay.h"
#include "usart.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef SYSTEM_SUPPORT_OS
#include "app.h"
#endif

wifi_soc_address MY_COMP = {"192.168.0.101", 8080};

#define BUF_MAX 40
#define RECV_DATA_MAX_BUF 2048

/************************************************************************/
static unsigned int wifi_recived_len = 0;       //网络收到的数据计数器
static unsigned int wifi_get_date_len = 0;      //网络获取的数据长度
static unsigned char wifi_cnd_len = 0;  //命令收到的数据计数器
static unsigned char begin_recive_flag = FALSE; //开始收集网络数据标志
char wifi_date_buf[RECV_DATA_MAX_BUF] = {0};        //网络收取的数据缓存
char wifi_cmd_date[BUF_MAX] = {0};
/************************************************************************/

/*******************************************************************************
函 数 名： Wifi_Esp_Cmd(void)
功能描述： wifi发送数据
入口参数：  cmd 需要发送的数据 reply1需要返回的数据 reply2需要返回的数据    wait_time超时时间ms
返回参数：发送是否成功 1：成功 0:存在字符 2 :超时
创建时间: 2018-1-05 by zam
********************************************************************************/
void USART3_IRQHandler(void)
{
    u8 RES;
#ifdef SYSTEM_SUPPORT_OS
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();
#endif

    if(USART_GetITStatus(WIFI_UART_N, USART_IT_RXNE) != RESET)
    {
        RES = USART_ReceiveData(WIFI_UART_N);

        if(begin_recive_flag)
        {
            wifi_date_buf[wifi_recived_len] = RES;
            wifi_recived_len++;

            if(wifi_recived_len >= wifi_get_date_len)   //表明收取数据完成
            {
                begin_recive_flag = FALSE;
                wifi_date_buf[wifi_recived_len + 1] = 0;
            }
        }
        else
        {
            wifi_cmd_date[wifi_cnd_len] = RES;
            wifi_cnd_len++;

            if(wifi_cnd_len >= BUF_MAX) //确保命令数据不会超过
            {
                wifi_cnd_len = 0;
            }
        }
    }

#ifdef SYSTEM_SUPPORT_OS
    OS_CRITICAL_EXIT();
#endif
}
/*******************************************************************************
函 数 名：void clear_buf(void)
功能描述：清除串口接收缓存区,并置接收偏移量为0
入口参数：
返回参数：
创建时间: 2017-11-02 by zam
********************************************************************************/
static void clear_buf(void)
{
    memset(wifi_cmd_date, 0, BUF_MAX);
    wifi_cnd_len = 0;              //接收字符串的起始存储位置
}
/*******************************************************************************
函 数 名： Wifi_Esp_Cmd(void)
功能描述： wifi发送数据
入口参数：  cmd 需要发送的数据 reply1需要返回的数据 reply2需要返回的数据    wait_time超时时间ms
返回参数：发送是否成功 1：成功 0:存在字符 2 :超时
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_Esp_Cmd(char *cmd, char *reply1, unsigned int wait_time)
{
    unsigned char ret = 0;
    clear_buf();
    Wif_Send("%s\r\n", cmd);

    if((reply1 == NULL) || (wait_time == 0))   //不接数据
    {
        return 1;
    }

    Time_wait_cnt = wait_time;
    Time_statr_flag = TRUE;

    while(1)
    {
        if(strstr(wifi_cmd_date, reply1) != NULL)
        {
            ret = 0; //存在字符串
            break;
        }

        if(Time_out_flag)
        {
            ret = 2; //超时
            break;
        }
    }

    Time_statr_flag = FALSE;
    Time_out_flag = FALSE;
    return ret;
}
/*******************************************************************************
函 数 名：void Wifi_Io_Init(void)
功能描述： wifi控制端口初始化
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Wifi_Io_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(WIFI_CH_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = WIFI_CH_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WIFI_CH_PORT, &GPIO_InitStructure);
    WIFI_CH_OFF;
    RCC_APB2PeriphClockCmd(WIFI_RST_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = WIFI_RST_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WIFI_RST_PORT, &GPIO_InitStructure);
    WIFI_RST_ON;
#ifdef DEBUG_MEG
    __DBG__("WIFI功能打开!\r\n");
#endif
}
/*******************************************************************************
函 数 名：void Wifi_Net_Mode(void)
功能描述： wifi模式设置
入口参数：  mode
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_Net_Mode(ENUM_Net_ModeTypeDef mode)
{
    switch(mode)
    {
        case STA:
            return Wifi_Esp_Cmd("AT+CWMODE=1", "OK", 2500);

        case AP:
            return Wifi_Esp_Cmd("AT+CWMODE=2", "OK",  2500);

        case STA_AP:
            return Wifi_Esp_Cmd("AT+CWMODE=3", "OK",  2500);

        default:
            return FALSE;
    }
}
/*******************************************************************************
函 数 名：unsigned char Wifi_JoinAP(void)
功能描述： wifi加入到网络
入口参数：  pSSID 网络名字  pPassWord网络密码
返回参数：1：成功 0:存在字符 2 :超时
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_JoinAP(char * pSSID, char * pPassWord)
{
    char cCmd [120];
    sprintf(cCmd, "AT+CWJAP=\"%s\",\"%s\"", pSSID, pPassWord);
    return Wifi_Esp_Cmd(cCmd, "OK", 7000);
}
/*******************************************************************************
函 数 名：Wifi_Connet
功能描述： wifi连接到服务器
入口参数：  服务器地址和端口号
返回参数：0:连接成功 1:连接失败
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_Connet(wifi_soc_address addrss)
{
    char connect_buf[200];
    Wifi_Esp_Cmd("AT+CIPMUX=0", "OK", 500);
    memset(connect_buf, 0, sizeof(connect_buf));
    sprintf(connect_buf, "AT+CIPSTART=\"TCP\",\"%s\",%d", addrss.hostName, addrss.port);
#ifdef DEBUG_MEG
    __DBG__("%s\r\n", connect_buf);
#endif

    if(Wifi_Esp_Cmd(connect_buf, "OK", 1000) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("连接服务器出错!\r\n");
#endif
        return 1;   //错误
    }
    else
        return 0;
}
/*******************************************************************************
函 数 名：Wifi_Clse
功能描述： wifi关闭连接到服务器
入口参数：无
返回参数：0:连接成功 1:连接失败
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_Clse(void)
{
    if(Wifi_Esp_Cmd("AT+CIPCLOSE", "OK", 5000) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("无法关闭服务器!\r\n");
#endif
        return 1;   //错误
    }
    else
        return 0;
}

/*******************************************************************************
函 数 名：Wifi_Send
功能描述： wifi发送数据到服务器
入口参数：  send_buffer:数据
返回参数：0:连接成功 1:连接失败
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Wifi_Send(char *send_buffer)
{
    char sendbuf[1024];
    Wifi_Esp_Cmd("AT+CIPMODE=1", "OK", 500); //透传模式

    if(Wifi_Esp_Cmd("AT+CIPSEND=0,", ">", 500) != 0) //进入发送模式
    {
#ifdef DEBUG_MEG
        __DBG__("无法进入发送数据模式!\r\n");
#endif
        return 1;
    }

    sprintf(sendbuf, "%s\32\0", send_buffer);

    if(Wifi_Esp_Cmd(sendbuf, "OK", 5000) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("发送数据失败!\r\n");
#endif
        return 1;
    }
    else
        return 0;
}
/*******************************************************************************
函 数 名：Wifi_Init
功能描述： wifi初始化
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Wifi_Init()
{
    unsigned char ret = 0;
    WIFI_CH_ON;
    WIFI_RST_ON;
//  Wifi_Esp_Cmd("AT+RST", "OK",1000);
    Wifi_Net_Mode(AP);
//  Wifi_Esp_Cmd("AT+RST", "OK",1000);
    Wifi_Esp_Cmd("AT+CWLAP", "OK", 500);
    //Wifi_Esp_Cmd("AT+RST", "OK",1000);
    ret = Wifi_JoinAP(Pssid, PassWord); //连接wifi
#ifdef DEBUG_MEG

    if(ret == 0)
    {
        __DBG__("连接wifi:%s,密码:%s\r\n", Pssid, PassWord);
    }
    else if(ret == 2)
    {
        __DBG__("连接wifi超时!\r\n");
    }

#endif
}



