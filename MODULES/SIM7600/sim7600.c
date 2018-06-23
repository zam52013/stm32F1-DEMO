/*
 * File      : sim7600.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-10-18     Bernard      the first version
 */

#include "sim7600.h"
#include "usart.h"
#include <string.h>
#include "delay.h"
#include <stdlib.h>
#include <stdio.h>

/*******************************************************************************
函 数 名：void Sim_contro(void)
功能描述： 初始化控制端口
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Sim_contro(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(SIM_POWER_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(SIM_FLIGHTMODE_RCC, ENABLE);
    RCC_APB2PeriphClockCmd(SIM_POWKEY_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = SIM_POWER_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SIM_POWER_CONTRO_PORT, &GPIO_InitStructure);
    SIM_POWER_ON;
    GPIO_InitStructure.GPIO_Pin = SIM_FLIGHTMODE_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SIM_FLIGHTMODE_PORT, &GPIO_InitStructure);
    SIM_FLIGHTMODE_OFF;
    GPIO_InitStructure.GPIO_Pin = SIM_POWKEY_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SIM_POWKEY_PORT, &GPIO_InitStructure);
    SIM_POWKEY_ON;
#ifdef DEBUG_MEG
    __DBG__("SIM7600功能打开!\r\n");
#endif
//      SIM_POWKEY_OFF;
}
/*******************************************************************************
函 数 名： Sim_Net_cmd(void)
功能描述： 网络模块发送数据
入口参数：  cmd 需要发送的数据 reply1需要返回的数据 reply2需要返回的数据    wait_time超时时间ms
返回参数：发送是否成功 1：成功 0:存在字符 2 :超时
创建时间: 2018-1-12 by zam
********************************************************************************/
unsigned char Sim_Net_cmd(char *cmd, char *reply1, unsigned int wait_time)
{
    unsigned char ret = 0;
    SIMsend("%s\r\n", cmd);

    if((reply1 == NULL) || (wait_time == 0))   //不接数据
    {
        return 1;
    }

    Time_wait_cnt = wait_time;
    Time_statr_flag = TRUE;

    while(1)
    {
        if(SIM_Mess.receve_ok)
        {
            if(strstr(SIM_Mess.messg_buff, reply1) != NULL)
            {
                ret = 0; //存在字符串
                SIM_Mess.receve_ok = FALSE;
                break;
            }

            SIM_Mess.receve_ok = FALSE;
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
函 数 名：int8_t Init_Gsm_Gprs(void)
功能描述：初始化网络
入口参数：
返回参数：0: 成功   其他: 失败
创建时间: 2017-11-02 by zam
********************************************************************************/

unsigned char Init_Gsm_Gprs(void)
{
    int8_t ret = 0;

    if(Sim_Net_cmd("AT+CICCID", "SIM not", 500) == 0)
    {
#ifdef DEBUG_MEG
        __DBG__("没有sim卡!\r\n");
#endif
        return 1;
    }

    if(Sim_Net_cmd("ATE0", "SIM not", 500) != 0)
    {
        return 1;
    }

    if((ret = Sim_Net_cmd("AT+CGREG=1", "OK", 500)) != 0) //设置GPRS移动台类别为A,支持包交换和数据交换
    {
#ifdef DEBUG_MEG
        __DBG__("网络注册失败!\r\n");
#endif
        return ret;
    }

    if((ret = Sim_Net_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"", "OK", 200)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("IP设置错误!\r\n");
#endif
        //设置PDP上下文,互联网接协议,接入点等信息
        return ret;
    }

    if((ret = Sim_Net_cmd("AT+CSOCKSETPN=1", "OK", 200)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("CSOCKSETPN设置失败!\r\n");
#endif
        return ret;
    }

    if((ret = Sim_Net_cmd("AT+CIPMODE=0", "OK", 200)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("模式设置失败!\r\n");
#endif
        //附着GPRS业务
        return ret;
    }

    if((ret = Sim_Net_cmd("AT+NETOPEN", "OK", 200)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("网络未打开!\r\n");
#endif
        return ret;
    }

    if((ret = Sim_Net_cmd("AT+CIPHEAD=1", "OK", 200)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("设置IP头失败!\r\n");
#endif
        //设置接收数据显示IP头(方便判断数据来源,仅在单路连接有效)
        return ret;
    }

    return ret;
}
/*******************************************************************************
函 数 名：void TCP_Soc_Connet
功能描述：连接网络服务器
入口参数：soc:通道 address:服务器地址
返回参数：0 成功
创建时间: 2018-01-12 by zam
********************************************************************************/
unsigned char TCP_Soc_Connet(tcp_soc_address address)
{
    int8_t ret = 0;
    char connect_buf[200];
    memset(connect_buf, 0, sizeof(connect_buf));
    sprintf(connect_buf, "AT+CIPOPEN=0,\"TCP\",\"%s\",%d", address.hostName, address.port);

    if((ret = Sim_Net_cmd(connect_buf, "OPEN", 1000)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("连接出错!\r\n");
#endif
        return ret;
    }

    return 0;
}
/*******************************************************************************
函 数 名：void TCP_Soc_Send
功能描述：发送数据
入口参数：send_buffer:待发送数据 length:数据长度
返回参数：
创建时间: 2017-11-02 by zam
********************************************************************************/
unsigned char TCP_Soc_Send(char *send_buffer, unsigned int length)
{
    int8_t ret = 0;
    char sendbuf[1024];
    memset(sendbuf, 0, sizeof(sendbuf));

    if((ret = Sim_Net_cmd("AT+CIPSEND=0,", ">", 1000)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("无法发送数据到服务器!\r\n");
#endif
        return ret;
    }

    sprintf(sendbuf, "%s\32\0", send_buffer);

    if((ret = Sim_Net_cmd(send_buffer, "OK", 1000)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("发送数据错误.....\r\n");
#endif
        return ret;
    }

    return ret;
}
/*******************************************************************************
函 数 名：void qxwz_soc_close
功能描述：关闭网络模块数据
入口参数：
返回参数：
创建时间: 2017-11-02 by zam
********************************************************************************/
unsigned char TPC_Soc_Close(void)
{
    int8_t ret = 0;
    ret = Sim_Net_cmd("AT+CIPCLOSE=0", "CLOSE", 2000);  //关闭连接

    if(ret != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("关闭socket错误.....\r\n");
#endif
        return ret;
    }

    return ret;
}

unsigned char Http_Soc_Send(tcp_soc_address address, char *send_buffer)
{
    int8_t ret = 0;
    char sendbuf[1024];
    char connect_buf[200];
    memset(sendbuf, 0, sizeof(sendbuf));
    memset(connect_buf, 0, sizeof(connect_buf));
    sprintf(connect_buf, "AT+CHTTPACT=\"%s\",%d", address.hostName, address.port);

    if((ret = Sim_Net_cmd(connect_buf, "REQUEST", 2000)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("无法发送数据到服务器!\r\n");
#endif
        return ret;
    }

    sprintf(sendbuf, "%s\32\0", send_buffer); //////这里可能要更换一下

    if((ret = Sim_Net_cmd(send_buffer, "OK", 1000)) != 0)
    {
#ifdef DEBUG_MEG
        __DBG__("发送数据错误.....\r\n");
#endif
        return ret;
    }

    return ret;
}


