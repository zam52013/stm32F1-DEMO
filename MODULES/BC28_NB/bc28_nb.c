/*
 * File      : bc28_nb.c
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-05-3     Bernard      the first version
 */

#include "bc28_nb.h"
#include "delay.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/*******************************************************************************
函 数 名： NB_Net_cmd(void)
功能描述： 网络模块发送数据
入口参数：  cmd 需要发送的数据 reply1需要返回的数据 reply2需要返回的数据    wait_time超时时间ms
返回参数：发送是否成功 1：成功 0:存在字符 2 :超时
创建时间: 2018-1-12 by zam
********************************************************************************/
unsigned char NB_Net_cmd(char *cmd, char *reply1, unsigned int wait_time)
{
    unsigned char ret = 0;
    NBsend("%s\r\n", cmd);

    if((reply1 == NULL) || (wait_time == 0))   //不接数据
    {
        return 1;
    }

    Time_wait_cnt = wait_time;
    Time_statr_flag = TRUE;

    while(1)
    {
        if(NB_Messg.receve_ok)
        {
            if(strstr(NB_Messg.messg_buff, reply1) != NULL)
            {
                ret = 0; //存在字符串
                NB_Messg.receve_ok = FALSE;
                break;
            }

            NB_Messg.receve_ok = FALSE;
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

unsigned char Init_NB_Gprs(void)
{
    char connect_buf[200];

    if(NB_Net_cmd("AT+NBAND?", "5", 20) != 0)
    {
        if(NB_Net_cmd("AT+NBAND=5", "OK", 20) == 0)
        {
            if(NB_Net_cmd("AT+NRB", "OK", 5000) == 0)
            {
#ifdef DEBUG_MEG
                __DBG__("NB start\r\n");
#endif
            }
            else
            {
                return 2;
            }
        }
        else
        {
            return 1;
        }
    }
    else
    {
#ifdef DEBUG_MEG
        __DBG__("NB start\r\n");
#endif
    }

    if(NB_Net_cmd("AT+NCONFIG=AUTOCONNECT,TRUE", "OK", 500) == 0)
    {
        if(NB_Net_cmd("AT+NCONFIG=CR_0354_0338_SCRAMBLING,TRUE", "OK", 500) == 0)
        {
            if(NB_Net_cmd("AT+NCONFIG=CR_0859_SI_AVOID,TRUE", "OK", 500) == 0)
            {
                if(NB_Net_cmd("AT+NRB", "OK", 5000) == 0)
                {
#ifdef DEBUG_MEG
                    __DBG__("自动寻网打开!\r\n");
#endif
                }
                else
                {
                    return 6;
                }
            }
            else
            {
                return 5;
            }
        }
        else
        {
            return 4;
        }
    }
    else
    {
        return 3;
    }

    if(NB_Net_cmd("AT+CGSN=1", "CGSN", 500) != 0)
    {
        return 7;
    }

    NB_Net_cmd("AT+CSQ", "CSQ", 500);

    if(NB_Net_cmd("AT+CFUN=0", "OK", 500) != 0)
    {
        return 8;
    }

    sprintf(connect_buf, "AT+NCDP=\"%s\",%d", NB_hostName, NB_port);

    if(NB_Net_cmd(connect_buf, "OK", 1000) == 0)
    {
        if(NB_Net_cmd("AT+NRB", "OK", 5000) == 0)
        {
#ifdef DEBUG_MEG
            __DBG__("net open!\r\n");
#endif
        }
    }
    else
    {
        return 9;
    }

    if(NB_Net_cmd("AT+CGATT?", "1", 100) == 0)
    {
#ifdef DEBUG_MEG
        __DBG__("has net!\r\n");
#endif
    }
    else
    {
        return 10;
    }

    return 0;
}