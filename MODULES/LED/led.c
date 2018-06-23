/*
 * File      : fm25v01.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-28     Bernard      the first version
 */
#include "led.h"
#include "usart.h"
/*******************************************************************************
函 数 名：void led_init(void)
功能描述： led初始化
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_POW | LED_RED | LED_ST | LED_ST1 | LED_ST2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
#ifdef DEBUG_MEG
    __DBG__("LED功能打开!\r\n");
#endif
}
/*******************************************************************************
函 数 名：void led_on(void)
功能描述： 打开LED灯
入口参数：  led_ch:LED灯
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/

void led_on(LED_CHANGE led_ch)
{
    switch (led_ch)
    {
        case LED_1:
            GPIO_SetBits(LED_PORT, LED_POW);
            break;
        case LED_2:
            GPIO_SetBits(LED_PORT, LED_RED);
            break;
        case LED_3:
            GPIO_SetBits(LED_PORT, LED_ST);
            break;
        case LED_4:
            GPIO_SetBits(LED_PORT, LED_ST1);
            break;
        case LED_5:
            GPIO_SetBits(LED_PORT, LED_ST2);
            break;
        default:
            break;
    }
}

/*******************************************************************************
函 数 名：void led_off(void)
功能描述： 关闭LED灯
入口参数：  led_ch:LED灯
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void led_off(LED_CHANGE led_ch)
{
    switch (led_ch)
    {
        case LED_1:
            GPIO_ResetBits(LED_PORT, LED_POW);
            break;
        case LED_2:
            GPIO_ResetBits(LED_PORT, LED_RED);
            break;
        case LED_3:
            GPIO_ResetBits(LED_PORT, LED_ST);
            break;
        case LED_4:
            GPIO_ResetBits(LED_PORT, LED_ST1);
            break;
        case LED_5:
            GPIO_ResetBits(LED_PORT, LED_ST2);
            break;
        default:
            break;
    }
}
/*******************************************************************************
函 数 名：void led_on_off(void)
功能描述： 关闭LED灯
入口参数：  led_ch:LED灯
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void led_on_off(LED_CHANGE led_ch)
{
    switch (led_ch)
    {
        case LED_1:
            GPIO_WriteBit(LED_PORT, LED_POW,!GPIO_ReadOutputDataBit(LED_PORT,LED_POW));
            break;
        case LED_2:
            GPIO_WriteBit(LED_PORT, LED_RED,!GPIO_ReadOutputDataBit(LED_PORT,LED_RED));
            break;
        case LED_3:
            GPIO_WriteBit(LED_PORT, LED_ST,!GPIO_ReadOutputDataBit(LED_PORT,LED_ST));
            break;
        case LED_4:
            GPIO_WriteBit(LED_PORT, LED_ST1,!GPIO_ReadOutputDataBit(LED_PORT,LED_ST1));
            break;
        case LED_5:
            GPIO_WriteBit(LED_PORT, LED_ST2,!GPIO_ReadOutputDataBit(LED_PORT,LED_ST2));
            break;
        default:
            break;
    }
}

