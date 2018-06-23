/*
 * File      : led.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */

#ifndef __LED_H_
#define __LED_H_

#include "stm32f10x.h"
/********************************/
/*       œµÕ≥≈‰÷√∫Ø ˝           */
/********************************/
#define LED_PORT GPIOF
#define LED_RCC RCC_APB2Periph_GPIOF

#define LED_POW GPIO_Pin_6

#define LED_RED GPIO_Pin_7

#define LED_ST GPIO_Pin_8

#define LED_ST1 GPIO_Pin_9

#define LED_ST2 GPIO_Pin_10


typedef enum
{
	LED_1=1,
	LED_2,
	LED_3,
	LED_4,
	LED_5
}LED_CHANGE;


void Led_Init(void);
void led_on(LED_CHANGE led_ch);
void led_off(LED_CHANGE led_ch);
void led_on_off(LED_CHANGE led_ch);




#endif

/********************************************************************************************************
**                            End Of File
********************************************************************************************************/


