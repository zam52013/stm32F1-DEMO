/*
 * File      : ds18b20.h
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-03-12     Bernard      the first version
 */

#ifndef __DS18B20_H
#define __DS18B20_H


#include "stm32f10x.h"
/**************DHT∂Àø⁄≈‰÷√**********************/
#define DS18B20_PORT GPIOE
#define DS18B20_RCC RCC_APB2Periph_GPIOE
#define DS18B20_GPIO GPIO_Pin_2
#define DS18B20_SET GPIO_SetBits(DS18B20_PORT,DS18B20_GPIO)
#define DS18B20_RESET GPIO_ResetBits(DS18B20_PORT,DS18B20_GPIO)
#define DS18B20_READ GPIO_ReadInputDataBit(DS18B20_PORT,DS18B20_GPIO)
/******************* ‰»Î- ‰≥ˆ≈‰÷√*******************************/
#define DS18B20_OUT {GPIO_InitTypeDef GPIO_InitStructure;               \
                                        RCC_APB2PeriphClockCmd(DS18B20_RCC,ENABLE);     \
                                        GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO;     \
                                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        \
                                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           \
                                        GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);}
#define DS18B20_IN  {GPIO_InitTypeDef GPIO_InitStructure;               \
                                        GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO ;            \
                                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   \
                                        GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);}
/**************************************************/

short DS18B20_Get_Temp(void);
unsigned char DS18B20_Init(void);

#endif

