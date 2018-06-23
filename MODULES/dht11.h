/*
 * File      : dht11.h
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
#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f10x.h"
/**************DHT∂Àø⁄≈‰÷√**********************/
#define DHT11_PORT GPIOE
#define DHT11_RCC RCC_APB2Periph_GPIOE
#define DTH11_GPIO GPIO_Pin_3
#define DHT11_SET GPIO_SetBits(DHT11_PORT,DTH11_GPIO)
#define DHT11_RESET GPIO_ResetBits(DHT11_PORT,DTH11_GPIO)
/******************* ‰»Î- ‰≥ˆ≈‰÷√*******************************/
#define DHT11_OUT {GPIO_InitTypeDef GPIO_InitStructure;             \
                                        RCC_APB2PeriphClockCmd(DHT11_RCC,ENABLE);       \
                                        GPIO_InitStructure.GPIO_Pin = DTH11_GPIO;       \
                                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        \
                                        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           \
                                        GPIO_Init(DHT11_PORT, &GPIO_InitStructure);}
#define DHT11_IN    {GPIO_InitTypeDef GPIO_InitStructure;               \
                                        GPIO_InitStructure.GPIO_Pin = DTH11_GPIO ;          \
                                        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   \
                                        GPIO_Init(DHT11_PORT, &GPIO_InitStructure);}
/**************************************************/

typedef struct
{
	float temperature;
	float humidity;
}sensor_dht_11;

extern sensor_dht_11 SENSOR_DHT_11;

void DHT11_Rst(void);
u8 DHT11_Read_Data(void);
u8 DHT11_Init(void);
void Mavlink_msg_dht11_send(USART_TypeDef* USARTx,uint8_t bitheart);
#endif

