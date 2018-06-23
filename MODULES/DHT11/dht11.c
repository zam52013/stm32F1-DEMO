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
#include "dht11.h"
#include "delay.h"
#include "usart.h"
#include "messg_send.h"

sensor_dht_11 SENSOR_DHT_11;
/*******************************************************************************
函 数 名：void DHT11_Rst(void)
功能描述： 复位DHT11
入口参数：   无
返回参数：无
创建时间: 2018-1-13 by zam
********************************************************************************/
void DHT11_Rst(void)
{
    DHT11_OUT;  //SET OUTPUT
    DHT11_RESET;    //拉低DQ
    delay_ms(5);        //拉低至少0.8ms
    DHT11_SET;  //DQ=1
    //DHT11_IN;
    //delay_us(10);         //主机拉高20~40us
}
/*******************************************************************************
函 数 名：void DHT11_Check(void)
功能描述：等待DHT11的回应
入口参数：   无
返回参数：返回1:未检测到DHT11的存在 返回0:存在
创建时间: 2018-1-13 by zam
********************************************************************************/
u8 DHT11_Check(void)
{
    u16 retry = 0x00;
    DHT11_IN;//SET INPUT

    //delay_us(1);
    while(retry++ < 5);

    while((GPIO_ReadInputDataBit(DHT11_PORT, DTH11_GPIO)) && (retry < 0x1ff)) //DHT11会拉低40~80us
    {
        retry++;
        //delay_us(1);
    };

    if(retry >= 0x190)
        return 1;
    else retry = 0x00;

    while((!GPIO_ReadInputDataBit(DHT11_PORT, DTH11_GPIO)) && (retry < 0x1ff)) //DHT11会拉低40~80us
    {
        retry++;
        //delay_us(1);
    };

    if(retry >= 0x190)
        return 1;
    else
        retry = 0x00;

    while((GPIO_ReadInputDataBit(DHT11_PORT, DTH11_GPIO)) && (retry < 0x1ff)) //等待变为低电平
    {
        retry++;
        //delay_us(1);
    }

    if(retry >= 0x190)
        return 1;
    else
        return 0;
}
/*******************************************************************************
函 数 名：void DHT11_Read_Bit(void)
功能描述：从DHT11读取一个位
入口参数：   无
返回参数：返回值：1/0
创建时间: 2018-1-13 by zam
********************************************************************************/
unsigned char DHT11_Read_Bit(void)
{
    u16 retry = 0;

    while((!GPIO_ReadInputDataBit(DHT11_PORT, DTH11_GPIO)) && (retry < 0x1ff)) //等待变为低电平
    {
        retry++;
        //delay_us(1);
    }

    retry = 0x00;

    while((GPIO_ReadInputDataBit(DHT11_PORT, DTH11_GPIO)) && (retry < 0x1ff)) //等待变高电平
    {
        retry++;
        //delay_us(1);
    }

    if(retry > 0x41)
        return 1;
    else
        return 0;
}
/*******************************************************************************
函 数 名：void DHT11_Read_Byte(void)
功能描述：从DHT11读取一个字节
入口参数：   无
返回参数：读到的数据
创建时间: 2018-1-13 by zam
********************************************************************************/
unsigned char DHT11_Read_Byte(void)
{
    char i;
    unsigned char dat;
    dat = 0;

    for(i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= DHT11_Read_Bit();
    }

    return dat;
}
/*******************************************************************************
函 数 名：void DHT11_Read_Data(void)
功能描述：temp:温度值(范围:0~50°)   humi:湿度值(范围:20%~90%)
入口参数：   无
返回参数：0,正常;1,读取失败
创建时间: 2018-1-13 by zam
********************************************************************************/
u8 DHT11_Read_Data()
{
    unsigned char buf[5];
    u8 i;
    DHT11_Rst();

    if(DHT11_Check() == 0)
    {
        for(i = 0; i < 5; i++) //读取40位数据
        {
            buf[i] = DHT11_Read_Byte();
        }

        DHT11_OUT;
        if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            if(buf[2] & 0x80)
            {
                SENSOR_DHT_11.temperature = -((float)(buf[2] & 0x7F) * 256 + buf[3]) / 10.0;
            }
            else
            {
                SENSOR_DHT_11.temperature = (float)(buf[2] * 256 + buf[3]) / 10.0;
            }

            SENSOR_DHT_11.humidity = (float)(buf[0] * 256 + buf[1]) / 10.0;
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
}
/*******************************************************************************
函 数 名：void DHT11_Init(void)
功能描述：初始化DHT11的IO口 DQ 同时检测DHT11的存在
入口参数：   无
返回参数：1:不存在 0:存在
创建时间: 2018-1-13 by zam
********************************************************************************/
u8 DHT11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(DHT11_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = DTH11_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_PORT, &GPIO_InitStructure);
    DHT11_SET;
#ifdef DEBUG_MEG
    __DBG__("DHT11功能打开!\r\n");
#endif
    DHT11_Rst();
    return DHT11_Check();
}
/*******************************************************************************
函 数 名：void mavlink_msg_dht11_send(USART_TypeDef* USARTx,float temp,float hum,uint8_t bitheart)
功能描述：数据发送
入口参数：   无
返回参数：1:不存在 0:存在
创建时间: 2018-1-13 by zam
********************************************************************************/
void Mavlink_msg_dht11_send(USART_TypeDef* USARTx,uint8_t bitheart)
{
	sensor_dht_11 packet;
	packet.humidity=SENSOR_DHT_11.humidity;
	packet.temperature=SENSOR_DHT_11.temperature;
	Message_Chan_Send(USARTx,DHT11_MAVLINK_ID,(const char *)&packet,8,bitheart);
}
