/*
 * File      : ds18b20.c
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

#include "ds18b20.h"
#include "usart.h"
#include "delay.h"
/*******************************************************************************
函 数 名：void DS18B20_Reset(void)
功能描述：复位设备
入口参数：   无
返回参数：
创建时间: 2018-3-12 by zam
********************************************************************************/
static void DS18B20_Reset(void)
{
    DS18B20_OUT;
    DS18B20_RESET;
    delay_us(750);
    DS18B20_IN;
    delay_us(5);
}
/*******************************************************************************
函 数 名：void DS18B20_Check(void)
功能描述：检测设备
入口参数：   无
返回参数：1:不存在 0:存在
创建时间: 2018-3-12 by zam
********************************************************************************/
static unsigned char DS18B20_Check(void)
{
    unsigned char DS18B20_Cont = 0;

    while(DS18B20_READ && (DS18B20_Cont < 200))
    {
        DS18B20_Cont++;
        delay_us(1);
    }

    if(DS18B20_Cont >= 200)
    {
        return 1;
    }
    else
    {
        DS18B20_Cont = 0;
    }

    while(!DS18B20_READ && (DS18B20_Cont < 240))
    {
        DS18B20_Cont++;
        delay_us(1);
    }

    if(DS18B20_Cont >= 240)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/*******************************************************************************
函 数 名： DS18B20_Read_Bit(void)
功能描述：读一位
入口参数：   无
返回参数：读到的数据
创建时间: 2018-3-12 by zam
********************************************************************************/
static unsigned char DS18B20_Read_Bit(void)
{
    unsigned char date;
    DS18B20_OUT;
    DS18B20_RESET;
    delay_us(2);
    DS18B20_SET;
    DS18B20_IN;
    delay_us(12);

    if(DS18B20_READ)
    {
        date = 1;
    }
    else
    {
        date = 0;
    }

    delay_us(50);
    return date;
}
/*******************************************************************************
函 数 名： DS18B20_Read_Byte(void)
功能描述：读一个字
入口参数：   无
返回参数：读到的数据
创建时间: 2018-3-12 by zam
********************************************************************************/
static unsigned char DS18B20_Read_Byte()
{
    unsigned char i, j, dat = 0;

    for(i = 0; i < 8; i++)
    {
        j = DS18B20_Read_Bit();
        dat = (j << 7) | (dat >> 1);
    }

    return dat;
}
/*******************************************************************************
函 数 名： DS18B20_Write_Byte(void)
功能描述：写命令
入口参数：   命令
返回参数：无
创建时间: 2018-3-12 by zam
********************************************************************************/
static void DS18B20_Write_Byte(unsigned char cmd)
{
    u8 j;
    u8 testb;
    DS18B20_OUT;

    for(j = 1; j <= 8; j++)
    {
        testb = cmd & 0x01;
        cmd = cmd >> 1;

        if(testb)
        {
            DS18B20_RESET;  // Write 1
            delay_us(2);
            DS18B20_SET;
            delay_us(60);
        }
        else
        {
            DS18B20_RESET;  // Write 0
            delay_us(60);
            DS18B20_SET;
            delay_us(2);
        }
    }
}
/*******************************************************************************
函 数 名： DS18B20_Start(void)
功能描述：启动转换
入口参数：  无
返回参数：无
创建时间: 2018-3-12 by zam
********************************************************************************/
static void DS18B20_Start(void)
{
    DS18B20_Reset();
    DS18B20_Check();
    DS18B20_Write_Byte(0xcc);   // skip rom
    DS18B20_Write_Byte(0x44);   // convert
}
/*******************************************************************************
函 数 名： DS18B20_Get_Temp(void)
功能描述：获取到温度值
入口参数：  无
返回参数：无
创建时间: 2018-3-12 by zam
********************************************************************************/
short DS18B20_Get_Temp(void)
{
    u8 temp;
    u8 TL, TH;
    short tem;
    DS18B20_Start();            // ds1820 start convert
    DS18B20_Reset();
    DS18B20_Check();
    DS18B20_Write_Byte(0xcc);   // skip rom
    DS18B20_Write_Byte(0xbe);   // convert
    TL = DS18B20_Read_Byte();   // LSB
    TH = DS18B20_Read_Byte();   // MSB

    if(TH > 7)
    {
        TH = ~TH;
        TL = ~TL;
        temp = 0;               //温度为负
    }
    else temp = 1;              //温度为正

    tem = TH;                   //获得高八位
    tem <<= 8;
    tem += TL;                  //获得底八位
    tem = (float)tem * 0.625;   //转换

    if(temp)return tem;         //返回温度值
    else return -tem;
}
/*******************************************************************************
函 数 名： DS18B20_Init(void)
功能描述：初始化端口
入口参数：  无
返回参数：无
创建时间: 2018-3-12 by zam
********************************************************************************/
unsigned char DS18B20_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(DS18B20_RCC, ENABLE);     //使能PORTG口时钟

    GPIO_InitStructure.GPIO_Pin = DS18B20_GPIO;             //PORTG.11 推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DS18B20_PORT, &GPIO_InitStructure);
#ifdef DEBUG_MEG
    __DBG__("DS18B20功能打开!\r\n");
#endif
    DS18B20_SET;    //输出1
    DS18B20_Reset();
    return DS18B20_Check();
}