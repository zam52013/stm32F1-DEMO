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
#include "fm25v01.h"
#include "usart.h"
/*******************************************************************************
函 数 名：void fm25v_init(void)
功能描述： fm25v CS初始化
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void fm25v_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(FM25V_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = FM25V_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FM25V_PORT, &GPIO_InitStructure);
#ifdef DEBUG_MEG
    __DBG__("Fram功能打开!\r\n");
#endif
}
/*******************************************************************************
函 数 名：void Fram_wirte(void)
功能描述： fram写数据
入口参数：  addr：地址 length：数据长度 date：数据
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Fram_wirte(u16 addr, u16 lenth, unsigned char *date)
{
    u8 addr_h, addr_l;
    u16 count;
    //u8 delay_cnt=10;
    addr_h = (u8)((addr & 0xff00) >> 8);
    addr_l = (u8)((addr & 0x00ff));
    FM25V_RESET;
    SPI_ReadWriteByte(FM25V_SPI, FM25V_WREN);
    FM25V_SET;
    //while(delay_cnt--);
    FM25V_RESET;
    SPI_ReadWriteByte(FM25V_SPI, FM25V_WRITE);
    SPI_ReadWriteByte(FM25V_SPI, addr_h);
    SPI_ReadWriteByte(FM25V_SPI, addr_l);

    for(count = 0; count < lenth; count++)
    {
        SPI_ReadWriteByte(FM25V_SPI, *date++);
    }

    //SPI_ReadWriteByte(FM25V_ID,FM25V_WRDI);
    FM25V_SET;
}
/*******************************************************************************
函 数 名：void Fram_read(void)
功能描述： fram读数据
入口参数：  addr：地址 length：数据长度 date：数据
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Fram_read(u16 addr, u16 lenth, unsigned char *date)
{
    u8 addr_h, addr_l;
    u16 count;
    addr_h = (u8)((addr & 0xff00) >> 8);
    addr_l = (u8)((addr & 0x00ff));
    FM25V_RESET;
    SPI_ReadWriteByte(FM25V_SPI, FM25V_READ);
    SPI_ReadWriteByte(FM25V_SPI, addr_h);
    SPI_ReadWriteByte(FM25V_SPI, addr_l);

    for(count = 0; count < lenth; count++)
    {
        *date++ = SPI_ReadWriteByte(FM25V_SPI, 0x00);
    }

    FM25V_SET;
}











