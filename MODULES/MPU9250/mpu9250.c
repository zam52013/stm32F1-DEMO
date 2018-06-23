/*
 * File      : mpu9250.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */
#include "mpu9250.h"
#include "spi.h"
#include "usart.h"
/*******************************************************************************
函 数 名：void Init_9250_Cs(void)
功能描述：9250片选初始化
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void Init_9250_Cs(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(MPU9250_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = MPU9250_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU9250_PORT, &GPIO_InitStructure);
#ifdef DEBUG_MEG
    __DBG__("MPU9250功能打开!\r\n");
#endif
}
/*******************************************************************************
函 数 名： Mpu9250_Read_Reg(void)
功能描述：9250读寄存器
入口参数：  reg :地址
返回参数：值
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Mpu9250_Read_Reg(unsigned char reg)
{
    unsigned char value;
    MPU9250_RESET;
    SPI_ReadWriteByte(MPU9250_SPI, reg | DIR_READ);
    value = SPI_ReadWriteByte(MPU9250_SPI, DIR_WRITE);
    MPU9250_SET;
    return value;
}
/*******************************************************************************
函 数 名： Mpu9250_Write_Reg(void)
功能描述：9250写寄存器
入口参数：  reg :地址 value:值
返回参数：值
创建时间: 2018-1-05 by zam
********************************************************************************/
unsigned char Mpu9250_Write_Reg(unsigned char reg, unsigned char value)
{
    unsigned char status;
    MPU9250_RESET;
    status = SPI_ReadWriteByte(MPU9250_SPI, reg);
    SPI_ReadWriteByte(MPU9250_SPI, value);
    MPU9250_SET;
    return status;
}
/*******************************************************************************
函 数 名： Mpu9250_Init(void)
功能描述：9250初始
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
void Mpu9250_Init(void)
{
    Init_9250_Cs();
    Mpu9250_Write_Reg(MPUREG_PWR_MGMT_1, 0x03);//电源配置
    Mpu9250_Write_Reg(MPUREG_PWR_MGMT_2, 0x00);//电源配置
    Mpu9250_Write_Reg(MPUREG_SMPLRT_DIV, 0x00);//采样速率
    Mpu9250_Write_Reg(MPUREG_CONFIG, 0x02);
    Mpu9250_Write_Reg(MPUREG_GYRO_CONFIG, 0x08);//500
    Mpu9250_Write_Reg(MPUREG_ACCEL_CONFIG, 0x08);// 4g
    Mpu9250_Write_Reg(MPUREG_ACCEL_CONFIG2, 0x0E);
}
/*******************************************************************************
函 数 名： Get_Raw_Date(void)
功能描述：9250获取数据
入口参数：  无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
u8 raw_dat[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
void Get_Raw_Date(void)
{
    unsigned char i;

    for(i = 0; i < 14; i++)
    {
        raw_dat[i] = Mpu9250_Read_Reg(MPUREG_ACCEL_XOUT_H + i);
    }
}

