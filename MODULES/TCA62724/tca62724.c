/*
 * File      : tca62724.c
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *  led IIC驱动
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-1-16     Bernard      the first version
 */

#include "tca62724.h"
#include "iic.h"
#include "usart.h"

/********************************************************************/
rgbled_rgbset RGB_SET_VALUE;
/********************************************************************
***函数名称: Send_Led_Enabel
***函数说明:模式设置
***输入参数:无
***输出参数:无
***
********************************************************************/
static void Send_Led_Enabel(unsigned char enable)
{
    unsigned char setting_byte = 0;
    unsigned char msg[1];

    if(enable)
    {
        setting_byte |= SETTING_ENABLE;
    }

    setting_byte |= SETTING_NOT_POWERSAVE;
    msg[0] = setting_byte;
    //I2C_Write_Multi(I2C_OBDEV_LED_ADD,SUB_ADD_SETTINGS_ON,msg,sizeof(msg));
}
/********************************************************************
***函数名称: Set_Color
***函数说明:颜色设置
***输入参数:颜色值
***输出参数:无
***
********************************************************************/
static void Set_Color(rgbled_color_t color)
{
    switch(color)
    {
        case REGLED_COLOR_OFF:
            RGB_SET_VALUE.RED_LED = 0;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_RED:
            RGB_SET_VALUE.RED_LED = 0x0F;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_YELLOW:
            RGB_SET_VALUE.RED_LED = 0x0F;
            RGB_SET_VALUE.GREEN_LED = 0x0C;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_PURPLE:
            RGB_SET_VALUE.RED_LED = 0x0F;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0x0F;
            break;

        case REGLED_COLOR_GREEN:
            RGB_SET_VALUE.RED_LED = 0;
            RGB_SET_VALUE.GREEN_LED = 0x0F;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_BLUE:
            RGB_SET_VALUE.RED_LED = 0;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0x0F;
            break;

        case REGLED_COLOR_WHITE:
            RGB_SET_VALUE.RED_LED = 0x0F;
            RGB_SET_VALUE.GREEN_LED = 0x0F;
            RGB_SET_VALUE.BLUE_LED = 0x0F;
            break;

        case REGLED_COLOR_AMBER:
            RGB_SET_VALUE.RED_LED = 0x0F;
            RGB_SET_VALUE.GREEN_LED = 0x05;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_DIM_RED:
            RGB_SET_VALUE.RED_LED = 0x06;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_DIM_YELLOW:
            RGB_SET_VALUE.RED_LED = 0x05;
            RGB_SET_VALUE.GREEN_LED = 0x02;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_DIM_PURPLE:
            RGB_SET_VALUE.RED_LED = 0x03;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0x03;
            break;

        case REGLED_COLOR_DIM_GREEN:
            RGB_SET_VALUE.RED_LED = 0;
            RGB_SET_VALUE.GREEN_LED = 0x06;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        case REGLED_COLOR_DIM_BLUE:
            RGB_SET_VALUE.RED_LED = 0;
            RGB_SET_VALUE.GREEN_LED = 0;
            RGB_SET_VALUE.BLUE_LED = 0x06;
            break;

        case REGLED_COLOR_DIM_WHITE:
            RGB_SET_VALUE.RED_LED = 0x02;
            RGB_SET_VALUE.GREEN_LED = 0x02;
            RGB_SET_VALUE.BLUE_LED = 0x02;
            break;

        case REGLED_COLOR_DIM_AMBER:
            RGB_SET_VALUE.RED_LED = 0x05;
            RGB_SET_VALUE.GREEN_LED = 0x01;
            RGB_SET_VALUE.BLUE_LED = 0;
            break;

        default:
            break;
    }
}
/********************************************************************
***函数名称: Send_Led_Rgb
***函数说明:输出颜色值
***输入参数:颜色值，是否使能PWM自增
***输出参数:无
***
********************************************************************/
void Send_Led_Rgb(rgbled_color_t color, unsigned char enable)
{
    unsigned char msg[3];
    Send_Led_Enabel(enable);
    Set_Color(color);
    msg[0] = RGB_SET_VALUE.RED_LED;
    msg[1] = RGB_SET_VALUE.GREEN_LED;
    msg[2] = RGB_SET_VALUE.BLUE_LED;
    //I2C_Write_Multi(I2C_OBDEV_LED_ADD,SUB_ADD_ON_PWM0,msg,sizeof(msg));
}







