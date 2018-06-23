/*
 * File      : tca62724.h
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

#ifndef __TCA627424_
#define __TCA627424_

#include "stm32f10x.h"


/****************************************************************/
#define I2C_OBDEV_LED_ADD 0x55    //tca627424地址
#define I2C_OBDEV I2C1             //设备接入IIC
/****************************************************************/

/****************************************************************/
#define SUB_ADD_OFF_PWM0 0x81            //tca627424PWM0关闭自动递增  地址
#define SUB_ADD_OFF_PWM1 0x82              //tca627424PWM1关闭自动递增  地址
#define SUB_ADD_OFF_PWM2 0x83              //tca627424PWM2关闭自动递增  地址

#define SUB_ADD_ON_PWM0 0x01             //tca627424PWM0自动递增  地址
#define SUB_ADD_ON_PWM1 0x02               //tca627424PWM1自动递增  地址
#define SUB_ADD_ON_PWM2 0x03               //tca627424PWM2自动递增  地址

#define SUB_ADD_SETTINGS_ON 0x04       //tca627424PWM2自动递增  地址  启用SHDN数据设置
#define SUB_ADD_SETTINGS_OFF 0x84      //tca627424PWM2关闭自动递增  地址  启用SHDN数据设置
#define SETTING_ENABLE 0x02
#define SETTING_NOT_POWERSAVE 0x01

/*************颜色*******************/
typedef enum
{
    REGLED_COLOR_OFF = 0,
    REGLED_COLOR_RED,
    REGLED_COLOR_YELLOW,
    REGLED_COLOR_PURPLE,
    REGLED_COLOR_GREEN,
    REGLED_COLOR_BLUE,
    REGLED_COLOR_WHITE,
    REGLED_COLOR_AMBER,
    REGLED_COLOR_DIM_RED,
    REGLED_COLOR_DIM_YELLOW,
    REGLED_COLOR_DIM_PURPLE,
    REGLED_COLOR_DIM_GREEN,
    REGLED_COLOR_DIM_BLUE,
    REGLED_COLOR_DIM_WHITE,
    REGLED_COLOR_DIM_AMBER
} rgbled_color_t;

typedef struct
{
    unsigned char RED_LED;
    unsigned char GREEN_LED;
    unsigned char BLUE_LED;
} rgbled_rgbset;

/****************************************************************/
extern rgbled_rgbset RGB_SET_VALUE;
void Send_Led_Rgb(rgbled_color_t color, unsigned char enable);
/****************************************************************/


#endif








