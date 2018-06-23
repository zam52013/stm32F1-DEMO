/*
 * File      : lcd_drver.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-10-18     Bernard      the first version
 */

#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include "stm32f10x.h"
#include "stdlib.h"

////////////////////////////////////////////////////////////////////
//-----------------LCD端口定义---------------- 
#define LCD_LED_PORT 	GPIOG
#define LCD_LED_RCC 	RCC_APB2Periph_GPIOG
#define LCD_LED_GPIO    GPIO_Pin_11
#define LCD_LED_ON		GPIO_SetBits(LCD_LED_PORT,LCD_LED_GPIO)
#define LCD_LED_OFF		GPIO_ResetBits(LCD_LED_PORT,LCD_LED_GPIO)


#define LCD_RS_PORT 	GPIOG
#define LCD_RS_RCC 		RCC_APB2Periph_GPIOG
#define LCD_RS_GPIO    	GPIO_Pin_15
#define LCD_RS_ON		GPIO_SetBits(LCD_RS_PORT,LCD_RS_GPIO)
#define LCD_RS_OFF		GPIO_ResetBits(LCD_RS_PORT,LCD_RS_GPIO)

  /***SPI设备lcd cs选择**************************************/
#define LCD_SPI SPI2
#define LCD_CS_PORT GPIOB
#define LCD_CS_RCC RCC_APB2Periph_GPIOB
#define LCD_CS GPIO_Pin_12
#define LCD_CS_SET GPIO_SetBits(LCD_CS_PORT,LCD_CS)
#define LCD_CS_RESET GPIO_ResetBits(LCD_CS_PORT,LCD_CS)
/*****************************************************/
/***************************************************************/


//LCD重要参数集
typedef struct  
{										    
	u16 width;			//LCD 宽度
	u16 height;			//LCD 高度
	u16 id;				//LCD ID
	u8  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	u16	 wramcmd;		//开始写gram指令
	u16  setxcmd;		//设置x坐标指令
	u16  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数

#define USE_HORIZONTAL  	0	//定义是否使用横屏 		0,不使用.1,使用.
//使用硬件SPI 还是模拟SPI作为测试  
//定义LCD的尺寸
#if USE_HORIZONTAL==1	//使用横屏
	#define LCD_W 320
	#define LCD_H 240
#else
	#define LCD_W 240
	#define LCD_H 320
#endif

//TFTLCD部分外要调用的函数		   
extern u16  POINT_COLOR;//默认红色    
extern u16  BACK_COLOR; //背景颜色.默认为白色


//画笔颜色
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	0X841F //浅绿色
//#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
extern u16 BACK_COLOR, POINT_COLOR ; 


void LCD_WR_REG(u8 Index);
void LCD_WR_DATA(u8 Data);
void LCD_WR_DATA_16Bit(u16 Data);
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void Lcd_WriteReg(u8 Index,u8 Data);
void LCD_WriteRAM_Prepare(void);
void LCD_SetParam(void);
void LCD_DrawPoint(u16 x,u16 y);
void LCD_Init(void);
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void LCD_Clear(u16 Color);


#endif

