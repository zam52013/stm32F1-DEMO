/*
 * File      : sim7600.c
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

#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
/***************************************************************************/
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
u16 DeviceCode;
/*******************************************************************************
函 数 名：void LCD_GPIO_Init(void)
功能描述： 液晶端口初始化
入口参数：无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/

//液晶IO初始化配置
static void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(LCD_CS_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LCD_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_CS_PORT, &GPIO_InitStructure);
    RCC_APB2PeriphClockCmd(LCD_LED_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LCD_LED_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_LED_PORT, &GPIO_InitStructure);
    LCD_LED_ON;
    RCC_APB2PeriphClockCmd(LCD_RS_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LCD_RS_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_RS_PORT, &GPIO_InitStructure);
#ifdef DEBUG_MEG
    __DBG__("LCD功能打开!\r\n");
#endif
}

//向液晶屏写一个8位指令
void LCD_WR_REG(u8 Index)
{
    //SPI 写命令时序开始
    LCD_CS_RESET;
    LCD_RS_OFF;
    SPI_ReadWriteByte(LCD_SPI, Index);
    LCD_CS_SET;
}

//向液晶屏写一个8位数据
void LCD_WR_DATA(u8 Data)
{
    LCD_CS_RESET;
    LCD_RS_ON;
    SPI_ReadWriteByte(LCD_SPI, Data);
    LCD_CS_SET;
}
//向液晶屏写一个16位数据
void LCD_WR_DATA_16Bit(u16 Data)
{
    LCD_CS_RESET;
    LCD_RS_ON;
    SPI_ReadWriteByte(LCD_SPI, Data >> 8); 	//写入高8位数据
    SPI_ReadWriteByte(LCD_SPI, Data); 			//写入低8位数据
    LCD_CS_SET;
}
/*************************************************
函数名：LCD_SetCursor
功能：设置光标位置
入口参数：xy坐标
返回值：无
*************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
    Lcd_SetRegion(Xpos, Ypos, Xpos, Ypos);
}
//******************************************************************
//函数名：  LCD_WriteReg
//功能：    写寄存器数据
//输入参数：LCD_Reg:寄存器地址
//			LCD_RegValue:要写入的数据
//返回值：  无
//修改记录：无
//******************************************************************
void Lcd_WriteReg(u8 Index, u8 Data)
{
    LCD_WR_REG(Index);
    LCD_WR_DATA(Data);
}

//******************************************************************
//函数名：  LCD_WriteRAM_Prepare
//功能：    开始写GRAM
//			在给液晶屏传送RGB数据前，应该发送写GRAM指令
//输入参数：无
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_WriteRAM_Prepare(void)
{
    LCD_WR_REG(lcddev.wramcmd);
}

//******************************************************************
//函数名：  LCD_DrawPoint
//功能：    在指定位置写入一个像素点数据
//输入参数：(x,y):光标坐标
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_DrawPoint(u16 x, u16 y)
{
    LCD_SetCursor(x, y); //设置光标位置
    LCD_WR_DATA_16Bit(POINT_COLOR);
}

//设置LCD参数
//方便进行横竖屏模式切换
void LCD_SetParam(void)
{
    lcddev.wramcmd = 0x2C;
#if USE_HORIZONTAL==1	//使用横屏	  
    lcddev.dir = 1; //横屏
    lcddev.width = 128;
    lcddev.height = 128;
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;
    Lcd_WriteReg(0x36, 0xA8);
#else//竖屏
    lcddev.dir = 0; //竖屏
    lcddev.width = 128;
    lcddev.height = 128;
    lcddev.setxcmd = 0x2A;
    lcddev.setycmd = 0x2B;
    Lcd_WriteReg(0x36, 0xC8);
#endif
}
//******************************************************************
//函数名：  LCD_Init
//功能：    LCD初始化
//输入参数：无
//返回值：  无
//修改记录：无
//******************************************************************
void LCD_Init(void)
{
    LCD_GPIO_Init();
    delay_ms(150);
    //************* Start Initial Sequence **********//
    //开始初始化液晶屏
    LCD_WR_REG(0x11);//Sleep exit
    delay_ms(120);
    //ST7735R Frame Rate
    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x2D);
    LCD_WR_REG(0xB2);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x2D);
    LCD_WR_REG(0xB3);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x2D);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x2D);
    LCD_WR_REG(0xB4); //Column inversion
    LCD_WR_DATA(0x07);
    //ST7735R Power Sequence
    LCD_WR_REG(0xC0);
    LCD_WR_DATA(0xA2);
    LCD_WR_DATA(0x02);
    LCD_WR_DATA(0x84);
    LCD_WR_REG(0xC1);
    LCD_WR_DATA(0xC5);
    LCD_WR_REG(0xC2);
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0xC3);
    LCD_WR_DATA(0x8A);
    LCD_WR_DATA(0x2A);
    LCD_WR_REG(0xC4);
    LCD_WR_DATA(0x8A);
    LCD_WR_DATA(0xEE);
    LCD_WR_REG(0xC5); //VCOM
    LCD_WR_DATA(0x0E);
    LCD_WR_REG(0x36); //MX, MY, RGB mode
    LCD_WR_DATA(0xC8);
    //ST7735R Gamma Sequence
    LCD_WR_REG(0xe0);
    LCD_WR_DATA(0x0f);
    LCD_WR_DATA(0x1a);
    LCD_WR_DATA(0x0f);
    LCD_WR_DATA(0x18);
    LCD_WR_DATA(0x2f);
    LCD_WR_DATA(0x28);
    LCD_WR_DATA(0x20);
    LCD_WR_DATA(0x22);
    LCD_WR_DATA(0x1f);
    LCD_WR_DATA(0x1b);
    LCD_WR_DATA(0x23);
    LCD_WR_DATA(0x37);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x07);
    LCD_WR_DATA(0x02);
    LCD_WR_DATA(0x10);
    LCD_WR_REG(0xe1);
    LCD_WR_DATA(0x0f);
    LCD_WR_DATA(0x1b);
    LCD_WR_DATA(0x0f);
    LCD_WR_DATA(0x17);
    LCD_WR_DATA(0x33);
    LCD_WR_DATA(0x2c);
    LCD_WR_DATA(0x29);
    LCD_WR_DATA(0x2e);
    LCD_WR_DATA(0x30);
    LCD_WR_DATA(0x30);
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x3f);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x07);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0x10);
    LCD_WR_REG(0x2a);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x7f);
    LCD_WR_REG(0x2b);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x9f);
    LCD_WR_REG(0xF0); //Enable test command
    LCD_WR_DATA(0x01);
    LCD_WR_REG(0xF6); //Disable ram power save mode
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0x3A); //65k mode
    LCD_WR_DATA(0x05);
    LCD_WR_REG(0x29);//Display on
    LCD_SetParam();//设置LCD参数
//	LCD_LED_SET;//点亮背光
    //LCD_Clear(WHITE);
}


/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void Lcd_SetRegion(u16 x_start, u16 y_start, u16 x_end, u16 y_end)
{
#if USE_HORIZONTAL==1	//使用横屏
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(x_start >> 8);
    LCD_WR_DATA(0x00FF & x_start + 3);
    LCD_WR_DATA(x_end >> 8);
    LCD_WR_DATA(0x00FF & x_end + 3);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(y_start >> 8);
    LCD_WR_DATA(0x00FF & y_start + 2);
    LCD_WR_DATA(y_end >> 8);
    LCD_WR_DATA(0x00FF & y_end + 2);
#else
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(x_start >> 8);
    LCD_WR_DATA(0x00FF & x_start + 2);
    LCD_WR_DATA(x_end >> 8);
    LCD_WR_DATA(0x00FF & x_end + 2);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(y_start >> 8);
    LCD_WR_DATA(0x00FF & y_start + 3);
    LCD_WR_DATA(y_end >> 8);
    LCD_WR_DATA(0x00FF & y_end + 3);
#endif
    LCD_WriteRAM_Prepare();	//开始写入GRAM
}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(u16 x, u16 y)
{
    Lcd_SetRegion(x, y, x, y);
}


/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void Gui_DrawPoint(u16 x, u16 y, u16 Data)
{
    Lcd_SetRegion(x, y, x + 1, y + 1);
    LCD_WR_DATA_16Bit(Data);
}

/*****************************************
 函数功能：读TFT某一点的颜色
 出口参数：color  点颜色值
******************************************/
unsigned int Lcd_ReadPoint(u16 x, u16 y)
{
    unsigned int Data;
    Lcd_SetXY(x, y);
    //Lcd_ReadData();//丢掉无用字节
    //Data=Lcd_ReadData();
    LCD_WR_DATA(Data);
    return Data;
}


/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void LCD_Clear(u16 Color)
{
    u16 i, j;
    Lcd_SetRegion(0, 0, lcddev.width - 1, lcddev.height - 1);

    for(i = 0; i < lcddev.width; i++)
    {
        for(j = 0; j < lcddev.height; j++)
            LCD_WR_DATA_16Bit(Color);	//写入数据
    }
}





