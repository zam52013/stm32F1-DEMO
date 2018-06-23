/*
 * File      : ms5611.h
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *    2320780008@qq.com
 * Change Logs:
 * Date           Author       ZAM
 * 2018-03-08     Bernard      the first version
 */
 #ifndef __MS5611_H
 #define __MS5611_H
 
 #include "stm32f10x.h"
 
 
 /***SPI…Ë±∏ms5611 cs—°‘Ò**************************************/
#define MS5611_SPI SPI3
#define MS5611_PORT GPIOB
#define MS5611_RCC RCC_APB2Periph_GPIOB
#define MS5611_CS GPIO_Pin_8
#define MS5611_SET GPIO_SetBits(MS5611_PORT,MS5611_CS)
#define MS5611_RESET GPIO_ResetBits(MS5611_PORT,MS5611_CS)
 /*****************************************************/
 
 #define DIR_READ			0x80
 #define DIR_WRITE			0x00

 #define CMD_MS5611_RESET 0x1E
 
 #define CMD_MS5611_D1_256 0x40
 #define CMD_MS5611_D1_512 0x42
 #define CMD_MS5611_D1_1024 0x44
 #define CMD_MS5611_D1_2048 0x46
 #define CMD_MS5611_D1_4096 0x48
 #define CMD_MS5611_D2_256 0x50
 #define CMD_MS5611_D2_512 0x52
 #define CMD_MS5611_D2_1024 0x54
 #define CMD_MS5611_D2_2048 0x56
 #define CMD_MS5611_D2_4096 0x58
 
 #define CMD_MS5611_ADC_READ 0x00
 #define CMD_MS5611_PROM_READ 0xA0
 
 void MS5611_Init(void);
 void MS5611_GetTP(double *TEMP,double *PRES);
 
 #endif
 
 