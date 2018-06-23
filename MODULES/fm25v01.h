/*
 * File      : fm25v01.h
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
#ifndef __FM25V01_
#define __FM25V01_
#include "stm32f10x.h"
#include "spi.h"

/***SPI设备Fram cs选择**************************************/
#define FM25V_SPI SPI1
#define FM25V_PORT GPIOC
#define FM25V_RCC RCC_APB2Periph_GPIOC
#define FM25V_CS GPIO_Pin_4
#define FM25V_SET GPIO_SetBits(FM25V_PORT,FM25V_CS)
#define FM25V_RESET GPIO_ResetBits(FM25V_PORT,FM25V_CS)
/*****************************************************/

#define FM25V_WREN 0x06         /***写使能****/
#define FM25V_WRDI 0x04         /***写失能****/
#define FM25V_RDSR 0x05         /***读状态****/
#define FM25V_WRSR 0x01         /***写状态****/
#define FM25V_READ 0x03         /***读数据****/
#define FM25V_FSTRD 0x0B        /***快速读数据****/
#define FM25V_WRITE 0x02        /***写数据****/
#define FM25V_SLEEP 0xB9        /***进入休眠模式****/
#define FM25V_RDID 0x9f         /***读ID****/


void fm25v_init(void);
void Fram_wirte(u16 addr, u16 lenth, unsigned char *date);
void Fram_read(u16 addr, u16 lenth, unsigned char *date);

#endif


