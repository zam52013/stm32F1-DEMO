/*
 * File      : flash_st.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *对片上存储进行操作这里可以用于boot升级用
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-01-24     Bernard      the first version
 */

#ifndef __FLASH_ST_H
#define __FLASH_ST_H


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f10x.h"

typedef  void (*pFunction)(void);
extern pFunction Jump_To_Application;

#define BANK1_WRITE_END_ADDR 0x08080000                    //结束地址 这个跟容量大小而定
#define BANK1_WRITE_START_ADDR 0x08004000              //分配16k内存用于存储boot
#define FLASH_PAGE_SIZE 0x800      //分页大小 2k这个跟容量大小而

void Boot_Flash_ErasePage();
void Boot_Flash_Write(uint32_t Addr, uint32_t *date, uint16_t num);
void Boot_Flash_Read(uint32_t Addr, uint32_t *date, uint16_t num);
void BootLoader_Jump(void);

#ifdef __cplusplus
}
#endif

#endif

