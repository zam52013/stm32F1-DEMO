/*
 * File      : flash_st.c
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

#include "flash_st.h"

pFunction Jump_To_Application;
uint32_t JumpAddress;
/*******************************************************************************
函 数 名：void Boot_Flash_ErasePage(void)
功能描述： 擦除flash
入口参数：  无
返回参数：无
创建时间: 2018-01-24 by zam
********************************************************************************/
void Boot_Flash_ErasePage()
{
    uint32_t NbrOfPage;
    uint32_t EraseCounter;
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    FLASH_UnlockBank1();
    NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
    {
        FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
    }

    FLASH_LockBank1();
}
/*******************************************************************************
函 数 名：void Boot_Flash_Write(void)
功能描述： 写flash
入口参数：  Addr 地址  date 数据  num个数
返回参数：无
创建时间: 2018-01-24 by zam
********************************************************************************/

void Boot_Flash_Write(uint32_t Addr, uint32_t *date, uint16_t num)
{
    uint16_t cnt = 0;
    FLASH_UnlockBank1();

    for(cnt = 0; cnt < num; num++)
    {
        FLASH_ProgramWord(Addr + (cnt * 2), *(date++));
    }

    FLASH_LockBank1();
}
/*******************************************************************************
函 数 名：void Boot_Flash_Read(void)
功能描述： 读flash
入口参数：  Addr 地址  date 数据  num个数
返回参数：无
创建时间: 2018-01-24 by zam
********************************************************************************/
void Boot_Flash_Read(uint32_t Addr, uint32_t *date, uint16_t num)
{
    uint16_t cnt = 0;
    FLASH_UnlockBank1();

    for(cnt = 0; cnt < num; num++)
    {
        *(date+cnt)=(*(__IO uint32_t*) (Addr+cnt*4));
    }

    FLASH_LockBank1();
}
/*******************************************************************************
函 数 名：void BootLoader_Jump(uint32_t Sect,uint32_t Msp,uint32_t Reset)
功能描述： 跳转到程序去执行
入口参数：
返回参数：无
创建时间: 2018-01-24 by zam
********************************************************************************/
void BootLoader_Jump()
{
    if(((*(__IO uint32_t*)BANK1_WRITE_START_ADDR) & 0x2FFE0000) == 0x20000000)
    {
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*)(BANK1_WRITE_START_ADDR + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) BANK1_WRITE_START_ADDR);
        Jump_To_Application();
    }
}
