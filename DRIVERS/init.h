/*
 * File      : init.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-24     Bernard      the first version
 */
#ifndef __INIT_H
#define __INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

void SysTickInit(void);
void IWDG_Init(void);
void NVIC_Configuration(void);
void IWDG_Feed(void);
#ifdef __cplusplus
}
#endif

#endif
