/*
 * File      : bc28_nb.h
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-05-3     Bernard      the first version
 */


#ifndef BC28_NB_H
#define BC28_NB_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define NB_USART USART2
#define NB_Use_Usart USE_USART2
#define NBsend(fmt,args...) {USE_USART=NB_Use_Usart;printf(fmt,  ##args);}
#define NB_Messg USART2_MESSG

#define NB_hostName "117.60.157.137"
#define NB_port 5683

unsigned char NB_Net_cmd(char *cmd, char *reply1, unsigned int wait_time);
unsigned char Init_NB_Gprs(void);


#ifdef __cplusplus
}
#endif
#endif


