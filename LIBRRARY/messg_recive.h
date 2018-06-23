/*
 * File      : messg_revive.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-01-8     Bernard      the first version
 */
 
 #ifndef __MESSG_RECIVE_H
 #define __MESSG_RECIVE_H
 
 #include "stm32f10x.h"
 
 
 #define MAVLINK_RECIVE_UART UART5
 
 typedef struct
 {
	uint8_t messge_stx;
	 uint8_t messge_length_h;
	 uint8_t messge_length_l;
	 uint8_t messge_bitheart;
	 uint8_t messge_sysid;
	 uint8_t messge_cmpid;
	 uint8_t messge_msgid;
	 uint8_t buf[512];
	 uint8_t crc_h;
	 uint8_t crc_l;
 }get_message;
 


 void Messge_Chan_Recive();
 
 #endif
 


 