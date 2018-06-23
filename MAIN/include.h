/*
 * File      : include.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *设备文件包含
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */


#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f10x.h"

/*******************数学函数***************************/
#include "math_stm.h"
/**********************************************/

/****************设备驱动******************************/
#include "init.h"
#include "usart.h"
#include "spi.h"
#include "ad.h"
#include "delay.h"
#include "iic.h"
#include "can.h"
#include "da.h"
#ifdef USE_SDIO_SD
#include "stm32_eval_sdio_sd.h"
#endif
#ifdef USE_SPI_SD
#include "stm32_eval_spi_sd.h"
#endif
/**********************************************/

/*******************设备库***************************/
#include "gui.h"
#include "messg_recive.h"
#include "messg_send.h"
#include "param.h"
#include "mavlink_cpu.h"
/**********************************************/

/**************设备********************************/
#include "led.h"
#include "fm25v01.h"
#include "mpu9250.h"
#include "sim7600.h"
#include "lcd_driver.h"
#include "dht11.h"
#include "tca62724.h"
#include "wifi_esp.h"
#include "mpu6050.h"
#include "bc28_nb.h"
/**********************************************/

#endif

