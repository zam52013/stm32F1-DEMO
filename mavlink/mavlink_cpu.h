/*
 * File      : mavlink_cpu.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */
#ifndef __MAVLINK_CPU_H
#define __MAVLINK_CPU_H

#ifdef __cplusplus
extern "C" {
#endif
#include "mavlink_types.h"		

//#define MAVPACKED( __Declaration__ ) __Declaration__

#define  inline  __INLINE 

#define MAVLINK_SEPARATE_HELPERS	
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS


mavlink_system_t mavlink_system =
{
    1,
    1
};
	
#include "mavlink.h"
#include "mavlink_helpers.h"

#ifdef __cplusplus
}
#endif
#endif

