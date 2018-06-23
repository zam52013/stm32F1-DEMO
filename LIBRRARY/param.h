/*
 * File      : param.h
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

#ifndef PARAM_H
#define PARAM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define false 0
#define	true 1


#define ITEM_MAX 20

typedef enum
{
    PARAM_TYPE_UINT8 = 1, /* 8-bit unsigned integer | */
    PARAM_TYPE_INT8 = 2, /* 8-bit signed integer | */
    PARAM_TYPE_UINT16 = 3, /* 16-bit unsigned integer | */
    PARAM_TYPE_INT16 = 4, /* 16-bit signed integer | */
    PARAM_TYPE_UINT32 = 5, /* 32-bit unsigned integer | */
    PARAM_TYPE_INT32 = 6, /* 32-bit signed integer | */
    PARAM_TYPE_REAL32 = 7, /* 32-bit floating-point | */
} PARAM_TYPE_CLASS;

typedef enum
{
    PARAM_IDEX_P = 1,
    PARAM_IDEX_I = 2,
    PARAM_IDEX_D = 3,
    PARAM_IDEX_END,
} PARAM_IDEX;
typedef struct param_info_s
{
    char name[ITEM_MAX];
    union
    {
        uint8_t param_uint8_val;
        int8_t param_int8_val;
        uint16_t param_uint16_val;
        int16_t param_int16_val;
        uint32_t param_uint32_val;
        int32_t param_int32_val;
        float param_float_val;
    } param_type;
} param_info_s;


extern param_info_s param_head[PARAM_IDEX_END - 1];


bool Param_define(char* name, float val, PARAM_TYPE_CLASS param_type_class, PARAM_IDEX idex,
						uint8_t* uint8_value,int8_t* int8_value,uint16_t* uint16_value,
						int16_t* int16_value,uint32_t* uint32_value,int32_t* int32_value,float* float_value);

param_info_s Param_Idex_get(PARAM_IDEX idex, PARAM_TYPE_CLASS param_type_class);
bool Param32_Name_get(char* name, float* val, unsigned char* idex, PARAM_TYPE_CLASS param_type_class);

bool Param32_set(char* name, PARAM_TYPE_CLASS param_type_class,
						uint8_t uint8_value,int8_t int8_value,uint16_t uint16_value,
						int16_t int16_value,uint32_t uint32_value,int32_t int32_value,float float_value);

#ifdef __cplusplus
}
#endif
#endif


