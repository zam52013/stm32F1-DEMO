/*
 * File      : param.c
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
#include "param.h"
#include <string.h>
#include <stdio.h>

param_info_s param_form[PARAM_IDEX_END - 1] =
{
    "NULL",
    0.0,
};
/**
  * @brief  it's can use idex get param
  *         parameters get in param_info_s
  * @param  idex param_type_class
  *
  *     @arg idex: idex value
  *     @arg param_type_class: the param type
  * @retval param name and value
  */
param_info_s Param_Idex_get(PARAM_IDEX idex, PARAM_TYPE_CLASS param_type_class)
{
    param_info_s param_value =
    {
        "NULL",
        0.0,
    };
    idex -= 1;

    if(idex < (PARAM_IDEX_END - 1))
    {
        switch(param_type_class)
        {
            case PARAM_TYPE_UINT8:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_uint8_val = (param_form[idex].param_type.param_uint8_val);
                break;

            case PARAM_TYPE_INT8:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_int8_val = (param_form[idex].param_type.param_int8_val);
                break;

            case PARAM_TYPE_UINT16:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_uint16_val = (param_form[idex].param_type.param_uint16_val);
                break;

            case PARAM_TYPE_INT16:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_int16_val = (param_form[idex].param_type.param_int16_val);
                break;

            case PARAM_TYPE_UINT32:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_uint32_val = (param_form[idex].param_type.param_uint32_val);
                break;

            case PARAM_TYPE_INT32:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_int32_val = (param_form[idex].param_type.param_int32_val);
                break;

            case PARAM_TYPE_REAL32:
                strncpy(param_value.name, param_form[idex].name, ITEM_MAX);
                param_value.param_type.param_float_val = (param_form[idex].param_type.param_float_val);
                break;

            default:
                break;
        }

        return param_value;
    }
    else
    {
        strncpy(param_value.name, "NULL", ITEM_MAX);
        param_value.param_type.param_float_val = 0.0;
        return param_value;
    }
}
/**
  * @brief  it's can use name get param
  *
  * @param  name- in
  *   param_type_class - in
  *		 val - out
  *  	idex - out
  *     @arg idex: idex value
  *     @arg param_type_class: the param type
  * @retval true is right false is erro
  */

bool Param32_Name_get(char* name, float* val, unsigned char* idex, PARAM_TYPE_CLASS param_type_class)
{
    unsigned char param_lenth = 0;
    unsigned char len;
    int ptr;
    len = strlen(name);

    for(param_lenth = 0; param_lenth < (PARAM_IDEX_END - 1); param_lenth++)
    {
        if((ptr = strncmp(param_form[param_lenth].name, name, len)) == 0)
        {
            switch(param_type_class)
            {
                case PARAM_TYPE_UINT8:
                    *val = (float)(param_form[param_lenth].param_type.param_uint8_val);
                    break;

                case PARAM_TYPE_INT8:
                    *val = (float)(param_form[param_lenth].param_type.param_int8_val);
                    break;

                case PARAM_TYPE_UINT16:
                    *val = (float)(param_form[param_lenth].param_type.param_uint16_val);
                    break;

                case PARAM_TYPE_INT16:
                    *val = (float)(param_form[param_lenth].param_type.param_int16_val);
                    break;

                case PARAM_TYPE_UINT32:
                    *val = (float)(param_form[param_lenth].param_type.param_uint32_val);
                    break;

                case PARAM_TYPE_INT32:
                    *val = (float)(param_form[param_lenth].param_type.param_int32_val);
                    break;

                case PARAM_TYPE_REAL32:
                    *val = (float)(param_form[param_lenth].param_type.param_float_val);
                    break;

                default:
                    return false;
            }

            *idex = param_lenth + 1;
            return true;
        }
    }

    return false;
}
/**
  * @brief  it's can use set param
  *
  * @param  name- in
  *   param_type_class - in
  *		 val - in
  *  	param_type_class - in
  *     @arg name: name
  *     @arg param_type_class: the param type
  * @retval true is right false is erro
  */

bool Param32_set(char* name, PARAM_TYPE_CLASS param_type_class,
						uint8_t uint8_value,int8_t int8_value,uint16_t uint16_value,
						int16_t int16_value,uint32_t uint32_value,int32_t int32_value,float float_value)
{
    unsigned char param_lenth = 0;
    unsigned char len;
    len = strlen(name);

    for(param_lenth = 0; param_lenth < (PARAM_IDEX_END - 1); param_lenth++)
    {
        if((strncmp(param_form[param_lenth].name, name, len)) == 0)
        {
            switch(param_type_class)
            {
                case PARAM_TYPE_UINT8:
                 
                    param_form[param_lenth].param_type.param_uint8_val = uint8_value;
                    break;

                case PARAM_TYPE_INT8:
               
                    param_form[param_lenth].param_type.param_int8_val = int8_value;
                    break;

                case PARAM_TYPE_UINT16:
               
                    param_form[param_lenth].param_type.param_uint16_val = uint16_value;
                    break;

                case PARAM_TYPE_INT16:
                   
                    param_form[param_lenth].param_type.param_int16_val = int16_value;
                    break;

                case PARAM_TYPE_UINT32:
                 
                    param_form[param_lenth].param_type.param_uint32_val = uint32_value;
                    break;

                case PARAM_TYPE_INT32:
                 
                    param_form[param_lenth].param_type.param_int32_val = int32_value;
                    break;

                case PARAM_TYPE_REAL32:
                   
                    param_form[param_lenth].param_type.param_float_val = float_value;
                    break;

                default:
                    return false;
            }

            return true;
        }
    }

    return false;
}
/**
  * @brief  it's can define param
  *
  * @param  name- in
  *   param_type_class - in
  *		 val - in
  *  	idex - in is only
  *     @arg name: name
  *     @arg param_type_class: the param type
  * @retval true is right false is erro
  */

bool Param_define(char* name, float val, PARAM_TYPE_CLASS param_type_class, PARAM_IDEX idex,
						uint8_t* uint8_value,int8_t* int8_value,uint16_t* uint16_value,
						int16_t* int16_value,uint32_t* uint32_value,int32_t* int32_value,float* float_value)
{
    unsigned char len;
    len = strlen(name);
    idex -= 1;

    if(len > ITEM_MAX)
    {
        len = ITEM_MAX;
    }

    if(idex < (PARAM_IDEX_END - 1))
    {
        switch(param_type_class)
        {
            case PARAM_TYPE_UINT8:
                val = (uint8_t)(val);
                param_form[idex].param_type.param_uint8_val = val;
                uint8_value = &param_form[idex].param_type.param_uint8_val;
                break;

            case PARAM_TYPE_INT8:
                val = (int8_t)(val);
                param_form[idex].param_type.param_int8_val = val;
                int8_value = &param_form[idex].param_type.param_int8_val;
                break;

            case PARAM_TYPE_UINT16:
                val = (uint16_t)(val);
                param_form[idex].param_type.param_uint16_val = val;
                uint16_value = &param_form[idex].param_type.param_uint16_val;
                break;

            case PARAM_TYPE_INT16:
                val = (int16_t)(val);
                param_form[idex].param_type.param_int16_val = val;
                int16_value = &param_form[idex].param_type.param_int16_val;
                break;

            case PARAM_TYPE_UINT32:
                val = (uint32_t)(val);
                param_form[idex].param_type.param_uint32_val = val;
                uint32_value = &param_form[idex].param_type.param_uint32_val;
                break;

            case PARAM_TYPE_INT32:
                val = (int32_t)(val);
                param_form[idex].param_type.param_int32_val = val;
                int32_value = &param_form[idex].param_type.param_int32_val;
                break;

            case PARAM_TYPE_REAL32:
                val = (float)(val);
                param_form[idex].param_type.param_float_val = val;
                float_value = &param_form[idex].param_type.param_float_val;
                break;

            default:
                return false;
        }

        strncpy(param_form[idex].name, name, ITEM_MAX);
        return true;
    }
    else
    {
        return false;
    }
}


