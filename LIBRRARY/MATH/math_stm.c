 /*
 * File      : math_stm.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *		关于数学运算的函数
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */
 #include "math_stm.h"
 
 /*******************************************************************************
函 数 名： invSqrt(void)
功能描述： 快速开平方求倒    1/sqrt(x*x)
入口参数：开方求倒值
返回参数：结果  https://cs.uwaterloo.ca/~m32rober/rsqrt.pdf
创建时间: 2018-1-05 by zam
********************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float rsqrt32(float number) 
{
	uint32_t i;
	float x2, y;
	x2 = number * 0.5F;
	y = number;
	i = *(uint32_t *) &y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float *) &i;
	y = y * (1.5F - (x2 * y * y));
	return y;
}

