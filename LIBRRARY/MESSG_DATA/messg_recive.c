/*
 * File      : messg_revive.c
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
#include "messg_recive.h"
#include "messg_send.h"

get_message mavlink_messge;
unsigned char mavlink_messge_flag = 0;

#define X25_INIT_CRC 0xffff
/*******************************************************************************
函 数 名：void crc_init(void)
功能描述：校验初始
入口参数：  crcAccum：校验初始值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_init(uint16_t* crcAccum)
{
    *crcAccum = X25_INIT_CRC;
}
/*******************************************************************************
函 数 名：void crc_accumulate(void)
功能描述：校验计算
入口参数：  data:校验数据   crcAccum:校验输出值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}
/*******************************************************************************
函 数 名：void crc_calculate(void)
功能描述：校验计算
入口参数：  pBuffer:校验数据   length:校验长度
返回参数：校验值CRC16
创建时间: 2018-1-05 by zam
********************************************************************************/
static uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
    uint16_t crcTmp;
    crc_init(&crcTmp);

    while(length--)
    {
        crc_accumulate(*pBuffer++, &crcTmp);
    }

    return crcTmp;
}
/*******************************************************************************
函 数 名：void crc_calculate(void)
功能描述：校验计算
入口参数：  pBuffer:校验数据   length:校验长度 crcAccum:校验输出值
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
static void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
    const uint8_t *p = (const uint8_t *)pBuffer;

    while(length--)
    {
        crc_accumulate(*p++, crcAccum);
    }
}

#if 1
void UART5_IRQHandler(void)
{
    u8 RES;
		static u8 statu=0;
    static unsigned int mav_buf_cnt = 0;
    static unsigned int mav_dat_len = 0;
    unsigned int crc_check;

#ifdef SYSTEM_SUPPORT_OS
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();
#endif

    if(USART_GetITStatus(MAVLINK_RECIVE_UART, USART_IT_RXNE) != RESET)
    {
        RES = USART_ReceiveData(MAVLINK_RECIVE_UART);

        if(!mavlink_messge_flag)
        {
            switch(statu)
            {
                case 0:
                    if(RES == MESSG_STX)
                    {
                        statu++;
                        mavlink_messge.messge_stx = RES;
                    }
                    else
                    {
                        statu = 0;
                    }

                    break;

                case 1:
                    mavlink_messge.messge_length_h = RES;
                    statu++;
                    break;

                case 2:
                    mavlink_messge.messge_length_l = RES;
                    statu++;
                    mav_dat_len = (mavlink_messge.messge_length_h << 8) + mavlink_messge.messge_length_l;
                    break;

                case 3:
                    mavlink_messge.messge_bitheart = RES;
                    statu++;
                    break;

                case 4:
                    mavlink_messge.messge_sysid = RES;
                    statu++;
                    break;

                case 5:
                    mavlink_messge.messge_cmpid = RES;
                    statu++;
                    break;

                case 6:
                    mavlink_messge.messge_msgid = RES;
                    statu++;
                    mav_buf_cnt = 0;
                    break;

                case 7:
                    if(mav_buf_cnt <mav_dat_len)
                    {
                        mavlink_messge.buf[mav_buf_cnt] = RES;
                        mav_buf_cnt++;
											break;
                    }
                    else
                    {
                        statu++;
                    }

                    

                case 8:
                    mavlink_messge.crc_h = RES;
                    statu++;
                    break;

                case 9:
                    mavlink_messge.crc_l = RES;
                    statu++;
                    crc_check = crc_calculate(&mavlink_messge.messge_length_h, mav_dat_len + 6);

                    if(((unsigned char)crc_check == mavlink_messge.crc_h) && (((unsigned char)(crc_check >> 8)) == mavlink_messge.crc_l))
                    {
                        mavlink_messge_flag = TRUE;
                    }
                    else
                    {
                        statu = 0;
                    }

                    break;
				default:
					statu = 0;
					 break;
            }
        }
    }

#ifdef SYSTEM_SUPPORT_OS
    OS_CRITICAL_EXIT();
#endif
}
#endif

void Messge_Chan_Recive()
{
    if(mavlink_messge_flag)
    {
        switch(mavlink_messge.messge_msgid)
        {
            case DHT11_MAVLINK_ID:

            //(将数据拷贝出来)
            default:
                break;
        }

        mavlink_messge_flag = FALSE;
    }
}

