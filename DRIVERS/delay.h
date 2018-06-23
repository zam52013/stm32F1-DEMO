#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f10x.h"

#define FALSE 0
#define TRUE  1

#define FRAME_COUNT   2000

#define COUNT_500HZ   2         // Number of 1000 Hz frames for 500 Hz Loop
#define COUNT_100HZ   10        // Number of 1000 Hz frames for 100 Hz Loop
#define COUNT_50HZ    20        // Number of 1000 Hz frames for  50 Hz Loop
#define COUNT_10HZ    100       // Number of 1000 Hz frames for  10 Hz Loop
#define COUNT_5HZ     200       // Number of 1000 Hz frames for   5 Hz Loop
#define COUNT_2HZ     500       // Number of 1000 Hz frames for   5 Hz Loop
#define COUNT_1HZ     1000      // Number of 1000 Hz frames for   1 Hz Loop
#define COUNT_0_5HZ     2000      // Number of 1000 Hz frames for   1 Hz Loop


#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

typedef union
{
    unsigned char time_all;
    struct
    {
        unsigned flag_500hz;
        unsigned flag_100hz;
        unsigned flag_50hz;
        unsigned flag_10hz;
        unsigned flag_5hz;
        unsigned flag_2hz;
        unsigned flag_1hz;
        unsigned flag_0_5hz;
    } time_sub;
} time_flag;

extern unsigned char Time_statr_flag;	//启动定时标志
extern unsigned char Time_out_flag;		//超时标志
extern unsigned int Time_wait_cnt;		//等待时间
extern time_flag TIME_FLAG;

uint32_t micros(void);
uint32_t millis(void);
void cycleCounterInit(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_nor(u16 time);


#endif
