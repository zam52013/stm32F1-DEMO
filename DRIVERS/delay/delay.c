#include "delay.h"

static uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;
static volatile uint32_t usTicks = 0;
uint16_t frameCounter = 0;
time_flag TIME_FLAG;


unsigned char Time_statr_flag = 0;  //启动定时标志
unsigned char Time_out_flag = 0;    //超时标志
unsigned int Time_wait_cnt = 0;     //等待时间
/*--------------------------------------
函数名：time_tick
函数描述：用于超时作用
输入：无
输出：无
;-------------------------------------*/
void time_tick()
{
    static unsigned int Time_tic_cnt = 0;

    if(Time_statr_flag)
    {
        if(Time_tic_cnt >= Time_wait_cnt)
        {
            Time_out_flag = 1;
        }

        Time_tic_cnt++;
    }
    else
    {
        Time_tic_cnt = 0;
    }
}
void SysTick_Handler(void)
{
    sysTickCycleCounter = *DWT_CYCCNT;
    sysTickUptime++;
    frameCounter++;
    time_tick();

    if(frameCounter > FRAME_COUNT)
    {
        frameCounter = 1;
    }

    if((frameCounter % COUNT_500HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_500hz = TRUE;
    }

    if((frameCounter % COUNT_100HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_100hz = TRUE;
    }

    if((frameCounter % COUNT_50HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_50hz = TRUE;
    }

    if((frameCounter % COUNT_10HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_10hz = TRUE;
    }

    if((frameCounter % COUNT_5HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_5hz = TRUE;
    }

    if((frameCounter % COUNT_2HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_2hz = TRUE;
    }

    if((frameCounter % COUNT_1HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_1hz = TRUE;
    }

    if((frameCounter % COUNT_0_5HZ) == 0)
    {
        TIME_FLAG.time_sub.flag_0_5hz = TRUE;
    }
}
uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;

    do
    {
        timeMs = __LDREXW(&sysTickUptime);
        cycle = *DWT_CYCCNT;
        oldCycle = sysTickCycleCounter;
    }
    while(__STREXW(timeMs, &sysTickUptime));

    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

uint32_t millis(void)
{
    return sysTickUptime;
}
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;
}

void delay_us(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = *DWT_CYCCNT;

    for(;;)
    {
        register uint32_t current_count = *DWT_CYCCNT;
        uint32_t elapsed_us;
        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;
        // convert to microseconds
        elapsed_us = elapsed / usTicks;

        if(elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;
        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay_ms(uint32_t ms)
{
    while(ms--)
        delay_us(1000);
}
void delay_nor(u16 time)
{
    u16 i = 0;

    while(time--)
    {
        i = 10;

        while(i--);
    }
}
