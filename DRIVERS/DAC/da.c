/*
 * File      : da.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-30     Bernard      the first version
 */
#include "da.h"
#include "usart.h"

#ifdef SinWAVE
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420

/* Init Structure definition */
uint32_t Da_Idx = 0;

/* Private macro -------------------------------------------------------------*/
/* Private variables -正弦信号参数表--------------------------------------------------------*/
const uint16_t Sine12bit[32] =
{
    2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
    3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
    599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647
};
/* Private variables -三角信号参数表--------------------------------------------------------*/
uint32_t DualSine12bit[32];
#endif
/***********DA端口配置表******************/
const uint32_t DA_IO_CONFIG[3][3] =
{
    {RCC_APB2Periph_GPIOA, GPIO_Pin_4, POARTA},
    {RCC_APB2Periph_GPIOA, GPIO_Pin_5, POARTA},
    {0, 0, 0}
    //{RCC_APB2Periph_GPIOA,GPIO_Pin_9,POARTA}
};
/*****************************************/
/********************************************************************
***函数名称: DA_RCC_Configuration
***函数说明:时钟初始化
***输入参数:
***输出参数:
***
********************************************************************/
void DA_RCC_Configuration(void)
{
#ifdef SinWAVE
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
#endif
    /* DAC Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    /* TIM2 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
/********************************************************************
***函数名称: DA_GPIO_Configuration
***函数说明:端口初始化
***输入参数:
***输出参数:
***
********************************************************************/
void DA_GPIO_Configuration(void)
{
    unsigned int x;
    unsigned char i;
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef            DAC_InitStructure;
    DMA_InitTypeDef            DMA_InitStructure;
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    x = sizeof(DA_IO_CONFIG);
    x = x / 12;

    for(i = 0; i < x; i++)
    {
        if(DA_IO_CONFIG[i][0] != 0)
        {
            RCC_APB2PeriphClockCmd(DA_IO_CONFIG[i][0], ENABLE);
            GPIO_InitStructure.GPIO_Pin = DA_IO_CONFIG[i][1] & 0x0000ffff;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

            switch(DA_IO_CONFIG[i][2] & 0x0000000f)
            {
                case 1:
                    GPIO_Init(GPIOA, &GPIO_InitStructure);
                    break;

                case 2:
                    GPIO_Init(GPIOB, &GPIO_InitStructure);
                    break;

                case 3:
                    GPIO_Init(GPIOC, &GPIO_InitStructure);
                    break;

                case 4:
                    GPIO_Init(GPIOD, &GPIO_InitStructure);
                    break;

                case 5:
                    GPIO_Init(GPIOE, &GPIO_InitStructure);
                    break;

                case 6:
                    GPIO_Init(GPIOF, &GPIO_InitStructure);
                    break;

                case 7:
                    GPIO_Init(GPIOG, &GPIO_InitStructure);
                    break;
            }
        }
    }

#ifdef SinWAVE
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0x19;
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
#endif
#ifdef TriangelWAVE
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0xF;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
#endif
#ifdef SinWAVE
    /* DAC channel1 Configuration */
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);

    /* DAC channel2 Configuration */
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    /* Fill Sine32bit table */
    for(Da_Idx = 0; Da_Idx < 32; Da_Idx++)
    {
        DualSine12bit[Da_Idx] = (Sine12bit[Da_Idx] << 16) + (Sine12bit[Da_Idx]);
    }

    /* DMA1 channel4 configuration */
    DMA_DeInit(DMA2_Channel4);

    DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12RD_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&DualSine12bit;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 32;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA2_Channel4, &DMA_InitStructure);
    /* Enable DMA1 Channel4 */
    DMA_Cmd(DMA2_Channel4, ENABLE);
    /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
         automatically connected to the DAC converter. */
    DAC_Cmd(DAC_Channel_1, ENABLE);
    /* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
         automatically connected to the DAC converter. */
    DAC_Cmd(DAC_Channel_2, ENABLE);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
    /* Enable DMA for DAC Channel2 */
    DAC_DMACmd(DAC_Channel_2, ENABLE);
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
#endif
#ifdef TriangelWAVE
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);

    /* DAC channel2 Configuration */
    DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
    automatically connected to the DAC converter. */
    DAC_Cmd(DAC_Channel_1, ENABLE);

    /* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
    automatically connected to the DAC converter. */
    DAC_Cmd(DAC_Channel_2, ENABLE);

    /* Set DAC dual channel DHR12RD register */
    DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
#endif
}

void DA_Init()
{
    DA_RCC_Configuration();
    DA_GPIO_Configuration();
#ifdef DEBUG_MEG
    __DBG__("DAC功能打开!\r\n");
#endif
    DHT11_Rst();
}



