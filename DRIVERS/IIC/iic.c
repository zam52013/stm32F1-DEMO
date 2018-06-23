/*
 * File      : IIC.C
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-10-19     Bernard      the first version
 */
#include "iic.h"
#include "usart.h"
#include "delay.h"

#if 0
#ifdef SOFT_IIC
/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
static void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(SCL_RCC | SDA_RCC, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  SCL_GPIO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  SDA_GPIO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);
}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
    u8 i = 20; //这里可以优化速度  ，经测试最低到5还能写入

    while(i)
    {
        i--;
    }
}

void delay5ms(void)
{
    int i = 5000;

    while(i)
    {
        i--;
    }
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather  Start
****************************************************************************** */
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();

    if(!SDA_read)return FALSE;  //SDA线为低电平则总线忙,退出

    SDA_L;
    I2C_delay();

    if(SDA_read) return FALSE;  //SDA线为高电平则总线出错,退出

    SDA_L;
    I2C_delay();
    return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather  Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void)   //返回为:=1有ACK,=0无ACK
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();

    if(SDA_read)
    {
        SCL_L;
        I2C_delay();
        return FALSE;
    }

    SCL_L;
    I2C_delay();
    return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(unsigned char SendByte) //数据从高位到低位//
{
    unsigned char i = 8;

    while(i--)
    {
        SCL_L;
        I2C_delay();

        if(SendByte & 0x80)
            SDA_H;
        else
            SDA_L;

        SendByte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }

    SCL_L;
}
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave
****************************************************************************** */
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{
    unsigned char i = 8;
    unsigned char ReceiveByte = 0;
    SDA_H;

    while(i--)
    {
        ReceiveByte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();

        if(SDA_read)
        {
            ReceiveByte |= 0x01;
        }
    }

    SCL_L;
    return ReceiveByte;
}
//ZRX
//单字节写入*******************************************

bool Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)           //void
{
    if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_SendByte(REG_Address);    //设置低起始地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_SendByte(REG_data);

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_Stop();
    return TRUE;
}

bool I2C_Write_Multi(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *Date_buff, uint8_t Wrnum)
{
    if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_SendByte(REG_Address);    //设置低起始地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    while(Wrnum)
    {
        I2C_SendByte(*Date_buff);

        if(!I2C_WaitAck())
        {
            return FALSE;
        }
    }

    I2C_Stop();
    return TRUE;
}

//单字节读取*****************************************
unsigned char Single_Read(uint8_t SlaveAddress, uint8_t REG_Address)
{
    unsigned char REG_data;

    if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_SendByte((u8) REG_Address);   //设置低起始地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_Start();
    I2C_SendByte(SlaveAddress + 1);

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    REG_data = I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    return REG_data;
}
bool I2C_Read_Multi(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *Date_buff, uint8_t Renum)
{
    if(!I2C_Start())return FALSE;

    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_SendByte((u8) REG_Address);   //设置低起始地址

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    I2C_Start();
    I2C_SendByte(SlaveAddress + 1);

    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return FALSE;
    }

    while(Renum)
    {
        *Date_buff = I2C_RadeByte();
        Date_buff++;

        if(Renum == 1)
        {
            I2C_NoAck();
            I2C_Stop();
        }
        else
        {
            I2C_Ack();
        }

        Renum--;
    }

    return TRUE;
}

#else
__IO uint32_t Timeout = 0;
//时间溢出判断
#define Timed(x) Timeout = 0xFFFF; while (x) { if ((Timeout--)== 0) goto errReturn;}

uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t ReadAddr, uint8_t *Date_buff, uint8_t Renum)
{
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2Cx, ENABLE);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, Address, I2C_Direction_Transmitter);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2Cx, ReadAddr);
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) == RESET);
    I2C_GenerateSTART(I2Cx, ENABLE);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, Address, I2C_Direction_Receiver);
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) == RESET);

    while(Renum)
    {
        Timed(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *Date_buff = I2C_ReceiveData(I2Cx);
        Date_buff++;

        if(Renum == 1)
        {
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }
        else
        {
            I2C_AcknowledgeConfig(I2Cx, ENABLE);
        }

        Renum--;
    }

    return 0;
errReturn:
    return 1;
}
uint8_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t WrintAddr, uint8_t *Date_buff, uint8_t Wrnum)
{
    Timed(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2Cx, ENABLE);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, Address, I2C_Direction_Transmitter);
    Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2Cx, WrintAddr);
    Timed(I2C_GetFlagStatus(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);

    while(Wrnum)
    {
        I2C_SendData(I2Cx, *Date_buff);
        Timed(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        Wrnum--;
        Date_buff++;
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    return 0;
errReturn:
    return 1;
}

/********************************************************************
***函数名称: IIC_Io_Init
***函数说明:IIC端口初始化
***输入参数:无
***输出参数:无
***
********************************************************************/
static void IIC_Io_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
#ifdef IIC1_PERPH
    struct iic_config config1 = IIC_CONFIG_1;
#endif
#ifdef IIC2_PERPH
    struct iic_config config2 = IIC_CONFIG_2;
#endif
#ifdef IIC1_PERPH
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_InitStructure.I2C_Mode = config1.iic_mode;
    I2C_InitStructure.I2C_DutyCycle = config1.iic_duty;
    I2C_InitStructure.I2C_OwnAddress1 = config1.iic_addr;
    I2C_InitStructure.I2C_Ack = config1.iic_ack;
    I2C_InitStructure.I2C_AcknowledgedAddress = config1.iic_ackaddr;
    I2C_InitStructure.I2C_ClockSpeed = config1.iic_clock;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
#ifdef DEBUG_MEG
    __DBG__("IIC1功能打开!\r\n");
#endif
#endif
#ifdef IIC2_PERPH
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_InitStructure.I2C_Mode = config2.iic_mode;
    I2C_InitStructure.I2C_DutyCycle = config2.iic_duty;
    I2C_InitStructure.I2C_OwnAddress1 = config2.iic_addr;
    I2C_InitStructure.I2C_Ack = config2.iic_ack;
    I2C_InitStructure.I2C_AcknowledgedAddress = config2.iic_ackaddr;
    I2C_InitStructure.I2C_ClockSpeed = config2.iic_clock;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
#ifdef DEBUG_MEG
    __DBG__("IIC2功能打开!\r\n");
#endif
#endif
}
/********************************************************************
***函数名称: IIC_Dma_Init
***函数说明:IICdma
***输入参数:无
***输出参数:无
***
********************************************************************/
static void IIC_Dma_Init()
{
    DMA_InitTypeDef  I2CDMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef IIC1_DMA_TR
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(I2C1_DMA_CHANNEL_TX);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C1_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C1_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
    /* I2C1 RX DMA Channel configuration */
    DMA_DeInit(I2C1_DMA_CHANNEL_RX);
    DMA_Init(I2C1_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
    DMA_ITConfig(I2C1_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);
    DMA_ITConfig(I2C1_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);
    I2C_DMACmd(I2C1, ENABLE);
#endif
#ifdef IIC2_DMA_TR
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* I2C2 TX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_TX);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    I2CDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;
    I2CDMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;   /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;    /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_BufferSize = 0xFFFF;            /* This parameter will be configured durig communication */
    I2CDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    I2CDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    I2CDMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    I2CDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    I2CDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    I2CDMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(I2C2_DMA_CHANNEL_TX, &I2CDMA_InitStructure);
    /* I2C2 RX DMA Channel configuration */
    DMA_DeInit(I2C2_DMA_CHANNEL_RX);
    DMA_Init(I2C2_DMA_CHANNEL_RX, &I2CDMA_InitStructure);
    DMA_ITConfig(I2C2_DMA_CHANNEL_TX, DMA_IT_TC, ENABLE);
    DMA_ITConfig(I2C2_DMA_CHANNEL_RX, DMA_IT_TC, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);
#endif
}
#endif
/********************************************************************
***函数名称: IIC_Init
***函数说明:IIC初始化
***输入参数:无
***输出参数:无
***
********************************************************************/
void IIC_Init(void)
{
#ifdef SOFT_IIC
    I2C_GPIO_Config();
#else
    IIC_Io_Init();
    IIC_Dma_Init();
#endif
}
#endif

/**
  ******************************************************************************
  * @file    iic.c
  * @author  zam
  * @version V1.0
  * @date    08-4-2018
  * @brief   Header for iic.h module
  ******************************************************************************/

#define I2C_DEFAULT_TIMEOUT 30000

static I2C_TypeDef *I2Cx;

static volatile uint16_t i2cErrorCount = 0;

static volatile bool busy;

static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t writing;
static volatile uint8_t reading;
static volatile uint8_t *write_p;
static volatile uint8_t *read_p;

static void i2c_er_handler(I2C_TypeDef *I2Cx)
{
    volatile uint32_t SR1Register, SR2Register;
    SR1Register = I2Cx->SR1;                                             //Read the I2C1 status register

    if(SR1Register & 0x0F00)    //an error
    {
        // I2C1error.error = ((SR1Register & 0x0F00) >> 8);              //save error
        // I2C1error.job = job;    //the task
    }

    if(SR1Register & 0x0700)                                             //If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    {
        SR2Register = I2Cx->SR2;                                         //read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                         //disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)

        if(!(SR1Register & 0x0200) && !(I2Cx->CR1 & 0x0200))             //if we dont have an ARLO error, ensure sending of a stop
        {
            if(I2Cx->CR1 & 0x0100)                                       //We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
            {
                while(I2Cx->CR1 & 0x0100);                               //wait for any start to finish sending

                I2C_GenerateSTOP(I2Cx, ENABLE);                          //send stop to finalise bus transaction

                while(I2Cx->CR1 & 0x0200);                               //wait for stop to finish sending

                I2cInit(I2Cx);                                           //reset and configure the hardware
            }
            else
            {
                I2C_GenerateSTOP(I2Cx, ENABLE);                          //stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);    //Disable EVT and ERR interrupts while bus inactive
            }
        }
    }

    I2Cx->SR1 &= ~0x0F00;                                                //reset all the error bits to clear the interrupt
    busy = 0;
}

bool I2cWriteBuffer(I2C_TypeDef *I2Cx, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    uint8_t i;
    uint8_t my_data[16];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
    addr = addr_ << 1;
    reg = reg_;
    writing = 1;
    reading = 0;
    write_p = my_data;
    read_p = my_data;
    bytes = len_;
    busy = 1;

    if(len_ > 16)
        return FALSE;                                                    //too long

    for(i = 0; i < len_; i++)
        my_data[i] = data[i];

    if(!(I2Cx->CR2 & I2C_IT_EVT))                                        //if we are restarting the driver
    {
        if(!(I2Cx->CR1 & 0x0100))                                        //ensure sending a start
        {
            while(I2Cx->CR1 & 0x0200)
            {
                ;   //wait for any stop to finish sending
            }

            I2C_GenerateSTART(I2Cx, ENABLE);                             //send the start for the new job
        }

        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);             //allow the interrupts to fire off again
    }

    while(busy && --timeout > 0);

    if(timeout == 0)
    {
        i2cErrorCount++;
        I2cInit(I2Cx);                                                   //reinit peripheral + clock out garbage
        return FALSE;
    }

    return TRUE;
}

///////////////////////////////////////////////////////////////////////////////

bool I2cRead(I2C_TypeDef *I2Cx, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
    addr = addr_ << 1;
    reg = reg_;
    writing = 0;
    reading = 1;
    read_p = buf;
    write_p = buf;
    bytes = len;
    busy = 1;

    if(!(I2Cx->CR2 & I2C_IT_EVT))                                        //if we are restarting the driver
    {
        if(!(I2Cx->CR1 & 0x0100))                                        //ensure sending a start
        {
            while(I2Cx->CR1 & 0x0200)
            {
                ;   //wait for any stop to finish sending
            }

            I2C_GenerateSTART(I2Cx, ENABLE);                             //send the start for the new job
        }

        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);             //allow the interrupts to fire off again
    }

    while(busy && --timeout > 0);

    if(timeout == 0)
    {
        i2cErrorCount++;                                                 // reinit peripheral + clock out garbage
        I2cInit(I2Cx);
        return FALSE;
    }

    return TRUE;
}

void I2c_ev_handler(I2C_TypeDef *I2Cx)
{
    static uint8_t subaddress_sent, final_stop;                          //flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                 //index is signed -1==send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                          //read the status register here

    if(SReg_1 & 0x0001)                                                  //we just sent a start - EV5 in ref manual
    {
        I2Cx->CR1 &= ~0x0800;                                            //reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                             //make sure ACK is on
        index = 0;              //reset the index

        if(reading && (subaddress_sent || 0xFF == reg))                  //we have sent the subaddr
        {
            subaddress_sent = 1;                                         //make sure this is set in case of no subaddress, so following code runs correctly

            if(bytes == 2)
                I2Cx->CR1 |= 0x0800;                                     //set the POS bit so NACK applied to the final byte in the two byte read

            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);     //send the address and set hardware mode
        }
        else                                                             //direction is Tx, or we havent sent the sub and rep start
        {
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter);  //send the address and set hardware mode

            if(reg != 0xFF)                                              //0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                              //send a subaddress
        }
    }
    else if(SReg_1 & 0x0002)                                             //we just sent the address - EV6 in ref manual
    {
        //Read SR1,2 to clear ADDR
        volatile uint8_t a;
        __DMB();                                                         //memory fence to control hardware

        if(bytes == 1 && reading && subaddress_sent)                     //we are receiving 1 byte - EV6_3
        {
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                        //turn off ACK
            __DMB();
            a = I2Cx->SR2;                                               //clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                              //program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                      //allow us to have an EV7
        }
        else                                                             //EV6 and EV6_1
        {
            a = I2Cx->SR2;                                               //clear the ADDR here
            __DMB();

            if(bytes == 2 && reading && subaddress_sent)                 //rx 2 bytes - EV6_1
            {
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                    //turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                 //disable TXE to allow the buffer to fill
            }
            else if(bytes == 3 && reading && subaddress_sent)            //rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                 //make sure RXNE disabled so we get a BTF in two bytes time
            else                                                         //receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    }
    else if(SReg_1 & 0x004)                                              //Byte transfer finished - EV7_2, EV7_3 or EV8_2
    {
        final_stop = 1;

        if(reading && subaddress_sent)                                   //EV7_2, EV7_3
        {
            if(bytes > 2)                                                //EV7_2
            {
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                    //turn off ACK
                read_p[index++] = I2C_ReceiveData(I2Cx);                 //read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                          //program the Stop
                final_stop = 1;                                          //required to fix hardware
                read_p[index++] = I2C_ReceiveData(I2Cx);                 //read data N-1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                  //enable TXE to allow the final EV7
            }
            else                                                         //EV7_3
            {
                if(final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                      //program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                     //program a rep start

                read_p[index++] = I2C_ReceiveData(I2Cx);                 //read data N-1
                read_p[index++] = I2C_ReceiveData(I2Cx);                 //read data N
                index++;                                                 //to show job completed
            }
        }
        else                                                             //EV8_2, which may be due to a subaddress sent or a write completion
        {
            if(subaddress_sent || (writing))
            {
                if(final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                      //program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                     //program a rep start

                index++;                                                 //to show that the job is complete
            }
            else                                                         //We need to send a subaddress
            {
                I2C_GenerateSTART(I2Cx, ENABLE);                         //program the repeated Start
                subaddress_sent = 1;                                     //this is set back to zero upon completion of the current task
            }
        }

        while(I2Cx->CR1 & 0x0100)
        {
            ;   //we must wait for the start to clear, otherwise we get constant BTF
        }
    }
    else if(SReg_1 & 0x0040)                                             //Byte received - EV7
    {
        read_p[index++] = I2C_ReceiveData(I2Cx);

        if(bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                     //disable TXE to allow the buffer to flush so we can get an EV7_2

        if(bytes == index)                                               //We have completed a final EV7
            index++;                                                     //to show job is complete
    }
    else if(SReg_1 & 0x0080)                                             //Byte transmitted -EV8/EV8_1
    {
        if(index != -1)
        {
            //we dont have a subaddress to send
            I2C_SendData(I2Cx, write_p[index++]);

            if(bytes == index)                                           //we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                 //disable TXE to allow the buffer to flush
        }
        else
        {
            index++;
            I2C_SendData(I2Cx, reg);                                     //send the subaddress

            if(reading || !bytes)                                        //if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                 //disable TXE to allow the buffer to flush
        }
    }

    if(index == bytes + 1)                                               //we have completed the current job
    {
        //Completion Tasks go here
        //End of completion tasks
        subaddress_sent = 0;                                             //reset this here

        if(final_stop)                                                   //If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);        //Disable EVT and ERR interrupts while bus inactive

        busy = 0;
    }
}

//void i2cUnstick(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    uint8_t i;

//    // SCL  PB10
//    // SDA  PB11

//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;

//    GPIO_Init(GPIOB, &GPIO_InitStructure);

//    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);

//    for (i = 0; i < 8; i++)
//    {
//        while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10))               // Wait for any clock stretching to finish
//            delayMicroseconds(3);

//        GPIO_ResetBits(GPIOB, GPIO_Pin_10);                              //Set bus low
//        delayMicroseconds(3);

//        GPIO_SetBits(GPIOB, GPIO_Pin_10);                                //Set bus high
//        delayMicroseconds(3);
//    }

//    // Generate a start then stop condition

//    GPIO_ResetBits(GPIOB, GPIO_Pin_11);                                  //Set bus data low
//    delayMicroseconds(3);
//    GPIO_ResetBits(GPIOB, GPIO_Pin_10);                                  //Set bus scl low
//    delayMicroseconds(3);
//    GPIO_SetBits(GPIOB, GPIO_Pin_10);                                    //Set bus scl high
//    delayMicroseconds(3);
//    GPIO_SetBits(GPIOB, GPIO_Pin_11);                                    //Set bus sda high
//    delayMicroseconds(3);
//}

///////////////////////////////////////////////////////////////////////////////

void I2cInit(I2C_TypeDef *I2Cx)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef IIC1_PERPH
    struct iic_config config1 = IIC_CONFIG_1;
#endif
#ifdef IIC2_PERPH
    struct iic_config config2 = IIC_CONFIG_2;
#endif
#ifdef IIC1_PERPH
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_InitStructure.I2C_Mode = config1.iic_mode;
    I2C_InitStructure.I2C_DutyCycle = config1.iic_duty;
    I2C_InitStructure.I2C_OwnAddress1 = config1.iic_addr;
    I2C_InitStructure.I2C_Ack = config1.iic_ack;
    I2C_InitStructure.I2C_AcknowledgedAddress = config1.iic_ackaddr;
    I2C_InitStructure.I2C_ClockSpeed = config1.iic_clock;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
#ifdef DEBUG_MEG
    __DBG__("IIC1功能打开!\r\n");
#endif
#endif
#ifdef IIC2_PERPH
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_InitStructure.I2C_Mode = config2.iic_mode;
    I2C_InitStructure.I2C_DutyCycle = config2.iic_duty;
    I2C_InitStructure.I2C_OwnAddress1 = config2.iic_addr;
    I2C_InitStructure.I2C_Ack = config2.iic_ack;
    I2C_InitStructure.I2C_AcknowledgedAddress = config2.iic_ackaddr;
    I2C_InitStructure.I2C_ClockSpeed = config2.iic_clock;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
#ifdef DEBUG_MEG
    __DBG__("IIC2功能打开!\r\n");
#endif
#endif
//    i2cUnstick();                                                        // clock out stuff to make sure slaves arent stuck
//    // SCL  PB10
//    // SDA  PB11
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//    I2Cx = I2C;
//    // Init I2C
//    I2C_DeInit(I2Cx);
//    I2C_StructInit(&I2C_InitStructure);
//    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);                //Disable EVT and ERR interrupts - they are enabled by the first request
//    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
//    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
//    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_InitStructure.I2C_ClockSpeed          = 400000;
//    I2C_Init(I2Cx, &I2C_InitStructure);
//    I2C_Cmd(I2Cx, ENABLE);
#ifdef IIC1_PERPH
    // I2C ER Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // I2C EV Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
#endif
#ifdef IIC2_PERPH
    // I2C ER Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // I2C EV Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

///////////////////////////////////////////////////////////////////////////////

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2C1);
}

void I2C1_EV_IRQHandler(void)
{
    I2c_ev_handler(I2C1);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2C2);
}

void I2C2_EV_IRQHandler(void)
{
    I2c_ev_handler(I2C2);
}
