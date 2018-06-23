/*
 * File      : IIC.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *+-----------------------------------------------------------+
 * |                     引脚说明                            |
 +-----------------------------+---------------+-------------+
 * IIC1:   PB6...SCL      PB7....SDA
 * IIC2:		PB10...SCL     PB11...SDA
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-10-19     Bernard      the first version
 */
#ifndef __IIC_H
#define __IIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"


/***IIC设备选择****/
#define IIC1_PERPH

//#define IIC2_PERPH
//#define SOFT_IIC
/*********************/

//************************************
/*模拟IIC端口输出输入定义*/
/**************DHT端口配置**********************/
#define SCL_PORT GPIOB
#define SCL_RCC RCC_APB2Periph_GPIOB
#define SCL_GPIO GPIO_Pin_6

#define SDA_PORT GPIOB
#define SDA_RCC RCC_APB2Periph_GPIOB
#define SDA_GPIO GPIO_Pin_7


#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6

#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7
//////////////////////////////////////////////////////////
/***IIC-DMA设备选择****/
//#define IIC1_DMA_TR

//#define IIC2_DMA_TR
/*********************/
#define I2C1_DMA_CHANNEL_TX           DMA1_Channel6
#define I2C1_DMA_CHANNEL_RX           DMA1_Channel7

#define I2C2_DMA_CHANNEL_TX           DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX           DMA1_Channel5

#define I2C1_DR_Address              0x40005410
#define I2C2_DR_Address              0x40005810

typedef enum
{
    Polling = 0x00,
    Interrupt = 0x01,
    DMA = 0x02
} I2C_ProgrammingModel;

/*********************/
typedef enum {Error = 0, Success = !Error } Status;
/*********************/

/********配置参数*************/
#define IIC_CONFIG_1	\
{						\
	I2C_Mode_I2C,		\
	I2C_DutyCycle_2,	\
	0xA0,				\
	I2C_Ack_Enable,		\
	I2C_AcknowledgedAddress_7bit,\
	100000						\
}
#define IIC_CONFIG_2	\
{						\
	I2C_Mode_I2C,		\
	I2C_DutyCycle_2,	\
	0x0A,				\
	I2C_Ack_Enable,		\
	I2C_AcknowledgedAddress_7bit,\
	100000						\
}

/*********************/
struct iic_config
{
    uint16_t iic_mode;		/*配置模式*/
    uint16_t iic_duty;		/*时钟频率*/
    uint16_t iic_addr;		/*本身地址*/
    uint16_t iic_ack;		/*应答使能*/
    uint16_t iic_ackaddr;	/*数据位*/
    uint32_t iic_clock;		/*时钟频率*/
};

/****iic函数*****************/
#ifdef SOFT_IIC
bool Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
bool I2C_Write_Multi(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *Date_buff, uint8_t Wrnum);
unsigned char Single_Read(uint8_t SlaveAddress, uint8_t REG_Address);
bool I2C_Read_Multi(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *Date_buff, uint8_t Renum);

#else
uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t ReadAddr, uint8_t *Date_buff, uint8_t Renum);
uint8_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t Address, uint8_t WrintAddr, uint8_t *Date_buff, uint8_t Wrnum);
#endif

void I2cInit(I2C_TypeDef *I2Cx);

///////////////////////////////////////////////////////////////////////////////

bool I2cWriteBuffer(I2C_TypeDef *I2Cx, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);

///////////////////////////////////////////////////////////////////////////////

bool I2cRead(I2C_TypeDef *I2Cx, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf);
///////////////////////////////////////////////////////////////////////////////

void IIC_Init(void);

/*********************/
#ifdef __cplusplus
}
#endif

#endif
