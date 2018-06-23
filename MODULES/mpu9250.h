/*
 * File      : mpu9250.h
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */
 #ifndef __MPU9250_H
 #define __MPU9250_H
 
 #include "stm32f10x.h"
 
 
  /***SPI…Ë±∏MPU9250 cs—°‘Ò**************************************/
#define MPU9250_SPI SPI3
#define MPU9250_PORT GPIOB
#define MPU9250_RCC RCC_APB2Periph_GPIOB
#define MPU9250_CS GPIO_Pin_7
#define MPU9250_SET GPIO_SetBits(MPU9250_PORT,MPU9250_CS)
#define MPU9250_RESET GPIO_ResetBits(MPU9250_PORT,MPU9250_CS)
 /*****************************************************/
 
#define DIR_READ			0x80
#define DIR_WRITE			0x00
#define DMP_VALE 1073741824.0

// MPU 9250 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C

#define DMP_CFG_1        0x70
#define DMP_CFG_2        0x71

#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_ACCEL_CONFIG2_41HZ		0x03

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10


#define MPU_WHOAMI_9250			0x71

#define MPU9250_ACCEL_DEFAULT_RATE	1000
#define MPU9250_ACCEL_MAX_OUTPUT_RATE			280
#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_RATE	1000
/* rates need to be the same between accel and gyro */
#define MPU9250_GYRO_MAX_OUTPUT_RATE			MPU9250_ACCEL_MAX_OUTPUT_RATE
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_DEFAULT_ONCHIP_FILTER_FREQ	41

#define MPU9250_ONE_G					9.80665f

#define GROFS  65.5
#define ACCFS  8192
#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)
#define PI 3.1415926535898

extern u8 raw_dat[14];

void Mpu9250_Init(void);
void Get_Raw_Date(void);

#endif
