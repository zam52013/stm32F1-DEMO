/*
 * File      : ms5611.h
 * This file is ef comp
 * COPYRIGHT (C) 2018,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at my addr
 *
 *    2320780008@qq.com
 * Change Logs:
 * Date           Author       ZAM
 * 2018-03-08     Bernard      the first version
 */
 #include "ms5611.h"
 #include "spi.h"
 #include "usart.h"
 #include "delay.h"
 #include <math.h>
 /*************存放PROM缓存****************/
 unsigned int MS5611_PROM_BUFF[7];
/*****************************************/
/*******************************************************************************
函 数 名： MS5611_Reset(void)
功能描述：复位5611
入口参数：	
返回参数：
创建时间: 2018-3-08 by zam
********************************************************************************/
 static void MS5611_Reset()
 {
		MS5611_RESET;
		SPI_ReadWriteByte(MS5611_SPI,CMD_MS5611_RESET);
		delay_ms(3);
		MS5611_SET;
 }
 /*******************************************************************************
函 数 名： MS5611_Read_Reg(void)
功能描述：读取寄存器值5611
 入口参数：	reg :地址
返回参数：读到的值
创建时间: 2018-3-08 by zam
********************************************************************************/
 unsigned char MS5611_Read_Reg(unsigned char reg)
{
		unsigned char value;
		MS5611_RESET;
		SPI_ReadWriteByte(MS5611_SPI,reg|DIR_READ);
		value=SPI_ReadWriteByte(MS5611_SPI,DIR_WRITE);
		MS5611_SET;
		return value;
}
 /*******************************************************************************
函 数 名： MS5611_Read_PROM(void)
功能描述：读取寄存器值PROM
 入口参数：
返回参数：
创建时间: 2018-3-08 by zam
********************************************************************************/
 static void MS5611_Read_PROM()
 {
	unsigned char i;
	unsigned char buff_h,buff_l;
	for(i=0;i<7;i++)
	{
		buff_h=MS5611_Read_Reg(CMD_MS5611_PROM_READ+i*2);
		buff_l=MS5611_Read_Reg(CMD_MS5611_PROM_READ+i*2+1);
		MS5611_PROM_BUFF[i]=((unsigned int)buff_h<<8)+buff_l;
	}	
 }
/*******************************************************************************
函 数 名： MS5611_ReadADCResult(void)
功能描述：读取寄存器值
 入口参数：command:地址
返回参数：值
创建时间: 2018-3-08 by zam
********************************************************************************/
 u32 MS5611_ReadADCResult(u8 command)
{
	u32 adc=0;
	u8 Dx_TempH=0,
	Dx_TempM=0,
	Dx_TempL=0;

	MS5611_RESET;

	SPI_ReadWriteByte(MS5611_SPI,command|DIR_READ);			 //发送转换指令
	//delay_ms(20);
	
	MS5611_SET;
	MS5611_RESET;
	/*
	注意:虽然24位数据只需要都3次,但是由时序图可知,第一次读取的8位数据是无效的,所以要"空读"一次
	这样就能把这个数据给去掉
	*/
	SPI_ReadWriteByte(MS5611_SPI,CMD_MS5611_ADC_READ);										//跳过开头8位无用字节
	Dx_TempH=MS5611_Read_Reg(DIR_READ);           //读数  
	Dx_TempM=MS5611_Read_Reg(DIR_READ);           //读数  
	Dx_TempL=MS5611_Read_Reg(DIR_READ);     		  //读数 
	MS5611_SET;
	
	adc=(((u32)Dx_TempH)<<16)+(((u32)Dx_TempM)<<8)+Dx_TempL;

	return adc;
}
/*******************************************************************************
函 数 名： MS5611_GetTP(double *TEMP,double *PRES)
功能描述：将得到的温度和压力转化为℃和mbar
 入口参数：
返回参数：
创建时间: 2018-3-08 by zam
********************************************************************************/
void MS5611_GetTP(double *TEMP,double *PRES)
{
	uint32_t D1,D2;
	
	double dT,OFF,SENS;
	double temperatureTemp,pressureTemp,T2;
	float Aux,OFF2,SENS2;  //温度校验值
	D1=MS5611_ReadADCResult(CMD_MS5611_D1_4096);   // 读未补偿的压力值 
	D2=MS5611_ReadADCResult(CMD_MS5611_D2_4096);   // 读未补偿的温度值 
	dT=D2-MS5611_PROM_BUFF[5]*pow(2.0,8.0);
		
	OFF=MS5611_PROM_BUFF[2]*pow(2.0,16.0)+dT*MS5611_PROM_BUFF[4]/pow(2.0,7.0);     
	SENS=MS5611_PROM_BUFF[1]*pow(2.0,15.0)+dT*MS5611_PROM_BUFF[3]/pow(2.0,8.0); 
	temperatureTemp=(2000+(dT*MS5611_PROM_BUFF[6])/pow(2.0,23.0));
	
	/*二阶温度补偿*/
	if(temperatureTemp < 2000)// second order temperature compensation when under 20 degrees C
	{
		T2 = (dT*dT) /pow(2.0,31.0);
		Aux = (temperatureTemp-2000)*(temperatureTemp-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(temperatureTemp < -1500)
		{
			Aux = (temperatureTemp+1500)*(temperatureTemp+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 =SENS2+ 5.5*Aux;
		}
	}
	else  //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;	
	
	*TEMP =( temperatureTemp - T2)/100;		//单位:℃

	*PRES=(D1*SENS/2097152-OFF)/32768/100;	//单位:mbar  1mbar=100Pa
}
 /*******************************************************************************
函 数 名：void Init_MS5611_Cs(void)
功能描述：5611片选初始化
入口参数：	无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
 static void Init_MS5611_Cs(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MS5611_RCC,ENABLE);
	GPIO_InitStructure.GPIO_Pin = MS5611_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MS5611_PORT, &GPIO_InitStructure);
	#ifdef DEBUG_MEG
		__DBG__("MS5611功能打开!\r\n");
	#endif
 }
 /*******************************************************************************
函 数 名：void MS5611_Init(void)
功能描述：5611初始化
入口参数：	无
返回参数：无
创建时间: 2018-1-05 by zam
********************************************************************************/
 void MS5611_Init()
 {
	Init_MS5611_Cs();
	MS5611_Reset();
	MS5611_Read_PROM();
 }
 
 
 
 