#include "init.h"
#include "usart.h"

/*******************************************************************************
函 数 名：void IWDG_Init(void)
功能描述：看门狗初始化
入口参数：
返回参数：
创建时间: 2011.6.24
********************************************************************************/
void IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);    //最小
    IWDG_SetReload(0x1F4);       //40KHz内部时钟 (1/40000 * 64 * 0x1F4 = 0.8s)
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    IWDG_Enable();
    IWDG_ReloadCounter();
#ifdef DEBUG_MEG
    __DBG__("看门狗功能打开!\r\n");
#endif
}
/*******************************************************************************
函 数 名：void IWDG_Feed(void)
功能描述：喂狗程序
入口参数：
返回参数：
创建时间: 2011.6.24
********************************************************************************/
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}
/*************************************************************************
函数名称:Soft_Reset
函数功能:软件复位函数
输入参数:无
输出参数:无
*************************************************************************/
void Soft_Reset()
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
/*******************************************************************************
* Function Name :void InterruptOrder(void)
* Description   :中断向量，优先级
* Input         :
* Output        :
* Other         :
* Date          :2011.10.27  11:50:05
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //优先级设置  全为抢占式优先级
}
/*******************************************************************************
* Function Name :void SysTickInit(void)
* Description   :系统定时器时间配置
* Input         :
* Output        :
* Other         :时基为1ms
* Date          :2011.11.03  12:59:13
*******************************************************************************/
void SysTickInit(void)
{
    SysTick_Config(SystemCoreClock / 1000);         //uCOS时基1ms
}


