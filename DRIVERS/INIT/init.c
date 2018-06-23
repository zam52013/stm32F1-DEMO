#include "init.h"
#include "usart.h"

/*******************************************************************************
�� �� ����void IWDG_Init(void)
�������������Ź���ʼ��
��ڲ�����
���ز�����
����ʱ��: 2011.6.24
********************************************************************************/
void IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);    //��С
    IWDG_SetReload(0x1F4);       //40KHz�ڲ�ʱ�� (1/40000 * 64 * 0x1F4 = 0.8s)
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    IWDG_Enable();
    IWDG_ReloadCounter();
#ifdef DEBUG_MEG
    __DBG__("���Ź����ܴ�!\r\n");
#endif
}
/*******************************************************************************
�� �� ����void IWDG_Feed(void)
����������ι������
��ڲ�����
���ز�����
����ʱ��: 2011.6.24
********************************************************************************/
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//reload										   
}
/*************************************************************************
��������:Soft_Reset
��������:�����λ����
�������:��
�������:��
*************************************************************************/
void Soft_Reset()
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
/*******************************************************************************
* Function Name :void InterruptOrder(void)
* Description   :�ж����������ȼ�
* Input         :
* Output        :
* Other         :
* Date          :2011.10.27  11:50:05
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //���ȼ�����  ȫΪ��ռʽ���ȼ�
}
/*******************************************************************************
* Function Name :void SysTickInit(void)
* Description   :ϵͳ��ʱ��ʱ������
* Input         :
* Output        :
* Other         :ʱ��Ϊ1ms
* Date          :2011.11.03  12:59:13
*******************************************************************************/
void SysTickInit(void)
{
    SysTick_Config(SystemCoreClock / 1000);         //uCOSʱ��1ms
}


