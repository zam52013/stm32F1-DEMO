/*
 * File      : main.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2017-09-26     Bernard      the first version
 */

#include "include.h"


int main(int argc, char *argv[])
{
	uint64_t times;
	
    SystemInit();
    cycleCounterInit();
    SysTick_Config(SystemCoreClock / 1000);
	
    NVIC_Configuration();   //中断优先级设置
    Usart_init();
//    Spi_init();
//    Led_Init();
//    DHT11_Init();
//    LCD_Init();
		Init_NB_Gprs();
	
//    LCD_Clear(BLACK);    //清屏
//    gui_circle(128 / 2, 128 / 2, WHITE, 128 / 2 - 2, 0); //画圆
//    gui_circle(128 / 2, 128 / 2, WHITE, 128 / 2 - 3, 0); //画圆
//    Gui_StrCenter(0, 10, BLUE, YELLOW, "Mcudev", 16, 1); //居中显示
//    Gui_StrCenter(0, 28, RED, BLUE, "Temp:32℃", 16, 1); //居中显示
//    Gui_StrCenter(0, 46, YELLOW, BLUE, "Hum:20%", 16, 1); //居中显示
//    Gui_StrCenter(0, 64, WHITE, BLUE, "VOL:3.2V", 16, 1); //居中显示
//    Gui_StrCenter(0, 82, GREEN, BLUE, "I:1.25mA", 16, 1); //居中显示
//    Gui_StrCenter(0, 100, RED, BLUE, "12:20AM", 16, 1); //居中显示
		
    IWDG_Init();
    while(1)
    {
        if(TIME_FLAG.time_sub.flag_2hz)
        {
            IWDG_Feed();
          //  led_on_off(LED_3);
            TIME_FLAG.time_sub.flag_2hz = FALSE;
        }

        if(TIME_FLAG.time_sub.flag_1hz)
        {
  //      	mavlink_msg_heartbeat_send(0,MAV_TYPE_HEXAROTOR,MAV_AUTOPILOT_PX4,MAV_MODE_FLAG_STABILIZE_ENABLED,0x0402,MAV_STATE_ACTIVE);
//        mavlink_msg_param_set_send(0,17,84,"abc",20,MAV_PARAM_TYPE_REAL32);
      //    led_on_off(LED_5);
          TIME_FLAG.time_sub.flag_1hz = FALSE;
        }

        if(TIME_FLAG.time_sub.flag_0_5hz)
        {
        	times+=2000;
 //         	led_on_off(LED_4);
//				mavlink_msg_raw_imu_send(0, times, 2231, 1245, 45254, 4524, 5343, 45, 4523, 54443, 4543);
//        mavlink_msg_altitude_send(0,times,123.34,345.56,234.23,234.23,23.4,23.2);
//				mavlink_msg_altitude_send(0,times,123.34,345.56,234.23,234.23,23.4,23.2);
//				mavlink_msg_altitude_send_struct(0,&altitude);
 //         if(!DHT11_Read_Data())
 //         {
//						mavlink_msg_attitude_send(0,times,SENSOR_DHT_11.temperature,SENSOR_DHT_11.humidity,0.0,0.0,0.0,0.0);
//            Mavlink_msg_dht11_send(UART5,12);
//#ifdef DEBUG_MEG
//                __DBG__("温度=%.2f\r\n", SENSOR_DHT_11.temperature);
//#endif
//#ifdef DEBUG_MEG
//                __DBG__("湿度=%.2f\r\n", SENSOR_DHT_11.humidity);
//#endif
 //          }
           TIME_FLAG.time_sub.flag_0_5hz = FALSE;
        }
    }

    return 0;
}


