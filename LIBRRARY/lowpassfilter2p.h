/**
  ******************************************************************************
  * @file    lowpassfilter2p.h 
  * @author  zam
  * @version V1.0
  * @date    20-3-2018
  * @brief   Header for lowpassfilter2p.h module
  ******************************************************************************/
#ifndef __LOWPASSFILTER2P_H
#define __LOWPASSFILTER2P_H

#ifdef __cplusplus
	extern "C" {
	#endif
		
	#ifndef _M_PI_F
	#define _M_PI_F 3.14159265358979323846f
	#endif
	typedef struct 
	{
		float _sample_freq;
		float _cutoff_freq;
		float _a1;
		float _a2;
		float _b0;
		float _b1;
		float _b2;
		float _delay_element_1;
		float _delay_element_2;
	}lowpassfilter2p;

 lowpassfilter2p Set_Cutoff_Frequency(float samp_freq,float cutoff_freq);
 float LowPassFilter2p_Apply(lowpassfilter2p lowpassfilter_date,float samp_value);
	
#ifdef __cplusplus
		}
#endif
#endif