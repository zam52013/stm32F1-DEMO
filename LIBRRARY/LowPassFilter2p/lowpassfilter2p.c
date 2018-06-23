/**
  ******************************************************************************
  * @file    lowpassfilter2p.c
  * @author  zam
  * @version V1.0
  * @date    20-3-2018
  * @brief   Header for lowpassfilter2p.h module
  ******************************************************************************/
#include <math.h>
#include "lowpassfilter2p.h"

lowpassfilter2p Set_Cutoff_Frequency(float samp_freq,float cutoff_freq)
{
	lowpassfilter2p lowpassfilter_date;
	float fr;
	float ohm;
	float c;
	lowpassfilter_date._sample_freq=samp_freq;
	lowpassfilter_date._cutoff_freq=cutoff_freq;
	fr=samp_freq/cutoff_freq;
	ohm=tanf(_M_PI_F/fr);
	c=1.0+2.0*cosf(_M_PI_F/4.0f)*ohm+ohm*ohm;
	lowpassfilter_date._b0=ohm*ohm/c;
	lowpassfilter_date._b1=2.0f*lowpassfilter_date._b0;
	lowpassfilter_date._b2=lowpassfilter_date._b0;
	lowpassfilter_date._a1=2.0f*(ohm*ohm-1.0f)/c;
	lowpassfilter_date._a2=(1.0f-2.0f*cosf(_M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	return lowpassfilter_date;
}

float LowPassFilter2p_Apply(lowpassfilter2p lowpassfilter_date,float samp_value)
{
	float delay_element_0;
	float output;
	if(lowpassfilter_date._cutoff_freq<=0.0f)
	{
		return samp_value;
	}
	delay_element_0=samp_value-lowpassfilter_date._delay_element_1*lowpassfilter_date._a1-lowpassfilter_date._delay_element_2*lowpassfilter_date._a2;
	output=delay_element_0*lowpassfilter_date._b0+lowpassfilter_date._delay_element_1*lowpassfilter_date._b1+lowpassfilter_date._delay_element_2*lowpassfilter_date._b2;
	lowpassfilter_date._delay_element_2=lowpassfilter_date._delay_element_1;
	lowpassfilter_date._delay_element_1=delay_element_0;
	return output;
}