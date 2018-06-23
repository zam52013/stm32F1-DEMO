/**
  ******************************************************************************
  * @file    mpu6050calibraton.h 
  * @author  zam
  * @version V1.0
  * @date    08-4-2018
  * @brief   Header for mpu6050calibraton.h module
  ******************************************************************************/
void Mpu6050_Tpc_Calibration()
{
		unsigned int sampleRate      = 1000;
    unsigned int numberOfSamples = 2000;

    float accelBias1[3]       = { 0.0f, 0.0f, 0.0f };
    float gyroBias1[3]        = { 0.0f, 0.0f, 0.0f };
    float mpu6050Temperature1 = 0.0f;

    float accelBias2[3]       = { 0.0f, 0.0f, 0.0f };
    float gyroBias2[3]        = { 0.0f, 0.0f, 0.0f };
    float mpu6050Temperature2 = 0.0f;
		unsigned int index;
		for (index = 0; index < numberOfSamples; index++)
    {
        //读取传感器数据//readMPU6050();

        //对传感器累积求和//rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;

        //accelBias1[XAXIS]    += rawAccel[XAXIS].value;
        //accelBias1[YAXIS]    += rawAccel[YAXIS].value;
        //accelBias1[ZAXIS]    += rawAccel[ZAXIS].value;
        //gyroBias1[ROLL ]     += rawGyro[ROLL ].value;
        //gyroBias1[PITCH]     += rawGyro[PITCH].value;
        //gyroBias1[YAW  ]     += rawGyro[YAW  ].value;
        //mpu6050Temperature1  += (float)(rawMPU6050Temperature.value) / 340.0f + 35.0f;

        //加入延时进去//delayMicroseconds(sampleRate);
			//求平均
		}
			//accelBias1[XAXIS]   /= (float) numberOfSamples;
			//accelBias1[YAXIS]   /= (float) numberOfSamples;
			//accelBias1[ZAXIS]   /= (float) numberOfSamples;
			//gyroBias1[ROLL ]    /= (float) numberOfSamples;
			//gyroBias1[PITCH]    /= (float) numberOfSamples;
			//gyroBias1[YAW  ]    /= (float) numberOfSamples;
			//mpu6050Temperature1 /= (float) numberOfSamples;
			
			//等待一段时间 delay(600000); 
			
		for (index = 0; index < numberOfSamples; index++)
		{
				//读取数据//readMPU6050();

				//累积求和//rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;

				//accelBias2[XAXIS]    += rawAccel[XAXIS].value;
				//accelBias2[YAXIS]    += rawAccel[YAXIS].value;
				//accelBias2[ZAXIS]    += rawAccel[ZAXIS].value;
				//gyroBias2[ROLL ]     += rawGyro[ROLL ].value;
				//gyroBias2[PITCH]     += rawGyro[PITCH].value;
				//gyroBias2[YAW  ]     += rawGyro[YAW  ].value;
				//mpu6050Temperature2  += (float)(rawMPU6050Temperature.value) / 340.0f + 35.0f;

				//延时一段时间delayMicroseconds(sampleRate);
		}
		//累积求平均
		//accelBias2[XAXIS]   /= (float) numberOfSamples;
    //accelBias2[YAXIS]   /= (float) numberOfSamples;
    //accelBias2[ZAXIS]   /= (float) numberOfSamples;
    //gyroBias2[ROLL ]    /= (float) numberOfSamples;
   //gyroBias2[PITCH]    /= (float) numberOfSamples;
    //gyroBias2[YAW  ]    /= (float) numberOfSamples;
    //mpu6050Temperature2 /= (float) numberOfSamples;

		//计算温度漂移
	  //eepromConfig.accelTCBiasSlope[XAXIS]     = (accelBias2[XAXIS] - accelBias1[XAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);
    //eepromConfig.accelTCBiasSlope[YAXIS]     = (accelBias2[YAXIS] - accelBias1[YAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);
    //eepromConfig.accelTCBiasSlope[ZAXIS]     = (accelBias2[ZAXIS] - accelBias1[ZAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);

		//计算温度漂移偏差
    //eepromConfig.accelTCBiasIntercept[XAXIS] = accelBias2[XAXIS] - eepromConfig.accelTCBiasSlope[XAXIS] * mpu6050Temperature2;
    //eepromConfig.accelTCBiasIntercept[YAXIS] = accelBias2[YAXIS] - eepromConfig.accelTCBiasSlope[YAXIS] * mpu6050Temperature2;
    //eepromConfig.accelTCBiasIntercept[ZAXIS] = accelBias2[ZAXIS] - eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6050Temperature2;

    //eepromConfig.gyroTCBiasSlope[ROLL ]      = (gyroBias2[ROLL ] - gyroBias1[ROLL ]) / (mpu6050Temperature2 - mpu6050Temperature1);
    //eepromConfig.gyroTCBiasSlope[PITCH]      = (gyroBias2[PITCH] - gyroBias1[PITCH]) / (mpu6050Temperature2 - mpu6050Temperature1);
    //eepromConfig.gyroTCBiasSlope[YAW  ]      = (gyroBias2[YAW  ] - gyroBias1[YAW  ]) / (mpu6050Temperature2 - mpu6050Temperature1);

    //eepromConfig.gyroTCBiasIntercept[ROLL ]  = gyroBias2[ROLL ] - eepromConfig.gyroTCBiasSlope[ROLL ] * mpu6050Temperature2;
    //eepromConfig.gyroTCBiasIntercept[PITCH]  = gyroBias2[PITCH] - eepromConfig.gyroTCBiasSlope[PITCH] * mpu6050Temperature2;
    //eepromConfig.gyroTCBiasIntercept[YAW  ]  = gyroBias2[YAW  ] - eepromConfig.gyroTCBiasSlope[YAW  ] * mpu6050Temperature2;
}