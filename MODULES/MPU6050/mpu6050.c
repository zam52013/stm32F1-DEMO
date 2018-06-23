/*
 * File      : mpu6050.c
 * This file is ef comp
 * COPYRIGHT (C) 2017,
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 *
 * Change Logs:
 * Date           Author       ZAM
 * 2018-04-05     Bernard      the first version
 */
#include "mpu6050.h"
#include "iic.h"
#include "delay.h"
#include "usart.h"

#include <math.h>

int16_t orientationMatrix[9];
mpu6050_raw Sensors_Raw;


uint8_t MargAHRSinitialized = FALSE;//���˳�ʼ��

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// auxiliary variables to reduce number of repeated operations
float q0q0, q0q1, q0q2, q0q3;
float q1q1, q1q2, q1q3;
float q2q2, q2q3;
float q3q3;

#define SQR(x)  ((x) * (x))

float accelOneG = 9.8065;
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
float accConfidenceDecay = 0.0f;
float accConfidence      = 1.0f;
float KpAcc, KiAcc;
float KpMag;
float KiMag;

void orientIMU(uint8_t imuOrientation) //����IMU��Ԫ�ķ�λȷ������A��ֵ
{
    switch(imuOrientation)
    {
        case 1: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 2: // Dot Front/Right/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 3: // Dot Back/Right/Top
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 4: // Dot Back/Left/Top
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;

        case 5: // Dot Front/Left/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] = -1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] = -1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 6: // Dot Front/Right/Bottom
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] = -1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 7: // Dot Back/Right/Bottom
            orientationMatrix[0] =  0;
            orientationMatrix[1] =  1;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  1;
            orientationMatrix[4] =  0;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        case 8: // Dot Back/Left/Bottom
            orientationMatrix[0] = -1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] = -1;
            break;

        default: // Dot Front/Left/Top
            orientationMatrix[0] =  1;
            orientationMatrix[1] =  0;
            orientationMatrix[2] =  0;
            orientationMatrix[3] =  0;
            orientationMatrix[4] =  1;
            orientationMatrix[5] =  0;
            orientationMatrix[6] =  0;
            orientationMatrix[7] =  0;
            orientationMatrix[8] =  1;
            break;
    }
}
void matrixMultiply(uint8_t aRows, uint8_t aCols_bRows, uint8_t bCols, int16_t matrixC[], int16_t matrixA[], int16_t matrixB[])
{
    uint8_t i, j, k;

    for(i = 0; i < aRows * bCols; i++)
    {
        matrixC[i] = 0.0;
    }

    for(i = 0; i < aRows; i++)
    {
        for(j = 0; j < aCols_bRows; j++)
        {
            for(k = 0;  k < bCols; k++)
            {
                matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
            }
        }
    }
}
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];
    //ͨ��ʹ��atan2f����������ٶ����ݵõ�ŷ���� ��ת�Ǻ� �����ǡ�
    accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
    accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);
    //���� smoothAcc ��ͨ�����ٶ����ݾ���atan2f�������������ŷ���ǣ����ҽ�����һ���ͺ��˲�
    //�����˲��㷨Ҳ���ڵ�ͨ�˲���һ�֣����ŵ㣺 �������Ը��ž������õ��������� �����ڲ���Ƶ�ʽϸߵĳ���,
    // ȱ�㣺 ��λ�ͺ������ȵ� �ͺ�̶�ȡ����aֵ��С�� ���������˲�Ƶ�ʸ��ڲ���Ƶ�ʵ�1/2�ĸ����ź�,������a��ֵ��99.0f
    smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
    smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;
    gyroRate[PITCH] =  gyroData[PITCH];
    //ͨ�������˲����ںϸ��ݼ��ٶȺ������Ǽ�������ĽǶȣ�orient[PITCH]���ϴ��ںϺ�ĽǶȣ�gyroRate[PITCH] * dt�Ǹ���������
    //���ݼ���õ��ĽǶȣ����ٶ�*����ʱ�������ǻ��ȣ����ȺͽǶȺ����׵Ŀ����໥ת������ΪʲôҪ���������ںϣ�
    //�𣺼��ٶȼƺ������Ƕ��ܼ������̬����Ϊ��Ҫ�������ںϣ�����Ϊ���ٶȼƶ���֮����Ŷ������У����������ݼ��������̬���ţ�
    //����������Ȼ������Щ�����У�������ʹ�������ǻ����Ư�ƣ��������Ҫ���л������������������ǣ��������ż��ٶȼ�.
    //��ͨ�����ٶȼƵõ��ĽǶȼ�ȥ��һ���ںϺ�ĽǶ�Ȼ�����һ������ϵ�����������ϵ��ԽС���ںϵļ��ٶȼƵ����ݱ���ԽС��
    //�������������ǣ����������ǵı���������1���������ż��ٶȼƣ����ٶȼƵ������������������ǵ�Ư�Ʋ�������
    //�����������ǵ�Ư�ƽ�������������Ч�������˼��ٶȼƺ������Ǹ��Ե�������ʱ���ƫ��.
    orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);
    //���������һ����Һ�x�ᵥ������Ƕ�,��Ϊ����֪�������Ĵ�С
    //����IMU��Ԫ�����Ǿ�ֹˮƽ״̬
    gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
    //ͨ�������˲����ںϸ��ݼ��ٶȺ������Ǽ�������ĽǶȣ�orient[PITCH]���ϴ��ںϺ�ĽǶȣ�gyroRate[PITCH] * dt�Ǹ���������
    //���ݼ���õ��ĽǶȣ����ٶ�*����ʱ�������ǻ��ȣ����ȺͽǶȺ����׵Ŀ����໥ת������ΪʲôҪ���������ںϣ�
    //�𣺼��ٶȼƺ������Ƕ��ܼ������̬����Ϊ��Ҫ�������ںϣ�����Ϊ���ٶȼƶ���֮����Ŷ������У����������ݼ��������̬���ţ�
    //����������Ȼ������Щ�����У�������ʹ�������ǻ����Ư�ƣ��������Ҫ���л������������������ǣ��������ż��ٶȼ�.
    //��ͨ�����ٶȼƵõ��ĽǶȼ�ȥ��һ���ںϺ�ĽǶ�Ȼ�����һ������ϵ�����������ϵ��ԽС���ںϵļ��ٶȼƵ����ݱ���ԽС��
    //�������������ǣ����������ǵı���������1���������ż��ٶȼƣ����ٶȼƵ������������������ǵ�Ư�Ʋ�������
    //�����������ǵ�Ư�ƽ�������������Ч�������˼��ٶȼƺ������Ǹ��Ե�������ʱ���ƫ��.
    orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);
    //���������һ����Һ�x�ᵥ������Ƕ�,��Ϊ����֪�������Ĵ�С
    //����IMU��Ԫ�����Ǿ�ֹˮƽ״̬
    gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
    orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);//�������ǽ��л��ֵõ�ƫ����YAW
}
//====================================================================================================
// Initialization
//====================================================================================================
//���˲ο�ϵͳ��ʼ��
void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;
    //ʹ�ü��ٶ����ݼ���ŷ���� ����ת�Ǻ͸�����
    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);
    //��ŷ���ǽ������Һ����Ҽ��㣬�ֱ�Ѽ�������������
    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);
    magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;
    initialHdg = atan2f(-magY, magX);//���㺽���
    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);
    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);
    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);
    //�õ���Ԫ��
    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    //�Ѽ���ο������õ���ֵ�ȶ������,�����ظ�����,��ΪMargAHRSupdate��������Ҫ�õ���
    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}
///////////////////////////////////////////////////////////////////////////////
// Constrain
///////////////////////////////////////////////////////////////////////////////

float constrain(float input, float minValue, float maxValue)
{
    if(input < minValue)
        return minValue;
    else if(input > maxValue)
        return maxValue;
    else
        return input;
}
void calculateAccConfidence(float accMag)
{
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity
    static float accMagP = 1.0f;
    accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's
    accMag  = HardFilter(accMagP, accMag);
    accMagP = accMag;
    accConfidence = constrain(1.0f - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.0f, 1.0f);
}
//====================================================================================================
// Function
//====================================================================================================
//���˲ο�ϵͳ����
void MargAHRSupdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    uint8_t magDataUpdate, float dt)
{
    float norm, normR;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float q0i, q1i, q2i, q3i;
    float exAcc    = 0.0f,    eyAcc = 0.0f,    ezAcc = 0.0f; // accel error
    float exAccInt = 0.0f, eyAccInt = 0.0f, ezAccInt = 0.0f; // accel integral error
    float exMag    = 0.0f, eyMag    = 0.0f, ezMag    = 0.0f; // mag error
    float exMagInt = 0.0f, eyMagInt = 0.0f, ezMagInt = 0.0f; // mag integral error
    float kpAcc, kiAcc;
    float halfT;
    //-------------------------------------------

    if((MargAHRSinitialized == FALSE))  // HJI && (magDataUpdate == true))
    {
        //������˲ο�ϵͳ������û�г�ʼ��������ôִ��AHRS��ʼ��
        MargAHRSinit(ax, ay, az, mx, my, mz);
        MargAHRSinitialized = TRUE;//��Ǻ��˲ο�ϵͳ�����Ѿ���ʼ����
    }

    //-------------------------------------------

    if(MargAHRSinitialized == TRUE) //������˲ο�ϵͳ�����Ѿ���ʼ����
    {
        halfT = dt * 0.5f;//�����ڣ������Ԫ��΢�ַ���ʱ�õõ���
        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//���ٶȹ�һ��

        if(norm != 0.0f) //�����һ�����ģ����0 ����ô˵�����ٶ����ݻ��ߴ���������������������� ��һ����Ľ������� 1.0 �������ص㡣
        {
            calculateAccConfidence(norm);//���ڴ����˶�״̬������Ҫ������ٶ����ݹ�һ����Ŀ��Ŷ�
            kpAcc = KpAcc * accConfidence; //���ٶȱ���ϵ�� * ���Ŷ�
            kiAcc = KiAcc * accConfidence;//���ٶȻ���ϵ�� * ���Ŷ�
            normR = 1.0f / norm; //���ٶȹ�һ��
            ax *= normR;
            ay *= normR;
            az *= normR;
            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3 - q0q2);//���㷽�����Ҿ���
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;
            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
            //������ɴ����������Ĳο������뷽��֮��Ĳ��,�ɴ�
            //�õ�һ�����������ͨ���������������������������ݡ�
            exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;
            gx += exAcc * kpAcc;//����������Ƽ��ٶȼƵ������ٶ�
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if(kiAcc > 0.0f) //�û���������������ǵ�ƫ����������
            {
                exAccInt += exAcc * kiAcc;
                eyAccInt += eyAcc * kiAcc;
                ezAccInt += ezAcc * kiAcc;
                gx += exAccInt;
                gy += eyAccInt;
                gz += ezAccInt;
            }
        }

        //-------------------------------------------
        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//��������ƹ�һ��

        if((magDataUpdate == TRUE) && (norm != 0.0f)) //�����ڲ���magDataUpdate == true���ҹ�һ���Ľ��norm����0���ŶԴ��������ݽ��и��¼���
        {
            normR = 1.0f / norm;//����ų���һ��
            mx *= normR;
            my *= normR;
            mz *= normR;
            // compute reference direction of flux
            //����ο�����
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
            bx = sqrt((hx * hx) + (hy * hy));
            bz = hz;
            // estimated direction of flux (w)
            //���ݲο����������̨���巽��
            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));
            exMag = my * wz - mz * wy;//����ų��͹��Ʒ�����в������,������Ʒ���������ų���ƫ��
            eyMag = mz * wx - mx * wz;
            ezMag = mx * wy - my * wx;
            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
            //ʹ�ù��Ƶľ�ֵ�������ֵ���и��£�dT����Ӧ���ڴ����Ƽ����У����ʱ�䱻Ƕ����KpMag �� KiMag����
            gx += exMag * KpMag;//����������ƴ�ǿ�������ٶ�
            gy += eyMag * KpMag;
            gz += ezMag * KpMag;

            if(KiMag > 0.0f) //�û���������������ǵ�ƫ����������
            {
                exMagInt += exMag * KiMag;
                eyMagInt += eyMag * KiMag;
                ezMagInt += ezMag * KiMag;
                gx += exMagInt;
                gy += eyMagInt;
                gz += ezMagInt;
            }
        }

        //-------------------------------------------
        // integrate quaternion rate
        //��Ԫ��΢�ַ��̣�����halfTΪ�������ڣ�gΪ�����ǽ��ٶȣ����඼����֪��������ʹ����һ����������������Ԫ��΢�ַ��̡�
        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = (q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;
        // normalise quaternion
        //��Ԫ����һ����Ϊʲô��Ҫ��һ���أ�������Ϊ�����������������Ԫ��ʧȥ�˹淶����(ģ������1��),����Ҫ���¹�һ��
        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normR;
        q1 *= normR;
        q2 *= normR;
        q3 *= normR;
        // auxiliary variables to reduce number of repeated operations
        //�Ѽ���ο������õ���ֵ�ȶ������,�����������ŷ����ʱ����ظ����㡣
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        //��������Ԫ�������������ŷ���ǵ�ת����ϵ������Ԫ��ת����ŷ����
        // sensors.margAttitude500Hz[ROLL ] = atan2f(2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3);
        //sensors.margAttitude500Hz[PITCH] = -asinf(2.0f * (q1q3 - q0q2));
        // sensors.margAttitude500Hz[YAW    ] = atan2f(2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
    }
}
uint8_t MPU6050_Init()
{
    uint8_t msg;
    msg = BIT_H_RESET;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1, &msg); // Device Reset
    delay_us(150);
    msg = MPU_CLK_SEL_PLLGYROZ;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 1, &msg); // Clock Source
    delay_us(150);
    msg = 0x00;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 1, &msg); // turn off all standby
    delay_us(150);
    msg = 0x00;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 1, &msg); // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    delay_us(150);
    msg = BITS_DLPF_CFG_98HZ;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_CONFIG, 1, &msg); // Accel and Gyro DLPF Setting
    delay_us(150);
    msg = BITS_FS_4G;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 1, &msg); // Accel +/- 4 G Full Scale
    delay_us(150);
    msg = BITS_FS_500DPS;
    I2cWriteBuffer(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 1, &msg); // Gyro +/- 500 DPS Full Scale
    delay_us(100);
    orientIMU(4);
    //Single_Write(MPU6050_ADDRESS,MPU6050_PWR_MGMT_1,BIT_H_RESET);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_PWR_MGMT_1)!=BIT_H_RESET)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_PWR_MGMT_1)!=MPU_CLK_SEL_PLLGYROZ)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_PWR_MGMT_2,0x00);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_PWR_MGMT_2)!=0x00)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_SMPLRT_DIV,0x00);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_SMPLRT_DIV)!=0x00)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_CONFIG,BITS_DLPF_CFG_98HZ);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_CONFIG)!=BITS_DLPF_CFG_98HZ)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG,BITS_FS_4G);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG)!=BITS_FS_4G)
////        {
////            return 1;
////        }
////        Single_Write(MPU6050_ADDRESS,MPU6050_GYRO_CONFIG,BITS_FS_500DPS);
//      delay_us(100);
////        if(Single_Read(MPU6050_ADDRESS,MPU6050_GYRO_CONFIG)!=BITS_FS_500DPS)
////        {
////            return 1;
////        }
    return 0;
}

mpu6050_raw MPU6050_Read()
{
    uint8_t axis;
    uint8_t I2C2_Buffer_Rx[14];
    //int16andUint8_t rawDate[3];
    int16_t straightAccelData[3];
    int16_t straightGyroData[3];
    int16_t rotatedAccelData[3];
    int16_t rotatedGyroData[3];
    mpu6050_raw raw_date;
    I2cRead(MPU6050_IIC, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, &I2C2_Buffer_Rx[0]);
    //I2C_Read_Multi(MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H,&I2C2_Buffer_Rx[0],14);
    raw_date.Raw_Accel[YAXIS].bytes[1]       = I2C2_Buffer_Rx[ 0];//����ԭʼ����
    raw_date.Raw_Accel[YAXIS].bytes[0]       = I2C2_Buffer_Rx[ 1];
    raw_date.Raw_Accel[XAXIS].bytes[1]       = I2C2_Buffer_Rx[ 2];
    raw_date.Raw_Accel[XAXIS].bytes[0]       = I2C2_Buffer_Rx[ 3];
    raw_date.Raw_Accel[ZAXIS].bytes[1]       = I2C2_Buffer_Rx[ 4];
    raw_date.Raw_Accel[ZAXIS].bytes[0]       = I2C2_Buffer_Rx[ 5];
    raw_date.Tempreature.bytes[1] = I2C2_Buffer_Rx[ 6];
    raw_date.Tempreature.bytes[0] = I2C2_Buffer_Rx[ 7];
    raw_date.Raw_Gyro[PITCH].bytes[1]        = I2C2_Buffer_Rx[ 8];
    raw_date.Raw_Gyro[PITCH].bytes[0]        = I2C2_Buffer_Rx[ 9];
    raw_date.Raw_Gyro[ROLL ].bytes[1]        = I2C2_Buffer_Rx[10];
    raw_date.Raw_Gyro[ROLL ].bytes[0]        = I2C2_Buffer_Rx[11];
    raw_date.Raw_Gyro[YAW  ].bytes[1]        = I2C2_Buffer_Rx[12];
    raw_date.Raw_Gyro[YAW  ].bytes[0]        = I2C2_Buffer_Rx[13];

    for(axis = 0; axis < 3; axis++)
    {
        straightAccelData[axis] = raw_date.Raw_Accel[axis].value;//��ȡ���ٶȾ���B��ֵ
        straightGyroData[axis]  = raw_date.Raw_Gyro[axis].value;//��ȡ�����Ǿ���B��ֵ
    }

    //����orientationMatrix * ����straightAccelData ������orientationMatrixά����3*3 ������straightAccelDataά����3*1
    //����ŵ�����rotatedAccelData ,����ά����3*1
    matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData);
    //����orientationMatrix * ����straightGyroData ������orientationMatrixά����3*3 ������straightGyroDataά����3*1
    //����ŵ�����rotatedGyroData ,����ά����3*1
    matrixMultiply(3, 3, 1, rotatedGyroData,  orientationMatrix, straightGyroData);
//  for (axis = 0; axis < 3; axis++)
//    {
//        raw_date.Raw_Accel[axis].value = rotatedAccelData[axis];//���ݾ�����˺�Ľ��
//        raw_date.Raw_Gyro[axis].value  = rotatedGyroData[axis];
//    }
    printf("ax=%d,ay=%d,az=%d\r\n", raw_date.Raw_Accel[0].value, raw_date.Raw_Accel[1].value, raw_date.Raw_Accel[2].value);
    printf("gx=%d,gy=%d,gz=%d\r\n", raw_date.Raw_Gyro[0].value, raw_date.Raw_Gyro[1].value, raw_date.Raw_Gyro[2].value);
    printf("\r\n");
    return raw_date;
}
void Imu_update(float dt)
{
}





