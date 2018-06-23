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


uint8_t MargAHRSinitialized = FALSE;//航姿初始化

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

void orientIMU(uint8_t imuOrientation) //根据IMU单元的方位确定矩阵A的值
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
    //通过使用atan2f函数计算加速度数据得到欧拉角 滚转角和 俯仰角。
    accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
    accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);
    //其中 smoothAcc 是通过加速度数据经过atan2f函数计算得来的欧拉角，并且进行了一阶滞后滤波
    //（此滤波算法也属于低通滤波的一种），优点： 对周期性干扰具有良好的抑制作用 适用于波动频率较高的场合,
    // 缺点： 相位滞后，灵敏度低 滞后程度取决于a值大小， 不能消除滤波频率高于采样频率的1/2的干扰信号,代码中a的值是99.0f
    smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
    smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;
    gyroRate[PITCH] =  gyroData[PITCH];
    //通过互补滤波来融合根据加速度和陀螺仪计算出来的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根据陀螺仪
    //数据计算得到的角度（角速度*持续时间结果就是弧度，弧度和角度很容易的可以相互转换），为什么要进行数据融合？
    //答：加速度计和陀螺仪都能计算出姿态，但为何要对它们融合，是因为加速度计对振动之类的扰动很敏感，但长期数据计算出的姿态可信，
    //而陀螺仪虽然对振动这些不敏感，但长期使用陀螺仪会出现漂移，因此我们要进行互补，短期相信陀螺仪，长期相信加速度计.
    //先通过加速度计得到的角度减去上一次融合后的角度然后乘以一个比例系数，这个比例系数越小，融合的加速度计的数据比重越小，
    //短期相信陀螺仪，所以陀螺仪的比重这里是1，长期相信加速度计，加速度计的数据用来修正陀螺仪的漂移产生的误差，
    //这样对陀螺仪的漂移进行了修正，有效地抑制了加速度计和陀螺仪各自单独工作时候的偏差.
    orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);
    //可以用正弦或余弦和x轴单独算出角度,因为我们知道重力的大小
    //但是IMU单元必须是静止水平状态
    gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
    //通过互补滤波来融合根据加速度和陀螺仪计算出来的角度，orient[PITCH]是上次融合后的角度，gyroRate[PITCH] * dt是根据陀螺仪
    //数据计算得到的角度（角速度*持续时间结果就是弧度，弧度和角度很容易的可以相互转换），为什么要进行数据融合？
    //答：加速度计和陀螺仪都能计算出姿态，但为何要对它们融合，是因为加速度计对振动之类的扰动很敏感，但长期数据计算出的姿态可信，
    //而陀螺仪虽然对振动这些不敏感，但长期使用陀螺仪会出现漂移，因此我们要进行互补，短期相信陀螺仪，长期相信加速度计.
    //先通过加速度计得到的角度减去上一次融合后的角度然后乘以一个比例系数，这个比例系数越小，融合的加速度计的数据比重越小，
    //短期相信陀螺仪，所以陀螺仪的比重这里是1，长期相信加速度计，加速度计的数据用来修正陀螺仪的漂移产生的误差，
    //这样对陀螺仪的漂移进行了修正，有效地抑制了加速度计和陀螺仪各自单独工作时候的偏差.
    orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);
    //可以用正弦或余弦和x轴单独算出角度,因为我们知道重力的大小
    //但是IMU单元必须是静止水平状态
    gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
    orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);//对陀螺仪进行积分得到偏航角YAW
}
//====================================================================================================
// Initialization
//====================================================================================================
//航姿参考系统初始化
void MargAHRSinit(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;
    //使用加速度数据计算欧拉角 ，滚转角和俯仰角
    initialRoll  = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);
    //对欧拉角进行余弦和正弦计算，分别把计算结果保存下来
    cosRoll  = cosf(initialRoll);
    sinRoll  = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);
    magX = 1.0f;  // HJI mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    magY = 0.0f;  // HJI my * cosRoll - mz * sinRoll;
    initialHdg = atan2f(-magY, magX);//解算航向角
    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);
    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);
    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);
    //得到四元数
    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    //把计算参考方向用到的值先都计算好,减少重复计算,因为MargAHRSupdate函数里面要用到。
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
//航姿参考系统更新
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
        //如果航姿参考系统参数还没有初始化过，那么执行AHRS初始化
        MargAHRSinit(ax, ay, az, mx, my, mz);
        MargAHRSinitialized = TRUE;//标记航姿参考系统参数已经初始化过
    }

    //-------------------------------------------

    if(MargAHRSinitialized == TRUE) //如果航姿参考系统参数已经初始化过
    {
        halfT = dt * 0.5f;//半周期，求解四元数微分方程时用得到。
        norm = sqrt(SQR(ax) + SQR(ay) + SQR(az));//加速度归一化

        if(norm != 0.0f) //如果归一化后的模等于0 ，那么说明加速度数据或者传感器不正常，正常情况下 归一化后的结果恒等于 1.0 ，这是重点。
        {
            calculateAccConfidence(norm);//由于处于运动状态，所有要计算加速度数据归一化后的可信度
            kpAcc = KpAcc * accConfidence; //加速度比例系数 * 可信度
            kiAcc = KiAcc * accConfidence;//加速度积分系数 * 可信度
            normR = 1.0f / norm; //加速度归一化
            ax *= normR;
            ay *= normR;
            az *= normR;
            // estimated direction of gravity (v)
            vx = 2.0f * (q1q3 - q0q2);//计算方向余弦矩阵
            vy = 2.0f * (q0q1 + q2q3);
            vz = q0q0 - q1q1 - q2q2 + q3q3;
            // error is sum of cross product between reference direction
            // of fields and direction measured by sensors
            //误差是由传感器测量的参考方向与方向之间的叉积,由此
            //得到一个误差向量，通过这个误差向量来修正陀螺仪数据。
            exAcc = vy * az - vz * ay;
            eyAcc = vz * ax - vx * az;
            ezAcc = vx * ay - vy * ax;
            gx += exAcc * kpAcc;//比例增益控制加速度计的收敛速度
            gy += eyAcc * kpAcc;
            gz += ezAcc * kpAcc;

            if(kiAcc > 0.0f) //用积分增益控制陀螺仪的偏差收敛速率
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
        norm = sqrt(SQR(mx) + SQR(my) + SQR(mz));//三轴磁力计归一化

        if((magDataUpdate == TRUE) && (norm != 0.0f)) //如果入口参数magDataUpdate == true并且归一化的结果norm不是0，才对磁力计数据进行更新计算
        {
            normR = 1.0f / norm;//三轴磁场归一化
            mx *= normR;
            my *= normR;
            mz *= normR;
            // compute reference direction of flux
            //计算参考方向
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
            bx = sqrt((hx * hx) + (hy * hy));
            bz = hz;
            // estimated direction of flux (w)
            //根据参考方向估计云台机体方向
            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));
            exMag = my * wz - mz * wy;//三轴磁场和估计方向进行叉积运算,计算估计方向与三轴磁场的偏差
            eyMag = mz * wx - mx * wz;
            ezMag = mx * wy - my * wx;
            // use un-extrapolated old values between magnetometer updates
            // dubious as dT does not apply to the magnetometer calculation so
            // time scaling is embedded in KpMag and KiMag
            //使用估计的旧值与磁力计值进行更新，dT不能应用在磁力计计算中，因此时间被嵌入在KpMag 和 KiMag里面
            gx += exMag * KpMag;//比例增益控制磁强计收敛速度
            gy += eyMag * KpMag;
            gz += ezMag * KpMag;

            if(KiMag > 0.0f) //用积分增益控制陀螺仪的偏差收敛速率
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
        //四元数微分方程，其中halfT为测量周期，g为陀螺仪角速度，其余都是已知量，这里使用了一阶龙格库塔法求解四元数微分方程。
        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
        q1i = (q0 * gx + q2 * gz - q3 * gy) * halfT;
        q2i = (q0 * gy - q1 * gz + q3 * gx) * halfT;
        q3i = (q0 * gz + q1 * gy - q2 * gx) * halfT;
        q0 += q0i;
        q1 += q1i;
        q2 += q2i;
        q3 += q3i;
        // normalise quaternion
        //四元数归一化，为什么又要归一化呢？这是因为引入了误差向量后四元数失去了规范性了(模不等于1了),所以要重新归一化
        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normR;
        q1 *= normR;
        q2 *= normR;
        q3 *= normR;
        // auxiliary variables to reduce number of repeated operations
        //把计算参考方向用到的值先都计算好,减少下面计算欧拉角时候的重复计算。
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
        //最后根据四元数方向余弦阵和欧拉角的转换关系，把四元数转换成欧拉角
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
    raw_date.Raw_Accel[YAXIS].bytes[1]       = I2C2_Buffer_Rx[ 0];//传输原始数据
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
        straightAccelData[axis] = raw_date.Raw_Accel[axis].value;//获取加速度矩阵B的值
        straightGyroData[axis]  = raw_date.Raw_Gyro[axis].value;//获取陀螺仪矩阵B的值
    }

    //矩阵orientationMatrix * 矩阵straightAccelData ，矩阵orientationMatrix维度是3*3 ，矩阵straightAccelData维度是3*1
    //结果放到矩阵rotatedAccelData ,矩阵维度是3*1
    matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData);
    //矩阵orientationMatrix * 矩阵straightGyroData ，矩阵orientationMatrix维度是3*3 ，矩阵straightGyroData维度是3*1
    //结果放到矩阵rotatedGyroData ,矩阵维度是3*1
    matrixMultiply(3, 3, 1, rotatedGyroData,  orientationMatrix, straightGyroData);
//  for (axis = 0; axis < 3; axis++)
//    {
//        raw_date.Raw_Accel[axis].value = rotatedAccelData[axis];//传递矩阵相乘后的结果
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





