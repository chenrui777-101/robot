#include "zf_common_headfile.h"
#include "deal_imu.h"

/************************************************************************************************************************/
//         author: SWUST-CR
//         date:2024-5-3
/************************************************************************************************************************/

Posture_Offset Init_GyroOffset;

move_filter_struct gyro_x_filter;
move_filter_struct gyro_y_filter;
move_filter_struct gyro_z_filter;
move_filter_struct acc_off_ground_filter;

float acc_off_ground;//离地检测加速度

void Gyro_Off_SetInit(void)//采集陀螺仪零飘值
{
    Init_GyroOffset.Xdata = 0;
    Init_GyroOffset.Ydata = 0;
    Init_GyroOffset.Zdata = 0;

    for (uint16 i = 0; i < 1000; i++)
    {
        imu963ra_get_gyro();
        Init_GyroOffset.Xdata += imu963ra_gyro_x;
        Init_GyroOffset.Ydata += imu963ra_gyro_y;
        Init_GyroOffset.Zdata += imu963ra_gyro_z;
    }
    Init_GyroOffset.Xdata /=1000.0;
    Init_GyroOffset.Ydata /=1000.0;
    Init_GyroOffset.Zdata /=1000.0;

    imu963ra_get_acc();
    ax=imu963ra_acc_transition(imu963ra_acc_x);
    ay=imu963ra_acc_transition(imu963ra_acc_y);
    az=imu963ra_acc_transition(imu963ra_acc_z);

    /***********初始化采集角加速度第一次计算angle并赋值*********/
    Angle_X_Final=atan2f(az,ay)*180.0/3.141593f;
    Angle_Y_Final=atan2f(ax,ay)*180.0/3.141593f;
    Angle_Z_Final=0;
}

void Move_Avg_Filter_imu963ra_gyro(void)//滑动平均滤波处理角速度
{
    imu963ra_get_gyro();

    imu963ra_gyro_x -= Init_GyroOffset.Xdata;
    imu963ra_gyro_y -= Init_GyroOffset.Ydata;
    imu963ra_gyro_z -= Init_GyroOffset.Zdata;

    move_filter_calc(&gyro_x_filter, imu963ra_gyro_transition(imu963ra_gyro_x));//滑动平均滤波使数据更平滑
    move_filter_calc(&gyro_y_filter, imu963ra_gyro_transition(imu963ra_gyro_y));
    move_filter_calc(&gyro_z_filter, imu963ra_gyro_transition(imu963ra_gyro_z));

    Gyro_X = gyro_x_filter.data_average;
    Gyro_Y = gyro_y_filter.data_average;
    Gyro_Z = gyro_z_filter.data_average;
    if(Gyro_Y<1&&Gyro_Y>-1)
        Gyro_Y = 0;
}

void Med_Avg_Filter_imu963ra_acc(void)//中位值滤波获取加速度信息
{
    float temp;
    int16 value_buf1[N],value_buf2[N],value_buf3[N];

    for(int8 count = 0; count < N; count++ )    //采样N次
    {
        imu963ra_get_acc();

        value_buf1[count] =imu963ra_acc_x;
        value_buf2[count] =imu963ra_acc_y;
        value_buf3[count] =imu963ra_acc_z;
    }
/************************冒泡排序升序*************************/
    for(uint8 j = 0; j < N - 1; j++ )
    {
        for(uint8 i = 0; i < N - j - 1; i++ )
        {
            if( value_buf1[i] > value_buf1[i + 1] )
            {
                temp = value_buf1[i];
                value_buf1[i] = value_buf1[i + 1];
                value_buf1[i + 1] = temp;
            }
            if( value_buf2[i] > value_buf2[i + 1] )
            {
                temp = value_buf2[i];
                value_buf2[i] = value_buf2[i + 1];
                value_buf2[i + 1] = temp;
            }
            if( value_buf3[i] > value_buf3[i + 1] )
            {
                temp = value_buf3[i];
                value_buf3[i] = value_buf3[i + 1];
                value_buf3[i + 1] = temp;
            }
        }
    }

    imu963ra_acc_x=value_buf1[((N-1)/2)];
    imu963ra_acc_y=value_buf2[((N-1)/2)];
    imu963ra_acc_z=value_buf3[((N-1)/2)];

    ax = imu963ra_acc_transition(imu963ra_acc_x);
    ay = imu963ra_acc_transition(imu963ra_acc_y);
    az = imu963ra_acc_transition(imu963ra_acc_z);
}

uint8 Off_ground_detection()//离地检测
{
    acc_off_ground = ay/cosf(Angle_Y_Final*AtR);
    move_filter_calc(&acc_off_ground_filter,acc_off_ground);
    acc_off_ground = acc_off_ground_filter.data_average;

    if(acc_off_ground - 0.985<-0.5)//阈值判断
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**********************************************************MOVE_FILTER**********************************************/
float weighting_sum = 0;//权值和
void move_filter_calc(move_filter_struct *move_filter, float new_data)//滑动平均滤波
{
    int i = 0;

    move_filter->data_sum = 0;//加权和
    for(i = 0;i < MOVE_AVERAGE_SIZE - 1;i++)//更新数据顺序
    {
        move_filter->data_buffer[i] = move_filter->data_buffer[i+1];
        move_filter->data_sum += move_filter->data_buffer[i] * (i + 1);
    }
    move_filter->data_buffer[MOVE_AVERAGE_SIZE - 1] = new_data;//新数据输入
    move_filter->data_sum += move_filter->data_buffer[MOVE_AVERAGE_SIZE - 1] * MOVE_AVERAGE_SIZE;
    //加权计算平均值
    move_filter->data_average = move_filter->data_sum / weighting_sum;

}

void move_filter_init(move_filter_struct *move_filter)//滑动平均滤波初始化
{
    move_filter->data_sum = 0;
    move_filter->data_average   = 0;
    //设置缓冲区大小
    move_filter->buffer_size    = MOVE_AVERAGE_SIZE;

    uint8 i;
    for(i = 0; i < move_filter->buffer_size; i ++)
    {
        move_filter->data_buffer[i] = 0;
    }
}

void weighting_sum_init(void)//加权权重初始化
{
    uint8 i;
    for(i = 1;i <= MOVE_AVERAGE_SIZE;i++)
    {
        weighting_sum += i;
    }
}

/*********************************************************************************************************/

// #define MOVE_AVERAGE_SIZE   9      // 窗口大小
// #define GAUSSIAN_SIGMA      2.0f    // 高斯分布标准差

// // 中心模式枚举
// typedef enum {
//     CENTER_FIXED,       // 固定几何中心 (索引 4)
//     CENTER_LATEST,      // 最新样本中心 (索引 9)
//     CENTER_MEAN         // 窗口均值映射中心
// } center_mode_t;

// // 滤波器结构体
// typedef struct {
//     uint8_t buffer_size;
//     float   data_buffer[MOVE_AVERAGE_SIZE];
//     float   data_average;
//     float   sigma;              // 当前标准差
//     center_mode_t mode;         // 当前中心模式
// } move_filter_struct;

// /* 初始化滤波器 */
// void move_filter_init(move_filter_struct *mf, float sigma, center_mode_t mode) {
//     mf->buffer_size = MOVE_AVERAGE_SIZE;
//     mf->sigma = sigma;
//     mf->mode = mode;
//     mf->data_average = 0.0f;
//     for (int i = 0; i < MOVE_AVERAGE_SIZE; i++) {
//         mf->data_buffer[i] = 0.0f;
//     }
// }

// /* 计算窗口内数据的均值 */
// static float calc_mean(float *arr, int n) {
//     float sum = 0.0f;
//     for (int i = 0; i < n; i++) sum += arr[i];
//     return sum / (float)n;
// }

// /* 滑动高斯滤波（动态中心） */
// void move_filter_calc(move_filter_struct *mf, float new_data) {
//     // 1. 更新数据缓冲区（FIFO）
//     for (int i = 0; i < MOVE_AVERAGE_SIZE - 1; i++) {
//         mf->data_buffer[i] = mf->data_buffer[i + 1];
//     }
//     mf->data_buffer[MOVE_AVERAGE_SIZE - 1] = new_data;

//     // 2. 确定高斯中心索引
//     float center;
//     switch (mf->mode) {
//         case CENTER_FIXED:
//             center = (MOVE_AVERAGE_SIZE - 1) / 2.0f;   // 固定中心 4.5
//             break;
//         case CENTER_LATEST:
//             center = MOVE_AVERAGE_SIZE - 1;            // 最新样本索引 9
//             break;
//         case CENTER_MEAN:
//         default: {
//             // 计算窗口均值，并线性映射到 [0, MOVE_AVERAGE_SIZE-1] 作为中心索引
//             float min_val = mf->data_buffer[0];
//             float max_val = mf->data_buffer[0];
//             for (int i = 1; i < MOVE_AVERAGE_SIZE; i++) {
//                 if (mf->data_buffer[i] < min_val) min_val = mf->data_buffer[i];
//                 if (mf->data_buffer[i] > max_val) max_val = mf->data_buffer[i];
//             }
//             float mean_val = calc_mean(mf->data_buffer, MOVE_AVERAGE_SIZE);
//             if (max_val - min_val > 1e-6f) {
//                 // 映射：最小值 -> 0，最大值 -> MOVE_AVERAGE_SIZE-1
//                 center = (mean_val - min_val) / (max_val - min_val) * (MOVE_AVERAGE_SIZE - 1);
//             } else {
//                 center = (MOVE_AVERAGE_SIZE - 1) / 2.0f; // 全等时退化为固定中心
//             }
//             break;
//         }
//     }

//     // 3. 生成高斯权重并计算加权平均值
//     float sigma = mf->sigma;
//     float weights[MOVE_AVERAGE_SIZE];
//     float weight_sum = 0.0f;
//     for (int i = 0; i < MOVE_AVERAGE_SIZE; i++) {
//         float diff = (float)i - center;
//         weights[i] = expf(- (diff * diff) / (2.0f * sigma * sigma));
//         weight_sum += weights[i];
//     }

//     float weighted_sum = 0.0f;
//     if (weight_sum > 0.0f) {
//         for (int i = 0; i < MOVE_AVERAGE_SIZE; i++) {
//             weighted_sum += mf->data_buffer[i] * (weights[i] / weight_sum);
//         }
//     }
//     mf->data_average = weighted_sum;
// }


//-------------------------------------------------------------------------------------------------------------------
//******************************************* 四元数法姿态解算*************************************************
//-------------------------------------------------------------------------------------------------------------------
float q0=1,q1,q2,q3;
float g1,g2,g3,g4,g5;
float dt_quaternions      = 0.002;//dt为陀螺仪采样时间

float Vx,Vy,Vz;//姿态矩阵中的重力分量
float ex,ey,ez;//三个轴的误差元素
float accex,accey,accez;//误差积分
float Kp=0.8,Ki=0.01;//理想状况下会有一个值趋于0
//Kp比例增益 决定了加速度计和磁力计的收敛速度
//Ki积分增益 决定了陀螺仪偏差的收敛速度

float Desired_Angle_X_Rad, Desired_Angle_Y_Rad, Desired_Angle_Z_Rad;//三轴归零所需旋转弧度

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) // 互补法求解四元数
{
    gx = gx * AtR;                    // 转化成弧度，1°=0.0174弧度
    gy = gy * AtR;
    gz = gz * AtR;

    // 加速度归一化
    float acc_g = invSqrt(ax*ax + ay*ay + az*az);
    ax *= acc_g;
    ay *= acc_g;
    az *= acc_g;

    // 根据当前四元数计算重力方向分量
    Vx = 2 * (q1*q3 - q0*q2);
    Vy = 2 * (q0*q1 + q2*q3);
    Vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 向量外积得到姿态误差
    ex = (ay*Vz - az*Vy);
    ey = (az*Vx - ax*Vz);
    ez = (ax*Vy - ay*Vx);

    // 误差积分
    accex += ex * Ki * dt_quaternions;
    accey += ey * Ki * dt_quaternions;
    accez += ez * Ki * dt_quaternions;

    // 将误差PI补偿到陀螺仪，消除零点漂移
    gx += Kp*ex + accex;
    gy += Kp*ey + accey;
    gz += Kp*ez + accez;              // 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // 计算半个时间步长的角增量（用于四元数更新）
    float half_dt_gx = 0.5f * dt_quaternions * gx;
    float half_dt_gy = 0.5f * dt_quaternions * gy;
    float half_dt_gz = 0.5f * dt_quaternions * gz;

    // 一阶龙格库塔法更新四元数（已修正符号错误）
    q0 += -q1 * half_dt_gx - q2 * half_dt_gy - q3 * half_dt_gz;
    q1 +=  q0 * half_dt_gx + q2 * half_dt_gz - q3 * half_dt_gy;
    q2 +=  q0 * half_dt_gy - q1 * half_dt_gz + q3 * half_dt_gx;
    q3 +=  q0 * half_dt_gz + q1 * half_dt_gy - q2 * half_dt_gx;

    // 四元数归一化
    float q_g = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= q_g;
    q1 *= q_g;
    q2 *= q_g;
    q3 *= q_g;

    // 计算旋转矩阵元素（用于提取欧拉角）
    g1 = 2.0f * (q1*q3 - q0*q2);
    g2 = 2.0f * (q0*q1 + q2*q3);
    g3 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    g4 = 2.0f * (q1*q2 + q0*q3);
    g5 = q0*q0 + q1*q1 - q2*q2 - q3*q3;

    // 输出欧拉角（角度制）
    Angle_Y_Final = asinf(g1) * RtA;
    Angle_X_Final = atan2f(g2, g3) * RtA;
    Angle_Z_Final = atan2f(g4, g5) * RtA;

    // ========== 三轴归零所需旋转矢量（机体系，无死锁） ==========
    float qe1 = -q1, qe2 = -q2, qe3 = -q3;          // 去掉负号可使输出符号与当前角度一致
    float norm_xyz = sqrtf(qe1*qe1 + qe2*qe2 + qe3*qe3);
    float angle_rad = 2.0f * atan2f(norm_xyz, q0);

//    // 可选：限制最大输出角度为 π，避免 over-range
//    if (angle_rad > 3.14159f) angle_rad = 3.14159f;

    if (norm_xyz > 1e-9f)
    {
        float factor = angle_rad / norm_xyz;
        Desired_Angle_X_Rad = qe1 * factor;
        Desired_Angle_Y_Rad = qe2 * factor;
        Desired_Angle_Z_Rad = qe3 * factor;
    }
    else
    {
        Desired_Angle_X_Rad = 0.0f;
        Desired_Angle_Y_Rad = 0.0f;
        Desired_Angle_Z_Rad = 0.0f;
    }
    
}


float  beta = 8000.0;// 2 * 比例增益 (Kp)
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)// 梯度下降法求解四元数
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 陀螺仪四元数的变化率
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 仅当加速度计测量有效时才计算反馈 (避免加速度计归一化中的NaN)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // 归一化加速度计测量
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 辅助变量避免重复运算
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 梯度体面算法校正步骤
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一阶跃幅度
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用反馈步骤
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 将四元数的变化率积分为生成四元数
    q0 += qDot1 * dt_quaternions;
    q1 += qDot2 * dt_quaternions;
    q2 += qDot3 * dt_quaternions;
    q3 += qDot4 * dt_quaternions;

    // 正常化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    float q_g=invSqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=q_g;
    q1*=q_g;
    q2*=q_g;
    q3*=q_g;//四元数归一化

    g1=2.0f*(q1*q3-q0*q2);
    g2=2.0f*(q0*q1+q2*q3);
    g3=q0*q0-q1*q1-q2*q2+q3*q3;
    g4=2.0f*(q1*q2+q0*q3);
    g5=q0*q0+q1*q1-q2*q2-q3*q3;

    Angle_Y_Final=asinf(g1) * RtA;
    Angle_X_Final=atan2f(g2,g3) * RtA;
    Angle_Z_Final=atan2f(g4,g5) * RtA;

    if(Angle_Z_Final<0)
    {
        Angle_Z_Final += 360;
    }

}

float invSqrt(float x)
{
    float halfx=0.5f*x;
    float y=x;
    long i=*(long*)&y;
    i=0x5f3759df-(i>>1);
    y=*(float*)&i;
    y=y*(1.5f-(halfx*y*y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
//******************************************* 卡尔曼滤波解算角度值*************************************************
//-------------------------------------------------------------------------------------------------------------------
float Angle_x_temp,Angle_y_temp,Angle_z_temp=0.0,ax,ay,az,Gyro_X,Gyro_Y,Gyro_Z,Gyro_Z_Actual;
float Angle_X_Final,Angle_Y_Final,Angle_Z_Final,Gyro_X_Final,Gyro_Y_Final,Gyro_Z_Final;
float dt      = 0.002;//dt为kalman滤波器采样时间

void kalman_get_imu963ra()
{
    //用加速度计算三个轴和水平面坐标系之间的夹角
    Angle_x_temp=atan2f(az,ay)*180.0/3.141593f;
    Angle_y_temp=atan2f(ax,ay)*180.0/3.141593f;

    Kalman_Filter_X(Angle_x_temp,-Gyro_X);  //卡尔曼滤波计算X倾角
    Kalman_Filter_Y(Angle_y_temp,Gyro_Z);  //卡尔曼滤波计算Y倾角

}

//卡尔曼参数
/*-------------------------------------------------------------------------------------------------------------*/
/*
Q值为过程噪声，越小系统越容易收敛，我们对模型预测的值信任度越高；
但是太小则容易发散，如果Q为零，那么我们只相信预测值；
Q值越大我们对于预测的信任度就越低，而对测量值的信任度就变高；
如果Q值无穷大，那么我们只信任测量值；

R值为测量噪声，太小太大都不一定合适。
R太大，卡尔曼滤波响应会变慢，因为它对新测量的值的信任度降低；
越小系统收敛越快，但过小则容易出现震荡；
测试时可以保持陀螺仪不动，记录一段时间内陀螺仪的输出数据，
这个数据近似正态分布，按3σ原则，取正态分布的(3σ)^2作为R的初始化值。

测试时可以先将Q从小往大调整，将R从大往小调整；先固定一个值去调整另外一个值，看收敛速度与波形输出。

系统中还有一个关键值P，它是误差协方差初始值，表示我们对当前预测状态的信任度，
它越小说明我们越相信当前预测状态；
它的值决定了初始收敛速度，一般开始设一个较小的值以便于获取较快的收敛速度。
随着卡尔曼滤波的迭代，P的值会不断的改变，当系统进入稳态之后P值会收敛成一个最小的估计方差矩阵，
这个时候的卡尔曼增益也是最优的，所以这个值只是影响初始收敛速度。
Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏

R:测量噪声，R增大，动态响应变慢，收敛稳定性变好

*/
float Q_angle = 0.001;
float Q_gyro  = 0.003;
float R_angle = 0.5;
char  C_0     = 1;
float Q_bias_X, Angle_err_X,Q_bias_Y, Angle_err_Y,Q_bias_Z, Angle_err_Z;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP_X[2][2] = { { 1, 0 },{ 0, 1 } };
float PP_Y[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
{
    Angle_X_Final += (Gyro - Q_bias_X) * dt; //先验估计
    Pdot[0]=Q_angle - PP_X[0][1] - PP_X[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]= -PP_X[1][1];
    Pdot[2]= -PP_X[1][1];
    Pdot[3]= Q_gyro;

    PP_X[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP_X[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP_X[1][0] += Pdot[2] * dt;
    PP_X[1][1] += Pdot[3] * dt;

    Angle_err_X = Accel - Angle_X_Final;  //zk-先验估计

    PCt_0 = C_0 * PP_X[0][0];
    PCt_1 = C_0 * PP_X[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP_X[0][1];

    PP_X[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP_X[0][1] -= K_0 * t_1;
    PP_X[1][0] -= K_1 * t_0;
    PP_X[1][1] -= K_1 * t_1;

    Angle_X_Final += K_0 * Angle_err_X;    //后验估计
    Q_bias_X      += K_1 * Angle_err_X;    //后验估计
//    Gyro_X_Final  = Gyro - Q_bias_X;  //输出值（后验估计）的微分 = 角速度
}

void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数
{
    Angle_Y_Final += (Gyro - Q_bias_Y) * dt; //先验估计
    Pdot[0]=Q_angle - PP_Y[0][1] - PP_Y[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]= -PP_Y[1][1];
    Pdot[2]= -PP_Y[1][1];
    Pdot[3]= Q_gyro;

    PP_Y[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP_Y[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP_Y[1][0] += Pdot[2] * dt;
    PP_Y[1][1] += Pdot[3] * dt;

    Angle_err_Y = Accel - Angle_Y_Final;  //zk-先验估计

    PCt_0 = C_0 * PP_Y[0][0];
    PCt_1 = C_0 * PP_Y[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP_Y[0][1];

    PP_Y[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP_Y[0][1] -= K_0 * t_1;
    PP_Y[1][0] -= K_1 * t_0;
    PP_Y[1][1] -= K_1 * t_1;

    Angle_Y_Final += K_0 * Angle_err_Y;    //后验估计
    Q_bias_Y      += K_1 * Angle_err_Y;    //后验估计
//    Gyro_Z_Final  = Gyro - Q_bias_Y;  //输出值（后验估计）的微分 = 角速度
}

//-------------------------------------------------------------------------------------------------------------------
//******************************************* 拓展卡尔曼滤波解算角度值***********************************
//-------------------------------------------------------------------------------------------------------------------

