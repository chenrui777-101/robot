#ifndef _DEAL_IMU_H
#define _DEAL_IMU_H

#include "zf_common_typedef.h"

#define N 5         //加速度计采样次数必须为奇数，与滤波处理函数有关

#define RtA         57.295779  //弧度->角度
#define AtR         0.0174533  //角度->弧度

typedef struct Posture_Offset
{
    double Xdata;
    double Ydata;
    double Zdata;
}Posture_Offset;
extern Posture_Offset Init_GyroOffset;

#define MOVE_AVERAGE_SIZE   10 // 定义缓冲区最大限度  该值越大平滑效果越好，滞后效果增强

typedef struct
{
    uint8 buffer_size;                      // buffer大小
    float data_buffer[MOVE_AVERAGE_SIZE];   // 缓冲区
    float data_average;                     // 数据平均值
    float data_sum;                         // 数据加权和
}move_filter_struct;
extern float weighting_sum;

extern move_filter_struct gyro_x_filter;
extern move_filter_struct gyro_y_filter;
extern move_filter_struct gyro_z_filter;
extern move_filter_struct acc_off_ground_filter;

extern float Angle_X_Final,Angle_Y_Final,Angle_Z_Final,Gyro_X_Final,Gyro_Y_Final,Gyro_Z_Final;
extern float ax,ay,az,Gyro_X,Gyro_Y,Gyro_Z;
extern float Angle_x_temp,Angle_y_temp;
extern float acc_off_ground;

void Move_Avg_Filter_imu963ra_gyro(void);
void Med_Avg_Filter_imu963ra_acc(void);
void Gyro_Off_SetInit(void);
uint8 Off_ground_detection(void);
void move_filter_init(move_filter_struct *move_filter);
void move_filter_calc(move_filter_struct *move_filter, float new_data);
void weighting_sum_init(void);
void kalman_get_imu963ra(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);
#endif
