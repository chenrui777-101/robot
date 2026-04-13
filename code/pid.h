#ifndef _PID_H
#define _PID_H
#include "zf_common_typedef.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*************************************************************************************************************/

typedef struct Pid_Positional
{
    float P;                      //P参数
    float I;                      //I参数
    float D;                      //D参数
    float C;                      //串级期望传递增益
    float Err;                    //偏差值
    float Err_Next;               //定义上一个偏差值
    float Set;                    //定义设定值
    float Set_Next;               //定义上一个设定值
    float Actual;                 //实际值
    float Actual_Next;            //上一个实际值
    float Out;                    //PID输出
    float increment;              //定义增量
    float proportion;             //PID比例项
    float integration;            //PID积分项
    float differential;           //PID微分项
    float Set_differential;       //设定值的微分
    float integration_K;          //变速积分速率
    float integration_A;          //变速积分超调值
    float integration_B;          //变速积分下限
    float Limit_increment;        //积分限幅
}Pid_Pos;

typedef struct Pid_Increment
{
    float P;                        //P参数
    float I;                        //I参数
    float D;                        //D参数
    float Err;                      //偏差值
    float Err_Last;                 //定义上上个偏差值
    float Err_Next;                 //定义上一个偏差值
    float Set;                      //定义设定值
    float Actual;                   //实际值
    float Out;                      //电机输出
    float increment;                //定义增量
    float proportion;               //PID比例项
    float integration;              //PID积分项
    float differential;             //PID微分项
}Pid_Inc;

typedef struct Feedforward
{
    float Set_value;            //定义目标值
    float Set_value_Next;       //定义上一个目标值
    float Compensate_Out;       //定义前馈补偿量
    float Compensate_Out_differential;  //定义前馈微分补偿量
    float Out;                  //前馈输出值
    float K1_Feedforward;       //第一次前馈增益系数
    float K2_Feedforward;       //第二次前馈增益系数
}Feedforward;

/*******动量轮平衡环结构体*******/
extern Pid_Pos LR_Balance_Gyro;
extern Pid_Pos LR_Balance_Angle;
extern Pid_Pos LR_Balance_Speed;
/*******行进轮平衡环结构体*******/
extern Pid_Pos FB_Balance_Gyro;
extern Pid_Pos FB_Balance_Angle;
extern Pid_Pos FB_Balance_Speed;
/*******动量轮转向环结构体*******/
extern Pid_Pos Servo_Angle;
extern Pid_Pos Servo_Speed;
extern Pid_Pos Servo_Gyro;
/*******动态零点平衡环结构体*******/
extern Pid_Pos Dynamic_LR_Angle;
extern Pid_Pos Dynamic_FB_Speed;

/*******平衡前馈补偿结构体***/
extern Feedforward LR_Gyro_Compensate;
extern Feedforward LR_Angle_Compensate;
extern Feedforward FB_Gyro_Compensate;
extern Feedforward FB_Angle_Compensate;

extern float LR_EFF[7];
extern float LR_DFF[7];
extern float FB_EFF[7];
extern float FB_DFF[7];

extern float LR_UFF_kp[7];
extern float LR_UFF_kd[7];
extern float FB_UFF_kp[7];
extern float FB_UFF_kd[7];

void Pid_Position(Pid_Pos *pid);
void DF_Pid_Position(Pid_Pos *pid);
void Pid_Increse(Pid_Inc *pid);
void Feedforward_compensate(Feedforward *compensate);
float limit_value(float value,float positive_value,float negative_value);
float Fuzzy_P(float E,float EC,float *EFF,float *DFF,float *UFF);
float Fuzzy_D(float E,float EC,float *EFF,float *DFF,float *UFF);

#endif
