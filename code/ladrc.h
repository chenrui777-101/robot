#ifndef _LADRC_H
#define _LADRC_H

// TD跟踪微分器结构体
typedef struct
{
    float v1;    // 过渡过程
    float v2;    // 过渡过程的微分

    float h;     //积分步长（周期）
    float r;     //速度因子，决定跟踪的快慢
}TD;

// ESO扩张状态观测器结构体（这里仅估计一个额外的状态，例如总扰动）
typedef struct
{
    float w0;     //观测器带宽
    float wc;     //控制器带宽
    float b0;     //系统参数
    float u;      //控制器输出
    float z1,z2,z3;     ////观测器输出

    float h;     //积分步长（周期）
}ESO;

extern TD TD_LR_Angle;//左右跟踪微分器
extern TD TD_FB_Angle;//前后跟踪微分器

extern ESO ESO_LR_Angle;//左右扩张状态观测器
extern ESO ESO_FB_Angle;//前后扩张状态观测器


void TD_Update(TD *td, float v0);
void ESO_Update(ESO *eso, float measured_value);
int sign(float x);

#endif
