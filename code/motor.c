#include "zf_common_headfile.h"

void Brushless_Out_Pwm(Direction_enum Left_or_Right,Direction_enum CW_or_CCW ,uint32 out)
{
    if(Left_or_Right==0)//左飞轮
    {
        if(CW_or_CCW==0)//0为CCW逆时针
        {
            Left_CCW;
            Left_Start;
            pwm_set_duty(ATOM1_CH5_P02_5 ,PWM_DUTY_MAX - out);
        }
        if(CW_or_CCW==1)//0为CW顺时针
        {
            Left_CW;
            Left_Start;
            pwm_set_duty(ATOM1_CH5_P02_5 ,PWM_DUTY_MAX - out);
        }
    }
    if(Left_or_Right==1)//右飞轮
    {
        if(CW_or_CCW==0)//0为CCW逆时针
        {
            Right_CCW;
            Right_Start;
            pwm_set_duty(ATOM1_CH7_P02_7 ,PWM_DUTY_MAX - out);
        }
        if(CW_or_CCW==1)//1为CW顺时针
        {
            Right_CW;
            Right_Start;
            pwm_set_duty(ATOM1_CH7_P02_7 ,PWM_DUTY_MAX - out);
        }
    }
}

void DC_Out_Pwm(Direction_enum Go_or_Back,uint32 out)
{
    if(Go_or_Back==1)
    {
        gpio_set_level(P21_3,1);//方向引脚，0反转，1正转
        pwm_set_duty (ATOM0_CH0_P21_2,out);
    }
    if(Go_or_Back==0)
    {
        gpio_set_level(P21_3,0);//方向引脚，0反转，1正转
        pwm_set_duty (ATOM0_CH0_P21_2,out);
    }
}

float Increase_Velocity(float Increase_rate,float Velocity,float Velocity_limit)
{
    if(Velocity<Velocity_limit)
    {
        Velocity+=Increase_rate;
        if(Velocity>Velocity_limit)
            Velocity=Velocity_limit;
    }
    else if(Velocity>Velocity_limit)
    {
        Velocity-=Increase_rate;
        if(Velocity<Velocity_limit)
            Velocity=Velocity_limit;
    }

    return Velocity;
}

float Increase_Differ(float Increase_rate,float Differ,float Differ_limit)
{
    if(Differ<Differ_limit)
    {
        Differ+=Increase_rate;
        if(Differ>Differ_limit)
            Differ=Differ_limit;
    }
    else if(Differ>Differ_limit)
    {
        Differ-=Increase_rate;
        if(Differ<Differ_limit)
            Differ=Differ_limit;
    }
    return Differ;
}

