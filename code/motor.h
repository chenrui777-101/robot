#ifndef _MOTOR_H
#define _MOTOR_H

typedef enum // ·½ÏòÃ¶¾Ù
{
    Go = 1,
    Back = 0,
    Left = 0,
    Right = 1,
    CW = 1,
    CCW = 0,
}Direction_enum;

#define Left_CW         gpio_set_level(P02_4,1);
#define Left_CCW        gpio_set_level(P02_4,0);
#define Left_Stop       gpio_set_level(P11_2,0);
#define Left_Start      gpio_set_level(P11_2,1);

#define Right_CW        gpio_set_level(P02_6,1);
#define Right_CCW       gpio_set_level(P02_6,0);
#define Right_Stop      gpio_set_level(P11_3,0);
#define Right_Start     gpio_set_level(P11_3,1);


void Brushless_Out_Pwm(Direction_enum Left_or_Right,Direction_enum CW_or_CCW ,uint32 out);
void DC_Out_Pwm(Direction_enum Go_or_Back,uint32 out);
float Increase_Velocity(float Increase_rate,float Velocity,float Velocity_limit);
float Increase_Differ(float Increase_rate,float Differ,float Differ_limit);


#endif
