#include "zf_common_headfile.h"

float L_Balance_Gyro_Out=0;
float R_Balance_Gyro_Out=0;
float Mid_angle_X, Mid_angle_Y;

float Mid_angle_X_zero = -0.6;//初始零点
float Mid_angle_Y_zero = 0.5;//初始零点

float expect_angle = 0.5;//期望角度
float Expect_Velocity = 0.0;
float Velocity_limit = 0.0;
float Velocity_limit_zero = 800;//期望速度上限
float Velocity_limit_servo = 600;//弯道固定速度
float Velocity_limit_strike = 450;//撞击速度
float differ = 0;//转向偏差值
float servo = 0;

float deadband = 1.0;//转向角度环闭环死区

float Turn_PWM = 0.0;//转向占空比之差

int16 speed_march;//编码器读取的速度值
int16 speed_front;
int16 speed_back;

int32 distance = 0;

move_filter_struct speed_march_filter;

float FB_Last_Out = 0;
uint8 FB_Gyro_flag = 0;

/******动态零点参数*******/
float Dynamic_EX = 6.0;

void LR_Balance_Gyro_Deal(void)
{
    LR_Balance_Gyro.Set         =LR_Balance_Angle.Out + LR_Balance_Angle.Set_differential;
    LR_Balance_Gyro.Actual      =Gyro_X;

    LR_Gyro_Compensate.Set_value = LR_Balance_Angle.Out;
    Feedforward_compensate(&LR_Gyro_Compensate);//前馈

    Pid_Position(&LR_Balance_Gyro);

    LR_Balance_Gyro.Out += LR_Gyro_Compensate.Out;

    L_Balance_Gyro_Out = LR_Balance_Gyro.Out + Turn_PWM;
    R_Balance_Gyro_Out = LR_Balance_Gyro.Out - Turn_PWM;

//    L_Balance_Gyro_Out = LR_Balance_Gyro.Out + Turn_PWM * (cosf((Mid_angle_Y - Angle_Y_Final) * AtR) + sinf((Mid_angle_Y - Angle_Y_Final) * AtR));//后飞轮
//    R_Balance_Gyro_Out = LR_Balance_Gyro.Out - Turn_PWM * (cosf((Mid_angle_Y - Angle_Y_Final) * AtR) - sinf((Mid_angle_Y - Angle_Y_Final) * AtR));//前飞轮

    if( Angle_X_Final<(-20.0) || Angle_X_Final>20.0 || Angle_Y_Final<(-30.0) || Angle_Y_Final>30.0 )//安全角度
    {
        L_Balance_Gyro_Out = 0;
        R_Balance_Gyro_Out = 0;
        Blance_flag = 0;
    }

    L_Balance_Gyro_Out = limit_value(L_Balance_Gyro_Out,9999,(-9999));
    R_Balance_Gyro_Out = limit_value(R_Balance_Gyro_Out,9999,(-9999));

    if(L_Balance_Gyro_Out>=0)
    {
        Brushless_Out_Pwm(Left,CCW,L_Balance_Gyro_Out);
    }
    if(R_Balance_Gyro_Out>=0)
    {
        Brushless_Out_Pwm(Right,CW,R_Balance_Gyro_Out);
    }
    if(L_Balance_Gyro_Out<0)
    {
        Brushless_Out_Pwm(Left,CW,(-L_Balance_Gyro_Out));
    }
    if(R_Balance_Gyro_Out<0)
    {
        Brushless_Out_Pwm(Right,CCW,(-R_Balance_Gyro_Out));
    }
}

void LR_Balance_Angle_Deal(void)//模糊处理
{
    LR_Balance_Angle.Set         =LR_Balance_Speed.Out + Mid_angle_X;
    LR_Balance_Angle.Actual      =Angle_X_Final;

    LR_Balance_Angle.Set_differential = (LR_Balance_Angle.Set - LR_Balance_Angle.Set_Next) * LR_Balance_Angle.C;

    LR_Balance_Angle.Err = LR_Balance_Angle.Set - LR_Balance_Angle.Actual;

    if(abs(LR_Balance_Angle.Err) <= LR_Balance_Angle.integration_B)
    {
        LR_Balance_Angle.integration_K = 1;
    }
    else if(abs(LR_Balance_Angle.Err) > LR_Balance_Angle.integration_B && abs(LR_Balance_Angle.Err) <= LR_Balance_Angle.integration_A + LR_Balance_Angle.integration_B)//A + B 为上限
    {
        LR_Balance_Angle.integration_K = (LR_Balance_Angle.integration_A - abs(LR_Balance_Angle.Err) + LR_Balance_Angle.integration_B)/LR_Balance_Angle.integration_A;
    }
    else
    {
        LR_Balance_Angle.integration_K = 0;
    }

    LR_Balance_Angle.increment += (LR_Balance_Angle.Err + LR_Balance_Angle.Err_Next) * 0.5f * LR_Balance_Angle.integration_K;

    LR_Balance_Angle.increment = limit_value(LR_Balance_Angle.increment,LR_Balance_Angle.Limit_increment,(-LR_Balance_Angle.Limit_increment));//积分限幅

    LR_Balance_Angle.P = Fuzzy_P(Mid_angle_X - LR_Balance_Angle.Actual,LR_Balance_Angle.Actual_Next - LR_Balance_Angle.Actual,LR_EFF,LR_DFF,LR_UFF_kp);//模糊Kp
//    LR_Balance_Angle.D = Fuzzy_D(Mid_angle_X - LR_Balance_Angle.Actual,LR_Balance_Angle.Actual_Next - LR_Balance_Angle.Actual,LR_EFF,LR_DFF,LR_UFF_kd);//模糊Kd

    LR_Balance_Angle.proportion =  LR_Balance_Angle.P * LR_Balance_Angle.Err;

    LR_Balance_Angle.integration = LR_Balance_Angle.I * LR_Balance_Angle.increment;

    LR_Balance_Angle.differential = -(LR_Balance_Angle.Actual - LR_Balance_Angle.Actual_Next) * LR_Balance_Angle.D;

    LR_Balance_Angle.Err_Next = LR_Balance_Angle.Err;

    LR_Balance_Angle.Actual_Next = LR_Balance_Angle.Actual;

    LR_Balance_Angle.Set_Next = LR_Balance_Angle.Set;

    LR_Balance_Angle.Out = LR_Balance_Angle.proportion  + LR_Balance_Angle.integration + LR_Balance_Angle.differential;

}

void LR_Balance_Speed_Deal(void)
{
    LR_Balance_Speed.Set         =0;
    LR_Balance_Speed.Actual      = (speed_front + speed_back)*0.5;

    Pid_Position(&LR_Balance_Speed);

    LR_Balance_Speed.Out = limit_value(LR_Balance_Speed.Out,20,(-20));
}

void FB_Balance_Gyro_Deal(void)
{
    FB_Balance_Gyro.Set         =FB_Balance_Angle.Out + FB_Balance_Angle.Set_differential;
    FB_Balance_Gyro.Actual      =-Gyro_Z;

    FB_Gyro_Compensate.Set_value = FB_Balance_Angle.Out;
    Feedforward_compensate(&FB_Gyro_Compensate);

    Pid_Position(&FB_Balance_Gyro);

    FB_Balance_Gyro.Out += FB_Gyro_Compensate.Out;

    if(FB_Balance_Gyro.Out>0)//  死区线性补偿配合内环积分 控制静态平衡
        FB_Balance_Gyro.Out += 120;
    else if(FB_Balance_Gyro.Out<0)
        FB_Balance_Gyro.Out -= 120;

    if(FB_Gyro_flag==0)//做一次一阶低通滤波处理高频信号
    {
        FB_Last_Out = FB_Balance_Gyro.Out;
        FB_Gyro_flag=1;
    }
    else
    {
        FB_Balance_Gyro.Out = FB_Last_Out*0.3+0.7*FB_Balance_Gyro.Out;
        FB_Last_Out = FB_Balance_Gyro.Out;
    }

    FB_Balance_Gyro.Out =limit_value(FB_Balance_Gyro.Out,4500,(-4500));

    if( Angle_X_Final<(-20.0) || Angle_X_Final>20.0 || Angle_Y_Final<(-30.0) || Angle_Y_Final>30.0 )//安全角度
    {
        FB_Balance_Gyro.Out=0;
        Blance_flag = 0;
    }

    if(FB_Balance_Gyro.Out>=0)
    {
        DC_Out_Pwm(Go,FB_Balance_Gyro.Out);
    }
    if(FB_Balance_Gyro.Out<0)
    {
        DC_Out_Pwm(Back,(-FB_Balance_Gyro.Out));
    }
}

void FB_Balance_Angle_Deal(void)//模糊处理
{
    FB_Balance_Angle.Set         =FB_Balance_Speed.Out + Mid_angle_Y;
    FB_Balance_Angle.Actual      =Angle_Y_Final;

    FB_Balance_Angle.Set_differential = (FB_Balance_Angle.Set - FB_Balance_Angle.Set_Next) * FB_Balance_Angle.C;

    FB_Balance_Angle.Err = FB_Balance_Angle.Set - FB_Balance_Angle.Actual;

    if(abs(FB_Balance_Angle.Err) <= FB_Balance_Angle.integration_B)
    {
        FB_Balance_Angle.integration_K = 1;
    }
    else if(abs(FB_Balance_Angle.Err) > FB_Balance_Angle.integration_B && abs(FB_Balance_Angle.Err) <= FB_Balance_Angle.integration_A + FB_Balance_Angle.integration_B)//A + B 为上限
    {
        FB_Balance_Angle.integration_K = (FB_Balance_Angle.integration_A - abs(FB_Balance_Angle.Err) + FB_Balance_Angle.integration_B)/FB_Balance_Angle.integration_A;
    }
    else
    {
        FB_Balance_Angle.integration_K = 0;
    }

    FB_Balance_Angle.increment += (FB_Balance_Angle.Err + FB_Balance_Angle.Err_Next) * 0.5f * FB_Balance_Angle.integration_K;

    FB_Balance_Angle.increment = limit_value(FB_Balance_Angle.increment,FB_Balance_Angle.Limit_increment,(-FB_Balance_Angle.Limit_increment));//积分限幅

    FB_Balance_Angle.P = Fuzzy_P(Mid_angle_Y - FB_Balance_Angle.Actual,FB_Balance_Angle.Actual_Next - FB_Balance_Angle.Actual,FB_EFF,FB_DFF,FB_UFF_kp);//模糊Kp
    FB_Balance_Angle.D = Fuzzy_D(Mid_angle_Y - FB_Balance_Angle.Actual,FB_Balance_Angle.Actual_Next - FB_Balance_Angle.Actual,FB_EFF,FB_DFF,FB_UFF_kd);//模糊Kd

    FB_Balance_Angle.proportion =  FB_Balance_Angle.P * FB_Balance_Angle.Err;

    FB_Balance_Angle.integration = FB_Balance_Angle.I * FB_Balance_Angle.increment;

    FB_Balance_Angle.differential = -(FB_Balance_Angle.Actual - FB_Balance_Angle.Actual_Next) * FB_Balance_Angle.D;

    FB_Balance_Angle.Err_Next = FB_Balance_Angle.Err;

    FB_Balance_Angle.Actual_Next = FB_Balance_Angle.Actual;

    FB_Balance_Angle.Set_Next = FB_Balance_Angle.Set;

    FB_Balance_Angle.Out = FB_Balance_Angle.proportion  + FB_Balance_Angle.integration + FB_Balance_Angle.differential;

//    if(Off_ground_detection())//离地检测
//    {
//        FB_Balance_Angle.Out = 0;
//    }
}

void FB_Balance_Speed_Deal(void)
{
    FB_Balance_Speed.Set         = Expect_Velocity - Dynamic_FB_Speed.Out;
    FB_Balance_Speed.Actual      = speed_march;

    Pid_Position(&FB_Balance_Speed);

    FB_Balance_Speed.Out = limit_value( FB_Balance_Speed.Out,30,(-30));
}

void Servo_Gyro_Deal(void)
{
    Servo_Gyro.Set         =Servo_Angle.Out;
    Servo_Gyro.Actual      =Gyro_Y;//角速度转向处理

    Pid_Position(&Servo_Gyro);
    Servo_Gyro.Out = limit_value(Servo_Gyro.Out,9000,-9000);

    Turn_PWM = Servo_Gyro.Out;
}

void Servo_Angle_Deal(void)
{
    Servo_Angle.Set         =Servo_Speed.Out;
    Servo_Angle.Actual      = differ;

    Pid_Position(&Servo_Angle);

    Servo_Angle.Out = limit_value(Servo_Angle.Out,160,-160);
}

void Servo_Speed_Deal(void)
{
    Servo_Speed.Set         =0;
    if(speed_front - speed_back < 10 && speed_front - speed_back > -10)
    {
        Servo_Speed.Actual  =   0;
    }
    else
    {
        Servo_Speed.Actual      =speed_front - speed_back;//加速度转向处理
    }

    Pid_Position(&Servo_Speed);
    Servo_Speed.Out = limit_value(Servo_Speed.Out,60,-60);
}

void Encoder_Deal(void)
{
    speed_march = encoder_get_count(TIM4_ENCODER);
    encoder_clear_count(TIM4_ENCODER);

    move_filter_calc(&speed_march_filter, speed_march);//防止速度突增平滑处理
    speed_march = speed_march_filter.data_average;

    if(ins_write)//惯性导航采集路径信息
    {
        ins.distance = speed_march;
        ins.Angle_deviation = yaw ;
        trajectory_calculation_2d(&ins);
        if(INS_forward_flag)
        {
            INS_forward += speed_march;
            if(INS_forward > Distance_INS)
            {
                dlla_out[4]=10000;
                dlla_out[3]=10000;
                dlla_out[2]=10000;
                dlla_out[1]=10000;
                dlla_out[0]=10000;
                INS_forward_flag = 0;
                INS_forward = 0;
                m++;
            }
        }
    }

    speed_front = encoder_get_count(TIM2_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    speed_back = -encoder_get_count(TIM5_ENCODER);
    encoder_clear_count(TIM5_ENCODER);

}

void Dynamic_Deal_Angle(void)
{
    Dynamic_LR_Angle.Set         =0;

    if(differ < 5 && differ > -5)
    {
        LR_Balance_Speed.increment = Increase_Differ(20, LR_Balance_Speed.increment, 0);
        Servo_Angle.increment = Increase_Differ(1, Servo_Angle.increment, 0);
        Dynamic_LR_Angle.Actual = 0;
    }
    else
    {
        Dynamic_LR_Angle.Actual = Gyro_Y;
    }

    Pid_Position(&Dynamic_LR_Angle);

    if(speed_march < 100)
    {
        Dynamic_LR_Angle.Out = Dynamic_LR_Angle.integration;
    }

    Dynamic_LR_Angle.Out = limit_value(Dynamic_LR_Angle.Out,Dynamic_EX,-Dynamic_EX);

    Mid_angle_X = Mid_angle_X_zero + Dynamic_LR_Angle.Out;
}

void Dynamic_Deal_Speed(void)
{
    Dynamic_FB_Speed.Set         =0;

    if(differ < variable_speed_servo && differ > -variable_speed_servo)
    {
        Dynamic_FB_Speed.Actual = 0;
    }
    else if(differ > 90 || differ < -90)
    {
        Dynamic_FB_Speed.Actual = 1;
    }
    else
    {
        Dynamic_FB_Speed.Actual = fabs(sinf(differ * AtR));
    }

    Pid_Position(&Dynamic_FB_Speed);
}
