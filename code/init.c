#include "zf_common_headfile.h"

void init_all()
{
   motor_int();                                   //行进轮电机初始化

   blance_gpio_init();                            //动量轮电机初始化

   Gpio_init();                               //按键初始化

   ips_init();                                //IPS屏幕初始化

   imu963ra_init();                               //九轴陀螺仪初始化

   //     wireless_uart_init();                       //无线串口初始化

    encoder_init();                                //编码器初始化

    Move_filter_init();                            //滑动平均滤波初始化

    Gyro_Off_SetInit();                           //陀螺仪零漂采集

    Mode_selection();                             //模式选择

    pid_init_highspeed();

    Mid_Zero_init();                               //动态零点初始化

    PIT_init();                                    //PIT定时中断初始化
}

void Mode_selection(void)
{
    if(KEY_mode)
    {
        dot_matrix_screen_init();                   // 点阵屏幕初始化
        gnss_init(GN43RFA);                            //rtk初始化
        car_flag = 2;
    }
    else if(!KEY_mode)
    {
        init_ins(&ins);                                //惯性导航初始化
        dl1b_init();                                   //测距模块初始化
        scc8660_init();                                  //凌瞳初始化
        car_flag = 3;
    }

    if(KEY_mode)
    {
        Mode_ = "RTK";
    }
    else
    {
        Mode_ = "INS";
    }

}

void motor_int()
{
    gpio_init(P21_3,GPO,0,GPO_PUSH_PULL);//方向引脚，0反转，1正转
    pwm_init(ATOM0_CH0_P21_2, 12000, 0); //直流电机PWM
}

void blance_gpio_init()
{
    /********无刷前*********/
    pwm_init(ATOM1_CH5_P02_5, 13333, 10000);//PWM波0为满转
    gpio_init(P11_2,GPO,0,GPO_PUSH_PULL);//刹车引脚，0刹车，1不刹车
    gpio_init(P02_4,GPO,0,GPO_PUSH_PULL);//1以CW(顺时针)转，0以CCW(逆时针)转

    /********无刷后*********/
    pwm_init(ATOM1_CH7_P02_7, 13333, 10000);
    gpio_init(P11_3,GPO,0,GPO_PUSH_PULL);
    gpio_init(P02_6,GPO,0,GPO_PUSH_PULL);
}

void ips_init()
{
    ips200_init(IPS200_TYPE_PARALLEL8);//ips屏幕初始化
    ips200_full(RGB565_BLACK);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
}

void Mid_Zero_init()//动态零点初始化
{
    Mid_angle_X=Mid_angle_X_zero;
    Mid_angle_Y=Mid_angle_Y_zero;
}

void Gpio_init()//按键初始化
{


    gpio_init(P20_6, GPI, 1, GPI_PULL_UP);
    gpio_init(P20_7, GPI, 1, GPI_PULL_UP);
    gpio_init(P20_8, GPI, 1, GPI_PULL_UP);
    gpio_init(P20_2, GPI, 1, GPI_PULL_UP);

    gpio_init(P21_6, GPI, 1, GPI_PULL_UP);

    gpio_init(P22_1, GPI, 0, GPI_FLOATING_IN);
    gpio_init(P22_0, GPI, 0, GPI_FLOATING_IN);

}

void encoder_init()
{
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);//行进轮编码器初始化
    encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);//前后动量轮编码器初始化
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
}

void yaokong (void)
{
    gpio_init(P22_2, GPO, 0, GPO_PUSH_PULL);  //e34模式配置初始化
    gpio_init(P21_7, GPO, 1, GPO_PUSH_PULL);
    uart_init(UART_0, 9600, UART0_TX_P14_0, UART0_RX_P14_1);   //e34初始化
    uart_rx_interrupt(UART_0,1);
}

void PIT_init()
{
    pit_ms_init(CCU60_CH0, 2);

}

void Move_filter_init()//滑动平均滤波初始化
{
    weighting_sum_init();
    move_filter_init(&gyro_x_filter);
    move_filter_init(&gyro_y_filter);
    move_filter_init(&gyro_z_filter);
    move_filter_init(&acc_off_ground_filter);
    move_filter_init(&speed_march_filter);
}

void TD_init()//TD跟踪微分器初始化
{
    TD_LR_Angle.v1 = 0;
    TD_LR_Angle.v2 = 0;
    TD_LR_Angle.h = 0.006;
    TD_LR_Angle.r = 150;

    TD_FB_Angle.v1 = 0;
    TD_FB_Angle.v2 = 0;
    TD_FB_Angle.h = 0.006;
    TD_FB_Angle.r = 150;
}

void ESO_init()//ESO扩张状态观测器初始化
{
    ESO_LR_Angle.z1 = 0;
    ESO_LR_Angle.z2 = 0;
    ESO_LR_Angle.z3 = 0;
    ESO_LR_Angle.w0 = 10;
    ESO_LR_Angle.wc = 280;
    ESO_LR_Angle.b0 = 0.006;
    ESO_LR_Angle.u = 0.0;
    ESO_LR_Angle.h = 0.006;

    ESO_FB_Angle.z1 = 0;
    ESO_FB_Angle.z2 = 0;
    ESO_FB_Angle.z3 = 0;
    ESO_FB_Angle.w0 = 10;
    ESO_FB_Angle.wc = 280;
    ESO_FB_Angle.b0 = 0.006;
    ESO_FB_Angle.u = 0.0;
    ESO_FB_Angle.h = 0.006;
}

void pid_init_highspeed()
{

    /**********动量轮参数*********/
        //角速度环引入前馈控制
        LR_Gyro_Compensate.Set_value           =0;
        LR_Gyro_Compensate.Set_value_Next      =0;
        LR_Gyro_Compensate.Compensate_Out      =0;
        LR_Gyro_Compensate.Compensate_Out_differential =0;
        LR_Gyro_Compensate.Out                 =0;
        LR_Gyro_Compensate.K1_Feedforward      =82.2;
        LR_Gyro_Compensate.K2_Feedforward      =8.35;

        LR_Balance_Gyro.Actual      =0;
        LR_Balance_Gyro.Actual_Next =0;
        LR_Balance_Gyro.Set         =0;
        LR_Balance_Gyro.Set_Next    =0;
        LR_Balance_Gyro.Err         =0;
        LR_Balance_Gyro.Err_Next    =0;
        LR_Balance_Gyro.P           =490.0;//490
        LR_Balance_Gyro.integration_A   =0;
        LR_Balance_Gyro.integration_B   =0;
        LR_Balance_Gyro.Limit_increment =200;
        LR_Balance_Gyro.I           =0.8;//2.6
        LR_Balance_Gyro.D           =22.0;//32
        LR_Balance_Gyro.differential=0;
        LR_Balance_Gyro.integration =0;
        LR_Balance_Gyro.increment   =0;
        LR_Balance_Gyro.proportion  =0;
        LR_Balance_Gyro.Out         =0;

        //角度环使用模糊控制（P）
        LR_Balance_Angle.Actual      =0;
        LR_Balance_Angle.Actual_Next =0;
        LR_Balance_Angle.Set         =0;
        LR_Balance_Angle.Set_Next    =0;
        LR_Balance_Angle.Err         =0;
        LR_Balance_Angle.Err_Next    =0;
        LR_Balance_Angle.P           =0;//在模糊控制器里确定
        LR_Balance_Angle.integration_A   =9;
        LR_Balance_Angle.integration_B   =1;
        LR_Balance_Angle.Limit_increment =100;
        LR_Balance_Angle.I           =-0.025;//0.08
        LR_Balance_Angle.D           =-66;//-13.6
        LR_Balance_Angle.C           =-5;
        LR_Balance_Angle.differential=0;
        LR_Balance_Angle.integration =0;
        LR_Balance_Angle.proportion  =0;
        LR_Balance_Angle.increment   =0;
        LR_Balance_Angle.Out         =0;

        LR_Balance_Speed.Actual      =0;
        LR_Balance_Speed.Actual_Next =0;
        LR_Balance_Speed.Set         =0;
        LR_Balance_Speed.Set_Next    =0;
        LR_Balance_Speed.Err         =0;
        LR_Balance_Speed.Err_Next    =0;
        LR_Balance_Speed.P           =0.048;//0.045
        LR_Balance_Speed.integration_A   =0;
        LR_Balance_Speed.integration_B   =0;
        LR_Balance_Speed.Limit_increment =8000;
        LR_Balance_Speed.I           =0.00022;//0.00026
        LR_Balance_Speed.D           =0.00;//0.0
        LR_Balance_Speed.differential=0;
        LR_Balance_Speed.integration =0;
        LR_Balance_Speed.proportion  =0;
        LR_Balance_Speed.increment   =0;
        LR_Balance_Speed.Out         =0;

        /**********行进轮轮参数*********/
        //角速度环引入前馈控制
        FB_Gyro_Compensate.Set_value           = 0;
        FB_Gyro_Compensate.Set_value_Next      = 0;
        FB_Gyro_Compensate.Compensate_Out      = 0;
        FB_Gyro_Compensate.Compensate_Out_differential = 0;
        FB_Gyro_Compensate.Out                 = 0;
        FB_Gyro_Compensate.K1_Feedforward      = 3.7;
        FB_Gyro_Compensate.K2_Feedforward      = 0.25;

        FB_Balance_Gyro.Actual      =0;
        FB_Balance_Gyro.Actual_Next =0;
        FB_Balance_Gyro.Set         =0;
        FB_Balance_Gyro.Set_Next    =0;
        FB_Balance_Gyro.Err         =0;
        FB_Balance_Gyro.Err_Next    =0;
        FB_Balance_Gyro.P           =17.2;//12.2
        FB_Balance_Gyro.integration_A   =0;
        FB_Balance_Gyro.integration_B   =0;
        FB_Balance_Gyro.Limit_increment =200;
        FB_Balance_Gyro.I           =0.12;//0.4 配合电机死区补偿
        FB_Balance_Gyro.D           =26.8;//21.6
        FB_Balance_Gyro.differential=0;
        FB_Balance_Gyro.integration =0;
        FB_Balance_Gyro.increment   =0;
        FB_Balance_Gyro.proportion  =0;
        FB_Balance_Gyro.Out         =0;

         //角度环使用模糊控制（P）
        FB_Balance_Angle.Actual      =0;
        FB_Balance_Angle.Actual_Next =0;
        FB_Balance_Angle.Set         =0;
        FB_Balance_Angle.Set_Next    =0;
        FB_Balance_Angle.Err         =0;
        FB_Balance_Angle.Err_Next    =0;
        FB_Balance_Angle.P           =0;//在模糊控制器中确定
        FB_Balance_Angle.integration_A   =18;
        FB_Balance_Angle.integration_B   =2;
        FB_Balance_Angle.Limit_increment =200;
        FB_Balance_Angle.I           =-0.04;//0.08
        FB_Balance_Angle.D           =-72;//-72  在模糊控制器中确定
        FB_Balance_Angle.C           =-12;
        FB_Balance_Angle.differential=0;
        FB_Balance_Angle.integration =0;
        FB_Balance_Angle.proportion  =0;
        FB_Balance_Angle.increment   =0;
        FB_Balance_Angle.Out         =0;

        FB_Balance_Speed.Actual      =0;
        FB_Balance_Speed.Actual_Next =0;
        FB_Balance_Speed.Set         =0;
        FB_Balance_Speed.Set_Next    =0;
        FB_Balance_Speed.Err         =0;
        FB_Balance_Speed.Err_Next    =0;
        FB_Balance_Speed.P           =-0.022;//-0.022
        FB_Balance_Speed.integration_A   =0;
        FB_Balance_Speed.integration_B   =0;
        FB_Balance_Speed.Limit_increment =3000;
        FB_Balance_Speed.I           =-0.0009;//-0.00003
        FB_Balance_Speed.D           =0.0;//-0.0
        FB_Balance_Speed.differential=0;
        FB_Balance_Speed.integration =0;
        FB_Balance_Speed.proportion  =0;
        FB_Balance_Speed.increment   =0;
        FB_Balance_Speed.Out         =0;

        /**********转向环参数*********/
        Servo_Gyro.Actual      =0;
        Servo_Gyro.Actual_Next =0;
        Servo_Gyro.Set         =0;
        Servo_Gyro.Set_Next    =0;
        Servo_Gyro.Err         =0;
        Servo_Gyro.Err_Next    =0;
        Servo_Gyro.P           =100.0;//50
        Servo_Gyro.integration_A   =0;
        Servo_Gyro.integration_B   =0;
        Servo_Gyro.Limit_increment =300;
        Servo_Gyro.I           =3.0;
        Servo_Gyro.D           =8.0;//5
        Servo_Gyro.differential=0;
        Servo_Gyro.integration =0;
        Servo_Gyro.proportion  =0;
        Servo_Gyro.increment   =0;
        Servo_Gyro.Out         =0;

        Servo_Angle.Actual      =0;
        Servo_Angle.Actual_Next =0;
        Servo_Angle.Set         =0;
        Servo_Angle.Set_Next    =0;
        Servo_Angle.Err         =0;
        Servo_Angle.Err_Next    =0;
        Servo_Angle.P           =1.75;
        Servo_Angle.integration_A   =0;
        Servo_Angle.integration_B   =0;
        Servo_Angle.Limit_increment =500;
        Servo_Angle.I           =0.02;
        Servo_Angle.D           =30;
        Servo_Angle.differential=0;
        Servo_Angle.integration =0;
        Servo_Angle.proportion  =0;
        Servo_Angle.increment   =0;
        Servo_Angle.Out         =0;

        Servo_Speed.Actual      =0;
        Servo_Speed.Actual_Next =0;
        Servo_Speed.Set         =0;
        Servo_Speed.Set_Next    =0;
        Servo_Speed.Err         =0;
        Servo_Speed.Err_Next    =0;
        Servo_Speed.P           =-0.0;  //-0.012
        Servo_Speed.integration_A   =0;
        Servo_Speed.integration_B   =0;
        Servo_Speed.Limit_increment =0;
        Servo_Speed.I           =0.0;
        Servo_Speed.D           =0.0;
        Servo_Speed.differential=0;
        Servo_Speed.integration =0;
        Servo_Speed.proportion  =0;
        Servo_Speed.increment   =0;
        Servo_Speed.Out         =0;

        /**********动态零点平衡环参数*********/
        Dynamic_LR_Angle.Actual      =0;
        Dynamic_LR_Angle.Actual_Next =0;
        Dynamic_LR_Angle.Set         =0;
        Dynamic_LR_Angle.Set_Next    =0;
        Dynamic_LR_Angle.Err         =0;
        Dynamic_LR_Angle.Err_Next    =0;
        Dynamic_LR_Angle.P           =-0.04;//-0.04
        Dynamic_LR_Angle.integration_A   =0;
        Dynamic_LR_Angle.integration_B   =0;
        Dynamic_LR_Angle.Limit_increment =500;
        Dynamic_LR_Angle.I           =-0.00;
        Dynamic_LR_Angle.D           =-0.02;//-0.01
        Dynamic_LR_Angle.differential=0;
        Dynamic_LR_Angle.integration =0;
        Dynamic_LR_Angle.proportion  =0;
        Dynamic_LR_Angle.increment   =0;
        Dynamic_LR_Angle.Out         =0;

        Dynamic_FB_Speed.Actual      =0;
        Dynamic_FB_Speed.Actual_Next =0;
        Dynamic_FB_Speed.Set         =0;
        Dynamic_FB_Speed.Set_Next    =0;
        Dynamic_FB_Speed.Err         =0;
        Dynamic_FB_Speed.Err_Next    =0;
        Dynamic_FB_Speed.P           =-0;
        Dynamic_FB_Speed.integration_A   =0;
        Dynamic_FB_Speed.integration_B   =0;
        Dynamic_FB_Speed.Limit_increment =20000;
        Dynamic_FB_Speed.I           =0.0;
        Dynamic_FB_Speed.D           =0;
        Dynamic_FB_Speed.differential=0;
        Dynamic_FB_Speed.integration =0;
        Dynamic_FB_Speed.proportion  =0;
        Dynamic_FB_Speed.increment   =0;
        Dynamic_FB_Speed.Out         =0;

}
