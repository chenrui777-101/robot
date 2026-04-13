#ifndef _DEAL_DATA_H
#define _DEAL_DATA_H
#include "deal_imu.h"
void LR_Balance_Gyro_Deal(void);
void LR_Balance_Angle_Deal(void);
void LR_Balance_Speed_Deal(void);
void FB_Balance_Gyro_Deal(void);
void FB_Balance_Angle_Deal(void);
void FB_Balance_Speed_Deal(void);
void Servo_Gyro_Deal(void);
void Servo_Angle_Deal(void);
void Servo_Speed_Deal(void);
void Encoder_Deal(void);
void Dynamic_Deal_Angle(void);
void Dynamic_Deal_speed(void);
void Dynamic_Deal_Speed(void);

extern float L_Balance_Gyro_Out;
extern float R_Balance_Gyro_Out;
extern float Mid_angle_X;
extern float Mid_angle_X_zero;
extern float Mid_angle_Y;
extern float Mid_angle_Y_zero;
extern float expect_angle;
extern float Expect_Velocity;
extern float Turn_PWM;
extern float Velocity_limit;
extern float Velocity_limit_zero;
extern float Velocity_limit_servo;
extern float Velocity_limit_strike;
extern float Dynamic_EX;
extern float differ;
extern float speed_last;
extern float servo_last;
extern float servo;

extern int16 speed_march;
extern int16 speed_front;
extern int16 speed_back;
extern int32 distance;

extern move_filter_struct speed_march_filter;

#endif
