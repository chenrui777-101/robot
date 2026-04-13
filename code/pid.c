#include "pid.h"
/************************************************************************************************************************/
//         author: SWUST-CR
//         date:2024-6-16
/************************************************************************************************************************/

/*******动量轮平衡环结构体*******/
Pid_Pos LR_Balance_Gyro;
Pid_Pos LR_Balance_Angle;
Pid_Pos LR_Balance_Speed;

/*******行进轮平衡环结构体*******/
Pid_Pos FB_Balance_Gyro;
Pid_Pos FB_Balance_Angle;
Pid_Pos FB_Balance_Speed;

/*******动量轮转向环结构体*******/
Pid_Pos Servo_Angle;
Pid_Pos Servo_Speed;
Pid_Pos Servo_Gyro;

/*******动态零点平衡环结构体*******/
Pid_Pos Dynamic_LR_Angle;
Pid_Pos Dynamic_FB_Speed;


/*******平衡角度环模糊控制参数***/
float LR_EFF[7] = {-6,-3,-1,0,1,3,6};//*输入量P语言值特征点*/EFF，即err的范围
float LR_DFF[7] = {-3,-1.5,-0.5,0,0.5,1.5,3};//*输入量D语言值特征点*/DFF，即err微分的范围 通常设置为EFF的1/2至1/3
float FB_EFF[7] = {-16,-8,-3,0,3,8,16};
float FB_DFF[7] = {-8,-4,-1.5,0,1.5,4,8};

float LR_UFF_kp[7] = {-8.8,-9.55,-11.48,-13.61,-16.775,-18.12,-20.12};//增益
float LR_UFF_kd[7] = {-38.36,-46.64,-52.53,-57.96,-61.77,-64.25,-66.82};//微分 max: -66
float FB_UFF_kp[7] = {-36.22,-39.76,-43.95,-48.82,-50.52,-52.8,-54.86};//比例
float FB_UFF_kd[7] = {-55.28,-56.73,-58.45,-60.28,-66.86,-70.12,-72.35};//微分 max: -72

/*******平衡前馈补偿结构体***/
Feedforward LR_Gyro_Compensate;
Feedforward LR_Angle_Compensate;

Feedforward FB_Gyro_Compensate;
Feedforward FB_Angle_Compensate;

void Pid_Position(Pid_Pos *pid)//位置式
{
    pid->Err = pid->Set - pid->Actual;

    if(pid->integration_B < pid->integration_A)//变速积分环节 抗饱和模式
    {
        if(abs(pid->Err) <= pid->integration_B)
        {
            pid->integration_K = 1;
        }
        else if(abs(pid->Err) > pid->integration_B && abs(pid->Err) <= pid->integration_A + pid->integration_B)//A + B 为上限
        {
            pid->integration_K = (pid->integration_A - abs(pid->Err) + pid->integration_B)/pid->integration_A;
        }
        else
        {
            pid->integration_K = 0;
        }
    }
    else if(pid->integration_B > pid->integration_A)   //变速积分环节 A < B 增强型积分模式
    {
        if(abs(pid->Err) <= pid->integration_A)
        {
            pid->integration_K = 0;
        }
        else if(abs(pid->Err) > pid->integration_A && abs(pid->Err) <= pid->integration_A + pid->integration_B)//A + B 为上限
        {
            pid->integration_K = (pid->integration_B - abs(pid->Err) + pid->integration_A)/pid->integration_B;
        }
        else
        {
            pid->integration_K = 1;
        }
    }
    else if(pid->integration_B == pid->integration_A)//不启用变速积分
    {
        pid->integration_K = 1;
    }

    pid->increment += (pid->Err + pid->Err_Next) * 0.5f * pid->integration_K;  //梯形积分

//    pid->increment += (pid->Err - pid->Err_Next)/2.0;//微小项积分补偿

    pid->increment = limit_value(pid->increment,pid->Limit_increment,(-pid->Limit_increment));

    pid->proportion =  pid->P * pid->Err;

    pid->integration = pid->I * pid->increment;

    pid->differential = -(pid->Actual - pid->Actual_Next) * pid->D;

    pid->Err_Next = pid->Err;

    pid->Actual_Next = pid->Actual;

    pid->Set_Next = pid->Set;

    pid->Out = pid->proportion  + pid->integration + pid->differential;
}

void Pid_Increse(Pid_Inc *pid)//增量式
{
    pid->Err = pid->Set - pid->Actual;

    pid->proportion = pid->P * (pid->Err - pid->Err_Next);

    pid->integration = pid->I * pid->Err;

    pid->differential = pid->D * (pid->Err - 2*pid->Err_Next + pid->Err_Last);

    pid->increment = pid->proportion + pid->integration + pid->differential;

    pid->Out += pid->increment;

    pid->Err_Last = pid->Err_Next;

    pid->Err_Next = pid->Err;
}

void DF_Pid_Position(Pid_Pos *pid)//位置式 Differentiation first  微分先行  对于设定值频繁变化的系统控制效果更佳
{
    pid->Err = pid->Set - pid->Actual;

    pid->increment += pid->Err * pid->integration_K;

    pid->increment = limit_value(pid->increment,pid->Limit_increment,(-pid->Limit_increment));

    pid->proportion =  pid->P * pid->Err;

    pid->integration = pid->I * pid->increment;

    pid->differential = -(pid->Actual - pid->Actual_Next) * pid->D;

    pid->Err_Next= pid->Err;

    pid->Actual_Next = pid->Actual;

    pid->Out = pid->proportion  + pid->integration + pid->differential;
}

void Feedforward_compensate(Feedforward *compensate)//前馈补偿
{
    compensate->Compensate_Out = compensate->Set_value*compensate->K1_Feedforward;
    compensate->Compensate_Out_differential = (compensate->Set_value - compensate->Set_value_Next)*compensate->K2_Feedforward;
    compensate->Out = compensate->Compensate_Out + compensate->Compensate_Out_differential;

    compensate->Set_value_Next = compensate->Set_value;
}

float limit_value(float value,float positive_value,float negative_value)
{
    if(value>positive_value)
    {
        value=positive_value;
    }
    if(value<negative_value)
    {
        value=negative_value;
    }
    return value;
}

/*************************************************FUZZY_CONTROL*****************************************************************/
/*模糊PID可以克服普通PID无法及时调整参数的缺点*/
float Fuzzy_P(float E,float EC,float *EFF,float *DFF,float *UFF)
{
    int rule[7][7]=
    {
        { 6 , 5 , 4 , 3 , 2 , 1 , 0},//0
        { 5 , 4 , 3 , 2 , 1 , 0 , 1},//1
        { 4 , 3 , 2 , 1 , 0 , 1 , 2},//2
        { 3 , 2 , 1 , 0 , 1 , 2 , 3},//3
        { 2 , 1 , 0 , 1 , 2 , 3 , 4},//4
        { 1 , 0 , 1 , 2 , 3 , 4 , 5},//5
        { 0 , 1 , 2 , 3 , 4 , 5 , 6},//6
    };

    float U=0;  /*偏差,偏差微分以及输出值的精确量*/
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*偏差,偏差微分以及输出值的隶属度*/
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*隶属度的确定*/
    /*根据PD的指定语言值获得有效隶属度*/
    if(E>EFF[0] && E<EFF[6])
    {
        if(E<=EFF[1])
        {
            Pn=-2;
            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
        }
        else if(E<=EFF[2])
        {
            Pn=-1;
            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
        }
        else if(E<=EFF[3])
        {
            Pn=0;
            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
        }
        else if(E<=EFF[4])
        {
            Pn=1;
            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
        }
        else if(E<=EFF[5])
        {
            Pn=2;
            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
        }
        else if(E<=EFF[6])
        {
            Pn=3;
            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
        }
    }

    else if(E<=EFF[0])
    {
        Pn=-2;
        PF[0]=1;
    }
    else if(E>=EFF[6])
    {
        Pn=3;
        PF[0]=0;
    }

    PF[1]=1-PF[0];

    //判断D的隶属度
    if(EC>DFF[0]&&EC<DFF[6])
    {
        if(EC<=DFF[1])
        {
            Dn=-2;
            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
        }
        else if(EC<=DFF[2])
        {
            Dn=-1;
            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
        }
        else if(EC<=DFF[3])
        {
            Dn=0;
            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
        }
        else if(EC<=DFF[4])
        {
            Dn=1;
            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
        }
        else if(EC<=DFF[5])
        {
            Dn=2;
            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
        }
        else if(EC<=DFF[6])
        {
            Dn=3;
            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
        }
    }
    //不在给定的区间内
    else if (EC<=DFF[0])
    {
        Dn=-2;
        DF[0]=1;
    }
    else if(EC>=DFF[6])
    {
        Dn=3;
        DF[0]=0;
    }

    DF[1]=1-DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0]=rule[Pn+2][Dn+2];
    Un[1]=rule[Pn+3][Dn+2];
    Un[2]=rule[Pn+2][Dn+3];
    Un[3]=rule[Pn+3][Dn+3];

    if(PF[0]<=DF[0])    //求小
        UF[0]=PF[0];
    else
        UF[0]=DF[0];

    if(PF[1]<=DF[0])
        UF[1]=PF[1];
    else
        UF[1]=DF[0];

    if(PF[0]<=DF[1])
        UF[2]=PF[0];
    else
        UF[2]=DF[1];

    if(PF[1]<=DF[1])
        UF[3]=PF[1];
    else
        UF[3]=DF[1];
    /*同隶属函数输出语言值求大*/
    if(Un[0]==Un[1])
    {
        if(UF[0]>UF[1])
            UF[1]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[2])
    {
        if(UF[0]>UF[2])
            UF[2]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[3])
    {
        if(UF[0]>UF[3])
            UF[3]=0;
        else
            UF[0]=0;
    }
    if(Un[1]==Un[2])
    {
        if(UF[1]>UF[2])
            UF[2]=0;
        else
            UF[1]=0;
    }
    if(Un[1]==Un[3])
    {
        if(UF[1]>UF[3])
            UF[3]=0;
        else
            UF[1]=0;
    }
    if(Un[2]==Un[3])
    {
        if(UF[2]>UF[3])
            UF[3]=0;
        else
            UF[2]=0;
    }
    t1=UF[0]*UFF[Un[0]];
    t2=UF[1]*UFF[Un[1]];
    t3=UF[2]*UFF[Un[2]];
    t4=UF[3]*UFF[Un[3]];
    temp1=t1+t2+t3+t4;
    temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
    U=temp1/temp2;
    return U;
}

float Fuzzy_D(float E,float EC,float *EFF,float *DFF,float *UFF)
{
    int rule[7][7]=
    {
            { 6 , 1 , 2 , 3 , 4 , 5 , 6},//0
            { 1 , 2 , 3 , 4 , 5 , 6 , 5},//1
            { 2 , 3 , 4 , 5 , 6 , 5 , 4},//2
            { 3 , 4 , 5 , 6 , 5 , 4 , 3},//3
            { 4 , 5 , 6 , 5 , 4 , 3 , 2},//4
            { 5 , 6 , 5 , 4 , 3 , 2 , 1},//5
            { 6 , 5 , 4 , 3 , 2 , 1 , 0},//6
    };

    float U=0;  /*偏差,偏差微分以及输出值的精确量*/
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*偏差,偏差微分以及输出值的隶属度*/
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*隶属度的确定*/
    /*根据PD的指定语言值获得有效隶属度*/
    if(E>EFF[0] && E<EFF[6])
    {
        if(E<=EFF[1])
        {
            Pn=-2;
            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
        }
        else if(E<=EFF[2])
        {
            Pn=-1;
            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
        }
        else if(E<=EFF[3])
        {
            Pn=0;
            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
        }
        else if(E<=EFF[4])
        {
            Pn=1;
            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
        }
        else if(E<=EFF[5])
        {
            Pn=2;
            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
        }
        else if(E<=EFF[6])
        {
            Pn=3;
            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
        }
    }

    else if(E<=EFF[0])
    {
        Pn=-2;
        PF[0]=1;
    }
    else if(E>=EFF[6])
    {
        Pn=3;
        PF[0]=0;
    }

    PF[1]=1-PF[0];


    //判断D的隶属度
    if(EC>DFF[0]&&EC<DFF[6])
    {
        if(EC<=DFF[1])
        {
            Dn=-2;
            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
        }
        else if(EC<=DFF[2])
        {
            Dn=-1;
            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
        }
        else if(EC<=DFF[3])
        {
            Dn=0;
            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
        }
        else if(EC<=DFF[4])
        {
            Dn=1;
            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
        }
        else if(EC<=DFF[5])
        {
            Dn=2;
            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
        }
        else if(EC<=DFF[6])
        {
            Dn=3;
            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
        }
    }
    //不在给定的区间内
    else if (EC<=DFF[0])
    {
        Dn=-2;
        DF[0]=1;
    }
    else if(EC>=DFF[6])
    {
        Dn=3;
        DF[0]=0;
    }

    DF[1]=1-DF[0];

    /*使用误差范围优化后的规则表rule[7][7]*/
    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
    /*一般都是四个规则有效*/
    Un[0]=rule[Pn+2][Dn+2];
    Un[1]=rule[Pn+3][Dn+2];
    Un[2]=rule[Pn+2][Dn+3];
    Un[3]=rule[Pn+3][Dn+3];

    if(PF[0]<=DF[0])    //求小
        UF[0]=PF[0];
    else
        UF[0]=DF[0];
    if(PF[1]<=DF[0])
        UF[1]=PF[1];
    else
        UF[1]=DF[0];
    if(PF[0]<=DF[1])
        UF[2]=PF[0];
    else
        UF[2]=DF[1];
    if(PF[1]<=DF[1])
        UF[3]=PF[1];
    else
        UF[3]=DF[1];
    /*同隶属函数输出语言值求大*/
    if(Un[0]==Un[1])
    {
        if(UF[0]>UF[1])
            UF[1]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[2])
    {
        if(UF[0]>UF[2])
            UF[2]=0;
        else
            UF[0]=0;
    }
    if(Un[0]==Un[3])
    {
        if(UF[0]>UF[3])
            UF[3]=0;
        else
            UF[0]=0;
    }
    if(Un[1]==Un[2])
    {
        if(UF[1]>UF[2])
            UF[2]=0;
        else
            UF[1]=0;
    }
    if(Un[1]==Un[3])
    {
        if(UF[1]>UF[3])
            UF[3]=0;
        else
            UF[1]=0;
    }
    if(Un[2]==Un[3])
    {
        if(UF[2]>UF[3])
            UF[3]=0;
        else
            UF[2]=0;
    }
    t1=UF[0]*UFF[Un[0]];
    t2=UF[1]*UFF[Un[1]];
    t3=UF[2]*UFF[Un[2]];
    t4=UF[3]*UFF[Un[3]];
    temp1=t1+t2+t3+t4;
    temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
    U=temp1/temp2;
    return U;
}
