#ifndef CODE_DEAL_RTK_H_
#define CODE_DEAL_RTK_H_

typedef struct
{
    double Angle_deviation;                   // ∆´∫ΩΩ«
    double distance;                          // “∆∂Øæ‡¿Î
    double distance_x,distance_y;             // X,Y÷·…œµƒ“∆∂Øæ‡¿Î
}ins_struct;
extern ins_struct ins;

extern double tan_angel[4500];
extern double tans[4500];

void Angle_Deal(uint8 mode);
void data_get(uint8 mode);
void Velocity_Deal(float angle, double distance, double latitude_A, double longitude_A, double latitude_B, double longitude_B);
void caidian(uint8 mode);
void exist_point(void);
void trajectory_calculation_2d(ins_struct *ins);
double ins_deviation(double distance_x2,double distance_y2,double distance_x1,double distance_y1);
double count_distence(double x1,double y1,double x2,double y2);
void init_ins(ins_struct *ins);

void u32_to_u8(uint32 *data,uint8 *temp0,uint8 *temp1,uint8 *temp2,uint8 *temp3);
void u8_to_u32(uint32 *data,uint8 *temp0,uint8 *temp1,uint8 *temp2,uint8 *temp3);
void Disassembly_GPS(double latitude,double longitude,uint32 *x,uint32 *y);
void Disassembly_INS(double latitude,double longitude,uint32 *x,uint32 *y);
void Package_GPS(uint32 latitude,uint32 longitude,double *x,double *y);
void Package_INS(uint32 latitude,uint32 longitude,double *x,double *y);
void double_to_uint8(void);
void uint8_to_double(void);

extern uint8 reduce [50];

extern uint8 latitude0 [50];
extern uint8 latitude1 [50];
extern uint8 latitude2 [50];
extern uint8 latitude3 [50];

extern uint8 longitude0 [50];
extern uint8 longitude1 [50];
extern uint8 longitude2 [50];
extern uint8 longitude3 [50];

extern uint32 x,y;

extern double  point_data[50][2];
extern uint8 aim_point;
extern uint8 reduce_point;
extern double Distance_Final;
extern double Distance_slow;
extern double Distance_high;
extern double Distance_advance;
extern uint32 Distance_INS;         //…„œÒÕ∑≈–∂®æ‡¿Î
extern double Distance_;
extern float Azimuth;
extern float angle_final;
extern float yaw;
extern float Angle_Z_Final_zero;
extern float variable_speed_servo;
extern double latitude,longitude;
extern double ins_x,ins_y;
extern uint16 n ;
extern uint8 m;
extern uint8 ins_write;
extern uint8 ins_read ;
extern boolean xianshi_flag ;
extern int16 calibration;

extern uint16 Distance_with_ban;
extern uint16 dlla_out[5];
extern uint16 dlla_fianl_out;
extern uint32 INS_forward;
extern uint8  INS_forward_flag;
extern uint16 Distance_with;

#endif
