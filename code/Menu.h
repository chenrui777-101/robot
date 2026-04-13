#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#define Up_      1
#define Down_    2
#define Left_    3
#define Right_   4
#define Mid_     5

#define KEY_mode     !gpio_get_level(P22_1)
#define KEY_blance   !gpio_get_level(P22_0)

#define key_down      !gpio_get_level(P20_8)
#define key_left      !gpio_get_level(P20_7)
#define key_right     !gpio_get_level(P20_6)
#define key_mid       !gpio_get_level(P20_2)

#define key_up        !gpio_get_level(P21_6)

#define Line       16

#define Flash_Size 10//41


#define flag_yaokong  1
#define flag_RTK  2
#define flag_INS  3

extern float flash_data[Flash_Size];  //FlashÊý¾Ý

extern int16 key;

extern char* Mode_;

extern uint8 car_flag;
extern uint8 cargo;
extern uint8 Blance_flag;
extern uint8 jiesuan_flag;
extern uint8 stop_flag;

void Menu(void);
void one_grade(void);
void two_grade(void);
void ReadFlashData(void);
void WriteFlashData(void);
int16 key_scan(void);
void aim_value(uint8 max);

#endif

