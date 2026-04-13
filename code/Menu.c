#include "zf_common_headfile.h"

float flash_data[Flash_Size] = {0};  //Flash数据
int8 Flash_Page = 0;
uint8 grade = 1;    //菜单级数
int16 key = 0;
int8 aim = 0; //箭头位置

char* Mode_;

uint8 Blance_flag = 0;
uint8 cargo = 0;
uint8 car_flag = 1;
uint8 caidian_flag = 1;
uint8 jiesuan_flag = 0;
uint8 stop_flag = 0;
uint8 qingchu = 0;

void Menu(void)
{
    int8 flag_grade = 0;
    /********读flash数据*********/
    flash_buffer_clear();
    if(!flash_check(0, 7))
    {
       flag_grade=1;
    }
        /********如果有数据则将数据提取出来*******/
    if(flag_grade==0)
    {
       flash_read_page_to_buffer (0,6);
       Flash_Page = flash_union_buffer[0].uint8_type;
    }
        /********若没有数据则将原始数据保存到扇区*******/
    else
    {
       flash_union_buffer[0].uint8_type = Flash_Page;
       flash_write_page_from_buffer (0, 6);
    }

    while(1)
    {
        if(grade == 1)
            one_grade();
        else if(grade == 2)
            two_grade();
        if(cargo == 1)
            break;
    }

    if(!qingchu)
       WriteFlashData();

    switch(car_flag)
    {
        case flag_yaokong:
        {
            Velocity_limit_zero = 0;
            break;
        }
        case flag_RTK:
        {
            Velocity_limit = Velocity_limit_zero;
            while (1)
            {
                ips200_show_string(0,0,"Record the point");
                ips200_show_int(140,0,caidian_flag,1);
                key=key_scan();
                if(key == Right_)
                {
                    caidian_flag++;
                    if(caidian_flag > 1)
                    {
                        caidian_flag = 0;
                    }
                }
                else if(key == Left_)
                {
                    caidian_flag--;
                    if(caidian_flag < 0)
                    {
                        caidian_flag = 1;
                    }
                }

                else if(key == Mid_)
                {
                    break;
                }
            }
            ips200_clear();

            if(caidian_flag == 0)
            {
                exist_point();
            }
            else
            {
                caidian(car_flag);
            }
            break;
        }
        case flag_INS:
        {
            ins_write = 1;
            Velocity_limit = Velocity_limit_zero;
            while (1)
            {
                ips200_show_string(0,0,"Record the point");
                ips200_show_int(140,0,caidian_flag,1);
                key=key_scan();
                if(key == Right_)
                {
                    caidian_flag++;
                    if(caidian_flag > 1)
                    {
                        caidian_flag = 0;
                    }
                }
                else if(key == Left_)
                {
                    caidian_flag--;
                    if(caidian_flag < 0)
                    {
                        caidian_flag = 1;
                    }
                }

                else if(key == Mid_)
                {
                    break;
                }
            }
            ips200_clear();

            if (caidian_flag == 0)
            {
                exist_point();
            }
            else
            {
                caidian(car_flag);
            }
            break;
        }
        default:break;
    }

    while (1)
    {
        ips200_show_string(0,0,"Blance?");
        key=key_scan();
        if(key == Mid_)
        {
            break;
        }
    }
    ips200_clear();

    if(car_flag == flag_INS)
    {
        Angle_Z_Final_zero = yaw;
    }

    jiesuan_flag = 1;

    Dynamic_FB_Speed.P = (Velocity_limit_servo - Velocity_limit_strike)/3;

    Blance_flag = 1;

    system_delay_ms(300);

    if(car_flag == flag_INS)
    {
        init_ins(&ins);
    }

    while (1)
    {
        ips200_show_float(80, Line*5, Mid_angle_Y - FB_Balance_Angle.Actual, 2, 1);
        ips200_show_float(80, Line*6, speed_march, 4, 1);
        ips200_show_string(0,0,"Car_go?");
        ips200_show_string(0,Line*2,"servo");
        ips200_show_float(90,Line*2,servo,3,1);
        key=key_scan();
        if(key == Mid_)
        {
            break;
        }
    }
    ips200_clear();

    cargo = 2;

}

void one_grade(void)
{
    aim = 0;
    while(1)
    {
        ips200_show_string(0,Line*0,Mode_);

        ips200_show_string(0,Line,"Page");
        if(Flash_Page == 0)
        {
            ips200_show_string(90,Line,"RTK");
        }
        else if(Flash_Page == 1)
        {
            ips200_show_string(90,Line,"INS");
        }
        else
        {
            ips200_show_int(90,Line,Flash_Page,2);
        }

        ips200_show_string(0,Line*2,"flash_erase_page_all");
        ips200_show_char(200,Line*aim,'<');

       key=key_scan();
       aim_value(2);

       if(key == Right_&&aim == 1)
       {
           Flash_Page++;
           if(Flash_Page>5)
               Flash_Page = 0;
       }
       else if(key == Left_&&aim == 1)
       {
           Flash_Page--;
           if(Flash_Page<0)
               Flash_Page = 5;
       }
       else if(key == Mid_&&aim == 1)
       {
           ReadFlashData();
           grade = 2;
           break;
       }
       else if(key == Mid_&&aim == 2)
       {
           flash_erase_page(0,Flash_Page);
           qingchu = 1;
           grade = 0;
           cargo = 1;
           break;
       }
       else if(key == Mid_&& aim == 0)
       {
           ReadFlashData();
           flash_erase_page(0,6);
           flash_buffer_clear();
           flash_union_buffer[0].uint8_type = Flash_Page;
//           flash_union_buffer[1].uint8_type = car_flag;
           flash_write_page_from_buffer (0, 6);
           grade = 0;
           cargo = 1;
           break;
       }
    }
    ips200_clear();
}

void two_grade(void)
{
    aim = 0;
    while(1)
    {
        ips200_show_string(0,Line*0,Mode_);

        ips200_show_string(0,Line*1,"calibration");
        ips200_show_string(0,Line*2,"Speed_ZE");
        ips200_show_string(0,Line*3,"Speed_SE");
        ips200_show_string(0,Line*4,"Speed_ST");
        ips200_show_string(0,Line*5,"Distance_slow");
        ips200_show_string(0,Line*6,"Distance_high");
        ips200_show_string(0,Line*7,"Distance_ad");
        ips200_show_string(0,Line*8,"variable_servo");
        ips200_show_string(0,Line*9,"Distance_INS");
        ips200_show_string(0,Line*10,"Distance_ban");

        ips200_show_int(130,Line*1,flash_data[0],4);
        ips200_show_float(130,Line*2,flash_data[1],4,1);
        ips200_show_float(130,Line*3,flash_data[2],4,1);
        ips200_show_float(130,Line*4,flash_data[3],4,1);
        ips200_show_float(130,Line*5,flash_data[4],1,3);
        ips200_show_float(130,Line*6,flash_data[5],1,3);
        ips200_show_float(130,Line*7,flash_data[6],2,3);
        ips200_show_float(130,Line*8,flash_data[7],3,1);
        ips200_show_int(130,Line*9,flash_data[8],6);
        ips200_show_int(130,Line*10,flash_data[9],4);

        ips200_show_char(200,Line*aim,'<');

        key=key_scan();
        aim_value(10);

        if(key == Mid_&& aim == 0)
        {
            flash_erase_page(0,6);
            flash_buffer_clear();
            flash_union_buffer[0].uint8_type = Flash_Page;
            flash_write_page_from_buffer (0, 6);
            grade = 0;
            cargo = 1;
            break;
        }


        else if(key == Left_&&aim == 1)
        {
            flash_data[0] -=1;
        }
        else if(key == Right_&&aim == 1)
        {
            flash_data[0] +=1;
        }
        else if(key == Left_&&aim == 2)
        {
            flash_data[1] -=20;
        }
        else if(key == Right_&&aim == 2)
        {
            flash_data[1] +=20;
        }
        else if(key == Left_&&aim == 3)
        {
            flash_data[2] -=20;
        }
        else if(key == Right_&&aim == 3)
        {
            flash_data[2] +=20;
        }
        else if(key == Left_&&aim == 4)
        {
            flash_data[3] -=20;
        }
        else if(key == Right_&&aim == 4)
        {
            flash_data[3] +=20;
        }
        else if(key == Left_&&aim == 5)
        {
            flash_data[4] -=0.01;
        }
        else if(key == Right_&&aim == 5)
        {
            flash_data[4] +=0.01;
        }
        else if(key == Left_&&aim == 6)
        {
            flash_data[5] -=0.01;
        }
        else if(key == Right_&&aim == 6)
        {
            flash_data[5] +=0.01;
        }
        else if(key == Left_&&aim == 7)
        {
            flash_data[6] -=0.01;
        }
        else if(key == Right_&&aim == 7)
        {
            flash_data[6] +=0.01;
        }
        else if(key == Left_&&aim == 8)
        {
            flash_data[7] -=1;
        }
        else if(key == Right_&&aim == 8)
        {
            flash_data[7] +=1;
        }
        else if(key == Left_&&aim == 9)
        {
            flash_data[8] -=1000;
        }
        else if(key == Right_&&aim == 9)
        {
            flash_data[8] +=1000;
        }
        else if(key == Left_&&aim == 10)
        {
            flash_data[9] -=20;
        }
        else if(key == Right_&&aim == 10)
        {
            flash_data[9] +=20;
        }


    }
    ips200_clear();
}

void ReadFlashData(void)
{
    int16 i;
    int8 flag=0;
    /********读flash数据*********/
    if(!flash_check(0, Flash_Page))
        flag=1;
    flash_buffer_clear();

    /********如果有数据则将数据提取出来*******/
    if(flag==0)
    {
       flash_read_page_to_buffer (0,Flash_Page);
       for(i=0;i<Flash_Size;i++)
       {
           flash_data[i] = flash_union_buffer[i].float_type;
       }
    }

    /********若没有数据则将原始数据保存到扇区*******/
    else
    {
        flash_data[0] = calibration;
        flash_data[1] = Velocity_limit_zero;
        flash_data[2] = Velocity_limit_servo;
        flash_data[3] = Velocity_limit_strike;
        flash_data[4] = Distance_slow;
        flash_data[5] = Distance_high;
        flash_data[6] = Distance_advance;
        flash_data[7] = variable_speed_servo;
        flash_data[8] = Distance_INS;
        flash_data[9] = Distance_with;

        for(i=0;i<Flash_Size;i++)
        {
            flash_union_buffer[i].float_type=flash_data[i];
        }
        flash_write_page_from_buffer (0, Flash_Page);
    }
}

void WriteFlashData(void)
{
    int16 i;
    /********擦flash数据*********/
    flash_erase_page(0,Flash_Page);
    flash_buffer_clear();
    for(i=0;i<Flash_Size;i++)
    {
        flash_union_buffer[i].float_type=flash_data[i];
    }
    flash_write_page_from_buffer (0, Flash_Page);

    //将flash内的值取出
    calibration                 = flash_data[0];
    Velocity_limit_zero         = flash_data[1];
    Velocity_limit_servo        = flash_data[2];
    Velocity_limit_strike       = flash_data[3];
    Distance_slow               = flash_data[4];
    Distance_high               = flash_data[5];
    Distance_advance            = flash_data[6];
    variable_speed_servo        = flash_data[7];
    Distance_INS                = flash_data[8];
    Distance_with               = flash_data[9];

}

int16 key_scan(void)    //按键扫描
{
    if(key_up)
    {
        gpio_set_level(P14_3,1);
        system_delay_ms(20);
        if(key_up)
            while(key_up);
        gpio_set_level(P14_3,0);
        return Up_;
    }
    else if(key_down)
    {
        gpio_set_level(P14_3,1);
        system_delay_ms(20);
        if(key_down)
            while(key_down);
        gpio_set_level(P14_3,0);
        return Down_;
    }
    else if(key_left)
    {
        gpio_set_level(P14_3,1);
        system_delay_ms(20);
        if(key_left)
            while(key_left);
        gpio_set_level(P14_3,0);
        return Left_;
    }
    else if(key_right)
    {
        gpio_set_level(P14_3,1);
        system_delay_ms(20);
        if(key_right)
            while(key_right);
        gpio_set_level(P14_3,0);
        return Right_;
    }
    else if(key_mid)
    {
        gpio_set_level(P14_3,1);
        system_delay_ms(20);
        if(key_mid)
            while(key_mid);
        gpio_set_level(P14_3,0);
        return Mid_;
    }
    else
        return 0;
}


void aim_value(uint8 max)  //箭头清除   到第几行输入就为第几
{
    uint8 i,j;
    uint8 last_aim;
    last_aim = aim;
    if(key == Up_)
    {
        aim--;
        if(aim<0)
            aim = max;

    }
    else if(key == Down_)
    {
        aim++;
        if(aim>max)
            aim = 0;
    }
    if(last_aim != aim)
    {
        for (i = last_aim*Line;i<(last_aim+1)*Line;i++)   //清除箭头
           {
               for (j=200;j<208;j++)
                  {
                   ips200_draw_point (j,i, RGB565_BLACK);
                  }
            }
    }
}
