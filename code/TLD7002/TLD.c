#include "zf_common_headfile.h"

uint8 flag_tld = 0;
char Tld_xianshi[3];

void Tld (void)
{
    switch(flag_tld)
    {
        case flag_state:
        {
            if(Blance_flag)
            {
                 dot_matrix_screen_show_string("OK ");

            }
            else
            {
                dot_matrix_screen_show_string("NG ");
            }
            break;
        }
        case flag_aim:
        {
            sprintf(Tld_xianshi, "%c%2d",'B',m);
            dot_matrix_screen_show_string(Tld_xianshi);
            break;
        }
        case flag_servo:
        {
            if(differ < 1 && differ > -1)
            {
                dot_matrix_screen_show_string("$  ");
            }
            else if (differ > 1)
            {
                dot_matrix_screen_show_string("&  ");
            }
            else
            {
                dot_matrix_screen_show_string("%  ");
            }
            break;
        }
        default:break;
    }
}

