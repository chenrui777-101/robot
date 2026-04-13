#include "zf_common_headfile.h"
uint8 Rx_flag;
uint8 Rx_Count;
uint8 Rx_Buf[200] = {0};
float Rx_Data;
uint8 Deal_Rx_Buf[10] = {0};
uint8 Rx_temp = 0;

char  tail[4] = {0x00, 0x00, 0x80, 0x7f};
float Blance_data[4]={0,0,0,0};

void vofasend (float data_0 , float data_1 , float data_2 , float data_3)
{
    //vofa
    Blance_data[0] = data_0;
    Blance_data[1] = data_1;
    Blance_data[2] = data_2;
    Blance_data[3] = data_3;
    wireless_uart_send_buffer((uint8*)Blance_data,sizeof(Blance_data));
    wireless_uart_send_buffer((uint8*)tail, sizeof(tail));    // 发送帧尾
}

void E34read (void)
{
    if(Rx_flag==1)
    {
        if(Rx_Buf[0] == '=' )
        {

            for(uint8 t=0 ; t<5 ; t++)
            {
                Deal_Rx_Buf[t] = Rx_Buf[t+1];
            }

            Velocity_limit = -1.2*atof((char *)Deal_Rx_Buf);
            Velocity_limit = Velocity_limit/10*10;
            for(uint8 t=0;t<5;t++)
            {
                Deal_Rx_Buf[t] = 0x00;
            }

            for(uint8 t=0 ; t<3 ; t++)
            {
                Deal_Rx_Buf[t] = Rx_Buf[t+6];
            }

            servo = atof((char *)Deal_Rx_Buf);

            for(uint8 t=0;t<3;t++)
            {
                Deal_Rx_Buf[t] = 0x00;
            }
        }

            for(uint8 t=0;t<Rx_Count;t++)
            {
                Rx_Buf[t] = 0x00;
            }
            Rx_Count=0;
            Rx_flag=0;
    }

}

void jeishou (void)  //无线串口数据接收
{
   uart_query_byte(UART_0,&Rx_temp);
   if(Rx_flag == 0)//接收未完成
   {
       if(Rx_temp == '!')
       {
           Rx_flag=1;
       }
       Rx_Buf[Rx_Count]=Rx_temp;
       Rx_Count++;
   }
}
