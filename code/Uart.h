#ifndef CODE_UART_H_
#define CODE_UART_H_

extern uint8 Rx_flag;
extern uint8 Rx_Count;
extern uint8 Rx_Buf[200];
extern float Rx_Data;
extern uint8 Deal_Rx_Buf[10];
extern uint8 Rx_temp;

extern char  tail[4];
extern float Blance_data[4];

void vofasend (float data_0 , float data_1 , float data_2 , float data_3);
void E34read (void);
void jeishou(void);
#endif
