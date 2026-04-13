#ifndef _INIT_H
#define _INIT_H

void init_all(void);
void motor_int(void);
void blance_gpio_init(void);
void ips_init(void);
void Mid_Zero_init(void);
void Gpio_init(void);
void encoder_init(void);
void PIT_init(void);
void pid_init_highspeed(void);
void yaokong (void);
void Move_filter_init(void);
void TD_init(void);
void ESO_init(void);

void Mode_selection(void);

#endif
