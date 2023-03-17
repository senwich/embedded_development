#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "remote_control.h"
#include "PID.h"
#include "main.h"

#define MAX_CHASSIS_RPM 5000.0
#define MAX_CHASSIS_OUT 50000
#define MAX_CHASSIS_IOUT 2000
#define MAX_CHASSIS_SPEED_TMP 1000
#define MAX_CONTROL_1 50000
#define MAX_CONTROL_0 6000
#define MAX_CONTROL_2 7000
//#define MAX_CONTROL_(X) MAX_CONTROL_X
//#define f(x,y) chassis_rc->rc.ch[x]>0?(chassis_receive[y]<MAX_CONTROL_(x)?chassis_receive[y]:MAX_CONTROL_(x)):(chassis_receive[y]>-MAX_CONTROL_(x)?chassis_receive[y]:-MAX_CONTROL_(x));

extern void chassis_init(void);
extern void chassis_ctrl(void);

#endif