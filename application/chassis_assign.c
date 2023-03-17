#include "chassis_assign.h"

const RC_ctrl_t * chassis_rc;//data collected from remote controller
int32_t vx,vy,wz,v[4];

const motor_measure_t * chassis_point[4];//data collected from 4 motors
pid_type_def chassis_pid[4];//data sent as pid packs
const fp32 chassis_K[3] = {10,0,0.5};//pid parameters
fp32 chassis_receive[3]={0};



//get data from the remote controller which sends requires for attributes of motion>>>>translate to v(i) a set>>>
void get_rc(void)
{
	chassis_rc = get_remote_control_point();
	chassis_receive[0]=(fp32)(chassis_rc->rc.ch[1])/0x294*2000;
	chassis_receive[1]=(fp32)(chassis_rc->rc.ch[0])/0x294*2000;
	chassis_receive[2]=(fp32)(chassis_rc->rc.ch[2])/0x294*2000;
	vx =chassis_rc->rc.ch[1]>0?(chassis_receive[0]<MAX_CONTROL_1?chassis_receive[0]:MAX_CONTROL_1):(chassis_receive[0]>-MAX_CONTROL_1?chassis_receive[0]:-MAX_CONTROL_1);
	vy = chassis_rc->rc.ch[0]>0?(chassis_receive[1]<MAX_CONTROL_0?chassis_receive[1]:MAX_CONTROL_0):(chassis_receive[1]>-MAX_CONTROL_0?chassis_receive[1]:-MAX_CONTROL_0);
	wz = chassis_rc->rc.ch[2]>0?(chassis_receive[2]<MAX_CONTROL_2?chassis_receive[2]:MAX_CONTROL_2):(chassis_receive[2]>-MAX_CONTROL_2?chassis_receive[2]:-MAX_CONTROL_2);
	v[0] = -vx + vy + wz;
	v[1] = vx + vy + wz;
	v[2] = vx - vy + wz;
	v[3] = -vx - vy + wz;
}


//init
void chassis_init(void)
{
	for(int8_t i = 0;i < 4;i++)
	{
		PID_init(&chassis_pid[i],PID_POSITION,chassis_K,MAX_CHASSIS_OUT,MAX_CHASSIS_IOUT);
	}
}

//main call function
void chassis_ctrl(void)
{
	get_rc();
	for(int8_t i = 0;i < 4;i++)
	{
		chassis_point[i] = get_chassis_motor_measure_point(i);
		PID_calc(&chassis_pid[i],chassis_point[i]->speed_rpm,v[i]);	
	}
	CAN_cmd_chassis(chassis_pid[0].out,chassis_pid[1].out,chassis_pid[2].out,chassis_pid[3].out);
	for(int8_t i=0;i<0x03;i++)
		usart_printf("%d,%d\r\n",chassis_point[i]->speed_rpm,v[i]);
}