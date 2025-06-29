#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#define PI 3.14159265358979f
#define CHASSIS_ANGLE_1 -PI/4
typedef struct
{

	float Vx;
	float Vy;
	float Resultant_V;
	float Now_Wheel_Angle;  //轮子当前的角度
	float Angle_Gap;  //轮子与目标速度角度的差值，也就是要转的角度
	float V_Angle; //目标合速度的角度
	float Absolute_Angle;
	float Angle_Sum;
} Chassis_Wheels_Struct;

void Steering_Wheel_Init(void);
void PID_Init_DJI_Motor(void);
void Robot_Wheel_Control(float Vx, float Vy, float Vw);
void Robot_Wheel_speed_Control_VESC(void);
#endif
