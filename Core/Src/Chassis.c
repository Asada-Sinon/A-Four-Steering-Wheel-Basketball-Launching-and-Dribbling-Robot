#include "pid.h"
#include "Chassis.h"
#include "VESC.h"
#include "math.h"
#include "mycan.h"

uint8_t Steering_Wheel_Init_Flag = 0;
// 底盘舵向电机
Motor_Struct Chassis_Steering_Motor[4];
// 轮子
Chassis_Wheels_Struct Chassis_Wheels[4];
// CAN1发送数据
extern uint8_t Can_1_Data[8];
// 绝对值函数
static float ABS(float a)
{
    return a > 0 ? a : -a;
}
void Steering_Wheel_Init(void)
{
    while (Steering_Wheel_Init_Flag == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            Chassis_Steering_Motor[i].Target_Angle = 0;
        }
        if(Chassis_Steering_Motor[0].Now_Angle >= -3 && Chassis_Steering_Motor[0].Now_Angle <= 3)
        {
            Steering_Wheel_Init_Flag = 1;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
        }
    }
}
/*********************************************************************************
 * @name    Get_Angle
 * @brief   在车身坐标系下，求出要转的角度
 * @param   sin_num     当前车头方向与目标方向夹角的sin值
 * @param   cos_num     当前车头方向与目标方向夹角的cos值
 *********************************************************************************/
static float Get_Angle(float sin_num, float cos_num)
{
    if (sin_num >= 0 && cos_num >= 0) // 第一象限点
        return asin(sin_num);
    else if (sin_num >= 0 && cos_num <= 0) // 第二象限
        return PI - asin(sin_num);
    else if (sin_num <= 0 && cos_num <= 0) // 第三象限
        return PI - asin(sin_num);
    else if (sin_num <= 0 && cos_num >= 0) // 第四象限
        return 2 * PI + asin(sin_num);
    return 0;
}
/*********************************************************************************
 * @name    Gain_Gap_Angle
 * @brief   求出当前轮子要转的角度
 *********************************************************************************/
void Gain_Gap_Angle(Chassis_Wheels_Struct *wheel_v, Motor_Struct *motor)
{
    float V = wheel_v->Resultant_V;
    wheel_v->Absolute_Angle = motor->Angle_Transform_360;
    wheel_v->Angle_Gap = wheel_v->V_Angle - wheel_v->Absolute_Angle; // 轮接下来要转的角度，目标速度角度－当前舵轮角度，两个量都已是[0,360),结果为负则舵轮顺时针转
    if (wheel_v->Angle_Gap > 270)
    {
        wheel_v->Angle_Gap = -(360 - wheel_v->Angle_Gap);
        V = V;
    }
    else if (wheel_v->Angle_Gap > 180)
    {
        wheel_v->Angle_Gap = -180 + wheel_v->Angle_Gap;
        V = -V;
    }
    else if (wheel_v->Angle_Gap > 90)
    {
        wheel_v->Angle_Gap = -(180 - wheel_v->Angle_Gap);
        V = -V;
    }

    if (wheel_v->Angle_Gap < -270)
    {
        wheel_v->Angle_Gap = (360 + wheel_v->Angle_Gap);
        V = V;
    }
    else if (wheel_v->Angle_Gap < -180)
    {
        wheel_v->Angle_Gap = (180 + wheel_v->Angle_Gap);
        V = -V;
    }

    else if (wheel_v->Angle_Gap < -90)
    {
        wheel_v->Angle_Gap = (180 + wheel_v->Angle_Gap);
        V = -V;
    }
    wheel_v->Resultant_V = V;
}
/*********************************************************************************
 * @name    Resultant_V_Calculate
 * @brief   计算轮子的速度
 * @param   ID          轮子的ID
 * @param   Wheel_Vx0   轮子的自转速度
 * @param   Wheel_Vy0   轮子的自转速度
 * @param   VX          轮子的X方向速度
 * @param   VY          轮子的Y方向速度
 *********************************************************************************/
void Resultant_V_Calculate(int ID, float Wheel_Vx0, float Wheel_Vy0, float VX, float VY)
{
    Chassis_Wheels[ID].Vx = Wheel_Vx0 + VX;
    Chassis_Wheels[ID].Vy = Wheel_Vy0 + VY;
    Chassis_Wheels[ID].Resultant_V = sqrt(Chassis_Wheels[ID].Vx * Chassis_Wheels[ID].Vx + Chassis_Wheels[ID].Vy * Chassis_Wheels[ID].Vy);
    Chassis_Wheels[ID].V_Angle = Get_Angle(Chassis_Wheels[ID].Vx / Chassis_Wheels[ID].Resultant_V, Chassis_Wheels[ID].Vy / Chassis_Wheels[ID].Resultant_V) * 180 / PI;
    Gain_Gap_Angle(&Chassis_Wheels[ID], &Chassis_Steering_Motor[ID]);
}
/*********************************************************************************
 * @name    Robot_Wheel_Control
 * @brief   控制底盘轮子的速度
 * @param   Vx  X方向速度
 * @param   Vy  Y方向速度
 * @param   Vw  自转速度
 *********************************************************************************/
void Robot_Wheel_Control(float Vx, float Vy, float Vw)
{
    Vy *= -1;
    Vw *= -1;
    // 自转速度分配到每个轮子上
    float WheelVx0 = Vw * cos(CHASSIS_ANGLE_1);
    float WheelVy0 = Vw * sin(CHASSIS_ANGLE_1);
    Resultant_V_Calculate(0, WheelVx0, WheelVy0, Vx, Vy);
    Resultant_V_Calculate(1, -WheelVx0, WheelVy0, Vx, Vy);
    Resultant_V_Calculate(2, WheelVx0, -WheelVy0, Vx, Vy);
    Resultant_V_Calculate(3, -WheelVx0, -WheelVy0, Vx, Vy);
    if (!((ABS(Chassis_Wheels[0].Resultant_V) < 20) && (ABS(Chassis_Wheels[1].Resultant_V) < 20) && (ABS(Chassis_Wheels[2].Resultant_V) < 20) && (ABS(Chassis_Wheels[3].Resultant_V) < 20)))
    {
        // 防止keep_position时频繁抖动
        // for (int i = 0; i < 4; i++)
        // {
        //     Chassis_Wheels[i].Angle_Sum += Chassis_Wheels[i].Angle_Gap;
        // }
    }
    for (int i = 0; i < 4; i++)
    {
        Chassis_Steering_Motor[i].Target_Angle = Chassis_Wheels[i].V_Angle * 4095 / 360;
    }
    Robot_Wheel_speed_Control_VESC();
}
/*********************************************************************************
 * @name    Robot_Wheel_speed_Control_VESC
 * @brief   控制底盘轮子的速度
 *********************************************************************************/
void Robot_Wheel_speed_Control_VESC(void)
{
    VESC_Set_RPM(1, Chassis_Wheels[0].Resultant_V, &hfdcan2);//1号轮子
    VESC_Set_RPM(2, Chassis_Wheels[1].Resultant_V, &hfdcan2);//2号轮子
    VESC_Set_RPM(4, -Chassis_Wheels[2].Resultant_V, &hfdcan2);//4号轮子
    VESC_Set_RPM(3, -Chassis_Wheels[3].Resultant_V, &hfdcan2);//3号轮子
}
/*********************************************************************************
 * @name    PID_Init_DJI_Motor
 * @brief   初始化底盘舵向电机PID
 *********************************************************************************/
void PID_Init_DJI_Motor(void)
{
    for (int i = 0; i < 4; i++)
    {
        PID_Init_Each(&Chassis_Steering_Motor[i].Speed_PID, 8, 0.01, 5);
        PID_Init_Each(&Chassis_Steering_Motor[i].Angle_PID, 55, 0, 0);
    }
}
