#ifndef __PID_H__
#define __PID_H__
#include "stm32h7xx.h"
typedef struct
{
    float Target;
    float Measure;
    float Error;
    float LastError;
    float LastLastError;
    float Kp;
    float Ki;
    float Kd;
    float Forward;//前馈
    float P_out;
    float I_out;
    float D_out;
    float Output;
} PID_Struct;

typedef struct
{
    int32_t Target_Angle;
    int16_t Target_Speed;
    int16_t Now_Angle; // 2006多圈累加值（舵轮绝对值编码器使用值）
    int16_t Now_Speed;
    int16_t Angle_Transform_360;
    int32_t Absolute_Angle; // 2006单圈绝对值编码器
    int16_t Angle_Last;
    int32_t Angle_Sum; // 累计角度
    int16_t Lap_Count; // 旋转的圈数
    PID_Struct Speed_PID;
    PID_Struct Angle_PID;
    int16_t Current_Output;
} Motor_Struct; // 电机结构体

void PID_Init_Each(PID_Struct *PID, float Kp, float Ki, float Kd);
void PID_Calculate_Incremental(PID_Struct *PID, float Measure, float Target);
void PID_Calculate_Positional(PID_Struct *PID, float Measure, float Target);
void Motor_DJI_Angle_PID_Output_Calculate(Motor_Struct *Motor_DJI, int32_t Angle);
void Motor_DJI_Speed_PID_Output_Calculate(Motor_Struct *Motor_DJI, int16_t Speed);
void PID_Calculate_Positional_With_Forward(PID_Struct *PID, float Measure, float Target);
void PID_Clear(PID_Struct *PID);

#endif
