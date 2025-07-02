#include "return_data_process.h"
#include "pid.h"

void PID_Calculate_Incremental(PID_Struct *PID, float Measure, float Target)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;

	PID->P_out = PID->Kp * (PID->Error - PID->LastError);
	PID->I_out = PID->Ki * PID->Error;
	PID->D_out = PID->Kd * (PID->Error - PID->LastError - (PID->LastError - PID->LastLastError));
	PID->Output = PID->P_out + PID->I_out + PID->D_out;

	PID->LastLastError = PID->LastError;
	PID->LastError = PID->Error;
}
void PID_Calculate_Positional(PID_Struct *PID, float Measure, float Target)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;

	PID->P_out = PID->Kp * PID->Error;
	PID->I_out = PID->Ki * PID->Error + PID->I_out;
	PID->D_out = PID->Kd * (PID->Error - PID->LastError);
	PID->Output = PID->P_out + PID->I_out + PID->D_out;

	PID->LastLastError = PID->LastError;
	PID->LastError = PID->Error;
}

void Motor_DJI_Angle_PID_Output_Calculate(Motor_Struct *Motor_DJI, int32_t Angle)
{
	PID_Calculate_Positional(&(Motor_DJI->Angle_PID), Motor_DJI->Angle_Sum, Angle);

	if (Motor_DJI->Angle_PID.Output > 10000)
		Motor_DJI->Angle_PID.Output = 10000;
	if (Motor_DJI->Angle_PID.Output < -10000)
		Motor_DJI->Angle_PID.Output = -10000;

	PID_Calculate_Positional(&(Motor_DJI->Speed_PID), Motor_DJI->Now_Speed, Motor_DJI->Angle_PID.Output);

	if (Motor_DJI->Speed_PID.Output > 10000)
		Motor_DJI->Speed_PID.Output = 10000;
	if (Motor_DJI->Speed_PID.Output < -10000)
		Motor_DJI->Speed_PID.Output = -10000;

	if(Motor_DJI->Speed_PID.Error < 10 && Motor_DJI->Speed_PID.Error >-10)
	{
		Motor_DJI->Speed_PID.Output = 0;
	}

	Motor_DJI->Current_Output = Motor_DJI->Speed_PID.Output;
}

void Motor_DJI_Speed_PID_Output_Calculate(Motor_Struct *Motor_DJI, int16_t Speed)
{
	PID_Calculate_Positional(&(Motor_DJI->Speed_PID), Motor_DJI->Now_Speed, Speed);

	if (Motor_DJI->Speed_PID.Output > 10000)
		Motor_DJI->Speed_PID.Output = 10000;
	if (Motor_DJI->Speed_PID.Output < -10000)
		Motor_DJI->Speed_PID.Output = -10000;

	Motor_DJI->Current_Output = Motor_DJI->Speed_PID.Output;
}

void PID_Init_Each(PID_Struct *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
}

void PID_Clear (PID_Struct *PID)
{
	PID->I_out = 0;
	PID->LastError = 0;
	PID->Output = 0;
}
//带前馈的PID计算
void PID_Calculate_Positional_With_Forward(PID_Struct *PID, float Measure, float Target)
{
    PID->Measure = Measure;
    PID->Target = Target;
    PID->Error = Target - Measure;

    // PID三项计算
    PID->P_out = PID->Kp * PID->Error;
    PID->I_out = PID->Ki * PID->Error + PID->I_out;
    PID->D_out = PID->Kd * (PID->Error - PID->LastError);;
    
    // 总输出 = PID输出 + 前馈输出
    PID->Output = PID->P_out + PID->I_out + PID->D_out + (PID->Error > 0 ? PID->Forward : -PID->Forward);

    PID->LastLastError = PID->LastError;
    PID->LastError = PID->Error;
}
