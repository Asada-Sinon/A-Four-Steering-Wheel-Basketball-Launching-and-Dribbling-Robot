#include "return_data_process.h"
#include "stdio.h"
#include "route.h"

//printf重定向
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000
struct __FILE { int handle; /* Add whatever is needed */ };
FILE __stdout;
FILE __stdin;


int fputc( int ch, FILE *f ) {
    if (DEMCR & TRCENA) 
    {
        while (ITM_Port32(0) == 0);
        ITM_Port8(0) = ch;
    }
    return(ch);
}

uint8_t Encoder_Flag = 1;
uint8_t Flag_Of_2006 = 0; // 2006电机数据标志位，第一次接收数据时将其置为1，之后置为0
// 绝对值函数
static int16_t ABS(int16_t num)
{
	return num > 0 ? num : -num;
}

void Motor_DJI_Speed_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[], Motor_Struct *Motor_DJI)
{
	uint8_t num = RxHeader->Identifier & 0x00F;
	Motor_DJI[num - 1].Now_Speed = Data[2] << 8 | Data[3];
}

void Motor_DJI_Angel_Sum_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[], Motor_Struct *Motor_DJI)
{
	uint8_t num = RxHeader->Identifier & 0x00F;
	Motor_DJI[num - 1].Now_Angle = Data[0] << 8 | Data[1];
	Motor_DJI[num - 1].Now_Speed = Data[2] << 8 | Data[3];

	int16_t Angle_Add_1, Angle_Add_2, Angle_Add_Fin = 0;

	if (Motor_DJI[num - 1].Now_Angle > Motor_DJI[num - 1].Angle_Last)
	{
		Angle_Add_1 = Motor_DJI[num - 1].Now_Angle - Motor_DJI[num - 1].Angle_Last;
		Angle_Add_2 = -8191 + Motor_DJI[num - 1].Now_Angle - Motor_DJI[num - 1].Angle_Last;
	}
	else
	{
		Angle_Add_1 = Motor_DJI[num - 1].Now_Angle - Motor_DJI[num - 1].Angle_Last;
		Angle_Add_2 = 8191 - Motor_DJI[num - 1].Now_Angle - Motor_DJI[num - 1].Angle_Last;
	}
	if (ABS(Angle_Add_1) > ABS(Angle_Add_2))
		Angle_Add_Fin = Angle_Add_2;
	else
		Angle_Add_Fin = Angle_Add_1;
	if (Flag_Of_2006 == 0)
	{
		Angle_Add_Fin = 0; // 第一次接收数据时将角度变化量置为0
		Flag_Of_2006 = 1;  // 将标志位置为1，之后不再将角度变化量置为0
	}
	Motor_DJI[num - 1].Angle_Sum += Angle_Add_Fin;
	Motor_DJI[num - 1].Angle_Last = Motor_DJI[num - 1].Now_Angle;
}

void Absolute_Encoder_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[], Motor_Struct *Motor_DJI)
{
	uint32_t Temp = 0;
	Temp |= (Data[6] << 24);
	Temp |= (Data[5] << 16);
	Temp |= (Data[4] << 8);
	Temp |= Data[3];
	Motor_DJI[0].Absolute_Angle = Temp;
	if (Encoder_Flag)
	{
		// 只进一次将angle_last赋为初始值
		Motor_DJI[0].Angle_Last = Motor_DJI[0].Absolute_Angle;
		Encoder_Flag = 0;
	}
	// 转化为360度制
	Motor_DJI[0].Angle_Transform_360 = Motor_DJI[0].Absolute_Angle * 360 / 4095;
	// 将单圈绝对值编码器的值转化为多圈
	if ((Motor_DJI[0].Absolute_Angle - Motor_DJI[0].Angle_Last) > 3000)
	{
		Motor_DJI[0].Lap_Count--;
	}
	if ((Motor_DJI[0].Absolute_Angle - Motor_DJI[0].Angle_Last) < -3000)
	{
		Motor_DJI[0].Lap_Count++;
	}
	// 将多圈的值赋给数组
	for (int i = 0; i < 4; i++)
	{
		Motor_DJI[i].Now_Angle = Motor_DJI[0].Absolute_Angle + 4096 * Motor_DJI[0].Lap_Count;
	}
	for (int i = 0; i < 4; i++)
	{
		Motor_DJI[i].Angle_Last = Motor_DJI[0].Absolute_Angle;
	}
}

void VESC_Data_Receive(uint8_t *Data, VESC_Data_From_Subcontroller_Struct *VESC_Data_Array)
{
	// 检查起始标记是否正确
	if (Data[0] == 0x0A)
	{
		// 获取轮子ID
		uint8_t wheel_id = Data[5];

		// 检查ID是否在有效范围内
		if (wheel_id < 5)
		{
			// 从Data[1]到Data[4]重建转速值并更新对应ID的数据结构
			VESC_Data_Array[wheel_id].RPM_From_Subcontroller = ((int32_t)Data[1] << 24) |
															   ((int32_t)Data[2] << 16) |
															   ((int32_t)Data[3] << 8) |
															   (int32_t)Data[4];
			VESC_Data_Array[wheel_id].ID_Of_Wheel = wheel_id;
		}
	}
}
