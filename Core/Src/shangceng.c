#include "shangceng.h"
#include "main.h"
#include "Bluetooth.h"
#include "cmsis_os.h"
#include "pid.h"
#include "mycan.h"
#include "Teaching_Pendant.h"
#include "A1_Motor.h"
#include "Computer_Vision.h"
#include "route.h"

#define A1_MOTOR_POSITION_BOTTOM 215		// 丝杠拉到最下面的位置
#define A1_MOTOR_POSITION_TOP -7			// 丝杠拉到最上面扳机扣上的位置
#define A1_MOTOR_POSITION_THREE_POINTE 50 // 丝杠三分线位置

Motor_Struct ShangCeng_motor[4]; // 上层电机，0是运球左侧2006，1是运球右侧2006，2是相机2006，3是扳机3508
// 0是运球左侧2006
// 1是运球右侧2006
// 2是相机2006
// 3是扳机3508

DribbleMotorAngleType Dribble_Motor_Angle; // 运球2006位置
CameraAngleType Camera_Angle;			   // 相机角度
TriggerAngleType Trigger_Angle;			   // 扳机角度
float A1_Command_Pos = 0;				   // 这个变量用于存储A1电机位置命令

void Dribble_Once() // 正赛运球
{
	// 运球之前先等一下停稳球
	osDelay(1000);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // 小气缸推球
	osDelay(300);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	   // 大气缸推球
	osDelay(300);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // 收小气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); // 收大气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  // 气泵开始吸气
}

void Dribble_Twice() // 运球赛运球
{
	// 运球之前先等一下停稳球
	osDelay(1000);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // 小气缸推球
	osDelay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	   // 大气缸推球
	osDelay(300);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // 收小气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); // 收大气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  // 气泵开始吸气
	// 连续两次运球中间间隔时间
	//git test
	// osDelay(1500);
	osDelay(1000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // 小气缸推球
	osDelay(500);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	   // 大气缸推球
	osDelay(300);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // 收小气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET); // 收大气缸
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  // 气泵开始吸气
	// 收完气缸之后等球弹上来再走
	osDelay(300);
}

void ShangCeng_Init()
{
	// 上层电机初始化
	PID_Init_Each(&ShangCeng_motor[0].Speed_PID, 1, 0, 3); // 0是运球左侧2006
	PID_Init_Each(&ShangCeng_motor[1].Speed_PID, 1, 0, 3); // 1是运球右侧2006
	PID_Init_Each(&ShangCeng_motor[2].Speed_PID, 3, 0, 3); // 2是相机2006
	PID_Init_Each(&ShangCeng_motor[3].Speed_PID, 1, 0, 3); // 3是扳机3508
	PID_Init_Each(&ShangCeng_motor[0].Angle_PID, 1, 0, 10); // 0是运球左侧2006
	PID_Init_Each(&ShangCeng_motor[1].Angle_PID, 1, 0, 10); // 1是运球右侧2006
	PID_Init_Each(&ShangCeng_motor[2].Angle_PID, 1, 0, 10); // 2是相机2006
	PID_Init_Each(&ShangCeng_motor[3].Angle_PID, 1, 0, 15); // 3是扳机3508
	Camera_Angle = CAMERA_ANGLE_START;					   // 相机起始位置
	Dribble_Motor_Angle = DRIBBLE_MOTOR_ANGLE_START;	   // 运球2006起始位置
	Trigger_Angle = Trigger_ANGLE_START;				   // 扳机起始位置
	g_a1motor.command.id = 0x00;
	g_a1motor.command.mode = 0x0A;
	g_a1motor.command.kw = 3;
	while(!Fire_Start_Check)
	{//上电让丝杠上升去找光电门
		g_a1motor.command.velocity = -20;
	}
	//A1_Angle_I_Want = A1_MOTOR_POSITION_TOP;// 丝杠拉到最上面扳机扣上的位置
	Trigger_Angle = Trigger_ANGLE_LOCK; // 扳机锁死位置
}

/**
 * @brief  判断A1电机是否接近目标位置,这个函数作为while的判据，没到一直是1，到了就是0
 *			里面是和feedback作差
 * @param  target_position 目标位置
 * @param  threshold 位置误差阈值，默认值为100
 * @return 0=已达到目标位置 1=未达到目标位置
 */
uint8_t Judge_A1_Position(float target_position, float threshold)
{
	// 计算当前位置与目标位置的绝对差值
	float position_error = fabsf(g_a1motor.feedback.Position_Sum - target_position);
	// 判断是否在误差允许范围内
	if (position_error < threshold)
	{
		return 0; // 已达到目标位置
	}
	else
	{
		return 1; // 未达到目标位置
	}
}
/*********************************************************************************
 * @name 	ABSint32
 * @brief   计算int32_t类型的绝对值
 * @param   value 输入的整数值
 * @return  int32_t 返回输入值的绝对值
 *********************************************************************************/
static int32_t ABSint32(int32_t value)
{
	return (value < 0) ? -value : value; // 返回绝对值
}
/*********************************************************************************
 * @name 	Pass_Ball_From_Dribble_To_Shoot
 * @brief   将球从运球装置转移到投球装置上
 * 			调用这个函数的时候，运球装置在外面伸着，调用结束后运球装置在中间位置
 * @param   Flag 球已经到发射装置的标志位
 * @return  uint8_t 1:运球装置已经到中间位置
 *********************************************************************************/
uint8_t Pass_Ball_From_Dribble_To_Shoot(uint8_t *Flag)
{
	int32_t Angle_D1 = ABSint32(ShangCeng_motor[0].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
	int32_t Angle_D2 = ABSint32(ShangCeng_motor[1].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
	while (Angle_D1 > 50 || Angle_D2 > 50)
	{
		Angle_D1 = ABSint32(ShangCeng_motor[0].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
		Angle_D2 = ABSint32(ShangCeng_motor[1].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
		Dribble_Motor_Angle = DRIBBLE_MOTOR_ANGLE_IN;
		osDelay(2);
	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // 小气缸推球
	osDelay(200);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
	osDelay(200);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // 收小气缸
	*Flag = 1;
	Angle_D1 = ABSint32(ShangCeng_motor[0].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
	Angle_D2 = ABSint32(ShangCeng_motor[1].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_IN);
	while (Angle_D1 > 50 || Angle_D2 > 50)
	{
		Angle_D1 = ABSint32(ShangCeng_motor[0].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_MEDIUM);
		Angle_D2 = ABSint32(ShangCeng_motor[1].Angle_Sum) - ABSint32(DRIBBLE_MOTOR_ANGLE_MEDIUM);
		Dribble_Motor_Angle = DRIBBLE_MOTOR_ANGLE_MEDIUM;
		osDelay(2);
	}
	return 1;
}

/*********************************************************************************
 * @name 	Reload
 * @brief   换弹，先将丝杠拉到最上面位置，然后拉到三分线投球的位置
 *********************************************************************************/
uint8_t Reload(void)
{
	while (Judge_A1_Position(A1_MOTOR_POSITION_TOP, 0.1))
	{
		A1_Angle_I_Want = A1_MOTOR_POSITION_TOP;
		Trigger_Angle = Trigger_ANGLE_LOCK;
		osDelay(2);
	}
	//加气缸加速的话，代码写在这里
	// while (Judge_A1_Position(A1_MOTOR_POSITION_THREE_POINTE, 0.1))
	// {
	// 	A1_Angle_I_Want = A1_MOTOR_POSITION_THREE_POINTE;
	// 	osDelay(2);
	// }
	return 1;
}

float labiaoData = 0;
void labiao(uint8_t competition_type)
{
	float target_x, target_y;
	if (competition_type == Competition_Mode_Shoot_Preliminary)
	{
		// 预选赛篮筐
		target_x = Pre_Basket_Position.X;
		target_y = Pre_Basket_Position.Y;
	}
	else if (competition_type == Competition_Mode_Final)
	{
		// 正赛篮筐
		target_x = Basket_Position.X;
		target_y = Basket_Position.Y;
	}
	labiaoData = sqrtf(pow(Computer_Vision_Data.LiDAR.X - target_x, 2) +
					   pow(Computer_Vision_Data.LiDAR.Y - target_y, 2));
}
/*********************************************************************************
 * @name 	Calculate_Fire_Position
 * @brief   计算A1拉丝杠到的位置
 * 			（手操模式的时候直接放到循环换里面跑）（预选赛在开火前跑一下）
 * @param   competition_type 比赛类型(2:预选赛, 3:正赛)
 *********************************************************************************/
void Calculate_Fire_Position(uint8_t competition_type)
{
	float target_x, target_y;
	if (competition_type == Competition_Mode_Shoot_Preliminary)
	{
		// 预选赛篮筐
		target_x = Pre_Basket_Position.X;
		target_y = Pre_Basket_Position.Y;
	}
	else if (competition_type == Competition_Mode_Final)
	{
		// 正赛篮筐
		target_x = Basket_Position.X;
		target_y = Basket_Position.Y;
	}
	float distance = sqrtf(pow(Computer_Vision_Data.LiDAR.X - target_x, 2) +
						   pow(Computer_Vision_Data.LiDAR.Y - target_y, 2));
	A1_Command_Pos = distance; // 这里应该是等于distance的函数，等待去江阴拟合
	A1Motor_distance_mode(&g_a1motor, A1_Command_Pos, 10, 800);
}
/*********************************************************************************
 * @name 	Fire
 * @brief   开火，先将丝杠拉到开火位置，然后解锁轮子
 * @return  uint8_t 1:开火完成
 *********************************************************************************/
extern uint8_t Reload_Flag; // 外部变量，表示是否需要换弹
uint8_t Fire()
{
	Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = 0;
	Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = 0;
	Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = 0; // 停止指令
	// 不是靠这里拉，这里只是防止在开火的时候丝杠没到位就开始发射
	while (Judge_A1_Position(A1_Angle_I_Want, 0.1))
	{
		// 丝杠没到位就直接卡死在这里
		osDelay(2);
	}
	osDelay(500); // 等待丝杠到位
	Trigger_Angle = Trigger_ANGLE_FIRE; // 扳机拉到开火位置
	osDelay(500); // 等待扳机到位
	Reload_Flag = 1;						// 开火后需要换弹
	return 1;								// 开火完成
}
