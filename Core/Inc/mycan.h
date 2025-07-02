#ifndef __MYCAN_H__
#define __MYCAN_H__

#include "stm32h7xx.h"
#include "fdcan.h"
#include "pid.h"
#include "route.h"
#include "return_data_process.h"

typedef struct
{
    uint8_t Wheel_Break; // 车轮刹车命令，0为继续，1分控那边调用刹车函数
    uint8_t Wheel_Lock; // 车轮速度命令，0为继续，1分控那边调用速度函数
    uint8_t Order_2; // 车轮方向命令，0为继续，1分控那边调用方向函数
    uint8_t Order_3; // 车轮方向命令，0为继续，1分控那边调用方向函数
} Order_To_Subcontroller_Struct; // Kinect相机数据结构体

extern Order_To_Subcontroller_Struct Order_To_Subcontroller;   //给分控发送的命令
extern VESC_Data_From_Subcontroller_Struct VESC_Data_From_Subcontroller[4]; // 从分控接收的VESC数据
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
HAL_StatusTypeDef FDCAN_Config_Start(FDCAN_HandleTypeDef *hfdcan);
void CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *pData, Motor_Struct *Motor, uint16_t Len);
void FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t Data[16], Coordinate_Speed_Struct *Speed, Order_To_Subcontroller_Struct *Order);

#endif
