#ifndef __RETURN_DATA_PROCESS_H__
#define __RETURN_DATA_PROCESS_H__
#include "pid.h"
#include "fdcan.h"
#include "route.h"

typedef struct
{
    int32_t RPM_From_Subcontroller; // 从分控接收的转速
    uint8_t ID_Of_Wheel; // 车轮ID
} VESC_Data_From_Subcontroller_Struct;


void Motor_DJI_Angel_Sum_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[], Motor_Struct *Motor_DJI);
void Absolute_Encoder_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[], Motor_Struct *Motor_DJI);
void Motor_DJI_Speed_Return_Process(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[],Motor_Struct *Motor_DJI);
void VESC_Data_Receive(uint8_t *Data, VESC_Data_From_Subcontroller_Struct *VESC_Data_Array);
#endif
