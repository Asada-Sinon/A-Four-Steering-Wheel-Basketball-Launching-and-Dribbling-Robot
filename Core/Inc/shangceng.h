#ifndef __SHANGCENG__
#define __SHANGCENG__

#include "stdio.h"
#include "pid.h"

// 定义摄像头角度枚举类型
typedef enum
{
    CAMERA_ANGLE_START, // 相机上电位置
    CAMERA_ANGLE_LOW = -7000, // 看地上点的相机位置
    CAMERA_ANGLE_HIGH = -72000  // 看篮筐的相机位置
} CameraAngleType;

// 定义运球2006角度枚举类型
typedef enum
{
    DRIBBLE_MOTOR_ANGLE_START,           // 放完球回到位置,上电的位置，在右齿条和铝管平齐处
    DRIBBLE_MOTOR_ANGLE_OUT = -627288,   // 运球时候的角度，机构拉到最外面
    DRIBBLE_MOTOR_ANGLE_IN = 160000,     // 放球的角度
    DRIBBLE_MOTOR_ANGLE_MEDIUM = -170000 // 发射的时候不干涉的位置
} DribbleMotorAngleType;                 // 这个是shangceng[0]电机的角度，shangceng[1]的角度直接取反

typedef enum
{
    Trigger_ANGLE_START,     // 扳机上电位置,扳机初始位置在距离白色打印件最远处
    Trigger_ANGLE_LOCK = 8000, // 扳机锁死位置
    Trigger_ANGLE_FIRE = 28000 // 扳机开火位置
} TriggerAngleType;

extern Motor_Struct ShangCeng_motor[4];
extern DribbleMotorAngleType Dribble_Motor_Angle;
extern CameraAngleType Camera_Angle;
extern TriggerAngleType Trigger_Angle;
extern float A1_Command_Pos;
extern float labiaoData;

uint8_t Fire(void);
uint8_t Reload(void);
uint8_t Pass_Ball_From_Dribble_To_Shoot(uint8_t *Flag);
void Calculate_Fire_Position(uint8_t competition_type);
uint8_t Judge_A1_Position(float target_position, float threshold);
void Dribble_Once(void);
void Dribble_Twice(void);
void ShangCeng_Init(void);
void labiao(uint8_t competition_type);

#endif
