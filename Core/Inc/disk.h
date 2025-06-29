#ifndef __DISK_H__
#define __DISK_H__
#include "stdint.h"
#include "route.h"

#define Disk_Error 0
#define Disk_Ok 1
/**
 * @brief 偏航结构体
 * 存储码盘读取的偏航角度信息
 */
typedef struct
{
    float Now_Yaw;                    // 从码盘读取的此时偏航角
    float Last_Yaw;                   // 从码盘读取的上一时刻偏航角
    float Sum_Angle;                  // 计算车身累计旋转角度时,单次累加的累加值
    float Accumulated_Rotation_Angle; // 计算车身累计旋转角度时,累加值之和
    float Reset_Rotation_Angle;       // 重定位或者初始化时车身相对世界坐标系的角度
    float World_Rotation_Angle;       // 每一时刻车身相对世界坐标系角度
} Yaw_Struct;

/**
 * @brief 坐标结构体
 * 存储码盘读取的坐标信息
 */
typedef struct
{
    float RE_X; // 读取码盘(RE)回传的X,Y坐标
    float RE_Y;
    Coordinate_Position_Struct Chassis_Position_From_Disk; // 码盘回传后经过处理的底盘坐标
    Coordinate_Position_Struct Disk_Position;           // 码盘坐标
} Cod_Struct;

/**
 * @brief 编码器结构体
 * 存储编码器的所有数据
 */
typedef struct
{
    uint8_t Rec_Data[32]; // 接收数据缓冲区
    Yaw_Struct Yaw;       // 偏航角度信息
    Cod_Struct Cod;       // 坐标信息
} Disk_Encoder_Struct;

extern Disk_Encoder_Struct Disk_Encoder;
extern uint8_t Disk_State;
void Disk_Encoder_Restart(void);
// void Disk_Encoder_Init(void);
void Disk_Encoder_Data_Process(Disk_Encoder_Struct *Encoder);
float Get_Float_From_4u8(unsigned char *p);

#endif
