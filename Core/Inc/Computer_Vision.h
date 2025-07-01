#ifndef __COMPUTER__VISION__
#define __COMPUTER__VISION__
#include "stdint.h"
#include "route.h"

typedef struct
{
    float X;    // x位置
    float Y;    // y位置
    float W;    // 车在世界坐标系下的角度（逆时针为正）（左边半个180是正的0~180）（右边半个180是负的0~-180），在180和-180之间会有跳变
} LiDAR_Struct; // 激光雷达数据储存

typedef struct
{
    float X;     // X是相机画面中心线距离篮框的像素点多少
    float Z;     // Z是篮筐深度
} Kinect_Struct; // Kinect相机数据结构体

typedef struct
{
    float X;        // X是世界坐标系下红点的X
    float Y;        // Y是世界坐标系下红点的Y
} RealSense_Struct; // RealSense相机数据结构体

typedef struct
{
    Kinect_Struct Kinect;       // Kinect篮筐识别相机数据
    RealSense_Struct RealSense; // RealSense预选赛地面点位识别相机数据
} Camera_Struct;                // 摄像机数据结构体

// 车身运动状态结构体
typedef struct
{
    float X;     // X方向速度 (mm/s)
    float Y;     // Y方向速度 (mm/s)
    float W;     // 角速度 (°/s)
} Vehicle_Velocity_Struct;

typedef struct
{
    float X;     // X方向加速度 (mm/s²)
    float Y;     // Y方向加速度 (mm/s²)
    float W;     // 角加速度 (°/s²)
} Vehicle_Acceleration_Struct;

// 运动状态估计器结构体
typedef struct
{
    float LIDAR_Speed;                // 激光雷达速度
    float LIDAR_Acceleration;                // 激光雷达加速度
    // 历史位置数据
    LiDAR_Struct last_position;      // 上一次位置
    LiDAR_Struct current_position;   // 当前位置
    
    // 历史速度数据
    Vehicle_Velocity_Struct last_velocity;      // 上一次速度
    Vehicle_Velocity_Struct current_velocity;   // 当前速度
    
    // 当前加速度
    Vehicle_Acceleration_Struct current_acceleration;
    
    // 滤波器（可选）
    float velocity_filter_weight;    // 速度滤波权重
    float accel_filter_weight;       // 加速度滤波权重
    
    // 时间戳和标志
    uint32_t last_timestamp;         // 上次更新时间戳(ms)
    uint32_t current_timestamp;      // 当前时间戳(ms)
    uint8_t initialized;             // 初始化标志
    uint8_t valid_data_count;        // 有效数据计数
} Motion_Estimator_Struct;

typedef struct
{
    // 前两位是校验位
    // Kinect深度Z
    // Kinect画面中心线距离篮框的像素点X
    // RealSense世界坐标系下红点X
    // RealSense世界坐标系下红点Y
    // LiDAR世界坐标系下车身位置X
    // LiDAR世界坐标系下车身位置Y
    // LiDAR世界坐标系下车身角度W
    // 最后两位是校验位
    uint8_t Computer_Vision_Rec_Data[32]; // 数据缓冲区
    LiDAR_Struct LiDAR;                   // 激光雷达数据
    Camera_Struct Camera;                 // 摄像机数据
    Motion_Estimator_Struct Motion;       // 运动状态估计器
} Computer_Vision_Struct;                 // 视觉相关数据结构体

extern Computer_Vision_Struct Computer_Vision_Data;

// 函数声明
void Computer_Vision_Data_Process(Computer_Vision_Struct *Computer_Vision);
void Computer_Vision_Restart(void);

// 运动估计相关函数
void Motion_Estimator_Init(Motion_Estimator_Struct *estimator);
void Motion_Estimator_Update(Motion_Estimator_Struct *estimator, LiDAR_Struct *new_position, uint32_t timestamp);
Vehicle_Velocity_Struct Get_Vehicle_Velocity(Motion_Estimator_Struct *estimator);
Vehicle_Acceleration_Struct Get_Vehicle_Acceleration(Motion_Estimator_Struct *estimator);
float Normalize_Angle_Difference(float angle_diff);

// 新增便捷函数
float Get_Vehicle_Speed(Motion_Estimator_Struct *estimator);           // 获取车身合速度 (mm/s)
float Get_Vehicle_Angular_Speed(Motion_Estimator_Struct *estimator);   // 获取角速度 (°/s)
float Get_Vehicle_Acceleration_Magnitude(Motion_Estimator_Struct *estimator); // 获取加速度大小 (mm/s²)
uint8_t Is_Motion_Data_Valid(Motion_Estimator_Struct *estimator);       // 检查运动数据是否有效

#endif
