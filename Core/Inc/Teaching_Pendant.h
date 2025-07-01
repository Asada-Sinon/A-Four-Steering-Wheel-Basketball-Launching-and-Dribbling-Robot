#ifndef __TEACHING__PENDANT__
#define __TEACHING__PENDANT__
#include "stdint.h"
#include "route.h"
#include "Filter.h"

// 手柄路径选择指令,只回传一个uint8_t
typedef enum
{
    Route_Type_0,     // 运球赛：起始点到S&E点      //投球赛：起始点到第一个视觉识别点
    Route_Type_1,     // 运球赛：S&E点到第二个点    //投球赛：视觉识别点到场地标定投球点
    Route_Type_2,     // 运球赛：第二个点到第三个点 //投球赛：场地标定投球点到视觉识别点
    Route_Type_3,     // 运球赛：第三个点到第四个点 //投球赛：第七个场地标定投球点到定点一直投那个点
    Route_Type_4,     // 运球赛：第四个点到第五个点 //投球赛：将操作权还给操作手
    Route_Type_5,     // 运球赛：第五个点到第六个点
    Route_Type_6,     // 运球赛：第六个点到第七个点
    Route_Type_7,     // 运球赛：第七个点到S&E点左边一个点
    Route_Type_8,     // 运球赛，已经完赛机器人回初始点（方便调试）
    Route_Type_Reset, // 回初始点Reset
} Robot_Route_From_Teaching_Pendant_ENUM;

typedef enum
{
    Competition_Mode_None, // 没有竞赛模式，日常调试挂这个挡位
    Competition_Mode_Dribble_Preliminary, // 运球预选赛
    Competition_Mode_Shoot_Preliminary,       // 投球预选赛
    Competition_Mode_Final                    // 正赛
} Competition_Mode_ENUM;

typedef struct
{
    // 这几个开关从左到右
    float Fire; // 三档带回弹，默认是0,Fire-1
    float Automatic_Switch; // 两档,自动-1/手动1切换
    float Automatic_Aiming_Switch; // 两档，自动-1/手动1切换
    float Death; // 三档没回弹,默认是0,Reset-1/Route_Death1

    float switch5; // 旋钮左
    float switch6; // 旋钮右

    uint8_t Route_Type; // 路径选择指令,这个不是从手柄收来的
    Coordinate_Speed_Struct Speed_Data_From_Teaching_Pendant; // 手柄速度数据，经过处理之后的速度数据
    Coordinate_Speed_Struct Joystick_V; // 经过处理之后的数据，直接发送给分控的速度
} remote_control;

// 非线性映射配置结构体
typedef struct
{
    float max_speed_vx;               // X方向最大速度 (mm/s)
    float max_speed_vy;               // Y方向最大速度 (mm/s)
    float max_speed_vw;               // 角度最大速度 (°/s)
    float deadzone_threshold;         // 死区阈值 (0-1范围)
    float nonlinear_power;            // 非线性指数 (推荐1.5-3.0)
    uint8_t enable_smooth_transition; // 是否启用平滑过渡
} NonlinearMapping_Config_t;

// 双重加速度限制器配置结构体 - 加速和减速分离控制
typedef struct
{
    float max_accel_vx; // X方向加速时的最大加速度 (mm/s²) - 限制加速不要过快
    float max_accel_vy; // Y方向加速时的最大加速度 (mm/s²) - 限制加速不要过快
    float max_accel_vw; // 角速度加速时的最大加速度 (°/s²) - 限制加速不要过快

    float max_decel_vx; // X方向减速时的最大加速度 (mm/s²) - 限制减速不要过快
    float max_decel_vy; // Y方向减速时的最大加速度 (mm/s²) - 限制减速不要过快
    float max_decel_vw; // 角速度减速时的最大加速度 (°/s²) - 限制减速不要过快

    float speed_deadzone; // 速度死区阈值，小于此值认为是零 (mm/s)
    float control_period; // 控制周期 (秒)
} Dual_Accel_Limiter_Config_t;

// 集成的手柄处理器结构体 - 重构版
typedef struct
{
    s_LPFilter filter_magnitude; // 幅值专用低通滤波器 - 新增
    s_LPFilter filter_vw;        // 角速度低通滤波器

    // 角度滤波相关
    float last_velocity_angle; // 上次的速度角度
    uint8_t angle_filter_init; // 角度滤波器初始化标志

    NonlinearMapping_Config_t mapping_config; // 非线性映射配置
    Dual_Accel_Limiter_Config_t accel_config; // 双重加速度限制配置

    Coordinate_Speed_Struct last_output; // 上一次输出
    uint8_t initialized;                 // 初始化标志
} Enhanced_Teaching_Pendant_t;


extern remote_control Teaching_Pendant_Data; // 手柄数据结构体
extern remote_control Teaching_Pendant; // 手柄数据结构体
extern uint8_t Teaching_Pendant_buffer[30]; // 手柄数据接收缓冲区

// 函数声明
void Teaching_Pendant_Restart(void);
void HT10A_process(uint8_t buffer[30]);
// 增强版手柄处理函数
void Enhanced_Teaching_Pendant_Init(void);
Coordinate_Speed_Struct Get_Enhanced_Speed_From_Teaching_Pendant(void);

#endif
