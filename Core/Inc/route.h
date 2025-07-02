#ifndef __ROUTE__
#define __ROUTE__
#include "stm32h7xx.h"
#include <stdbool.h>

typedef struct
{
    float X;                  // x位置
    float Y;                  // y位置
    float W;                  // 绕坐标原点旋转角度（车的朝向）
} Coordinate_Position_Struct; // 坐标位置结构体（车的状态结构体）

typedef struct
{
    float Vx;              // x速度
    float Vy;              // y速度
    float Vw;              // 角速度
} Coordinate_Speed_Struct; // 速度结构体

typedef struct
{
    float Rho;
    float Theta;
} Polar_Coordinates_Struct; // 极坐标结构体

typedef enum
{
    Robot_LineType_Cycle_Clockwise,    // 顺时针圆弧
    Robot_LineType_Cycle_Anticlockwise // 逆时针圆弧
} CIRCLE_TYPE_ENUM;

typedef struct
{
    struct // 状态结构体
    {
        FunctionalState Work_Start;      // 标志位，用于判断路径是否结束
        FunctionalState W_Error;         // 只命令车平移时，防止进入最大速度区域
        FunctionalState Tangent;         // 是否沿切线，enable为沿切线
        CIRCLE_TYPE_ENUM Circle_Type;    // 选择顺时针圆弧、逆时针圆弧
        FunctionalState Robot_Direction; // 用于圆弧路径里面车头方向是否一直沿切线，enable为沿切线，disable为不变
    } Flag;

    struct // 坐标系结构体
    {
        Coordinate_Position_Struct Zero_Point;      // 世界坐标系零点（0,0）
        Coordinate_Speed_Struct Zero_Speed;         // 世界坐标系零速度
        Coordinate_Position_Struct Start_Position;  // 启动位置(世界坐标系)
        Coordinate_Position_Struct Target_Position; // 目标位置(世界坐标系)

        Coordinate_Position_Struct Line_Now_Position;    // 线坐标系下当前位置
        Coordinate_Position_Struct Line_Target_Position; // 线坐标系下目标位置
        Coordinate_Speed_Struct Line_Target_V;           // 线坐标系下目标速度

        Coordinate_Position_Struct World_HeartPos;                // 圆心
        Polar_Coordinates_Struct Polar_NowPos;                    // 极坐标系下机器人当前位置（当前位置与圆心连线为极轴）
        Coordinate_Speed_Struct Polar_Target_V;                   // 极坐标系下机器人速度
        Coordinate_Speed_Struct World_Coordinate_System_Target_V; // 世界坐标系下目标速度
        Coordinate_Speed_Struct Robot_Coordinate_System_Target_V; // 车身坐标系下目标速度
        Coordinate_Speed_Struct Robot_Coordinate_System_V;        // 路径计算出的车身速度，用于车轮速度分配
    } Coordinate_System;

    struct // 参数结构体
    {
        float Line_Route_Slow_Area;    // 减速区
        float Line_Route_Stop_Area;    // 停止区
        float Line_Route_Speedup_Area; // 加速区
        float Start_Speed;             // 起始速度
        float Max_Speed;               // 最大速度
        float End_Speed;               // 末端速度

        float Angle_Slow_Area;    // 减速角度区
        float Angle_Stop_Area;    // 停止角度区
        float Angle_Speedup_Area; // 加速角度区
        float Start_W_Speed;      // 启动角速度
        float Max_W_Speed;        // 最大角速度
        float End_W_Speed;        // 末端角速度
        // float Angle_Rotate_Sum;   // 要转的角度（不是目标绝对角度）（时代的眼泪了，码盘时期无法读出绝对角度）

        float Target_Angle_Sum; // 目标累加圆心角
        float Now_Angle_Sum;    // 当前累加圆心角
        float R;                // 半径

        float Speedup_Rate;   // 加速区间比例
        float Slow_Rate;      // 减速区间比例
        float Speedup_W_Rate; // 角速度加速区间比例
        float Slow_W_Rate;    // 角速度减速区间比例
        float Distance;       // 到目标点的距离

        // 新增调试变量
        float Total_Angle_Change;      // 总角度变化量（用于调试）
        float Current_Angle_Error;     // 当前角度误差（带符号，用于调试）
        float Abs_Angle_Error;         // 当前角度绝对误差（用于调试）
        float Angle_Distance_Traveled; // 已转过的角度距离（用于调试）
        
        float Kp;
        float Kd;
    } Parameter;
} Route_STU;

typedef struct
{
    float Max_Speed;
    float End_Speed;
    float Max_w;
    float Up_Rate;
    float Down_Rate;
    float Stop_Area;
    float Kp;
    float Kd;
} Parameter_Of_Route_With_Automatic_Aiming; // 用于Calculate_Nearest_Vision_Point函数，用来给不同距离的不同速度赋值

// 定义路径点结构体，包含所有路径参数
typedef struct
{
    int X;              // X坐标
    int Y;              // Y坐标
    int W;              // 角度
    int start_speed;    // 起始速度
    int max_speed;      // 最大速度
    int end_speed;      // 结束速度
    int min_speed;      // 最小速度
    int max_w;          // 最大角速度
    float up_rate;      // 加速比率
    float down_rate;    // 减速比率
    int stop_area;      // 停止区域
    bool needs_dribble; // 是否需要运球
    float Kp;
    float Kd;
} RoutePoint;

extern Coordinate_Position_Struct World_Coordinate_System_NowPos; // 世界坐标系下当前位置
extern Coordinate_Position_Struct Pre_Basket_Position;            // 预选赛世界坐标系篮筐位置
extern Coordinate_Position_Struct Basket_Position;                // 正赛世界坐标系篮筐位置
extern Route_STU Route_Status; // 路径状态结构体
float Calculate_Line_Distance(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point);
float Calculate_Line_Angle(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point);
void Route_Init(void);
void Nearest_Vision_Point_Route(void);
void Dribble_Pre_Competition(void);
void Check_Near_Vision_Points(uint8_t *flag_variable, float distance_threshold);
void Automatic_Aiming_When_Nonautomatic(void);
float Automatic_Aiming_W_Calculate(uint8_t target_type, float custom_x, float custom_y);
void Keep_Position_Speed(float Target_X, float Target_Y, float Target_W, float Limit_Speed);
// uint8_t Vision_Search_Rotation(float rotation_speed, uint8_t direction_first);
Coordinate_Position_Struct Position_Coordinate_Transformation(Coordinate_Position_Struct *Coordinitioate_To_Convert, Coordinate_Position_Struct *CoordinateSystem, float angle);
Coordinate_Speed_Struct Speed_Coordinate_Transformation(Coordinate_Speed_Struct *Speed_To_Convert, Coordinate_Speed_Struct *Zero_Speed, float angle);
void Chassis_Line_Route(float Target_X, float Target_Y, float Target_W, float Start_Speed, float Max_Speed, float End_Speed, float Start_w, float Max_w, float Up_Rate, float Down_Rate, float Stop_Area, float Kp, float Kd);
void Chassis_Circle_Route(float Target_X, float Target_Y, float Angle, int Start_V, int End_V, int V_Max, FunctionalState Yanqie, float Up_Rate, float Down_Rate, float Kp, float Kd);
#endif
