#include "route.h"
#include "pid.h"
#include "math.h"
#include "disk.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Computer_Vision.h"
#include "mycan.h"
#include "Teaching_Pendant.h"
#include "shangceng.h"
#include "A1_Motor.h"
#include <stdint.h>
#include "arm_math.h"

/*===================================================================================================================
                                                操作手须知
1.有关初始位置，运球机构的初始位置在右侧铝管与齿条边缘重合处，扳机初始位置在距离白色打印件最远处
2.在运球预选赛的时候打开自动路径即开启自动跑全程
=====================================================================================================================*/
// 角度制转换为弧度制系数
#define CHANGE_TO_RADIAN (0.01745329251994f)
// 弧度制转换为角度制系数
#define CHANGE_TO_ANGLE (57.29577951308232f)
// 角度限制处理函数，这个是在江阴路径角度出现问题，更改时添加的
#define PI 3.14159265358979f
#define ANGLE_MIN_AREA_PROTECTION 0.01f // 角度区间最小保护值（度）
#define VW_MAX_LIMIT 2000.0f            // Vw最大限制值
#define FITTING_INPUT_MIN 0.0f          // 插值函数输入最小值
#define FITTING_INPUT_MAX 1.0f          // 插值函数输入最大值
#define SMALL_ANGLE_THRESHOLD 1.0f      // 小角度阈值（度），小于此值时使用保守参数

Coordinate_Position_Struct World_Coordinate_System_NowPos; // 世界坐标系下当前位置
Coordinate_Speed_Struct World_Coordinate_System_Target_V;  // 世界坐标系下目标速度
Coordinate_Speed_Struct Robot_Coordinate_System_Target_V;  // 车身坐标系下目标速度
Coordinate_Speed_Struct Robot_Coordinate_System_V;         // 路径计算出的车身速度，用于车轮速度分配
Coordinate_Position_Struct Zero_Point = {0, 0, 0};         // 世界坐标系零点

Route_STU Route_Status; // 路径结构体

PID_Struct Line_AdjustPID; // 直线法向pid调节
PID_Struct Line_StopPID;   // 直线停止区切向pid调节
PID_Struct Line_AnglePID;  // 直线停止区角度pid调节
PID_Struct Cir_AdjustPID;  // 圆弧法向pid调节
PID_Struct Cir_StopPID;    // 圆弧停止区切向pid调节
PID_Struct Cir_AnglePID;   // 圆弧角度pid调节
PID_Struct Keep_X_PID;     // 保持x方向位置
PID_Struct Keep_Y_PID;     // 保持y方向位置
PID_Struct Keep_W_PID;     // 保持角度

float Angle_Aim = 0;    // 目标角度值（绝对角度），中间变量
float Angle_Offset = 0; // 剩余要转的角度，中间变量
float Line_Angle = 0;   // 线坐标系与世界坐标系夹角
float Angle_Dis = 0;    // 角度转动距离，中间变量

extern Disk_Encoder_Struct Disk_Encoder;

/*===================================================================================================================
                                            此部分往下为float保护部分
=====================================================================================================================*/
/*********************************************************************************
 * @name   Safe_fabs
 * @brief  安全的绝对值函数，防止异常输入
 * @param  value: 输入值
 * @retval 安全的绝对值
 *********************************************************************************/
static inline float Safe_fabs(float value)
{
    // 检查输入是否为有效数值
    if (isnan(value) || isinf(value))
    {
        return 0.0f;
    }
    return fabsf(value);
}

/*********************************************************************************
 * @name   Clamp_Float
 * @brief  浮点数限幅函数
 * @param  value: 输入值
 * @param  min_val: 最小值
 * @param  max_val: 最大值
 * @retval 限幅后的值
 *********************************************************************************/
static inline float Clamp_Float(float value, float min_val, float max_val)
{
    if (isnan(value) || isinf(value))
    {
        return 0.0f;
    }
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

/*********************************************************************************
 * @name   Safe_Division
 * @brief  安全除法，防止除零和极小值
 * @param  numerator: 分子
 * @param  denominator: 分母
 * @param  protection_value: 保护值，当分母过小时使用
 * @retval 安全的除法结果
 *********************************************************************************/
static inline float Safe_Division(float numerator, float denominator, float protection_value)
{
    float abs_denominator = Safe_fabs(denominator);

    // 如果分母过小，使用保护值
    if (abs_denominator < protection_value)
    {
        abs_denominator = protection_value;
    }

    return numerator / abs_denominator;
}

/*********************************************************************************
 * @name   Enhanced_Polynomial_Fitting_Function
 * @brief  增强的5次多项式插值函数 - 更平滑的加减速曲线
 * @param  x: 输入比例 (0.0 到 1.0)
 * @retval 插值结果 (0.0 到 1.0)
 *
 * 使用5次多项式: f(x) = 6x^5 - 15x^4 + 10x^3
 * 特点：在0和1处一阶、二阶导数都为0，保证平滑过渡
 *********************************************************************************/
float Enhanced_Polynomial_Fitting_Function(float x)
{
    // 输入范围保护
    x = Clamp_Float(x, FITTING_INPUT_MIN, FITTING_INPUT_MAX);

    // 5次多项式插值: 6x^5 - 15x^4 + 10x^3
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;

    float result = 6.0f * x5 - 15.0f * x4 + 10.0f * x3;

    // 输出范围保护
    return Clamp_Float(result, 0.0f, 1.0f);
}

/*********************************************************************************
 * @name   Power_S_Curve_Fitting_Function
 * @brief  幂次S曲线插值函数 - 可调节的平滑度
 * @param  x: 输入比例 (0.0 到 1.0)
 * @param  power: 幂次参数 (推荐2.0-4.0)
 * @retval 插值结果 (0.0 到 1.0)
 *********************************************************************************/
float Power_S_Curve_Fitting_Function(float x, float power)
{
    // 输入范围保护
    x = Clamp_Float(x, FITTING_INPUT_MIN, FITTING_INPUT_MAX);
    power = Clamp_Float(power, 1.0f, 6.0f); // 限制幂次范围

    // 幂次S曲线: x^power / (x^power + (1-x)^power)
    float x_power = powf(x, power);
    float one_minus_x_power = powf(1.0f - x, power);

    // 防止除零
    float denominator = x_power + one_minus_x_power;
    if (denominator < 1e-6f)
    {
        return x; // 退化为线性
    }

    float result = x_power / denominator;
    return Clamp_Float(result, 0.0f, 1.0f);
}

/*********************************************************************************
 * @name   Safe_Angle_Area_Calculate
 * @brief  安全的角度区间计算，防止除零和极小值问题
 * @param  angle_rotate_sum: 总旋转角度
 * @param  area_rate: 区间比例 (0.0 到 1.0)
 * @param  min_protection: 最小保护值
 * @retval 安全的角度区间大小
 *********************************************************************************/
float Safe_Angle_Area_Calculate(float angle_rotate_sum, float area_rate, float min_protection)
{
    // 输入参数保护
    float abs_angle = Safe_fabs(angle_rotate_sum);
    area_rate = Clamp_Float(area_rate, 0.0f, 1.0f);

    // 计算区间大小
    float area_size = area_rate * abs_angle;

    // 应用最小保护值
    if (area_size < min_protection)
    {
        area_size = min_protection;
    }

    return area_size;
}

/*********************************************************************************
 * @name   Adaptive_Angle_Parameters
 * @brief  自适应角度参数调整 - 根据角度大小动态调整参数
 * @param  angle_rotate_sum: 总旋转角度
 * @param  speedup_rate: 原始加速区比例
 * @param  slow_rate: 原始减速区比例
 * @param  adjusted_speedup_rate: 调整后的加速区比例 (输出)
 * @param  adjusted_slow_rate: 调整后的减速区比例 (输出)
 *********************************************************************************/
void Adaptive_Angle_Parameters(float angle_rotate_sum,
                               float speedup_rate,
                               float slow_rate,
                               float *adjusted_speedup_rate,
                               float *adjusted_slow_rate)
{
    float abs_angle = Safe_fabs(angle_rotate_sum);

    // 小角度时使用更保守的参数
    if (abs_angle < SMALL_ANGLE_THRESHOLD)
    {
        *adjusted_speedup_rate = speedup_rate * 0.5f; // 减少加速区
        *adjusted_slow_rate = slow_rate * 0.5f;       // 减少减速区
    }
    // 中等角度
    else if (abs_angle < 10.0f)
    {
        *adjusted_speedup_rate = speedup_rate * 0.8f;
        *adjusted_slow_rate = slow_rate * 0.8f;
    }
    // 大角度时使用原参数
    else
    {
        *adjusted_speedup_rate = speedup_rate;
        *adjusted_slow_rate = slow_rate;
    }

    // 确保调整后的参数合理
    *adjusted_speedup_rate = Clamp_Float(*adjusted_speedup_rate, 0.01f, 0.5f);
    *adjusted_slow_rate = Clamp_Float(*adjusted_slow_rate, 0.01f, 0.5f);

    // 确保加速区+减速区不超过100%
    float total_rate = *adjusted_speedup_rate + *adjusted_slow_rate;
    if (total_rate > 1.0f)
    {
        float scale = 0.9f / total_rate; // 留10%给恒速区
        *adjusted_speedup_rate *= scale;
        *adjusted_slow_rate *= scale;
    }
}

/*********************************************************************************
 * @name   Safe_Vw_Calculate
 * @brief  安全的角速度计算 - 集成所有保护机制
 * @param  angle_rotate_sum: 总旋转角度
 * @param  angle_dis: 已转过的角度
 * @param  angle_offset: 剩余角度
 * @param  start_w_speed: 起始角速度
 * @param  max_w_speed: 最大角速度
 * @param  speedup_rate: 加速区比例
 * @param  slow_rate: 减速区比例
 * @param  angle_stop_area: 角度停止区
 * @retval 安全限幅后的角速度Vw
 *********************************************************************************/
float Safe_Vw_Calculate(float angle_rotate_sum,
                        float angle_dis,
                        float angle_offset,
                        float start_w_speed,
                        float max_w_speed,
                        float speedup_rate,
                        float slow_rate,
                        float angle_stop_area)
{
    float vw = 0.0f;

    // 输入参数验证
    if (Safe_fabs(angle_rotate_sum) < 0.001f)
    {
        return 0.0f; // 无需旋转
    }

    // 自适应参数调整
    float adjusted_speedup_rate, adjusted_slow_rate;
    Adaptive_Angle_Parameters(angle_rotate_sum, speedup_rate, slow_rate,
                              &adjusted_speedup_rate, &adjusted_slow_rate);

    // 计算安全的角度区间
    float angle_speedup_area = Safe_Angle_Area_Calculate(angle_rotate_sum,
                                                         adjusted_speedup_rate,
                                                         ANGLE_MIN_AREA_PROTECTION);
    float angle_slow_area = Safe_Angle_Area_Calculate(angle_rotate_sum,
                                                      adjusted_slow_rate,
                                                      ANGLE_MIN_AREA_PROTECTION);

    // 确保停止区不会过大
    angle_stop_area = Clamp_Float(angle_stop_area, 0.1f, angle_slow_area * 0.5f);

    // 角速度计算 - 使用增强的插值函数
    if (angle_dis < angle_speedup_area)
    {
        // 加速区
        float ratio = Safe_Division(angle_dis, angle_speedup_area, ANGLE_MIN_AREA_PROTECTION);
        float fitting_value = Enhanced_Polynomial_Fitting_Function(ratio);
        vw = start_w_speed + fitting_value * (max_w_speed - start_w_speed);
    }
    else if (angle_dis >= angle_speedup_area &&
             angle_offset > angle_slow_area &&
             angle_offset <= Safe_fabs(angle_rotate_sum))
    {
        // 恒速区
        vw = max_w_speed;
    }
    else if (angle_offset <= angle_slow_area && angle_offset > angle_stop_area)
    {
        // 减速区
        float ratio = Safe_Division(angle_offset, angle_slow_area, ANGLE_MIN_AREA_PROTECTION);
        float fitting_value = Enhanced_Polynomial_Fitting_Function(ratio);
        vw = fitting_value * max_w_speed;
    }
    else if (angle_offset <= angle_stop_area)
    {
        // 停止区 - 这里应该用PID控制，返回一个小的期望值供PID处理
        vw = 0.0f; // 由外部PID控制器处理
    }

    // 考虑旋转方向
    if (angle_rotate_sum < 0)
    {
        vw *= -1.0f;
    }

    // 最终限幅保护
    vw = Clamp_Float(vw, -VW_MAX_LIMIT, VW_MAX_LIMIT);

    return vw;
}

/*********************************************************************************
 * @name 	ABS
 * @brief	安全的绝对值函数
 * @param	a: 输入值
 * @retval	绝对值
 *********************************************************************************/
static float ABS(float a)
{
    return a > 0 ? a : -a;
}
/*===================================================================================================================
                                            此部分往上为float保护部分
=====================================================================================================================*/

/*********************************************************************************
 * @name 	Calculate_Line_Angle
 * @brief	计算世界坐标系下起始点到终止点连线和X轴的夹角
 * @param	Start_Point:起始点：
 * @param	End_Point:终止点;
 * @retval	两点矢量方向角度 -180~180
 *********************************************************************************/
float Calculate_Line_Angle(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point)
{
    float a = 0.0f;
    float b = 0.0f;

    a = End_Point.Y - Start_Point.Y;
    b = End_Point.X - Start_Point.X;
    // atan2f范围可以包含-180到180
    return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
 * @name 	Calculate_Line_Distance
 * @brief	计算点到点的距离
 * @param	Start_Point 起始点
 * @param	End_Point 结束点
 * @retval   返回两点之间的距离
 *********************************************************************************/
float Calculate_Line_Distance(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point)
{
    float Distant;
    Distant = sqrt((Start_Point.X - End_Point.X) * (Start_Point.X - End_Point.X) + (Start_Point.Y - End_Point.Y) * (Start_Point.Y - End_Point.Y));
    return Distant;
}

/*********************************************************************************
 * @name 	Position_Coordinate_Transformation
 * @brief: 位置坐标系旋转变换
 * @param  Coordinate_To_Convert 要转换的坐标系
 * @param  CoordinateSystem 新的坐标系的原点
 * @param  angle 坐标系夹角，逆时针旋转为正
 * @retval 转换后的坐标系
 *********************************************************************************/
Coordinate_Position_Struct Position_Coordinate_Transformation(Coordinate_Position_Struct *Coordinate_To_Convert, Coordinate_Position_Struct *CoordinateSystem, float angle)
{
    float rad = (float)angle * CHANGE_TO_RADIAN;
    Coordinate_Position_Struct Coordinate_Result;
    Coordinate_Result.Y = +(Coordinate_To_Convert->Y - CoordinateSystem->Y) * arm_cos_f32(rad) - (Coordinate_To_Convert->X - CoordinateSystem->X) * arm_sin_f32(rad);
    Coordinate_Result.X = +(Coordinate_To_Convert->Y - CoordinateSystem->Y) * arm_sin_f32(rad) + (Coordinate_To_Convert->X - CoordinateSystem->X) * arm_cos_f32(rad);
    Coordinate_Result.W = Coordinate_To_Convert->W;
    return Coordinate_Result;
}
/*********************************************************************************
 * @name 	Speed_Coordinate_Transformation
 * @brief: 速度坐标系旋转变换
 * @param  Coordinate_To_Convert 要转换的坐标系
 * @param  CoordinateSystem 新的坐标系的原点
 * @param  angle 坐标系夹角，逆时针旋转为正
 * @retval 转换后的坐标系
 *********************************************************************************/
Coordinate_Speed_Struct Speed_Coordinate_Transformation(Coordinate_Speed_Struct *Coordinate_To_Convert, Coordinate_Speed_Struct *CoordinateSystem, float angle)
{
    double rad = angle * CHANGE_TO_RADIAN;
    Coordinate_Speed_Struct Coordinate_Result;
    Coordinate_Result.Vy = +(Coordinate_To_Convert->Vy - CoordinateSystem->Vy) * arm_cos_f32(rad) - (Coordinate_To_Convert->Vx - CoordinateSystem->Vx) * arm_sin_f32(rad);
    Coordinate_Result.Vx = +(Coordinate_To_Convert->Vy - CoordinateSystem->Vy) * arm_sin_f32(rad) + (Coordinate_To_Convert->Vx - CoordinateSystem->Vx) * arm_cos_f32(rad);
    Coordinate_Result.Vw = Coordinate_To_Convert->Vw;
    return Coordinate_Result;
}
/*********************************************************************************
 * @name 	Fitting_Function
 * @brief: 加速区和减速区的拟合函数
 * @param  x 传入加速区所占的比例，如果是减速区的话传入(1-x),(传入的数在0到1之间)
 *********************************************************************************/
float Fitting_Function(float x)
{
    float V_Rate;
    V_Rate = (sinf(PI * (x - 0.5f)) + 1.0f) / 2.0f;
    return V_Rate;
}

/*********************************************************************************
 * @name 	Stop_Area_Slide_PID_Calculate
 * @brief: 停止区滑行PID计算
 * @param  PID PID结构体指针
 * @param  Target 目标位置
 * @param  Measure 当前测量位置
 * @param  Start 起始位置
 * @retval 返回计算后的PID结构体
 *********************************************************************************/
PID_Struct Stop_Area_Slide_PID_Calculate(PID_Struct *PID, float Target, float Measure, float Start, float Kp_Parameter, float Kd_Parameter)
{
    float Distance = Target - Start;                   // 计算目标位置和当前测量位置的距离
    float Rest_Distance = Target - Measure;            // 计算目标位置和当前测量位置的剩余距离
    float Parameter = fabsf(Rest_Distance / Distance); // 计算剩余距离占总距离的比例,越近越小
    PID->Kp = Parameter * Kp_Parameter + 10;           // 有个5保证至少有值
    PID->Ki = 0;
    PID->Kd = Parameter * Kd_Parameter + 10; // 这里没给D后续效果不好可以考虑给
    return *PID;
}
/*********************************************************************************
 * @name 	Line
 * @brief: 直线路径规划，使用的时候不调用它，调用Chassis_Line_Route
 * @param  Status 路径结构体
 *********************************************************************************/
void Line(Route_STU *Status)
{
    // 局部变量，每次进函数都置为0，为了每次刷新Line_StopPID的值为初始值
    // 名词解释：线坐标系：以起始点为原点，起始点和目标点连线为X轴的坐标系
    // 计算初始参数，一段路径只运行一次
    // 算两点连线与X轴的夹角
    Line_Angle = Calculate_Line_Angle(Status->Coordinate_System.Start_Position, Status->Coordinate_System.Target_Position);
    // 计算线坐标系下的目标位置 ，这个Start_Position是线坐标系下的原点
    Status->Coordinate_System.Line_Target_Position = Position_Coordinate_Transformation(&Status->Coordinate_System.Target_Position, &Status->Coordinate_System.Start_Position, Line_Angle);
    // 此时已经换到线坐标系下，计算到目标点的距离就是X坐标
    Status->Parameter.Distance = Status->Coordinate_System.Line_Target_Position.X;
    // 更新初始角度，当前路径初始角度为上一段路径的目标角度
    Status->Coordinate_System.Start_Position.W = Status->Coordinate_System.Target_Position.W;
    // 计算绝对目标角度，初始角度+要转的角度
    Angle_Aim = Status->Coordinate_System.Start_Position.W + Status->Parameter.Angle_Rotate_Sum;
    Status->Coordinate_System.Target_Position.W = Angle_Aim;
    // 计算路径加速区和减速区大小 (停止区直接给参数)
    Status->Parameter.Line_Route_Slow_Area = Status->Parameter.Slow_Rate * Status->Parameter.Distance;
    Status->Parameter.Line_Route_Speedup_Area = Status->Parameter.Speedup_Rate * Status->Parameter.Distance;
    // 计算角速度的加速区减速区(停止区直接给参数)
    // Status->Parameter.Angle_Slow_Area = Status->Parameter.Slow_W_Rate * fabs(Status->Parameter.Angle_Rotate_Sum);
    // Status->Parameter.Angle_Speedup_Area = Status->Parameter.Speedup_W_Rate * fabs(Status->Parameter.Angle_Rotate_Sum);
    // 使用安全的角度区间计算，防止极小值和除零问题
    Status->Parameter.Angle_Slow_Area = Safe_Angle_Area_Calculate(
        Status->Parameter.Angle_Rotate_Sum,
        Status->Parameter.Slow_W_Rate,
        ANGLE_MIN_AREA_PROTECTION);
    Status->Parameter.Angle_Speedup_Area = Safe_Angle_Area_Calculate(
        Status->Parameter.Angle_Rotate_Sum,
        Status->Parameter.Speedup_W_Rate,
        ANGLE_MIN_AREA_PROTECTION);
    // 清pid避免上一段路径的误差积累
    PID_Clear(&Line_AdjustPID);
    PID_Clear(&Line_StopPID);
    PID_Clear(&Line_AnglePID);
    Status->Flag.Work_Start = ENABLE;
    while (Status->Flag.Work_Start)
    {
        // 更新世界坐标系下坐标，读码盘回传后处理过的值
        World_Coordinate_System_NowPos.X = Computer_Vision_Data.LiDAR.X;
        World_Coordinate_System_NowPos.Y = Computer_Vision_Data.LiDAR.Y;
        World_Coordinate_System_NowPos.W = Computer_Vision_Data.LiDAR.W;
        // 这里注释掉了码盘的回传改用雷达的回传
        //  World_Coordinate_System_NowPos.X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
        //  World_Coordinate_System_NowPos.Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y;
        //  World_Coordinate_System_NowPos.W = Disk_Encoder.Yaw.World_Rotation_Angle;
        //  更新线坐标系下坐标
        Status->Coordinate_System.Line_Now_Position = Position_Coordinate_Transformation(&World_Coordinate_System_NowPos, &Status->Coordinate_System.Start_Position, Line_Angle);
        /*===================================================================================================================
                                                       以下是角速度计算部分
        =====================================================================================================================*/
        // 还需要转的角度和已经转的角度
        Angle_Dis = ABS(World_Coordinate_System_NowPos.W - Status->Coordinate_System.Start_Position.W);
        Angle_Offset = ABS(Angle_Aim - World_Coordinate_System_NowPos.W);
        if ((Angle_Dis + Angle_Offset) > (Status->Parameter.Angle_Rotate_Sum - Status->Coordinate_System.Start_Position.W) &&
            Angle_Dis > Status->Parameter.Angle_Speedup_Area)
        {
            Status->Flag.W_Error = ENABLE;
        }
        else
        {
            Status->Flag.W_Error = DISABLE;
        }
        // 停止区 - 修正PID输入，先判断停止区，为了保持平移角度稳定
        if (Angle_Offset <= Status->Parameter.Angle_Stop_Area || Status->Flag.W_Error)
        {
            // 计算带方向的角度误差
            float angle_error = Angle_Aim - World_Coordinate_System_NowPos.W;
            PID_Calculate_Positional(&Line_AnglePID, angle_error, 0);
            // 添加PID输出限幅，防止角速度过大
            if (Line_AnglePID.Output > 800.0f)
                Line_AnglePID.Output = 800.0f;
            else if (Line_AnglePID.Output < -800.0f)
                Line_AnglePID.Output = -800.0f;
            Status->Coordinate_System.Line_Target_V.Vw = Line_AnglePID.Output;
        }
        // 使用安全除法进行角速度计算
        else if (Angle_Dis < Status->Parameter.Angle_Speedup_Area)
        {
            // 加速区 - 使用安全除法
            float ratio = Safe_Division(Angle_Dis, Status->Parameter.Angle_Speedup_Area, ANGLE_MIN_AREA_PROTECTION);
            ratio = Clamp_Float(ratio, 0.0f, 1.0f); // 确保比例在合理范围内

            Status->Coordinate_System.Line_Target_V.Vw =
                Status->Parameter.Start_W_Speed +
                Fitting_Function(ratio) *
                    (Status->Parameter.Max_W_Speed - Status->Parameter.Start_W_Speed);
        }
        // 修正恒速区判断条件
        else if (Angle_Dis >= Status->Parameter.Angle_Speedup_Area &&
                 Angle_Offset > Status->Parameter.Angle_Slow_Area)
        {
            Status->Coordinate_System.Line_Target_V.Vw = Status->Parameter.Max_W_Speed;
        }
        // 减速区
        else if (Angle_Offset <= Status->Parameter.Angle_Slow_Area &&
                 Angle_Offset > Status->Parameter.Angle_Stop_Area)
        {
            // 使用安全除法
            float ratio = Safe_Division(Angle_Offset, Status->Parameter.Angle_Slow_Area, ANGLE_MIN_AREA_PROTECTION);
            ratio = Clamp_Float(ratio, 0.0f, 1.0f);

            Status->Coordinate_System.Line_Target_V.Vw =
                Fitting_Function(ratio) * Status->Parameter.Max_W_Speed;
        }
        // 顺时针旋转则速度取反（前面都是绝对值计算）
        if (Status->Parameter.Angle_Rotate_Sum < 0)
        {
            Status->Coordinate_System.Line_Target_V.Vw *= -1;
        }

        // 添加最终安全限幅
        Status->Coordinate_System.Line_Target_V.Vw = Clamp_Float(
            Status->Coordinate_System.Line_Target_V.Vw,
            -VW_MAX_LIMIT,
            VW_MAX_LIMIT);

        // 移到函数末尾进行角度清零，避免影响当前计算
        // if (Angle_Offset < 1.0f)  // 增大阈值提高稳定性
        // {
        //     Status->Parameter.Angle_Rotate_Sum = 0;
        // }
        /*===================================================================================================================
                                                       以下是线速度计算部分
        =====================================================================================================================*/
        // 加速区内线速度计算
        if ((Status->Coordinate_System.Line_Now_Position.X) < Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Line_Target_V.Vx =
                Status->Parameter.Start_Speed +
                Fitting_Function(Status->Coordinate_System.Line_Now_Position.X / Status->Parameter.Line_Route_Speedup_Area) *
                    (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
        }
        // 在最大速度区
        else if ((Status->Coordinate_System.Line_Now_Position.X) >= Status->Parameter.Line_Route_Speedup_Area &&
                 (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area))
        {
            Status->Coordinate_System.Line_Target_V.Vx = Status->Parameter.Max_Speed;
        }
        // 减速区内线速度计算
        else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area) &&
                 (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
        {
            Status->Coordinate_System.Line_Target_V.Vx = // 期望速度
                Fitting_Function(((Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X) / Status->Parameter.Line_Route_Slow_Area)) *
                    (Status->Parameter.Max_Speed) +
                Status->Parameter.End_Speed;
        }
        // 在停止区用PID拉到指定位置
        else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
        {
            // 实现滑动PID控制
            Line_StopPID = Stop_Area_Slide_PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Target_Position.X,
                                                         Status->Coordinate_System.Line_Now_Position.X,
                                                         (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area),
                                                         Status->Parameter.Kp, Status->Parameter.Kd);
            PID_Calculate_Positional(&Line_StopPID, Status->Coordinate_System.Line_Now_Position.X, Status->Coordinate_System.Line_Target_Position.X);
            Status->Coordinate_System.Line_Target_V.Vx = Line_StopPID.Output;
        }
        // 法向保持直线
        if (ABS(Status->Coordinate_System.Line_Now_Position.X - Status->Coordinate_System.Line_Target_Position.X) > 5) // 快到直线结尾时不调节法向速度，避免出现轮子抖动
        {
            if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 3)
            {
                PID_Calculate_Positional(&Line_AdjustPID, Status->Coordinate_System.Line_Now_Position.Y, 0);
                // 前馈调节，差太多的话给一个大力拽回来
                if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 1.5f)
                {
                    if (Line_AdjustPID.Output > 0)
                        Line_AdjustPID.Output += 3;
                    else if (Line_AdjustPID.Output < 0)
                        Line_AdjustPID.Output -= 3;
                }
                Status->Coordinate_System.Line_Target_V.Vy = Line_AdjustPID.Output;
            }
        }
        else
            Status->Coordinate_System.Line_Target_V.Vy = 0;
        // // 将线坐标系下的速度转换成世界坐标系下速度
        Status->Coordinate_System.World_Coordinate_System_Target_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.Line_Target_V, &Status->Coordinate_System.Zero_Speed, -Line_Angle);
        // // 将世界坐标系下速度转换成车身坐标系下速度
        Status->Coordinate_System.Robot_Coordinate_System_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.World_Coordinate_System_Target_V, &Status->Coordinate_System.Zero_Speed, Computer_Vision_Data.LiDAR.W);
        // 判断路径是否走完
        // 这里死区由3改为5，原因是雷达本身有数字跳变，为最大限度防止路径卡死
        if (Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X < 10 &&
                Angle_Offset < 3 &&
                VESC_Data_From_Subcontroller[0].RPM_From_Subcontroller < 100 &&
                VESC_Data_From_Subcontroller[1].RPM_From_Subcontroller < 100 &&
                VESC_Data_From_Subcontroller[2].RPM_From_Subcontroller < 100 &&
                VESC_Data_From_Subcontroller[3].RPM_From_Subcontroller < 100 ||
            Teaching_Pendant_Data.Route_Death)
        {
            Status->Flag.Work_Start = DISABLE;
            Status->Parameter.Angle_Rotate_Sum = 0;
            Status->Coordinate_System.Line_Target_V.Vx = 0;
            Status->Coordinate_System.Line_Target_V.Vy = 0;
            Status->Coordinate_System.Line_Target_V.Vw = 0;
            Order_To_Subcontroller.Wheel_Break = 1; // 停止指令
        }
        if (Teaching_Pendant_Data.Reset_Confirm && Teaching_Pendant.Reset_Confirm)
        {
            // 如果手柄上按下了复位键，直接跳出路径，用于球掉的情况
            // 这里没有将路径Disable为了不让轮子锁死，加快Reset时间
            break;
        }
    }
}
/*********************************************************************************
 * @name 	Chassis_Line_Route
 * @brief: 直线路径规划
 * @param  Up_Rate 加速区间比例(输入一个0到1的数)
 * @param  Down_Rate 减速区间比例(输入一个0到1的数)
 * @param  Stop_Area 停止区间大小(正常输入大小)
 *********************************************************************************/
void Chassis_Line_Route(float Target_X, float Target_Y, float Target_W, float Start_Speed, float Max_Speed, float End_Speed, float Start_w, float Max_w, float Up_Rate, float Down_Rate, float Stop_Area, float Kp, float Kd)
{
    Order_To_Subcontroller.Wheel_Break = 0;
    Route_Status.Coordinate_System.Start_Position.X = Computer_Vision_Data.LiDAR.X;
    Route_Status.Coordinate_System.Start_Position.Y = Computer_Vision_Data.LiDAR.Y;
    Route_Status.Coordinate_System.Start_Position.W = Computer_Vision_Data.LiDAR.W;
    // 这里注释掉了码盘的回传改用雷达的回传
    // Route_Status.Coordinate_System.Start_Position.X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
    // Route_Status.Coordinate_System.Start_Position.Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y;
    // Route_Status.Coordinate_System.Start_Position.W = Disk_Encoder.Yaw.World_Rotation_Angle;
    Route_Status.Coordinate_System.Target_Position.X = Target_X;
    Route_Status.Coordinate_System.Target_Position.Y = Target_Y;
    Route_Status.Parameter.Angle_Rotate_Sum = Target_W;

    Route_Status.Parameter.Start_Speed = Start_Speed;
    Route_Status.Parameter.Max_Speed = Max_Speed;
    Route_Status.Parameter.End_Speed = End_Speed;
    Route_Status.Parameter.Start_W_Speed = Start_w;
    Route_Status.Parameter.Max_W_Speed = Max_w;

    Route_Status.Parameter.Speedup_Rate = Up_Rate;
    Route_Status.Parameter.Slow_Rate = Down_Rate;
    Route_Status.Parameter.Speedup_W_Rate = 0.4;
    Route_Status.Parameter.Slow_W_Rate = 0.4;
    Route_Status.Parameter.Line_Route_Stop_Area = Stop_Area;
    Route_Status.Parameter.Angle_Stop_Area = 0.5f;

    Route_Status.Parameter.Kp = Kp;
    Route_Status.Parameter.Kd = Kd;
    Line(&Route_Status);
}

/*********************************************************************************
 * @name 	Circle
 * @brief: 圆弧路径规划，使用的时候不调用它，调用Chassis_Circle_Route
 * @param  Status 路径结构体
 *********************************************************************************/
void Circle(Route_STU *Status)
{
    static float Slope_angle;                            // 极坐标速度坐标系与世界坐标系夹角
    static float Angle_last;                             // 机器人在目标线坐标系下走过的角度、上一时刻的角度
    static float angle_temp_1, angle_temp_2, angle_temp; // 一些计算角度的临时变量
    static float Arc_Remain;                             // 与目标点的距离差值，这个单位是弧长
    static float Arc_Dis;                                // 已经走的弧长
    // 计算初始参数,一段路径只运行一次
    // 更新起始角度
    Status->Coordinate_System.Start_Position.W = Status->Coordinate_System.Target_Position.W;
    Status->Parameter.Distance = ABS(Status->Parameter.Target_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R;
    // 计算减速区大小
    Status->Parameter.Line_Route_Slow_Area = Status->Parameter.Slow_Rate * Status->Parameter.Distance;
    // 计算加速区大小
    Status->Parameter.Line_Route_Speedup_Area = Status->Parameter.Speedup_Rate * Status->Parameter.Distance;
    // PID误差归零
    PID_Clear(&Cir_AdjustPID);
    PID_Clear(&Cir_StopPID);
    PID_Clear(&Cir_AnglePID);
    // 计算路径起始点在极坐标系下的坐标
    Status->Coordinate_System.Polar_NowPos.Rho = Calculate_Line_Distance(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
    Status->Coordinate_System.Polar_NowPos.Theta = Calculate_Line_Angle(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
    // 更新Angle_last
    Angle_last = Status->Coordinate_System.Polar_NowPos.Theta;
    Status->Parameter.Now_Angle_Sum = 0;
    Status->Flag.Work_Start = ENABLE;
    while (Status->Flag.Work_Start)
    {
        // 更新世界坐标系坐标
        World_Coordinate_System_NowPos.X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
        World_Coordinate_System_NowPos.Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y;
        // 更新极坐标
        Status->Coordinate_System.Polar_NowPos.Rho = Calculate_Line_Distance(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
        Status->Coordinate_System.Polar_NowPos.Theta = Calculate_Line_Angle(Status->Coordinate_System.World_HeartPos, World_Coordinate_System_NowPos);
        // 首先计算与目标点的角度差值，计算方法借鉴了M3508无刷电机计算角度的方法
        if (Angle_last < Status->Coordinate_System.Polar_NowPos.Theta) // 一种情况
        {
            angle_temp_1 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last; // 逆时针
            angle_temp_2 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last - 2 * PI;
        }
        else
        {
            angle_temp_1 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last; // 顺时针
            angle_temp_2 = 2 * PI + Status->Coordinate_System.Polar_NowPos.Theta - Angle_last;
        }
        // 无论顺时针转还是逆时针转，都是取小的那个角度
        angle_temp = (ABS(angle_temp_1)) > (ABS(angle_temp_2)) ? angle_temp_2 : angle_temp_1;
        Status->Parameter.Now_Angle_Sum += angle_temp;
        Angle_last = Status->Coordinate_System.Polar_NowPos.Theta; // 当前所在位置极角变为last
        // 再计算与目标点的距离差值
        Arc_Dis = ABS(Status->Parameter.Now_Angle_Sum) * Status->Parameter.R;
        Arc_Remain = ABS((Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R);
        // 如果在加速区
        if (Arc_Dis < Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = // 期望速度
                Status->Parameter.Start_Speed +
                Fitting_Function(Arc_Dis / Status->Parameter.Line_Route_Speedup_Area) *
                    (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
        }
        // 如果没在加速区也没在减速区，切向速度为最大速度
        else if (Arc_Remain >= Status->Parameter.Line_Route_Slow_Area && Arc_Dis >= Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = Status->Parameter.Max_Speed;
        }
        // 如果在减速区内，停止区外
        else if (Arc_Remain >= Status->Parameter.Line_Route_Stop_Area && Arc_Remain < Status->Parameter.Line_Route_Slow_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = // 期望速度
                Fitting_Function(Arc_Remain / Status->Parameter.Line_Route_Slow_Area) *
                (Status->Parameter.Max_Speed);
        }
        // 如果在停止区内，用PID拉到指定位置
        else if (Arc_Remain < Status->Parameter.Line_Route_Stop_Area)
        {
            // 实现滑动PID控制
            Line_StopPID = Stop_Area_Slide_PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Target_Position.X,
                                                         Status->Coordinate_System.Line_Now_Position.X,
                                                         (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area),
                                                         Status->Parameter.Kp, Status->Parameter.Kd);
            PID_Calculate_Positional(&Cir_StopPID, Arc_Remain, 0);
            Status->Coordinate_System.Polar_Target_V.Vx = -Cir_StopPID.Output;
        }
        // 顺时针旋转则切向速度取反(还没测试不知道需不需要)
        if (Status->Flag.Circle_Type == Robot_LineType_Cycle_Clockwise)
        {
            Status->Coordinate_System.Polar_Target_V.Vx *= -1;
        }
        // 如果沿切线，时时刻刻改变车头方向
        if (Status->Flag.Robot_Direction)
        {
            Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W + Status->Parameter.Now_Angle_Sum;
        }
        // 如果不沿切线，车头方向不变
        else
        {
            Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W;
        }
        // PID保持车身角度
        PID_Calculate_Positional(&Cir_AnglePID, Disk_Encoder.Yaw.World_Rotation_Angle, Status->Coordinate_System.Target_Position.W);
        //////Robot_Coordinate_System_V.Vw = Cir_AnglePID.Output;
        // 法向用PID保持在圆上
        PID_Calculate_Positional(&Cir_AdjustPID, Status->Coordinate_System.Polar_NowPos.Rho, Status->Parameter.R);
        Status->Coordinate_System.Polar_Target_V.Vy = -Cir_AdjustPID.Output;
        // 当极坐标速度坐标系法向方向偏差过大时可以尝试去减小切向方向速度(待测试)
        // 计算极坐标速度坐标系与世界坐标系夹角，原先是以y轴为基准，现在以x轴为基准
        Slope_angle = Status->Coordinate_System.Polar_NowPos.Theta - 90.0f;
        // 将极坐标速度坐标系的速度转换成世界坐标系下速度
        World_Coordinate_System_Target_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.Polar_Target_V, &Status->Coordinate_System.Zero_Speed, -Slope_angle);
        // 将世界坐标系下速度转换成车身坐标系下速度
        Robot_Coordinate_System_V = Speed_Coordinate_Transformation(&World_Coordinate_System_Target_V, &Status->Coordinate_System.Zero_Speed, Disk_Encoder.Yaw.World_Rotation_Angle);
        // 判断路径是否走完
        if (Arc_Remain < 5)
        {
            if (Status->Flag.Robot_Direction)
                Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W + Status->Parameter.Target_Angle_Sum;
            else
                Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W;
            Status->Flag.Work_Start = DISABLE;
        }
        // Robot_Wheel_Control(Robot_Coordinate_System_V.Vx, Robot_Coordinate_System_V.Vy, Robot_Coordinate_System_V.Vw);
        osDelay(2);
    }
}

/*********************************************************************************
 * @name 	Chassis_Circle_Route
 * @brief: 圆弧路径规划
 * @param  Target_X 目标X坐标
 * @param  Target_Y 目标Y坐标
 * @param  Angle 圆心角(顺时针为正，逆时针为负)(单位：度)
 * @param  Start_V 起始速度
 * @param  End_V 结束速度
 * @param  V_Max 最大速度
 * @param  Yanqie 是否沿切线(enable为沿切线,disable为方向不变)
 * @param  Up_Rate 加速区间比例(输入一个0到1的数)
 * @param  Down_Rate 减速区间比例(输入一个0到1的数)
 *********************************************************************************/
void Chassis_Circle_Route(float Target_X, float Target_Y, float Angle, int Start_V, int End_V, int V_Max, FunctionalState Yanqie, float Up_Rate, float Down_Rate, float Kp, float Kd)
{
    float Gamma, R, Epsilon, D, C_X, C_Y;
    float Rad = Angle * CHANGE_TO_RADIAN;
    // 这个D是当前点到目标点的距离
    D = sqrt((Target_X - Disk_Encoder.Cod.Chassis_Position_From_Disk.X) * (Target_X - Disk_Encoder.Cod.Chassis_Position_From_Disk.X) +
             (Target_Y - Disk_Encoder.Cod.Chassis_Position_From_Disk.Y) * (Target_Y - Disk_Encoder.Cod.Chassis_Position_From_Disk.Y));
    // 这个R是圆的半径
    R = D / 2.0f / sinf(Rad / 2.0f);
    // 计算圆心坐标
    Gamma = acosf((Target_X - Disk_Encoder.Cod.Chassis_Position_From_Disk.X) / D);
    if ((Target_Y - Disk_Encoder.Cod.Chassis_Position_From_Disk.Y) < 0)
    {
        Gamma = 2 * PI - Gamma;
    }
    if (Angle >= 0) // 顺时针
    {
        Epsilon = Gamma - (PI / 2 - Rad / 2);
        Route_Status.Flag.Circle_Type = Robot_LineType_Cycle_Clockwise; // 顺时针
    }
    else // 逆时针
    {
        Epsilon = Gamma + (PI / 2 - Rad / 2);
        Route_Status.Flag.Circle_Type = Robot_LineType_Cycle_Anticlockwise; // 逆时针
    }
    // 这个C_X和C_Y是圆心坐标
    C_X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X + R * cos(Epsilon);
    C_Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y + R * sin(Epsilon);
    // 起始位置替换为上一段路径的目标角度
    Route_Status.Coordinate_System.Start_Position.X = Route_Status.Coordinate_System.Target_Position.X;
    Route_Status.Coordinate_System.Start_Position.Y = Route_Status.Coordinate_System.Target_Position.Y;
    Route_Status.Coordinate_System.Target_Position.X = Target_X; // 目标位置
    Route_Status.Coordinate_System.Target_Position.Y = Target_Y;

    Route_Status.Parameter.Start_Speed = Start_V; // 开始速度
    Route_Status.Parameter.Max_Speed = V_Max;     // 最大速度
    Route_Status.Parameter.End_Speed = End_V;     // 路径末端速度

    Route_Status.Parameter.Speedup_Rate = Up_Rate;         // 加速区间比例
    Route_Status.Parameter.Slow_Rate = Down_Rate;          // 减速区间比例
    Route_Status.Parameter.Line_Route_Stop_Area = 20;      // 停止区长度
    Route_Status.Flag.Robot_Direction = Yanqie;            // 是否沿切线
    Route_Status.Parameter.Target_Angle_Sum = Angle;       // 圆心角
    Route_Status.Parameter.R = R;                          // 半径
    Route_Status.Coordinate_System.World_HeartPos.X = C_X; // 圆心
    Route_Status.Coordinate_System.World_HeartPos.Y = C_Y;

    Route_Status.Parameter.Kp = Kp;
    Route_Status.Parameter.Kd = Kd;
    Circle(&Route_Status);
}

void Route_Init(void)
{
    PID_Init_Each(&Line_AdjustPID, 8, 0, 10);
    PID_Init_Each(&Line_StopPID, 20, 0, 15);
    PID_Init_Each(&Line_AnglePID, 80, 0, 30);
    PID_Init_Each(&Cir_AdjustPID, 0.5f, 0.0f, 0.0f);
    PID_Init_Each(&Cir_StopPID, 0.5f, 0.0f, 0.0f);
    PID_Init_Each(&Cir_AnglePID, 0.5f, 0.0f, 0.0f);
    PID_Init_Each(&Keep_X_PID, 0.5f, 0.0f, 0.0f);
    PID_Init_Each(&Keep_Y_PID, 0.5f, 0.0f, 0.0f);
    PID_Init_Each(&Keep_W_PID, 0.5f, 0.0f, 0.0f);
}

/*===================================================================================================================
                                              运球预选赛相关
=====================================================================================================================*/
// 定义所有路径点常量
Coordinate_Position_Struct Last_Point;
uint8_t Last_Route = 0;
const RoutePoint PATH_POINTS[] = {
    // Route_Type_0: 起始点到S&E点，长路径
    {2458, -2204, 0, 1500, 10000, 1700, 50, 2000, 0.01f, 0.6f, 100, true, 25, 10},
    // Route_Type_1: S&E点到第二个点，短路径
    {1730, -2976, 0, 1500, 4000, 500, 50, 2000, 0.01f, 0.4f, 200, true, 10, 15},
    // Route_Type_2: 第二个点到第三个点，中路径
    {1741, -4705, 0, 1500, 5000, 500, 50, 2000, 0.01f, 0.4f, 100, true, 10, 13},
    // Route_Type_3: 第三个点到第四个点，中路径
    {3765, -4666, 0, 1500, 6000, 800, 50, 2000, 0.01f, 0.5f, 200, true, 10, 15},
    // Route_Type_4: 第四个点到第五个点，中路径
    {5777, -4648, 0, 1500, 6000, 800, 50, 2000, 0.01f, 0.5f, 200, true, 10, 15},
    // Route_Type_5: 第五个点到第六个点，中路径
    {5745, -2920, 0, 1500, 5000, 500, 50, 2000, 0.01f, 0.4f, 100, true, 10, 13},
    // Route_Type_6: 第六个点到第七个点，短路径
    {4972, -2175, 0, 1500, 4000, 1000, 50, 2000, 0.01f, 0.4f, 200, true, 8, 15},
    // Route_Type_7: 第七个点到S&E点左侧，不运球
    {1000, -2175, 0, 1500, 12000, 1000, 50, 2000, 0.01f, 0.7f, 100, false, 25, 15},
    // Route_Type_8: S&E点左侧到出发点，不运球
    {200, -200, 0, 1500, 5000, 1500, 50, 2000, 0.01f, 0.4f, 100, false, 15, 15}};

/*********************************************************************************
 * @name 	Variable_Element_Route
 * @brief   根据目标点与当前位置的距离和其他条件，动态计算并设置路径参数（如速度、加速度等），实现更平滑和高效的导航。
 * @param   Target_Point 目标坐标点
 *********************************************************************************/
void Variable_Element_Route(Coordinate_Position_Struct *Target_Point)
{
    float Line_Distance = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Target_Point->X, 2) +
                                pow(Computer_Vision_Data.LiDAR.Y - Target_Point->Y, 2));
    Parameter_Of_Route_With_Automatic_Aiming Route_Params;
    // 根据距离调整速度参数
    if (Line_Distance > 3000)
    {
        Route_Params.Max_Speed = 10000;
        Route_Params.End_Speed = 1700;
        Route_Params.Down_Rate = 0.6;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 25;
        Route_Params.Kd = 10;
    }
    else if (Line_Distance > 2000)
    {
        Route_Params.Max_Speed = 6000;
        Route_Params.End_Speed = 800;
        Route_Params.Down_Rate = 0.5;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 200;
        Route_Params.Kp = 10;
        Route_Params.Kd = 15;
    }
    else if (Line_Distance > 1000)
    {
        Route_Params.Max_Speed = 5000;
        Route_Params.End_Speed = 500;
        Route_Params.Down_Rate = 0.4;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 10;
        Route_Params.Kd = 13;
    }
    else
    {
        Route_Params.Max_Speed = 4000;
        Route_Params.End_Speed = 500;
        Route_Params.Down_Rate = 0.4;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 10;
        Route_Params.Kd = 15;
    }
    // 执行路径规划
    Chassis_Line_Route(Target_Point->X, Target_Point->Y, Target_Point->W,
                       1200, Route_Params.Max_Speed, Route_Params.End_Speed,
                       50, Route_Params.Max_w, Route_Params.Up_Rate,
                       Route_Params.Down_Rate, Route_Params.Stop_Area,
                       Route_Params.Kp, Route_Params.Kd);
}

/*********************************************************************************
 * @name 	Dribble_Pre_Competition
 * @brief   运球预选赛函数，调用即可全自动跑完
 *********************************************************************************/
void Dribble_Pre_Competition(void)
{
    // 在运球预选赛，打开自动路径开关即开始跑全自动流程
    if (Teaching_Pendant_Data.Automatic_Switch && Teaching_Pendant.Automatic_Switch)
    {
        // 处理重置确认
        if (Teaching_Pendant.Reset_Confirm && Teaching_Pendant_Data.Reset_Confirm)
        {
            Last_Route = Teaching_Pendant.Route_Type;
            Teaching_Pendant.Route_Type = Route_Type_Reset;
        }
        // 如果需要复位（运球运丢了）
        if (Teaching_Pendant.Route_Type == Route_Type_Reset)
        {
            Teaching_Pendant.Reset_Confirm = 0;
            // 调用最近视觉点路径函数，自动选择最近的视觉点
            Nearest_Vision_Point_Route();
            osDelay(1000);
            Teaching_Pendant.Reset_Confirm = 1;
            Teaching_Pendant.Route_Type = Last_Route;
            Variable_Element_Route(&Last_Point);
            // 适用于运球运丢了的情况
            Dribble_Twice();
        }
        // 正常路径执行
        else if (!Teaching_Pendant_Data.Reset_Confirm)
        {
            // 获取路径类型索引（注意路径类型枚举从0开始）
            uint8_t route_index = Teaching_Pendant.Route_Type;
            // 检查索引有效性
            if (route_index < sizeof(PATH_POINTS) / sizeof(PATH_POINTS[0]))
            {
                const RoutePoint *current = &PATH_POINTS[route_index];
                // 执行路径
                Chassis_Line_Route(
                    current->X, current->Y, current->W, current->start_speed,
                    current->max_speed, current->end_speed, current->min_speed,
                    current->max_w, current->up_rate, current->down_rate,
                    current->stop_area, current->Kp, current->Kd);
                // 如果需要运球
                if (current->needs_dribble)
                {
                    Dribble_Twice();
                }
                // 设置下一路径点
                if (Teaching_Pendant.Route_Type == Route_Type_8)
                {
                    Teaching_Pendant.Route_Type = Route_Type_0;
                    Teaching_Pendant.Automatic_Switch = 0; // 完成所有路径，关闭自动模式
                }
                else
                {
                    Last_Point.X = current->X;
                    Last_Point.Y = current->Y;
                    Last_Point.W = current->W;
                    Teaching_Pendant.Route_Type++; // 切换到下一路径点
                }
            }
        }
    }
}

/*********************************************************************************
 * @name 	Calculate_Nearest_Vision_Point
 * @brief   计算最近的视觉定位点并执行路径规划
 * @details 计算两个预设定位点与当前位置的距离，选择距离最近的点作为目标点，
 *          并根据距离动态设置路径参数执行路径规划
 *********************************************************************************/
void Nearest_Vision_Point_Route(void)
{
    // 定义两个预设定位点的坐标
    const Coordinate_Position_Struct Vision_Point1 = {200, -1996, -90};
    const Coordinate_Position_Struct Vision_Point2 = {7227, -1996, 90};
    // 计算当前位置到两个点的距离
    float Distance_1 = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Vision_Point1.X, 2) +
                             pow(Computer_Vision_Data.LiDAR.Y - Vision_Point1.Y, 2));
    float Distance_2 = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Vision_Point2.X, 2) +
                             pow(Computer_Vision_Data.LiDAR.Y - Vision_Point2.Y, 2));
    // 选择距离较近的点作为目标点
    Coordinate_Position_Struct Target_Point;
    if (Distance_1 < Distance_2)
    {
        Target_Point = Vision_Point1;
    }
    else
    {
        Target_Point = Vision_Point2;
    }
    Variable_Element_Route(&Target_Point);
}

/*===================================================================================================================
                                              投球预选赛相关
=====================================================================================================================*/

Coordinate_Position_Struct Pre_Basket_Position = {3757, -620, 0}; // 预选赛世界坐标系篮筐位置
Coordinate_Position_Struct Basket_Position = {2000, 6000, 0};     // 正赛世界坐标系篮筐位置
Coordinate_Position_Struct Data_For_Automatic_Aiming;             // 算角度的中间变量，X和Y存的是篮筐位置和现在位置的差值，W存的是世界坐标系下目标角度的绝对角度
PID_Struct Automatic_Aiming_PID;                                  // 自瞄pid
/*********************************************************************************
 * @name    Automatic_Aiming_W_Calculate
 * @brief   计算自动瞄准所需的目标角度
 * @details 根据输入参数的不同，可以:
 *          1. 使用比赛类型自动选择预设篮筐位置
 *          2. 使用自定义坐标点作为瞄准目标
 *
 * @param   target_type  目标类型: 2 - 预选赛篮筐, 3 - 正赛篮筐, 其他 - 自定义坐标
 *                       (Competition_Mode_Shoot_Preliminary: 2, Competition_Mode_Final: 3
 * @param   custom_x     当target_type=2时使用的自定义X坐标，否则忽略此参数
 * @param   custom_y     当target_type=2时使用的自定义Y坐标，否则忽略此参数
 * @return  float        计算得到的目标角度(角度制)
 *********************************************************************************/
float Automatic_Aiming_W_Calculate(uint8_t target_type, float custom_x, float custom_y)
{
    float target_x, target_y;
    // 根据target_type选择目标点
    if (target_type == Competition_Mode_Shoot_Preliminary)
    {
        // 预选赛篮筐
        target_x = Pre_Basket_Position.X;
        target_y = Pre_Basket_Position.Y;
    }
    else if (target_type == Competition_Mode_Final)
    {
        // 正赛篮筐
        target_x = Basket_Position.X;
        target_y = Basket_Position.Y;
    }
    else
    {
        // 自定义坐标
        target_x = custom_x;
        target_y = custom_y;
    }
    // 计算相对位置
    // 这里直接用雷达的数据（全局变量）
    float delta_x = target_x - Computer_Vision_Data.LiDAR.X;
    float delta_y = target_y - Computer_Vision_Data.LiDAR.Y;
    // 使用atan2f计算角度，处理所有象限情况
    float angle_rad = atan2f(delta_x, delta_y);
    // 转换为角度制并返回
    return -angle_rad * CHANGE_TO_ANGLE;
}

/*********************************************************************************
 * @name    Automatic_Aiming_When_Nonautomatic
 * @brief   手操模式自动瞄准函数
 * @details 根据传感器数据智能选择瞄准模式：
 *          - 当摄像机数据有效时，优先使用视觉反馈进行精确瞄准
 *          - 当视觉数据无效时，使用位置信息计算目标角度
 * @param   competition_type 比赛类型:2 - 预选赛篮筐, 3 - 正赛篮筐, 其他 - 自定义坐标
 *                           (Competition_Mode_Shoot_Preliminary: 2, Competition_Mode_Final: 3
 * @return  void
 *********************************************************************************/
void Automatic_Aiming_When_Nonautomatic(uint8_t competition_type)
{
    // 基于传感器选择瞄准模式
    if (Computer_Vision_Data.Camera.Kinect.Z < 10000)
    {
        // 视觉模式：使用摄像机数据进行精确瞄准
        // 注：像素坐标系中，向右值增大，目标是将目标居中(值为0)
        PID_Calculate_Positional(&Automatic_Aiming_PID, Computer_Vision_Data.Camera.Kinect.X, 0);
    }
    else
    {
        // 位置模式：使用新的角度计算函数，传入比赛类型参数
        float target_angle = Automatic_Aiming_W_Calculate(competition_type, 0, 0);
        // 控制机器人当前角度接近目标角度
        PID_Calculate_Positional(&Automatic_Aiming_PID, World_Coordinate_System_NowPos.W, target_angle);
    }
    // 将PID输出应用到机器人角速度
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_PID.Output;

    // 可选：限制最大角速度，防止转动过快
    if (Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw > 2000)
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = 2000;
    else if (Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw < -2000)
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = -2000;
}

/*********************************************************************************
 * @name    Check_Near_Vision_Points
 * @brief   判断当前位置是否接近视觉识别点
 *
 * @details 计算当前雷达位置到两个预设视觉点的距离，
 *          如果接近视觉点1，则设置标志为1；
 *          如果接近视觉点2，则设置标志为2；
 *          如果不接近任何点，则设置标志为0
 *
 * @param   flag_variable 需要设置的标志变量指针
 * @param   distance_threshold 判定为"接近"的距离阈值(单位与坐标单位相同)
 * @return  void
 *********************************************************************************/
void Check_Near_Vision_Points(uint8_t *flag_variable, float distance_threshold)
{
    // 定义两个预设的视觉识别点坐标
    const Coordinate_Position_Struct Vision_Point1 = {200, -1996, -90};
    const Coordinate_Position_Struct Vision_Point2 = {7227, -1996, 90};

    // 计算当前雷达位置到两个视觉点的距离
    float distance1 = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Vision_Point1.X, 2) +
                            pow(Computer_Vision_Data.LiDAR.Y - Vision_Point1.Y, 2));

    float distance2 = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Vision_Point2.X, 2) +
                            pow(Computer_Vision_Data.LiDAR.Y - Vision_Point2.Y, 2));

    // 判断是否接近视觉点
    if (distance1 < distance_threshold)
    {
        *flag_variable = 1; // 接近视觉识别点1
    }
    else if (distance2 < distance_threshold)
    {
        *flag_variable = 2; // 接近视觉识别点2
    }
    else
    {
        *flag_variable = 0; // 不接近任何视觉点
    }
}

/*********************************************************************************
 * @name    RotateToAngle
 * @brief   旋转到指定角度，期间检测目标（Vision_Search_Rotation的内置函数）
 * @param   target_angle 目标角度
 * @param   rotation_speed 旋转速度(带方向)
 * @return  uint8_t 1:检测到目标 0:未检测到目标
 *********************************************************************************/
static uint8_t RotateToAngle(float target_angle, float rotation_speed)
{
    // 旋转直到角度差值小于阈值
    while (fabsf(World_Coordinate_System_NowPos.W - target_angle) > 1.0f) // 1.0度阈值
    {
        // 检查视觉数据
        if (Computer_Vision_Data.Camera.RealSense.X != 0 &&
            Computer_Vision_Data.Camera.RealSense.Y != 0)
        {
            // 检测到目标，停止旋转
            Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = 0;
            // 锁住轮子确保能停住不转过，注释掉是因为还没找到在哪里把他置回0
            // Order_To_Subcontroller.Wheel_Break = 1;
            return 1;
        }
        // 设置旋转速度
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = rotation_speed;
        // 短暂延时避免阻塞
        osDelay(2);
    }
    // 到达目标位置，停止旋转
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = 0;
    return 0; // 未检测到目标
}

/*********************************************************************************
 * @name    Vision_Search_Rotation
 * @brief   执行视觉搜索旋转，查找目标
 *
 * @param   rotation_speed 旋转角速度(正值，度/秒)
 * @param   direction_first 初始旋转方向(0:先左转 1:先右转)
 * @return  uint8_t 0:未检测到目标 1:检测到目标
 *********************************************************************************/
uint8_t Vision_Search_Rotation(float rotation_speed, uint8_t direction_first)
{
    // 记录初始角度和目标角度
    float initial_angle = World_Coordinate_System_NowPos.W;
    float left_limit = initial_angle + 50.0f;  // 向左极限
    float right_limit = initial_angle - 50.0f; // 向右极限
    // 根据方向执行第一次旋转
    float speed = direction_first ? -rotation_speed : rotation_speed;
    float target = direction_first ? right_limit : left_limit;
    // 第一次旋转
    if (RotateToAngle(target, speed))
        return 1; // 找到目标
    // 切换方向，执行第二次旋转 (从一个极限到另一个极限)
    speed = -speed;
    target = direction_first ? left_limit : right_limit;
    if (RotateToAngle(target, speed))
        return 1; // 找到目标
    // 返回初始位置
    speed = (World_Coordinate_System_NowPos.W < initial_angle) ? rotation_speed : -rotation_speed;
    if (RotateToAngle(initial_angle, speed))
        return 1; // 找到目标
    return 0;     // 未找到目标
}

extern uint8_t Vision_Point_Flag;         // 到达视觉识别点1时置为1，到达视觉识别点2时置为2，其余时间都是0
uint8_t Vision1_Swing_To_Search_Flag = 0; // 在视觉识别点1左右旋转来寻找角落里的点
uint8_t Vision2_Swing_To_Search_Flag = 0; // 在视觉识别点2左右旋转来寻找角落里的点

/*********************************************************************************
 * @name    No_Point_To_Go
 * @brief   处理无视觉数据情况的行为控制（函数内部while卡死）
 * @details 函数在无视觉回传时会持续执行，每次调用只进行一次摆头搜索，
 *          若搜索不到目标则持续前进，直到检测到视觉数据
 *********************************************************************************/
void No_Point_To_Go(void)
{
    // 后续优化考虑在超过某个X的时候执行别的逻辑，一直看不到点的话会直接撞墙
    uint8_t Flag = 0; // 保证在调用一次No_Point_To_Go函数的时候不会摆两次头
    while (Computer_Vision_Data.Camera.RealSense.X == 0 &&
           Computer_Vision_Data.Camera.RealSense.Y == 0)
    {
        // 这里不用return因为在有视觉的数的时候就直接跳出了，不需要return也能达到同等效力
        //  确保在每个视觉识别点都只摆头两次
        //  根据当前位置执行一次摆头搜索（每次函数调用）
        if (!Flag)
        {
            // 根据视觉点标志选择相应的搜索策略
            if (Vision_Point_Flag == 1 && Vision1_Swing_To_Search_Flag < 2)
            {
                Vision_Search_Rotation(800, Vision1_Swing_To_Search_Flag);
                Vision1_Swing_To_Search_Flag++;
                Flag = 1;
            }
            else if (Vision_Point_Flag == 2 && Vision2_Swing_To_Search_Flag < 2)
            {
                Vision_Search_Rotation(800, Vision2_Swing_To_Search_Flag);
                Vision2_Swing_To_Search_Flag++;
                Flag = 1;
            }
        }
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = 1000;
        osDelay(2);
    }
}

/*********************************************************************************
 * @name 	Line_Route_With_Changing_Target
 * @brief: 带目标点变化的直线路径规划（在这套代码里面是将摄像头识别的位置不断地赋给Target）（局限性 -> 只能适用于现在视觉识别的情况）
 *         这个函数难点在于目标位置在不断刷新，需要滤波，且参数滑动，但参数不能不断刷
 * @param  Start_Speed 起始的线速度（重要的参数都在循环里面不断刷，这三个其实随便给了）
 * @param  Start_w 起始的角速度
 * @param  Status 路径结构体
 *********************************************************************************/
void Line_Route_With_Changing_Target(float Start_Speed, float Start_w, Route_STU *Status)
{
    // 因为要不断刷Target点，所以这个函数入参里面把Target去掉了（函数入参只在函数调用的时候入一次），现在的Target点在下面的while里面用全局变量不断刷
    // 带来局限性 -> 只能适用于现在视觉识别的情况
    uint8_t Check_Flag = 0; // 用于判断小电脑刚开始回传数据是否准确（刚开始刷三遍数据标志位）
    float Line_Distance = 0;
    Coordinate_Position_Struct Now_Target_Position;
    Coordinate_Position_Struct Last_Target_Position;
    Coordinate_Position_Struct Last_Last_Target_Position;
    Order_To_Subcontroller.Wheel_Break = 0;
    Status->Coordinate_System.Start_Position.X = Computer_Vision_Data.LiDAR.X;
    Status->Coordinate_System.Start_Position.Y = Computer_Vision_Data.LiDAR.Y;
    Status->Coordinate_System.Start_Position.W = Computer_Vision_Data.LiDAR.W;
    // 名词解释：线坐标系：以起始点为原点，起始点和目标点连线为X轴的坐标系
    // 清pid避免上一段路径的误差积累
    PID_Clear(&Line_AdjustPID);
    PID_Clear(&Line_StopPID);
    PID_Clear(&Line_AnglePID);
    Status->Flag.Work_Start = ENABLE;
    while (Status->Flag.Work_Start)
    {
        // 为了确保第一个刷上的Target点一定偏差不大，取三次点作差
        if (Last_Last_Target_Position.X != 0 &&
            Last_Last_Target_Position.Y != 0 &&
            Last_Target_Position.X != 0 &&
            Last_Target_Position.Y != 0 &&
            (Now_Target_Position.X - Last_Target_Position.X) < 100 &&
            (Now_Target_Position.Y - Last_Target_Position.Y) < 100 &&
            (Now_Target_Position.X - Last_Last_Target_Position.X) < 100 &&
            (Now_Target_Position.Y - Last_Last_Target_Position.Y) < 100 &&
            !Check_Flag)
        {
            Check_Flag = 1;
            // 这个Line_Distance只在开始算一次，用以确定滑动区域参数
            Line_Distance = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Now_Target_Position.X, 2) +
                                  pow(Computer_Vision_Data.LiDAR.Y - Now_Target_Position.Y, 2));
        }
        // 不断刷这个三个点的数据
        Last_Last_Target_Position = Last_Target_Position;
        Last_Target_Position = Now_Target_Position;
        // 点的位置信息在这里用全局变量刷
        Now_Target_Position.X = Computer_Vision_Data.Camera.RealSense.X;
        Now_Target_Position.Y = Computer_Vision_Data.Camera.RealSense.Y;
        if (Check_Flag)
        {
            if ((Now_Target_Position.X - Status->Coordinate_System.Target_Position.X) < 100 &&
                (Now_Target_Position.Y - Status->Coordinate_System.Target_Position.Y) < 100)
            {
                // 由于是变终点路径，所以需要不断刷Target，不断算出新的各个区域长度
                Status->Coordinate_System.Target_Position.X = Computer_Vision_Data.Camera.RealSense.X;
                Status->Coordinate_System.Target_Position.Y = Computer_Vision_Data.Camera.RealSense.Y;
                Status->Parameter.Angle_Rotate_Sum = Automatic_Aiming_W_Calculate(Competition_Mode_Shoot_Preliminary, 0, 0);

                Status->Parameter.Start_Speed = Start_Speed;
                Status->Parameter.Start_W_Speed = Start_w;
                Status->Parameter.Speedup_W_Rate = 0;
                Status->Parameter.Slow_W_Rate = 0.3;
                Status->Parameter.Angle_Stop_Area = 0.5f;
                // 根据距离调整速度参数
                if (Line_Distance > 3000)
                {
                    Status->Parameter.Max_Speed = 11000;
                    Status->Parameter.End_Speed = 500;
                    Status->Parameter.Slow_Rate = 0.5;
                    Status->Parameter.Max_W_Speed = 2000;
                    Status->Parameter.Speedup_Rate = 0.01;
                    Status->Parameter.Line_Route_Stop_Area = 100;
                    Status->Parameter.Kp = 15;
                    Status->Parameter.Kd = 15;
                }
                else if (Line_Distance > 2000)
                {
                    Status->Parameter.Max_Speed = 7000;
                    Status->Parameter.End_Speed = 500;
                    Status->Parameter.Slow_Rate = 0.5;
                    Status->Parameter.Max_W_Speed = 2000;
                    Status->Parameter.Speedup_Rate = 0.01;
                    Status->Parameter.Line_Route_Stop_Area = 100;
                    Status->Parameter.Kp = 15;
                    Status->Parameter.Kd = 15;
                }
                else if (Line_Distance > 1000)
                {
                    Status->Parameter.Max_Speed = 6000;
                    Status->Parameter.End_Speed = 500;
                    Status->Parameter.Slow_Rate = 0.4;
                    Status->Parameter.Max_W_Speed = 2000;
                    Status->Parameter.Speedup_Rate = 0.01;
                    Status->Parameter.Line_Route_Stop_Area = 100;
                    Status->Parameter.Kp = 15;
                    Status->Parameter.Kd = 15;
                }
                else
                {
                    Status->Parameter.Max_Speed = 5000;
                    Status->Parameter.End_Speed = 800;
                    Status->Parameter.Slow_Rate = 0.4;
                    Status->Parameter.Max_W_Speed = 2000;
                    Status->Parameter.Speedup_Rate = 0.01;
                    Status->Parameter.Line_Route_Stop_Area = 100;
                    Status->Parameter.Kp = 15;
                    Status->Parameter.Kd = 15;
                }
                // 算两点连线与X轴的夹角
                Line_Angle = Calculate_Line_Angle(Status->Coordinate_System.Start_Position, Status->Coordinate_System.Target_Position);
                // 计算线坐标系下的目标位置 ，这个Start_Position是线坐标系下的原点
                Status->Coordinate_System.Line_Target_Position = Position_Coordinate_Transformation(&Status->Coordinate_System.Target_Position, &Status->Coordinate_System.Start_Position, Line_Angle);
                // 此时已经换到线坐标系下，计算到目标点的距离就是X坐标
                Status->Parameter.Distance = Status->Coordinate_System.Line_Target_Position.X;
                // 更新初始角度，当前路径初始角度为上一段路径的目标角度
                Status->Coordinate_System.Start_Position.W = Status->Coordinate_System.Target_Position.W;
                // 计算绝对目标角度，初始角度+要转的角度
                Angle_Aim = Status->Coordinate_System.Start_Position.W + Status->Parameter.Angle_Rotate_Sum;
                Status->Coordinate_System.Target_Position.W = Angle_Aim;
                // 计算路径加速区和减速区大小 (停止区直接给参数)
                Status->Parameter.Line_Route_Slow_Area = Status->Parameter.Slow_Rate * Status->Parameter.Distance;
                Status->Parameter.Line_Route_Speedup_Area = Status->Parameter.Speedup_Rate * Status->Parameter.Distance;
                // 计算角速度的加速区减速区(停止区直接给参数)
                Status->Parameter.Angle_Slow_Area = Status->Parameter.Slow_W_Rate * Status->Parameter.Angle_Rotate_Sum;
                Status->Parameter.Angle_Speedup_Area = Status->Parameter.Speedup_W_Rate * Status->Parameter.Angle_Rotate_Sum;
            }
            // 更新世界坐标系下坐标，读码盘回传后处理过的值
            World_Coordinate_System_NowPos.X = Computer_Vision_Data.LiDAR.X;
            World_Coordinate_System_NowPos.Y = Computer_Vision_Data.LiDAR.Y;
            World_Coordinate_System_NowPos.W = Computer_Vision_Data.LiDAR.W;
            // 这里注释掉了码盘的回传改用雷达的回传
            //  World_Coordinate_System_NowPos.X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
            //  World_Coordinate_System_NowPos.Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y;
            //  World_Coordinate_System_NowPos.W = Disk_Encoder.Yaw.World_Rotation_Angle;
            //  更新线坐标系下坐标
            Status->Coordinate_System.Line_Now_Position = Position_Coordinate_Transformation(&World_Coordinate_System_NowPos, &Status->Coordinate_System.Start_Position, Line_Angle);
            /*===================================================================================================================
                                                           以下是角速度计算部分
            =====================================================================================================================*/
            // 还需要转的角度和已经转的角度
            Angle_Offset = ABS(Angle_Aim - World_Coordinate_System_NowPos.W);
            Angle_Dis = ABS(World_Coordinate_System_NowPos.W - Status->Coordinate_System.Start_Position.W);
            // 加速区角速度计算
            if (Angle_Dis < Status->Parameter.Angle_Speedup_Area)
            {
                Status->Coordinate_System.Line_Target_V.Vw =
                    Status->Parameter.Start_W_Speed +                                      // 初速度
                    Fitting_Function(Angle_Dis / Status->Parameter.Angle_Speedup_Area) *   // 加速比例（已经转过的角度/加速区角度）（因为Fitting_Function只能输入一个0到1的数，所以要把已经转过的角度转换成0到1的比例）
                        (Status->Parameter.Max_W_Speed - Status->Parameter.Start_W_Speed); // 最大速度减去最小速度
            }
            // 在最大速度区
            else if (Angle_Dis >= Status->Parameter.Angle_Speedup_Area && Angle_Offset > Status->Parameter.Angle_Slow_Area && Angle_Offset <= Status->Parameter.Angle_Rotate_Sum)
            {
                Status->Coordinate_System.Line_Target_V.Vw = Status->Parameter.Max_W_Speed;
            }
            // 如果在减速区内停止区外
            else if (Angle_Offset <= Status->Parameter.Angle_Slow_Area && Angle_Offset > Status->Parameter.Angle_Stop_Area)
            {
                Status->Coordinate_System.Line_Target_V.Vw =
                    Fitting_Function((Angle_Offset / Status->Parameter.Angle_Slow_Area)) * // 减速比例
                    (Status->Parameter.Max_W_Speed);                                       // 速度变化量
            }
            // 在停止区里用PID拉到指定位置（单环PID）
            else if (Angle_Offset <= Status->Parameter.Angle_Stop_Area)
            {
                PID_Calculate_Positional(&Line_AnglePID, Angle_Offset, 0);
                Status->Coordinate_System.Line_Target_V.Vw = Line_AnglePID.Output;
            }
            // 顺时针旋转则速度取反（前面都是绝对值计算）
            if (Status->Parameter.Angle_Rotate_Sum < 0)
            {
                Status->Coordinate_System.Line_Target_V.Vw *= -1;
            }
            // 角度转完之后将Angle_Rotate_Sum置为0
            // if (Angle_Offset < 0.5f)
            // {
            //     Status->Parameter.Angle_Rotate_Sum = 0;
            // }
            /*===================================================================================================================
                                                           以下是线速度计算部分
            =====================================================================================================================*/
            // 加速区内线速度计算
            if ((Status->Coordinate_System.Line_Now_Position.X) < Status->Parameter.Line_Route_Speedup_Area)
            {
                Status->Coordinate_System.Line_Target_V.Vx =
                    Status->Parameter.Start_Speed +
                    Fitting_Function(Status->Coordinate_System.Line_Now_Position.X / Status->Parameter.Line_Route_Speedup_Area) *
                        (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
            }
            // 在最大速度区
            else if ((Status->Coordinate_System.Line_Now_Position.X) >= Status->Parameter.Line_Route_Speedup_Area &&
                     (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area))
            {
                Status->Coordinate_System.Line_Target_V.Vx = Status->Parameter.Max_Speed;
            }
            // 减速区内线速度计算
            else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area) &&
                     (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
            {
                Status->Coordinate_System.Line_Target_V.Vx = // 期望速度
                    Fitting_Function(((Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X) / Status->Parameter.Line_Route_Slow_Area)) *
                        (Status->Parameter.Max_Speed) +
                    Status->Parameter.End_Speed;
            }
            // 在停止区用PID拉到指定位置
            else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
            {
                // 实现滑动PID控制
                Line_StopPID = Stop_Area_Slide_PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Target_Position.X,
                                                             Status->Coordinate_System.Line_Now_Position.X,
                                                             (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area),
                                                             Status->Parameter.Kp, Status->Parameter.Kd);
                PID_Calculate_Positional(&Line_StopPID, Status->Coordinate_System.Line_Now_Position.X, Status->Coordinate_System.Line_Target_Position.X);
                Status->Coordinate_System.Line_Target_V.Vx = Line_StopPID.Output;
            }
            // 法向保持直线
            if (ABS(Status->Coordinate_System.Line_Now_Position.X - Status->Coordinate_System.Line_Target_Position.X) > 5) // 快到直线结尾时不调节法向速度，避免出现轮子抖动
            {
                if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 3)
                {
                    PID_Calculate_Positional(&Line_AdjustPID, Status->Coordinate_System.Line_Now_Position.Y, 0);
                    // 前馈调节，差太多的话给一个大力拽回来
                    if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 1.5f)
                    {
                        if (Line_AdjustPID.Output > 0)
                            Line_AdjustPID.Output += 3;
                        else if (Line_AdjustPID.Output < 0)
                            Line_AdjustPID.Output -= 3;
                    }
                    Status->Coordinate_System.Line_Target_V.Vy = Line_AdjustPID.Output;
                }
            }
            else
                Status->Coordinate_System.Line_Target_V.Vy = 0;
            // // 将线坐标系下的速度转换成世界坐标系下速度
            Status->Coordinate_System.World_Coordinate_System_Target_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.Line_Target_V, &Status->Coordinate_System.Zero_Speed, -Line_Angle);
            // // 将世界坐标系下速度转换成车身坐标系下速度
            Status->Coordinate_System.Robot_Coordinate_System_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.World_Coordinate_System_Target_V, &Status->Coordinate_System.Zero_Speed, Disk_Encoder.Yaw.World_Rotation_Angle);
            // 判断路径是否走完
            // 这里死区由3改为5，原因是雷达本身有数字跳变，为最大限度防止路径卡死
            if (Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X < 20 &&
                    Angle_Offset < 3 &&
                    VESC_Data_From_Subcontroller[0].RPM_From_Subcontroller < 100 &&
                    VESC_Data_From_Subcontroller[1].RPM_From_Subcontroller < 100 &&
                    VESC_Data_From_Subcontroller[2].RPM_From_Subcontroller < 100 &&
                    VESC_Data_From_Subcontroller[3].RPM_From_Subcontroller < 100 ||
                Teaching_Pendant_Data.Route_Death)
            {
                Status->Flag.Work_Start = DISABLE;
                Status->Parameter.Angle_Rotate_Sum = 0;
                Status->Coordinate_System.Line_Target_V.Vx = 0;
                Status->Coordinate_System.Line_Target_V.Vy = 0;
                Status->Coordinate_System.Line_Target_V.Vw = 0;
                Order_To_Subcontroller.Wheel_Break = 1; // 停止指令
            }
            if (Teaching_Pendant_Data.Reset_Confirm && Teaching_Pendant.Reset_Confirm)
            {
                // 如果手柄上按下了复位键，直接跳出路径，用于球掉的情况
                // 这里没有将路径Disable为了不让轮子锁死，加快Reset时间
                break;
            }
        }
        osDelay(2);
    }
}

Coordinate_Position_Struct Fix_Point_For_Shoot; // 定点一直投的定点，这个点选择在三分线之外
extern uint8_t Reload_Flag;                     // 换弹标志位，置为1时执行换弹
extern uint8_t Finish_Fire_Flag;                // 投球完成标志位，置为1时表示投球完成
extern uint8_t Adjusting_Fire_Angle_Flag;       // 微调发射丝杠位置标志位，置为1时微调发射丝杠位置
uint8_t Shoot_Point_Count = 0;
float A1_Remember_Pos = 0;
/*********************************************************************************
 * @name 	Judge_Fix_Point
 * @brief   存最合适定点投篮的点，在每次发射前调用一下即可
 * @details 存点逻辑是先判是否在三分线之外，再判离中轴线的距离
 *********************************************************************************/
void Judge_Fix_Point(void)
{
    // 这个是算当前点到篮框的距离，如果大于3125mm则认为是三分线外
    float Distance = sqrtf(pow(Computer_Vision_Data.LiDAR.X - Pre_Basket_Position.X, 2) +
                           pow(Computer_Vision_Data.LiDAR.Y - Pre_Basket_Position.Y, 2));
    // 当前点离中轴线的距离
    float Distance2 = fabsf(Computer_Vision_Data.LiDAR.X - Pre_Basket_Position.X);
    // 预设点离中轴线的距离
    float Distance3 = fabsf(Fix_Point_For_Shoot.X - Pre_Basket_Position.X);
    if (Distance > 3125)
    {
        // 当前点距离中轴线更近就刷当前点
        if (Distance2 < Distance3)
        {
            Fix_Point_For_Shoot.X = Computer_Vision_Data.LiDAR.X;
            Fix_Point_For_Shoot.Y = Computer_Vision_Data.LiDAR.Y;
            Fix_Point_For_Shoot.W = Computer_Vision_Data.LiDAR.W;
        }
    }
}
/*********************************************************************************
 * @name 	Shoot_Pre_Competition
 * @brief   投球预选赛函数，调用即可全自动跑完
 * @details 开始的时候球在发射装置之上，如果不是需要修改
 *********************************************************************************/
void Shoot_Pre_Competition(void)
{
    if (Teaching_Pendant_Data.Automatic_Switch && Teaching_Pendant.Automatic_Switch)
    {
        if (Teaching_Pendant.Route_Type == Route_Type_0)
        {
            Reload_Flag = 1; // 开跑的时候先换一次弹
            // 起始点到左半场视觉识别点，长路径
            Chassis_Line_Route(291, -2129, -90, 1500, 11000, 500, 50, 2000, 0.01, 0.5, 100, 15, 15);
            Teaching_Pendant.Route_Type = Route_Type_1;
        }
        if (Computer_Vision_Data.Camera.RealSense.X == 0 &&
            Computer_Vision_Data.Camera.RealSense.Y == 0 &&
            Vision_Point_Flag != 0)
        {
            // 在两个视觉识别点上，但是这时候摄像头没有看到点，一直没有数据的话，函数会在这一步卡住
            No_Point_To_Go();
        }
        if (Teaching_Pendant.Route_Type == Route_Type_1 &&
            Computer_Vision_Data.Camera.RealSense.X != 0 &&
            Computer_Vision_Data.Camera.RealSense.Y != 0)
        {
            // Route_Type_1是从视觉识别点到场地标定投球点
            //  视觉识别到点时，开始根据视觉回传拉点，这个路径是有自喵的（是根据坐标信息解算出来的）
            Line_Route_With_Changing_Target(2000, 50, &Route_Status);
            Judge_Fix_Point();
            Teaching_Pendant.Fire_Confirm = 1;
            Shoot_Point_Count++; // 计数器加1，计数器加到7的时候，执行定点一直投
            // 在Fire里面是有Reload的，发射完立马就开始换弹
            if (Shoot_Point_Count <= 7)
            {
                Teaching_Pendant.Route_Type = Route_Type_2;
            }
            else
            {
                // 如果计数器大于7，说明拉点已经完了，直接跳到Route_Type_3，开始定点一直投
                Teaching_Pendant.Route_Type = Route_Type_3;
            }
        }
        if (Teaching_Pendant.Route_Type == Route_Type_2 && Finish_Fire_Flag)
        {
            // Route_Type_2是从投球点到视觉识别点
            Nearest_Vision_Point_Route(); // 计算最近的视觉点并执行路径规划
            Teaching_Pendant.Route_Type = Route_Type_1;
            osDelay(2000); // 在视觉识别点停两秒等待操作手放球
        }
        if (Teaching_Pendant.Route_Type == Route_Type_3 && Finish_Fire_Flag)
        {
            // Route_Type_3是从最后一个点到定点一直投那个点，存点的时候已经存了自瞄角度
            Variable_Element_Route(&Fix_Point_For_Shoot);
            Teaching_Pendant.Route_Type = Route_Type_4;
        }
        if (Teaching_Pendant.Route_Type == Route_Type_4)
        {
            // 这里将车的控制权还给操作手，用来解决：1.自动选点选的不好 2.没有把点完全覆盖到
            Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
            Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
            Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
            if (Teaching_Pendant_Data.Back_To_Programme)
            {
                Teaching_Pendant.Route_Type = Route_Type_5;
            }
        }
        if (Teaching_Pendant.Route_Type == Route_Type_5)
        {
            Adjusting_Fire_Angle_Flag = 1;
            Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = (Teaching_Pendant_Data.Joystick_V.Vw) / 5;
            // 手柄拉丝杠微调,这里直接给速度
            g_a1motor.command.velocity = Teaching_Pendant_Data.Fire_Angle;
            // 这里直接用A1_Command_Pos来存储操作手微调的位置
            A1_Command_Pos = g_a1motor.feedback.position;
            A1Motor_distance_mode(&g_a1motor, A1_Command_Pos, 10, 800);
        }
    }
}
