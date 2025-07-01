#include "Teaching_Pendant.h"
#include "usart.h"
#include "disk.h"
#include "route.h"
#include "math.h"

uint8_t Teaching_Pendant_buffer[30];
remote_control Teaching_Pendant = {.Death = -1, .Automatic_Switch = -1};//程序里面可以直接被刷掉的标志位
remote_control Teaching_Pendant_Data;//手柄直接的数据

// 增强版手柄处理器
static Enhanced_Teaching_Pendant_t enhanced_pendant;

/*********************************************************************************
 * @name   Apply_Deadzone
 * @brief  应用死区处理,这个函数是一个比例死区函数
 * @param  value: 输入值(归一化到-1到1)
 * @param  threshold: 死区阈值(0-1)
 * @retval 处理后的值
 *********************************************************************************/
static float Apply_Deadzone(float value, float threshold)
{
    float abs_value = fabsf(value);

    if (abs_value < threshold)
    {
        return 0.0f;
    }

    // 重新映射到去除死区后的范围
    float sign = (value >= 0) ? 1.0f : -1.0f;
    float mapped_value = (abs_value - threshold) / (1.0f - threshold);

    return sign * mapped_value;
}

/*********************************************************************************
 * @name   Nonlinear_Mapping
 * @brief  非线性映射函数 - 实现y=x^n的平滑曲线
 * @param  input: 输入值(归一化到-1到1)
 * @param  power: 非线性指数
 * @param  enable_smooth: 是否启用平滑过渡
 * @retval 映射后的值
 * @note   当power=1时为线性，power=2时为二次方，power越大中心区域越不敏感
 *********************************************************************************/
static float Nonlinear_Mapping(float input, float power, uint8_t enable_smooth)
{
    float abs_input = fabsf(input);
    float sign = (input >= 0) ? 1.0f : -1.0f;

    if (enable_smooth && power > 1.0f)
    {
        // 使用平滑过渡的非线性映射
        // 结合线性和非线性部分，避免在中心附近过度不敏感
        float linear_part = abs_input * 0.2f;                 // 20%线性分量
        float nonlinear_part = powf(abs_input, power) * 0.8f; // 80%非线性分量

        return sign * (linear_part + nonlinear_part);
    }
    else
    {
        // 纯非线性映射
        return sign * powf(abs_input, power);
    }
}

/*********************************************************************************
 * @name   Normalize_Joystick_Input
 * @brief  归一化摇杆输入
 * @param  raw_value: 原始摇杆值
 * @param  max_value: 摇杆最大值
 * @retval 归一化后的值(-1到1)
 *********************************************************************************/
static float Normalize_Joystick_Input(float raw_value, float max_value)
{
    if (max_value == 0.0f)
        return 0.0f;

    float normalized = raw_value / max_value;

    // 限制到[-1, 1]范围
    if (normalized > 1.0f)
        normalized = 1.0f;
    if (normalized < -1.0f)
        normalized = -1.0f;

    return normalized;
}
/*********************************************************************************
 * @name   Detect_Motion_State
 * @brief  检测运动状态 - 判断是加速还是减速
 * @param  target: 目标速度 (mm/s 或 °/s)
 * @param  current: 当前速度 (mm/s 或 °/s)
 * @param  stop_threshold: 停止阈值，低于此值认为是静止状态
 * @retval 运动状态：1=减速, 0=加速
 * @note   减速情况包括：
 *         1. 方向改变（正负号不同且都不接近零）
 *         2. 同向减速（目标速度绝对值小于当前速度绝对值）
 *         3. 制动停止（目标速度接近零）
 *********************************************************************************/
static uint8_t Detect_Motion_State(float target, float current, float stop_threshold)
{
    // 1. 制动检测 - 目标速度接近零
    if (fabsf(target) < stop_threshold)
    {
        return 1; // 减速（制动）
    }

    // 2. 方向改变检测 - 当前和目标速度符号相反且都不接近零
    if ((target > stop_threshold && current < -stop_threshold) ||
        (target < -stop_threshold && current > stop_threshold))
    {
        return 1; // 减速（方向改变）
    }

    // 3. 同向减速检测 - 目标速度绝对值小于当前速度绝对值
    if (fabsf(target) < fabsf(current) - stop_threshold) // 加入阈值避免抖动
    {
        return 1; // 减速（同向减速）
    }

    // 其他情况为加速
    return 0; // 加速
}
/*********************************************************************************
 * @name   Is_Deceleration_Case
 * @brief  判断是否为减速情况
 * （时代的眼泪了，是Smart_Acceleration_Limit的子函数）
 * （此处保留等待有缘人修改完善吧）
 *********************************************************************************/
// static uint8_t Is_Deceleration_Case(float target, float current)
// {
//     if ((target > 0 && current < 0) || (target < 0 && current > 0))
//     {
//         return 1; // 方向改变
//     }

//     if (fabsf(target) < fabsf(current))
//     {
//         return 1; // 同向减速
//     }

//     if (fabsf(target) < 0.1f)
//     {
//         return 1; // 制动
//     }

//     return 0; // 加速
// }
/*********************************************************************************
 * @name   Smart_Acceleration_Limit
 * @brief  智能加速度限制函数 - 防止速度变化过快或过慢
 * （时代的眼泪了，这个减速限最小加速度仅为理论可行，真实过程中会由于部分误差导致数字跳变很危险的）
 * （此处保留等待有缘人修改完善吧）
 * @param  target: 目标速度 (mm/s 或 °/s)
 * @param  current: 当前速度 (mm/s 或 °/s)
 * @param  max_accel: 加速时允许的最大加速度 (mm/s² 或 °/s²)
 * @param  min_decel: 减速时要求的最小加速度 (mm/s² 或 °/s²)
 * @param  dt: 控制周期时间间隔 (秒)
 * @retval 限制后的速度值
 * @note   这个函数实现两种保护机制：
 *         1. 加速保护：限制加速度不要过大，防止机械冲击和打滑
 *         2. 减速保证：确保减速度足够大，保证快速制动和安全停止
 * @example:
 *         // 限制X轴速度变化，最大加速度2500mm/s²，最小减速度5000mm/s²
 *         float limited_vx = Smart_Acceleration_Limit(target_vx, current_vx,
 *                                                    2500.0f, 5000.0f, 0.002f);
 *********************************************************************************/
// static float Smart_Acceleration_Limit(float target, float current, float max_accel, float min_decel, float dt)
// {
//     // 计算目标速度与当前速度的差值
//     // speed_diff > 0: 需要增加速度（可能是加速或从负速度向零减速）
//     // speed_diff < 0: 需要减少速度（可能是减速或从正速度向零减速）
//     float speed_diff = target - current;
//     // 计算达到目标速度所需的加速度大小（取绝对值）
//     // required_accel = |Δv| / Δt，单位：mm/s² 或 °/s²
//     float required_accel = fabsf(speed_diff) / dt;
//     // 使用Is_Deceleration_Case函数判断当前是否为减速情况
//     // 减速情况包括：方向改变、同向减速、制动停止
//     if (Is_Deceleration_Case(target, current))
//     {
//         // === 减速情况处理 ===
//         // 目标：确保减速度足够大，保证快速制动和安全性

//         // 检查当前所需的减速度是否小于最小要求
//         if (required_accel < min_decel)
//         {
//             // 当前减速度不够，强制使用最小减速度
//             // 这确保了机器人能够快速停止，提高安全性

//             if (speed_diff > 0)
//             {
//                 // target > current，需要向正方向移动来实现减速
//                 // 例如：从-1000mm/s减速到0，需要增加速度值
//                 return current + min_decel * dt;
//             }
//             else
//             {
//                 // target < current，需要向负方向移动来实现减速
//                 // 例如：从1000mm/s减速到0，需要减少速度值
//                 return current - min_decel * dt;
//             }
//         }
//         // 如果减速度已经足够大(required_accel >= min_decel)，
//         // 则跳到函数末尾，直接返回target，允许自然减速
//     }
//     else
//     {
//         // === 加速情况处理 ===
//         // 目标：限制加速度不要过大，防止机械冲击、打滑和系统不稳定

//         // 检查当前所需的加速度是否超过最大允许值
//         if (required_accel > max_accel)
//         {
//             // 加速度过大，需要限制到最大允许值
//             // 这防止了机器人突然启动造成的机械冲击

//             if (speed_diff > 0)
//             {
//                 // target > current，需要正向加速
//                 // 使用最大允许加速度进行渐进加速
//                 return current + max_accel * dt;
//             }
//             else
//             {
//                 // target < current，需要负向加速（反向加速）
//                 // 使用最大允许加速度进行渐进加速
//                 return current - max_accel * dt;
//             }
//         }
//         // 如果加速度在允许范围内(required_accel <= max_accel)，
//         // 则跳到函数末尾，直接返回target，允许自然加速
//     }

//     // 如果执行到这里，说明：
//     // 1. 减速情况下，减速度已经足够大(>= min_decel)，或
//     // 2. 加速情况下，加速度在允许范围内(<= max_accel)
//     // 此时直接返回目标值，不进行任何限制
//     return target;
// }

/*********************************************************************************
 * 函数工作原理详解：
 *
 * 1. 【减速保护机制】
 *    - 当检测到减速情况时，要求减速度至少达到min_decel
 *    - 如果自然减速度不够，强制使用最小减速度
 *    - 目的：确保机器人能够快速停止，提高安全性
 *    - 应用场景：紧急制动、方向快速切换、精确定位
 *
 * 2. 【加速保护机制】
 *    - 当检测到加速情况时，限制加速度不超过max_accel
 *    - 如果所需加速度过大，限制到最大允许值
 *    - 目的：防止机械冲击、电机过载、系统不稳定
 *    - 应用场景：平滑启动、防止打滑、保护机械结构
 *
 * 3. 【参数设置建议】
 *    - max_accel: 通常设置为机器人舒适加速度，考虑机械限制
 *    - min_decel: 通常设置为max_accel的1.5-2倍，确保制动性能
 *    - dt: 控制周期，需要与实际调用频率一致
 *
 * 4. 【数值示例】
 *    max_accel = 2500 mm/s², min_decel = 5000 mm/s², dt = 0.002s
 *
 *    加速情况：从0加速到3000mm/s
 *    - 所需加速度 = 3000/0.002 = 1,500,000 mm/s² (过大!)
 *    - 限制后输出 = 0 + 2500*0.002 = 5 mm/s (渐进加速)
 *
 *    减速情况：从1000mm/s减速到0
 *    - 所需加速度 = 1000/0.002 = 500,000 mm/s² (足够大)
 *    - 直接输出目标值 = 0 mm/s (快速制动)
 *********************************************************************************/
/*********************************************************************************
 * @name   Dual_Speed_Limiter
 * @brief  双重速度限制器 - 分别限制加速和减速的最大加速度
 * @param  target: 目标速度 (mm/s 或 °/s)
 * @param  current: 当前速度 (mm/s 或 °/s)
 * @param  max_accel: 加速时的最大加速度 (mm/s² 或 °/s²)
 * @param  max_decel: 减速时的最大加速度 (mm/s² 或 °/s²)
 * @param  dt: 控制周期时间间隔 (秒)
 * @retval 限制后的速度值
 * @note   与Smart_Acceleration_Limit不同，这个函数纯粹进行加速度限制，
 *         不强制最小减速度，更加稳定可靠
 *********************************************************************************/
static float Dual_Speed_Limiter(float target, float current, float max_accel, float max_decel, float dt)
{
    // 计算速度差值
    float speed_diff = target - current;

    // 如果差值很小，直接返回目标值
    if (fabsf(speed_diff) < 0.1f)
    {
        return target;
    }

    // 计算所需的加速度
    float required_accel = fabsf(speed_diff) / dt;

    // 使用改进的运动状态检测
    uint8_t is_decelerating = Detect_Motion_State(target, current, 10.0f); // 10mm/s作为停止阈值

    if (is_decelerating)
    {
        // === 减速情况：限制减速加速度不要过大 ===
        if (required_accel > max_decel)
        {
            // 减速度过大，限制到最大允许值
            if (speed_diff > 0)
            {
                return current + max_decel * dt;
            }
            else
            {
                return current - max_decel * dt;
            }
        }
    }
    else
    {
        // === 加速情况：限制加速加速度不要过大 ===
        if (required_accel > max_accel)
        {
            // 加速度过大，限制到最大允许值
            if (speed_diff > 0)
            {
                return current + max_accel * dt;
            }
            else
            {
                return current - max_accel * dt;
            }
        }
    }

    // 如果加速度在允许范围内，直接返回目标值
    return target;
}
/*********************************************************************************
 * @name   Enhanced_Teaching_Pendant_Init
 * @brief  初始化增强版手柄处理器 - 双重加速度限制版本
 * @retval 无
 * @note   使用分离的加速和减速加速度限制策略
 *********************************************************************************/
void Enhanced_Teaching_Pendant_Init(void)
{
    // 初始化低通滤波器 (20ms采样周期)
    LPFilter_Init(&enhanced_pendant.filter_magnitude, 0.02f, 4.0f); // 幅值滤波器 - 新增
    LPFilter_Init(&enhanced_pendant.filter_vw, 0.02f, 12.0f);       // 角速度12Hz截止频率

    // 配置非线性映射参数
    enhanced_pendant.mapping_config.max_speed_vx = 15000.0f;      // X方向最大8m/s
    enhanced_pendant.mapping_config.max_speed_vy = 15000.0f;      // Y方向最大8m/s
    enhanced_pendant.mapping_config.max_speed_vw = 2500.0f;       // 角速度最大1500°/s
    enhanced_pendant.mapping_config.deadzone_threshold = 0.08f;   // 8%死区
    enhanced_pendant.mapping_config.nonlinear_power = 2.2f;       // 2.2次方映射
    enhanced_pendant.mapping_config.enable_smooth_transition = 1; // 启用平滑过渡

    // 配置双重加速度限制参数
    // 加速限制 - 相对保守，保护机械结构
    enhanced_pendant.accel_config.max_accel_vx = 8000.0f; // X方向加速限制3m/s²
    enhanced_pendant.accel_config.max_accel_vy = 8000.0f; // Y方向加速限制3m/s²
    enhanced_pendant.accel_config.max_accel_vw = 3000.0f; // 角速度加速限制600°/s²
    // 111
    //  减速限制 - 相对激进，保证响应性
    enhanced_pendant.accel_config.max_decel_vx = 15000.0f; // X方向减速限制6m/s²
    enhanced_pendant.accel_config.max_decel_vy = 15000.0f; // Y方向减速限制6m/s²
    enhanced_pendant.accel_config.max_decel_vw = 6000.0f;       // 角速度减速限制1200°/s²

    enhanced_pendant.accel_config.speed_deadzone = 40.0f; // 速度死区10mm/s
    enhanced_pendant.accel_config.control_period = 0.01f; // 20ms控制周期

    // 初始化状态
    enhanced_pendant.last_output.Vx = 0.0f;
    enhanced_pendant.last_output.Vy = 0.0f;
    enhanced_pendant.last_output.Vw = 0.0f;
    enhanced_pendant.angle_filter_init = 0;
    enhanced_pendant.last_velocity_angle = 0.0f;
    enhanced_pendant.initialized = 1;
}
//时代的眼泪了，原来想用极坐标来分开处理速度角度的，给角度滤波，速度限幅，后来发现直接用矢量速度更好
// /*********************************************************************************
//  * @name   Vector_To_Polar
//  * @brief  将矢量速度转换为极坐标（幅值和角度）
//  * @param  vx: X方向速度
//  * @param  vy: Y方向速度
//  * @param  magnitude: 输出幅值指针
//  * @param  angle: 输出角度指针（弧度）
//  *********************************************************************************/
// static void Vector_To_Polar(float vx, float vy, float *magnitude, float *angle)
// {
//     *magnitude = sqrtf(vx * vx + vy * vy);
//     *angle = atan2f(vy, vx);
// }

// /*********************************************************************************
//  * @name   Polar_To_Vector
//  * @brief  将极坐标转换为矢量速度
//  * @param  magnitude: 幅值
//  * @param  angle: 角度（弧度）
//  * @param  vx: 输出X方向速度指针
//  * @param  vy: 输出Y方向速度指针
//  *********************************************************************************/
// static void Polar_To_Vector(float magnitude, float angle, float *vx, float *vy)
// {
//     *vx = magnitude * cosf(angle);
//     *vy = magnitude * sinf(angle);
// }

// /*********************************************************************************
//  * @name   Normalize_Angle_Difference
//  * @brief  角度差值归一化到±π范围
//  * @param  angle_diff: 角度差值（弧度）
//  * @retval 归一化后的角度差值
//  *********************************************************************************/
// static float Normalize_Angle_Difference(float angle_diff)
// {
//     while (angle_diff > 3.14159265f)
//     {
//         angle_diff -= 2.0f * 3.14159265f;
//     }
//     while (angle_diff < -3.14159265f)
//     {
//         angle_diff += 2.0f * 3.14159265f;
//     }
//     return angle_diff;
// }

/*********************************************************************************
 * @name   Get_Enhanced_Speed_From_Teaching_Pendant
 * @brief  获取增强处理后的手柄速度数据（分离式滤波版本）
 * @retval 处理后的速度结构体
 * @note   核心特性：分离式滤波策略
 *         - 加速时：对速度进行低通滤波，保证平滑启动
 *         - 减速时：对方向角度进行低通滤波，速度大小不滤波，保证快速制动
 *********************************************************************************/
Coordinate_Speed_Struct Get_Enhanced_Speed_From_Teaching_Pendant(void)
{
    Coordinate_Speed_Struct output = {0};
    // 确保已初始化
    if (!enhanced_pendant.initialized)
    {
        Enhanced_Teaching_Pendant_Init();
    }
    // 第一步：归一化摇杆输入，这里是摇杆原始值的最大值，这里手柄最大值是6800（这里并不是车的最大速度）
    float normalized_vx = Normalize_Joystick_Input(Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vx, 8000.0f);
    float normalized_vy = Normalize_Joystick_Input(Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vy, 8000.0f);
    float normalized_vw = Normalize_Joystick_Input(Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vw, 8000.0f);
    // 第二步：应用死区处理，为了处理手柄回中时的数字抖动
    normalized_vx = Apply_Deadzone(normalized_vx, enhanced_pendant.mapping_config.deadzone_threshold);
    normalized_vy = Apply_Deadzone(normalized_vy, enhanced_pendant.mapping_config.deadzone_threshold);
    normalized_vw = Apply_Deadzone(normalized_vw, enhanced_pendant.mapping_config.deadzone_threshold);
    // 第三步：非线性映射
    float mapped_vx = Nonlinear_Mapping(normalized_vx,
                                        enhanced_pendant.mapping_config.nonlinear_power,
                                        enhanced_pendant.mapping_config.enable_smooth_transition);
    float mapped_vy = Nonlinear_Mapping(normalized_vy,
                                        enhanced_pendant.mapping_config.nonlinear_power,
                                        enhanced_pendant.mapping_config.enable_smooth_transition);
    float mapped_vw = Nonlinear_Mapping(normalized_vw,
                                        enhanced_pendant.mapping_config.nonlinear_power,
                                        enhanced_pendant.mapping_config.enable_smooth_transition);
    // 第四步：转换为实际速度值，这里的max是车的max值
    float target_vx = mapped_vx * enhanced_pendant.mapping_config.max_speed_vx;
    float target_vy = mapped_vy * enhanced_pendant.mapping_config.max_speed_vy;
    float target_vw = mapped_vw * enhanced_pendant.mapping_config.max_speed_vw;
    /*===================================================================================================================
                                                角度和速度滤波（效果不好）
    =====================================================================================================================*/
    // // 第五步：分离式滤波策略 - 修改为统一角度滤波
    // float filtered_vx, filtered_vy, filtered_vw;
    // // === 处理平移速度 (Vx, Vy) - 统一角度滤波策略 ===
    // // 计算当前和目标的矢量幅值和角度
    // float current_magnitude, current_angle;
    // float target_magnitude, target_angle;
    // Vector_To_Polar(enhanced_pendant.last_output.Vx, enhanced_pendant.last_output.Vy,
    //                 &current_magnitude, &current_angle);
    // Vector_To_Polar(target_vx, target_vy, &target_magnitude, &target_angle);
    // // 判断是加速还是减速（基于矢量幅值）
    // uint8_t is_accelerating = (target_magnitude > current_magnitude) ||
    //                           (current_magnitude < 50.0f); // 低速时也当作加速处理
    // // === 统一的角度滤波处理 ===
    // static float last_filtered_angle = 0.0f;
    // static uint8_t angle_filter_initialized = 0;

    // if (!angle_filter_initialized && target_magnitude > 0.1f)
    // {
    //     last_filtered_angle = target_angle;
    //     angle_filter_initialized = 1;
    // }
    // if (is_accelerating)
    // {
    //     // === 加速情况：对速度进行低通滤波 + 角度滤波 ===
    //     if (target_magnitude > 0.01f)
    //     {
    //         // 计算自适应角度变化率
    //         float adaptive_angle_rate;
    //         if (target_magnitude > 8000.0f)
    //         {
    //             adaptive_angle_rate = 0.15f; // 高速时角度变化更慢
    //         }
    //         else if (target_magnitude > 4000.0f)
    //         {
    //             float speed_ratio = (target_magnitude - 4000.0f) / 4000.0f;
    //             adaptive_angle_rate = 0.3f - (0.15f * speed_ratio);
    //         }
    //         else
    //         {
    //             adaptive_angle_rate = 0.3f; // 低速时正常变化率
    //         }
    //         // 角度滤波
    //         float angle_diff = Normalize_Angle_Difference(target_angle - last_filtered_angle);
    //         float filtered_angle_diff = adaptive_angle_rate * angle_diff;
    //         float filtered_angle = last_filtered_angle + filtered_angle_diff;
    //         last_filtered_angle = filtered_angle;
    //         // 对幅值也进行滤波（加速情况）
    //         float filtered_magnitude = LPFilter_Process(&enhanced_pendant.filter_magnitude, target_magnitude);
    //         // 重建滤波后的矢量
    //         Polar_To_Vector(filtered_magnitude, filtered_angle, &filtered_vx, &filtered_vy);
    //     }
    //     else
    //     {
    //         // 静止状态
    //         filtered_vx = 0.0f;
    //         filtered_vy = 0.0f;
    //     }
    // }
    // else
    // {
    //     // === 减速情况：幅值不滤波 + 角度滤波 ===
    //     if (target_magnitude > 0.01f)
    //     {
    //         // 计算自适应角度变化率
    //         float adaptive_angle_rate;
    //         if (target_magnitude > 8000.0f)
    //         {
    //             adaptive_angle_rate = 0.15f; // 高速时角度变化更慢
    //         }
    //         else if (target_magnitude > 4000.0f)
    //         {
    //             float speed_ratio = (target_magnitude - 4000.0f) / 4000.0f;
    //             adaptive_angle_rate = 0.3f - (0.15f * speed_ratio);
    //         }
    //         else
    //         {
    //             adaptive_angle_rate = 0.3f; // 低速时正常变化率
    //         }

    //         // 角度滤波
    //         float angle_diff = Normalize_Angle_Difference(target_angle - last_filtered_angle);
    //         float filtered_angle_diff = adaptive_angle_rate * angle_diff;
    //         float filtered_angle = last_filtered_angle + filtered_angle_diff;
    //         last_filtered_angle = filtered_angle;

    //         // 使用滤波后的角度和原始幅值重建矢量（减速情况幅值不滤波）
    //         Polar_To_Vector(target_magnitude, filtered_angle, &filtered_vx, &filtered_vy);
    //     }
    //     else
    //     {
    //         // 静止状态，保持角度
    //         if (angle_filter_initialized)
    //         {
    //             Polar_To_Vector(0.0f, last_filtered_angle, &filtered_vx, &filtered_vy);
    //         }
    //         else
    //         {
    //             filtered_vx = 0.0f;
    //             filtered_vy = 0.0f;
    //         }
    //     }
    // }
    // // === 处理角速度 ===
    // // 角速度始终使用正常滤波
    // filtered_vw = LPFilter_Process(&enhanced_pendant.filter_vw, target_vw);
    // 第六步：加速度限制
    output.Vx = Dual_Speed_Limiter(target_vx,
                                   enhanced_pendant.last_output.Vx,
                                   enhanced_pendant.accel_config.max_accel_vx, // 加速限制
                                   enhanced_pendant.accel_config.max_decel_vx, // 减速限制
                                   enhanced_pendant.accel_config.control_period);
    output.Vy = Dual_Speed_Limiter(target_vy,
                                   enhanced_pendant.last_output.Vy,
                                   enhanced_pendant.accel_config.max_accel_vy, // 加速限制
                                   enhanced_pendant.accel_config.max_decel_vy, // 减速限制
                                   enhanced_pendant.accel_config.control_period);
    output.Vw = Dual_Speed_Limiter(target_vw,
                                   enhanced_pendant.last_output.Vw,
                                   enhanced_pendant.accel_config.max_accel_vw, // 加速限制
                                   enhanced_pendant.accel_config.max_decel_vw, // 减速限制
                                   enhanced_pendant.accel_config.control_period);
    // 第七步：应用最终速度死区
    if (fabsf(output.Vx) < enhanced_pendant.accel_config.speed_deadzone)
    {
        output.Vx = 0.0f;
    }
    if (fabsf(output.Vy) < enhanced_pendant.accel_config.speed_deadzone)
    {
        output.Vy = 0.0f;
    }
    if (fabsf(output.Vw) < 20.0f) // 角速度使用固定死区20°/s
    {
        output.Vw = 0.0f;
    }
    // 更新历史记录
    enhanced_pendant.last_output = output;
    return output;
}

void Teaching_Pendant_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, &Teaching_Pendant_buffer[0], 30);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}
//时代的眼泪了，这里写的是自研的手柄的通讯协议，但是只有50hz频率太低了，故弃用
// void Teaching_Pendant_Data_Process(Teaching_Pendant_Data_Struct *Teaching_Pendant)
// {
//     Teaching_Pendant->Automatic_Switch = Teaching_Pendant->Teaching_Pendant_Rec_Data[1];
//     Teaching_Pendant->Route_Type = Teaching_Pendant->Teaching_Pendant_Rec_Data[2];
//     Teaching_Pendant->Fire_Confirm = Teaching_Pendant->Teaching_Pendant_Rec_Data[3];
//     Teaching_Pendant->Automatic_Aiming_Switch = Teaching_Pendant->Teaching_Pendant_Rec_Data[4];
//     Teaching_Pendant->Dribble = Teaching_Pendant->Teaching_Pendant_Rec_Data[5];
//     Teaching_Pendant->Pass_Ball = Teaching_Pendant->Teaching_Pendant_Rec_Data[6];
//     Teaching_Pendant->Reset_Confirm = Teaching_Pendant->Teaching_Pendant_Rec_Data[7];
//     Teaching_Pendant->Route_Death = Teaching_Pendant->Teaching_Pendant_Rec_Data[8];
//     Teaching_Pendant->Competition_Mode = Teaching_Pendant->Teaching_Pendant_Rec_Data[9];
//     Teaching_Pendant->Back_To_Programme = Teaching_Pendant->Teaching_Pendant_Rec_Data[10];

//     Speed_Data_From_Teaching_Pendant.Vx = Get_Float_From_4u8(&Teaching_Pendant->Teaching_Pendant_Rec_Data[11]);
//     Speed_Data_From_Teaching_Pendant.Vy = Get_Float_From_4u8(&Teaching_Pendant->Teaching_Pendant_Rec_Data[15]);
//     Speed_Data_From_Teaching_Pendant.Vw = Get_Float_From_4u8(&Teaching_Pendant->Teaching_Pendant_Rec_Data[19]);

//     Teaching_Pendant_Data.Joystick_V = Get_Enhanced_Speed_From_Teaching_Pendant();
//     Teaching_Pendant->Fire_Angle = Get_Float_From_4u8(&Teaching_Pendant->Teaching_Pendant_Rec_Data[23]);
// }

void HT10A_process(uint8_t buffer[30])
{
    int16_t _channels[16];
    if (buffer[0] != 0x0F || buffer[24] != 0x00)
        return; // 数据包头尾正确则开始处理
    _channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;
    // 通道1: bits 11-21 (byte2低5位 + byte3高6位)
    _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;
    // 通道2: bits 22-32 (byte3低2位 + byte4全8位 + byte5高1位)
    _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;
    // 通道3: bits 33-43 (byte5低7位 + byte6高4位)
    _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;
    // 通道4: bits 44-54 (byte6低4位 + byte7高7位)
    _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;
    // 通道5: bits 55-65 (byte7低1位 + byte8全8位 + byte9高2位)
    _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;
    // 通道6: bits 66-76 (byte9低6位 + byte10高5位)
    _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
    // 通道7: bits 77-87 (byte10低3位 + byte11全8位)
    _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;
    // 通道8: bits 88-98 (byte12低8位 + byte13高3位)
    _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;
    // 通道9: bits 99-109 (byte13低5位 + byte14高6位)
    _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;
    // 通道10: bits 110-120 (byte14低2位 + byte15全8位 + byte16高1位)
    _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;
    // 通道11: bits 121-131 (byte16低7位 + byte17高4位)
    _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;
    // 通道12: bits 132-142 (byte17低4位 + byte18高7位)
    _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;
    // 通道13: bits 143-153 (byte18低1位 + byte19全8位 + byte20高2位)
    _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;
    // 通道14: bits 154-164 (byte20低6位 + byte21高5位)
    _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;
    // 通道15: bits 165-175 (byte21低3位 + byte22全8位)
    _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;

    // 摇杆[-8000,8000]
    // Teaching_Pendant_Data.x = (_channels[3] - 992) * 10;
    // Teaching_Pendant_Data.y = (_channels[2] - 992) * 10;
    // Teaching_Pendant_Data.z = (_channels[0] - 992) * 10;
    Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vx = (_channels[3] - 992) * 10; // X方向速度
    Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vy = (_channels[2] - 992) * 10; // Y方向速度
    Teaching_Pendant_Data.Speed_Data_From_Teaching_Pendant.Vw = (_channels[0] - 992) * 10; // 角速度
    // 四个开关
    Teaching_Pendant_Data.Fire = (_channels[4] - 992) / 800;
    Teaching_Pendant_Data.Automatic_Switch = (_channels[5] - 992) / 800;
    Teaching_Pendant_Data.Automatic_Aiming_Switch = (_channels[6] - 992) / 800;
    Teaching_Pendant_Data.Death = (_channels[7] - 992) / 800;
    // 旋钮[-8000,8000]
    Teaching_Pendant_Data.switch5 = (_channels[8] - 992) * 10;
    Teaching_Pendant_Data.switch6 = (_channels[9] - 992) * 10;
    //处理摇杆的原始数据
    Teaching_Pendant_Data.Joystick_V = Get_Enhanced_Speed_From_Teaching_Pendant();
}
