#include "Computer_Vision.h"
#include "usart.h"
#include "disk.h"
#include "math.h"

Computer_Vision_Struct Computer_Vision_Data;

void Computer_Vision_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &Computer_Vision_Data.Computer_Vision_Rec_Data[0], 32);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/*********************************************************************************
 * @name   Normalize_Angle_Difference
 * @brief  角度差值归一化函数
 * @param  angle_diff: 角度差值
 * @retval 归一化后的角度差值 (-180° ~ +180°)
 * @note   处理角度在±180°边界处的跳变问题
 *********************************************************************************/
float Normalize_Angle_Difference(float angle_diff)
{
    while (angle_diff > 180.0f) {
        angle_diff -= 360.0f;
    }
    while (angle_diff < -180.0f) {
        angle_diff += 360.0f;
    }
    return angle_diff;
}

/*********************************************************************************
 * @name   Motion_Estimator_Init
 * @brief  运动估计器初始化
 * @param  estimator: 运动估计器指针
 * @retval 无
 *********************************************************************************/
void Motion_Estimator_Init(Motion_Estimator_Struct *estimator)
{
    // 清零所有数据
    estimator->last_position.X = 0.0f;
    estimator->last_position.Y = 0.0f;
    estimator->last_position.W = 0.0f;
    
    estimator->current_position.X = 0.0f;
    estimator->current_position.Y = 0.0f;
    estimator->current_position.W = 0.0f;
    
    estimator->last_velocity.X = 0.0f;
    estimator->last_velocity.Y = 0.0f;
    estimator->last_velocity.W = 0.0f;
    
    estimator->current_velocity.X = 0.0f;
    estimator->current_velocity.Y = 0.0f;
    estimator->current_velocity.W = 0.0f;
    
    estimator->current_acceleration.X = 0.0f;
    estimator->current_acceleration.Y = 0.0f;
    estimator->current_acceleration.W = 0.0f;
    
    // 由于数据稳定（30Hz），不使用滤波器
    estimator->velocity_filter_weight = 1.0f;    // 不滤波，直接使用新数据
    estimator->accel_filter_weight = 1.0f;       // 不滤波，直接使用新数据
    
    // 初始化标志
    estimator->last_timestamp = 0;
    estimator->current_timestamp = 0;
    estimator->initialized = 0;
    estimator->valid_data_count = 0;
}

/*********************************************************************************
 * @name   Motion_Estimator_Update
 * @brief  更新运动估计器（使用固定时间间隔版本）
 * @param  estimator: 运动估计器指针
 * @param  new_position: 新的位置数据
 * @param  timestamp: 时间戳计数器（每次递增1）
 * @retval 无
 *********************************************************************************/
void Motion_Estimator_Update(Motion_Estimator_Struct *estimator, LiDAR_Struct *new_position, uint32_t timestamp)
{
    // 更新时间戳
    estimator->last_timestamp = estimator->current_timestamp;
    estimator->current_timestamp = timestamp;
    
    // 更新位置数据
    estimator->last_position = estimator->current_position;
    estimator->current_position = *new_position;
    
    // 首次初始化，第一次初始化没有last
    if (!estimator->initialized) {
        estimator->initialized = 1;
        estimator->valid_data_count = 1;
        return;
    }
    
    // 固定时间间隔：30Hz = 1/30 = 0.03333秒
    const float dt = 1.0f / 30.0f; // 33.33ms = 0.03333秒
    
    // 计算位置差值（单位：毫米）
    float dx = estimator->current_position.X - estimator->last_position.X;
    float dy = estimator->current_position.Y - estimator->last_position.Y;
    float dw = estimator->current_position.W - estimator->last_position.W;
    
    // 处理角度跳变问题
    dw = Normalize_Angle_Difference(dw);
    
    // 更新历史速度
    estimator->last_velocity = estimator->current_velocity;
    
    // 直接计算速度（不滤波，数据稳定）
    estimator->current_velocity.X = dx / dt;  // mm/s
    estimator->current_velocity.Y = dy / dt;  // mm/s
    estimator->current_velocity.W = dw / dt;  // °/s
    estimator->LIDAR_Speed = sqrtf(estimator->current_velocity.X * estimator->current_velocity.X +
                                             estimator->current_velocity.Y * estimator->current_velocity.Y);
    // 计算加速度（需要至少2个数据点）
    if (estimator->valid_data_count >= 2) {
        estimator->current_acceleration.X = (estimator->current_velocity.X - estimator->last_velocity.X) / dt; // mm/s²
        estimator->current_acceleration.Y = (estimator->current_velocity.Y - estimator->last_velocity.Y) / dt; // mm/s²
        estimator->current_acceleration.W = (estimator->current_velocity.W - estimator->last_velocity.W) / dt; // °/s²
        estimator->LIDAR_Acceleration = sqrtf(estimator->current_acceleration.X * estimator->current_acceleration.X +
                                             estimator->current_acceleration.Y * estimator->current_acceleration.Y);
    }
    
    // 更新有效数据计数
    if (estimator->valid_data_count < 255) {
        estimator->valid_data_count++;
    }
}

/*********************************************************************************
 * @name   Get_Vehicle_Velocity
 * @brief  获取车身速度
 * @param  estimator: 运动估计器指针
 * @retval 车身速度结构体
 *********************************************************************************/
Vehicle_Velocity_Struct Get_Vehicle_Velocity(Motion_Estimator_Struct *estimator)
{
    return estimator->current_velocity;
}

/*********************************************************************************
 * @name   Get_Vehicle_Acceleration
 * @brief  获取车身加速度
 * @param  estimator: 运动估计器指针
 * @retval 车身加速度结构体
 *********************************************************************************/
Vehicle_Acceleration_Struct Get_Vehicle_Acceleration(Motion_Estimator_Struct *estimator)
{
    return estimator->current_acceleration;
}

/*********************************************************************************
 * @name   Get_Vehicle_Speed
 * @brief  获取车身合速度（标量）
 * @param  estimator: 运动估计器指针
 * @retval 车身合速度 (mm/s)
 *********************************************************************************/
float Get_Vehicle_Speed(Motion_Estimator_Struct *estimator)
{
    float vx = estimator->current_velocity.X;
    float vy = estimator->current_velocity.Y;
    return sqrtf(vx * vx + vy * vy);
}

/*********************************************************************************
 * @name   Get_Vehicle_Angular_Speed
 * @brief  获取车身角速度
 * @param  estimator: 运动估计器指针
 * @retval 角速度 (°/s)
 *********************************************************************************/
float Get_Vehicle_Angular_Speed(Motion_Estimator_Struct *estimator)
{
    return estimator->current_velocity.W;
}

/*********************************************************************************
 * @name   Get_Vehicle_Acceleration_Magnitude
 * @brief  获取车身加速度大小（标量）
 * @param  estimator: 运动估计器指针
 * @retval 加速度大小 (mm/s²)
 *********************************************************************************/
float Get_Vehicle_Acceleration_Magnitude(Motion_Estimator_Struct *estimator)
{
    float ax = estimator->current_acceleration.X;
    float ay = estimator->current_acceleration.Y;
    return sqrtf(ax * ax + ay * ay);
}

/*********************************************************************************
 * @name   Is_Motion_Data_Valid
 * @brief  检查运动数据是否有效
 * @param  estimator: 运动估计器指针
 * @retval 1: 数据有效, 0: 数据无效
 *********************************************************************************/
uint8_t Is_Motion_Data_Valid(Motion_Estimator_Struct *estimator)
{
    return (estimator->initialized && estimator->valid_data_count >= 2);
}

/*********************************************************************************
 * @name   Computer_Vision_Data_Process
 * @brief  计算机视觉数据处理函数（增强版）
 * @param  Computer_Vision: 视觉数据结构体指针
 * @retval 无
 *********************************************************************************/
void Computer_Vision_Data_Process(Computer_Vision_Struct *Computer_Vision)
{
    //前两位是标志位
    // Kinect数据处理
    Computer_Vision->Camera.Kinect.Z = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[2]);
    Computer_Vision->Camera.Kinect.X = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[6]);
    // RealSense数据处理
    Computer_Vision->Camera.RealSense.X = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[10]);
    Computer_Vision->Camera.RealSense.Y = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[14]);
    // LiDAR数据处理
    Computer_Vision->LiDAR.X = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[18]);
    Computer_Vision->LiDAR.Y = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[22]);
    Computer_Vision->LiDAR.W = Get_Float_From_4u8(&Computer_Vision->Computer_Vision_Rec_Data[26]);
    // 最后两位是标志位
    
    Position_Coordinate_Transformation(&Computer_Vision->LiDAR, &Computer_Vision->mingsang_Coordinate, 90.0f);
    // *** 新增：更新运动估计器 ***
    // 使用静态计数器代替时间戳
    static uint32_t data_counter = 0;
    data_counter++;
    
    // 初始化运动估计器（如果未初始化）
    if (!Computer_Vision->Motion.initialized) {
        Motion_Estimator_Init(&Computer_Vision->Motion);
    }
    
    // 更新运动状态估计（使用计数器作为时间戳）
    Motion_Estimator_Update(&Computer_Vision->Motion, &Computer_Vision->LiDAR, data_counter);
}

