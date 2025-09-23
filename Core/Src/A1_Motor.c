#include "A1_Motor.h"
#include "stm32h7xx.h"
#include "usart.h"
#include "disk.h"

/* 全局变量定义 */
A1Motor g_a1motor;

/* 内部结构体定义 */
#pragma pack(1)

// 用于类型转换的联合体
typedef union
{
    int32_t i32;
    uint32_t u32;
    uint8_t u8[4];
    uint16_t u16[2];
    float f32;
} DataConverter;

// 通信包头结构
typedef struct
{
    uint8_t start[2]; // 包头标识 0xFE, 0xEE
    uint8_t motorID;  // 电机ID只能为0.1.2
    uint8_t reserved; // 他们公司自用的保留位，没用
} PacketHeader;

// 发送到电机的数据包
typedef struct
{
    uint8_t mode;            // 电机模式，0:停转, 5:开环慢转, 10:闭环FOC
    uint8_t modifyBit;       // 修改标志位，没用
    uint8_t readBit;         // 读取标志位，没用
    uint8_t reserved;        // 保留字节，没用
    DataConverter reserved2; // 保留
    int16_t torque;          // 力矩
    int16_t velocity;        // 速度
    int32_t position;        // 位置
    int16_t kp;              // 位置刚度系数
    int16_t kw;              // 速度刚度系数
    uint8_t reserved3[6];    // 其他保留字节
} CommandData;

// 完整的发送数据包
typedef struct
{
    PacketHeader header;
    CommandData data;
    DataConverter crc;
} CommandPacket;

#pragma pack()

/* 内部函数声明 */
static uint32_t CRC32_Calculate(uint32_t *data, uint32_t length);
static void PackCommandData(CommandPacket *packet, const A1MotorCommand *cmd);
static A1MotorFeedback UnpackFeedbackData(const uint8_t *data);

/* 函数实现 */

/**
 * @brief 计算CRC32校验码
 * @param data: 要计算的数据
 * @param length: 数据长度(uint32_t单位)
 * @return CRC32校验值
 */
static uint32_t CRC32_Calculate(uint32_t *data, uint32_t length)
{
    uint32_t xbit = 0;
    uint32_t current = 0;
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;
    for (uint32_t i = 0; i < length; i++)
    {
        xbit = 1U << 31;
        current = data[i];
        for (uint32_t bit = 0; bit < 32; bit++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ polynomial;
            }
            else
            {
                crc = crc << 1;
            }
            if (current & xbit)
            {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }
    return crc;
}

/**
 * @brief 打包命令数据到通信协议格式
 * @param packet: 输出的数据包
 * @param cmd: 输入的命令参数
 */
static void PackCommandData(CommandPacket *packet, const A1MotorCommand *cmd)
{
    // 设置包头
    packet->header.start[0] = A1_PACKET_HEADER_0;
    packet->header.start[1] = A1_PACKET_HEADER_1;
    packet->header.motorID = cmd->id;
    packet->header.reserved = 0;
    // 设置命令数据
    packet->data.mode = cmd->mode;
    packet->data.modifyBit = 0xFF; // 修改所有参数
    packet->data.readBit = 0;
    // 转换物理量为通信协议单位
    packet->data.torque = (int16_t)(cmd->torque * A1_TORQUE_SCALE);
    packet->data.velocity = (int16_t)(cmd->velocity * A1_GEAR_RATIO * A1_VELOCITY_SCALE);
    packet->data.position = (int32_t)(cmd->position * A1_GEAR_RATIO * A1_POSITION_SCALE);
    packet->data.kp = (int16_t)(cmd->kp * A1_KP_SCALE);
    packet->data.kw = (int16_t)(cmd->kw * A1_KW_SCALE);
    // 计算CRC校验
    packet->crc.u32 = CRC32_Calculate((uint32_t *)packet, 7);
}
static uint8_t Flag_Of_A1_Feedback = 0; // 用于第一次接收数据时的标志位
A1MotorFeedback A1_feedback;
/**
 * @brief 解析接收到的反馈数据
 * @param data: 接收到的原始数据
 * @return 解析后的电机反馈
 */
static A1MotorFeedback UnpackFeedbackData(const uint8_t *data)
{
    float Angle_Add;
    if (data[0] != A1_PACKET_HEADER_0 || data[1] != A1_PACKET_HEADER_1)
    {
        // 包头错误
        return A1_feedback;
    }
    // 解析关键数据 (简化版，实际需要更详细的解析)
    A1_feedback.torque = ((int16_t)(data[12] | (data[13] << 8))) / A1_TORQUE_SCALE;
    A1_feedback.velocity = ((int16_t)(data[14] | (data[15] << 8))) / (A1_VELOCITY_SCALE * A1_GEAR_RATIO);
    A1_feedback.position = ((int32_t)(data[30] | (data[31] << 8) |
                                      (data[32] << 16) | (data[33] << 24))) /
                           (A1_POSITION_SCALE * A1_GEAR_RATIO);
    // 累计位置
    A1_feedback.temperature = (int8_t)data[6];
    A1_feedback.acceleration = (int16_t)(data[28] | (data[29] << 8));
    A1_feedback.error_code = data[7];                             // MError
    Angle_Add = A1_feedback.position - A1_feedback.Position_Last; // 计算角度增量
    if (!Flag_Of_A1_Feedback)
    {
        A1_feedback.Position_Sum = 0; // 初始化位置累积
        Flag_Of_A1_Feedback = 1;      // 第一次接收数据，初始化位置和角度
    }
    if (A1_feedback.position - A1_feedback.Position_Last > 200)
    {
        Angle_Add = Angle_Add - 357.442856f;
    }
    else if (A1_feedback.position - A1_feedback.Position_Last < -200)
    {
        Angle_Add = Angle_Add + 357.442856f;
    }
    A1_feedback.Position_Sum += Angle_Add;            // 累计位置
    A1_feedback.Position_Last = A1_feedback.position; // 保存上次位置
    return A1_feedback;
}
/**
 * @brief 发送控制命令到A1电机/
 * @param huart: 串口句柄
 * @param motor_cmd: 控制命令结构体指针
 * @param gpio: 485方向控制GPIO
 * @param gpio_pin: 485方向控制引脚
 */
void A1Motor_SendCommand(UART_HandleTypeDef *huart, A1MotorCommand *motor_cmd,
                         GPIO_TypeDef *gpio, uint16_t gpio_pin)
{
    CommandPacket packet = {0};
    // 打包数据
    PackCommandData(&packet, motor_cmd);
    // 设置485为发送模式
    HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_SET);
    // 发送数据
    HAL_UART_Transmit(huart, (uint8_t *)&packet, A1_PACKET_SIZE_TX, 0xFFF);
    // 设置485为接收模式
    HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_RESET);
}
static uint8_t A1_Rec_Data[100]; // 数据缓冲区
/**
 * @brief 启动电机通信接收
 */
void A1Motor_Restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, A1_Rec_Data, 100);
    __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
}

/**
 * @brief 处理接收到的电机反馈数据
 * @param huart: 串口句柄
 * @param usart_rx: 串口接收缓冲区
 * @param hdma_rx: DMA句柄
 * @return 解析后的电机反馈数据
 */
A1MotorFeedback feedback = {0};
A1MotorFeedback A1Motor_ProcessFeedback(UART_HandleTypeDef *huart,
                                        DMA_HandleTypeDef *hdma_rx)
{
    // 停止DMA接收
    HAL_UART_DMAStop(huart);
    // 检查并解析数据
    if (A1_Rec_Data[0] == A1_PACKET_HEADER_0 &&
        A1_Rec_Data[1] == A1_PACKET_HEADER_1)
    {
        feedback = UnpackFeedbackData(A1_Rec_Data);
    }
    // 重启接收
    A1Motor_Restart();
    return feedback;
}

/*********************************************************************************
 * @name 	A1Motor_distance_mode
 * @brief A1电机位置模式转化为速度模式（防止位置模式最大电流一直跑，导致烧电机）
 *         但是这个方法死区比较难调，可能会导致电机抖动
 * @param motor: A1电机结构体
 * @param distance: 命令的电机编码器的值
 * @param kw: 是速度模式下的P
 * @param v: 控的速度
 * @return 解析后的电机反馈数据
 *********************************************************************************/
void A1Motor_distance_mode(A1Motor *motor, float distance, float kw, float v)
{
    motor->command.mode = 10;
    motor->command.kw = kw;
    float dead_zone = 50;
    if (motor->feedback.position >= distance - dead_zone &&
        motor->feedback.position <= distance + dead_zone)
        motor->command.velocity = 0; // 如果位置已经到达目标位置，则速度为0
    else if (motor->feedback.position < distance - dead_zone)
        motor->command.velocity = v;
    else
        motor->command.velocity = -v;
}

/**
 * @brief  角度平滑分割函数
 * @details 将目标角度在指定时间内均匀分割，每次调用返回当前应达到的角度
 *          函数内部自动跟踪时间进度，每次调用间隔应为2ms
 *          这个是机械臂运动控制中常用的角度平滑过渡算法
 *
 * @param  total_time_ms 完成角度变化所需的总时间(毫秒)
 * @param  target_angle 目标最终角度
 * @return 当前应设置的角度值
 */
float A1_Angle_Splitting(uint32_t total_time_ms, float target_angle)
{
    // 静态变量，保持函数调用间的状态
    static uint32_t elapsed_time_ms = 0;   // 已经过去的时间(毫秒)
    static float start_angle = 0.0f;       // 起始角度
    static float current_angle = 0.0f;     // 当前角度
    static uint32_t last_total_time = 0;   // 上次设定的总时间
    static float last_target_angle = 0.0f; // 上次设定的目标角度
    // 检测是否是新的角度变换请求
    if (total_time_ms != last_total_time || target_angle != last_target_angle)
    {
        // 新的变换请求，重置状态
        start_angle = g_a1motor.feedback.position; // 使用当前电机位置作为起点
        elapsed_time_ms = 0;
        last_total_time = total_time_ms;
        last_target_angle = target_angle;
        current_angle = start_angle;
    }
    else
    {
        // 继续现有变换，更新已用时间(每次调用增加2ms)
        elapsed_time_ms += 2;
        // 确保不超过总时间
        if (elapsed_time_ms > total_time_ms)
        {
            elapsed_time_ms = total_time_ms;
        }
    }
    // 计算当前应该输出的角度
    if (elapsed_time_ms >= total_time_ms)
    {
        // 时间已到，直接输出目标角度
        current_angle = target_angle;
    }
    else
    {
        // 线性插值计算当前角度,有速度和加速度突变，控制效果欠佳
        // float progress = (float)elapsed_time_ms / total_time_ms;
        // current_angle = start_angle + (target_angle - start_angle) * progress;
        // 多项式插值计算当前角度（替换原线性插值）
        float progress = (float)elapsed_time_ms / total_time_ms; // 归一化时间 t∈[0,1]
        // 5次多项式: s(t) = s₀ + (s₁-s₀)(10t³ - 15t⁴ + 6t⁵)
        // 这里还学到了贝塞尔曲线的平滑插值方法，这个应该还能延伸到路径算法上，今年应该是没时间研究了，回头再看
        // 这保证了起点和终点的速度和加速度均为0
        float t3 = progress * progress * progress;        // t³
        float t4 = t3 * progress;                         // t⁴
        float t5 = t4 * progress;                         // t⁵
        float smooth_factor = 10 * t3 - 15 * t4 + 6 * t5; // 平滑因子
        // 计算当前角度
        current_angle = start_angle + (target_angle - start_angle) * smooth_factor;
    }
    return current_angle;
}

/**
 * @brief 上面两个函数的缝合体，实现位置闭环但是发送速度（原因是A1电机本身无法读出±178.722092以外的数，只能增量来控速度）
 * @details 结合角度平滑分割与速度控制，实现高品质位置控制
 *          每次调用间隔应为2ms
 *
 * @param motor: A1电机结构体
 * @param target_position: 目标位置
 * @param total_time_ms: 期望到达目标位置所需的总时间(毫秒)
 * @param kw: 速度刚度系数
 * @param max_velocity: 最大速度限制
 */
uint8_t Flag_Of_A1Motor_Smooth_Position_Control = 0; // 停止更新数据，防止A1电机一直抽搐（A1真垃圾）
void A1Motor_Smooth_Position_Control(A1Motor *motor, float target_position,
                                     uint32_t total_time_ms, float kw, float max_velocity)
{
    // 静态变量，保持函数调用间的状态
    static uint32_t elapsed_time_ms = 0;      // 已经过去的时间(毫秒)
    static float start_position = 0.0f;       // 起始位置
    static float reference_position = 0.0f;   // 参考位置轨迹
    static uint32_t last_total_time = 0;      // 上次设定的总时间
    static float last_target_position = 0.0f; // 上次设定的目标位置

    // 检测是否是新的位置控制请求
    if (total_time_ms != last_total_time || target_position != last_target_position)
    {
        // 新的位置请求，重置状态
        start_position = motor->feedback.Position_Sum; // 使用当前电机位置作为起点
        elapsed_time_ms = 0;
        last_total_time = total_time_ms;
        last_target_position = target_position;
        reference_position = start_position;
        Flag_Of_A1Motor_Smooth_Position_Control = 1; // 开始更新数据
    }
    else
    {
        // 继续现有过程，更新已用时间(每次调用增加2ms)
        elapsed_time_ms += 2;
        // 确保不超过总时间
        if (elapsed_time_ms > total_time_ms)
        {
            elapsed_time_ms = total_time_ms;
        }
    }
    if (Flag_Of_A1Motor_Smooth_Position_Control) // 没有这个A1就会嗞哇乱叫
    {
        // 计算参考轨迹位置
        if (elapsed_time_ms >= total_time_ms)
        {
            // 时间已到，目标位置作为参考
            reference_position = target_position;
        }
        else
        {
            // 5次多项式平滑插值计算参考位置
            float progress = (float)elapsed_time_ms / total_time_ms;
            float t3 = progress * progress * progress;
            float t4 = t3 * progress;
            float t5 = t4 * progress;
            float smooth_factor = 10 * t3 - 15 * t4 + 6 * t5;
            // 计算当前参考位置
            reference_position = start_position +
                                 (target_position - start_position) * smooth_factor;
        }
        // 配置电机控制参数
        motor->command.mode = 10; // 闭环FOC控制
        motor->command.kw = kw;   // 速度刚度系数
        // 计算当前位置与参考位置的偏差
        float position_error = reference_position - motor->feedback.Position_Sum;
        float total_error = target_position - motor->feedback.Position_Sum;
        // 死区处理 - 接近目标位置时停止
        float dead_zone = 0.1f; // 使用更小的死区提高精度
        if (fabs(total_error) < dead_zone)
        {
            // 到达目标位置且平滑过程已完成，停止电机
            motor->command.velocity = 0;
            Flag_Of_A1Motor_Smooth_Position_Control = 0; // 停止更新数据
        }
        else
        {
            // 根据位置偏差计算速度指令
            // 位置偏差越大，速度越大，但不超过max_velocity
            float velocity_cmd = position_error * 2.0f; // 比例系数可调
            // 速度限制
            if (velocity_cmd > max_velocity)
                velocity_cmd = max_velocity;
            else if (velocity_cmd < -max_velocity)
                velocity_cmd = -max_velocity;
            motor->command.velocity = velocity_cmd;
        }
    }
}

// void A1Motor_Smooth_Position_Control(A1Motor *motor, float target_position,
//                                      uint32_t total_time_ms, float kw, float max_velocity)
// {
//     motor->command.kw = kw;   // 速度刚度系数

//     float error = target_position - motor->feedback.Position_Sum;

//     // 你要求的逻辑：差值大于0.1就最大速度，否则停止
//     if (fabsf(error) > 0.1f)
//     {
//         motor->command.velocity = (error > 0) ? max_velocity : -max_velocity;
//     }
//     else
//     {
//         motor->command.velocity = 0;
//     }
// }
