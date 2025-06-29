#ifndef _A1_MOTOR_H
#define _A1_MOTOR_H

#include "stdint.h"
#include "usart.h"
#include "myusart.h"

/* 通信协议常量定义 */
#define A1_PACKET_HEADER_0   0xFE
#define A1_PACKET_HEADER_1   0xEE
#define A1_PACKET_SIZE_TX    34   // 发送数据包大小
#define A1_PACKET_SIZE_RX    78   // 接收数据包大小

/* 电机工作模式 */
#define A1_MODE_IDLE         0    // 空闲模式
#define A1_MODE_OPEN_LOOP    5    // 开环控制
#define A1_MODE_FOC          10   // 闭环FOC控制

/* 单位转换常量 */
#define A1_TORQUE_SCALE      256.0f    // 力矩缩放因子 (Q8格式)
#define A1_VELOCITY_SCALE    128.0f    // 速度缩放因子 (Q7格式)
#define A1_POSITION_SCALE    (16384.0f/6.2832f)  // 位置缩放因子 (编码器计数/弧度)
#define A1_KP_SCALE          2048.0f   // 位置刚度缩放因子 (Q11格式)
#define A1_KW_SCALE          1024.0f   // 速度刚度缩放因子 (Q10格式)
#define A1_GEAR_RATIO        9.0f      // 减速比

typedef enum
{
    A1_MOTOR_POSITION_BOTTOM = 215,          // 丝杠拉到最下面的位置
    A1_MOTOR_POSITION_TOP = 37354,   // 丝杠拉到最上面的位置
    A1_MOTOR_POSITION_THREE_POINTE = 3855 // 丝杠拉到三分线投球的位置
} A1MotorPositionType;

/**
 * A1电机控制参数结构体 - 用于发送指令给电机
 */
typedef struct {
    uint8_t id;       // 电机ID (0~254, 0xFF为广播)
    uint8_t mode;     // 控制模式 (0:空闲, 5:开环, 10:闭环FOC)
    float torque;     // 力矩 (Nm)
    float velocity;   // 角速度 (rad/s)
    float position;   // 位置 (rad)
    float kp;         // 位置刚度系数
    float kw;         // 速度刚度系数
} A1MotorCommand;

/**
 * A1电机反馈参数结构体 - 用于接收电机状态
 */
typedef struct {
    float torque;          // 实际力矩 (Nm)
    float velocity;        // 实际角速度 (rad/s)
    float position;        // 实际位置 (rad)
    float Position_Last;      // 角度累积 (rad)
    float Position_Sum; // 位置累积 (rad)
    int8_t temperature;    // 温度 (°C)
    int16_t acceleration;  // 加速度
    uint8_t error_code;    // 错误代码
} A1MotorFeedback;

/**
 * 电机通信管理器结构体
 */
typedef struct {
    A1MotorCommand command;    // 电机控制命令
    A1MotorFeedback feedback;  // 电机状态反馈
} A1Motor;

/* 函数原型声明 */
void A1Motor_Smooth_Position_Control(A1Motor *motor, float target_position, 
                                     uint32_t total_time_ms, float kw, float max_velocity);
void A1Motor_SendCommand(UART_HandleTypeDef *huart, A1MotorCommand *motor_cmd, GPIO_TypeDef* gpio, uint16_t gpio_pin);
A1MotorFeedback A1Motor_ProcessFeedback(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_rx);
float A1_Angle_Splitting(uint32_t total_time_ms, float target_angle);
void A1Motor_distance_mode(A1Motor *motor, float distance, float kw, float v);
void A1Motor_Restart(void);

/* 全局变量声明 */
extern A1Motor g_a1motor;  // 全局电机实例
extern A1MotorFeedback A1_feedback;
#endif /* _A1_MOTOR_H */

