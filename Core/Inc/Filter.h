#ifndef __FILTER__
#define __FILTER__

#include <stdint.h>
// 低通滤波器结构体
typedef struct s_LPFilter{
    float weight;        // 滤波权重系数
    uint8_t start;       // 初始化标志位
    float pastValue;     // 上一次的滤波输出值
}s_LPFilter;

void LPFilter_Init(s_LPFilter *lpf, float samplePeriod, float cutFrequency);
float LPFilter_Process(s_LPFilter *lpf, float newValue);
void LPF_clear(s_LPFilter* lpf);

 typedef struct kalman_par {
    float q;        // 过程噪声协方差
    float r1;       // 传感器1的测量噪声协方差
    float r2;       // 传感器2的测量噪声协方差
    float p_;       // 先验估计协方差
    float p;        // 后验估计协方差
    float x;        // 后验状态估计（最终输出）
    float x_;       // 先验状态估计
    float k1, k2;   // 两个传感器的卡尔曼增益
} KalmanFilter;

void kalman_init(KalmanFilter* kf, float q, float r1, float r2, float initial_value);
void kalman_update(KalmanFilter* kf, float z1, float z2); 
#endif
