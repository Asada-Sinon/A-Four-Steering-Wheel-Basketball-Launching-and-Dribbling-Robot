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

#endif
