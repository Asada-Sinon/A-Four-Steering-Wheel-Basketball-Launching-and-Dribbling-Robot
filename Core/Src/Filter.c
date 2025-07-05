#include "Filter.h"
#include <stdint.h>

// 圆周率定义，用于滤波器权重计算
#define PI 3.14159265358979f

/*********************************************************************************
 * @name   LPFilter_init
 * @brief  低通滤波器初始化函数
 * @param  lpf: 指向低通滤波器结构体的指针
 * @param  samplePeriod: 采样周期(秒) - 系统调用滤波器的时间间隔
 * @param  cutFrequency: 截止频率(Hz) - 高于此频率的信号将被衰减
 * @retval 无
 * @note   截止频率越高，滤波器响应越快，但去噪效果越弱
 *         截止频率越低，滤波器响应越慢，但去噪效果越强
 * @example:
 *         // 2ms采样周期，10Hz截止频率的滤波器
 *         LPFilter_init(&my_filter, 0.002f, 10.0f);
 *********************************************************************************/
void LPFilter_Init(s_LPFilter *lpf, float samplePeriod, float cutFrequency)
{
    // 计算滤波器权重系数
    // 公式：α = 1 / (1 + 1/(2π × T × fc))
    // 其中：T = 采样周期, fc = 截止频率
    // 权重范围 [0,1]: 越接近1响应越快，越接近0滤波越强
    float denominator = 2.0f * PI * samplePeriod * cutFrequency;
    lpf->weight = 1.0f / (1.0f + 1.0f / denominator);

    // 初始化启动标志为0，表示滤波器尚未开始工作
    // 首次调用LPFilter()时会进行特殊初始化处理
    lpf->start = 0;
    lpf->pastValue = 0.0f;
}

/*********************************************************************************
 * @name   LPFilter_Process
 * @brief  低通滤波器处理函数
 * @param  lpf: 指向低通滤波器结构体的指针
 * @param  newValue: 新输入值 - 需要进行滤波的原始数据
 * @retval 返回滤波后的输出值
 * @note   首次调用时会自动初始化pastValue为newValue，避免初始偏差
 *         后续调用会根据权重系数进行加权平均计算
 * @example:
 *         float filtered_value = LPFilter_Process(&my_filter, raw_value);
 *********************************************************************************/
float LPFilter_Process(s_LPFilter *lpf, float newValue)
{
    // 首次调用检查：如果滤波器还未启动
    if (!lpf->start)
    {
        // 设置启动标志，表示滤波器已经开始工作
        lpf->start = 1;

        // 首次调用时，直接将输入值作为初始滤波输出
        // 这样避免了从0开始可能导致的初始偏差
        lpf->pastValue = newValue;
    }

    // 低通滤波核心算法：加权平均
    // pastValue = weight × newValue + (1-weight) × pastValue
    // 新数据权重 + 历史数据权重 = 100%
    // weight越大：新数据影响越大，响应越快，滤波效果越弱
    // weight越小：历史数据影响越大，响应越慢，滤波效果越强
    lpf->pastValue = lpf->weight * newValue + (1.0f - lpf->weight) * lpf->pastValue;
    return lpf->pastValue;
}

/*********************************************************************************
 * @name   LPF_clear
 * @brief  清除低通滤波器状态，重置为初始状态
 * @param  lpf: 指向低通滤波器结构体的指针
 * @retval 无
 * @note   调用此函数后，滤波器会忘记所有历史数据
 *         下次调用LPFilter()时会重新进行首次初始化
 * @use_case:
 *         - 传感器重新标定后
 *         - 系统模式切换时
 *         - 检测到数据跳变需要重新开始滤波时
 * @example:
 *         if (sensor_error_detected) {
 *             LPF_clear(&my_filter);  // 清除可能的错误历史数据
 *         }
 *********************************************************************************/
void LPF_clear(s_LPFilter *lpf)
{
    // 重置启动标志，下次调用LPFilter()时会重新初始化
    lpf->start = 0;

    // 清除历史滤波值，避免残留数据影响
    // 虽然下次LPFilter()会重新设置这个值，但清零更安全
    lpf->pastValue = 0.0f;
}

/*********************************************************************************
 * 使用示例和最佳实践：
 *
 * 1. 参数选择建议：
 *    - 角度控制：截止频率 10-20Hz（需要较快响应）
 *    - 速度控制：截止频率 5-15Hz（平衡响应与平滑）
 *    - 距离测量：截止频率 1-5Hz（噪声大，需要强滤波）
 *
 * 2. 注意事项：
 *    - 采样周期必须与实际调用频率一致
 *    - 截止频率不能超过奈奎斯特频率（采样频率的一半）
 *    - 滤波会引入相位延迟，高精度控制时需要考虑
 *********************************************************************************/


void kalman_init(KalmanFilter* kf, float q, float r1, float r2, float initial_value) {
    kf->q = q;
    kf->r1 = r1;
    kf->r2 = r2;
    kf->p = 1.0f;   // 初始后验协方差（可调整）
    kf->x = initial_value;
    kf->x_ = 0.0f;
    kf->p_ = 0.0f;
    kf->k1 = kf->k2 = 0.0f;
}

void kalman_filter_update(KalmanFilter* kf, float data1, float data2) {
    // 1. 预测阶段（时间更新）
    kf->x_ = kf->x;                 // 状态预测（假设系统模型为 x = x_prev）
    kf->p_ = kf->p + kf->q;         // 协方差预测

    // 2. 更新阶段（测量更新）
    // 传感器1更新
    kf->k1 = kf->p_ / (kf->p_ + kf->r1);
    kf->x = kf->x_ + kf->k1 * (data1 - kf->x_);
    kf->p = (1.0f - kf->k1) * kf->p_;

    // 传感器2更新（基于传感器1更新后的结果）
    kf->k2 = kf->p / (kf->p + kf->r2);
    kf->x = kf->x + kf->k2 * (data2 - kf->x);
    kf->p = (1.0f - kf->k2) * kf->p;
}

