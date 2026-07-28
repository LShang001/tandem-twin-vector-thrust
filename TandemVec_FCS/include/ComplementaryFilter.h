// ComplementaryFilter.h
#ifndef ComplementaryFilter_h
#define ComplementaryFilter_h

#include "Arduino.h"

/**
 * @brief 互补滤波器类，用于对传感器数据进行滤波处理。
 *
 * 互补滤波器结合了两种不同特性的传感器数据。例如，它可以将加速度计的短期
 * 响应与陀螺仪的长期稳定性结合起来，以获得更准确的姿态估计。
 */
class ComplementaryFilter
{
private:
    float alpha;             /**< 滤波系数，取值范围为 [0, 1]。alpha 越接近 1，滤波器越信任当前输入值，响应越快，但滤波效果越弱；alpha 越接近 0，滤波器越信任上一次的滤波结果，响应越慢，但滤波效果越强。*/
    float lastFilteredValue; /**< 上一次的滤波结果，用于递归计算新的滤波值。*/

public:
    /**
     * @brief 构造函数
     *
     * @param filterCoefficient 滤波系数，取值范围为 [0, 1]，建议的典型值为 0.9 - 0.999。
     *                          如果传入的值不在 [0, 1] 范围内，将被限制在 [0, 1] 内。
     * @param initialValue      可选参数，用于初始化滤波器的初始值。如果未提供，则初始值为 0。
     */
    ComplementaryFilter(float filterCoefficient, float initialValue = 0.0f)
    {
        // 限制 alpha 的取值范围为 [0, 1]
        alpha = constrain(filterCoefficient, 0.0f, 1.0f);
        lastFilteredValue = initialValue;
    }

    /**
     * @brief 对新的输入值进行滤波处理。
     *
     * @param currentValue 新的输入值。
     * @return 滤波后的值。
     */
    float filter(float currentValue)
    {
        // 使用互补滤波器的公式进行滤波
        lastFilteredValue = alpha * currentValue + (1 - alpha) * lastFilteredValue;
        return lastFilteredValue;
    }

    /**
     * @brief 使用第一个输入值初始化滤波器。
     *
     * 这可以避免初始滤波结果偏向 0 的问题。
     *
     * @param firstValue 第一个输入值，用于初始化 lastFilteredValue。
     */
    void initialize(float firstValue)
    {
        lastFilteredValue = firstValue;
    }

    /**
     * @brief 重置滤波器状态。
     *
     * 将上一次的滤波结果重置为 0。
     */
    void reset()
    {
        lastFilteredValue = 0;
    }
};

#endif
