#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

/**
 * 增量式PID控制器类
 * 提供增量式PID控制算法的实现，包括PID参数设置、输出限幅设置和PID控制器状态重置等功能
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
class IncrementalPID
{
public:
    IncrementalPID(float kp = 0.0, float ki = 0.0, float kd = 0.0); // 构造函数,初始化PID参数
    void setParams(float kp, float ki, float kd);                   // 设置PID参数
    void setOutputLimits(float min, float max);                     // 设置输出限幅
    void reset();                                                   // 重置PID控制器的内部状态
    float compute(float setpoint, float input);                     // 计算PID控制器输出

private:
    float kp_; // 比例系数Kp
    float ki_; // 积分系数Ki
    float kd_; // 微分系数Kd

    float last_error_; // 上一次误差e(k-1)
    float prev_error_; // 上上次误差e(k-2)

    float last_output_; // 上一次的控制输出u(k-1)

    float output_min_; // 输出下限
    float output_max_; // 输出上限
};

/**
 * 构造函数,初始化PID参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
IncrementalPID::IncrementalPID(float kp, float ki, float kd)
{
    setParams(kp, ki, kd);

    last_error_ = prev_error_ = 0.0;
    last_output_ = 0.0; // 初始化上一次的控制输出
    output_min_ = 0;
    output_max_ = 100;
}

/**
 * 设置PID参数
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void IncrementalPID::setParams(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/**
 * 设置输出限幅
 * @param min 输出下限
 * @param max 输出上限
 */
void IncrementalPID::setOutputLimits(float min, float max)
{
    output_min_ = min;
    output_max_ = max;
}

/**
 * 重置PID控制器的内部状态
 * 调用此方法可以清除积累的误差和历史误差，通常在重新设置PID参数后使用
 */
void IncrementalPID::reset()
{
    last_error_ = 0.0; // 清零上一次的误差
    prev_error_ = 0.0; // 清零上上次的误差
}

/**
 * 计算增量式PID控制器输出
 * @param setpoint 设定值
 * @param input 反馈值
 * @return PID输出量u(k)
 */
float IncrementalPID::compute(float setpoint, float input)
{
    float error = setpoint - input;

    // 增量式PID核心算法
    float p_error = error - last_error_;                                // Δe(k) = e(k) - e(k-1)
    float i_error = error;                                              // 积分项直接使用当前误差
    float d_error = error - 2 * last_error_ + prev_error_;              // Δ²e(k) = e(k) - 2*e(k-1) + e(k-2)
    float delta_output = kp_ * p_error + ki_ * i_error + kd_ * d_error; // Δu(k) = Kp*Δe(k) + Ki*e(k) + Kd*Δ²e(k)

    // 计算累积的控制量
    float output = delta_output + last_output_; // u(k) = Δu(k) + u(k-1)

    // 限制输出范围
    if (output > output_max_)
    {
        output = output_max_;
    }
    else if (output < output_min_)
    {
        output = output_min_;
    }

    // 更新历史数据
    prev_error_ = last_error_;
    last_error_ = error;
    last_output_ = output; // 更新上一次的控制输出

    return output;
}

#endif // INCREMENTAL_PID_H
