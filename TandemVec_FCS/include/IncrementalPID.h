/**
 * @file    IncrementalPID.h
 * @module  固件/控制（增量式 PID，历史实现）
 * @brief   增量式 PID：Δu(k) 递推后累加成绝对输出，带输出限幅
 * @details 输入：设定值与反馈值（同一单位）；处理：一阶/二阶误差差分 → Δu；
 *          输出：u(k) = u(k−1) + Δu(k)，钳到 [out_min, out_max] 并回写历史。
 *          与 PositionPID（位置式 v3）并存：本类无积分状态、无微分滤波、
 *          无抗饱和，仅用于不需要这些特性的历史调用点；新代码优先 PositionPID。
 * @theory  增量式 PID 递推式（数字控制标准形式，非本项目理论推导）：
 *          Δu(k) = Kp·Δe(k) + Ki·e(k) + Kd·Δ²e(k)
 * @frame   N/A（标量控制器，输入输出同一物理量纲）
 * @units   由调用方决定：本项目姿态环用 deg / deg/s / deg/s²；
 *          Ki 隐含采样周期（离散增益），换调用频率必须重算
 * @note   ① 采样周期不出现在公式里：Ki/Kd 已把 Ts 吸收进去，改 200 Hz → 其他
 *          频率时增益必须按频率重新整定（与 PositionPID 的连续域 Ki 不同）。
 *        ② reset() 只清误差历史，不清 last_output_（绝对输出保持上一拍），
 *          这是增量式的固有语义：重置后输出不会跳回 0。
 * @see     PositionPID.h（位置式，连续域 Ki，带无扰切换/抗饱和/NaN 防护）
 */
#ifndef INCREMENTAL_PID_H
#define INCREMENTAL_PID_H

/**
 * @brief 增量式 PID 控制器（构造时给增益，输出按拍累加）
 * @note 构造参数 kp/ki/kd 的物理量纲由调用方决定；默认输出限幅 0–100。
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
 * @brief 单步增量式 PID 计算
 * @param[in] setpoint 设定值（与 input 同量纲）
 * @param[in] input    反馈值
 * @return 累积后的绝对输出 u(k)，已限幅到 [out_min_, out_max_]
 * @formula Δu(k) = Kp·Δe(k) + Ki·e(k) + Kd·Δ²e(k)；
 *          u(k) = clamp(u(k−1) + Δu(k), out_min, out_max)
 * @pre  kp/ki/kd 已按采样频率整定（离散增益，隐含 Ts）
 * @post 历史误差 e(k−1)/e(k−2) 与 last_output_ 均更新为本拍值
 * @note  Ki 项直接取当前误差 e(k)（非误差差分），因此稳态需要持续误差才维持
 *        输出增量——这与位置式 PID 的积分状态等价，但没有任何积分限幅，
 *        长时间大误差会靠输出限幅兜底（可能残留积分）。
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
