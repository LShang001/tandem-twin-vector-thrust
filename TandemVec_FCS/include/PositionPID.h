#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <cmath> // For std::fabs

/**
 * @brief 位置式PID控制器类 (改进版)
 *
 * 实现了位置式PID控制算法，包括比例(P)、积分(I)和微分(D)项。
 * 特点：
 * 1. 统一的低通滤波处理，对所有微分项都进行滤波
 * 2. 积分分离功能，防止积分饱和 (当 integralThreshold_ = 0 时禁用)
 * 3. 积分限幅和输出限幅
 * 4. 支持外部微分输入（如陀螺仪数据）
 * 5. 支持微分先行（Derivative on Measurement）
 */
class PositionPID
{
public:
    /**
     * @brief 构造函数
     *
     * @param kp                比例系数
     * @param ki                积分系数
     * @param kd                微分系数
     * @param minOutput         输出下限
     * @param maxOutput         输出上限
     * @param enableIntegral    是否启用积分项
     * @param integralLimit     积分限幅值 (指积分项本身的限幅，不是累积值的原始限幅)
     * @param integralThreshold 积分分离阈值 (误差绝对值)，设置为0时禁用积分分离
     * @param filterCoefficient 微分项滤波系数，范围(0,1)，越接近1滤波效果越弱(越接近0滤波越强)
     */
    PositionPID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                float minOutput = -100.0f, float maxOutput = 100.0f,
                bool enableIntegral = true, float integralLimit = 250.0f,
                float integralThreshold = 0.0f, float filterCoefficient = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0f), previousError_(0.0f), previousInput_(0.0f), // 初始化 previousInput_
          previousDerivative_(0.0f),
          outputMin_(minOutput), outputMax_(maxOutput),
          enableIntegral_(enableIntegral), integralLimit_(integralLimit),
          integralThreshold_(integralThreshold), filterCoefficient_(filterCoefficient)
    {
        // 确保滤波系数在有效范围内
        setFilterCoefficient(filterCoefficient_);
    }

    /**
     * 设置PID控制器的参数
     *
     * @param kp 比例增益，决定比例项的反应速度
     * @param ki 积分增益，决定积分项的累积效应
     * @param kd 微分增益，决定微分项的预测效果
     *
     * 此函数用于配置PID控制器的比例、积分和微分三个核心参数，从而达到预期的控制性能
     */
    void setParams(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    /**
     * @brief 设置输出限幅
     */
    void setOutputLimits(float minOutput, float maxOutput)
    {
        outputMin_ = minOutput;
        outputMax_ = maxOutput;
    }

    /**
     * @brief 设置积分项启用状态
     */
    void setIntegralEnable(bool enable)
    {
        enableIntegral_ = enable;
        if (!enableIntegral_)
        {
            integral_ = 0.0f;
        }
    }

    /**
     * @brief 设置积分项输出的绝对限幅值
     * @param limit 积分项允许贡献的最大绝对值
     */
    void setIntegralLimit(float limit)
    {
        integralLimit_ = std::fabs(limit); // 确保是正值
    }

    /**
     * @brief 设置积分分离阈值, 设置为0时禁用积分分离
     * @param threshold 误差绝对值的阈值，小于此值才进行积分
     */
    void setIntegralThreshold(float threshold)
    {
        integralThreshold_ = std::fabs(threshold); // 确保是正值
    }

    /**
     * @brief 设置微分项滤波系数，并进行限幅处理
     *
     * 该函数用于设置微分项的滤波系数，同时对输入的系数进行限幅处理，
     * 确保其值在合理范围内（0 到 1 之间）。如果输入的系数超出此范围，
     * 将自动调整为边界值。
     *
     * @param coefficient 微分项滤波系数，越小滤波效果越强 (0 < coefficient <= 1)。接近0时滤波最强，为1时不滤波。设置为0或负数时禁用滤波。
     */
    void setFilterCoefficient(float coefficient)
    {
        // 滤波系数 α (alpha) 通常在 (0, 1] 范围内
        // α = Ts / (τ + Ts)  其中 τ 是时间常数
        // 这里 filterCoefficient_ 对应 α
        // 如果 coefficient <= 0，则禁用滤波 (等效于 α = 1)
        if (coefficient <= 0.0f)
        {
            filterCoefficient_ = 1.0f; // 不滤波
        }
        else if (coefficient > 1.0f)
        {
            filterCoefficient_ = 1.0f; // 不滤波
        }
        else
        {
            filterCoefficient_ = coefficient; // 正常设置
        }
    }

    /**
     * @brief 重置PID控制器状态
     */
    void reset()
    {
        integral_ = 0.0f;
        previousError_ = 0.0f;
        previousInput_ = 0.0f; // 重置 previousInput_
        previousDerivative_ = 0.0f;
    }

    /**
     * @brief 使用误差计算PID输出（标准方法，微分基于误差变化）
     *
     * @param setpoint 设定值
     * @param input    当前测量值
     * @return float   PID输出
     */
    float compute(float setpoint, float input)
    {
        float error = setpoint - input;
        // 微分项基于误差的变化率
        float derivative = error - previousError_;

        // 对计算得到的微分项进行滤波
        float filteredDerivative = filterDerivative(derivative);

        // 计算PID输出
        float output = computePID(error, filteredDerivative);

        // 更新状态
        previousError_ = error;
        previousInput_ = input; // 也更新 previousInput_，以便下次切换使用

        return output;
    }

    /**
 * @brief 使用外部微分值计算PID输出（适用于有外部传感器如陀螺仪的情况）
 *
 * @param setpoint   设定值，例如目标角度
 * @param input      当前测量值，例如当前角度
 * @param derivative 外部传入的微分值（例如，角速度，角度的导数）
 * @return float     PID输出
 */
float computeWithExternalDerivative(float setpoint, float input, float derivative)
{
    // 计算误差：目标值减去当前测量值
    float error = setpoint - input;
    // 对外部传入的微分值进行处理
    // 因为误差 e = setpoint - input，误差的微分 de/dt = -d(input)/dt
    // 而 derivative 是 d(input)/dt（角速度），因此需要取反
    float filteredDerivative = filterDerivative(-derivative);
    // 计算PID输出
    float output = computePID(error, filteredDerivative);
    // 更新状态，供下一次计算使用
    previousError_ = error;
    previousInput_ = input;
    return output;
}

    /**
     * @brief 使用微分先行计算PID输出（微分基于测量值变化）
     *
     * 这种方法可以减少设定点(setpoint)变化时引起的微分项“冲击”(Derivative Kick)。
     *
     * @param setpoint 设定值
     * @param input    当前测量值
     * @return float   PID输出
     */
    float computeDerivativeOnMeasurement(float setpoint, float input)
    {
        float error = setpoint - input;
        // 微分项基于测量值的变化率（注意符号）
        // dInput/dt = (input - previousInput) / dt
        // 使用 -dInput/dt 作为微分项输入，与 dError/dt 在setpoint不变时符号一致
        float derivative = previousInput_ - input; // 使用 -delta(Input)

        // 对计算得到的微分项进行滤波
        float filteredDerivative = filterDerivative(derivative);

        // 计算PID输出
        float output = computePID(error, filteredDerivative);

        // 更新状态
        previousError_ = error;
        previousInput_ = input; // 必须更新 previousInput_

        return output;
    }

private:
    /**
     * @brief 对当前导数进行滤波处理 (一阶低通滤波器)
     *
     * y[n] = α * x[n] + (1 - α) * y[n-1]
     * 其中 α 是 filterCoefficient_
     *
     * @param currentDerivative 当前计算得到的原始导数值 (x[n])
     * @return float 滤波后的导数值 (y[n])
     */
    float filterDerivative(float currentDerivative)
    {
        // filterCoefficient_ == 1.0f 意味着不滤波
        if (filterCoefficient_ >= 1.0f)
        {
            previousDerivative_ = currentDerivative; // 仍然更新 P.D. 以备切换滤波模式
            return currentDerivative;
        }

        // 计算滤波后的导数值
        // α * x[n] + (1 - α) * y[n-1]
        float filteredValue = filterCoefficient_ * currentDerivative +
                              (1.0f - filterCoefficient_) * previousDerivative_;
        // 更新上一次滤波后的导数值 (y[n-1] for next iteration)
        previousDerivative_ = filteredValue;
        // 返回滤波后的导数值
        return filteredValue;
    }

    /**
     * @brief PID核心计算函数
     *
     * @param error      当前误差 (Setpoint - Input)
     * @param derivative 当前（已滤波的）微分项值
     * @return float     未限幅的PID总输出
     */
    float computePID(float error, float derivative)
    {
        // 比例项
        float pOut = kp_ * error;

        // 积分项
        float iOut = 0.0f;
        if (enableIntegral_ && ki_ != 0.0f) // 只有启用且ki非零时才计算
        {
            // 积分分离逻辑
            if (integralThreshold_ <= 0.0f || std::fabs(error) < integralThreshold_)
            {
                // 累积误差，注意这里积分的是误差本身，乘以 dt 在实际离散应用中常省略或合并到 Ki
                // (假设 compute 被以固定周期 dt 调用, Ki 包含了 dt)
                integral_ += error;

                // 积分限幅（限制累积值 integral_ 本身）
                // 这样可以防止 integral_ 无限增大，即使输出被限幅
                // 注意：这里的限幅与 setIntegralLimit 的意图可能不同，
                // setIntegralLimit 通常限制的是 iOut = ki * integral_ 的绝对值。
                // 修正：将积分项输出限幅放在后面，这里仅累加。
                // 保留一个对 integral_ 自身的原始限幅可能也有用，防止溢出，
                // 但更常见的是限制最终的 iOut。
                // 我们将 iOut 限幅。

            } // 如果误差大于阈值，则不累加积分 (积分分离)

            // 计算积分输出，并进行限幅
            iOut = ki_ * integral_;
            iOut = constrain(iOut, -integralLimit_, integralLimit_);

            // 抗积分饱和 (Anti-windup): 如果总输出将被限幅，且当前输出方向与积分项方向相同，
            // 则可能需要停止积分甚至反向积分。这里采用简单的方式：仅对 iOut 进行限幅。
            // 更复杂的 Anti-windup 策略可以在这里添加。
        }
        else
        {
            integral_ = 0.0f; // 如果禁用积分或 Ki 为 0，重置积分累积值
        }

        // 微分项
        float dOut = kd_ * derivative;

        // 计算总输出
        float output = pOut + iOut + dOut;

        // 对总输出进行限幅
        output = constrain(output, outputMin_, outputMax_);

        // (可选) 如果需要更精细的抗饱和，可以在这里检查 output 是否被限幅，
        // 并根据情况调整 integral_ 的累积。例如，如果 output 被限幅，
        // 且 (output - (pOut + dOut)) 和 error 符号相同，则停止积分。

        return output;
    }

    // PID参数
    float kp_; ///< 比例系数
    float ki_; ///< 积分系数
    float kd_; ///< 微分系数

    // 控制器状态
    float integral_;           ///< 积分累积值 (表示 sum(error * dt)，dt常合并入Ki)
    float previousError_;      ///< 上一次误差
    float previousInput_;      ///< 上一次输入值 (用于微分先行)
    float previousDerivative_; ///< 上一次滤波后的微分值 (y[n-1] for filter)

    // 限制与配置参数
    float outputMin_;         ///< 输出下限
    float outputMax_;         ///< 输出上限
    bool enableIntegral_;     ///< 积分使能标志
    float integralLimit_;     ///< 积分项输出绝对值限幅 (limit for |iOut|)
    float integralThreshold_; ///< 积分分离阈值 (|error| < threshold to integrate)
    float filterCoefficient_; ///< 微分项滤波系数 α (alpha)
};

#endif // POSITION_PID_H