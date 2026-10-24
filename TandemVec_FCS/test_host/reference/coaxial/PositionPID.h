/**
 * @file    PositionPID.h
 * @module  固件/宿主参考（实飞共轴工程 PositionPID，逐字复制）
 * @brief   外部实飞共轴工程的 PositionPID 原实现，仅用于输出级参考对比
 * @details 逐字复制自 VTVL_ElectricDualRotor_FCS/include/PositionPID.h；
 *          不参与固件编译，不修改算法。宿主机编译需 Arduino 桩提供 constrain。
 * @theory  PID with derivative-on-measurement / external derivative
 *          @ref CTL-003 §2.2
 * @frame   N/A
 * @units   N/A
 * @note    仅测试用；实飞参数/限幅由 coax_reference.cpp 配置。
 */
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
          previousDerivative_(0.0f), lastOutput_(0.0f), nonFiniteCount_(0),
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
        lastOutput_ = 0.0f;
        // 注：nonFiniteCount_ 刻意不清零——跨模式切换后仍需保留非有限输入故障史
        // （与 TandemVec v3 一致）；如需归零显式调 clearNonFiniteCount()。
    }

    /**
     * @brief 钳制积分状态本身（回植 TandemVec v3 缺陷 #2 修复）
     *
     * 把 integral_ 限制在 |integralLimit_/ki_| 内，使 ki_·integral_ 恒不超过
     * 积分输出限幅。修复前 integral_ 无界，误差反向时需先卸掉巨量积分才响应
     * （integral windup，退饱和迟滞可达数秒）。
     * 稳态 iOut 与修复前一致（钳位处 ki_·bound = integralLimit_），故为等价改进。
     */
    void clampIntegralState()
    {
        if (ki_ == 0.0f)
        {
            integral_ = 0.0f;
            return;
        }
        const float bound = std::fabs(integralLimit_ / ki_);
        integral_ = constrain(integral_, -bound, bound);
    }

    /**
     * @brief 健康诊断：累计拦截的 NaN/Inf 输入次数
     *
     * 正常应恒为 0。非零表示上游（AHRS/陀螺/设定点/速度源）出现过非有限值，
     * 控制器已冻结对应拍输出保护自身，但上游故障需排查。
     */
    unsigned long getNonFiniteCount() const { return nonFiniteCount_; }
    void clearNonFiniteCount() { nonFiniteCount_ = 0; }

    /**
     * @brief 使用误差计算PID输出（标准方法，微分基于误差变化）
     *
     * @param setpoint 设定值
     * @param input    当前测量值
     * @param dt       控制周期（秒），积分用
     * @return float   PID输出
     */
    float compute(float setpoint, float input, float dt)
    {
        // ★ NaN/Inf 防护（回植自 TandemVec v3）：入口拦截非有限输入，冻结本拍
        //   返回上一拍输出。否则一个 NaN 会同时污染微分滤波器（filterDerivative
        //   写 previousDerivative_）与误差历史，此后输出恒 NaN 且 reset 前无法自愈。
        //   对所有有限输入行为完全不变。
        if (!isFiniteF(setpoint) || !isFiniteF(input))
        {
            ++nonFiniteCount_;
            return lastOutput_;
        }
        float error = setpoint - input;
        // 微分项基于误差的变化率
        float derivative = error - previousError_;

        // 对计算得到的微分项进行滤波
        float filteredDerivative = filterDerivative(derivative);

        // 计算PID输出
        float output = computePID(error, filteredDerivative, dt);

        // 更新状态
        previousError_ = error;
        previousInput_ = input; // 也更新 previousInput_，以便下次切换使用
        lastOutput_ = output;

        return output;
    }

    /**
 * @brief 使用外部微分值计算PID输出（适用于有外部传感器如陀螺仪的情况）
 *
 * @param setpoint   设定值，例如目标角度
 * @param input      当前测量值，例如当前角度
 * @param derivative 外部传入的微分值（例如，角速度，角度的导数）
 * @param dt         控制周期（秒），积分用
 * @return float     PID输出
 */
float computeWithExternalDerivative(float setpoint, float input, float derivative, float dt)
{
    // ★ NaN/Inf 防护（回植自 TandemVec v3）：setpoint/input/derivative 任一非有限
    //   即冻结本拍，避免 NaN 经 filterDerivative 永久污染微分滤波器（对有限输入不变）。
    if (!isFiniteF(setpoint) || !isFiniteF(input) || !isFiniteF(derivative))
    {
        ++nonFiniteCount_;
        return lastOutput_;
    }
    // 计算误差：目标值减去当前测量值
    float error = setpoint - input;
    // 对外部传入的微分值进行处理
    // 因为误差 e = setpoint - input，误差的微分 de/dt = -d(input)/dt
    // 而 derivative 是 d(input)/dt（角速度），因此需要取反
    float filteredDerivative = filterDerivative(-derivative);
    // 计算PID输出
    float output = computePID(error, filteredDerivative, dt);
    // 更新状态，供下一次计算使用
    previousError_ = error;
    previousInput_ = input;
    lastOutput_ = output;
    return output;
}

    /**
     * @brief 使用微分先行计算PID输出（微分基于测量值变化）
     *
     * 这种方法可以减少设定点(setpoint)变化时引起的微分项“冲击”(Derivative Kick)。
     *
     * @param setpoint 设定值
     * @param input    当前测量值
     * @param dt       控制周期（秒），积分用
     * @return float   PID输出
     */
    float computeDerivativeOnMeasurement(float setpoint, float input, float dt)
    {
        // ★ NaN/Inf 防护（回植自 TandemVec v3）：冻结本拍，不污染微分滤波/误差历史。
        if (!isFiniteF(setpoint) || !isFiniteF(input))
        {
            ++nonFiniteCount_;
            return lastOutput_;
        }
        float error = setpoint - input;
        // 微分项基于测量值的变化率（注意符号）
        // dInput/dt = (input - previousInput) / dt
        // 使用 -dInput/dt 作为微分项输入，与 dError/dt 在setpoint不变时符号一致
        float derivative = previousInput_ - input; // 使用 -delta(Input)

        // 对计算得到的微分项进行滤波
        float filteredDerivative = filterDerivative(derivative);

        // 计算PID输出
        float output = computePID(error, filteredDerivative, dt);

        // 更新状态
        previousError_ = error;
        previousInput_ = input; // 必须更新 previousInput_
        lastOutput_ = output;

        return output;
    }

private:
    /**
     * @brief 浮点数有限性检查（不依赖 std::isfinite 的 double 提升，嵌入式友好）
     *
     * NaN 自比较为假；±Inf 绝对值大于任意有限阈值。
     */
    static bool isFiniteF(float v)
    {
        return (v == v) && (v < 3.0e38f) && (v > -3.0e38f);
    }

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
     * @param dt         控制周期（秒）；积分按 integral_ += error*dt 累加，
     *                   ki_ 为连续域增益（I = ki_·∫error dt），与调用频率解耦
     * @return float     未限幅的PID总输出
     */
    float computePID(float error, float derivative, float dt)
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
                // 累积误差 × dt（连续域积分）：ki_ 为连续域增益，与控制频率解耦。
                // 旧约定 integral_ += error（ki_ 隐含调用周期 dt），2026-09 迁移自
                // TandemVec v3；现役增益已按 ki_cont = ki_old/dt 同步换算（dt=0.005s）。
                integral_ += error * dt;

            } // 如果误差大于阈值，则不累加积分 (积分分离)

            // ★ 2026-09-01 修复（回植 TandemVec v3 缺陷 #2）：钳制【积分状态本身】，
            //   而非仅钳制其输出贡献。原实现只做 iOut=constrain(ki_*integral_)，
            //   integral_ 可无限增长——yaw 环 ki=0.15/int_limit=50 下状态上界 333，
            //   持续 20 deg/s 误差 16.7s 即饱和，误差反向时必须先卸掉这个巨量才能
            //   响应（典型 integral windup，退饱和迟滞可达数秒）。
            //   钳位后稳态 iOut 与修复前一致（ki_·limit/ki_ = limit），退饱和立即响应。
            clampIntegralState();
            // 计算积分输出（数值安全兜底：clampIntegralState 后已天然 ≤ limit）
            iOut = ki_ * integral_;
            iOut = constrain(iOut, -integralLimit_, integralLimit_);
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
    float lastOutput_;         ///< 上一拍输出（NaN/Inf 冻结时返回）

    // 限制与配置参数
    float outputMin_;         ///< 输出下限
    float outputMax_;         ///< 输出上限
    bool enableIntegral_;     ///< 积分使能标志
    float integralLimit_;     ///< 积分项输出绝对值限幅 (limit for |iOut|)
    float integralThreshold_; ///< 积分分离阈值 (|error| < threshold to integrate)
    float filterCoefficient_; ///< 微分项滤波系数 α (alpha)

    // 健康诊断
    unsigned long nonFiniteCount_; ///< 累计拦截的 NaN/Inf 输入次数（遥测/排障用）
};

#endif // POSITION_PID_H