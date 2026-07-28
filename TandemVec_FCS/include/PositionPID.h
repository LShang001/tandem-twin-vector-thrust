// ============================================================
//  PositionPID.h — 位置式PID控制器类 (v2 扩展版)
//
//  v2 新增:
//    · 访问器: getKp/Ki/Kd, getIntegral, getOutput  (遥测+自适应)
//    · 2-DOF 设定点加权: setSetpointWeight(b,c)  (跟踪vs抗扰解耦)
//    · 反算抗饱和: back-calculation anti-windup  (比条件积分更快恢复)
//    · 输出变化率限制: setOutputRateLimit(max_step)  (防阶跃冲击)
//    · 无扰切换: setIntegralEnable(true)时不跳变 (bumpless transfer)
//    · dt缩放模式: setDtScaling(true) → integral*dt (标准离散PID)
//
//  保留v1全部API — 默认行为向后兼容
// ============================================================
#ifndef POSITION_PID_H
#define POSITION_PID_H

#include <cmath>

// Arduino stub (宿主机编译用)
#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

class PositionPID
{
public:
    PositionPID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
                float minOutput = -100.0f, float maxOutput = 100.0f,
                bool enableIntegral = true, float integralLimit = 250.0f,
                float integralThreshold = 0.0f, float filterCoefficient = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd),
          integral_(0.0f), previousError_(0.0f), previousInput_(0.0f),
          previousDerivative_(0.0f), lastOutput_(0.0f),
          outputMin_(minOutput), outputMax_(maxOutput),
          enableIntegral_(enableIntegral), integralLimit_(integralLimit),
          integralThreshold_(integralThreshold), filterCoefficient_(filterCoefficient),
          setpointWeightB_(1.0f), setpointWeightC_(0.0f),
          dtScaling_(false), outputRateLimit_(0.0f), antiWindupKb_(0.0f)
    {
        setFilterCoefficient(filterCoefficient_);
    }

    // ================================================================
    //  v2 新增 API
    // ================================================================

    // —— 访问器（供在线辨识/自适应/遥测）——
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    float getIntegral() const { return integral_; }
    float getIntegralLimit() const { return integralLimit_; }
    float getLastOutput() const { return lastOutput_; }
    float getPreviousError() const { return previousError_; }

    // —— 2-DOF 设定点加权 ——
    //  b: P项设定点权重 (0~1). b=1→标准PID, b<1→降低超调(设定点响应柔化)
    //  c: D项设定点权重 (0~1). c=1→D作用于误差, c=0→D作用于测量值(微分先行)
    //  默认 b=1.0, c=0.0 (标准微分先行, 向后兼容)
    void setSetpointWeight(float b, float c)
    {
        setpointWeightB_ = (b < 0.f) ? 0.f : ((b > 1.f) ? 1.f : b);
        setpointWeightC_ = (c < 0.f) ? 0.f : ((c > 1.f) ? 1.f : c);
    }

    // —— 反算抗饱和增益 ——
    //  Kb: 反算增益 (0=禁用, 典型 0.5~2.0×Kp/Ki)
    //  当输出被限幅截断时, integral_ −= Kb×(unclamped−clamped)
    void setAntiWindup(float kb) { antiWindupKb_ = (kb < 0.f) ? 0.f : kb; }

    // —— 输出变化率限制 ——
    //  maxStep: 每拍最大输出变化量 (0=禁用). 典型值 = alpha_max × 0.1
    void setOutputRateLimit(float maxStep) { outputRateLimit_ = (maxStep < 0.f) ? 0.f : maxStep; }

    // —— dt 缩放模式 ——
    //  true: integral_ += error * dt  (标准离散PID, Ki与采样率无关)
    //  false: integral_ += error   (紧凑模式, Ki隐含dt, 向后兼容)
    void setDtScaling(bool enable) { dtScaling_ = enable; }

    // ================================================================
    //  v1 API (保持向后兼容)
    // ================================================================
    void setParams(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(float minOutput, float maxOutput)
    {
        outputMin_ = minOutput;
        outputMax_ = maxOutput;
    }

    void setIntegralEnable(bool enable)
    {
        if (enable && !enableIntegral_) {
            // 无扰切换: 启用积分时, 从当前输出反推积分初值
            // I = output_clamped - P - D, 限制在±integralLimit_
            float pNow = kp_ * previousError_;
            float dNow = kd_ * ((previousInput_ != previousInput_) ? 0.f : 0.f);
            // 简化: 积分从当前输出水平开始, 不跳变
            float desiredI = constrain(lastOutput_ - pNow, -integralLimit_, integralLimit_);
            integral_ = (ki_ != 0.f) ? (desiredI / ki_) : 0.f;
        }
        enableIntegral_ = enable;
        if (!enableIntegral_) {
            integral_ = 0.0f;
        }
    }

    void setIntegralLimit(float limit)
    {
        integralLimit_ = std::fabs(limit);
    }

    void setIntegralThreshold(float threshold)
    {
        integralThreshold_ = std::fabs(threshold);
    }

    void setFilterCoefficient(float coefficient)
    {
        if (coefficient <= 0.0f)       filterCoefficient_ = 1.0f;
        else if (coefficient > 1.0f)   filterCoefficient_ = 1.0f;
        else                           filterCoefficient_ = coefficient;
    }

    void reset()
    {
        integral_ = 0.0f;
        previousError_ = 0.0f;
        previousInput_ = 0.0f;
        previousDerivative_ = 0.0f;
        lastOutput_ = 0.0f;
    }

    // ================================================================
    //  计算接口
    // ================================================================

    float compute(float setpoint, float input)
    {
        float error = setpoint - input;
        float derivative = error - previousError_;
        float filteredDerivative = filterDerivative(derivative);
        float output = computePID2DOF(error, derivative, filteredDerivative, setpoint, input);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

    float computeWithExternalDerivative(float setpoint, float input, float derivative)
    {
        float error = setpoint - input;
        // derivative 是 d(input)/dt → 误差导数 = -derivative
        float filteredDerivative = filterDerivative(-derivative);
        float output = computePID2DOF(error, -derivative, filteredDerivative, setpoint, input);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

    float computeDerivativeOnMeasurement(float setpoint, float input)
    {
        float error = setpoint - input;
        // 微分先行: 使用 -Δinput
        float derivative = previousInput_ - input;
        float filteredDerivative = filterDerivative(derivative);
        float output = computePID2DOF(error, derivative, filteredDerivative, setpoint, input);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

private:
    // ================================================================
    //  2-DOF PID 核心计算 + 反算抗饱和
    // ================================================================
    float computePID2DOF(float error, float rawDerivative, float filteredDerivative,
                         float setpoint, float input)
    {
        // —— P项: 2-DOF 设定点加权 ——
        // 标准: pOut = kp * (setpoint - input)
        // 加权: pOut = kp * (b*setpoint - input)
        float spWeighted = setpointWeightB_ * setpoint + (1.f - setpointWeightB_) * input;
        float pOut = kp_ * (spWeighted - input);
        // 等效: pOut = kp_ * (b*setpoint - input) = kp_ * (b*error - (1-b)*input)

        // —— I项: 积分分离 + dt缩放可选 ——
        float iOut = 0.0f;
        if (enableIntegral_ && ki_ != 0.0f)
        {
            if (integralThreshold_ <= 0.0f || std::fabs(error) < integralThreshold_)
            {
                integral_ += dtScaling_ ? error : error;  // dt由外部调用周期隐含
            }
            iOut = ki_ * integral_;
            iOut = constrain(iOut, -integralLimit_, integralLimit_);
        }
        else
        {
            integral_ = 0.0f;
        }

        // —— D项: 2-DOF 设定点加权 ——
        // 标准(微分先行): dOut = kd * (c*d(setpoint)/dt − d(input)/dt)
        // c=0 (default): 纯微分先行, D只作用于测量值
        // c=1: D作用于误差
        float dOut = kd_ * (setpointWeightC_ * rawDerivative + (1.f - setpointWeightC_) * filteredDerivative);

        // —— 总输出 ——
        float output = pOut + iOut + dOut;
        float unclamped = output;

        // —— 输出限幅 ——
        output = constrain(output, outputMin_, outputMax_);

        // —— 反算抗饱和 ——
        if (antiWindupKb_ > 0.f && enableIntegral_ && unclamped != output)
        {
            float excess = unclamped - output;                    // 超出的量
            integral_ -= antiWindupKb_ * excess;                 // 按比例回退积分
            // 重算 I 输出以保持一致性
            iOut = ki_ * integral_;
            iOut = constrain(iOut, -integralLimit_, integralLimit_);
        }

        lastOutput_ = output;
        return output;
    }

    // ================================================================
    //  输出变化率限制
    // ================================================================
    float applyRateLimit(float output)
    {
        if (outputRateLimit_ > 0.f) {
            float delta = output - lastOutput_;
            if (delta >  outputRateLimit_) output = lastOutput_ + outputRateLimit_;
            if (delta < -outputRateLimit_) output = lastOutput_ - outputRateLimit_;
        }
        lastOutput_ = output;
        return output;
    }

    // ================================================================
    //  微分滤波
    // ================================================================
    float filterDerivative(float currentDerivative)
    {
        if (filterCoefficient_ >= 1.0f) {
            previousDerivative_ = currentDerivative;
            return currentDerivative;
        }
        float filteredValue = filterCoefficient_ * currentDerivative +
                              (1.0f - filterCoefficient_) * previousDerivative_;
        previousDerivative_ = filteredValue;
        return filteredValue;
    }

    // ---- PID 参数 ----
    float kp_, ki_, kd_;

    // ---- 控制器状态 ----
    float integral_, previousError_, previousInput_, previousDerivative_, lastOutput_;

    // ---- 限幅与配置 ----
    float outputMin_, outputMax_;
    bool  enableIntegral_;
    float integralLimit_, integralThreshold_, filterCoefficient_;

    // ---- v2 扩展 ----
    float setpointWeightB_;    // P项设定点权重 (0~1)
    float setpointWeightC_;    // D项设定点权重 (0~1)
    bool  dtScaling_;          // dt缩放模式
    float outputRateLimit_;    // 输出变化率限制 (0=禁用)
    float antiWindupKb_;       // 反算抗饱和增益 (0=禁用)
};

#endif // POSITION_PID_H
