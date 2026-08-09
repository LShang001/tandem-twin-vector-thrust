// ============================================================
//  PositionPID.h — 位置式PID控制器类 (v3)
//
//  v2 特性:
//    · 访问器: getKp/Ki/Kd, getIntegral, getLastOutput  (遥测+自适应)
//    · 2-DOF 设定点加权: setSetpointWeight(b,c)  (跟踪vs抗扰解耦)
//    · 反算抗饱和: back-calculation anti-windup  (比条件积分更快恢复)
//    · 输出变化率限制: setOutputRateLimit(max_step)  (防阶跃冲击)
//    · 无扰切换: setIntegralEnable(true)时不跳变 (bumpless transfer)
//
//  v3 修复（均为原实现中的功能性缺陷，见各处 ★ 注释）:
//    1. 输出变化率限制失效 —— computePID2DOF 提前写 lastOutput_，
//       导致 applyRateLimit 中 delta 恒为 0，限速永不触发。
//    2. 积分状态无界 —— integralLimit_ 只钳制 iOut 而非 integral_ 本身，
//       200Hz/Ki=3e-4 下 1s 即累积到 1e4，退饱和迟滞可达数秒。
//    3. 无扰切换未扣除 D 项 —— dNow 是恒 0 的死表达式且从未被使用。
//    4. 反算抗饱和量纲错误 —— excess(输出量) 与 integral_(误差·拍) 差一个 ki_，
//       Ki=3e-4 时回退量被低估约 3000 倍，功能几乎无效。
//    5. 新增 NaN/Inf 防护 —— 单个非有限输入曾会永久污染积分器与微分滤波器，
//       此后输出恒为 NaN 且无法自愈；现拦截并冻结该拍，计数供遥测。
//
//  积分约定: integral_ += error*dt（dt 显式传入，秒）——Ki 为连续域增益，
//  与调用频率解耦（2026-08-09 重构；旧约定 Ki 隐含 200Hz，参数随频率漂移）。
//            原 setDtScaling(bool) 因接口拿不到 dt 恒为死代码，已删除。
//
//  行为兼容性: v1/v2 默认配置下（Kb=0、rateLimit=0、b=1、c=0）仅
//    "积分状态钳位"会改变现役行为 —— 稳态输出不变，饱和恢复更快。
//    回归测试见 test_host/test_position_pid.cpp
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
          outputRateLimit_(0.0f), antiWindupKb_(0.0f),
          nonFiniteCount_(0)
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
    // 上一拍进入 D 项的（滤波后）微分值。c=0（默认微分先行）时
    // D 项贡献恰为 kd_ × 该值，无扰切换据此扣除 D 分量。
    float getPreviousDerivative() const { return previousDerivative_; }

    // —— 健康诊断：累计拦截的 NaN/Inf 输入次数 ——
    //  正常飞行应恒为 0。非零表示上游（AHRS/陀螺/设定点）出现了非有限值，
    //  控制器已冻结该拍输出保护自身，但上游故障需排查。
    //  刻意不在 reset() 中清零：跨模式切换后仍需保留故障史。
    unsigned long getNonFiniteCount() const { return nonFiniteCount_; }
    void clearNonFiniteCount() { nonFiniteCount_ = 0; }

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

    // —— 积分约定说明（无 dt 缩放开关）——
    //  compute 系列接口接收 dt（秒），积分按 integral_ += error*dt 累加——
    //  Ki 为连续域增益（与调用频率解耦）。微分滤波 alpha 仍为每拍系数
    //  （kd 全轴为 0，当前无实际影响）。
    //  曾有 setDtScaling(bool) 声称可切到 integral_ += error*dt，但接口拿不到
    //  dt，实现恒等于非缩放分支（死代码），已删除以免误用。
    //  若确需标准离散积分，应新增接受 dt 的 compute 重载，而非布尔开关。

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
            // 无扰切换 (bumpless transfer): 启用积分时从当前输出反推积分初值，
            // 使切换瞬间总输出不跳变。
            //   output = P + I + D  →  I_desired = lastOutput_ − P − D
            //
            // 原实现的 dNow 是死代码:
            //   kd_ * ((previousInput_ != previousInput_) ? 0.f : 0.f)
            // 该表达式恒为 0（两个分支相同），且计算出来后从未被使用，
            // 导致 D 项贡献未被扣除，切换时仍会跳变 kd·D 的量。
            const float pNow = kp_ * previousError_;
            const float dNow = kd_ * previousDerivative_;   // 上一拍实际 D 项输入
            const float desiredI = constrain(lastOutput_ - pNow - dNow,
                                             -integralLimit_, integralLimit_);
            integral_ = (ki_ != 0.f) ? (desiredI / ki_) : 0.f;
            clampIntegralState();
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

    // ★ 三个接口均在入口做有限性检查：
    //   filterDerivative() 会写 previousDerivative_，previousError_/previousInput_
    //   也在末尾无条件赋值 —— 若不在入口拦截，一个 NaN 输入会同时污染
    //   微分滤波器与误差历史，即使 computePID2DOF 内部有防护也无法挽回。

    float compute(float setpoint, float input, float dt)
    {
        if (!isFiniteF(setpoint) || !isFiniteF(input)) { ++nonFiniteCount_; return lastOutput_; }
        float error = setpoint - input;
        float derivative = error - previousError_;
        float filteredDerivative = filterDerivative(derivative);
        float output = computePID2DOF(error, derivative, filteredDerivative, setpoint, input, dt);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

    float computeWithExternalDerivative(float setpoint, float input, float derivative, float dt)
    {
        if (!isFiniteF(setpoint) || !isFiniteF(input) || !isFiniteF(derivative))
        { ++nonFiniteCount_; return lastOutput_; }
        float error = setpoint - input;
        // derivative 是 d(input)/dt → 误差导数 = -derivative
        float filteredDerivative = filterDerivative(-derivative);
        float output = computePID2DOF(error, -derivative, filteredDerivative, setpoint, input, dt);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

    float computeDerivativeOnMeasurement(float setpoint, float input, float dt)
    {
        if (!isFiniteF(setpoint) || !isFiniteF(input)) { ++nonFiniteCount_; return lastOutput_; }
        float error = setpoint - input;
        // 微分先行: 使用 -Δinput
        float derivative = previousInput_ - input;
        float filteredDerivative = filterDerivative(derivative);
        float output = computePID2DOF(error, derivative, filteredDerivative, setpoint, input, dt);
        previousError_ = error;
        previousInput_ = input;
        return applyRateLimit(output);
    }

private:
    // ================================================================
    //  有限性检查（不依赖 std::isfinite 的 double 提升，嵌入式友好）
    // ================================================================
    static bool isFiniteF(float v)
    {
        // NaN 自比较为假；±Inf 的绝对值大于任意有限阈值
        return (v == v) && (v <  3.0e38f) && (v > -3.0e38f);
    }

    // ================================================================
    //  2-DOF PID 核心计算 + 反算抗饱和
    // ================================================================
    float computePID2DOF(float error, float rawDerivative, float filteredDerivative,
                         float setpoint, float input, float dt)
    {
        // ★ NaN/Inf 防护（安全关键）
        // 传感器毛刺或除零一旦产生 NaN，integral_ += NaN 会永久污染积分器：
        // 此后所有输出恒为 NaN，控制器再也无法恢复，且 reset() 之前无声失效。
        // 策略: 输入非有限时冻结本拍（保持上一拍输出），不更新任何内部状态。
        if (!isFiniteF(error) || !isFiniteF(rawDerivative) ||
            !isFiniteF(filteredDerivative) || !isFiniteF(input) || !isFiniteF(setpoint))
        {
            ++nonFiniteCount_;
            return lastOutput_;   // 保持上一拍输出，状态不变
        }
        // —— P项: 2-DOF 设定点加权 ——
        // 标准: pOut = kp * (setpoint - input)
        // 加权: pOut = kp * (b*setpoint - input)
        float spWeighted = setpointWeightB_ * setpoint + (1.f - setpointWeightB_) * input;
        float pOut = kp_ * (spWeighted - input);
        // 等效: pOut = kp_ * (b*setpoint - input) = kp_ * (b*error - (1-b)*input)

        // —— I项: 积分分离 + 积分状态钳位 ——
        float iOut = 0.0f;
        if (enableIntegral_ && ki_ != 0.0f)
        {
            if (integralThreshold_ <= 0.0f || std::fabs(error) < integralThreshold_)
            {
                integral_ += error * dt;  // ★2026-08-09 dt 显式传入：Ki 为连续域增益（参数不再隐含 200Hz）
            }
            // ★ 钳制【积分状态本身】，而非仅钳制其输出贡献。
            //   原实现只做 iOut=constrain(ki*integral)，integral_ 可无限增长：
            //   200Hz 下 50deg/s 误差累积 1s 即达 10000，误差反向时必须先卸掉
            //   这个巨量才能响应（典型 integral windup，恢复迟滞可达数秒）。
            //   钳位后稳态 iOut 与原实现一致，但退饱和立即响应。
            clampIntegralState();
            iOut = ki_ * integral_;
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

        // —— 反算抗饱和 (back-calculation) ——
        // 输出被限幅时，按超出量回退积分，使下一拍能立刻退出饱和。
        //
        // 量纲修正: excess 的单位是【输出量】，integral_ 的单位是【误差·拍】，
        //   两者相差一个 ki_。原实现 integral_ -= Kb*excess 直接混用单位，
        //   在 ki_=0.0003 时回退量被低估约 3000 倍，抗饱和几乎无效。
        //   正确形式: integral_ -= Kb * excess / ki_
        if (antiWindupKb_ > 0.f && enableIntegral_ && ki_ != 0.0f && unclamped != output)
        {
            const float excess = unclamped - output;
            integral_ -= antiWindupKb_ * excess / ki_;
            clampIntegralState();
            // 注: 本拍 output 不再重算 —— 限幅值即为实际执行量，
            //     回退只影响下一拍（原实现重算 iOut 但未回写 output，是死代码）。
        }

        // ★ 此处不再写 lastOutput_。
        //   原实现在这里赋值，随后 applyRateLimit 计算
        //   delta = output − lastOutput_ 恒为 0，速率限制永不触发（死功能）。
        //   现统一由 applyRateLimit 在限速后写入，作为"上一拍实际输出"。
        return output;
    }

    // ================================================================
    //  输出变化率限制
    //  必须在 computePID2DOF 之后调用，且是唯一写 lastOutput_ 的地方。
    // ================================================================
    float applyRateLimit(float output)
    {
        if (outputRateLimit_ > 0.f) {
            const float delta = output - lastOutput_;
            if (delta >  outputRateLimit_) output = lastOutput_ + outputRateLimit_;
            if (delta < -outputRateLimit_) output = lastOutput_ - outputRateLimit_;
            // 限速后仍须满足输出限幅（避免 rate limit 把值推出 [min,max]）
            output = constrain(output, outputMin_, outputMax_);
        }
        lastOutput_ = output;
        return output;
    }

    // ================================================================
    //  积分状态钳位
    //  由 integralLimit_ 反推积分状态上界：|ki·integral| ≤ integralLimit_
    //  ki=0 时积分无意义，直接归零。
    // ================================================================
    void clampIntegralState()
    {
        if (ki_ == 0.0f) { integral_ = 0.0f; return; }
        const float bound = std::fabs(integralLimit_ / ki_);
        integral_ = constrain(integral_, -bound, bound);
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
    float outputRateLimit_;    // 输出变化率限制 (0=禁用)
    float antiWindupKb_;       // 反算抗饱和增益 (0=禁用)

    // ---- 健康诊断 ----
    unsigned long nonFiniteCount_;  // 累计拦截的 NaN/Inf 输入次数（遥测用）
};

#endif // POSITION_PID_H
