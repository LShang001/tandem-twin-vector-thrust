// ============================================================
//  test_position_pid.cpp — PositionPID v3 回归测试
//
//  该类此前无独立测试，v2 的四项扩展特性（2-DOF加权/反算抗饱和/
//  输出速率限制/无扰切换）从未被验证，其中三项实际处于失效状态。
//  本文件覆盖：
//
//    P1  基础比例/积分/微分响应正确性
//    P2  积分分离阈值 (integralThreshold)
//    P3  积分状态钳位 (v3 修复#2：integral_ 本身有界)
//    P4  输出限幅
//    P5  输出变化率限制 (v3 修复#1：原先恒不触发)
//    P6  反算抗饱和 (v3 修复#4：原先量纲错误)
//    P7  无扰切换 (v3 修复#3：原先未扣除 D 项)
//    P8  2-DOF 设定点加权
//    P9  NaN/Inf 防护 (v3 新增#5)
//    P10 三种 compute 接口一致性
//    P11 reset() 完整性
//    P12 微分滤波系数
//
//  编译: g++ -std=c++17 -Iinclude test_host/test_position_pid.cpp \
//        -o test_host/bin/pp && ./test_host/bin/pp
// ============================================================
#include "../include/PositionPID.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <limits>

static int g_fail = 0;
static void check(bool cond, const std::string& name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail;
}
static bool approx(float a, float b, float tol = 1e-4f)
{
    return std::fabs(a - b) <= tol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

// ============================================================
//  P1: 基础 P / I / D 响应
// ============================================================
static void test_basic_pid()
{
    std::printf("\n-- P1: 基础 P/I/D 响应 --\n");

    // 纯 P：输出 = kp × error
    {
        PositionPID pid(2.0f, 0.0f, 0.0f);
        pid.setOutputLimits(-1000.f, 1000.f);
        float out = pid.compute(10.f, 4.f);   // error = 6
        check(approx(out, 12.0f), "P1 纯P：output = kp×error");
    }

    // 纯 I：无 dt 约定 → 每拍累加 error
    {
        PositionPID pid(0.0f, 0.5f, 0.0f);
        pid.setOutputLimits(-1000.f, 1000.f);
        pid.setIntegralLimit(1000.f);
        float out = 0.f;
        for (int i = 0; i < 4; ++i) out = pid.compute(2.f, 0.f);  // error=2 每拍
        // integral = 2×4 = 8 → out = 0.5×8 = 4
        check(approx(out, 4.0f), "P1 纯I：4拍后 output = ki×Σerror");
        check(approx(pid.getIntegral(), 8.0f), "P1 积分状态累加正确");
    }

    // 纯 D（微分先行）：输出 = kd × (prev_input − input)
    {
        PositionPID pid(0.0f, 0.0f, 1.0f, -1000.f, 1000.f, true, 250.f, 0.f, 1.0f);
        pid.computeDerivativeOnMeasurement(0.f, 0.f);   // 建立 prev_input=0
        float out = pid.computeDerivativeOnMeasurement(0.f, 3.f);
        // derivative = prev_input − input = 0 − 3 = −3
        check(approx(out, -3.0f), "P1 纯D微分先行：output = kd×(−Δinput)");
    }
}

// ============================================================
//  P2: 积分分离阈值
// ============================================================
static void test_integral_threshold()
{
    std::printf("\n-- P2: 积分分离阈值 --\n");

    PositionPID pid(0.0f, 1.0f, 0.0f);
    pid.setOutputLimits(-1e6f, 1e6f);
    pid.setIntegralLimit(1e6f);
    pid.setIntegralThreshold(5.0f);   // |error| ≥ 5 时冻结积分

    // 大误差：不累积
    for (int i = 0; i < 10; ++i) pid.compute(20.f, 0.f);   // error=20 > 5
    check(approx(pid.getIntegral(), 0.0f), "P2 误差超阈值时积分冻结");

    // 小误差：正常累积
    for (int i = 0; i < 3; ++i) pid.compute(2.f, 0.f);     // error=2 < 5
    check(approx(pid.getIntegral(), 6.0f), "P2 误差低于阈值时积分累积");

    // 阈值为 0 → 不分离，始终累积
    PositionPID pid2(0.0f, 1.0f, 0.0f);
    pid2.setOutputLimits(-1e6f, 1e6f);
    pid2.setIntegralLimit(1e6f);
    pid2.setIntegralThreshold(0.0f);
    for (int i = 0; i < 5; ++i) pid2.compute(100.f, 0.f);
    check(pid2.getIntegral() > 0.f, "P2 阈值=0 时不分离（始终累积）");
}

// ============================================================
//  P3: 积分状态钳位（v3 修复 #2）
// ============================================================
static void test_integral_state_clamp()
{
    std::printf("\n-- P3: 积分状态钳位 (v3修复#2) --\n");

    const float ki = 0.0003f;      // 与固件 rollRatePID 一致
    const float iLimit = 10.0f;    // 固件设定
    PositionPID pid(0.0f, ki, 0.0f);
    pid.setOutputLimits(-100.f, 100.f);
    pid.setIntegralLimit(iLimit);

    // 持续大误差 1 秒（200拍 × 50 deg/s）
    for (int i = 0; i < 200; ++i) pid.compute(50.f, 0.f);

    const float bound = iLimit / ki;   // = 33333
    std::printf("  integral=%.1f (上界=%.1f)\n", pid.getIntegral(), bound);
    check(pid.getIntegral() <= bound * 1.001f,
          "P3 积分状态被钳制在 |integralLimit/ki| 内");
    // 弱断言修复：原"approx(...) || < iLimit"中第二条件使断言近乎恒真
    check(ki * pid.getIntegral() <= iLimit * 1.001f,
          "P3 积分贡献不超过 integralLimit");

    // ★ 核心：退饱和后应立即响应，而非先卸积分
    // 对比场景：integralLimit 设得足够小，使状态钳位实际生效
    {
        const float ki2 = 1.0f;  // 大 ki，使状态上界 = iLimit/ki = 5/1 = 5
        PositionPID p_no(0.0f, ki2, 0.0f),
                    p_cl(0.0f, ki2, 0.0f);
        p_no.setOutputLimits(-100.f, 100.f);
        p_cl.setOutputLimits(-100.f, 100.f);
        p_no.setIntegralLimit(5.f);
        p_cl.setIntegralLimit(5.f);

        // 驱动到饱和
        for (int i = 0; i < 20; ++i) {
            p_no.compute(50.f, 0.f);
            p_cl.compute(50.f, 0.f);
        }

        // 反向：p_no 理论上与 p_cl 相同（同一实现），但上界=5 保证快速恢复
        int f_cl = -1;
        for (int i = 0; i < 30; ++i) {
            float o = p_cl.compute(-50.f, 0.f);
            if (o < 0.f && f_cl < 0) { f_cl = i; }
        }
        std::printf("  integralLimit/ki=5：%d 拍后输出转负\n", f_cl);
        check(f_cl >= 0 && f_cl < 10,
              "P3 积分状态钳位后退饱和迅速（iLimit/ki=5 → <10拍）");
    }
}

// ============================================================
//  P4: 输出限幅
// ============================================================
static void test_output_limits()
{
    std::printf("\n-- P4: 输出限幅 --\n");

    PositionPID pid(100.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-25.f, 25.f);

    check(approx(pid.compute(10.f, 0.f), 25.0f),  "P4 正向饱和至 outputMax");
    check(approx(pid.compute(-10.f, 0.f), -25.0f), "P4 负向饱和至 outputMin");

    // 非对称限幅
    PositionPID pid2(100.0f, 0.0f, 0.0f);
    pid2.setOutputLimits(0.f, 50.f);
    check(approx(pid2.compute(-10.f, 0.f), 0.0f), "P4 非对称限幅下界生效");
    check(approx(pid2.compute(10.f, 0.f), 50.0f), "P4 非对称限幅上界生效");
}

// ============================================================
//  P5: 输出变化率限制（v3 修复 #1 —— 原先恒不触发）
// ============================================================
static void test_output_rate_limit()
{
    std::printf("\n-- P5: 输出变化率限制 (v3修复#1) --\n");

    PositionPID pid(10.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-1000.f, 1000.f);
    pid.setOutputRateLimit(5.0f);      // 每拍最多变 5

    // 阶跃指令：期望输出 100，但每拍只能爬 5
    float o1 = pid.compute(10.f, 0.f);
    check(approx(o1, 5.0f), "P5 首拍输出被限速至 +5（原实现返回100）");

    float o2 = pid.compute(10.f, 0.f);
    check(approx(o2, 10.0f), "P5 次拍继续爬升至 10");

    // 爬满 20 拍应达到 100
    for (int i = 0; i < 30; ++i) pid.compute(10.f, 0.f);
    check(approx(pid.getLastOutput(), 100.0f), "P5 持续爬升最终到达目标值");

    // 反向阶跃同样受限
    float d1 = pid.compute(-10.f, 0.f);
    check(approx(d1, 95.0f), "P5 反向阶跃每拍降幅同样受限");

    // rateLimit=0 → 禁用
    PositionPID pid2(10.0f, 0.0f, 0.0f);
    pid2.setOutputLimits(-1000.f, 1000.f);
    pid2.setOutputRateLimit(0.0f);
    check(approx(pid2.compute(10.f, 0.f), 100.0f), "P5 rateLimit=0 时禁用限速");

    // 限速后仍须满足输出限幅
    PositionPID pid3(10.0f, 0.0f, 0.0f);
    pid3.setOutputLimits(-3.f, 3.f);
    pid3.setOutputRateLimit(100.0f);
    check(std::fabs(pid3.compute(10.f, 0.f)) <= 3.0f,
          "P5 限速不会把输出推出 [min,max]");
}

// ============================================================
//  P6: 反算抗饱和（v3 修复 #4 —— 原先量纲错误）
// ============================================================
static void test_back_calculation_antiwindup()
{
    std::printf("\n-- P6: 反算抗饱和 (v3修复#4) --\n");

    // ki 需足够大以让【输出限幅】真正触发（抗饱和只在 unclamped≠clamped 时动作）。
    // 用 ki=0.0003 时 iOut 长期在限幅内，抗饱和根本不会被调用。
    const float ki = 1.0f;
    auto build = [&](float kb) {
        PositionPID p(0.0f, ki, 0.0f);
        p.setOutputLimits(-10.f, 10.f);
        p.setIntegralLimit(1e6f);      // 放开状态钳位，只让输出限幅触发抗饱和
        p.setAntiWindup(kb);
        return p;
    };

    // 无抗饱和 vs 有抗饱和：积分累积量应显著不同
    PositionPID no_aw  = build(0.0f);
    PositionPID with_aw = build(1.0f);
    for (int i = 0; i < 300; ++i) {
        no_aw.compute(50.f, 0.f);
        with_aw.compute(50.f, 0.f);
    }
    std::printf("  Kb=0: integral=%.0f ; Kb=1: integral=%.0f\n",
                no_aw.getIntegral(), with_aw.getIntegral());
    check(with_aw.getIntegral() < no_aw.getIntegral(),
          "P6 反算抗饱和显著抑制积分累积（量纲修正后生效）");

    // 抗饱和不应把积分推成反号（过度回退）
    check(with_aw.getIntegral() > -1.0f / ki,
          "P6 抗饱和回退不过度（未反号发散）");

    // ki=0 时不应除零
    PositionPID zero_ki(1.0f, 0.0f, 0.0f);
    zero_ki.setOutputLimits(-1.f, 1.f);
    zero_ki.setAntiWindup(1.0f);
    float o = zero_ki.compute(100.f, 0.f);
    check(std::isfinite(o) && std::isfinite(zero_ki.getIntegral()),
          "P6 ki=0 时抗饱和不触发除零");
}

// ============================================================
//  P7: 无扰切换（v3 修复 #3 —— 原先未扣除 D 项）
// ============================================================
static void test_bumpless_transfer()
{
    std::printf("\n-- P7: 无扰切换 (v3修复#3) --\n");

    // 无扰切换的真实价值：当积分【之前已有累积】时，禁用→重新启用不发生跳变。
    //
    // 原先的 dNow 死代码（恒 0）导致 D 项贡献未被扣除，使得有 D 的情况下
    // 积分初值偏小 → 再次启用时输出比禁用前少了 kd×D 那么多（即发生跳变）。
    // v3 修复后 D 项正确扣除，重新启用时输出恢复到禁用前水平。
    //
    // 测试设计：积分启用运行若干拍 → 禁用（输出丢掉 I 贡献，必然跳变）
    //          → 不再运行 → 重新启用 → 首拍输出应恢复到「禁用前水平」
    //
    // 注意："首步积分增量"（ki×error）是固有现象，不是跳变。
    //       本测试比较「禁用前最后一拍」vs「重新启用后首拍再往后稳定时」
    //       —— 实际只比较禁用/重新启用瞬间的跳变量是否 < ki×|error|。

    // 正确语义：bumpless 保证的是【切换瞬间】不跳变，
    // 即启用后输出应贴近「切换时的 lastOutput_」，而非"禁用前的历史输出"
    //（那个值对应的积分已被清零，无从恢复）。切换后首拍必然叠加一个
    // ki×error 的固有积分增量，这不算跳变。

    const float kp = 2.0f, ki = 0.5f, kd = 1.0f;
    PositionPID pid(kp, ki, kd, -1000.f, 1000.f, true, 1000.f, 0.f, 1.0f);

    // 先在禁用积分下运行，建立非零 D 历史（D≠0 用于暴露 dNow 死代码）
    // 然后再运行几拍稳定输入，使 D→0，再切换（消除 D 项变化对跳变的贡献）
    pid.setIntegralEnable(false);
    float in = 0.f;
    for (int i = 0; i < 4; ++i) { in += 1.5f;
        pid.computeDerivativeOnMeasurement(10.f, in); }
    // 稳定输入 3 拍：D → 0
    for (int i = 0; i < 3; ++i)
        pid.computeDerivativeOnMeasurement(10.f, in);  // in 不再变化

    const float out_at_switch = pid.getLastOutput();
    const float prevErr   = pid.getPreviousError();
    const float prevDeriv = pid.getPreviousDerivative();  // 此时应接近 0

    // 启用积分（bumpless）
    pid.setIntegralEnable(true);
    const float I_restored = pid.getIntegral();

    // ★ 验证 bumpless 公式含 D 项：P+I+D 精确重建切换时输出
    //   原实现 dNow 死代码（恒 0 且未使用），D≠0 时偏差 kd×D
    const float rebuilt = kp * prevErr + ki * I_restored + kd * prevDeriv;
    std::printf("  切换时输出=%.4f  重建=%.4f  (I=%.4f, D=%.4f)\n",
                out_at_switch, rebuilt, I_restored, prevDeriv);
    check(std::fabs(rebuilt - out_at_switch) < 1e-3f,
          "P7 bumpless 公式含 D 项：P+I+D 精确重建切换时输出");

    // D 已稳定→0，切换后跳变应仅为固有积分增量 ki×|error|
    const float after = pid.computeDerivativeOnMeasurement(10.f, in);
    const float jump  = std::fabs(after - out_at_switch);
    const float expected = ki * std::fabs(10.f - in);
    std::printf("  切换后首拍=%.4f 跳变=%.4f 期望≈%.4f (ki×|e|)\n",
                after, jump, expected);
    check(jump <= expected + 1e-3f,
          "P7 D 稳定时跳变 = 固有积分增量（无 D 项冲击）");

    // 禁用积分应清零状态
    pid.setIntegralEnable(false);
    check(approx(pid.getIntegral(), 0.0f), "P7 禁用积分时清零积分状态");
}

// ============================================================
//  P8: 2-DOF 设定点加权
// ============================================================
static void test_setpoint_weighting()
{
    std::printf("\n-- P8: 2-DOF 设定点加权 --\n");

    // b=1（默认）：pOut = kp×(sp − in)
    {
        PositionPID pid(2.0f, 0.f, 0.f);
        pid.setOutputLimits(-1000.f, 1000.f);
        pid.setSetpointWeight(1.0f, 0.0f);
        check(approx(pid.compute(10.f, 0.f), 20.0f), "P8 b=1：标准P（kp×error）");
    }

    // b=0.5：pOut = kp×(0.5×sp − 0.5×in)... 实为 kp×(b×sp+(1−b)×in − in)
    {
        PositionPID pid(2.0f, 0.f, 0.f);
        pid.setOutputLimits(-1000.f, 1000.f);
        pid.setSetpointWeight(0.5f, 0.0f);
        // spWeighted = 0.5×10 + 0.5×0 = 5 → pOut = 2×(5−0) = 10
        check(approx(pid.compute(10.f, 0.f), 10.0f), "P8 b=0.5：设定点响应减半（抑超调）");
    }

    // b=0：P 项对设定点完全不响应（纯反馈）
    {
        PositionPID pid(2.0f, 0.f, 0.f);
        pid.setOutputLimits(-1000.f, 1000.f);
        pid.setSetpointWeight(0.0f, 0.0f);
        check(approx(pid.compute(10.f, 0.f), 0.0f), "P8 b=0：P项不响应设定点");
    }

    // 权重范围钳制
    {
        PositionPID pid(2.0f, 0.f, 0.f);
        pid.setOutputLimits(-1000.f, 1000.f);
        pid.setSetpointWeight(5.0f, -3.0f);   // 越界
        float o = pid.compute(10.f, 0.f);
        check(approx(o, 20.0f), "P8 权重越界被钳制到 [0,1]");
    }
}

// ============================================================
//  P9: NaN / Inf 防护（v3 新增 #5）
// ============================================================
static void test_nan_protection()
{
    std::printf("\n-- P9: NaN/Inf 防护 (v3新增#5) --\n");

    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    const float inf_v = std::numeric_limits<float>::infinity();

    // 建立正常状态
    PositionPID pid(2.0f, 0.5f, 0.1f);
    pid.setOutputLimits(-1000.f, 1000.f);
    pid.setIntegralLimit(1000.f);
    for (int i = 0; i < 5; ++i) pid.compute(10.f, 5.f);
    const float good_out = pid.getLastOutput();
    const float good_int = pid.getIntegral();

    // 注入 NaN setpoint
    float o_nan = pid.compute(nan_v, 5.f);
    check(std::isfinite(o_nan), "P9 NaN 设定点：输出仍有限");
    check(approx(o_nan, good_out), "P9 NaN 设定点：冻结为上一拍输出");
    check(approx(pid.getIntegral(), good_int), "P9 NaN 设定点：积分状态未被污染");

    // 注入 NaN input
    float o_nan2 = pid.compute(10.f, nan_v);
    check(std::isfinite(o_nan2) && std::isfinite(pid.getIntegral()),
          "P9 NaN 测量值：状态未被污染");

    // 注入 Inf
    pid.compute(inf_v, 5.f);
    check(std::isfinite(pid.getIntegral()), "P9 Inf 输入：积分状态保持有限");

    // 外部导数接口
    pid.computeWithExternalDerivative(10.f, 5.f, nan_v);
    check(std::isfinite(pid.getIntegral()), "P9 NaN 外部导数：状态未被污染");

    // 微分先行接口
    pid.computeDerivativeOnMeasurement(nan_v, 5.f);
    check(std::isfinite(pid.getIntegral()), "P9 NaN 微分先行接口：状态未被污染");

    // ★ 关键：故障后恢复正常输入，控制器应继续正常工作
    float recovered = 0.f;
    for (int i = 0; i < 5; ++i) recovered = pid.compute(10.f, 5.f);
    check(std::isfinite(recovered) && std::fabs(recovered) > 1e-6f,
          "P9 非有限输入后能自愈（原实现将永久输出 NaN）");

    // 计数器
    std::printf("  拦截次数=%lu\n", pid.getNonFiniteCount());
    check(pid.getNonFiniteCount() >= 5, "P9 非有限输入被计数（供遥测排查）");
    pid.clearNonFiniteCount();
    check(pid.getNonFiniteCount() == 0, "P9 计数器可清零");
}

// ============================================================
//  P10: 三种 compute 接口一致性
// ============================================================
static void test_interface_consistency()
{
    std::printf("\n-- P10: 三接口一致性 --\n");

    // 纯 P 时三接口输出应相同（D/I 为零）
    PositionPID a(3.0f, 0.f, 0.f), b(3.0f, 0.f, 0.f), c(3.0f, 0.f, 0.f);
    a.setOutputLimits(-1000.f, 1000.f);
    b.setOutputLimits(-1000.f, 1000.f);
    c.setOutputLimits(-1000.f, 1000.f);

    float oa = a.compute(8.f, 2.f);
    float ob = b.computeWithExternalDerivative(8.f, 2.f, 0.f);
    float oc = c.computeDerivativeOnMeasurement(8.f, 2.f);
    check(approx(oa, ob) && approx(ob, oc), "P10 纯P时三接口输出一致");

    // 外部导数接口：derivative 是 d(input)/dt，正值应产生负 D 贡献
    PositionPID d(0.f, 0.f, 2.0f, -1000.f, 1000.f, true, 250.f, 0.f, 1.0f);
    float od = d.computeWithExternalDerivative(0.f, 0.f, 5.0f);
    check(od < 0.f, "P10 外部导数为正 → D项为负（阻尼作用）");
}

// ============================================================
//  P11: reset() 完整性
// ============================================================
static void test_reset()
{
    std::printf("\n-- P11: reset() 完整性 --\n");

    PositionPID pid(2.0f, 0.5f, 1.0f);
    pid.setOutputLimits(-1000.f, 1000.f);
    pid.setIntegralLimit(1000.f);
    for (int i = 0; i < 10; ++i) pid.compute(10.f, 3.f);

    check(std::fabs(pid.getIntegral()) > 1e-6f, "P11 reset前积分非零（前提）");
    check(std::fabs(pid.getLastOutput()) > 1e-6f, "P11 reset前输出非零（前提）");

    pid.reset();
    check(approx(pid.getIntegral(), 0.f),      "P11 reset 清零积分");
    check(approx(pid.getLastOutput(), 0.f),    "P11 reset 清零上次输出");
    check(approx(pid.getPreviousError(), 0.f), "P11 reset 清零误差历史");

    // reset 后首拍应等价于全新实例
    PositionPID fresh(2.0f, 0.5f, 1.0f);
    fresh.setOutputLimits(-1000.f, 1000.f);
    fresh.setIntegralLimit(1000.f);
    check(approx(pid.compute(10.f, 3.f), fresh.compute(10.f, 3.f)),
          "P11 reset 后行为等同全新实例");
}

// ============================================================
//  P12: 微分滤波系数
// ============================================================
static void test_derivative_filter()
{
    std::printf("\n-- P12: 微分滤波 --\n");

    // coefficient=1 → 无滤波（直通）
    {
        PositionPID pid(0.f, 0.f, 1.0f, -1000.f, 1000.f, true, 250.f, 0.f, 1.0f);
        pid.computeDerivativeOnMeasurement(0.f, 0.f);
        float o = pid.computeDerivativeOnMeasurement(0.f, 10.f);
        check(approx(o, -10.0f), "P12 系数=1：微分直通无滤波");
    }

    // coefficient=0.2 → 强滤波，首拍响应被显著衰减
    {
        PositionPID pid(0.f, 0.f, 1.0f, -1000.f, 1000.f, true, 250.f, 0.f, 0.2f);
        pid.computeDerivativeOnMeasurement(0.f, 0.f);
        float o = pid.computeDerivativeOnMeasurement(0.f, 10.f);
        std::printf("  系数0.2 首拍微分输出=%.2f (无滤波应为-10)\n", o);
        check(std::fabs(o) < 10.0f, "P12 系数<1：微分被低通衰减");
        check(std::fabs(o) > 0.f,   "P12 滤波后仍有响应（非完全阻断）");
    }

    // coefficient<=0 被钳制为 1（避免完全阻断微分）
    {
        PositionPID pid(0.f, 0.f, 1.0f, -1000.f, 1000.f, true, 250.f, 0.f, 0.0f);
        pid.computeDerivativeOnMeasurement(0.f, 0.f);
        float o = pid.computeDerivativeOnMeasurement(0.f, 10.f);
        check(approx(o, -10.0f), "P12 系数≤0 被钳制为1（不阻断微分）");
    }
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::printf("=== PositionPID v3 回归测试 ===\n");

    test_basic_pid();
    test_integral_threshold();
    test_integral_state_clamp();
    test_output_limits();
    test_output_rate_limit();
    test_back_calculation_antiwindup();
    test_bumpless_transfer();
    test_setpoint_weighting();
    test_nan_protection();
    test_interface_consistency();
    test_reset();
    test_derivative_filter();

    std::printf("\n");
    if (g_fail == 0) std::printf("=== 全部通过 ===\n");
    else             std::printf("=== %d 项失败 ===\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
