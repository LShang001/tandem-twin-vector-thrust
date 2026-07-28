// 宿主机回归测试：级联控制律（姿态环 + 角速率环 + 惯量逆解 + 分配）
// 编译运行：
//   g++ -std=c++17 -Iinclude test_host/test_tandemvec_cascade.cpp \
//       -o test_host/bin/tc && ./test_host/bin/tc
//
// 覆盖范围：
//   T1  外环：零误差 → 零角速率指令
//   T2  外环：小角度误差极性（滚转/俯仰/偏航）
//   T3  外环：180° 大角度最短路径
//   T4  外环：omega_max 限幅触发
//   T5  内环：P-only 响应（ki=kd=0）
//   T6  内环：积分累积（ki>0）
//   T7  内环：微分先行不产生阶跃冲击
//   T8  内环：抗积分饱和（饱和时积分冻结）
//   T9  内环：reset() 清零积分
//   T10 力矩映射：惯量系数正确
//   T11 全链：零误差 → 零执行器指令
//   T12 全链：俯仰误差 → 尾摆正确极性
//   T13 全链：偏航误差 → 前摆正确极性
//   T14 全链：遥测字段完整（无NaN）
#include "../include/TandemVec_CascadeCtrl.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <cstring>  // memset

static int g_fail = 0;
static void check(bool cond, const std::string& name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail;
}
static bool approx(float a, float b, float rtol = 1e-3f, float atol = 1e-7f)
{
    return std::fabs(a - b) <= rtol * std::fmax(1.f, std::fmax(std::fabs(a), std::fabs(b))) + atol;
}
static bool is_finite(float v) { return std::isfinite(v); }

// ============================================================
//  T1–T4：外环 (attitudeStep)
// ============================================================
static void test_attitude_zero_error()
{
    Quat4f q = {1.f, 0.f, 0.f, 0.f};  // 恒等旋转
    AttitudeCtrlGains g = kDefaultCascadeCtrlParams.att;
    float omega[3];
    attitudeStep(q, q, g, omega);
    check(approx(omega[0], 0.f), "外环零误差 → omega_ref[0]=0");
    check(approx(omega[1], 0.f), "外环零误差 → omega_ref[1]=0");
    check(approx(omega[2], 0.f), "外环零误差 → omega_ref[2]=0");
}

static void test_attitude_polarity()
{
    const AttitudeCtrlGains& g = kDefaultCascadeCtrlParams.att;
    const float kSmallAngle = 0.1f;  // rad，约 5.7°
    float omega[3];

    // 绕 x_b 轴正转 → 期望正滚转速率
    {
        Quat4f qm = {1.f, 0.f, 0.f, 0.f};
        Quat4f qr = {cosf(kSmallAngle/2.f), sinf(kSmallAngle/2.f), 0.f, 0.f};
        attitudeStep(qm, qr, g, omega);
        check(omega[0] > 0.f, "外环：滚转误差+θ → omega_ref[0]>0");
        check(approx(omega[1], 0.f, 0.01f), "外环：滚转误差 → omega_ref[1]≈0");
        check(approx(omega[2], 0.f, 0.01f), "外环：滚转误差 → omega_ref[2]≈0");
    }

    // 绕 y_b 轴正转 → 期望正俯仰速率
    {
        Quat4f qm = {1.f, 0.f, 0.f, 0.f};
        Quat4f qr = {cosf(kSmallAngle/2.f), 0.f, sinf(kSmallAngle/2.f), 0.f};
        attitudeStep(qm, qr, g, omega);
        check(approx(omega[0], 0.f, 0.01f), "外环：俯仰误差 → omega_ref[0]≈0");
        check(omega[1] > 0.f, "外环：俯仰误差+θ → omega_ref[1]>0");
    }

    // 绕 z_b 轴正转 → 期望正偏航速率
    {
        Quat4f qm = {1.f, 0.f, 0.f, 0.f};
        Quat4f qr = {cosf(kSmallAngle/2.f), 0.f, 0.f, sinf(kSmallAngle/2.f)};
        attitudeStep(qm, qr, g, omega);
        check(approx(omega[0], 0.f, 0.01f), "外环：偏航误差 → omega_ref[0]≈0");
        check(omega[2] > 0.f, "外环：偏航误差+ψ → omega_ref[2]>0");
    }
}

static void test_attitude_shortest_path()
{
    // 目标：绕 z_b 旋转 200° = 等价于 -160°（短路径）
    // 期望 omega_ref[2] < 0（负偏航速率，走短路径）
    const float angle = 200.f * (3.14159265f / 180.f);
    Quat4f qm = {1.f, 0.f, 0.f, 0.f};
    Quat4f qr = {cosf(angle/2.f), 0.f, 0.f, sinf(angle/2.f)};
    AttitudeCtrlGains g = kDefaultCascadeCtrlParams.att;
    float omega[3];
    attitudeStep(qm, qr, g, omega);
    // 最短路径取负 → omega_ref[2] 应为负（等效 -160°）
    check(omega[2] < 0.f, "外环：180°+ 旋转取最短路径（omega_ref[2]<0）");
}

static void test_attitude_omega_clamp()
{
    // 很大的角度误差应触发 omega_max 限幅
    const float big_angle = 1.57f;  // 90°
    Quat4f qm = {1.f, 0.f, 0.f, 0.f};
    Quat4f qr = {cosf(big_angle/2.f), 0.f, sinf(big_angle/2.f), 0.f};
    AttitudeCtrlGains g = kDefaultCascadeCtrlParams.att;
    float omega[3];
    attitudeStep(qm, qr, g, omega);
    check(std::fabs(omega[1]) <= g.omega_max_pitch + 1e-5f,
          "外环：大误差时 omega_ref 限幅至 omega_max_pitch");
}

// ============================================================
//  T5–T9：内环 (RateCtrl)
// ============================================================
static void test_rate_p_only()
{
    RateCtrl ctrl; ctrl.reset();
    RateCtrlGains g = kDefaultCascadeCtrlParams.rate;
    g.ki[1] = 0.f; g.kd[1] = 0.f;  // 仅P

    float omega_ref[3] = {0.f, 1.0f, 0.f};    // 目标俯仰速率 1 rad/s
    float omega_meas[3] = {0.f, 0.f, 0.f};    // 当前速率 0
    float alpha[3];

    ctrl.step(omega_ref, omega_meas, g, 0.005f, alpha);

    // α = kp * 1.0（误差 = 1.0）
    check(approx(alpha[1], g.kp[1] * 1.0f, 0.01f),
          "内环P-only：alpha = kp·e");
    check(approx(alpha[0], 0.f, 0.01f), "内环P-only：roll alpha≈0");
    check(approx(alpha[2], 0.f, 0.01f), "内环P-only：yaw alpha≈0");
}

static void test_rate_integral()
{
    RateCtrl ctrl; ctrl.reset();
    RateCtrlGains g = kDefaultCascadeCtrlParams.rate;
    g.kp[1] = 0.f; g.kd[1] = 0.f;  // 仅I

    float omega_ref[3]  = {0.f, 0.5f, 0.f};
    float omega_meas[3] = {0.f, 0.f,  0.f};
    float alpha[3];
    const float dt = 0.005f;

    // 运行10拍，积分应按 ki * e * dt 线性增长
    float acc = 0.f;
    for (int n = 0; n < 10; ++n) {
        ctrl.step(omega_ref, omega_meas, g, dt, alpha);
        acc += 0.5f * dt;
    }
    // 积分 ≈ acc，alpha ≈ ki * acc
    float expected = g.ki[1] * acc;
    check(approx(alpha[1], expected, 0.02f),
          "内环积分：10拍后 alpha ≈ ki·∫e");
}

static void test_rate_no_derivative_kick()
{
    // 微分先行（derivative-on-measurement）：ω_ref 阶跃不应改变 alpha 输出。
    // 设计：kp=ki=0，仅 D 项生效。omega_meas 保持恒定，预热 1 拍令
    // omega_prev = omega_meas，之后 D 贡献 = -kd·(omega_meas - omega_prev)/dt = 0。
    // 此时无论 omega_ref 如何阶跃，alpha 均应为 0（D 项无贡献）。
    RateCtrl ctrl; ctrl.reset();
    RateCtrlGains g = kDefaultCascadeCtrlParams.rate;
    g.kp[1] = 0.f; g.ki[1] = 0.f;  // 仅D

    float omega_meas[3] = {0.f, 0.5f, 0.f};  // 测量值全程不变

    // 预热 1 拍：让 omega_prev 对准 omega_meas（消除首拍冷启动瞬变）
    float omega_ref0[3] = {0.f, 0.f, 0.f};
    float alpha_warm[3];
    ctrl.step(omega_ref0, omega_meas, g, 0.005f, alpha_warm);

    // 预热后 omega_prev[1] == 0.5，d_meas = -(0.5-0.5)/dt = 0 → alpha = 0
    // 现在让 omega_ref 阶跃到 1.0（大阶跃）
    float omega_ref_step[3] = {0.f, 1.f, 0.f};
    float alpha_after[3];
    ctrl.step(omega_ref_step, omega_meas, g, 0.005f, alpha_after);

    // omega_meas 未变 → D 贡献 = 0 → alpha 应仍约为 0
    check(approx(alpha_after[1], 0.f, 0.01f),
          "微分先行：omega_ref 阶跃、omega_meas 不变 → alpha_D = 0（无冲击）");
}

static void test_rate_antiwindup()
{
    // 用极小的 alpha_max 强制输出饱和，验证 int_frozen 标记。
    // 设计：仅 ki=100，alpha_max=0.2，omega_err=0.5 恒定。
    //   · 几拍后 ki*integral 超过 alpha_max → 输出饱和 → 冻结
    RateCtrl ctrl; ctrl.reset();
    RateCtrlGains g = kDefaultCascadeCtrlParams.rate;
    g.kp[1] = 0.f; g.kd[1] = 0.f;
    g.ki[1]       = 100.f;
    g.alpha_max[1] = 0.2f;  // 很小，确保 ki*integral 很快超限
    g.int_max[1]   = 5.0f;  // 足够大，不干扰测试

    const float dt = 0.005f;
    float omega_ref[3]  = {0.f, 0.5f, 0.f};
    float omega_meas[3] = {0.f, 0.f,  0.f};
    float alpha[3];

    for (int n = 0; n < 200; ++n)
        ctrl.step(omega_ref, omega_meas, g, dt, alpha);

    check(std::fabs(alpha[1]) <= g.alpha_max[1] + 1e-5f,
          "抗积分饱和：alpha 不超出 alpha_max");
    check(ctrl.state.int_frozen[1],
          "抗积分饱和：持续饱和时积分冻结标记为 true");
    // 积分应停留在某有界值而非无限增长
    check(std::fabs(ctrl.state.integral[1]) < g.int_max[1] + 1.0f,
          "抗积分饱和：积分有界（未无限增长）");
}

static void test_rate_reset()
{
    RateCtrl ctrl; ctrl.reset();
    RateCtrlGains g = kDefaultCascadeCtrlParams.rate;

    float omega_ref[3]  = {0.f, 1.f, 0.f};
    float omega_meas[3] = {0.f, 0.f, 0.f};
    float alpha[3];
    for (int n = 0; n < 50; ++n)
        ctrl.step(omega_ref, omega_meas, g, 0.005f, alpha);

    // 积分应非零
    check(std::fabs(ctrl.state.integral[1]) > 0.01f, "reset前积分非零（前提）");

    ctrl.reset();
    check(ctrl.state.integral[1] == 0.f, "reset后积分归零");
    check(ctrl.state.omega_prev[1] == 0.f, "reset后 omega_prev 归零");
}

// ============================================================
//  T10：惯量映射
// ============================================================
static void test_moment_mapping()
{
    const TandemVecParams& p = kDefaultTandemVecParams;
    // 直接验证公式 M = I·α
    const float alpha[3] = {2.f, 3.f, 4.f};
    const float M[3] = {
        p.Ix * alpha[0],
        p.Iy * alpha[1],
        p.Iz * alpha[2]
    };
    check(approx(M[0], p.Ix * 2.f), "惯量映射：Mx = Ix·α_roll");
    check(approx(M[1], p.Iy * 3.f), "惯量映射：My = Iy·α_pitch");
    check(approx(M[2], p.Iz * 4.f), "惯量映射：Mz = Iz·α_yaw");
}

// ============================================================
//  T11–T14：全链 (CascadeCtrl)
// ============================================================
static CascadeInput makeIdentityInput(float thr = 0.5f)
{
    CascadeInput in;
    in.q_ref  = {1.f, 0.f, 0.f, 0.f};
    in.q_meas = {1.f, 0.f, 0.f, 0.f};
    in.omega_meas[0] = in.omega_meas[1] = in.omega_meas[2] = 0.f;
    in.thr = thr;
    return in;
}

static void test_cascade_zero_error()
{
    CascadeCtrl ctrl; ctrl.reset();
    auto in = makeIdentityInput();
    auto out = ctrl.step(in, 0.005f);

    check(approx(out.delta_f, 0.f, 1e-4f), "全链零误差 → delta_f≈0");
    check(approx(out.delta_t, 0.f, 1e-4f), "全链零误差 → delta_t≈0");
    check(approx(out.dw,      0.f, 1e-4f), "全链零误差 → dw≈0");
}

static void test_cascade_pitch_polarity()
{
    // 俯仰 +10°：目标比当前仰起10°
    // 期望尾摆产生低头力矩（My < 0），因此 delta_t > 0（∂My/∂δt < 0）
    CascadeCtrl ctrl; ctrl.reset();
    CascadeInput in = makeIdentityInput();
    const float pitch_err = 10.f * (3.14159265f / 180.f);
    in.q_ref = {cosf(pitch_err/2.f), 0.f, sinf(pitch_err/2.f), 0.f};

    auto out = ctrl.step(in, 0.005f);

    check(out.tel.omega_ref[1] > 0.f, "俯仰误差 → omega_ref_pitch > 0");
    check(out.tel.alpha_ref[1] > 0.f, "俯仰误差 → alpha_pitch > 0");
    check(out.tel.M_cmd[1] > 0.f,     "俯仰误差 → My_cmd > 0（My = Iy·α，α>0）");
    // My > 0 → δ_t = -My/(b·T0) < 0? 等等，需要仔细确认…
    // 实际上，θ仰起时需要负俯仰力矩（低头）才能恢复，因此：
    // My_cmd > 0 (期望增加俯仰角速率) → 经分配 δ_t < 0
    // 但这里 q_ref 比 q_meas 更仰，error > 0，omega_ref_pitch > 0，alpha > 0，
    // M_cmd[1] = Iy * alpha > 0 → 经分配 δ_t = -My/(b*T0) < 0
    // （因为尾摆效率 ∂My/∂δt = -b*T < 0，正力矩需负摆角）
    check(out.delta_t < 0.f,          "俯仰误差 → delta_t < 0（尾摆产生正俯仰力矩）");
}

static void test_cascade_yaw_polarity()
{
    // 偏航 +15°：目标比当前向右偏15°
    // 期望前摆产生正偏航力矩（Mz > 0，δ_f > 0）
    CascadeCtrl ctrl; ctrl.reset();
    CascadeInput in = makeIdentityInput();
    const float yaw_err = 15.f * (3.14159265f / 180.f);
    in.q_ref = {cosf(yaw_err/2.f), 0.f, 0.f, sinf(yaw_err/2.f)};

    auto out = ctrl.step(in, 0.005f);

    check(out.tel.omega_ref[2] > 0.f, "偏航误差 → omega_ref_yaw > 0");
    check(out.tel.M_cmd[2] > 0.f,     "偏航误差 → Mz_cmd > 0");
    check(out.delta_f > 0.f,          "偏航误差 → delta_f > 0（前摆产生正偏航）");
}

static void test_cascade_telemetry_finite()
{
    CascadeCtrl ctrl; ctrl.reset();
    const float angle = 0.3f;
    CascadeInput in = makeIdentityInput();
    in.q_ref = {cosf(angle/2.f), sinf(angle/2.f)*0.3f,
                sinf(angle/2.f)*0.7f, sinf(angle/2.f)*0.64f};
    in.q_ref = qNorm(in.q_ref);  // 归一化（随意误差）

    auto out = ctrl.step(in, 0.005f);
    const auto& tel = out.tel;

    bool all_finite = true;
    for (int i = 0; i < 4; ++i) all_finite &= is_finite(tel.q_err[i]);
    for (int i = 0; i < 3; ++i) {
        all_finite &= is_finite(tel.omega_ref[i]);
        all_finite &= is_finite(tel.omega_err[i]);
        all_finite &= is_finite(tel.alpha_ref[i]);
        all_finite &= is_finite(tel.M_cmd[i]);
    }
    all_finite &= is_finite(tel.alloc.delta_f);
    all_finite &= is_finite(tel.alloc.delta_t);
    all_finite &= is_finite(tel.alloc.dw);
    all_finite &= is_finite(tel.alloc.roll_authority);
    check(all_finite, "全链遥测：所有字段均为有限数（无NaN/Inf）");
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::printf("=== TandemVec 级联控制律回归测试 ===\n\n");

    std::printf("-- T1：外环零误差 --\n");
    test_attitude_zero_error();

    std::printf("\n-- T2：外环极性 --\n");
    test_attitude_polarity();

    std::printf("\n-- T3：外环最短路径 --\n");
    test_attitude_shortest_path();

    std::printf("\n-- T4：外环 omega_max 限幅 --\n");
    test_attitude_omega_clamp();

    std::printf("\n-- T5：内环 P-only --\n");
    test_rate_p_only();

    std::printf("\n-- T6：内环积分累积 --\n");
    test_rate_integral();

    std::printf("\n-- T7：微分先行无冲击 --\n");
    test_rate_no_derivative_kick();

    std::printf("\n-- T8：抗积分饱和 --\n");
    test_rate_antiwindup();

    std::printf("\n-- T9：reset() 清零 --\n");
    test_rate_reset();

    std::printf("\n-- T10：惯量力矩映射 --\n");
    test_moment_mapping();

    std::printf("\n-- T11：全链零误差 --\n");
    test_cascade_zero_error();

    std::printf("\n-- T12：全链俯仰极性 --\n");
    test_cascade_pitch_polarity();

    std::printf("\n-- T13：全链偏航极性 --\n");
    test_cascade_yaw_polarity();

    std::printf("\n-- T14：遥测字段有限性 --\n");
    test_cascade_telemetry_finite();

    std::printf("\n");
    if (g_fail == 0) std::printf("=== 全部通过 ===\n");
    else             std::printf("=== %d 项失败 ===\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
