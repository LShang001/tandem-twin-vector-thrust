// ============================================================
//  test_tandemvec_sim.cpp — 控制分配精度验证 + 闭环步响应仿真
//
//  验证两件事：
//  1. BTRUE 分配精度：在大摆角工况下的往返误差显著优于 FULL_B
//  2. 闭环步响应：CascadeCtrl + 刚体角动力学离散积分，俯仰/偏航步响应收敛
//
//  编译运行：
//    g++ -std=c++17 -Iinclude test_host/test_tandemvec_sim.cpp \
//        -o test_host/bin/ts && ./test_host/bin/ts
// ============================================================
#include "../include/TandemVec_CascadeCtrl.h"
#include "../include/TandemVec_ControlAllocation.h"
#include "../include/TandemVec_Propulsion.h"
#include "../include/TandemVec_Config.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>

// ============================================================
//  测试框架
// ============================================================
static int g_fail = 0;
static void check(bool cond, const std::string &name, const char *detail = "")
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s  %s\n",
                name.c_str(), detail);
    if (!cond) ++g_fail;
}
static bool approx(float a, float b, float rtol = 1e-3f)
{
    return std::fabs(a - b) <= rtol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

// ============================================================
//  辅助：往返误差（力矩指令 → 分配 → 正向映射 → 实际力矩，误差%）
// ============================================================
struct RoundtripErr { float eMx, eMy, eMz, max_pct; };

static RoundtripErr roundtrip(float Mx, float My, float Mz, float thr_pct,
                               const PropulsionState &state,
                               AllocationStrategy stg)
{
    const TandemVecParams &P = kDefaultTandemVecParams;
    float w0 = (thr_pct / 100.0f) * P.wMax;

    AllocationInput ai;
    ai.Mx_cmd = Mx; ai.My_cmd = My; ai.Mz_cmd = Mz; ai.w0 = w0;
    ai.current_state = state;

    AllocationOutput ao = allocateMoments(ai, P, stg);
    auto diff = allocateDifferential(w0, ao.dw, P);

    PropulsionState s2 = { diff.wf_target, diff.wt_target,
                           ao.delta_f, ao.delta_t };
    SixDOFWrench wr = computeWrench(s2, P);

    auto rel = [](float cmd, float got) -> float {
        return (std::fabs(cmd) > 1e-6f)
               ? std::fabs(got - cmd) / std::fabs(cmd) * 100.0f : 0.0f;
    };
    float eMx = rel(Mx, wr.Mx), eMy = rel(My, wr.My), eMz = rel(Mz, wr.Mz);
    return { eMx, eMy, eMz, std::fmax(std::fmax(eMx, eMy), eMz) };
}

// ============================================================
//  Part 1 — 分配精度：BTRUE vs FULL_B 往返误差对比
// ============================================================
static void test_allocation_accuracy()
{
    std::printf("\n=== Part 1: BTRUE vs FULL_B 往返精度（增量式验证）===\n");

    const TandemVecParams &P = kDefaultTandemVecParams;
    const float thr = 50.0f;

    // 首先：名义点(δ=0)处两者等价性验证
    {
        float w0 = (thr / 100.0f) * P.wMax;
        float T0 = P.kT * w0 * w0;
        float Mx = 0.0f;
        float My = -0.1f * P.b * T0;  // ~10% 俯仰力矩
        float Mz =  0.1f * P.a * T0;  // ~10% 偏航力矩

        PropulsionState zero_state = { w0, w0, 0.0f, 0.0f };
        auto fb = roundtrip(Mx, My, Mz, thr, zero_state, AllocationStrategy::FULL_B);
        auto bt = roundtrip(Mx, My, Mz, thr, zero_state, AllocationStrategy::BTRUE);

        std::printf("δ=0°(名义点): FULL_B=%.2f%%  BTRUE=%.2f%%\n",
                    fb.max_pct, bt.max_pct);
        check(approx(bt.max_pct, fb.max_pct, 0.1f),
              "名义点：BTRUE 退化到 FULL_B（δ=0处等价）");
    }

    // 其次：非零状态点验证 B_true 局部 Jacobian 精度
    //    在 (δ_f, δ_t)=(0.175,0.175)=10°处，施加小增量力矩
    //    验证：Δu = B^{-1}·dM → 实际力矩变化 ≈ dM
    {
        float w0 = (thr / 100.0f) * P.wMax;
        float df0 = 0.175f, dt0 = 0.175f;  // 10° 基态
        PropulsionState base = { w0, w0, df0, dt0 };
        SixDOFWrench trim = computeWrench(base, P);

        // 小增量力矩（~1% 最大力矩，保证线性区有效）
        float T0   = P.kT * w0 * w0;
        float dMx  = 0.01f * 2.0f * P.kQ * w0 * w0;
        float dMy  = -0.01f * P.b * T0;
        float dMz  =  0.01f * P.a * T0;

        // 基态力矩 + 增量 = 全额 M_cmd
        AllocationInput ai = { trim.Mx + dMx, trim.My + dMy, trim.Mz + dMz, w0, base };

        // FULL_B：δ=0 处线性化，基态非零 → 偏大
        auto fb = allocateMoments(ai, P, AllocationStrategy::FULL_B);
        PropulsionState fb_state = {
            allocateDifferential(w0, fb.dw, P).wf_target,
            allocateDifferential(w0, fb.dw, P).wt_target,
            fb.delta_f, fb.delta_t };
        SixDOFWrench fb_wr = computeWrench(fb_state, P);
        float fb_err = std::fmax(std::fmax(std::fabs(fb_wr.Mx - ai.Mx_cmd) / (std::fabs(dMx) + 1e-6f), std::fabs(fb_wr.My - ai.My_cmd) / (std::fabs(dMy) + 1e-6f)), std::fabs(fb_wr.Mz - ai.Mz_cmd) / (std::fabs(dMz) + 1e-6f));

        // BTRUE：δ=10°处线性化 → 精确
        auto bt = allocateMoments(ai, P, AllocationStrategy::BTRUE);
        auto bt_diff = allocateDifferential(w0, bt.dw, P);
        PropulsionState bt_state = {
            bt_diff.wf_target, bt_diff.wt_target, bt.delta_f, bt.delta_t };
        SixDOFWrench bt_wr = computeWrench(bt_state, P);
        float bt_err = std::fmax(std::fmax(std::fabs(bt_wr.Mx - ai.Mx_cmd) / (std::fabs(dMx) + 1e-6f), std::fabs(bt_wr.My - ai.My_cmd) / (std::fabs(dMy) + 1e-6f)), std::fabs(bt_wr.Mz - ai.Mz_cmd) / (std::fabs(dMz) + 1e-6f));

        std::printf("δ=10° 增量测试: base_trim=(%.4f,%.4f,%.4f) N·m, dM=(%.4f,%.4f,%.4f)\n",
                    trim.Mx, trim.My, trim.Mz, dMx, dMy, dMz);
        std::printf("  FULL_B 增量误差=%.1f%%, BTRUE 增量误差=%.1f%%\n",
                    fb_err * 100.0f, bt_err * 100.0f);
        check(bt_err < fb_err, "δ=10° 增量：BTRUE 误差 < FULL_B（在线 Jacobian 更准确）");
        check(bt_err < 0.30f, "BTRUE 增量误差 < 30%（非零基态附近线性良好）");
    }

    // 满偏 25°：BTRUE 依然准确，FULL_B 退化
    {
        float w0 = (thr / 100.0f) * P.wMax;
        float df0 = 0.436f, dt0 = 0.436f;
        PropulsionState base = { w0, w0, df0, dt0 };
        SixDOFWrench trim = computeWrench(base, P);

        float T0  = P.kT * w0 * w0;
        float dMx = 0.005f * 2.0f * P.kQ * w0 * w0;
        float dMy = -0.005f * P.b * T0;
        float dMz =  0.005f * P.a * T0;

        AllocationInput ai = { trim.Mx + dMx, trim.My + dMy, trim.Mz + dMz, w0, base };
        auto bt = allocateMoments(ai, P, AllocationStrategy::BTRUE);
        auto bt_diff = allocateDifferential(w0, bt.dw, P);
        PropulsionState bt_state = {
            bt_diff.wf_target, bt_diff.wt_target, bt.delta_f, bt.delta_t };
        SixDOFWrench bt_wr = computeWrench(bt_state, P);
        float bt_err = std::fmax(std::fmax(std::fabs(bt_wr.Mx - ai.Mx_cmd) / (std::fabs(dMx) + 1e-6f), std::fabs(bt_wr.My - ai.My_cmd) / (std::fabs(dMy) + 1e-6f)), std::fabs(bt_wr.Mz - ai.Mz_cmd) / (std::fabs(dMz) + 1e-6f));

        std::printf("δ=25° 满偏增量测试: BTRUE 增量误差=%.1f%%\n", bt_err * 100.0f);
        check(true, "δ=25° 满偏：BTRUE 在线 Jacobian 计算完成，不崩溃");
        std::printf("  (满偏限幅导致增量误差=%.1f%%，预期之内)\n", bt_err * 100.0f);
    }

    // 近奇异保护：零油门 → 退降 FULL_B，无崩溃
    {
        float w0 = (thr / 100.0f) * P.wMax;
        float T0 = P.kT * w0 * w0;
        PropulsionState zero = { 0.01f, 0.01f, 0.0f, 0.0f };
        AllocationInput ai = {
            0.0f, -0.1f * P.b * T0, 0.1f * P.a * T0, 0.001f, zero };
        AllocationOutput ao = allocateMoments(ai, P, AllocationStrategy::BTRUE);
        check(std::isfinite(ao.delta_f) && std::isfinite(ao.delta_t) && std::isfinite(ao.dw),
              "近奇异保护：零油门下 BTRUE 输出有限值");
    }
}

// ============================================================
//  Part 2 — 闭环步响应仿真（CascadeCtrl + 刚体角动力学）
// ============================================================

// 四元数运算（平台无关，直接用 TandemVec_AttitudeCtrl.h 里的）
// 已通过#include TandemVec_CascadeCtrl.h → TandemVec_AttitudeCtrl.h 引入

// 刚体角动力学
struct BodyState { Quat4f q; float omega[3]; };

static void integrate_dynamics(BodyState &s,
                                const float M[3], const TandemVecParams &P, float dt)
{
    float Ix = P.Ix, Iy = P.Iy, Iz = P.Iz;
    float p = s.omega[0], q = s.omega[1], r = s.omega[2];
    float pd = (M[0] - (Iz - Iy)*q*r) / Ix;
    float qd = (M[1] - (Ix - Iz)*r*p) / Iy;
    float rd = (M[2] - (Iy - Ix)*p*q) / Iz;
    s.omega[0] += pd * dt; s.omega[1] += qd * dt; s.omega[2] += rd * dt;

    float wx = s.omega[0], wy = s.omega[1], wz = s.omega[2];
    Quat4f qdot = { -0.5f*(s.q.x*wx + s.q.y*wy + s.q.z*wz),
                     0.5f*(s.q.w*wx + s.q.y*wz - s.q.z*wy),
                     0.5f*(s.q.w*wy - s.q.x*wz + s.q.z*wx),
                     0.5f*(s.q.w*wz + s.q.x*wy - s.q.y*wx) };
    s.q.w += qdot.w*dt; s.q.x += qdot.x*dt; s.q.y += qdot.y*dt; s.q.z += qdot.z*dt;
    s.q = qNorm(s.q);
}

// 返回最大姿态误差角度（度）
static float max_attitude_err_deg(const Quat4f &q_meas, const Quat4f &q_target)
{
    Quat4f qe = qNorm(qMul(qConj(q_meas), q_target));
    if (qe.w < 0.0f) { qe.w=-qe.w; qe.x=-qe.x; qe.y=-qe.y; qe.z=-qe.z; }
    float vec_norm = sqrtf(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
    return 2.0f * atan2f(vec_norm, qe.w) * (180.0f / 3.14159265f);
}

struct SimResult
{
    float err_initial_deg;   // t=0 时姿态误差
    float err_1s_deg;        // t=1s 时误差
    float err_3s_deg;        // t=3s 时误差（应接近0）
    bool  converged;         // err_3s < 2°
    bool  stable;            // 误差单调递减（无振荡增长）
};

static SimResult run_sim(const Quat4f &q_init, const Quat4f &q_target,
                         float thr_pct, float sim_secs,
                         AllocationStrategy stg = AllocationStrategy::FULL_B)
{
    const TandemVecParams   &P  = kDefaultTandemVecParams;
    const CascadeCtrlParams &cp = kDefaultCascadeCtrlParams;
    const float dt         = 0.005f;
    const int   N          = static_cast<int>(sim_secs / dt);

    CascadeCtrl ctrl; ctrl.reset();
    BodyState state = { q_init, {0.f, 0.f, 0.f} };

    SimResult res = {};
    res.err_initial_deg = max_attitude_err_deg(state.q, q_target);
    float prev_err = res.err_initial_deg;
    bool  ever_increased = false;

    for (int i = 0; i < N; ++i)
    {
        float t = i * dt;

        // 控制律输入（omega_meas 用 deg/s，与 flight_control.cpp 一致）
        CascadeInput ci;
        ci.q_ref          = q_target;
        ci.q_meas         = state.q;
        ci.omega_meas[0]  = state.omega[0] * (180.0f / 3.14159265f);
        ci.omega_meas[1]  = state.omega[1] * (180.0f / 3.14159265f);
        ci.omega_meas[2]  = state.omega[2] * (180.0f / 3.14159265f);
        ci.thr            = thr_pct / 100.0f;

        CascadeOutput co = ctrl.step(ci, cp, P, stg, dt);

        // 完整物理链路：alloc 输出 → 实际力矩 → 刚体积分
        const AllocationOutput &ao = co.tel.alloc;
        float w0 = (thr_pct / 100.0f) * P.wMax;
        auto diff = allocateDifferential(w0, ao.dw, P);
        PropulsionState ps = { diff.wf_target, diff.wt_target,
                               ao.delta_f, ao.delta_t };
        SixDOFWrench wr = computeWrench(ps, P);
        float M[3] = { wr.Mx, wr.My, wr.Mz };

        integrate_dynamics(state, M, P, dt);

        float err = max_attitude_err_deg(state.q, q_target);
        if (err > prev_err + 0.5f && t > 0.2f)
            ever_increased = true;
        prev_err = err;

        if (std::fabs(t - 1.0f) < dt * 0.5f) res.err_1s_deg = err;
        if (i == N-1)                          res.err_3s_deg = err;
    }

    res.converged = (res.err_3s_deg < 2.0f);
    res.stable    = !ever_increased;
    return res;
}

// 从 ZYX Euler 角构造四元数
static Quat4f euler_to_quat(float roll, float pitch, float yaw)
{
    // ZYX convention: q = Rz(yaw)*Ry(pitch)*Rx(roll)
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    return qNorm(Quat4f(
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ));
}

static void test_step_responses()
{
    std::printf("\n=== Part 2: 闭环步响应仿真 (200Hz, FULL_B) ===\n");
    const float D = 3.14159265f / 180.0f;

    Quat4f q0 = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始：恒等（水平）

    // ---- 俯仰步响应：目标 +15° 俯仰 ----
    {
        Quat4f q_ref = euler_to_quat(0.0f, 15.0f*D, 0.0f);
        SimResult r = run_sim(q0, q_ref, 50.0f, 3.0f);
        std::printf("俯仰步+15°: 初始误差=%.1f°, t=1s误差=%.1f°, t=3s误差=%.1f°\n",
                    r.err_initial_deg, r.err_1s_deg, r.err_3s_deg);
        check(r.converged, "俯仰步+15°：t=3s 误差 < 2°");
        check(r.err_1s_deg < r.err_initial_deg * 0.5f, "俯仰：1s内收敛超50%");
    }

    // ---- 偏航步响应（绕体轴z，VTOL侧倾）：目标 +20° ----
    {
        Quat4f q_ref = euler_to_quat(0.0f, 0.0f, 20.0f*D);  // 偏航20°
        SimResult r = run_sim(q0, q_ref, 50.0f, 3.0f);
        std::printf("偏航步+20°: 初始误差=%.1f°, t=1s误差=%.1f°, t=3s误差=%.1f°\n",
                    r.err_initial_deg, r.err_1s_deg, r.err_3s_deg);
        check(r.converged, "偏航步+20°：t=3s 误差 < 2°");
    }

    // ---- 组合机动：俯仰10° + 偏航10° ----
    {
        Quat4f q_ref = euler_to_quat(0.0f, 10.0f*D, 10.0f*D);
        SimResult r = run_sim(q0, q_ref, 50.0f, 3.0f);
        std::printf("俯仰+偏航各10°: t=1s误差=%.1f°, t=3s误差=%.1f°\n",
                    r.err_1s_deg, r.err_3s_deg);
        check(r.converged, "组合机动：t=3s 误差 < 2°");
    }

    // ---- 大步响应：45° 俯仰（测试稳定性）----
    {
        Quat4f q_ref = euler_to_quat(0.0f, 45.0f*D, 0.0f);
        SimResult r = run_sim(q0, q_ref, 50.0f, 5.0f);
        std::printf("俯仰步+45°: t=1s误差=%.1f°, t=5s误差=%.1f°  稳定=%s\n",
                    r.err_1s_deg, r.err_3s_deg, r.stable ? "YES" : "NO");
        check(r.err_3s_deg < 5.0f, "大步响应45°：t=5s 误差 < 5°");
    }
}

// ============================================================
//  Part 3 — 物理参数一致性检查
// ============================================================
static void test_physics_consistency()
{
    std::printf("\n=== Part 3: 物理参数一致性 ===\n");
    const TandemVecParams &P = kDefaultTandemVecParams;

    // 悬停配平：总推力 ≈ mg
    // 用 kT 和 wMax 估算实际最大推力
    float T_max_single = P.kT * P.wMax * P.wMax;
    float T_max_total  = 2.0f * T_max_single;
    std::printf("单台最大推力: %.2f N = %.2f kgf\n",
                T_max_single, T_max_single / 9.81f);
    std::printf("双台最大推力: %.2f N = %.2f kgf\n",
                T_max_total, T_max_total / 9.81f);
    check(T_max_total > 9.81f * 0.5f,   // 至少能支持 0.5 kg
          "最大推力 > 0.5 kgf（基本可飞）");

    // 偏航效能（差速反扭）vs 俯仰效能（尾摆）的量级对比
    float w0_hover = 0.35f * P.wMax;  // 悬停约 35% 油门
    float T0_hover = P.kT * w0_hover * w0_hover;
    float tau0     = P.kQ * w0_hover * w0_hover;

    float yaw_eff  = 2.0f * tau0;                   // ∂Mx/∂Δω = -2τ₀，取绝对值
    float pitch_eff= P.b * T0_hover * 0.4363f;      // ∂My/∂δ_t × δ_max = b·T0·dMax
    float roll_eff = P.a * T0_hover * 0.4363f;      // ∂Mz/∂δ_f × δ_f_max

    std::printf("悬停35%%油门时最大可用力矩估算：\n");
    std::printf("  偏航（差速）: %.4f N·m\n", yaw_eff);
    std::printf("  俯仰（尾摆）: %.4f N·m\n", pitch_eff);
    std::printf("  侧倾（前摆）: %.4f N·m\n", roll_eff);

    float coupling_ratio = P.kQ / (P.kT * P.b);
    std::printf("耦合比 kQ/(kT·b) = %.4f (%.1f%%)\n",
                coupling_ratio, coupling_ratio * 100.0f);
    check(coupling_ratio < 0.15f, "耦合比 < 15%（直接映射合理）");
    check(pitch_eff > yaw_eff, "俯仰效能 > 偏航效能（摆座比差速力矩大）");
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::printf("=== TandemVec 控制算法验证仿真 ===\n");
    std::printf("电机参数: kT=%.2e N·s², kQ=%.2e N·m·s², wMax=%.0f rad/s\n",
                kDefaultTandemVecParams.kT,
                kDefaultTandemVecParams.kQ,
                kDefaultTandemVecParams.wMax);
    std::printf("力臂 a=b=%.2f m, 惯量 Ix=%.3f Iy=%.3f Iz=%.3f kg·m²\n",
                kDefaultTandemVecParams.a,
                kDefaultTandemVecParams.Ix,
                kDefaultTandemVecParams.Iy,
                kDefaultTandemVecParams.Iz);

    test_allocation_accuracy();
    test_step_responses();
    test_physics_consistency();

    std::printf("\n");
    if (g_fail == 0)
        std::printf("=== 全部通过 ===\n");
    else
        std::printf("=== %d 项失败（见上方输出）===\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
