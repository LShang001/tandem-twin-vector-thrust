/**
 * @file    test_alpha_tracking.cpp
 * @module  宿主机回归（共轴机型分配器 / 3DOF 角加速度跟踪）
 * @brief   把分配器接到刚体转动动力学上，做闭环 α 跟踪复核：
 *          ①名义：分配器认知惯量 = 真实惯量时，实际 α 应精确复现 α_cmd；
 *          ②失配：真实惯量 ×1.5 / ×0.67 时，开环分配只按 I_assumed/I_true
 *            缩放，不产生饱和/发散（用于量化 α 增益对惯量估计的敏感度）；
 *          ③饱和：超出物理权限时 α 钳位、摆角钳到 max_correction、可复算残差；
 *          ④偏航权限：差速行程是偏航 α 的硬上限，需同时置 kSatAlphaYaw
 *            与 kSatYawDiff。
 * @details 正向模型与 test_allocator_contracts.cpp::plantBodyMoment 同源：
 *          由电机 µs 反算推力与差速 → Mx=l·T·sinδ、My=l·T·sinδ_pitch、
 *          Mz=k_yaw·diff → body M={-w.Mz, w.My, -w.Mx}；再用积分器
 *          ω_{k+1}=ω_k+α·dt、α=M/I_true 推进。默认关闭 ω×Iω 前馈，
 *          与 CoaxialAllocatorParams::compensate_rate_coupling=false 一致。
 * @note    本测试不修改实现；只验证契约与敏感度。惯量仍为 MODEL-DEFAULT。
 * @see     docs/05-控制与分配/CTL-006、docs/07-验证与确认/VER-004 §11
 */
#include <cmath>
#include <cstdio>

#include "ControlContracts.h"
#include "CoaxialAllocator.h"

namespace
{
using namespace bfs;
using namespace bfs::control;

int g_pass = 0;
int g_fail = 0;

constexpr float kDeg2Rad = 0.017453292519943295f;
constexpr float kDt = 0.005f;

void expect(const bool ok, const char *name)
{
    if (ok)
    {
        ++g_pass;
        return;
    }
    ++g_fail;
    std::printf("[FAIL] %s\n", name);
}

void expectf(const bool ok, const char *name, const char *detail)
{
    if (ok)
    {
        ++g_pass;
        return;
    }
    ++g_fail;
    std::printf("[FAIL] %s | %s\n", name, detail);
}

/** @brief 无上下文细节的便捷重载 */
void expectf(const bool ok, const char *name)
{
    expect(ok, name);
}

/** @brief 3DOF 刚体转动状态（FRD，rad/s、rad/s²） */
struct Rigid3Dof
{
    float inertia_kgm2[3] = {0.05f, 0.05f, 0.08f};
    float omega_radps[3] = {0.0f, 0.0f, 0.0f};
    float alpha_radps2[3] = {0.0f, 0.0f, 0.0f};
};

struct StepResult
{
    CoaxialAllocatorOutput out;
    float alpha_actual[3] = {0.0f, 0.0f, 0.0f};
};

/** @brief 与 SIL plant 同源的正向力矩（由输出 µs/修正角反算 body M） */
void plantBodyMoment(const CoaxialAllocatorParams &p,
                     const CoaxialAllocatorOutput &out, float m_body[3])
{
    const float span = p.motor_max_us - p.motor_min_us;
    const float m1_pct = (out.motor1_us - p.motor_min_us) / span * 100.0f;
    const float m2_pct = (out.motor2_us - p.motor_min_us) / span * 100.0f;
    const float thrust = 0.5f * (m1_pct + m2_pct) * 0.01f * p.max_total_thrust_n;
    const float diff = (m1_pct - m2_pct) * 0.01f;
    const float w_mx = p.yaw_moment_nm_per_us * 500.0f * diff;
    const float w_my = p.lever_m * thrust * std::sin(out.servo_pitch_deg * kDeg2Rad);
    const float w_mz = -p.lever_m * thrust * std::sin(out.servo_roll_deg * kDeg2Rad);
    m_body[0] = -w_mz;
    m_body[1] = w_my;
    m_body[2] = -w_mx;
}

/** @brief 推进一步：分配 → 正向力矩 → α=M/I_true → ω 积分 */
StepResult step(Rigid3Dof &s, const CoaxialAllocatorParams &p,
                const float alpha_cmd[3], const float throttle_pct)
{
    CoaxialAllocatorInput in;
    for (int i = 0; i < 3; ++i)
    {
        in.alpha_body_radps2[i] = alpha_cmd[i];
        in.gyro_body_radps[i] = s.omega_radps[i];
    }
    in.throttle_pct = throttle_pct;
    in.thrust_estimate_n = throttle_pct * 0.01f * p.max_total_thrust_n;
    in.dt_s = kDt;

    StepResult r;
    r.out = allocateCoaxial(in, p);
    float m_body[3] = {0.0f, 0.0f, 0.0f};
    plantBodyMoment(p, r.out, m_body);
    for (int i = 0; i < 3; ++i)
    {
        r.alpha_actual[i] = m_body[i] / s.inertia_kgm2[i];
        s.alpha_radps2[i] = r.alpha_actual[i];
        s.omega_radps[i] += r.alpha_actual[i] * kDt;
    }
    return r;
}

/** @brief 跑 N 步，返回末段均值（用于稳态 α 跟踪判据） */
void runAndAverage(Rigid3Dof &s, const CoaxialAllocatorParams &p,
                   const float alpha_cmd[3], const float throttle_pct,
                   const int steps, const int avg_steps, float alpha_avg[3],
                   CoaxialAllocatorOutput *last_out, uint16_t *sat_union)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    const int begin = steps - avg_steps;
    for (int k = 0; k < steps; ++k)
    {
        const StepResult r = step(s, p, alpha_cmd, throttle_pct);
        if (k >= begin)
        {
            for (int i = 0; i < 3; ++i)
            {
                sum[i] += r.alpha_actual[i];
            }
        }
        if (last_out != nullptr)
        {
            *last_out = r.out;
        }
        if (sat_union != nullptr)
        {
            *sat_union = static_cast<uint16_t>(*sat_union | r.out.sat_mask);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        alpha_avg[i] = sum[i] / static_cast<float>(avg_steps);
    }
}

bool allFinite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

/** @brief 相对误差（分母取 |cmd| 与 1e-6 的较大者） */
float relError(const float actual, const float cmd)
{
    const float den = std::fmax(std::fabs(cmd), 1e-6f);
    return std::fabs(actual - cmd) / den;
}

void testNominalTracking()
{
    CoaxialAllocatorParams p;
    Rigid3Dof s; // 真实惯量 = 分配器认知惯量
    const float cmd[3] = {2.0f, -2.0f, 0.001f};
    float avg[3] = {0.0f, 0.0f, 0.0f};
    CoaxialAllocatorOutput last;
    uint16_t sat = kSatNone;
    runAndAverage(s, p, cmd, 60.0f, 200, 40, avg, &last, &sat);

    expectf(last.valid, "名义跟踪：分配器有效");
    expectf(sat == kSatNone, "名义跟踪：不应饱和");
    expectf(relError(avg[0], cmd[0]) < 0.005f, "名义 roll α 跟踪误差 <0.5%");
    expectf(relError(avg[1], cmd[1]) < 0.005f, "名义 pitch α 跟踪误差 <0.5%");
    expectf(relError(avg[2], cmd[2]) < 0.01f, "名义 yaw α 跟踪误差 <1%");
    expectf(allFinite3(avg), "名义跟踪：α 有限");
    expectf(sat == kSatNone, "名义跟踪：sat_mask 为空");
}

void testInertiaMismatch(const float factor, const float expected_ratio,
                         const char *name)
{
    CoaxialAllocatorParams p; // 分配器仍用名义 0.05/0.05/0.08
    Rigid3Dof s;
    for (int i = 0; i < 3; ++i)
    {
        s.inertia_kgm2[i] = p.inertia_kgm2[i] * factor;
    }
    const float cmd[3] = {2.0f, -2.0f, 0.001f};
    float avg[3] = {0.0f, 0.0f, 0.0f};
    CoaxialAllocatorOutput last;
    uint16_t sat = kSatNone;
    runAndAverage(s, p, cmd, 60.0f, 200, 40, avg, &last, &sat);

    const float ratio_roll = avg[0] / cmd[0];
    const float ratio_pitch = avg[1] / cmd[1];
    expectf(last.valid, name);
    expectf(std::fabs(ratio_roll - expected_ratio) < 0.01f,
            name, "roll α 比例应等于 I_assumed/I_true");
    expectf(std::fabs(ratio_pitch - expected_ratio) < 0.01f,
            name, "pitch α 比例应等于 I_assumed/I_true");
    expectf(sat == kSatNone, name, "失配不应触发饱和（权限仍充足）");
    expectf(allFinite3(avg), name, "失配：α 有限、未发散");
}

void testSaturation()
{
    CoaxialAllocatorParams p;
    Rigid3Dof s;
    const float cmd[3] = {100.0f, 0.0f, 0.0f};
    const StepResult r = step(s, p, cmd, 60.0f);
    expectf((r.out.sat_mask & kSatAlphaRoll) != 0,
            "饱和：置 kSatAlphaRoll");
    expectf((r.out.sat_mask & kSatTvcRoll) != 0,
            "饱和：TVC 同时钳到 max_correction");
    expectf(std::fabs(r.out.servo_roll_deg - p.max_correction_deg) < 1e-3f,
            "饱和：修正角钳到 max_correction");
    expectf(r.out.torque_residual_nm > 0.0f, "饱和：残差 > 0");
    expectf(std::isfinite(r.out.yaw_diff_us) && std::isfinite(r.out.motor1_us),
            "饱和：输出有限");
}

void testYawAuthority()
{
    CoaxialAllocatorParams p;
    Rigid3Dof s;
    const float cmd[3] = {0.0f, 0.0f, 0.01f};
    const StepResult r = step(s, p, cmd, 60.0f);
    expectf((r.out.sat_mask & kSatAlphaYaw) != 0,
            "偏航权限：置 kSatAlphaYaw");
    expectf((r.out.sat_mask & kSatYawDiff) != 0,
            "偏航权限：同时置 kSatYawDiff");
    expectf(std::fabs(r.out.yaw_diff_us - p.max_yaw_diff_us) < 1e-3f,
            "偏航权限：差速钳到 max_yaw_diff_us");
    expectf(std::isfinite(r.out.servo_roll_deg), "偏航权限：输出有限");
}

void testLegacyCommandIdentity()
{
    // alloc_mode=1 的语义证明：旧控制器修正角/差速 → α → 分配器逆解
    // 在未饱和时严格回到原值；换 I、换 T、换参数都不影响回路指令。
    const float servos[] = {-10.0f, -5.0f, -2.0f, 0.0f, 2.0f, 5.0f, 10.0f};
    const float yaws[] = {-180.0f, -90.0f, 0.0f, 90.0f, 180.0f};
    const float thrusts[] = {3.0f, 7.0f, 8.4f, 14.0f};
    const float scales[] = {0.5f, 1.0f, 2.0f};
    for (const float scale : scales)
    {
        CoaxialAllocatorParams p;
        p.inertia_kgm2[0] = 0.05f * scale;
        p.inertia_kgm2[1] = 0.05f * scale;
        p.inertia_kgm2[2] = 0.08f * scale;
        for (const float thrust : thrusts)
        {
            for (const float roll : servos)
            {
                for (const float pitch : servos)
                {
                    for (const float yaw : yaws)
                    {
                        float alpha[3] = {0.0f, 0.0f, 0.0f};
                        const bool ok = coaxialLegacyToAlpha(
                            roll, pitch, yaw, thrust, p, alpha);
                        CoaxialAllocatorInput in;
                        in.alpha_body_radps2[0] = alpha[0];
                        in.alpha_body_radps2[1] = alpha[1];
                        in.alpha_body_radps2[2] = alpha[2];
                        in.gyro_body_radps[0] = 0.0f;
                        in.gyro_body_radps[1] = 0.0f;
                        in.gyro_body_radps[2] = 0.0f;
                        in.throttle_pct =
                            100.0f * thrust / p.max_total_thrust_n;
                        in.thrust_estimate_n = thrust;
                        in.dt_s = kDt;
                        const CoaxialAllocatorOutput out =
                            allocateCoaxial(in, p);
                        // 旧 mix 先把差速夹到电机余量（满油门正向差速=0），
                        // 因此期望值要与 mapCoaxialVtvOutputs 同口径。
                        const float base_us =
                            p.motor_min_us +
                            in.throttle_pct * 0.01f *
                                (p.motor_max_us - p.motor_min_us);
                        const float expected_yaw =
                            controlClamp(yaw, p.motor_min_us - base_us,
                                         p.motor_max_us - base_us);
                        const float expected_m1 = controlClamp(
                            base_us - expected_yaw, p.motor_min_us,
                            p.motor_max_us);
                        const float expected_m2 = controlClamp(
                            base_us + expected_yaw, p.motor_min_us,
                            p.motor_max_us);
                        const bool recovered =
                            ok && out.valid &&
                            std::fabs(out.servo_roll_deg - roll) < 1e-3f &&
                            std::fabs(out.servo_pitch_deg - pitch) < 1e-3f &&
                            std::fabs(out.motor1_us - expected_m1) < 1e-2f &&
                            std::fabs(out.motor2_us - expected_m2) < 1e-2f;
                        expectf(recovered,
                                "legacy 指令→α→逆解：与旧 mix 逐值一致");
                    }
                }
            }
        }
    }

    // I 不同：α 的数值不同，但闭环指令相同（惯量在正逆中相消）。
    CoaxialAllocatorParams p1;
    CoaxialAllocatorParams p2;
    p2.inertia_kgm2[0] = p1.inertia_kgm2[0] * 2.0f;
    p2.inertia_kgm2[1] = p1.inertia_kgm2[1] * 2.0f;
    p2.inertia_kgm2[2] = p1.inertia_kgm2[2] * 2.0f;
    float a1[3] = {0.0f, 0.0f, 0.0f};
    float a2[3] = {0.0f, 0.0f, 0.0f};
    const bool ok1 = coaxialLegacyToAlpha(4.0f, -3.0f, 60.0f, 8.4f, p1, a1);
    const bool ok2 = coaxialLegacyToAlpha(4.0f, -3.0f, 60.0f, 8.4f, p2, a2);
    expectf(ok1 && ok2, "legacy→α：参数合法");
    expectf(std::fabs(a1[0] - 2.0f * a2[0]) < 1e-6f,
            "legacy→α：α 与 1/I 成正比（I×2 → α/2）");
    expectf(std::fabs(a1[2] - 2.0f * a2[2]) < 1e-9f,
            "legacy→α：偏航 α 与 1/Iz 成正比");

    CoaxialAllocatorInput in1;
    CoaxialAllocatorInput in2;
    in1.alpha_body_radps2[0] = a1[0];
    in1.alpha_body_radps2[1] = a1[1];
    in1.alpha_body_radps2[2] = a1[2];
    in2.alpha_body_radps2[0] = a2[0];
    in2.alpha_body_radps2[1] = a2[1];
    in2.alpha_body_radps2[2] = a2[2];
    in1.throttle_pct = in2.throttle_pct = 60.0f;
    in1.thrust_estimate_n = in2.thrust_estimate_n = 8.4f;
    in1.dt_s = in2.dt_s = kDt;
    const CoaxialAllocatorOutput o1 = allocateCoaxial(in1, p1);
    const CoaxialAllocatorOutput o2 = allocateCoaxial(in2, p2);
    expectf(o1.valid && o2.valid &&
                std::fabs(o1.servo_roll_deg - o2.servo_roll_deg) < 1e-3f &&
                std::fabs(o1.servo_pitch_deg - o2.servo_pitch_deg) < 1e-3f &&
                std::fabs(o1.yaw_diff_us - o2.yaw_diff_us) < 1e-2f,
            "I 估计不同：回路指令仍相同（惯量相消）");
}

} // namespace

int main()
{
    std::printf("=== 3DOF α 跟踪（CoaxialAllocator 闭环）===\n");
    testNominalTracking();
    testInertiaMismatch(1.5f, 1.0f / 1.5f, "惯量 ×1.5 失配");
    testInertiaMismatch(0.67f, 1.0f / 0.67f, "惯量 ×0.67 失配");
    testSaturation();
    testYawAuthority();
    testLegacyCommandIdentity();
    std::printf("--- %d passed, %d failed ---\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
