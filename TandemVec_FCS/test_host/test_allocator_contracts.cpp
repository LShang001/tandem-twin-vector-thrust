/**
 * @file    test_allocator_contracts.cpp
 * @module  宿主机回归（控制契约 / 共轴机型分配器）
 * @brief   对 include/ControlContracts.h 与 include/CoaxialAllocator.h 的独立复核：
 *          逆解往返、轴序与符号（对齐 SIL plant）、低推力/零推力边界、非有限输入、
 *          残差语义、名义点与 legacy 手感一致
 * @details 纯平台无关，不链接固件；正向模型逐行复刻
 *          test_host/fw_sil/fw_sil_env_main.cpp:278 的 `M = {-w.Mz, w.My, -w.Mx}`
 *          与 computeCoaxialWrench()（Mx=l·T·sinδ_roll、My=l·T·sinδ_pitch、
 *          Mz=k_yaw·yaw_diff），因此若“分配器逆解”与“plant 正向模型”轴序/符号
 *          不一致，本测试会直接失败，而不是靠文档自述。
 * @note    本文件只做复核，不修改被测实现；发现不一致只报告，不在此修补。
 * @see     docs/07-验证与确认/VER-004、CTL-003 §2
 */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "ControlContracts.h"
#include "CoaxialAllocator.h"

namespace
{
using namespace bfs;
using namespace bfs::control;

int g_pass = 0;
int g_fail = 0;

constexpr float kDeg2Rad = 0.017453292519943295f;
constexpr float kRad2Deg = 57.29577951308232f;
constexpr float kG = 9.80665f;

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

/** @brief 无名称重载（结构自检等静态断言） */
void expect(const bool ok)
{
    expect(ok, "(未命名断言)");
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

/** @brief 无上下文细节的便捷重载（静态用例直接用名称断言） */
void expectf(const bool ok, const char *name)
{
    expect(ok, name);
}

CoaxialAllocatorParams defaultParams()
{
    return CoaxialAllocatorParams{};
}

float motorBaseUs(const CoaxialAllocatorParams &p, const float throttle_pct)
{
    const float span = p.motor_max_us - p.motor_min_us;
    return p.motor_min_us + controlClamp(throttle_pct, 0.0f, 100.0f) * 0.01f * span;
}

/**
 * @brief 正向模型：由期望修正角/差速构造 α（与 SIL plant 同一套力矩公式）
 * @note  τ = I·α；Mx=l·T·sinδ_roll、My=l·T·sinδ_pitch、Mz=k_yaw·yaw_diff
 */
struct ForwardCase
{
    CoaxialAllocatorInput in;
    float tau_nm[3];
};

ForwardCase makeForward(const CoaxialAllocatorParams &p, const float thrust_n,
                        const float droll_deg, const float dpitch_deg,
                        const float yaw_diff_us, const float throttle_pct)
{
    ForwardCase fc{};
    fc.tau_nm[0] = p.lever_m * thrust_n * std::sin(droll_deg * kDeg2Rad);
    fc.tau_nm[1] = p.lever_m * thrust_n * std::sin(dpitch_deg * kDeg2Rad);
    fc.tau_nm[2] = p.yaw_moment_nm_per_us * yaw_diff_us;
    fc.in.alpha_body_radps2[0] = fc.tau_nm[0] / p.inertia_kgm2[0];
    fc.in.alpha_body_radps2[1] = fc.tau_nm[1] / p.inertia_kgm2[1];
    fc.in.alpha_body_radps2[2] = fc.tau_nm[2] / p.inertia_kgm2[2];
    fc.in.throttle_pct = throttle_pct;
    fc.in.thrust_estimate_n = thrust_n;
    fc.in.dt_s = 0.005f;
    return fc;
}

/** @brief 用 SIL plant 公式从分配器输出反算 body 三轴力矩 */
void plantBodyMoment(const CoaxialAllocatorParams &p,
                     const CoaxialAllocatorOutput &out, float m_body[3])
{
    const float span = p.motor_max_us - p.motor_min_us;
    const float m1_pct = (out.motor1_us - p.motor_min_us) / span * 100.0f;
    const float m2_pct = (out.motor2_us - p.motor_min_us) / span * 100.0f;
    // plant: avg=0.5(wf+wt)/wMax → 推力；diff=(wf-wt)/wMax → 差速归一
    const float thrust = 0.5f * (m1_pct + m2_pct) * 0.01f * p.max_total_thrust_n;
    const float diff = (m1_pct - m2_pct) * 0.01f;
    const float roll = out.servo_roll_deg * kDeg2Rad;
    const float pitch = out.servo_pitch_deg * kDeg2Rad;
    // 与 fw_sil_env_main.cpp:362-364 一致；k_yaw_norm = k_yaw_per_us·500µs
    const float w_mx = p.yaw_moment_nm_per_us * 500.0f * diff;
    const float w_my = p.lever_m * thrust * std::sin(pitch);
    const float w_mz = -p.lever_m * thrust * std::sin(roll);
    // 与 fw_sil_env_main.cpp:278 一致：body M = {-w.Mz, w.My, -w.Mx}
    m_body[0] = -w_mz;
    m_body[1] = w_my;
    m_body[2] = -w_mx;
}

/** @brief 残差契约：max(|τ_requested-τ_ach|) 与回报值一致（未钳位请求为基准） */
bool residualMatches(const CoaxialAllocatorOutput &out, const float tol_nm)
{
    float expect_res = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
        expect_res = std::fmax(
            expect_res,
            std::fabs(out.torque_requested_nm[i] - out.torque_achieved_nm[i]));
    }
    return std::fabs(out.torque_residual_nm - expect_res) <= tol_nm;
}

// ---------------------------------------------------------------------------
// 1) 逆解往返：T×δr×δp×yaw 全组合
// ---------------------------------------------------------------------------
void testRoundTrip()
{
    const CoaxialAllocatorParams p = defaultParams();
    const float Ts[] = {2.0f, 4.0f, 8.0f, 12.0f, 14.0f};
    const float drs[] = {-10.0f, -5.0f, -0.1f, 0.0f, 3.0f, 10.0f};
    const float dps[] = {-10.0f, -5.0f, -0.1f, 0.0f, 3.0f, 10.0f};
    const float ys[] = {-180.0f, -60.0f, 0.0f, 90.0f, 180.0f};

    int cases = 0;
    float max_delta_err_deg = 0.0f;
    float max_yaw_err_us = 0.0f;
    for (const float T : Ts)
    {
        const float throttle = T / p.max_total_thrust_n * 100.0f;
        const float base_us = motorBaseUs(p, throttle);
        for (const float dr : drs)
        {
            for (const float dp : dps)
            {
                for (const float y : ys)
                {
                    ++cases;
                    const ForwardCase fc = makeForward(p, T, dr, dp, y, throttle);
                    const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);

                    char ctx[192];
                    std::snprintf(ctx, sizeof(ctx),
                                  "T=%.2fN δr=%.2f° δp=%.2f° yaw=%.1fµs", T, dr, dp, y);
                    expectf(out.valid, "往返: valid", ctx);
                    if (!out.valid)
                    {
                        continue;
                    }

                    // 与旧 mix 同口径：先把差速夹到 [motor_min-base,
                    // motor_max-base]，再逐台钳位；因此可能只有一台电机撞限。
                    const float diff_lo2 = p.motor_min_us - base_us;
                    const float diff_hi2 = p.motor_max_us - base_us;
                    const float y_clamped =
                        std::fmin(std::fmax(y, diff_lo2), diff_hi2);
                    const bool yaw_limited = std::fabs(y_clamped - y) > 1.0e-3f;
                    const float m1_raw = base_us - y_clamped;
                    const float m2_raw = base_us + y_clamped;
                    const bool motor_clamp =
                        m1_raw < p.motor_min_us - 1.0e-4f ||
                        m1_raw > p.motor_max_us + 1.0e-4f ||
                        m2_raw < p.motor_min_us - 1.0e-4f ||
                        m2_raw > p.motor_max_us + 1.0e-4f;
                    const bool at_mech_limit =
                        std::fabs(dr) >= 9.999f || std::fabs(dp) >= 9.999f;

                    const float delta_err =
                        std::fmax(std::fabs(out.servo_roll_deg - dr),
                                  std::fabs(out.servo_pitch_deg - dp));
                    max_delta_err_deg = std::fmax(max_delta_err_deg, delta_err);
                    expectf(delta_err < 0.05f, "往返: δ 误差<0.05°", ctx);

                    if (!at_mech_limit)
                    {
                        const uint16_t axis_bits = static_cast<uint16_t>(
                            kSatAlphaRoll | kSatAlphaPitch | kSatTvcRoll | kSatTvcPitch);
                        expectf((out.sat_mask & axis_bits) == 0u,
                                "往返: 未饱和不误报轴位饱和", ctx);
                    }

                    if (yaw_limited || motor_clamp)
                    {
                        expectf((out.sat_mask & (kSatYawDiff | kSatMotorClamp)) != 0u,
                                "往返: 差速限幅/电机钳位必须置位", ctx);
                        expectf(std::fabs(out.yaw_diff_us) <=
                                    std::fabs(y) + 0.5f,
                                "往返: 钳位后差速不放大", ctx);
                    }
                    else
                    {
                        const float yaw_err = std::fabs(out.yaw_diff_us - y);
                        max_yaw_err_us = std::fmax(max_yaw_err_us, yaw_err);
                        expectf(yaw_err < 0.5f, "往返: yaw 误差<0.5µs", ctx);
                        expectf(std::fabs(out.motor1_us - (base_us - y)) < 0.5f &&
                                    std::fabs(out.motor2_us - (base_us + y)) < 0.5f,
                                "往返: 电机 µs 对称", ctx);
                    }

                    // 差速/电机都未限幅时 τ_cmd 必须可达（残差≈0）
                    if (!yaw_limited && !motor_clamp)
                    {
                        expectf(out.torque_residual_nm < 1.0e-6f,
                                "往返: 未饱和残差≈0", ctx);
                    }
                }
            }
        }
    }
    std::printf("往返: %d 组，δ 最大误差 %.6f°，yaw 最大误差 %.6f µs\n", cases,
                static_cast<double>(max_delta_err_deg),
                static_cast<double>(max_yaw_err_us));
}

// ---------------------------------------------------------------------------
// 2) 轴向隔离与符号（α_roll 只动 roll、α_pitch 只动 pitch、α_yaw 只动差速）
// ---------------------------------------------------------------------------
void testAxisIsolation()
{
    const CoaxialAllocatorParams p = defaultParams();
    const float T = 8.0f;
    const float throttle = T / p.max_total_thrust_n * 100.0f;
    const float base_us = motorBaseUs(p, throttle);

    {
        const ForwardCase fc = makeForward(p, T, 3.0f, 0.0f, 0.0f, throttle);
        const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);
        expect(out.valid, "轴隔离: α_roll valid");
        expect(out.servo_roll_deg > 0.0f, "轴隔离: α_roll 符号为正");
        expectf(std::fabs(out.servo_roll_deg - 3.0f) < 0.01f, "轴隔离: α_roll→δ_roll=3°",
                "αx=3°");
        expectf(out.servo_pitch_deg == 0.0f, "轴隔离: α_roll 不串 pitch");
        expectf(out.yaw_diff_us == 0.0f, "轴隔离: α_roll 不串 yaw");
        expectf(out.motor1_us == out.motor2_us && out.motor1_us == base_us,
                "轴隔离: α_roll 不动电机差速");
        expectf(out.sat_mask == kSatNone, "轴隔离: α_roll 单轴不饱和");
    }
    {
        const ForwardCase fc = makeForward(p, T, 0.0f, -4.0f, 0.0f, throttle);
        const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);
        expect(out.valid, "轴隔离: α_pitch valid");
        expect(out.servo_pitch_deg < 0.0f, "轴隔离: α_pitch 符号为负");
        expectf(std::fabs(out.servo_pitch_deg + 4.0f) < 0.01f, "轴隔离: α_pitch→δ_pitch=-4°",
                "αy=-4°");
        expectf(out.servo_roll_deg == 0.0f, "轴隔离: α_pitch 不串 roll");
        expectf(out.yaw_diff_us == 0.0f, "轴隔离: α_pitch 不串 yaw");
        expectf(out.motor1_us == out.motor2_us, "轴隔离: α_pitch 不动电机差速");
    }
    {
        const ForwardCase fc = makeForward(p, T, 0.0f, 0.0f, 60.0f, throttle);
        const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);
        expect(out.valid, "轴隔离: α_yaw valid");
        expectf(out.servo_roll_deg == 0.0f && out.servo_pitch_deg == 0.0f,
                "轴隔离: α_yaw 不动 TVC");
        expectf(std::fabs(out.yaw_diff_us - 60.0f) < 0.5f, "轴隔离: α_yaw→+60µs");
        // 正 α_yaw → 正 yaw_diff → motor1 减小 / motor2 增大（plant Mz=+k·diff）
        expectf(out.motor1_us < base_us && out.motor2_us > base_us,
                "轴隔离: α_yaw 差速方向");
        expectf(std::fabs((out.motor2_us - out.motor1_us) - 120.0f) < 1.0f,
                "轴隔离: 差速幅度 = 2×yaw_diff");
    }
    // 反向符号
    {
        const ForwardCase fc = makeForward(p, T, -3.0f, 4.0f, -60.0f, throttle);
        const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);
        expect(out.servo_roll_deg < 0.0f && out.servo_pitch_deg > 0.0f,
               "轴隔离: 负 roll/正 pitch 符号");
        expect(out.yaw_diff_us < 0.0f, "轴隔离: 负 α_yaw→负差速");
        expect(out.motor1_us > out.motor2_us, "轴隔离: 负差速电机方向");
    }
}

// ---------------------------------------------------------------------------
// 3) 轴序与符号：分配器输出 → plant 正向力矩 必须 = I·α_applied
//    （直接复刻 plant 的 M=[-w.Mz, w.My, -w.Mx]，锁死“轴序一致”）
// ---------------------------------------------------------------------------
void testPlantAxisOrder()
{
    const CoaxialAllocatorParams p = defaultParams();
    const float T = 9.0f;
    const float throttle = T / p.max_total_thrust_n * 100.0f;
    const float alpha[3] = {1.2f, -0.8f, 0.0015f}; // τz 小，避免差速饱和
    CoaxialAllocatorInput in;
    in.alpha_body_radps2[0] = alpha[0];
    in.alpha_body_radps2[1] = alpha[1];
    in.alpha_body_radps2[2] = alpha[2];
    in.throttle_pct = throttle;
    in.thrust_estimate_n = T;
    const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
    expect(out.valid, "轴序: valid");
    expectf(out.sat_mask == kSatNone, "轴序: 混合小指令不饱和");

    float m_body[3];
    plantBodyMoment(p, out, m_body);
    for (int i = 0; i < 3; ++i)
    {
        const float tau_expected = p.inertia_kgm2[i] * out.alpha_applied[i];
        char ctx[160];
        std::snprintf(ctx, sizeof(ctx),
                      "轴 %d: plant M=%.9f N·m，I·α=%.9f N·m", i,
                      static_cast<double>(m_body[i]),
                      static_cast<double>(tau_expected));
        expectf(std::fabs(m_body[i] - tau_expected) < 1.0e-4f,
                "轴序: plant 力矩 = I·α", ctx);
        // 回报的 torque_achieved 应与 plant 力矩一致（同一执行器事实）
        expectf(std::fabs(out.torque_achieved_nm[i] - m_body[i]) < 1.0e-4f,
                "轴序: torque_achieved = plant 力矩", ctx);
        // 指令力矩与 α 同号（正 α→正力矩，FRD 右手系）
        if (std::fabs(alpha[i]) > 1.0e-6f)
        {
            expectf((out.torque_cmd_nm[i] > 0.0f) == (alpha[i] > 0.0f),
                    "轴序: τ_cmd 与 α 同号", ctx);
        }
    }
}

// ---------------------------------------------------------------------------
// 4) 低推力 / 零推力：无 NaN/Inf，饱和位置位，TVC 归零
// ---------------------------------------------------------------------------
void testLowThrust()
{
    const CoaxialAllocatorParams p = defaultParams();
    const float low_T = 0.2f; // < min_thrust_for_tvc_n=0.35

    {
        // T=0.2 N、throttle=0：靠 thrust_estimate 给出真实推力
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[0] = 2.0f;
        in.alpha_body_radps2[1] = -2.0f;
        in.alpha_body_radps2[2] = 0.5f;
        in.throttle_pct = 0.0f;
        in.thrust_estimate_n = low_T;
        in.dt_s = 0.005f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expect(out.valid, "低推力: valid");
        expectf(std::isfinite(out.servo_roll_deg) && std::isfinite(out.servo_pitch_deg) &&
                    std::isfinite(out.yaw_diff_us) && std::isfinite(out.motor1_us) &&
                    std::isfinite(out.motor2_us) && std::isfinite(out.torque_residual_nm),
                "低推力: 输出有限无 NaN/Inf");
        expectf((out.sat_mask & kSatLowThrust) != 0u, "低推力: 置 kSatLowThrust");
        expectf((out.sat_mask & (kSatAlphaRoll | kSatAlphaPitch)) ==
                    (kSatAlphaRoll | kSatAlphaPitch),
                "低推力: α 被权限钳位并置位");
        expectf(out.servo_roll_deg == 0.0f && out.servo_pitch_deg == 0.0f,
                "低推力: TVC 归零");
        // 文档化行为：低推力只关 TVC、不关差速——与 legacy
        // mapCoaxialVtvOutputs 一致（有意保留，风险见 VER-004/CTL-003）。
        // 零油门下差速仍会抬起一只电机，行程钳位后回报减半。
        expectf(out.motor1_us == p.motor_min_us, "低推力: motor1 撞下限并置位");
        expectf((out.sat_mask & kSatMotorClamp) != 0u, "低推力: 电机钳位置位");
        expectf(std::fabs(out.yaw_diff_us - 90.0f) < 0.5f,
                "低推力: 差速仍生效、钳位后回报 90µs（文档化行为）");
        expectf(std::fabs(out.motor2_us - p.motor_min_us - 180.0f) < 0.5f,
                "低推力: motor2 相对 base 抬升 180µs（单侧起转）");
        expectf(out.motor2_us <= p.motor_max_us, "低推力: motor2 不越上限");
        // 残差口径 = 未钳位请求 − 实际可实现：TVC 不可达 ⇒ 残差 = |I·α_req|
        // （α_req 已被 α 权限钳位，但残差以未钳位的原始请求为基准）
        const float expected_res = p.inertia_kgm2[0] * 2.0f;
        expectf(std::fabs(out.torque_residual_nm - expected_res) < 0.05f * expected_res,
                "低推力: 残差 = 不可达 TVC 力矩");
        expectf(residualMatches(out, 1.0e-6f),
                "低推力: residual 可由 requested-achieved 复算(±1e-6)");
    }
    {
        // 零推力：alpha 全 0，不得产生 NaN；低推力位置位
        CoaxialAllocatorInput in;
        in.throttle_pct = 0.0f;
        in.thrust_estimate_n = 0.0f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expect(out.valid, "零推力: valid");
        expectf((out.sat_mask & kSatLowThrust) != 0u, "零推力: 置 kSatLowThrust");
        expectf(std::isfinite(out.torque_residual_nm) && out.torque_residual_nm == 0.0f,
                "零推力: 残差为 0");
        expectf(out.motor1_us == p.motor_min_us && out.motor2_us == p.motor_min_us,
                "零推力: 电机最小 µs");
    }
    {
        // 低推力但 α 在权限内：不置 α 位，只置低推力；TVC 仍归零、残差=指令
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[0] = 0.05f; // < alpha_max≈0.156 rad/s²
        in.throttle_pct = 0.0f;
        in.thrust_estimate_n = 0.3f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expectf((out.sat_mask & kSatAlphaRoll) == 0u, "低推力: α 未越权不置位");
        expectf((out.sat_mask & kSatLowThrust) != 0u, "低推力: 仍置 kSatLowThrust");
        expectf(out.servo_roll_deg == 0.0f, "低推力: TVC 归零");
        expectf(std::fabs(out.torque_residual_nm - 0.05f * p.inertia_kgm2[0]) < 1.0e-6f,
                "低推力: 残差 = 不可达力矩");
    }
}

// ---------------------------------------------------------------------------
// 5) 非有限输入 / 非法参数 → valid=false、输出回零、kSatInvalid
// ---------------------------------------------------------------------------
bool outputsZeroed(const CoaxialAllocatorOutput &out,
                   const CoaxialAllocatorParams &p)
{
    return out.servo_roll_deg == 0.0f && out.servo_pitch_deg == 0.0f &&
           out.yaw_diff_us == 0.0f && out.motor1_us == p.motor_min_us &&
           out.motor2_us == p.motor_min_us && out.thrust_used_n == 0.0f &&
           out.alpha_applied[0] == 0.0f && out.alpha_applied[1] == 0.0f &&
           out.alpha_applied[2] == 0.0f && out.torque_requested_nm[0] == 0.0f &&
           out.torque_requested_nm[1] == 0.0f && out.torque_requested_nm[2] == 0.0f &&
           out.torque_cmd_nm[0] == 0.0f &&
           out.torque_cmd_nm[1] == 0.0f && out.torque_cmd_nm[2] == 0.0f &&
           out.torque_achieved_nm[0] == 0.0f && out.torque_achieved_nm[1] == 0.0f &&
           out.torque_achieved_nm[2] == 0.0f && out.torque_residual_nm == 0.0f;
}

void testNonFinite()
{
    const CoaxialAllocatorParams p = defaultParams();
    CoaxialAllocatorInput base;
    base.alpha_body_radps2[0] = 0.5f;
    base.alpha_body_radps2[1] = -0.5f;
    base.alpha_body_radps2[2] = 0.001f;
    base.throttle_pct = 50.0f;
    base.dt_s = 0.005f;

    const float nan_v = std::nanf("");
    const float inf_v = std::numeric_limits<float>::infinity();

    // 逐字段注入 NaN / ±Inf（alpha[3]、gyro[3]、throttle、thrust_estimate、dt）
    for (int field = 0; field < 9; ++field)
    {
        for (int kind = 0; kind < 2; ++kind)
        {
            CoaxialAllocatorInput in = base;
            const float bad = (kind == 0) ? nan_v : inf_v;
            switch (field)
            {
            case 0: in.alpha_body_radps2[0] = bad; break;
            case 1: in.alpha_body_radps2[1] = bad; break;
            case 2: in.alpha_body_radps2[2] = bad; break;
            case 3: in.gyro_body_radps[0] = bad; break;
            case 4: in.gyro_body_radps[1] = bad; break;
            case 5: in.gyro_body_radps[2] = bad; break;
            case 6: in.throttle_pct = bad; break;
            case 7: in.thrust_estimate_n = bad; break;
            case 8: in.dt_s = bad; break;
            default: break;
            }
            const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
            char ctx[96];
            std::snprintf(ctx, sizeof(ctx), "字段%d %s", field,
                          (kind == 0) ? "NaN" : "+Inf");
            expectf(!out.valid, "非有限: valid=false", ctx);
            expectf(out.sat_mask == kSatInvalid, "非有限: sat=kSatInvalid", ctx);
            expectf(outputsZeroed(out, p), "非有限: 输出回零", ctx);
        }
    }
    // 负无穷与 dt<=0
    {
        CoaxialAllocatorInput in = base;
        in.alpha_body_radps2[0] = -inf_v;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expect(!out.valid && out.sat_mask == kSatInvalid);
    }
    {
        CoaxialAllocatorInput in = base;
        in.dt_s = 0.0f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expectf(!out.valid && out.sat_mask == kSatInvalid, "非有限: dt<=0 拒绝");
        expect(outputsZeroed(out, p));
    }
    // 非法参数：力臂 0 / 惯量 0 / 推力上限 0 / 电机量程倒置
    for (int bad_case = 0; bad_case < 4; ++bad_case)
    {
        CoaxialAllocatorParams bad = p;
        switch (bad_case)
        {
        case 0: bad.lever_m = 0.0f; break;
        case 1: bad.inertia_kgm2[2] = 0.0f; break;
        case 2: bad.max_total_thrust_n = 0.0f; break;
        case 3: bad.motor_max_us = bad.motor_min_us; break;
        default: break;
        }
        const CoaxialAllocatorOutput out = allocateCoaxial(base, bad);
        char ctx[64];
        std::snprintf(ctx, sizeof(ctx), "参数用例%d", bad_case);
        expectf(!out.valid && out.sat_mask == kSatInvalid, "非法参数: 拒绝", ctx);
        expectf(outputsZeroed(out, bad), "非法参数: 输出回零", ctx);
    }
}

// ---------------------------------------------------------------------------
// 6) 残差语义：饱和时 residual>0 且 = max|τ_cmd-τ_ach|；α 权限钳位不计残差
// ---------------------------------------------------------------------------
void testResidual()
{
    const CoaxialAllocatorParams p = defaultParams();

    // 6a) 电机行程钳位：高油门 + 正差速 → motor2 撞 2000µs
    //     请求 150µs（在 α 权限内，避免被 α 钳位遮蔽），base=2000 只能给到 75µs
    {
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[2] = 150.0e-6f / p.inertia_kgm2[2];
        in.throttle_pct = 100.0f;
        in.thrust_estimate_n = p.max_total_thrust_n;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expect(out.valid, "残差: 满油门差速限幅 valid");
        expectf((out.sat_mask & kSatAlphaYaw) == 0u, "残差: 150µs 未越 α 权限");
        expectf((out.sat_mask & kSatYawDiff) != 0u, "残差: 置 kSatYawDiff");
        expectf(out.motor2_us == p.motor_max_us, "残差: motor2 顶到上限");
        expectf(std::fabs(out.yaw_diff_us) < 0.5f, "残差: 满油门实际差速 = 0µs");
        expectf(out.torque_residual_nm > 0.0f, "残差: 饱和时 residual>0");
        expectf(residualMatches(out, 0.05f * out.torque_residual_nm),
                "残差: 与不可达力矩一致(±5%)");
        // 外部核算：请求 150µs、实际 0µs → 残差 = k_yaw·150µs
        const float expected = p.yaw_moment_nm_per_us * 150.0f;
        expectf(std::fabs(out.torque_residual_nm - expected) < 0.05f * expected,
                "残差: 差值 = k_yaw·Δyaw");
    }
    // 6b) 低油门 + 负差速 → motor2 撞 1000µs
    {
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[2] = -60.0e-6f / p.inertia_kgm2[2];
        in.throttle_pct = 0.0f;
        in.thrust_estimate_n = 0.0f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expectf((out.sat_mask & kSatYawDiff) != 0u, "残差: 零油门差速限幅置位");
        expectf(out.motor2_us == p.motor_min_us, "残差: motor2 顶到下限");
        expectf(std::fabs(out.yaw_diff_us) < 0.5f, "残差: 零油门实际差速 = 0µs");
        expectf(out.torque_residual_nm > 0.0f, "残差: 低油门 residual>0");
        expectf(residualMatches(out, 0.05f * out.torque_residual_nm),
                "残差: 低油门与不可达力矩一致");
        expectf(std::fabs(out.torque_residual_nm - p.yaw_moment_nm_per_us * 60.0f) <
                    0.05f * p.yaw_moment_nm_per_us * 60.0f,
                "残差: 零油门残差 = k_yaw·60µs");
    }
    // 6c) α 权限钳位（未撞电机行程）：残差以"未钳位请求"为基准，
    //     故 α 被钳位时 residual = |I·(α_req − α_applied)| > 0；
    //     但 torque_cmd_nm 存的是钳位后 τ，二者之差无法从输出复算（可观测性缺口）
    {
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[0] = 100.0f;
        in.throttle_pct = 60.0f;
        in.thrust_estimate_n = 8.4f;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expectf((out.sat_mask & kSatAlphaRoll) != 0u, "残差: α 越权置位");
        expectf(std::fabs(out.servo_roll_deg - p.max_correction_deg) < 1.0e-3f,
                "残差: α 钳位后 δ=max_correction");
        const float alpha_max_roll =
            p.lever_m * 8.4f * std::sin(p.max_correction_deg * kDeg2Rad) /
            p.inertia_kgm2[0];
        const float expected_res = p.inertia_kgm2[0] * (100.0f - alpha_max_roll);
        expectf(std::fabs(out.torque_residual_nm - expected_res) <
                    0.01f * expected_res,
                "残差: α 钳位 residual = I·Δα");
        // 新契约：未钳位请求单独输出，residual 可由 requested/achieved 直接复算
        expectf(std::fabs(out.torque_requested_nm[0] - 5.0f) < 1.0e-4f,
                "残差: requested 记录未钳位 τ_req");
        expectf(residualMatches(out, 1.0e-6f),
                "残差: residual 可由 requested-achieved 复算(±1e-6)");
        // cmd 仍是钳位后 τ；若用 cmd-achieved 复算会漏掉 α 权限缺口，必须不等于 residual
        float cmd_res = 0.0f;
        for (int i = 0; i < 3; ++i)
        {
            cmd_res = std::fmax(cmd_res, std::fabs(out.torque_cmd_nm[i] -
                                                   out.torque_achieved_nm[i]));
        }
        expectf(std::fabs(out.torque_residual_nm - cmd_res) > 1.0f,
                "残差: cmd-achieved 差分 ≠ residual（cmd 为钳位后）");
        expectf(std::fabs(out.torque_cmd_nm[0] - out.torque_achieved_nm[0]) < 1.0e-5f,
                "残差: cmd 与 achieved 在 α 钳位时相等");
    }
    // 6d) 偏航 α 权限 = 差速行程权限：α 钳位时同步置 kSatYawDiff，
    //     保证偏航饱和位可达（不再被 α 钳位遮蔽）
    {
        CoaxialAllocatorInput in;
        in.alpha_body_radps2[2] = 200.0e-6f / p.inertia_kgm2[2]; // 请求 200µs > 180µs
        in.throttle_pct = 100.0f;
        in.thrust_estimate_n = p.max_total_thrust_n;
        const CoaxialAllocatorOutput out = allocateCoaxial(in, p);
        expectf((out.sat_mask & kSatAlphaYaw) != 0u,
                "残差: 请求 200µs 触发 α 权限钳位");
        expectf((out.sat_mask & kSatYawDiff) != 0u,
                "残差: α 钳位同时置 kSatYawDiff");
        expectf((out.sat_mask & (kSatAlphaYaw | kSatYawDiff)) ==
                    (kSatAlphaYaw | kSatYawDiff),
                "残差: bit2 与 bit5 并列置位");
        expectf(std::fabs(out.alpha_applied[2] * p.inertia_kgm2[2] /
                              p.yaw_moment_nm_per_us - p.max_yaw_diff_us) < 0.1f,
                "残差: 偏航 α 权限 = max_yaw_diff 对应的 τ");
    }
}

// ---------------------------------------------------------------------------
// 7) 名义点（T=m·g）α 反解 == legacy 校正角/µs（锁“名义点不改手感”）
// ---------------------------------------------------------------------------
void testLegacyNominal()
{
    const CoaxialAllocatorParams p = defaultParams();
    const float T_nom = p.mass_kg * kG;
    const float throttle = T_nom / p.max_total_thrust_n * 100.0f;
    const float base_us = motorBaseUs(p, throttle);
    const float drs[] = {-10.0f, -5.0f, -0.1f, 0.0f, 3.0f, 10.0f};
    const float dps[] = {-10.0f, -5.0f, -0.1f, 0.0f, 3.0f, 10.0f};
    const float ys[] = {-180.0f, -60.0f, 0.0f, 90.0f, 180.0f};

    float max_delta_err = 0.0f;
    float max_yaw_err = 0.0f;
    for (const float dr : drs)
    {
        for (const float dp : dps)
        {
            for (const float y : ys)
            {
                const ForwardCase fc = makeForward(p, T_nom, dr, dp, y, throttle);
                const CoaxialAllocatorOutput out = allocateCoaxial(fc.in, p);
                char ctx[192];
                std::snprintf(ctx, sizeof(ctx),
                              "T=mg=%.4fN δr=%.2f° δp=%.2f° yaw=%.1fµs",
                              static_cast<double>(T_nom), dr, dp, y);
                expectf(out.valid, "名义点: valid", ctx);
                const float delta_err =
                    std::fmax(std::fabs(out.servo_roll_deg - dr),
                              std::fabs(out.servo_pitch_deg - dp));
                max_delta_err = std::fmax(max_delta_err, delta_err);
                expectf(delta_err < 0.01f, "名义点: δ 还原 <0.01°", ctx);
                const float yaw_err = std::fabs(out.yaw_diff_us - y);
                max_yaw_err = std::fmax(max_yaw_err, yaw_err);
                expectf(yaw_err < 0.1f, "名义点: yaw 还原 <0.1µs", ctx);
                expectf(std::fabs(out.motor1_us - (base_us - y)) < 0.1f &&
                            std::fabs(out.motor2_us - (base_us + y)) < 0.1f,
                        "名义点: 电机 µs 与 legacy 一致", ctx);
            }
        }
    }
    std::printf("名义点: δ 最大误差 %.6f°，yaw 最大误差 %.6f µs（T=mg=%.4f N）\n",
                static_cast<double>(max_delta_err),
                static_cast<double>(max_yaw_err), static_cast<double>(T_nom));
}

// ---------------------------------------------------------------------------
// 8) 契约结构自检：饱和位唯一、单位/默认值、状态字段完整性
// ---------------------------------------------------------------------------
void testContractShape()
{
    // 饱和位不得互相重叠（每个 bit 唯一），且 kSatNone=0
    const uint16_t bits[] = {kSatAlphaRoll,  kSatAlphaPitch, kSatAlphaYaw,
                             kSatTvcRoll,    kSatTvcPitch,   kSatYawDiff,
                             kSatLowThrust,  kSatMotorClamp, kSatInvalid,
                             kSatRateCoupling};
    expect(kSatNone == 0u);
    for (int i = 0; i < 10; ++i)
    {
        expect(bits[i] != 0u && (bits[i] & (bits[i] - 1u)) == 0u); // 单 bit
        for (int j = i + 1; j < 10; ++j)
        {
            expect((bits[i] & bits[j]) == 0u); // 不重叠
        }
    }
    // 默认参数与 SIL plant L1 集总参数一致（改参数需同步 plant 与验收）
    const CoaxialAllocatorParams p = defaultParams();
    expect(std::fabs(p.lever_m - 0.15f) < 1e-9f);
    expect(std::fabs(p.inertia_kgm2[0] - 0.05f) < 1e-9f);
    expect(std::fabs(p.inertia_kgm2[1] - 0.05f) < 1e-9f);
    expect(std::fabs(p.inertia_kgm2[2] - 0.08f) < 1e-9f);
    expect(std::fabs(p.yaw_moment_nm_per_us - 1.0e-6f) < 1e-12f);
    expect(p.max_correction_deg <= p.servo_half_angle_deg);
    expect(coaxialAllocParamsValid(p));
    // 契约默认值：单位/语义（α 为 FRD rad/s²，默认零指令）
    const AlphaCommand ac;
    expect(ac.alpha_body_radps2.x == 0.0f && ac.alpha_body_radps2.y == 0.0f &&
           ac.alpha_body_radps2.z == 0.0f && !ac.valid);
    const AllocatorStatus st;
    expect(st.torque_residual_nm == 0.0f && st.sat_mask == kSatNone && !st.valid);
}

} // namespace

int main()
{
    testRoundTrip();
    testAxisIsolation();
    testPlantAxisOrder();
    testLowThrust();
    testNonFinite();
    testResidual();
    testLegacyNominal();
    testContractShape();

    std::printf("=== allocator contracts: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
