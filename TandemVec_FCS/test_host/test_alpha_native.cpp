/**
 * @file    test_alpha_native.cpp
 * @module  宿主机回归（共轴控制器 / α-native 与 legacy δ 环等价性）
 * @brief   在固定推力/惯量的名义点，把同一输入序列同时喂给
 *          legacy 控制器（update()，输出修正角 δ）与 α-native 控制器
 *          （updateAlpha()，输出机体系 α），用 g=(l·T/I)·π/180 换算后
 *          逐拍比较，验证两条路径的闭环指令严格等价：
 *          ①ATTITUDE 外环 + 速率 PI（积分开），200 拍；
 *          ②大误差饱和：α 钳位到 (l·T/I)·sin(δmax)，与 legacy δ 钳位一致；
 *          ③RATE 模式：摇杆目标角速度直通，内环积分保持；
 *          ④偏航通道：α_yaw = (k_yaw/I_z)·diff_us；
 *          ⑤手动/未激活：α-native 退回 legacy δ 路径，alpha_valid=false；
 *          ⑥非法输入：两路径均回零且不输出 α；
 *          ⑦alpha_native=false 时 updateAlpha 与 update 逐字段相同（兼容路径）。
 * @details 只在名义点验证；推力调度/惯量偏差下的响应差异由 SIL A/B 量化
 *          （run_coaxial_ab.py --alloc-mode 2）。
 * @note    本测试不修改实现，也不改变任何增益数字。
 * @see     docs/05-控制与分配/CTL-006 §2.5、docs/07-验证与确认/VER-004 §11
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "CoaxialVtvControl.h"

namespace
{
using namespace bfs;

int g_pass = 0;
int g_fail = 0;

constexpr float kDt = 0.005f;
constexpr float kDeg2Rad = 0.017453292519943295f;
constexpr float kLever = 0.15f;
constexpr float kThrust = 8.4f;
constexpr float kIx = 0.05f;
constexpr float kIy = 0.05f;
constexpr float kIz = 0.08f;
constexpr float kYawNmPerUs = 1.0e-6f;

const float kGRoll = kLever * kThrust / kIx;               // (rad/s^2)/rad
const float kGPitch = kLever * kThrust / kIy;
const float kGDegRoll = kGRoll * kDeg2Rad;                 // (rad/s^2)/deg
const float kGDegPitch = kGPitch * kDeg2Rad;
const float kGYaw = kYawNmPerUs / kIz;                     // (rad/s^2)/us
const float kAlphaLimit = kGRoll * std::sin(10.0f * kDeg2Rad);

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

void expectClose(const float a, const float b, const float abs_tol,
                 const float rel_tol, const char *name)
{
    const float scale = (std::fabs(b) > std::fabs(a)) ? std::fabs(b) : std::fabs(a);
    const float tol = abs_tol + rel_tol * scale;
    if (std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= tol)
    {
        ++g_pass;
        return;
    }
    ++g_fail;
    std::printf("[FAIL] %s | native=%.9f legacy=%.9f tol=%.9f\n", name, a, b,
                tol);
}

CoaxialVtvParams makeParams()
{
    CoaxialVtvParams p;
    p.att_kp_roll = 4.25f;
    p.att_kp_pitch = 4.25f;
    p.rate_kp_roll = 0.25f;
    p.rate_kp_pitch = 0.25f;
    p.rate_ki_roll = 0.08f;
    p.rate_ki_pitch = 0.08f;
    p.rate_kp_yaw = 3.8f;
    p.rate_ki_yaw = 30.0f;
    p.max_target_rate_dps = 80.0f;
    p.max_correction_deg = 10.0f;
    p.max_yaw_diff_us = 180.0f;
    p.att_integral_limit = 40.0f;
    p.rate_integral_limit = 5.0f;
    p.yaw_integral_limit = 50.0f;
    p.yaw_integral_threshold = 40.0f;
    p.att_rate_filter_alpha = 0.85f;
    p.correction_filter_alpha = 0.25f;
    p.pid_derivative_filter_alpha = 0.2f;
    return p;
}

CoaxialVtvInput baseInput(const bool rate_mode)
{
    CoaxialVtvInput in;
    in.attitude_active = true;
    in.attitude_integral_active = !rate_mode;
    in.rate_integral_active = true;
    in.yaw_active = true;
    in.yaw_integral_active = true;
    in.rate_mode = rate_mode;
    in.dt_s = kDt;
    in.alpha_native = false;
    return in;
}

void enableNative(CoaxialVtvInput &in)
{
    in.alpha_native = true;
    in.alpha_per_deg_roll = kGDegRoll;
    in.alpha_per_deg_pitch = kGDegPitch;
    in.alpha_per_us_yaw = kGYaw;
    in.alpha_limit_roll = kAlphaLimit;
    in.alpha_limit_pitch = kGRoll * std::sin(10.0f * kDeg2Rad);
    in.alpha_limit_yaw = kGYaw * 180.0f;
}

struct Pair
{
    CoaxialVtvController legacy;
    CoaxialVtvController native;
    CoaxialVtvInput in;
};

void initPair(Pair &p, const bool rate_mode)
{
    p.legacy = CoaxialVtvController();
    p.native = CoaxialVtvController();
    p.legacy.setParams(makeParams());
    p.native.setParams(makeParams());
    p.in = baseInput(rate_mode);
}

/** 逐拍比较：native α 与 g·legacy δ 的差（rel_tol 允许饱和点 sin 与线性差异）。 */
void compareTick(Pair &p, const char *tag, const int i, const float rel_tol,
                 const float abs_tol)
{
    CoaxialVtvInput in_legacy = p.in;
    CoaxialVtvInput in_native = p.in;
    enableNative(in_native);
    const CoaxialVtvOutput out_l = p.legacy.update(in_legacy);
    const CoaxialVtvOutput out_n = p.native.updateAlpha(in_native);

    if (!out_l.valid || !out_n.alpha_valid)
    {
        expect(false, tag);
        return;
    }
    char name[160];
    std::snprintf(name, sizeof(name), "%s tick=%d roll", tag, i);
    expectClose(out_n.alpha_roll_radps2, kGDegRoll * out_l.roll_correction_deg,
                abs_tol, rel_tol, name);
    std::snprintf(name, sizeof(name), "%s tick=%d pitch", tag, i);
    expectClose(out_n.alpha_pitch_radps2,
                kGDegPitch * out_l.pitch_correction_deg, abs_tol, rel_tol,
                name);
    std::snprintf(name, sizeof(name), "%s tick=%d yaw", tag, i);
    expectClose(out_n.alpha_yaw_radps2, kGYaw * out_l.yaw_diff_us, abs_tol,
                rel_tol, name);
}

void testAttitudeTracking()
{
    Pair p;
    initPair(p, false);
    p.in.error_roll_deg = 3.0f;
    p.in.error_pitch_deg = -2.0f;
    p.in.rate_roll_dps = 0.0f;
    p.in.rate_pitch_dps = 0.0f;
    p.in.yaw_rate_target_dps = 20.0f;
    p.in.rate_yaw_dps = 0.0f;
    for (int i = 0; i < 200; ++i)
    {
        compareTick(p, "att-track", i, 2.0e-4f, 2.0e-5f);
    }
}

void testAttitudeSaturation()
{
    Pair p;
    initPair(p, false);
    p.in.error_roll_deg = 15.0f; // 外环输出 e = 4.25*15 = 63.75 dps -> 速率 PI
    p.in.error_pitch_deg = -15.0f;
    p.in.rate_roll_dps = 0.0f;
    p.in.rate_pitch_dps = 0.0f;
    p.in.yaw_rate_target_dps = 0.0f;
    // 第一拍 δ 尚未饱和（滤波 0.25），跑 40 拍让 α 逼近上限
    for (int i = 0; i < 40; ++i)
    {
        compareTick(p, "att-sat", i, 6.0e-3f, 2.0e-5f);
    }
    CoaxialVtvInput in = p.in;
    enableNative(in);
    CoaxialVtvOutput out = p.native.updateAlpha(in);
    expect(out.alpha_valid, "att-sat alpha_valid");
    expect(std::fabs(out.alpha_roll_radps2) <= kAlphaLimit + 1.0e-5f,
           "att-sat alpha clamped");
    expect(out.saturated, "att-sat saturated");
}

void testRateMode()
{
    Pair p;
    initPair(p, true);
    p.in.roll_rate_target_dps = 25.0f;
    p.in.pitch_rate_target_dps = -25.0f;
    p.in.rate_roll_dps = 0.0f;
    p.in.rate_pitch_dps = 0.0f;
    p.in.yaw_rate_target_dps = -30.0f;
    p.in.rate_yaw_dps = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        compareTick(p, "rate", i, 2.0e-4f, 2.0e-5f);
    }
    // RATE 下速率环积分必须保持（旧架构行为）：恒定误差下 α 指令跨拍增长。
    CoaxialVtvInput in = p.in;
    enableNative(in);
    const CoaxialVtvOutput first = p.native.updateAlpha(in);
    for (int i = 0; i < 40; ++i)
    {
        p.native.updateAlpha(in);
    }
    const CoaxialVtvOutput later = p.native.updateAlpha(in);
    expect(std::fabs(later.alpha_roll_radps2) >
               std::fabs(first.alpha_roll_radps2),
           "rate integral persists");
}

void testYawOnly()
{
    Pair p;
    initPair(p, false);
    p.in.error_roll_deg = 0.0f;
    p.in.error_pitch_deg = 0.0f;
    p.in.rate_roll_dps = 0.0f;
    p.in.rate_pitch_dps = 0.0f;
    p.in.yaw_rate_target_dps = 60.0f;
    p.in.rate_yaw_dps = 0.0f;
    for (int i = 0; i < 100; ++i)
    {
        compareTick(p, "yaw", i, 2.0e-4f, 2.0e-5f);
    }
}

void testManualFallback()
{
    Pair p;
    initPair(p, false);
    p.in.attitude_active = false;
    p.in.manual_tvc = true;
    p.in.manual_roll_deg = 4.0f;
    p.in.manual_pitch_deg = -3.0f;
    enableNative(p.in);
    const CoaxialVtvOutput out = p.native.updateAlpha(p.in);
    expect(!out.alpha_valid, "manual alpha_valid=false");
    expectClose(out.roll_correction_deg, 1.0f, 1.0e-6f, 0.0f,
               "manual first-tick roll filter");
    expectClose(out.pitch_correction_deg, -0.75f, 1.0e-6f, 0.0f,
               "manual first-tick pitch filter");
    CoaxialVtvOutput settled;
    for (int i = 0; i < 40; ++i)
    {
        settled = p.native.updateAlpha(p.in);
    }
    expectClose(settled.roll_correction_deg, 4.0f, 1.0e-4f, 0.0f,
                "manual settled roll");
    expectClose(settled.pitch_correction_deg, -3.0f, 1.0e-4f, 0.0f,
                "manual settled pitch");
}

void testInvalidInput()
{
    Pair p;
    initPair(p, false);
    p.in.error_roll_deg = std::nanf("");
    enableNative(p.in);
    const CoaxialVtvOutput out = p.native.updateAlpha(p.in);
    expect(!out.alpha_valid, "nan alpha_valid=false");
    expect(out.alpha_roll_radps2 == 0.0f && out.alpha_pitch_radps2 == 0.0f &&
               out.alpha_yaw_radps2 == 0.0f,
           "nan alpha zeros");
    expect(!out.valid, "nan valid=false");
}

void testLegacyCompatibility()
{
    Pair p;
    initPair(p, false);
    p.in.error_roll_deg = 3.0f;
    p.in.error_pitch_deg = -2.0f;
    p.in.yaw_rate_target_dps = 20.0f;
    for (int i = 0; i < 30; ++i)
    {
        CoaxialVtvInput in = p.in; // alpha_native=false
        const CoaxialVtvOutput a = p.legacy.update(in);
        const CoaxialVtvOutput b = p.native.updateAlpha(in);
        expect(a.roll_correction_deg == b.roll_correction_deg &&
                   a.pitch_correction_deg == b.pitch_correction_deg &&
                   a.yaw_diff_us == b.yaw_diff_us &&
                   a.valid == b.valid && a.saturated == b.saturated,
               "alpha_native=false bit-identical");
        expect(!b.alpha_valid, "alpha_native=false alpha_valid=false");
    }
}

} // namespace

int main()
{
    testAttitudeTracking();
    testAttitudeSaturation();
    testRateMode();
    testYawOnly();
    testManualFallback();
    testInvalidInput();
    testLegacyCompatibility();
    std::printf("[test_alpha_native] passed=%d failed=%d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
