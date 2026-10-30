/**
 * @file    test_liquid_rocket_allocator.cpp
 * @module  宿主机回归（液体火箭分配器试点）
 * @brief   独立复核 include/LiquidRocketAllocator.h：
 *          名义 α 往返、摆座饱和、RCS 滚转权限与饱和、零推力边界、
 *          非有限输入回零；正向模型与分配器同轴序（FRD）。
 * @details 正向模型只复刻分配器文件头声明的 pilot 约定：
 *          My=L·T·sinδy、Mz=L·T·sinδx、Mx=k_rcs·(left−right)/100；
 *          T=throttle×Tmax。该模型不替代实机台架/喷管安装极性核对。
 * @note    本测试不修改实现；RCS/摆座符号若与实机相反，应先在文档登记，
 *          再改 pilot 约定，而不是让测试去迁就实现。
 * @see     docs/05-控制与分配/CTL-006、include/LiquidRocketAllocator.h
 */
#include <cmath>
#include <cstdio>

#include "ControlContracts.h"
#include "LiquidRocketAllocator.h"

namespace
{
using namespace bfs;
using namespace bfs::control;

int g_pass = 0;
int g_fail = 0;

constexpr float kDeg2Rad = 0.017453292519943295f;

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

/** @brief 无上下文细节的便捷重载 */
void expectf(const bool ok, const char *name)
{
    expect(ok, name);
}

LiquidRocketAllocatorInput makeInput(const float ax, const float ay,
                                     const float az, const float throttle)
{
    LiquidRocketAllocatorInput in;
    in.alpha_body_radps2[0] = ax;
    in.alpha_body_radps2[1] = ay;
    in.alpha_body_radps2[2] = az;
    in.throttle_pct = throttle;
    in.dt_s = 0.005f;
    return in;
}

/** @brief 由分配器输出反算 body 三轴力矩（pilot 正向模型） */
void forwardMoment(const LiquidRocketAllocatorParams &p,
                   const LiquidRocketAllocatorOutput &out, float m_body[3])
{
    const float thrust = out.thrust_used_n;
    m_body[0] = p.rcs_moment_nm * (out.rcs_left_pct - out.rcs_right_pct) / 100.0f;
    m_body[1] = p.lever_m * thrust * std::sin(out.nozzle_y_deg * kDeg2Rad);
    m_body[2] = p.lever_m * thrust * std::sin(out.nozzle_x_deg * kDeg2Rad);
}

bool finite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

void testNominalRoundTrip()
{
    LiquidRocketAllocatorParams p;
    const LiquidRocketAllocatorInput in = makeInput(2.0f, 2.0f, 2.0f, 60.0f);
    const LiquidRocketAllocatorOutput out = allocateLiquidRocket(in, p);
    float m[3] = {0.0f, 0.0f, 0.0f};
    forwardMoment(p, out, m);

    expectf(out.valid, "名义：valid");
    expectf(out.sat_mask == kSatNone, "名义：不饱和");
    expectf(std::fabs(m[0] - out.torque_requested_nm[0]) < 5e-3f,
            "名义：roll 力矩往返 <5e-3 N·m");
    expectf(std::fabs(m[1] - out.torque_requested_nm[1]) < 5e-3f,
            "名义：pitch 力矩往返 <5e-3 N·m");
    expectf(std::fabs(m[2] - out.torque_requested_nm[2]) < 5e-3f,
            "名义：yaw 力矩往返 <5e-3 N·m");
    expectf(out.torque_residual_nm < 5e-3f, "名义：残差 <5e-3 N·m");
    expectf(finite3(m), "名义：力矩有限");
}

void testGimbalSaturation()
{
    LiquidRocketAllocatorParams p;
    const LiquidRocketAllocatorInput in = makeInput(0.0f, 100.0f, 0.0f, 60.0f);
    const LiquidRocketAllocatorOutput out = allocateLiquidRocket(in, p);

    expectf((out.sat_mask & kSatAlphaPitch) != 0,
            "摆座饱和：α_pitch 钳位");
    expectf((out.sat_mask & kSatGimbalLimit) != 0,
            "摆座饱和：置 kSatGimbalLimit");
    expectf(std::fabs(std::fabs(out.nozzle_y_deg) - p.max_gimbal_deg) < 1e-3f,
            "摆座饱和：摆角钳到 max_gimbal_deg");
    expectf(out.torque_residual_nm > 0.0f, "摆座饱和：残差 >0");
}

void testRcsSaturation()
{
    LiquidRocketAllocatorParams p;
    const LiquidRocketAllocatorInput in = makeInput(10.0f, 0.0f, 0.0f, 60.0f);
    const LiquidRocketAllocatorOutput out = allocateLiquidRocket(in, p);

    expectf((out.sat_mask & kSatAlphaRoll) != 0, "RCS 饱和：α_roll 钳位");
    expectf((out.sat_mask & kSatRcsLimit) != 0, "RCS 饱和：置 kSatRcsLimit");
    expectf(std::fabs(out.rcs_left_pct - 100.0f) < 1e-3f,
            "RCS 饱和：正滚转→左通道 100%");
    expectf(std::fabs(out.rcs_right_pct) < 1e-3f,
            "RCS 饱和：右通道 0%");
    expectf(std::fabs(out.torque_achieved_nm[0] - p.rcs_moment_nm) < 1e-3f,
            "RCS 饱和：实际滚转力矩 = 单侧满量");
}

void testLowThrust()
{
    LiquidRocketAllocatorParams p;
    const LiquidRocketAllocatorInput in = makeInput(0.5f, 0.5f, 0.5f, 0.0f);
    const LiquidRocketAllocatorOutput out = allocateLiquidRocket(in, p);

    expectf(out.valid, "零推力：valid（只是无摆座权限）");
    expectf((out.sat_mask & kSatLowThrust) != 0, "零推力：置 kSatLowThrust");
    expectf((out.sat_mask & kSatAlphaPitch) != 0,
            "零推力：pitch/yaw α 权限为 0");
    expectf(std::fabs(out.nozzle_x_deg) < 1e-6f &&
                std::fabs(out.nozzle_y_deg) < 1e-6f,
            "零推力：摆角归中");
    // 滚转权限由 RCS 提供，与推力无关：0.5 rad/s² 在 3.33 权限内
    expectf(std::fabs(out.rcs_left_pct - 15.0f) < 1e-3f,
            "零推力：滚转仍由 RCS 承担（15% 左喷）");
}

void testInvalid()
{
    LiquidRocketAllocatorParams p;
    LiquidRocketAllocatorInput in = makeInput(0.0f, 0.0f, 0.0f, 60.0f);
    in.alpha_body_radps2[0] = std::nanf("");
    const std::uint16_t zero_mask = kSatInvalid;
    const LiquidRocketAllocatorOutput out = allocateLiquidRocket(in, p);

    expectf(!out.valid, "非有限输入：valid=false");
    expectf(out.sat_mask == zero_mask, "非有限输入：sat_mask=kSatInvalid");
    expectf(out.nozzle_x_deg == 0.0f && out.nozzle_y_deg == 0.0f &&
                out.rcs_left_pct == 0.0f && out.rcs_right_pct == 0.0f,
            "非有限输入：输出回零");
}

} // namespace

int main()
{
    std::printf("=== 液体火箭分配器试点 ===\n");
    testNominalRoundTrip();
    testGimbalSaturation();
    testRcsSaturation();
    testLowThrust();
    testInvalid();
    std::printf("--- %d passed, %d failed ---\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
