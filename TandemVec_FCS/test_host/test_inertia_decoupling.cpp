// 宿主机回归测试：惯量逆解交叉耦合前馈（InertiaDecoupling.h）
// 编译运行：
//   g++ -std=c++17 -Iinclude test_host/test_inertia_decoupling.cpp \
//       -o test_host/bin/id && ./test_host/bin/id
//
// 覆盖范围：
//   1. ω×(I·ω) 陀螺耦合逐项数值展开（对照仿真 dynamics.mjs gx/gy/gz）
//   2. ω×h 转子角动量陀螺项展开
//   3. 使能掩码按位开关（mask=0 → 全零）
//   4. 理想刚体闭环对照：前馈开 → 跟踪误差 ~0；前馈关 → 误差 = 耦合项/I
//   5. 本机细长杆特性：Iy=Iz → 绕 x 轴陀螺耦合项恒 0
#include "../include/InertiaDecoupling.h"

#include <cmath>
#include <cstdio>
#include <string>

// ============================================================
//  测试框架（与现有宿主机测试风格一致）
// ============================================================
static int g_fail_count = 0;

static void check(bool cond, const std::string& name)
{
    std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
    if (!cond) ++g_fail_count;
}

static bool approx(float a, float b, float rel_tol = 1e-4f, float abs_tol = 1e-9f)
{
    float diff = std::fabs(a - b);
    float scale = std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
    return diff <= rel_tol * scale + abs_tol;
}

// 本机惯量（TandemVec_Config.h 同源）
static constexpr float kIx = 0.0021f;  // 轴向（绕推力轴/纵轴）
static constexpr float kIy = 0.022f;   // 横向
static constexpr float kIz = 0.022f;   // 横向（与 Iy 对称）

// ============================================================
//  测试 1：陀螺耦合展开（对照仿真 dynamics.mjs gx/gy/gz）
//   仿真：gx=(Iz−Iy)·wy·wz；gy=(Ix−Iz)·wz·wx；gz=(Iy−Ix)·wx·wy
//   ω=(1,2,3) rad/s：
//     gx = (0.022−0.022)·2·3        = 0
//     gy = (0.0021−0.022)·3·1       = −0.0597
//     gz = (0.022−0.0021)·1·2       = 0.0398
// ============================================================
static void test_gyro_coupling()
{
    const float omega[3] = {1.0f, 2.0f, 3.0f};
    const float h_zero[3] = {0.0f, 0.0f, 0.0f};
    const auto r = computeInertiaCompensation(omega, kIx, kIy, kIz, h_zero,
                                              kInertiaCompGyro);
    check(approx(r.Mx_ff, 0.0f), "陀螺耦合 Mx=0（Iy=Iz 细长杆）");
    check(approx(r.My_ff, (kIx - kIz) * 3.0f * 1.0f), "陀螺耦合 My=(Ix−Iz)·wz·wx");
    check(approx(r.Mz_ff, (kIy - kIx) * 1.0f * 2.0f), "陀螺耦合 Mz=(Iy−Ix)·wx·wy");
    check(r.Mx_ff == 0.0f, "Iy=Iz → 绕 x 轴陀螺耦合恒 0（对称杆）");
}

// ============================================================
//  测试 2：转子角动量陀螺项（ω×h，仿真同构）
//   h=(0.1,0,0)、ω=(0,1,0)：
//     ω×h = (ωy·hz − ωz·hy, ωz·hx − ωx·hz, ωx·hy − ωy·hx)
//         = (0, 0, −0.1)
// ============================================================
static void test_rotor_coupling()
{
    const float omega[3] = {0.0f, 1.0f, 0.0f};
    const float h[3] = {0.1f, 0.0f, 0.0f};
    const auto r = computeInertiaCompensation(omega, kIx, kIy, kIz, h,
                                              kInertiaCompRotor);
    check(approx(r.Mx_ff, 0.0f), "转子陀螺 Mx=0");
    check(approx(r.My_ff, 0.0f), "转子陀螺 My=0");
    check(approx(r.Mz_ff, -0.1f), "转子陀螺 Mz = ωx·hy − ωy·hx = −0.1（ω×h 符号）");

    // 只开转子项时陀螺耦合不生效（ω=(1,2,3)：Mz = ωx·hy − ωy·hx = −0.2）
    const float omega2[3] = {1.0f, 2.0f, 3.0f};
    const auto r2 = computeInertiaCompensation(omega2, kIx, kIy, kIz, h,
                                               kInertiaCompRotor);
    check(r2.Mx_ff == 0.0f && approx(r2.My_ff, 0.3f) && approx(r2.Mz_ff, -0.2f),
          "只开 bit1：ω×h 全分量（My=ωz·hx=0.3, Mz=−ωy·hx=−0.2）");
}

// ============================================================
//  测试 3：使能掩码（mask=0 → 全零；全开 = 两项之和）
// ============================================================
static void test_mask()
{
    const float omega[3] = {1.0f, 2.0f, 3.0f};
    const float h[3] = {0.1f, 0.0f, 0.0f};
    const auto r0 = computeInertiaCompensation(omega, kIx, kIy, kIz, h, 0);
    check(r0.Mx_ff == 0.0f && r0.My_ff == 0.0f && r0.Mz_ff == 0.0f,
          "mask=0 → 全零");
    check(!inertiaCompEnabled(0), "inertiaCompEnabled(0)=false");

    const auto rg = computeInertiaCompensation(omega, kIx, kIy, kIz, h,
                                               kInertiaCompGyro);
    const auto rr = computeInertiaCompensation(omega, kIx, kIy, kIz, h,
                                               kInertiaCompRotor);
    const auto rb = computeInertiaCompensation(omega, kIx, kIy, kIz, h,
                                               kInertiaCompGyro | kInertiaCompRotor);
    check(approx(rb.Mx_ff, rg.Mx_ff + rr.Mx_ff) &&
          approx(rb.My_ff, rg.My_ff + rr.My_ff) &&
          approx(rb.Mz_ff, rg.Mz_ff + rr.Mz_ff),
          "全开 = 两项线性叠加");
    check(inertiaCompEnabled(kInertiaCompGyro | kInertiaCompRotor),
          "inertiaCompEnabled(0x03)=true");
}

// ============================================================
//  测试 4：理想刚体闭环对照（前馈的价值验证）
//   被控对象（仿真 dynamics.mjs 同构）：I·ω̇ = M − ω×(I·ω) − ω×h
//   控制器：α_cmd = Kp·(ω_ref − ω)（比例速率环）
//   力矩：M = I·α_cmd + ff（前馈开）/ I·α_cmd（前馈关）
//   理想刚体下前馈开 → ω̇ = α_cmd 精确，跟踪误差应 ~0（浮点级）；
//   前馈关 → 误差 = 耦合项/I（随 ω² 增长）。
// ============================================================
static void test_rigid_body_closed_loop()
{
    // 模拟参数（量级贴近本机，取大角速度突出耦合）
    const float I[3] = {kIx, kIy, kIz};
    const float h[3] = {0.01f, 0.0f, 0.0f};   // 转子净角动量（差速时 ~Jp·Δω）
    const float Kp = 20.0f;                    // 速率环增益
    const float dt = 0.005f;                   // 200Hz
    const float omega_ref[3] = {2.0f, 1.5f, 3.0f};  // 高角速度机动

    auto sim = [&](uint8_t mask) -> float {
        float w[3] = {0.0f, 0.0f, 0.0f};
        constexpr int kSteps = 2000;      // 10s（Kp=20 → 收敛时间常数 50ms）
        constexpr int kSteady = 400;      // 稳态段：最后 2s
        float err_sum = 0.0f;
        int err_cnt = 0;
        for (int step = 0; step < kSteps; ++step)
        {
            // 耦合项 g = ω×(I·ω) + ω×h（被控对象物理，仿真同构）
            const float g[3] = {
                (I[2] - I[1]) * w[1] * w[2] + (w[1] * h[2] - w[2] * h[1]),
                (I[0] - I[2]) * w[2] * w[0] + (w[2] * h[0] - w[0] * h[2]),
                (I[1] - I[0]) * w[0] * w[1] + (w[0] * h[1] - w[1] * h[0]),
            };
            // 控制器
            const float alpha_cmd[3] = {
                Kp * (omega_ref[0] - w[0]),
                Kp * (omega_ref[1] - w[1]),
                Kp * (omega_ref[2] - w[2]),
            };
            // 前馈（与控制器共用同一惯量/角速度——理想情况）
            const auto ff = computeInertiaCompensation(w, I[0], I[1], I[2], h, mask);
            const float M[3] = {
                I[0] * alpha_cmd[0] + ff.Mx_ff,
                I[1] * alpha_cmd[1] + ff.My_ff,
                I[2] * alpha_cmd[2] + ff.Mz_ff,
            };
            // 被控对象积分（欧拉）
            for (int k = 0; k < 3; ++k)
            {
                const float wdot = (M[k] - g[k]) / I[k];
                w[k] += wdot * dt;
            }
            if (step >= kSteps - kSteady)
            {
                for (int k = 0; k < 3; ++k)
                {
                    const float e = omega_ref[k] - w[k];
                    err_sum += e * e;
                }
                err_cnt += 3;
            }
        }
        return std::sqrt(err_sum / err_cnt);
    };

    const float err_on  = sim(kInertiaCompGyro | kInertiaCompRotor);
    const float err_off = sim(0);
    std::printf("  [info] 前馈开 RMS 跟踪误差 = %.3e rad/s，关 = %.3e rad/s\n",
                err_on, err_off);
    check(err_off > 1e-3f, "前馈关：存在耦合跟踪误差（>1e-3 rad/s）");
    check(err_on < err_off * 1e-3f, "前馈开：误差比关闭小 3 个数量级（理想刚体精确补偿）");
}

int main()
{
    test_gyro_coupling();
    test_rotor_coupling();
    test_mask();
    test_rigid_body_closed_loop();

    std::printf("\n%s（失败 %d）\n", g_fail_count == 0 ? "全部通过" : "存在失败",
                g_fail_count);
    return g_fail_count;
}
