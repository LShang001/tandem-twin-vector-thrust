/**
 * @file    test_inertia_decoupling.cpp
 * @module  固件/宿主回归测试（fw-tests：惯量逆解的交叉耦合前馈）
 * @brief   验证 ω×(Iω) 与 ω×h 两项前馈的数值展开、掩码开关与闭环收益
 * @details 被测：`include/InertiaDecoupling.h`（`computeInertiaCompensation()`）。
 *          覆盖：1) ω×(I·ω) 陀螺耦合逐项数值展开（对照仿真 dynamics.mjs 的 gx/gy/gz）；
 *          2) ω×h 转子角动量陀螺项展开；3) 使能掩码按位开关（mask=0 → 全零）；
 *          4) 理想刚体闭环对照：前馈开 → 跟踪误差 ~0，前馈关 → 误差等于耦合项/I；
 *          5) 本机细长杆特性 Iy=Iz → 绕 x 轴陀螺耦合项恒 0。
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_inertia_decoupling.cpp -o bin/id
 * @theory  刚体角动力学 `I·ω̇ = M − ω×(Iω) − ω×h`；前馈项即把两个耦合项
 *          提前算进力矩指令，使闭环回到“解耦的二阶系统”。
 *          @ref REF-120 §9.2 p.267（角速度/角动量关系）、@ref THY-002 §2–3（转子角动量）
 * @frame   模型系（x′=推力轴）为前馈原语坐标系；存档系（x=滚转、y=俯仰、
 *          z=推力轴）由 `computeInertiaCompensationFromArchive()` 换算
 * @units   惯量 kg·m²，角速率 rad/s，角动量 kg·m²/s，力矩 N·m
 * @note    数值期望值全部手算：ω=(1,2,3)、Iy=Iz=0.022、Ix=0.0021 时
 *          My_ff=(Ix−Iz)·3·1=−0.0597、Mz_ff=(Iy−Ix)·1·2=0.0398。
 *          容差 1e-4 相对（float 在 1e-2 量级约 6–7 位有效数字，手算式与
 *          代码编译期常量同源）。
 * @test    T1 陀螺耦合逐项；T2 ω×h 展开；T3 掩码按位；T4 闭环前馈收益；
 *          T5 细长杆 Iy=Iz 时 Mx 恒零；T6 存档系封装（陀螺耦合）；
 *          T7 存档系封装（转子陀螺项 + 与显式置换等价）；
 *          T8 存档系直接叉乘（H=(0,0,−h)）与封装逐项一致
 * @see     TandemVec_FCS/include/InertiaDecoupling.h、src/flight_control.cpp（mix 层调用点）
 */
// 宿主机回归测试：惯量逆解交叉耦合前馈（InertiaDecoupling.h）
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
// @formula gx=(Iz−Iy)·wy·wz；gy=(Ix−Iz)·wz·wx；gz=(Iy−Ix)·wx·wy
// @note 取 ω=(1,2,3) rad/s 手算：gx=0、gy=−0.0597、gz=0.0398。
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

// ============================================================
//  测试 6：存档系封装——陀螺耦合的轴置换（2026-10-23 C-02 回归）
//  存档系 ω=(0,1,1) → 模型系 ω′=(−ωz, ωy, ωx)=(−1,1,0)；
//    I=(Ix=0.0021, Iy=0.022, Iz=0.022)：
//    Mz_ff=(Iy−Ix)·ωx′·ωy′=(0.022−0.0021)·(−1)·1=−0.0199 N·m。
//  旧实现直接传存档系 ω 且 Ix 当模型系 → 同用例三项全 0（漏补偿）。
// ============================================================
static void test_archive_frame_gyro()
{
    const float omega_archive[3] = {0.0f, 1.0f, 1.0f};
    const auto r = computeInertiaCompensationFromArchive(
        omega_archive, kIz, kIy, kIx, 0.0f, kInertiaCompGyro);
    check(approx(r.Mx_ff, 0.0f), "T6 存档系 Mx=0（模型系 Iz=Iy）");
    check(approx(r.My_ff, 0.0f), "T6 存档系 My=0（ωz′=0）");
    check(approx(r.Mz_ff, (kIy - kIx) * (-1.0f) * 1.0f),
          "T6 存档系 Mz=(Iy−Ix)·ωx′ωy′=−0.0199（轴置换）");

    // 反例：旧实现把存档系 ω 直接当模型系 → 本用例三项全 0（动力学上漏补偿）
    const float h_zero[3] = {0.0f, 0.0f, 0.0f};
    const auto legacy = computeInertiaCompensation(omega_archive, kIx, kIy, kIz,
                                                   h_zero, kInertiaCompGyro);
    check(legacy.Mx_ff == 0.0f && legacy.My_ff == 0.0f && legacy.Mz_ff == 0.0f,
          "T6 旧直调在该机动下恒 0（C-02 缺陷可复现）");
    check(std::fabs(r.Mz_ff) > 1e-3f, "T6 新封装恢复非零耦合补偿");
}

// ============================================================
//  测试 7：存档系封装——转子角动量 + 与显式置换等价
//  存档系 ω=(0,1,0)（俯仰速率），h 沿推力轴 0.1：
//    模型系 ω′=(0,1,0)、h′=(0.1,0,0) → ω′×h′=(0,0,−0.1)。
// ============================================================
static void test_archive_frame_rotor()
{
    const float omega_archive[3] = {0.0f, 1.0f, 0.0f};
    const auto r = computeInertiaCompensationFromArchive(
        omega_archive, kIz, kIy, kIx, 0.1f, kInertiaCompRotor);
    check(approx(r.Mx_ff, 0.0f) && approx(r.My_ff, 0.0f) &&
              approx(r.Mz_ff, -0.1f),
          "T7 存档系转子陀螺 Mz=−0.1（ω′×h′）");

    // 与显式轴置换逐项等价（mask 全开）
    const float omega_archive2[3] = {0.3f, -0.2f, 0.5f};
    const float h = 0.01f;
    const auto wrapped = computeInertiaCompensationFromArchive(
        omega_archive2, kIz, kIy, kIx, h,
        kInertiaCompGyro | kInertiaCompRotor);
    const float omega_model[3] = {-omega_archive2[2], omega_archive2[1],
                                  omega_archive2[0]};
    const float h_model[3] = {h, 0.0f, 0.0f};
    const auto explicit_ref = computeInertiaCompensation(
        omega_model, kIx, kIy, kIz, h_model,
        kInertiaCompGyro | kInertiaCompRotor);
    check(approx(wrapped.Mx_ff, explicit_ref.Mx_ff) &&
              approx(wrapped.My_ff, explicit_ref.My_ff) &&
              approx(wrapped.Mz_ff, explicit_ref.Mz_ff),
          "T7 封装 = 显式 (−ωz,ωy,ωx)+(h,0,0) 置换");
}

// ============================================================
//  测试 8：存档系直接叉乘（h_archive=(0,0,−h)）与封装逐项一致
//  说明：模型 x′=−z_b，故模型系 +x′ 的 h 对应存档系 −z 的 h；
//        本测试用存档系欧拉方程直接算，再按 M_model=(−M_z, M_y, M_x) 变换，
//        与封装输出逐项比较，锁定 h 符号约定。
// ============================================================
static void test_archive_direct_equivalence()
{
    const float w[3] = {0.5f, 0.3f, 1.0f};
    const float h = 0.005f;
    const float I_arch[3] = {kIz, kIy, kIx};  // 存档系 x=滚转、y=俯仰、z=推力轴
    const float H_arch[3] = {0.0f, 0.0f, -h};
    const float ff_arch[3] = {
        (I_arch[2] - I_arch[1]) * w[1] * w[2] +
            (w[1] * H_arch[2] - w[2] * H_arch[1]),
        (I_arch[0] - I_arch[2]) * w[2] * w[0] +
            (w[2] * H_arch[0] - w[0] * H_arch[2]),
        (I_arch[1] - I_arch[0]) * w[0] * w[1] +
            (w[0] * H_arch[1] - w[1] * H_arch[0]),
    };
    const auto r = computeInertiaCompensationFromArchive(
        w, kIz, kIy, kIx, h, kInertiaCompGyro | kInertiaCompRotor);
    check(approx(r.Mx_ff, -ff_arch[2]) &&
              approx(r.My_ff, ff_arch[1]) &&
              approx(r.Mz_ff, ff_arch[0]),
          "T8 封装 = 存档系直接叉乘经 (−z,y,x) 变换");
}

int main()
{
    test_gyro_coupling();
    test_rotor_coupling();
    test_mask();
    test_rigid_body_closed_loop();
    test_archive_frame_gyro();
    test_archive_frame_rotor();
    test_archive_direct_equivalence();

    std::printf("\n%s（失败 %d）\n", g_fail_count == 0 ? "全部通过" : "存在失败",
                g_fail_count);
    return g_fail_count;
}
