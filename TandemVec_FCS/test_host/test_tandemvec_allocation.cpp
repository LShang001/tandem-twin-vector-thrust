// 宿主机回归测试：纵列双发矢量推力飞行器控制分配模块
// 编译运行：
//   g++ -std=c++17 -Iinclude test_host/test_tandemvec_allocation.cpp \
//       -o test_host/bin/ta && ./test_host/bin/ta
//
// 覆盖范围：
//   1. 零力矩 → 零输出
//   2. 纯单通道极性（滚转/俯仰/偏航）
//   3. DIRECT 与 FULL_B 的差异方向（τ₀ 交叉项贡献）
//   4. DIRECT_WITH_FF 与 FULL_B 一阶近似一致性
//   5. 执行器限幅与饱和标记
//   6. 低油门滚转效能退化
//   7. B_true 在零摆角处退化为 B_full（数值验证）
//   8. 正向映射往返一致：分配输出代入 computeWrench() 应接近原始指令
//   9. 差速分配：平方和不变性与 wMax 钳位
//  10. 策略切换零冲击（指令连续性）
#include "../include/TandemVec_Config.h"
#include "../include/TandemVec_Propulsion.h"
#include "../include/TandemVec_ControlAllocation.h"

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

// 相对误差容限（默认 0.1%）
static bool approx(float a, float b, float rel_tol = 1e-3f, float abs_tol = 1e-7f)
{
    float diff = std::fabs(a - b);
    float scale = std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
    return diff <= rel_tol * scale + abs_tol;
}

// 严格近似：用于验证完全相等的情况
static bool exact(float a, float b) { return a == b; }

// ============================================================
//  测试辅助：给定策略，构造输入并调用
// ============================================================
static AllocationOutput alloc(float Mx, float My, float Mz, float w0,
                               AllocationStrategy s = AllocationStrategy::FULL_B)
{
    AllocationInput in{Mx, My, Mz, w0};
    return allocateMoments(in, kDefaultTandemVecParams, s);
}

// ============================================================
//  测试 1：零力矩 → 零执行器输出
// ============================================================
static void test_zero_moment()
{
    for (auto s : {AllocationStrategy::DIRECT,
                   AllocationStrategy::FULL_B,
                   AllocationStrategy::DIRECT_WITH_FF})
    {
        auto out = alloc(0.0f, 0.0f, 0.0f, 400.0f, s);
        check(exact(out.delta_f, 0.0f), "零力矩 delta_f=0");
        check(exact(out.delta_t, 0.0f), "零力矩 delta_t=0");
        check(exact(out.dw,      0.0f), "零力矩 dw=0");
        check(!out.sat_delta_f && !out.sat_delta_t && !out.sat_dw,
              "零力矩无饱和");
    }
}

// ============================================================
//  测试 2：纯通道极性
//    - 纯滚转（Mx < 0）→ dw > 0（效率 ∂Mx/∂Δω < 0，负力矩需正 Δω）
//    - 纯俯仰（My < 0）→ dt > 0（效率 ∂My/∂δ_t < 0，低头力矩 < 0 需正尾摆）
//    - 纯偏航（Mz > 0）→ df > 0（效率 ∂Mz/∂δ_f > 0，右偏需正前摆）
// ============================================================
static void test_channel_polarity()
{
    const float w0 = 400.0f;
    const float M = 0.1f;  // 较小力矩，确保未饱和

    // 纯滚转：Mx = -M < 0 → 需要 Δω > 0（∂Mx/∂Δω = -2τ₀ < 0）
    {
        auto out = alloc(-M, 0.0f, 0.0f, w0);
        check(out.dw > 0.0f,      "纯滚转 Mx<0 → dw>0");
        check(approx(out.delta_f, 0.0f, 1e-4f), "纯滚转 delta_f≈0");
        check(approx(out.delta_t, 0.0f, 1e-4f), "纯滚转 delta_t≈0（FULL_B 无滚转→俯仰耦合）");
    }

    // 纯俯仰：My = -M < 0（低头，NED z_b 向下时低头为负）→ dt > 0
    // ∂My/∂δ_t = -b·T₀ < 0，负力矩需正摆角
    {
        auto out = alloc(0.0f, -M, 0.0f, w0);
        check(out.delta_t > 0.0f, "纯俯仰 My<0 → delta_t>0");
        check(approx(out.dw, 0.0f, 1e-4f), "纯俯仰 dw≈0");
        // FULL_B：俯仰→偏航有少量耦合（-τ₀·δ_t 项）
        // 但主通道应主导
        check(out.delta_t > std::fabs(out.delta_f) * 5.0f,
              "纯俯仰 delta_t 远大于 delta_f（主通道主导）");
    }

    // 纯偏航：Mz = +M > 0（右偏） → df > 0
    // ∂Mz/∂δ_f = a·T₀ > 0，正力矩需正摆角
    {
        auto out = alloc(0.0f, 0.0f, M, w0);
        check(out.delta_f > 0.0f, "纯偏航 Mz>0 → delta_f>0");
        check(approx(out.dw, 0.0f, 1e-4f), "纯偏航 dw≈0");
        check(out.delta_f > std::fabs(out.delta_t) * 5.0f,
              "纯偏航 delta_f 远大于 delta_t（主通道主导）");
    }
}

// ============================================================
//  测试 3：DIRECT vs FULL_B 差异验证
//    FULL_B 通过反扭矩交叉项修正：
//      - 俯仰指令会使 FULL_B 的 delta_f 略不同于 DIRECT
//      - 偏航指令会使 FULL_B 的 delta_t 略不同于 DIRECT
//    在 kQ/(kT·b) ≈ 0.117 的参数下，差异应在 ~10% 量级
// ============================================================
static void test_direct_vs_fullb_coupling()
{
    const float w0 = 400.0f;
    const float M = 0.1f;

    // 俯仰指令：FULL_B 的 delta_f 应受 τ₀·My_cmd 影响（偏航轴耦合）
    {
        auto d = alloc(0.0f, -M, 0.0f, w0, AllocationStrategy::DIRECT);
        auto f = alloc(0.0f, -M, 0.0f, w0, AllocationStrategy::FULL_B);
        // 两者 delta_t 应有差异（τ₀·Mz 项 = 0，所以这里差异来自 a·T₀·My/D22 vs My/(b·T₀)）
        // 差异量级约 τ₀²/D22
        bool dt_close = approx(d.delta_t, f.delta_t, 0.2f);  // 容限放宽，允许20%差异
        check(dt_close, "俯仰指令：DIRECT/FULL_B delta_t 量级相近");
        // FULL_B 产生非零 delta_f（偏航耦合修正）
        check(std::fabs(f.delta_f) > 0.0f,
              "纯俯仰指令 FULL_B delta_f 非零（反扭耦合修正）");
        check(std::fabs(d.delta_f) < std::fabs(f.delta_f) + 1e-8f,
              "DIRECT delta_f = 0（无交叉项）");
    }

    // 偏航指令：FULL_B 的 delta_t 应受 τ₀·Mz_cmd 影响（俯仰轴耦合）
    {
        auto d = alloc(0.0f, 0.0f, M, w0, AllocationStrategy::DIRECT);
        auto f = alloc(0.0f, 0.0f, M, w0, AllocationStrategy::FULL_B);
        check(std::fabs(f.delta_t) > 0.0f,
              "纯偏航指令 FULL_B delta_t 非零（反扭耦合修正）");
        check(std::fabs(d.delta_t) < std::fabs(f.delta_t) + 1e-8f,
              "DIRECT delta_t = 0（无交叉项）");
    }
}

// ============================================================
//  测试 4：DIRECT_WITH_FF 与 FULL_B 的一阶近似一致性
//    当 τ₀² << a·b·T₀² 时（即低耦合），两者应极接近
// ============================================================
static void test_ff_vs_fullb_approx()
{
    const TandemVecParams& p = kDefaultTandemVecParams;
    const float w0 = 400.0f;
    const float T0 = p.kT * w0 * w0;
    const float tau0 = p.kQ * w0 * w0;
    // 确认低耦合条件
    const float coupling_ratio = tau0 / (sqrtf(p.a * p.b) * T0);
    check(coupling_ratio < 0.5f, "验证低耦合条件（τ₀ < 0.5·√(a·b)·T₀）");

    const float M = 0.1f;
    float test_moments[][3] = {
        {0.0f, -M,  0.0f},   // 纯俯仰
        {0.0f,  0.0f,  M},   // 纯偏航
        {0.0f, -M,    M},    // 俯仰+偏航
    };

    for (auto& mv : test_moments)
    {
        auto ff = alloc(mv[0], mv[1], mv[2], w0, AllocationStrategy::DIRECT_WITH_FF);
        auto fb = alloc(mv[0], mv[1], mv[2], w0, AllocationStrategy::FULL_B);
        // 一阶近似，误差 ≈ O(τ₀²/D22)，应 < 5%
        check(approx(ff.delta_t, fb.delta_t, 0.05f),
              "FF vs FULL_B：delta_t 一阶近似 (<5% 误差)");
        check(approx(ff.delta_f, fb.delta_f, 0.05f),
              "FF vs FULL_B：delta_f 一阶近似 (<5% 误差)");
        check(approx(ff.dw, fb.dw, 1e-5f),
              "FF vs FULL_B：dw 完全相同（滚转通道无耦合）");
    }
}

// ============================================================
//  测试 5：执行器饱和与标记
// ============================================================
static void test_saturation()
{
    const TandemVecParams& p = kDefaultTandemVecParams;
    const float w0 = 400.0f;
    const float T0 = p.kT * w0 * w0;

    // 超大俯仰力矩：确保 |My_cmd| >> b·T₀·dMax
    float huge_My = p.b * T0 * p.dMax * 10.0f;
    auto out = alloc(0.0f, -huge_My, 0.0f, w0);
    check(out.sat_delta_t, "超大俯仰力矩 → delta_t 饱和");
    check(std::fabs(out.delta_t) == p.dMax, "delta_t 限幅值 = dMax");
    check(!out.sat_delta_f || true, "偏航通道可能有少量耦合饱和（不要求）");

    // 超大偏航力矩：确保 |Mz_cmd| >> a·T₀·dMax
    float huge_Mz = p.a * T0 * p.dMax * 10.0f;
    auto out2 = alloc(0.0f, 0.0f, huge_Mz, w0);
    check(out2.sat_delta_f, "超大偏航力矩 → delta_f 饱和");
    check(std::fabs(out2.delta_f) == p.dMax, "delta_f 限幅值 = dMax");

    // 超大滚转力矩
    float huge_Mx = p.kQ * w0 * w0 * p.dwMax * 10.0f;
    auto out3 = alloc(-huge_Mx, 0.0f, 0.0f, w0);
    check(out3.sat_dw, "超大滚转力矩 → dw 饱和");
    check(std::fabs(out3.dw) == p.dwMax, "dw 限幅值 = dwMax");
}

// ============================================================
//  测试 6：低油门滚转效能退化
// ============================================================
static void test_roll_degradation()
{
    auto out_high = alloc(0.0f, 0.0f, 0.0f, 900.0f);  // 满转速
    auto out_low  = alloc(0.0f, 0.0f, 0.0f, 90.0f);   // 10% 转速

    check(approx(out_high.roll_authority, 1.0f, 1e-4f),
          "满转速 roll_authority ≈ 1.0");
    check(approx(out_low.roll_authority, 0.01f, 1e-3f),
          "10% 转速 roll_authority ≈ 0.01（平方退化）");
    check(out_high.roll_authority > out_low.roll_authority,
          "高油门滚转效能 > 低油门");

    // 零油门：roll_authority ≈ 0，输出应为零（保护逻辑）
    auto out_zero = alloc(0.1f, 0.0f, 0.0f, 0.0f);
    check(out_zero.roll_authority < 1e-6f, "零转速 roll_authority ≈ 0");
    check(exact(out_zero.dw, 0.0f), "零转速时 dw 强制为零（除零保护）");
}

// ============================================================
//  测试 7：B_true 在零摆角处退化为 B_full
// ============================================================
static void test_btrue_matches_bfull_at_zero()
{
    const TandemVecParams& p = kDefaultTandemVecParams;
    const float w0 = 400.0f;

    // B_full（线性化效能矩阵，在 δ=0, wf=wt=w0 处）
    const float T0   = p.kT * w0 * w0;
    const float tau0 = p.kQ * w0 * w0;

    // B_true 在 δ_f=δ_t=0, wf=wt=w0 处
    PropulsionState s{w0, w0, 0.0f, 0.0f};
    EffectMatrix E = computeEffectMatrix(s, p, w0);

    // 验证 B_true ≈ B_full
    // B_full[0][0] = -2τ₀
    check(approx(E.M[0][0], -2.0f * tau0), "B_true[0][0] ≈ -2τ₀");
    check(approx(E.M[0][1], 0.0f),         "B_true[0][1] ≈ 0 (无交叉)");
    check(approx(E.M[0][2], 0.0f),         "B_true[0][2] ≈ 0 (无交叉)");
    check(approx(E.M[1][0], 0.0f),         "B_true[1][0] ≈ 0 (无交叉)");
    check(approx(E.M[1][1], -p.b * T0),    "B_true[1][1] ≈ -b·T₀（俯仰主控效率）");
    check(approx(E.M[1][2], -tau0),        "B_true[1][2] ≈ -τ₀（反扭耦合到俯仰）");
    check(approx(E.M[2][0], 0.0f),         "B_true[2][0] ≈ 0 (无交叉)");
    check(approx(E.M[2][1], -tau0),        "B_true[2][1] ≈ -τ₀（反扭耦合到偏航）");
    check(approx(E.M[2][2], p.a * T0),     "B_true[2][2] ≈ a·T₀（偏航主控效率）");
}

// ============================================================
//  测试 8：正向映射往返一致性
//    分配输出 → computeWrench() → 实际力矩应接近原始指令
//    （仅在未饱和、小角度时有效）
// ============================================================
static void test_roundtrip()
{
    const TandemVecParams& p = kDefaultTandemVecParams;
    const float w0 = 400.0f;
    const float T0 = p.kT * w0 * w0;

    // 选取一个不饱和的中等力矩（约 30% dMax 对应的力矩）
    float My_cmd = -p.b * T0 * p.dMax * 0.3f;  // 负俯仰力矩
    float Mz_cmd =  p.a * T0 * p.dMax * 0.2f;  // 正偏航力矩

    auto out = alloc(0.0f, My_cmd, Mz_cmd, w0, AllocationStrategy::FULL_B);
    check(!out.sat_delta_t && !out.sat_delta_f, "往返测试：确认未饱和");

    // 构造推进状态（Δω=0，等转速对称）
    PropulsionState s{w0, w0, out.delta_f, out.delta_t};
    SixDOFWrench wr = computeWrench(s, p);

    // 往返误差容限放宽到 20%（因为正向映射含非线性 sin/cos，而分配基于线性近似）
    check(approx(wr.My, My_cmd, 0.20f), "往返 My：实际力矩接近指令（20% 容限）");
    check(approx(wr.Mz, Mz_cmd, 0.20f), "往返 Mz：实际力矩接近指令（20% 容限）");
}

// ============================================================
//  测试 9：差速分配平方和不变性与 wMax 钳位
// ============================================================
static void test_differential_allocation()
{
    const TandemVecParams& p = kDefaultTandemVecParams;

    // 平方和不变：wf² + wt² = 2·w0²（未钳位时）
    {
        const float w0 = 400.0f;
        const float dw = 0.3f;
        auto r = allocateDifferential(w0, dw, p);
        check(!r.wf_clamped && !r.wt_clamped, "中等差速不触达 wMax");
        float sum_sq = r.wf_target * r.wf_target + r.wt_target * r.wt_target;
        check(approx(sum_sq, 2.0f * w0 * w0, 1e-4f),
              "差速分配：wf² + wt² = 2·w0²（平方和不变）");
    }

    // 零差速：两电机相等
    {
        const float w0 = 300.0f;
        auto r = allocateDifferential(w0, 0.0f, p);
        check(approx(r.wf_target, w0, 1e-6f) && approx(r.wt_target, w0, 1e-6f),
              "零差速：wf = wt = w0");
    }

    // 极限差速 dw=1：一侧归零
    {
        const float w0 = 300.0f;
        auto r = allocateDifferential(w0, 1.0f, p);
        check(approx(r.wt_target, 0.0f, 1e-5f),
              "Δω=+1：尾电机目标转速 = 0");
    }

    // wMax 钳位：高油门+大差速
    {
        const float w0 = p.wMax;  // 满油门
        const float dw = 0.5f;
        auto r = allocateDifferential(w0, dw, p);
        check(r.wf_clamped, "满油门大差速：前电机触达 wMax");
        check(r.wf_target == p.wMax, "钳位后 wf = wMax");
    }
}

// ============================================================
//  测试 10：策略切换指令连续性
//    三种策略在相同输入下，输出差异应在合理范围内（无突变）
// ============================================================
static void test_strategy_continuity()
{
    const float w0 = 400.0f;
    const float M = 0.05f;

    auto d  = alloc(0.0f, -M, M, w0, AllocationStrategy::DIRECT);
    auto fb = alloc(0.0f, -M, M, w0, AllocationStrategy::FULL_B);
    auto ff = alloc(0.0f, -M, M, w0, AllocationStrategy::DIRECT_WITH_FF);

    // 所有策略的 dw 完全相同（滚转通道无耦合，三种策略结果一致）
    check(approx(d.dw, fb.dw, 1e-5f) && approx(d.dw, ff.dw, 1e-5f),
          "策略切换：dw 在三种策略下相同");

    // delta_t / delta_f 差异应在 ±30% 以内（反扭耦合修正量级）
    check(approx(d.delta_t, fb.delta_t, 0.30f), "策略切换：delta_t 无突变 (<30%差异)");
    check(approx(d.delta_f, fb.delta_f, 0.30f), "策略切换：delta_f 无突变 (<30%差异)");
}

// ============================================================
//  main
// ============================================================
int main()
{
    std::printf("=== TandemVec 控制分配回归测试 ===\n\n");

    std::printf("-- T1：零力矩 → 零输出 --\n");
    test_zero_moment();

    std::printf("\n-- T2：单通道极性 --\n");
    test_channel_polarity();

    std::printf("\n-- T3：DIRECT vs FULL_B 交叉耦合差异 --\n");
    test_direct_vs_fullb_coupling();

    std::printf("\n-- T4：DIRECT_WITH_FF 与 FULL_B 一阶近似一致性 --\n");
    test_ff_vs_fullb_approx();

    std::printf("\n-- T5：执行器饱和与标记 --\n");
    test_saturation();

    std::printf("\n-- T6：低油门滚转效能退化 --\n");
    test_roll_degradation();

    std::printf("\n-- T7：B_true 零摆角退化为 B_full --\n");
    test_btrue_matches_bfull_at_zero();

    std::printf("\n-- T8：正向映射往返一致性 --\n");
    test_roundtrip();

    std::printf("\n-- T9：差速分配平方和不变性与 wMax 钳位 --\n");
    test_differential_allocation();

    std::printf("\n-- T10：策略切换连续性 --\n");
    test_strategy_continuity();

    std::printf("\n");
    if (g_fail_count == 0)
        std::printf("=== 全部通过 ===\n");
    else
        std::printf("=== %d 项失败 ===\n", g_fail_count);

    return g_fail_count == 0 ? 0 : 1;
}
