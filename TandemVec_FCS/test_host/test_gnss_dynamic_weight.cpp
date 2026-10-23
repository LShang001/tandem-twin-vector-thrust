/**
 * @file    test_gnss_dynamic_weight.cpp
 * @module  固件/宿主回归测试（fw-tests：导航量测噪声阵 R 的纯函数层）
 * @brief   验证 GNSS 动态权重（精度字段→R 标准差）纯函数的门控、地板与缩放语义
 * @details 被测：`include/ins_gnss_dynamic_weight.h` 中的配置校验与
 *          `Icm45686ComputeGnssStd()` 类纯函数；不依赖 Arduino/STM32 HAL。
 *          输入：手工构造的 `Icm45686GnssDynamicWeightConfig` 与 GNSS 精度字段；
 *          处理：逐项断言 fix 门控、floor/cap 钳制、pDOP 缩放与 v_acc/s_acc 回退；
 *          输出：PASS/FAIL 计数，main 返回非零即回归失败。
 *          运行（仓库根目录）：
 *          g++ -std=c++17 -Iinclude test_host/test_gnss_dynamic_weight.cpp -o bin/gdw
 * @theory  GNSS 量测噪声阵 R 的工程整定，非教材公式：精度字段→1σ 的线性映射、
 *          噪声地板/天花板、pDOP 放大与 fix 类型门控。默认参数与
 *          `navigation_task.cpp` 的 kGnssDwCfg 一致，锁定实机配置。
 *          @ref AGENTS.md（EKF 踩坑：R 地板不可收紧，收紧后接受率暴跌）
 * @frame   N/A（标量精度字段，与 NED/FRD 无关）
 * @units   位置标准差 m；速度标准差 m/s；pDOP 无量纲；fix 类型为 UBX 枚举
 * @note    本文件锁定“pDOP 缩放在 clamp 之后（缩放结果可突破 cap）”的当前语义；
 *          改顺序属行为变更，须同步 `navigation_task.cpp` 并重采 EKF 回归证据。
 * @test    S1 fix 类型/卫星数/h_acc 门控；S2 精度区间内直通；
 *          S3 噪声地板与天花板；S4 v_acc/s_acc 为 0 回退；S5 pDOP 缩放；
 *          S6 pDOP 非法值视为未提供；S7 RTK 高 fix 类型
 * @see     TandemVec_FCS/include/ins_gnss_dynamic_weight.h、test_host/README.md
 */
#include "../include/ins_gnss_dynamic_weight.h"

#include <cmath>
#include <cstdio>
#include <string>

static int g_fail_count = 0;

static void check(bool cond, const std::string &name)
{
  if (cond)
  {
    std::printf("[PASS] %s\n", name.c_str());
  }
  else
  {
    std::printf("[FAIL] %s\n", name.c_str());
    ++g_fail_count;
  }
}

// 判定浮点近似相等（相对容差，避免不同平台 sqrt 实现微小差异）。
static bool approx(float a, float b, float rel = 1e-5f, float abs = 1e-6f)
{
  return std::fabs(a - b) <= std::fmax(abs, rel * std::fmax(std::fabs(a), std::fabs(b)));
}

int main()
{
  // 使用与 navigation_task.cpp 中 kGnssDwCfg 一致的默认配置，保证测试覆盖实机配置。
  Icm45686GnssDynamicWeightConfig cfg{};
  cfg.floor_pos_ne_m = 2.0f;
  cfg.floor_pos_d_m = 3.0f;
  cfg.floor_vel_ne_mps = 0.15f;
  cfg.floor_vel_d_mps = 0.25f;
  cfg.cap_pos_ne_m = 30.0f;
  cfg.cap_pos_d_m = 50.0f;
  cfg.cap_vel_mps = 3.0f;
  cfg.pdop_ref = 2.0f;
  cfg.min_sv = 3;

  // ---- 最小门限：低于 3D fix 不通过 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(2, 10, 2.0f, 3.0f, 0.2f, 2.0f, cfg);
    check(!r.passed_minimum, "fix<3 应不通过最小门限");
  }
  // ---- 最小门限：卫星数不足不通过 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 2, 2.0f, 3.0f, 0.2f, 2.0f, cfg);
    check(!r.passed_minimum, "num_sv<min_sv 应不通过最小门限");
  }
  // ---- 最小门限：h_acc<=0 不通过 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 0.0f, 3.0f, 0.2f, 2.0f, cfg);
    check(!r.passed_minimum, "h_acc<=0 应不通过最小门限");
    auto r2 = Icm45686ComputeGnssDynamicWeights(3, 10, -1.0f, 3.0f, 0.2f, 2.0f, cfg);
    check(!r2.passed_minimum, "h_acc<0 应不通过最小门限");
  }
  // ---- 正常输入：精度在 floor/cap 之间，直接作为 std，无 pDOP 缩放 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 5.0f, 4.0f, 0.5f, 1.0f, cfg);
    check(r.passed_minimum, "合法 3D fix 应通过最小门限");
    check(approx(r.eff_pos_ne_std_m, 5.0f), "pos_ne_std 取 h_acc (pDOP<=ref 不缩放)");
    check(approx(r.eff_pos_d_std_m, 4.0f), "pos_d_std 取 v_acc");
    check(approx(r.eff_vel_ne_std_mps, 0.5f), "vel_ne_std 取 s_acc");
    check(approx(r.eff_vel_d_std_mps, 0.5f), "vel_d_std 取 s_acc");
  }
  // ---- 噪声地板：h_acc 低于 floor 时被抬到 floor ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 0.5f, 1.0f, 0.05f, 1.0f, cfg);
    check(approx(r.eff_pos_ne_std_m, cfg.floor_pos_ne_m), "h_acc<floor 抬到 floor_pos_ne");
    check(approx(r.eff_pos_d_std_m, cfg.floor_pos_d_m), "v_acc<floor 抬到 floor_pos_d");
    check(approx(r.eff_vel_ne_std_mps, cfg.floor_vel_ne_mps), "s_acc<floor 抬到 floor_vel_ne");
    check(approx(r.eff_vel_d_std_mps, cfg.floor_vel_d_mps), "s_acc<floor 抬到 floor_vel_d");
  }
  // ---- 噪声天花板：精度超过 cap 时被压到 cap ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 100.0f, 200.0f, 50.0f, 1.0f, cfg);
    check(approx(r.eff_pos_ne_std_m, cfg.cap_pos_ne_m), "h_acc>cap 压到 cap_pos_ne");
    check(approx(r.eff_pos_d_std_m, cfg.cap_pos_d_m), "v_acc>cap 压到 cap_pos_d");
    check(approx(r.eff_vel_ne_std_mps, cfg.cap_vel_mps), "s_acc>cap 压到 cap_vel");
    check(approx(r.eff_vel_d_std_mps, cfg.cap_vel_mps), "s_acc>cap 压到 cap_vel (vel_d 共用 cap)");
  }
  // ---- v_acc/s_acc 为 0 时回退 h_acc*1.5 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 5.0f, 0.0f, 0.0f, 1.0f, cfg);
    // v_acc_eff = 5*1.5 = 7.5 (在 floor3~cap50 之间)
    check(approx(r.eff_pos_d_std_m, 7.5f), "v_acc=0 回退 h_acc*1.5");
    // s_acc_eff = 7.5 > cap_vel 3.0，应被 cap 压到 3.0
    check(approx(r.eff_vel_ne_std_mps, cfg.cap_vel_mps), "s_acc=0 回退值>cap 被压到 cap_vel");
    check(approx(r.eff_vel_d_std_mps, cfg.cap_vel_mps), "vel_d 同样被 cap 限制");
  }
  // ---- pDOP 缩放：pDOP>ref 时按 pDOP/ref 放大所有 std ----
  {
    const float h = 5.0f, v = 4.0f, s = 0.5f;
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, h, v, s, 4.0f, cfg); // pdop/ref = 4/2 = 2.0
    check(approx(r.eff_pos_ne_std_m, h * 2.0f), "pDOP 缩放 pos_ne");
    check(approx(r.eff_pos_d_std_m, v * 2.0f), "pDOP 缩放 pos_d");
    check(approx(r.eff_vel_ne_std_mps, s * 2.0f), "pDOP 缩放 vel_ne");
    check(approx(r.eff_vel_d_std_mps, s * 2.0f), "pDOP 缩放 vel_d");
  }
  // ---- pDOP<=0 视为未提供，缩放因子=1.0 ----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 5.0f, 4.0f, 0.5f, -1.0f, cfg);
    check(approx(r.eff_pos_ne_std_m, 5.0f), "pDOP<0 不缩放");
  }
  // ---- pDOP 缩放在 clamp 之后：缩放结果可突破 cap（锁定当前实现语义）----
  // 实现为 pdop_scale * clamp(h_acc, floor, cap)：h_acc=20 在 [2,30] 内 clamp 后仍 20，
  // 再乘 pDOP_scale=4/2=2.0 得 40。即 pDOP 缩放不受 cap 二次约束。
  // 本用例锁定该顺序，防止未来误把 clamp 移到缩放之后而改变 EKF 融合权重。
  {
    auto r = Icm45686ComputeGnssDynamicWeights(3, 10, 20.0f, 20.0f, 2.0f, 4.0f, cfg);
    check(approx(r.eff_pos_ne_std_m, 40.0f), "pDOP 缩放发生在 clamp 之后 (20*2=40, 突破 cap30)");
  }
  // ---- RTK 高 fix 类型也应通过（fix>=3 即可）----
  {
    auto r = Icm45686ComputeGnssDynamicWeights(5, 12, 0.3f, 0.4f, 0.05f, 1.0f, cfg);
    check(r.passed_minimum, "RTK fix (fix>=3) 通过最小门限");
    check(approx(r.eff_pos_ne_std_m, cfg.floor_pos_ne_m), "RTK 高精度被 floor 保护不过度自信");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== GNSS 动态权重测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}
