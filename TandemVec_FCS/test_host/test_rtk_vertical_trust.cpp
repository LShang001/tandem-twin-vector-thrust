/**
 * @file    test_rtk_vertical_trust.cpp
 * @module  固件/宿主回归测试（fw-tests：RTK 垂直可信仲裁纯函数层）
 * @brief   验证 RTK 垂直可信状态机的迟滞、四条件门、立即退出与地板渐降语义
 * @details 被测：`include/ins_rtk_vertical_trust.h` 的 RtkVerticalTrustUpdate/
 *          MakeState；不依赖 Arduino/STM32 HAL。输入：手工构造的配置与四条件
 *          （新鲜/fix==6/vAcc/无 spoof）+ dt；处理：逐项断言进入迟滞、各门拒绝、
 *          安全侧立即退出、地板渐降到 0 与回升、dt 钳制；输出：PASS/FAIL 计数。
 *          运行（仓库根目录）：
 *          g++ -std=c++17 -Iinclude test_host/test_rtk_vertical_trust.cpp -o bin/rvt
 * @theory  迟滞积分 + 线性渐降的工程整定，非教材公式；渐降避免 R 骤降导致 NIS
 *          暴涨被 χ² 门拒收。@ref REF-121 §7.3.1 p.255（NIS/FDI）
 * @frame   N/A（垂直通道噪声仲裁）
 * @units   精度/地板 m；时长 s
 * @test    S1 初始化；S2 进入迟滞；S3 vAcc 门（0/负/超限/边界）；S4 fix/新鲜/spoof 门；
 *          S5 立即退出+地板回升；S6 再进入须重新迟滞；S7 位置地板渐降到 0；S8 dt 钳制；
 *          S9 ramp_s≤0 立即到位；S10 非可信态地板恒为默认；S11 垂直速度地板同步渐降/回升
 * @see     TandemVec_FCS/include/ins_rtk_vertical_trust.h、test_host/README.md
 */
#include "../include/ins_rtk_vertical_trust.h"

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

static bool approx(float a, float b, float rel = 1e-5f, float abs = 1e-6f)
{
  return std::fabs(a - b) <= std::fmax(abs, rel * std::fmax(std::fabs(a), std::fabs(b)));
}

// 默认配置：confirm_s=1.0、max_dt=0.25（0.25 二进制精确，4 拍恰好 1.0 s，避免边界抖动）。
static bfs::RtkVerticalTrustConfig defaultCfg()
{
  bfs::RtkVerticalTrustConfig cfg;
  cfg.max_v_acc_m = 0.1f;
  cfg.confirm_s = 1.0f;
  cfg.floor_default_m = 3.0f;
  cfg.floor_rtk_m = 0.0f;
  cfg.floor_vel_default_mps = 0.25f;
  cfg.floor_vel_rtk_mps = 0.0f;
  cfg.ramp_s = 3.0f;
  cfg.max_dt_s = 0.25f;
  return cfg;
}

// 以 dt=0.25 跑 n 拍「全条件满足」，返回最后一拍的 trusted。
static bool runGood(bfs::RtkVerticalTrustState &s, const bfs::RtkVerticalTrustConfig &cfg, int n)
{
  bool t = false;
  for (int i = 0; i < n; ++i)
    t = bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.02f, false, 0.25f);
  return t;
}

int main()
{
  const auto cfg = defaultCfg();

  // ---- S1 初始化：地板=默认、非可信、累计=0 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    check(approx(s.floor_m, 3.0f), "S1 初始地板=floor_default");
    check(approx(s.floor_vel_mps, 0.25f), "S1 初始速度地板=floor_vel_default");
    check(!s.trusted, "S1 初始非可信");
    check(approx(s.confirm_accum_s, 0.0f), "S1 初始累计=0");
  }

  // ---- S2 进入迟滞：3 拍(0.75s)未可信，第 4 拍(1.0s)可信 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    runGood(s, cfg, 3);
    check(!s.trusted, "S2 0.75s 尚未可信（迟滞）");
    check(approx(s.confirm_accum_s, 0.75f), "S2 累计=0.75s");
    bool t = runGood(s, cfg, 1);
    check(t && s.trusted, "S2 1.0s 进入可信");
  }

  // ---- S3 vAcc 门：0/负/超限拒绝，边界 0.1 接受 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.0f, false, 0.25f);
    check(!s.trusted, "S3 vAcc=0（NMEA 未知）拒绝");
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, true, -1.0f, false, 0.25f);
    check(!s.trusted, "S3 vAcc<0 拒绝");
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.2f, false, 0.25f);
    check(!s.trusted, "S3 vAcc=0.2>门限 拒绝");
    auto s2 = bfs::RtkVerticalTrustMakeState(cfg);
    bool t = false;
    for (int i = 0; i < 4; ++i)
      t = bfs::RtkVerticalTrustUpdate(s2, cfg, true, true, 0.1f, false, 0.25f);
    check(t, "S3 vAcc=0.1 边界接受");
  }

  // ---- S4 fix/新鲜/spoof 门：任一不满足拒绝 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, false, 0.02f, false, 0.25f);
    check(!s.trusted, "S4 fix!=6 拒绝");
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, false, true, 0.02f, false, 0.25f);
    check(!s.trusted, "S4 GNSS 不新鲜拒绝");
    for (int i = 0; i < 20; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.02f, true, 0.25f);
    check(!s.trusted, "S4 垂直 spoof 疑似拒绝（互锁）");
  }

  // ---- S5 立即退出 + 地板回升：可信后一拍断流即退出，地板回 3 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    runGood(s, cfg, 16);  // 进入可信并渐降一段
    check(s.trusted && s.floor_m < 3.0f, "S5 前置：已可信且地板已降");
    bool t = bfs::RtkVerticalTrustUpdate(s, cfg, false, true, 0.02f, false, 0.25f);
    check(!t && !s.trusted, "S5 断流一拍立即退出");
    check(approx(s.floor_m, 3.0f), "S5 退出后地板立即回升 3");
    check(approx(s.floor_vel_mps, 0.25f), "S5 退出后速度地板立即回升 0.25");
    check(approx(s.confirm_accum_s, 0.0f), "S5 退出后累计清零");
  }

  // ---- S6 再进入须重新迟滞：退出后 3 拍好条件仍不可信 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    runGood(s, cfg, 16);
    bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.2f, false, 0.25f);  // vAcc 超限退出
    runGood(s, cfg, 3);
    check(!s.trusted, "S6 再进入 0.75s 仍不可信（须重新迟滞）");
    runGood(s, cfg, 1);
    check(s.trusted, "S6 再进入满 1.0s 恢复可信");
  }

  // ---- S7 地板渐降到 0 并钳制：rate=1m/s，足够拍后到 0 不再降 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    runGood(s, cfg, 4);  // 刚可信，floor=2.75
    check(approx(s.floor_m, 2.75f), "S7 可信首拍地板降一步(3→2.75)");
    float prev = s.floor_m;
    bool monotonic = true;
    for (int i = 0; i < 40; ++i)
    {
      runGood(s, cfg, 1);
      if (s.floor_m > prev + 1e-6f) monotonic = false;
      prev = s.floor_m;
    }
    check(monotonic, "S7 地板单调不增");
    check(approx(s.floor_m, 0.0f), "S7 地板渐降到 0（取消地板）");
    runGood(s, cfg, 5);
    check(approx(s.floor_m, 0.0f), "S7 到 0 后钳制不再降");
  }

  // ---- S8 dt 钳制：超 max_dt 钳到 0.25；负 dt 钳到 0 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.02f, false, 1.0f);
    check(approx(s.confirm_accum_s, 0.25f), "S8 dt=1.0 钳到 max_dt=0.25");
    auto s2 = bfs::RtkVerticalTrustMakeState(cfg);
    bfs::RtkVerticalTrustUpdate(s2, cfg, true, true, 0.02f, false, -0.5f);
    check(approx(s2.confirm_accum_s, 0.0f), "S8 负 dt 钳到 0");
  }

  // ---- S9 ramp_s≤0：可信后地板立即到位 ----
  {
    auto cfg2 = defaultCfg();
    cfg2.ramp_s = 0.0f;
    auto s = bfs::RtkVerticalTrustMakeState(cfg2);
    runGood(s, cfg2, 4);
    check(s.trusted && approx(s.floor_m, 0.0f), "S9 ramp_s=0 可信后立即取消地板");
  }

  // ---- S10 非可信态地板恒为默认（无论累计多少）----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    for (int i = 0; i < 3; ++i)
      bfs::RtkVerticalTrustUpdate(s, cfg, true, true, 0.02f, false, 0.25f);
    check(!s.trusted && approx(s.floor_m, 3.0f), "S10 迟滞中地板仍=默认");
    check(approx(s.floor_vel_mps, 0.25f), "S10 迟滞中速度地板仍=默认");
  }

  // ---- S11 垂直速度地板：与位置同步渐降到 0、退出回升 ----
  {
    auto s = bfs::RtkVerticalTrustMakeState(cfg);
    runGood(s, cfg, 4);  // 刚可信
    // rate = 0.25/3.0 ≈ 0.08333 m/s²，dt=0.25 → 首拍降 0.020833
    check(approx(s.floor_vel_mps, 0.25f - (0.25f / 3.0f) * 0.25f), "S11 可信首拍速度地板降一步");
    float prev = s.floor_vel_mps;
    bool mono = true;
    for (int i = 0; i < 40; ++i)
    {
      runGood(s, cfg, 1);
      if (s.floor_vel_mps > prev + 1e-6f) mono = false;
      prev = s.floor_vel_mps;
    }
    check(mono, "S11 速度地板单调不增");
    check(approx(s.floor_vel_mps, 0.0f), "S11 速度地板渐降到 0（取消地板）");
    bfs::RtkVerticalTrustUpdate(s, cfg, false, true, 0.02f, false, 0.25f);
    check(approx(s.floor_vel_mps, 0.25f), "S11 退出后速度地板回升 0.25");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== RTK 垂直可信仲裁测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}