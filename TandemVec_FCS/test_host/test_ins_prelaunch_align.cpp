/**
 * @file    test_ins_prelaunch_align.cpp
 * @module  固件/宿主测试（fw-tests：起飞前对准稳定性门）
 * @brief   验证 ins_prelaunch_align.h：高零偏静止可判稳，匀速转动/振荡/
 *          过载/角速度阶跃/未满窗不可判稳。
 * @details 200 Hz 窗口 100 帧；用理想比力与常值/振荡/阶跃角速度构造样本。
 * @frame   N/A（纯算法输入输出）
 * @units   m/s²、rad/s、rad
 */
#include "ins_prelaunch_align.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s\n", msg);                                            \
      ++g_fail;                                                                 \
    }                                                                           \
    else                                                                        \
    {                                                                           \
      std::printf("PASS: %s\n", msg);                                            \
    }                                                                           \
  } while (0)

static Eigen::Vector3f Accel(float tilt_rad, float load)
{
  const float g = 9.80665f;
  return Eigen::Vector3f(std::sin(tilt_rad) * g * load, 0.0f,
                         -std::cos(tilt_rad) * g * load);
}

static void Push(InsPreLaunchAlign &s, const Eigen::Vector3f &accel,
                 const Eigen::Vector3f &gyro, int n)
{
  for (int i = 0; i < n; ++i)
  {
    (void)InsPreLaunchAlignUpdate(s, accel, gyro);
  }
}

int main()
{
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    const Eigen::Vector3f gyro(0.010f, -0.006f, 0.012f);  // 1.55 dps 零偏
    Push(s, Accel(0.0f, 1.0f), gyro, 60);
    CHECK(!s.stable, "T1 窗口未满不判稳");
    Push(s, Accel(0.0f, 1.0f), gyro, 60);
    CHECK(s.stable, "T2 1.55 dps 零偏静止判稳");
    CHECK(s.gyro_mean_norm_radps > 0.015f, "T3 零偏均值被保留而非清零");
  }
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    Push(s, Accel(0.0f, 1.0f), Eigen::Vector3f(0.1f, 0.0f, 0.0f), 150);
    CHECK(!s.stable, "T4 0.1 rad/s 匀速转动不判稳");
  }
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    for (int i = 0; i < 150; ++i)
    {
      const float w = 0.04f * std::sin(static_cast<float>(i) * 0.8f);
      (void)InsPreLaunchAlignUpdate(s, Accel(0.0f, 1.0f),
                                    Eigen::Vector3f(w, 0.0f, 0.0f));
    }
    CHECK(!s.stable, "T5 角速度振荡不判稳");
  }
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    Push(s, Accel(0.0f, 2.0f), Eigen::Vector3f(0.01f, 0.0f, 0.01f), 150);
    CHECK(!s.stable, "T6 2 g 过载不判稳");
  }
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    Push(s, Accel(0.0f, 1.0f), Eigen::Vector3f(0.01f, 0.0f, 0.01f), 150);
    CHECK(s.stable, "T7 带零偏水平静置判稳");
    // 恒定倾角仍是静止（比力模长≈g、无转动），不作退出条件；退出应由
    // 转动/过载触发——这里用角速度阶跃填满半个窗口。
    Push(s, Accel(0.0f, 1.0f), Eigen::Vector3f(0.1f, 0.0f, 0.0f), 50);
    CHECK(!s.stable, "T8 静置后角速度阶跃退出判稳");
  }

  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    for (int i = 0; i < 150; ++i)
    {
      const float t = static_cast<float>(i) / 200.0f;
      const float v = std::sin(2.0f * 3.14159265f * 80.0f * t);
      (void)InsPreLaunchAlignUpdate(
          s, Eigen::Vector3f(9.8f * v, 0.0f, -9.8f - 0.5f * 9.8f * v),
          Eigen::Vector3f(0.014f * v, 0.009f * v, 0.014f * v));
    }
    CHECK(s.stable, "T9 1 g 80 Hz 振动：平均矢量判稳");
    CHECK(s.accel_mean_body.norm() > 9.0f && s.accel_mean_body.norm() < 10.1f,
          "T10 振动下平均矢量模长≈g，不被模长平均整流");
  }
  {
    InsPreLaunchAlign s;
    InsPreLaunchAlignReset(s);
    for (int i = 0; i < 150; ++i)
    {
      const float t = static_cast<float>(i) / 200.0f;
      const float v = std::sin(2.0f * 3.14159265f * 120.0f * t);
      (void)InsPreLaunchAlignUpdate(
          s, Eigen::Vector3f(2.0f * 9.8f * v, 0.0f, -9.8f - 9.8f * v),
          Eigen::Vector3f(0.028f * v, 0.018f * v, 0.028f * v));
    }
    std::printf("T11 diag: stab=%d acc=%.2f astd=%.2f gyr=%.3f gstd=%.3f\n",
                s.stable ? 1 : 0, s.accel_mean_norm_mps2, s.accel_std_mps2,
                s.gyro_mean_norm_radps, s.gyro_std_norm_radps);
    CHECK(s.stable, "T11 2 g 120 Hz 振动：平均矢量判稳");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_ins_prelaunch_align: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_ins_prelaunch_align: %d 项失败 ===\n", g_fail);
  return 1;
}
