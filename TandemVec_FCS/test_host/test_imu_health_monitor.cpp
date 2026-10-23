/**
 * @file    test_imu_health_monitor.cpp
 * @module  固件/宿主回归测试（fw-tests：IMU 健康监测）
 * @brief   验证非有限/无样本/低样本/冻结/饱和/高振动/零偏跳变状态机与复位
 * @details 被测：`include/ins_imu_health.h`（平台无关）。
 *          输入：合成比力/角速度、样本数、dt、振动估计、解锁/油门；
 *          处理：逐帧 `update()` 并检查状态/标志/计时器；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/eigen/src
 *                test_host/test_imu_health_monitor.cpp -o bin/imuhealth
 * @theory  系统级 FDI：连续超阈值确认，检测与隔离分离
 *          @ref REF-121 §7.3.1 p.255–259；AGENTS.md（IMU NaN/冻结）
 * @frame   N/A（三轴矢量差分，不解释姿态）
 * @units   m/s²、rad/s、s、Hz、g
 * @note    这是逻辑/边界测试；真实 IMU 电气冻结、饱和和安装振动需上板验证。
 *          冻结判据只覆盖“精确重复样本”，不覆盖缓慢变化或带噪声的假数据。
 * @test    T1 正常；T2 非有限；T3 无样本；T4 低样本；T5 冻结（空中）；
 *          T6 地面不误判冻结；T7 高振动+精确重复仍判冻结、T7b 高振动噪声不误判；
 *          T8 饱和；T9 零偏跳变；T10 复位
 * @see     TandemVec_FCS/include/ins_imu_health.h、src/navigation_task.cpp
 */
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>

#include "ins_imu_health.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

static bfs::ImuHealthMonitor makeMonitor()
{
  bfs::ImuHealthConfig cfg;
  cfg.frozen_confirm_s = 0.1f;
  cfg.no_sample_confirm_s = 0.05f;
  cfg.saturation_confirm_s = 0.05f;
  return bfs::ImuHealthMonitor(cfg);
}

int main()
{
  const Eigen::Vector3f accel(0.10f, -0.04f, -9.79f);
  const Eigen::Vector3f gyro(0.003f, -0.002f, 0.004f);

  // T1 正常样本：10 个 2kHz 样本/5ms，振动 0.1g。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 20; ++i)
    {
      const Eigen::Vector3f noisy =
          accel + Eigen::Vector3f(1.0e-3f * std::sin(float(i)),
                                  1.0e-3f * std::cos(float(i)), 0.0f);
      s = m.update(noisy, gyro, 10U, 0.005f, 0.1f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Ok, "T1 正常状态 Ok");
    CHECK(s.flags == 0U, "T1 无故障标志");
    CHECK(m.staticAidsAllowed(), "T1 正常允许静止辅助");
    CHECK(std::fabs(s.sample_rate_hz - 2000.0f) < 1.0f, "T1 样本率 2kHz");
  }

  // T2 非有限值：立即 Failed，且不保留冻结候选。
  {
    auto m = makeMonitor();
    Eigen::Vector3f a = accel;
    a.x() = std::nanf("");
    const auto s = m.update(a, gyro, 10U, 0.005f, 0.1f, true, 50.0f);
    CHECK(s.state == bfs::ImuHealthState::Failed, "T2 NaN 立即 Failed");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::NonFinite)) != 0U,
          "T2 NonFinite 标志");
    CHECK(!m.staticAidsAllowed(), "T2 Failed 禁止静止辅助");
  }

  // T3 无样本：0.05 s 后 Failed/NoSamples。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 20; ++i)
    {
      s = m.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0U,
                   0.005f, 0.0f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Failed, "T3 无样本 Failed");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::NoSamples)) != 0U,
          "T3 NoSamples 标志");
    CHECK(!m.staticAidsAllowed(), "T3 无样本禁止静止辅助");
  }

  // T4 低样本率：一帧 2/10 样本 → Warning，不 Failed。
  {
    auto m = makeMonitor();
    const auto s = m.update(accel, gyro, 2U, 0.005f, 0.1f, true, 50.0f);
    CHECK(s.state == bfs::ImuHealthState::Warning, "T4 低样本 Warning");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::LowSampleRate)) != 0U,
          "T4 LowSampleRate 标志");
  }

  // T5 空中精确冻结：0.1 s 后 Failed/Frozen。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 80; ++i)
    {
      s = m.update(accel, gyro, 10U, 0.005f, 0.0f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Failed, "T5 冻结 Failed");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::Frozen)) != 0U,
          "T5 Frozen 标志");
    CHECK(!m.staticAidsAllowed(), "T5 冻结禁止静止辅助");
  }

  // T6 地面未解锁：同样精确重复样本不应判冻结。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 100; ++i)
    {
      s = m.update(accel, gyro, 10U, 0.005f, 0.0f, false, 0.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Ok, "T6 地面静止不误判冻结");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::Frozen)) == 0U,
          "T6 地面无 Frozen 标志");
  }

  // T7 高振动 + 精确重复样本：冻结证据优先，仍判 Failed/Frozen。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 100; ++i)
    {
      s = m.update(accel, gyro, 10U, 0.005f, 4.0f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Failed, "T7 高振动+冻结 Failed");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::HighVibration)) != 0U,
          "T7 HighVibration 标志");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::Frozen)) != 0U,
          "T7 精确重复仍判 Frozen");
  }

  // T7b 高振动 + 正常噪声：不能误判冻结，只给 HighVibration 警告。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 100; ++i)
    {
      const Eigen::Vector3f noisy =
          accel + Eigen::Vector3f(1.0e-3f * std::sin(float(i)),
                                  1.0e-3f * std::cos(float(i)), 0.0f);
      s = m.update(noisy, gyro, 10U, 0.005f, 4.0f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Warning, "T7b 高振动噪声 Warning");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::Frozen)) == 0U,
          "T7b 有噪声不误判冻结");
  }

  // T8 饱和：±8g 量程外持续 0.05s → Warning/Saturated。
  {
    auto m = makeMonitor();
    bfs::ImuHealthStatus s;
    for (int i = 0; i < 20; ++i)
    {
      s = m.update(Eigen::Vector3f(80.0f, 0.0f, 0.0f), gyro, 10U, 0.005f,
                   0.1f, true, 50.0f);
    }
    CHECK(s.state == bfs::ImuHealthState::Warning, "T8 饱和 Warning");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::Saturated)) != 0U,
          "T8 Saturated 标志");
  }

  // T9 零偏跳变：外部标志映射到 Warning/BiasJump。
  {
    auto m = makeMonitor();
    const auto s = m.update(accel, gyro, 10U, 0.005f, 0.1f, true, 50.0f, true);
    CHECK(s.state == bfs::ImuHealthState::Warning, "T9 零偏跳变 Warning");
    CHECK((s.flags & bfs::imuHealthFlagBit(bfs::ImuHealthFlag::BiasJump)) != 0U,
          "T9 BiasJump 标志");
  }

  // T10 reset 清空状态/计时器，恢复允许静止辅助。
  {
    auto m = makeMonitor();
    m.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0U, 0.005f,
             0.0f, true, 50.0f);
    m.reset();
    const auto s = m.update(accel, gyro, 10U, 0.005f, 0.1f, true, 50.0f);
    CHECK(s.state == bfs::ImuHealthState::Ok, "T10 reset 后 Ok");
    CHECK(m.staticAidsAllowed(), "T10 reset 后允许静止辅助");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_imu_health_monitor: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_imu_health_monitor: %d 项失败 ===\n", g_fail);
  return 1;
}
