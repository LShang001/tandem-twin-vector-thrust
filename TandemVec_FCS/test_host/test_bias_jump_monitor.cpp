/**
 * @file    test_bias_jump_monitor.cpp
 * @module  固件/宿主回归测试（fw-tests：IMU 零偏跳变/缓漂监测）
 * @brief   验证静置噪声下不误报、慢漂能被识别、弱陀螺漂移可检出
 * @details 被测：`include/ins_bias_jump_monitor.h`（`InsBiasJumpMonitor::Update()`）。
 *          输入 A：8000 拍（40 s @200 Hz）零均值高斯噪声，σ_a=0.005 m/s²、σ_g=0.0005 rad/s；
 *          输入 B：8000 拍线性缓漂（加计 0.005 m/s²·s，陀螺 0.01 dps/s）；
 *          处理：累加各类标志的触发次数；
 *          输出：静置四类标志必须全零，缓漂两类至少各触发一次；否则进程返回 1。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_bias_jump_monitor.cpp
 * @theory  零偏监测是 EKF 零偏状态的独立哨兵：跳变判据看短窗均值突变，
 *          缓漂判据看长窗趋势；两者都必须在纯噪声下保持低虚警。
 *          @ref REF-121 §2.2.1 p.33（零偏作为随机游走状态）、@ref AGENTS.md（零偏可靠性数据更正）
 * @frame   机体系 FRD（三轴加计/陀螺零偏）
 * @units   加计 m/s²，陀螺 rad/s（输入按 dps 换算），步长 s
 * @note    随机种子固定为 1（std::mt19937(1)），结果可复现；噪声量级取自
 *          ICM-42688 标称值量级，不是实机标定值。
 * @test    T1 静置虚警计数为零；T2 缓漂检出；T3 弱陀螺漂移检出
 * @see     TandemVec_FCS/include/ins_bias_jump_monitor.h
 */
#include <cstdio>
#include <random>
#include "ins_bias_jump_monitor.h"

int main()
{
  InsBiasJumpMonitor m;
  m.Reset(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
  std::mt19937 rng(1);
  std::normal_distribution<float> n(0.0f, 0.005f);
  int false_accel = 0, false_gyro = 0;
  int false_drift_a = 0, false_drift_g = 0;
  for (int i = 0; i < 8000; ++i)
  {
    const auto r = m.Update(Eigen::Vector3f(n(rng), n(rng), n(rng)),
                            Eigen::Vector3f(n(rng), n(rng), n(rng)) * 0.1f,
                            0.005f);
    false_accel += r.accel_jump ? 1 : 0;
    false_gyro += r.gyro_jump ? 1 : 0;
    false_drift_a += r.accel_slow_drift ? 1 : 0;
    false_drift_g += r.gyro_slow_drift ? 1 : 0;
  }
  std::printf("quiet false accel=%d gyro=%d drift_a=%d drift_g=%d\n",
              false_accel, false_gyro, false_drift_a, false_drift_g);

  InsBiasJumpMonitor d;
  d.Reset(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
  bool drift_accel = false, drift_gyro = false, weak_gyro = false;
  for (int i = 0; i < 8000; ++i)
  {
    const float t = i * 0.005f;
    const auto r = d.Update(Eigen::Vector3f(0.005f * t, 0.0f, 0.0f),
                            Eigen::Vector3f(0.01f * t * 3.14159f / 180.0f, 0.0f, 0.0f),
                            0.005f);
    drift_accel |= r.accel_slow_drift;
    drift_gyro |= r.gyro_slow_drift;
    weak_gyro |= r.gyro_weak_drift;
  }
  std::printf("drift detect accel=%d gyro=%d\n",
              drift_accel ? 1 : 0, drift_gyro ? 1 : 0);
  return (!false_accel && !false_gyro && !false_drift_a && !false_drift_g &&
          drift_accel && weak_gyro) ? 0 : 1;
}
