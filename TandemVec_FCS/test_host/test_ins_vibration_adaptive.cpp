/**
 * @file    test_ins_vibration_adaptive.cpp
 * @module  固件/宿主回归测试（fw-tests：振动自适应过程噪声估计）
 * @brief   验证 20 Hz 单频振动下 RMS 估计抬升、high_vibration 置位与静置恢复
 * @details 被测：`include/ins_vibration_adaptive.h`（`InsVibrationEstimator::Update()`）。
 *          输入：三段各 8 s、dt=5 ms 的加速度序列——静置（仅 -g）、
 *          1 g 幅值 20 Hz 正弦叠加、再回到静置；
 *          处理：逐拍更新高通 RMS 估计；
 *          输出：静置 RMS < 0.05 g 且不置位、振动 RMS > 0.50 g 且置位、
 *          建议 Q > 0.25、恢复后回到静置判据。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_ins_vibration_adaptive.cpp
 * @theory  0.5 Hz 高通 + 0.5 s RMS 的振动量级估计，把机械振动与真实加速度分离；
 *          建议 Q 只作为候选，不直接写 EKF（安全版 Policy C 见 AGENTS.md）。
 *          @ref REF-121 §2.2.1 p.33（Q 的物理含义）
 * @frame   机体系 FRD 比力
 * @units   加速度 m/s²（对外换算 g），频率 Hz，时间 s
 * @note    1 g 正弦的 RMS 理论值约 0.707 g，门限取 0.50 g 留 30% 余量；
 *          静置门限 0.05 g 对应 0.5 Hz 高通的残余泄漏量级。
 * @test    T1 静置不误报；T2 振动检出门限；T3 建议 Q 抬升；T4 恢复后回落
 * @see     TandemVec_FCS/include/ins_vibration_adaptive.h
 */
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include "ins_vibration_adaptive.h"

int main()
{
  constexpr float dt = 0.005f;
  InsVibrationEstimator est;

  auto run = [&](float seconds, float amp_g, bool quiet) {
    for (int k = 0; k < static_cast<int>(seconds / dt); ++k)
    {
      const float t = k * dt;
      const float vib = quiet ? 0.0f
                              : amp_g * 9.79f * std::sin(2.0f * 3.1415926f * 20.0f * t);
      Eigen::Vector3f a(vib, 0.0f, -9.79f);
      Eigen::Vector3f g(0.0f, 0.0f, 0.0f);
      est.Update(a, g, dt);
    }
  };

  run(8.0f, 0.0f, true);
  const auto quiet = est.last();
  std::printf("quiet: rms=%.3f g std=%.3f high=%d\n",
              quiet.accel_rms_g, quiet.accel_std_suggest,
              quiet.high_vibration ? 1 : 0);

  run(8.0f, 1.0f, false);
  const auto vib = est.last();
  std::printf("vib:   rms=%.3f g std=%.3f high=%d\n",
              vib.accel_rms_g, vib.accel_std_suggest,
              vib.high_vibration ? 1 : 0);

  run(8.0f, 0.0f, true);
  const auto recover = est.last();
  std::printf("rec:   rms=%.3f g std=%.3f high=%d\n",
              recover.accel_rms_g, recover.accel_std_suggest,
              recover.high_vibration ? 1 : 0);

  const bool pass =
      std::isfinite(quiet.accel_rms_g) && std::isfinite(vib.accel_rms_g) &&
      quiet.accel_rms_g < 0.05f && !quiet.high_vibration &&
      vib.accel_rms_g > 0.50f && vib.high_vibration &&
      vib.accel_std_suggest > 0.25f &&
      recover.accel_rms_g < 0.05f && !recover.high_vibration;
  std::printf("[%s] ins_vibration_adaptive\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
