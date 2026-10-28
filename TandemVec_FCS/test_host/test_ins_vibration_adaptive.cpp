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
 * @test    T1 静置不误报；T2 振动检出门限；T3 建议 Q 抬升；T4 恢复后回落；
 *          T5 `InsClampGyroStd` 陀螺侧保守夹持 8 组边界；
 *          T6 特征锁定：0.48 Hz 纯机动（零振动）也会越过 0.15 g 高振动门限
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

  // T5 陀螺侧安全夹持：与加计侧同构的保守上界，防止机动误判把 gyro Q 长期抬高。
  struct ClampCase
  {
    float suggest, cap, want;
    const char *name;
  };
  const ClampCase kClamp[] = {
      {0.0015f, 0.0015f, 0.0015f, "cap=nominal"},
      {0.0034f, 0.0015f, 0.0015f, "2.3x->off"},
      {0.0034f, 0.0035f, 0.0034f, "2.3x within cap"},
      {0.0100f, 0.0035f, 0.0035f, "6.7x->capped"},
      {0.0100f, 0.0100f, 0.0100f, "historic default unchanged"},
      {0.0000f, 0.0035f, 0.0015f, "non-positive falls back"},
      {0.0010f, 0.0035f, 0.0015f, "never below nominal"},
      {0.0034f, 0.0010f, 0.0015f, "cap below nominal clamps to nominal"},
  };
  bool clamp_ok = true;
  for (const auto &c : kClamp)
  {
    const float got = InsClampGyroStd(c.suggest, c.cap);
    if (!(std::fabs(got - c.want) < 1.0e-6f))
    {
      std::printf("clamp FAIL %s: got=%.6f want=%.6f\n", c.name, got, c.want);
      clamp_ok = false;
    }
  }
  std::printf("clamp: %s (%u cases)\n", clamp_ok ? "ok" : "FAIL",
              static_cast<unsigned>(sizeof(kClamp) / sizeof(kClamp[0])));

  // T6 特征锁定（不是合格判据，而是把当前缺陷钉住）：0.5 Hz 高通的截止低于
  // 导弹 27.8 deg/s（约 0.48 Hz）持续机动，纯机动、零振动也会把 accel_rms_g
  // 推到 high_vibration 门限之上；SIL 实测洁净弹道占空比 63-72%。
  // 若将来提高截止或改喂低通前信号，本断言必须被有意改掉。
  InsVibrationEstimator manoeuvre_est;
  constexpr float kManeuverRateRadps = 0.485f;  // 27.8 deg/s
  constexpr float kManeuverAccelMps2 = 6.4f * 9.79f;
  for (int k = 0; k < static_cast<int>(20.0f / dt); ++k)
  {
    const float t = k * dt;
    // 机体系比力矢量以机动频率旋转：直流分量被高通保留、旋转分量进入"振动"带
    Eigen::Vector3f a(kManeuverAccelMps2 * std::cos(kManeuverRateRadps * t),
                      0.0f,
                      kManeuverAccelMps2 * std::sin(kManeuverRateRadps * t));
    manoeuvre_est.Update(a, Eigen::Vector3f::Zero(), dt);
  }
  const auto leak = manoeuvre_est.last();
  std::printf("maneuver-leak: rms=%.3f g high=%d (zero vibration injected)\n",
              leak.accel_rms_g, leak.high_vibration ? 1 : 0);
  const bool leak_locked = leak.accel_rms_g > 0.15f && leak.high_vibration;

  const bool pass =
      std::isfinite(quiet.accel_rms_g) && std::isfinite(vib.accel_rms_g) &&
      quiet.accel_rms_g < 0.05f && !quiet.high_vibration &&
      vib.accel_rms_g > 0.50f && vib.high_vibration &&
      vib.accel_std_suggest > 0.25f &&
      recover.accel_rms_g < 0.05f && !recover.high_vibration &&
      clamp_ok && leak_locked;
  std::printf("[%s] ins_vibration_adaptive\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
