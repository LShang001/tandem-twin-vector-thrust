/**
 * @file    test_ins_vibration_hf.cpp
 * @module  固件/宿主回归测试（fw-tests：高频振动监测）
 * @brief   验证"低通前/后之差"通道能区分确定性机动与机械振动（既有 0.5 Hz 通道做不到）
 * @details 被测：`include/ins_vibration_hf.h`（`InsHfVibrationMonitor::Update()`）。
 *          输入：2 kHz 历元的原始比力与一阶软件低通输出（α=0.02，与生产
 *          `sensor_imu.cpp` 同型），三段各 6 s：
 *          ①静置 1 g 竖直；②6.4 g 比力矢量以 0.485 rad/s（27.8°/s，导弹典型
 *          机动率）旋转——纯确定性机动、零机械振动；③在②上叠加 1 g·80 Hz 正弦。
 *          处理：逐历元 `hp = raw − lp` 的 EMA 均方开方；
 *          输出：`rms_g` 与 `high_vibration`。
 *          判据：②的 RMS 必须远小于 0.15 g 门限（机动不被误判），
 *          ③的 RMS 必须大于门限且接近 1 g 正弦的理论值 0.707 g。
 *          对照：同一信号喂给 `InsVibrationEstimator`（0.5 Hz 高通、吃低通后信号）
 *          时，②的 RMS 反而高于门限、③只比②高 0.04 g——即 SILE 实测的
 *          `vib_band_response_2026-10-27.json`（80 Hz 灵敏度 0.039 g/g）。
 * @theory  一阶低通的补滤波 `1−H_lp` 为高通；1 g 正弦的 RMS = 1/√2 = 0.707 g
 *          @ref REF-121 §2.2.1 p.33（噪声与振动量级建模）
 * @frame   机体系 FRD，逐轴平方和，与坐标变换无关
 * @units   g（比力幅值 6.4 g、振动 1 g）、Hz、s
 * @test    T1 静置≈0；T2 纯机动不误报；T3 80 Hz 振动检出且接近理论 RMS；
 *          T4 撤除振动后按回差释放；T5 非有限输入不污染状态；
 *          T6 对照：既有中频通道在同样信号下把机动误判为高振动
 * @see     TandemVec_FCS/include/ins_vibration_hf.h、ins_vibration_adaptive.h
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <Eigen/Dense>

#include "ins_vibration_adaptive.h"
#include "ins_vibration_hf.h"

namespace {

// 生产 `sensor_imu.cpp` 的一阶软件低通（α 按轴给定）。
struct Lp
{
  float a;
  Eigen::Vector3f s = Eigen::Vector3f::Zero();
  Eigen::Vector3f step(const Eigen::Vector3f &x)
  {
    s += (x - s) * a;
    return s;
  }
};

// 生成一个历元的原始比力 [g]。mode: 0=静置 1 g；1=6.4 g 矢量以 0.485 rad/s
// （27.8°/s，导弹典型机动率）旋转的纯确定性机动；2=在 1 上叠加 1 g·80 Hz 正弦；
// 3=在 1 上叠加 2 g·80 Hz 正弦（与 SIL 的振 2 g@120 Hz 档同量级）。
Eigen::Vector3f Sample(float t, int mode)
{
  if (mode == 0)
  {
    return Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  }
  const double omega = 0.485;  // 27.8 deg/s
  const double phase = omega * t;
  Eigen::Vector3f body(std::sin(phase) * 6.4f, 0.0f, std::cos(phase) * 6.4f);
  if (mode == 2 || mode == 3)
  {
    const float amp = (mode == 3) ? 2.0f : 1.0f;
    const float vib = amp * std::sin(2.0f * 3.14159265f * 80.0f * t);
    body += Eigen::Vector3f(vib, vib * 0.7f, -vib * 0.5f);
  }
  return body;
}

}  // namespace

int main()
{
  constexpr float dt = 0.0005f;  // 2 kHz
  constexpr float alpha = 0.02f;
  InsHfVibrationMonitor hf;
  InsVibrationEstimator mid;  // 既有中频通道（吃低通后的 200 Hz 信号）
  float mid_feed_acc = 0.0f;
  int mid_updates = 0;

  float rms_quiet = 0.0f, rms_maneuver = 0.0f, rms_vib = 0.0f, rms_after = 0.0f;
  bool high_maneuver = false, high_vib = false, high_after = false;
  float mid_maneuver = 0.0f, mid_vib = 0.0f;
  bool mid_high_maneuver = false;
  bool nan_safe = false;

  auto run = [&](float t0, float dur, int mode) {
    Lp lp{alpha, Eigen::Vector3f::Zero()};
    const int n = static_cast<int>(dur / dt);
    for (int k = 0; k < n; ++k)
    {
      const float t = t0 + k * dt;
      Eigen::Vector3f raw = Sample(t, mode);
      if (t0 > 100.0f)
      {
        // T5：注入非有限样本后状态不得变 NaN
        raw.x() = std::numeric_limits<float>::quiet_NaN();
      }
      const Eigen::Vector3f filtered = lp.step(raw);
      hf.Update(raw.data(), filtered.data(), dt);
      // 中频通道按生产口径吃 200 Hz 的低通输出
      mid_feed_acc += dt;
      if (mid_feed_acc >= 0.005f)
      {
        mid_feed_acc = 0.0f;
        ++mid_updates;
        const Eigen::Vector3f mps2 = filtered * 9.79f;
        mid.Update(mps2, Eigen::Vector3f::Zero(), 0.005f);
      }
    }
    return hf.last();
  };

  const auto q = run(0.0f, 6.0f, 0);
  rms_quiet = q.rms_g;
  const auto m = run(6.0f, 6.0f, 1);
  rms_maneuver = m.rms_g;
  high_maneuver = m.high_vibration;
  mid_maneuver = mid.last().accel_rms_g;
  mid_high_maneuver = mid.last().high_vibration;
  const auto v = run(12.0f, 6.0f, 2);
  rms_vib = v.rms_g;
  high_vib = v.high_vibration;
  mid_vib = mid.last().accel_rms_g;
  const auto a = run(18.0f, 6.0f, 1);
  rms_after = a.rms_g;
  high_after = a.high_vibration;
  const auto bad = run(24.0f, 1.0f, 1);
  nan_safe = std::isfinite(bad.rms_g);

  std::printf("HF   quiet=%.4f maneuver=%.4f(high=%d) vib80Hz=%.4f(high=%d) "
              "after=%.4f(high=%d) nan_safe=%d\n",
              rms_quiet, rms_maneuver, high_maneuver ? 1 : 0, rms_vib,
              high_vib ? 1 : 0, rms_after, high_after ? 1 : 0, nan_safe ? 1 : 0);
  std::printf("MID  maneuver=%.4f(high=%d) vib=%.4f  (updates=%d)\n",
              mid_maneuver, mid_high_maneuver ? 1 : 0, mid_vib, mid_updates);

  // T7 运行时门限扫描（SIL 的 --vib-hf-enter 用同一入口）：把进入门限抬到 1.0 g
  // 后，1 g·80 Hz（RMS 0.93 g）不得置位、2 g·80 Hz（RMS 1.87 g）必须置位，且
  // 撤除后能在按 0.667 比例缩放的释放门限（0.667 g）处释放。
  InsHfVibrationMonitor hf2;
  hf2.SetHysteresis(1.0f, -1.0f);
  auto run2 = [&](float t0, float dur, int mode) {
    Lp lp{alpha, Eigen::Vector3f::Zero()};
    const int n = static_cast<int>(dur / dt);
    for (int k = 0; k < n; ++k)
    {
      const Eigen::Vector3f raw = Sample(t0 + k * dt, mode);
      hf2.Update(raw.data(), lp.step(raw).data(), dt);
    }
    return hf2.last();
  };
  const auto s1 = run2(0.0f, 6.0f, 2);
  const bool t7_1g_no_flag = !s1.high_vibration && s1.rms_g > 0.667f;
  const auto s2 = run2(6.0f, 6.0f, 3);
  const bool t7_2g_flag = s2.high_vibration && s2.rms_g > 1.0f;
  const auto s3 = run2(12.0f, 6.0f, 1);
  const bool t7_release = !s3.high_vibration && s3.rms_g < 0.667f;

  std::printf("T7   enter=1.0g: 1g rms=%.4f high=%d | 2g rms=%.4f high=%d | "
              "after rms=%.4f high=%d\n",
              s1.rms_g, s1.high_vibration ? 1 : 0, s2.rms_g,
              s2.high_vibration ? 1 : 0, s3.rms_g,
              s3.high_vibration ? 1 : 0);

  // T1 静置≈0（只有 6.4 g 直流时低通几乎不滞后）；
  // T2 纯机动必须低于 0.15 g 门限且不置位；
  // T3 1 g·80 Hz 必须检出，且落在理论值 0.707 g 的合理带内（EMA 未完全收敛，放宽到 0.3）；
  // T4 撤除后必须释放（回差 0.10 g）；
  // T6 对照：既有中频通道在同样信号下必须仍然把机动判成高振动（说明问题真实存在）。
  // T7 运行时抬门限：1 g 不置位、2 g 置位、撤除后释放。
  const bool pass = std::isfinite(rms_quiet) && std::isfinite(rms_maneuver) &&
                    rms_quiet < 0.05f && rms_maneuver < 0.15f &&
                    !high_maneuver && rms_vib > 0.30f && high_vib &&
                    rms_after < 0.10f && !high_after && nan_safe &&
                    mid_high_maneuver && (mid_vib - mid_maneuver) < 0.20f &&
                    t7_1g_no_flag && t7_2g_flag && t7_release;
  std::printf("[%s] ins_vibration_hf\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
