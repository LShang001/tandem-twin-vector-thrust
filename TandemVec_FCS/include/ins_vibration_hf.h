/**
 * @file    ins_vibration_hf.h
 * @module  固件/导航（高频振动监测）
 * @brief   用软件低通"之前/之后"的差作为高频带通，在线估计机械振动 RMS
 * @details 输入：同一 2 kHz 历元的原始比力与软件低通输出（两者在 `sensor_imu.cpp`
 *          与导弹 SIL 中都已存在，本模块只做差与 EMA，不新增滤波级）；
 *          处理：`hp = raw − lp` ⇒ 保留带内能量、去掉重力/慢机动/推力台阶等 DC 与
 *          低频分量；对三轴平方和做时间常数 `kRmsTau` 的 EMA，开方得 RMS；
 *          输出：`InsHfVibration`（RMS 以 g 表示 + 回差标志）。
 * @theory  一阶低通的补滤波器 `1 − H_lp(s)` 是高通：本模块的可用频段由低通截止
 *          （生产 α=0.02/0.05 @2 kHz ⇒ fc≈6/16 Hz）到 2 kHz/2 决定，正好覆盖
 *          弹载 80–120 Hz 主谱；而 `ins_vibration_adaptive.h` 的 0.5 Hz 高通吃在
 *          低通**之后**，实测对该主谱只有 2–8% 的灵敏度（`vib_band_response`），
 *          且被确定性机动顶满 ⇒ 无法作为振动判据。
 * @frame   机体系 FRD（与低通输出同系；本模块只做逐轴平方和，坐标变换无关）
 * @units   输入 g，输出 g（1 g = 9.79 m/s² 仅用于显示换算，内部保持 g）
 * @note    回差 0.15 g 进 / 0.10 g 出与 `ins_vibration_adaptive.h` 一致，便于两路
 *          判据互换而不改变上层阈值语义；`kRmsTau=0.5 s` 与既有实现同量级。
 * @see     `test_host/test_ins_vibration_hf.cpp`、`ins_vibration_adaptive.h`
 */

#pragma once

#include <algorithm>
#include <cmath>

// 高频振动"高"判据的进入/释放门限 [g]，与既有中频判据保持一致。
#ifndef BFS_VIB_HF_ENTER_G
#define BFS_VIB_HF_ENTER_G 0.15f
#endif
#ifndef BFS_VIB_HF_RELEASE_G
#define BFS_VIB_HF_RELEASE_G 0.10f
#endif

// `gnc_tel.vib_high` 取哪个通道：0=既有 0.5 Hz 中频 RMS（历史行为），
// 1=本模块的高频通道（软件低通前/后之差）。
#ifndef BFS_VIB_HIGH_SOURCE_HF
#define BFS_VIB_HIGH_SOURCE_HF 0
#endif

struct InsHfVibration
{
  float rms_g = 0.0f;       ///< 高通残差 RMS（三轴合成）[g]
  bool high_vibration = false;
  bool valid = false;       ///< 至少收到过一个有效历元
};

class InsHfVibrationMonitor
{
public:
  void Reset()
  {
    ms_ = 0.0f;
    out_ = InsHfVibration{};
  }

  /**
   * @brief 覆盖回差门限（宏值是默认，宿主 A/B 用运行时值扫阈值）
   * @details 释放门限按 `kReleaseRatio` 随进入门限缩放，保持迟滞相对宽度不变，
   *          避免"提高进入门限但释放门限留在 0.10 g"造成进了就出不来。
   */
  void SetHysteresis(float enter_g, float release_g)
  {
    if (enter_g > 0.0f && std::isfinite(enter_g))
    {
      enter_g_ = enter_g;
    }
    const float rel = (release_g > 0.0f && std::isfinite(release_g))
                          ? release_g
                          : enter_g_ * kReleaseRatio;
    release_g_ = std::min(rel, enter_g_);
    out_.high_vibration = out_.rms_g > release_g_;
  }

  /**
   * @brief 单个 2 kHz 历元更新
   * @param raw_g   软件低通**之前**的三轴比力 [g]
   * @param lp_g    软件低通**之后**的三轴比力 [g]（同一历元）
   * @param dt_s    历元间隔 [s]
   */
  void Update(const float raw_g[3], const float lp_g[3], float dt_s)
  {
    if (!(dt_s > 1.0e-6f) || dt_s > 0.05f)
    {
      return;
    }
    float hp2 = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
      if (!std::isfinite(raw_g[i]) || !std::isfinite(lp_g[i]))
      {
        return;
      }
      const float hp = raw_g[i] - lp_g[i];
      hp2 += hp * hp;
    }
    // EMA 均方：与 `InsVibrationEstimator` 同形式，时间常数 0.5 s。
    constexpr float kRmsTau = 0.5f;
    const float b = dt_s / (kRmsTau + dt_s);
    ms_ += (hp2 - ms_) * b;
    ms_ = std::max(0.0f, ms_);
    out_.rms_g = std::sqrt(ms_);
    out_.valid = true;
    if (!out_.high_vibration && out_.rms_g > enter_g_)
    {
      out_.high_vibration = true;
    }
    else if (out_.high_vibration && out_.rms_g < release_g_)
    {
      out_.high_vibration = false;
    }
  }

  const InsHfVibration &last() const { return out_; }

private:
  // 迟滞相对宽度：默认 0.10/0.15，扫进入门限时按比例缩放释放门限。
  static constexpr float kReleaseRatio =
      BFS_VIB_HF_RELEASE_G / BFS_VIB_HF_ENTER_G;
  float enter_g_ = BFS_VIB_HF_ENTER_G;
  float release_g_ = BFS_VIB_HF_RELEASE_G;
  float ms_ = 0.0f;
  InsHfVibration out_{};
};

// 全局单例：生产由 `sensor_imu.cpp` 在 2 kHz 历元上更新，宿主 SIL 在它复刻
// 软件低通的地方更新，导航任务只读。定义在 `navigation_task.cpp`（两边都链接）。
extern InsHfVibrationMonitor g_ins_hf_vibration;
