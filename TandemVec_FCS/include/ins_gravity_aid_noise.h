/**
 * @file    ins_gravity_aid_noise.h
 * @module  固件/导航（无 GNSS 运动段重力方向量测噪声与门控）
 * @brief   为“运动中把加速度计比力方向直接当重力方向喂 EKF”给出自适应
 *          观测噪声与方向一致性门控，替代经 Madgwick 的 roll/pitch 量测。
 * @details 背景：无 GNSS 且非静止时，固件当前用 AHRS（Madgwick）的 roll/pitch
 *          作姿态量测。Madgwick 自身也是“陀螺积分 + 加速度重力修正”，其输出
 *          已含同一份陀螺积分信息，再作为独立量测喂回 EKF 属于信息重复计数，
 *          新息不白、NIS 统计不成立（2026-11-01 讨论结论）。
 *          本模块提供“直接重力方向量测”的噪声模型：EKF 用量测
 *          z = (f^b−b_a)/|f^b−b_a|，h(x) = C^b_n·(0,0,−1)，仅观测 2 个自由度。
 *
 *          噪声推导（方向观测的几何误差）：
 *            加速度计比力满足 f^b = g^n·C^n_b + a_lin^b；
 *            模长反解线性加速度 a_lin = sqrt(max(0, |f|² − g²))；
 *            方向误差上界 ≈ atan(a_lin/g)，传感器噪声在方向上贡献 σ_a/|f|；
 *            二者独立叠加：σ_dir² = (σ_a_eff² + a_lin²) / |f|²。
 * @theory  静基座/无加速度时 f^b = −C_n^b g^n，方向即重力；有气动/推力时
 *          不再成立，加速度计不能作倾角观测 @ref REF-120 §9.9.3 p.336；
 *          方向量测只观测姿态误差块 @ref REF-120 §9.8.2 (9.8.13) p.317–322；
 *          外部信息失效时不能靠新息“造”观测 @ref REF-121 §8.2.3 p.291。
 * @frame   机体系 FRD；比力 m/s²；方向单位化；噪声 rad（角度标准差）
 * @units   rad、m/s²、g（振动 RMS 以 g 计）
 * @approx  ① 以 a_lin 的一阶几何上界近似真实方向偏差，忽略转弯离心与杆臂；
 *          ② 振动 RMS 按加速度噪声线性并入 σ_a_eff，未区分带宽；
 *          ③ 门控用 EKF 自身预测方向作参考：姿态明显错误时可能误拒，因此
 *            只在“无 GNSS 运动段”启用，并与 AHRS 兜底二选一（模式可切）。
 * @see     `ins_ahrs_gravity_gate.h`（方向门原语）、`navigation_task.cpp`
 *          【B2】姿态量测分支、`ekf_15_state.h::MeasurementUpdateGravityDetailed`
 */

#pragma once

#include <algorithm>
#include <cmath>

// 无 GNSS 运动段姿态辅助模式（运行期全局 `g_att_aid_mode`，SIL 可覆盖）：
//   0 = 仅 AHRS 姿态量测（2026-10 旧行为，保留作 A/B 基线）
//   1 = 仅重力方向直连（去 Madgwick 一跳；2026-11-02 该路径还带“EKF 速度差分”
//       加速度前馈，但前馈是 opt-in、默认关闭，见 BFS_GRAVITY_AID_VEL_FF）
//   2 = 重力方向优先，被 NIS/门控拒绝时回退 AHRS 姿态量测
//   3 = 重力方向直连 + 控制层加速度指令扣比力污染 + 振动段比力低通（新默认：
//       2026-11-01 MC 的失联/风/无位置环条线综合最优）
#ifndef BFS_ATT_AID_MODE_DEFAULT
#define BFS_ATT_AID_MODE_DEFAULT 3U
#endif

// 加速度前馈开关（2026-11-02 的“EKF 速度差分扣比力”，默认 0=关）：
// 速度由同一 IMU 积分得到，用它扣比力属于自参照。2026-11-01 MC（250 runs）
// 实测：无位置环姿态 1.59°→4.26°、失联 10 s 2.26°→2.82°、振动 3.02°→2.93°
// （仅风条线 4.91°→3.53° 有改善），整体劣于 AHRS 且全面劣于 mode 3。
// 保留代码作研究对照，不默认启用。
#ifndef BFS_GRAVITY_AID_VEL_FF
#define BFS_GRAVITY_AID_VEL_FF 0
#endif

// 运动段方向一致性门限：恒定启用（不像 AHRS 门那样只在 VTOL 包线外启用）。
// 依据：低速巡航/悬停的比力方向与重力方向夹角可由气动升力/推力显著改变，
// 2026-10-27 导弹 SIL 中方向门把姿态 RMSE 从 9.80° 压到 0.65–0.91°。
#ifndef BFS_MOTION_GRAVITY_GATE_RAD
#define BFS_MOTION_GRAVITY_GATE_RAD 0.2617994f  // 15°
#endif

// 观测噪声的地板与上限（rad）：地板≈传感器噪声底，上限等价“放弃本拍量测”。
#ifndef BFS_GRAVITY_AID_NOISE_MIN_RAD
#define BFS_GRAVITY_AID_NOISE_MIN_RAD 0.02f   // ≈1.1°
#endif
#ifndef BFS_GRAVITY_AID_NOISE_MAX_RAD
#define BFS_GRAVITY_AID_NOISE_MAX_RAD 0.50f   // ≈28.6°
#endif

/**
 * @brief 运动段重力方向量测的自适应噪声标准差。
 * @param[in] accel_norm_mps2 机体系未扣零偏的比力模长 |f^b| [m/s²]
 * @param[in] vib_rms_g       软件 LPF 之后的加速度 RMS [g]（振动估计器输出）
 * @param[in] gravity_mps2    当地重力 [m/s²]（取 EKF 的 ekf_gravity_mps2，勿硬编码）
 * @param[in] base_sigma_mps2 加速度计标称噪声 [m/s²]（EKF 配置量级，默认 0.25）
 * @return 方向观测噪声标准差 [rad]，已限幅到 [MIN, MAX]
 * @formula a_lin = sqrt(max(0, |f|² − g²))；σ_a_eff = σ_a + 2·vib_rms·g；
 *          σ_dir = clamp( sqrt((σ_a_eff² + a_lin²)) / max(|f|, 1e-3), MIN, MAX )
 * @ref REF-120 §9.9.3 p.336（静基座观测条件）、REF-120 §9.8.2 (9.8.13) p.317–322
 * @test test_host/test_ins_gravity_aid_noise.cpp
 * @note 振动项系数 2.0 是保守取法：振动 RMS 已由软件 LPF 衰减一次，
 *       再按 2 倍计入加速度噪声，避免振动下把比力方向当重力过度信任。
 */
inline float InsGravityAidNoiseRad(float accel_norm_mps2,
                                   float vib_rms_g,
                                   float gravity_mps2,
                                   float base_sigma_mps2 = 0.25f)
{
  const float g = gravity_mps2;
  const float sigma_a = std::max(base_sigma_mps2, 0.0f) +
                        2.0f * std::max(vib_rms_g, 0.0f) * g;
  const float f2 = accel_norm_mps2 * accel_norm_mps2;
  const float a_lin2 = std::max(0.0f, f2 - g * g);
  const float sigma_a2 = sigma_a * sigma_a + a_lin2;
  const float denom = std::max(accel_norm_mps2, 1.0e-3f);
  float sigma_dir = std::sqrt(sigma_a2) / denom;
  if (!std::isfinite(sigma_dir))
  {
    sigma_dir = BFS_GRAVITY_AID_NOISE_MAX_RAD;
  }
  return std::clamp(sigma_dir, BFS_GRAVITY_AID_NOISE_MIN_RAD,
                    BFS_GRAVITY_AID_NOISE_MAX_RAD);
}
