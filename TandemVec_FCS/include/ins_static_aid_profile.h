/**
 * @file    ins_static_aid_profile.h
 * @module  固件/导航（静止辅助强度调度）
 * @brief   把“静止置信度/驻留时长”映射为 ZUPT/重力/静止陀螺的量测噪声与动作选择
 * @details 输入：静止置信度、确认帧数、协方差健康、各辅助到期标志、外部速度参照；
 *          处理：三档强度（弱约束→稳态→长静止升级）→ 分频动作仲裁（同帧优先
 *          ZUPT）→ ZUPT 残差自适应放大 → 无外部参照时噪声封底/停用；
 *          输出：`Icm45686StaticAidProfile` 与动作枚举。
 *          纯 constexpr/inline，无平台依赖，宿主 g++ 可测。
 * @theory  零速修正（ZUPT）把静止时的速度真值 0 作为量测；
 *          间接法滤波的阻尼必须来自其它导航系统的外来信息，不能用自身信息
 *          @ref REF-121 §8.2.3 p.291；
 *          系统级 FDI 的门限/χ² 思想 @ref REF-121 §7.3.1 p.255
 * @frame   机体系/NED 速度模长（标量门限），本文件不涉及矩阵运算
 * @units   速度 m/s，角度 rad，帧数无量纲（200 Hz 下 1 帧 = 5 ms）
 * @note    ZUPT 残差 >1.5 m/s 直接跳过（已不是“刚停稳残速”）；
 *          无外部参照 >0.75 m/s 立即拒绝、3 s 噪声封顶 0.30 m/s、30 s 停用。
 *          分频相位须避免 ZUPT/Gravity 公共解长期互相抢占（见 `*CanCollide`）。
 * @see     `ins_static_detector.h`、`handleNavigationSystem()` 静止辅助段
 */

#pragma once

#include <stdint.h>

/*
 * 静止辅助强度调度纯函数
 * ======================
 *
 * 目的：
 * - 刚进入“确认静止”时，不要立刻给 EKF 施加强零速/重力方向约束，避免误判静止时把状态硬拉偏。
 * - 连续静止越久、静止置信度越高、协方差越健康，就可以逐步收紧量测噪声，让桌面静止零偏更快收敛。
 *
 * 该文件只保留与平台无关的纯调度逻辑，便于宿主机直接编译小测试验证。
 */

struct Icm45686StaticAidProfile {
  float gravity_noise_rad;   // 重力方向量测噪声，越小表示越信任当前重力方向。
  float vel_ne_std_mps;      // ZUPT 水平速度观测标准差。
  float vel_d_std_mps;       // ZUPT 垂直速度观测标准差。
  bool prefer_gravity_first; // 刚进入静止恢复窗口时，是否优先尝试重力方向辅助。
};

enum class Icm45686StaticAidAction : uint8_t {
  None = 0,
  Zupt,
  Gravity,
  StaticGyro,
};

struct Icm45686ZuptNoiseSelection {
  float vel_ne_std_mps;
  float vel_d_std_mps;
  float noise_scale;
  bool allow_fusion;
};

constexpr uint32_t Icm45686NormalizeStaticAidDivider(uint32_t divider)
{
  // divider=0 在主程序中按“每帧都允许”处理；这里统一归一化，避免后续取模除零。
  return (divider == 0U) ? 1U : divider;
}

constexpr uint32_t Icm45686StaticAidGcd(uint32_t a, uint32_t b)
{
  // 欧几里得算法用于判断两个周期相位是否会在某个导航帧同时到期。
  while (b != 0U) {
    const uint32_t r = a % b;
    a = b;
    b = r;
  }
  return (a == 0U) ? 1U : a;
}

constexpr bool Icm45686StaticAidPhasesCanCollide(uint32_t zupt_divider,
                                                 uint32_t zupt_phase,
                                                 uint32_t gravity_divider,
                                                 uint32_t gravity_phase)
{
  /*
   * ZUPT 和 Gravity 在主程序里同帧只执行一个，且优先 ZUPT。若两个周期相位按
   * 中国剩余定理存在公共解，Gravity 就会在每次到期时被 ZUPT 抢占，严重时长期
   * gravity_fuse=0。该纯函数用于主机测试和编译期断言，提前捕获这种配置错误。
   */
  const uint32_t z_div = Icm45686NormalizeStaticAidDivider(zupt_divider);
  const uint32_t g_div = Icm45686NormalizeStaticAidDivider(gravity_divider);
  const uint32_t z_phase = zupt_phase % z_div;
  const uint32_t g_phase = gravity_phase % g_div;
  const uint32_t phase_delta =
      (z_phase >= g_phase) ? (z_phase - g_phase) : (g_phase - z_phase);
  return (phase_delta % Icm45686StaticAidGcd(z_div, g_div)) == 0U;
}

constexpr float Icm45686Clamp01(float value)
{
  // 静止置信度只允许在 [0, 1] 内参与档位判断，异常输入按边界处理。
  return (value <= 0.0f) ? 0.0f : ((value >= 1.0f) ? 1.0f : value);
}

constexpr Icm45686StaticAidAction Icm45686SelectStaticAidAction(
    bool zupt_due,
    bool gravity_due,
    bool static_gyro_due,
    bool prefer_gravity_first)
{
  // 档位 1 先调平再零速，避免姿态误差通过 ZUPT 残差污染加速度零偏。
  // ★ 2026-08-12 审查修复：原实现 gravity 未到期时直接返回 None，
  // 会连 ZUPT/StaticGyro 一起饿死（弱约束窗口内约 87.5% 帧无辅助动作）。
  // 改为 gravity 到期时优先 Gravity，未到期时正常执行 ZUPT/StaticGyro。
  if (prefer_gravity_first) {
    if (gravity_due) {
      return Icm45686StaticAidAction::Gravity;
    }
    if (zupt_due) {
      return Icm45686StaticAidAction::Zupt;
    }
    if (static_gyro_due) {
      return Icm45686StaticAidAction::StaticGyro;
    }
    return Icm45686StaticAidAction::None;
  }
  if (zupt_due) {
    return Icm45686StaticAidAction::Zupt;
  }
  if (gravity_due) {
    return Icm45686StaticAidAction::Gravity;
  }
  if (static_gyro_due) {
    return Icm45686StaticAidAction::StaticGyro;
  }
  return Icm45686StaticAidAction::None;
}

constexpr Icm45686ZuptNoiseSelection Icm45686AdaptZuptNoiseForResidual(
    Icm45686StaticAidProfile profile,
    float speed_residual_mps)
{
  /*
   * ZUPT 残差接近静止假设边界时采用软融合：放大量测噪声而不是立刻强拉速度。
   * residual > 1.5m/s 通常已不是“刚停稳残速”，而是静止误判或状态明显异常，跳过速度量测。
   */
  if (!(speed_residual_mps > 0.0f)) {
    speed_residual_mps = 0.0f;
  }
  if (speed_residual_mps > 1.5f) {
    return {profile.vel_ne_std_mps, profile.vel_d_std_mps, 1.0f, false};
  }
  const float scale =
      (speed_residual_mps <= 0.20f)
          ? 1.0f
          : ((speed_residual_mps <= 0.60f)
                 ? 1.0f + (speed_residual_mps - 0.20f) * 5.0f
                 : 3.0f + (speed_residual_mps - 0.60f) * 5.0f);
  const float capped_scale = (scale > 6.0f) ? 6.0f : scale;
  return {profile.vel_ne_std_mps * capped_scale,
          profile.vel_d_std_mps * capped_scale,
          capped_scale,
          true};
}

constexpr Icm45686StaticAidProfile Icm45686SelectStaticAidProfile(
    float static_confidence,
    uint32_t confirmed_static_frames,
    bool covariance_healthy)
{
  // 先做置信度限幅，避免上层计算抖动或未初始化值导致错误进入强约束档位。
  const float confidence = Icm45686Clamp01(static_confidence);

  /*
   * 档位 1：刚确认静止，弱约束窗口。
   * ★ 2026-08-12 审查修复：confirmed_static_frames 从确认当帧的
   * enter_min_frames（固件配置 20）起计，原帧门槛 16 恒不成立导致弱约束窗口
   * 实际只有 1 帧（5ms），与"确认后 80ms 弱约束"注释意图不符。改为
   * < 36（= 20 + 16，即确认后 16 帧 ≈ 80ms）。
   */
  // 档位 1：刚确认静止，帧门槛 32→16（80ms），减少弱约束窗口。
  if (confidence < 0.35f || confirmed_static_frames < 36U) {
    return {0.18f, 0.18f, 0.24f, true};
  }

  /*
   * 档位 2：已经稳定静止，但还没到长静止稳态
   * - 帧门槛 200→100（500ms）：dwell_score=1.0 在 60 帧时已满足，
   *   把帧门槛同步收紧以匹配更快的置信度爬升。
   */
  if (confidence < 0.80f || confirmed_static_frames < 100U ||
      !covariance_healthy) {
    return {0.10f, 0.10f, 0.14f, false};
  }

  /*
   * 档位 3：高置信度长静止稳态 — vel_ne_std 收紧到 0.02m/s, 静止收敛稳态下几乎锁死速度态
   */
  return {0.03f, 0.02f, 0.04f, false};
}

/*
 * ZUPT 外部参照门控（2026-10 P0-3）
 * =================================
 * 理论依据：
 *   秦永元、张洪钺、汪叔华《卡尔曼滤波与组合导航原理（第3版）》，
 *   西北工业大学出版社 2015，§8.2.3 p.291：间接法滤波中，要实现对某一导航
 *   系统的阻尼，必须利用来自其他导航系统的外来信息，而不能利用自身信息作为
 *   阻尼；§7.3.1 p.255：系统级 FDI 用 χ² 检验。
 * 工程含义：ZUPT 的残差/速度来自 EKF 自身，失联时没有外部参照可推翻"静止"
 * 假设；因此用 GNSS/光流的新鲜度作为结构门控，而不是继续调 NIS。
 *   - 有 GNSS/光流：完全放行（不改变原行为）；
 *   - 无外部参照且 |v_ekf|>0.75 m/s：立即拒绝 ZUPT（不等 3 s，防速度先被拉零）；
 *   - 无外部参照 ≥3 s：ZUPT 噪声封顶 0.30 m/s；
 *   - 无外部参照 ≥30 s：完全停用 ZUPT（保留 Gravity/StaticGyro）。
 * 0.75 m/s 是"真静止残速"与"慢速巡航"的折中；真静止时速度估计应接近 0。
 */
struct Icm45686ZuptExtRefGuard {
  float no_ext_ref_s = 0.0f;
  bool cap_noise = false;
  bool disable_zupt = false;
};

constexpr float Icm45686ZuptExtRefCapS = 3.0f;
constexpr float Icm45686ZuptExtRefDisableS = 30.0f;
constexpr float Icm45686ZuptExtRefNoiseFloorMps = 0.30f;
constexpr float Icm45686ZuptExtRefSpeedVetoMps = 0.75f;

inline Icm45686ZuptExtRefGuard Icm45686UpdateZuptExtRefGuard(
    Icm45686ZuptExtRefGuard guard,
    bool external_velocity_reference,
    float dt_s)
{
  if (external_velocity_reference) {
    guard.no_ext_ref_s = 0.0f;
    guard.cap_noise = false;
    guard.disable_zupt = false;
    return guard;
  }
  if (dt_s > 0.0f) {
    guard.no_ext_ref_s += dt_s;
  }
  guard.cap_noise = (guard.no_ext_ref_s >= Icm45686ZuptExtRefCapS);
  guard.disable_zupt = (guard.no_ext_ref_s >= Icm45686ZuptExtRefDisableS);
  return guard;
}

inline bool Icm45686ZuptExternalRefAllowsFusion(
    const Icm45686ZuptExtRefGuard &guard,
    float speed_mps)
{
  if (guard.disable_zupt) {
    return false;
  }
  if (guard.no_ext_ref_s > 0.0f &&
      speed_mps > Icm45686ZuptExtRefSpeedVetoMps) {
    // 无外部参照且估计仍在明显运动：立即拒绝，避免 ZUPT 先把 1 m/s 拉零。
    return false;
  }
  return true;
}

inline void Icm45686ApplyZuptExtRefNoiseFloor(
    Icm45686StaticAidProfile &profile,
    const Icm45686ZuptExtRefGuard &guard)
{
  if (!guard.cap_noise) {
    return;
  }
  if (profile.vel_ne_std_mps < Icm45686ZuptExtRefNoiseFloorMps) {
    profile.vel_ne_std_mps = Icm45686ZuptExtRefNoiseFloorMps;
  }
  if (profile.vel_d_std_mps < Icm45686ZuptExtRefNoiseFloorMps) {
    profile.vel_d_std_mps = Icm45686ZuptExtRefNoiseFloorMps;
  }
}
