#pragma once

#include <cmath>
#include <cstdint>

/*
 * GNSS 动态权重 R 矩阵计算 —— 纯函数
 * ===================================
 *
 * 目的：
 * - 把 GNSS 质量门限和动态 R 计算从主程序抽离为平台无关的纯函数，
 *   使 GNSS 融合策略可在主机上用 g++ 独立回归验证，不再依赖嵌入式硬件。
 *
 * 设计决策：
 * - 接收机报告的精度 (h_acc/v_acc/s_acc) 经 clamp(floor, cap) →
 *   pDOP 缩放 → 直接作为 EKF 量测噪声 R 矩阵的标准差。
 * - 与之前“硬门限一刀切”不同，这里不因精度/卫星数不达标而丢弃完整 GNSS 帧，
 *   只做连续衰减；EKF 内部 NIS 门控仍作为最后防线。
 * - v_acc 或 s_acc = 0 时用 h_acc * 1.5 作为回退估计，避免 receiver 未报告
 *   速度/垂直精度时噪声地板误伤。
 *
 * 使用方式：
 *   Icm45686GnssDynamicWeightConfig cfg = {};
 *   cfg.floor_pos_ne_m = 2.0f;
 *   // ... 其余按需求填充 ...
 *   auto result = Icm45686ComputeGnssDynamicWeights(fix, num_sv,
 *                                                    h_acc, v_acc, s_acc, p_dop, cfg);
 *   if (result.passed_minimum) {
 *       nav.gnss_pos_ne_std_m(result.eff_pos_ne_std_m);
 *       // ...
 *   }
 */

struct Icm45686GnssDynamicWeightConfig {
  float floor_pos_ne_m    = 2.0f;   // 水平位置噪声地板 (m)
  float floor_pos_d_m     = 3.0f;   // 垂直位置噪声地板 (m)
  float floor_vel_ne_mps  = 0.15f;  // 水平速度噪声地板 (m/s)
  float floor_vel_d_mps   = 0.25f;  // 垂直速度噪声地板 (m/s)
  float cap_pos_ne_m      = 30.0f;  // 水平位置噪声天花板 (m)
  float cap_pos_d_m       = 50.0f;  // 垂直位置噪声天花板 (m)
  float cap_vel_mps       = 3.0f;   // 速度噪声天花板 (m/s)
  float pdop_ref          = 2.0f;   // pDOP 缩放参考值（良好几何）
  int8_t min_sv           = 5;      // 最低参与融合的卫星数
};

struct Icm45686GnssDynamicWeightResult {
  // ---- 有效 R 矩阵噪声标准差（已 clamp + pDOP 缩放）----
  float eff_pos_ne_std_m  = 0.0f;
  float eff_pos_d_std_m   = 0.0f;
  float eff_vel_ne_std_mps = 0.0f;
  float eff_vel_d_std_mps = 0.0f;

  // ---- 是否通过最小门限（fix >= 3D, sv >= cfg.min_sv, h_acc > 0）----
  bool passed_minimum = false;
};

/*
 * 计算一帧 GNSS 观测对应的 EKF 量测噪声标准差。
 *
 * @param fix     定位类型（FIX_3D=3，低于 3 视为不通过最小门限）。
 * @param num_sv  参与解算的卫星数。
 * @param h_acc   接收机报告的水平精度 (m)。
 * @param v_acc   接收机报告的垂直精度 (m)；≤0 时用 h_acc * 1.5 回退。
 * @param s_acc   接收机报告的速度精度 (m/s)；≤0 时用 h_acc * 1.5 回退。
 * @param p_dop   位置精度因子；≤0 表示接收机未提供，此时 pDOP 缩放因子 = 1.0。
 * @param cfg     噪声地板/天花板/pDOP 参考等配置。
 *
 * @return 各通道有效噪声标准差 + 最小门限结果。
 */
inline Icm45686GnssDynamicWeightResult Icm45686ComputeGnssDynamicWeights(
    const int8_t fix,
    const int8_t num_sv,
    const float h_acc,
    const float v_acc,
    const float s_acc,
    const float p_dop,
    const Icm45686GnssDynamicWeightConfig &cfg)
{
  Icm45686GnssDynamicWeightResult result = {};

  // 最小门限：fix>=3D、卫星数达标、有有效水平精度
  if (fix < 3 || num_sv < cfg.min_sv || h_acc <= 0.0f)
  {
    return result;
  }
  result.passed_minimum = true;

  // v_acc 或 s_acc 为 0 时，用 h_acc * 1.5 作为合理回退估计，
  // 避免接收机未报告垂直/速度精度时噪声地板误伤。
  const float v_acc_eff =
      (v_acc > 0.0f) ? v_acc : h_acc * 1.5f;
  const float s_acc_eff =
      (s_acc > 0.0f) ? s_acc : h_acc * 1.5f;

  // pDOP 缩放：pDOP>参考值时按比例放大 R（几何差时接收机精度估计偏乐观）；
  // pDOP≤参考值时不缩小 R，保持噪声地板约束。
  // pDOP≤0 表示接收机未提供，等同于 pdop_scale=1.0（不缩放）。
  const float pdop_scale =
      (p_dop > cfg.pdop_ref && p_dop > 0.0f) ? (p_dop / cfg.pdop_ref) : 1.0f;

  // clamp(floor, cap) 防止接收机报告异常小/大值时 EKF 过度信任/完全不信任。
  // 然后乘以 pDOP 缩放因子。
  result.eff_pos_ne_std_m =
      pdop_scale *
      (h_acc < cfg.floor_pos_ne_m   ? cfg.floor_pos_ne_m
       : h_acc > cfg.cap_pos_ne_m   ? cfg.cap_pos_ne_m
       : h_acc);
  result.eff_pos_d_std_m =
      pdop_scale *
      (v_acc_eff < cfg.floor_pos_d_m   ? cfg.floor_pos_d_m
       : v_acc_eff > cfg.cap_pos_d_m   ? cfg.cap_pos_d_m
       : v_acc_eff);
  result.eff_vel_ne_std_mps =
      pdop_scale *
      (s_acc_eff < cfg.floor_vel_ne_mps   ? cfg.floor_vel_ne_mps
       : s_acc_eff > cfg.cap_vel_mps      ? cfg.cap_vel_mps
       : s_acc_eff);
  result.eff_vel_d_std_mps =
      pdop_scale *
      (s_acc_eff < cfg.floor_vel_d_mps   ? cfg.floor_vel_d_mps
       : s_acc_eff > cfg.cap_vel_mps     ? cfg.cap_vel_mps
       : s_acc_eff);

  return result;
}
