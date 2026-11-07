/**
 * @file    single_edf_plant.h
 * @module  固件/宿主 SIL（单涵道四燃气舵：6DOF 刚体 plant）
 * @brief   单涵道构型的正向动力学：1 路涵道推力 + 4 片燃气舵力矩，RK4 六自由度积分
 * @details 输入：`SingleEdfActuators`（4 片舵的有向舵角 deg + 推进器油门百分比）；
 *          处理：τ_j = k·Σ_i H[j][i]·δ_i（由 `bfs::kSingleEdf4VaneMatrix` 与
 *          `vane_moment_nm_per_deg` 正向合成三轴力矩）→ 完整欧拉方程 + RK4 四元数积分；
 *          输出：NED 位置/速度、机体四元数与角速率。
 * @theory  刚体平动 a = F/m + g，转动 I·ω̇ = τ − ω×(I·ω)；
 *          力矩合成是 `SingleEdf4VaneAllocator.h` 逆解（δ = Hᵀ·τ/(4k)）的正向对偶，
 *          由 H 的行正交性（每行 4 个 ±1，H·Hᵀ = 4I）保证往返一致。
 * @ref     CTL-008 §3（四舵矩阵与联合限幅）；THY-003 §3（3×4 分配）
 * @frame   平动 NED；转动/力矩机体 FRD。★ 单涵道的 roll 是"绕推力轴"
 *          （源机型语义），本 plant 不做存档系置换，与分配器一致。
 * @units   m、m/s、rad、rad/s、N、N·m、kg、kg·m²、deg、%
 * @note    ★ 这是**控制律能否收敛**的最小闭环证据：无气动、无风、无传感器噪声、
 *          舵机为理想瞬时（可选一阶），不是飞行品质或鲁棒性证据。
 *          参数全部取自 `kSingleEdf4VaneHwParamsDefaults`（MODEL-DEFAULT 未标定）。
 * @see     test_host/fw_sil/fw_sil_singleedf_main.cpp（闭环入口）、
 *          include/SingleEdf4VaneAllocator.h（逆解对偶）、
 *          docs/05-控制与分配/CTL-008-单涵道四燃气舵机型移植-2026-11-05.md
 */
#ifndef BFS_SIL_SINGLE_EDF_PLANT_H
#define BFS_SIL_SINGLE_EDF_PLANT_H

#include <cmath>

#include "FlightCtrlParams.h"
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "SingleEdf4VaneAllocator.h"
#include "TandemVec_Config.h" // kDefaultTandemVecParams.g（重力口径与纵列 plant 一致）

/// 单涵道执行器输入（plant 侧：已由固件输出反解出的物理量）
struct SingleEdfActuators
{
  float vane_deg[bfs::kSingleEdfVaneCount] = {0.0f, 0.0f, 0.0f, 0.0f}; ///< 有向舵角（含 vane_dir）
  float throttle_pct = 0.0f;                                      ///< 推进器油门 0..100
};

struct SilSingleEdfBody
{
  Quat4f q;
  float omega[3];
  float vel_n, vel_e, vel_d;
  float pos_n, pos_e, pos_d;
  float mass_now;

  SilSingleEdfBody()
      : q(1, 0, 0, 0), omega{0, 0, 0}, vel_n(0), vel_e(0), vel_d(0),
        pos_n(0), pos_e(0), pos_d(0),
        mass_now(kSingleEdf4VaneHwParamsDefaults.mass_kg) {}

  /// 由四舵角合成机体力矩 [N·m]。
  /// ★ 正解是分配器逆解（δ_i = dir_i·Σ_j H[j][i]·d_j，d_j = τ_j/(4k)）的对偶：
  ///   对上式两边同乘 H 并用 H·Hᵀ = 4I，得 τ_j = k·Σ_i H[j][i]·(δ_i / dir_i)。
  ///   漏掉 dir_i 会翻转 2、4 号舵的力矩符号 → 姿态正反馈发散（2026-11-07 手动
  ///   阶跃实测 t=11s 起翻滚至 ±180°，定高模式因舵不动而未暴露）。
  static void vaneTorque(const float vane_deg[bfs::kSingleEdfVaneCount], float tau[3])
  {
    const float k = kSingleEdf4VaneHwParamsDefaults.vane_moment_nm_per_deg;
    for (int j = 0; j < 3; ++j)
    {
      float sum = 0.0f;
      for (uint8_t i = 0; i < bfs::kSingleEdfVaneCount; ++i)
      {
        const float dir = (kSingleEdf4VaneHwParamsDefaults.vane_dir[i] >= 0.0f) ? 1.0f : -1.0f;
        sum += bfs::kSingleEdf4VaneMatrix[j][i] * (vane_deg[i] / dir);
      }
      tau[j] = k * sum;
    }
  }

  void deriv(const float *y, float thrust_n, const float tau[3], float *dy) const
  {
    const float qw = y[6], qx = y[7], qy = y[8], qz = y[9];
    const float wx = y[10], wy = y[11], wz = y[12];
    const float vn = y[3], ve = y[4], vd = y[5];

    dy[0] = vn; dy[1] = ve; dy[2] = vd;

    // 推力沿机体 -z：R 的第三列 = z_b 在 NED 的投影
    const float R31 = 2.0f * (qx * qz + qw * qy);
    const float R32 = 2.0f * (qy * qz - qw * qx);
    const float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    dy[3] = -thrust_n * R31 / mass_now;
    dy[4] = -thrust_n * R32 / mass_now;
    // ★ 重力口径与纵列 plant（fw_sil_plant_6dof.h）保持一致，取 TandemVec_Config
    //   的 g；不要在某个 plant 里单独引入 9.80665/9.7938，否则两机型闭环会因
    //   重力口径差产生虚假的配平误差（项目历史上曾把口径差误判为算法缺陷）。
    dy[5] = -thrust_n * R33 / mass_now + kDefaultTandemVecParams.g;

    // q̇ = ½·q⊗ω
    dy[6] = 0.5f * (-wx * qx - wy * qy - wz * qz);
    dy[7] = 0.5f * (wx * qw + wz * qy - wy * qz);
    dy[8] = 0.5f * (wy * qw - wz * qx + wx * qz);
    dy[9] = 0.5f * (wz * qw + wy * qx - wx * qy);

    // 完整欧拉方程（对角惯量）。轴向映射以 CTL-008 §「轴系对照」的
    // **2026-11-06 审计订正段（第 126–130 行）**为准，它明确推翻了同文档更上方
    // 「Mx = −M_yaw_EDF / Mz = +M_roll_EDF」的旧表述：
    //   kSingleEdf4VaneMatrix 的行直接对应本固件存档系——
    //   H[0] roll → Mx（绕 x_b 倾覆轴，惯量 inertia_roll）
    //   H[1] pitch→ My（绕 y_b，惯量 inertia_pitch）
    //   H[2] yaw  → Mz（绕 z_b = 推力轴 = 世界航向，惯量 inertia_yaw）
    // ★ 踩过的坑（2026-11-07）：按旧表述把 roll 映射到 Mz，结果手动阶跃
    //   t=11 s 起绕推力轴自旋发散。矩阵行序才是实物语义，旧文字已废止。
    const SingleEdf4VaneHwParams &p = kSingleEdf4VaneHwParamsDefaults;
    const float Ixx = p.inertia_roll_kgm2;
    const float Iyy = p.inertia_pitch_kgm2;
    const float Izz = p.inertia_yaw_kgm2;
    dy[10] = (tau[0] - (Izz - Iyy) * wy * wz) / Ixx;
    dy[11] = (tau[1] - (Ixx - Izz) * wz * wx) / Iyy;
    dy[12] = (tau[2] - (Iyy - Ixx) * wx * wy) / Izz;
  }

  /// 用固件本拍输出推进一个控制周期
  void step(const SingleEdfActuators &act, float dt)
  {
    const float thrust_n = (kSingleEdf4VaneHwParamsDefaults.max_total_thrust_n *
                            ((act.throttle_pct > 0.0f) ? act.throttle_pct : 0.0f)) /
                           100.0f;
    float tau[3] = {0.0f, 0.0f, 0.0f};
    vaneTorque(act.vane_deg, tau);

    float y[13] = {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d,
                   q.w, q.x, q.y, q.z, omega[0], omega[1], omega[2]};
    float k1[13], k2[13], k3[13], k4[13], tmp[13];

    deriv(y, thrust_n, tau, k1);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k1[i];
    deriv(tmp, thrust_n, tau, k2);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k2[i];
    deriv(tmp, thrust_n, tau, k3);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + dt * k3[i];
    deriv(tmp, thrust_n, tau, k4);

    for (int i = 0; i < 13; ++i)
    {
      y[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
    }

    pos_n = y[0]; pos_e = y[1]; pos_d = y[2];
    vel_n = y[3]; vel_e = y[4]; vel_d = y[5];
    q = qNorm(Quat4f{y[6], y[7], y[8], y[9]});
    omega[0] = y[10]; omega[1] = y[11]; omega[2] = y[12];
  }

  void euler(float &roll, float &pitch, float &yaw) const
  {
    Quaternion qq{q.w, q.x, q.y, q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
};

#endif // BFS_SIL_SINGLE_EDF_PLANT_H
