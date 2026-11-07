/**
 * @file    fw_sil_plant_6dof.h
 * @module  固件/宿主 SIL（高精度 6DOF 刚体 plant）
 * @brief   完整六自由度刚体动力学：RK4 四元数积分 + 欧拉方程 + 可选二次气动阻力
 * @details 状态向量 y = [pn, pe, pd, vn, ve, vd, qw, qx, qy, qz, wx, wy, wz]；
 *          推力沿机体 -z，模型系力矩按 x'=-z_b、y'=+y_b、z'=+x_b 置换；
 *          转动用完整欧拉方程 I·ω̇ = M - ω×(I·ω)，四元数每步保范。
 *          气动阻力 F = -½ρ·CdA·|v|·v，默认 CdA=0（用户要求先无气动），
 *          需要时用 -DBFS_SIL_CDA=0.05 打开。
 * @frame   平动 NED；转动/力矩机体 FRD
 * @units   m、m/s、rad、rad/s、N、N·m、kg
 * @note    定高、失效保护、制导三个场景入口共用本头；env 入口已有带风/地面/
 *          故障注入的独立 RK4 plant，不重复引入。禁止再在场景入口写只含垂向
 *          一阶积分的简化动力学。
 * @see     fw_sil_env_main.cpp、fw_sil_guidance_main.cpp、fw_sil_main.cpp
 */
#ifndef BFS_SIL_PLANT_6DOF_H
#define BFS_SIL_PLANT_6DOF_H

#include <cmath>

#include "Quat4f.h"
#include "QuaternionMath.h"
#include "TandemVec_Config.h"

#ifndef BFS_SIL_CDA
#define BFS_SIL_CDA 0.0f // 等效阻力面积 Cd·A (m²)；0 = 忽略气动
#endif

struct SilBody6Dof
{
  Quat4f q;
  float omega[3];
  float vel_n, vel_e, vel_d;
  float pos_n, pos_e, pos_d;
  float mass_now;

  SilBody6Dof()
      : q(1, 0, 0, 0), omega{0, 0, 0}, vel_n(0), vel_e(0), vel_d(0),
        pos_n(0), pos_e(0), pos_d(0), mass_now(kDefaultTandemVecParams.m) {}

  void deriv(const float *y, float Fx, const float *Mb, float *dy) const
  {
    const float qw = y[6], qx = y[7], qy = y[8], qz = y[9];
    const float wx = y[10], wy = y[11], wz = y[12];
    const float vn = y[3], ve = y[4], vd = y[5];

    dy[0] = vn; dy[1] = ve; dy[2] = vd;

    // 推力沿机体 -z：R 的第三列 = z_b 在 NED 的投影
    const float R31 = 2.0f * (qx * qz + qw * qy);
    const float R32 = 2.0f * (qy * qz - qw * qx);
    const float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    const float sp = sqrtf(vn * vn + ve * ve + vd * vd);
    const float kd = 0.5f * 1.225f * BFS_SIL_CDA;

    dy[3] = (-Fx * R31 - kd * sp * vn) / mass_now;
    dy[4] = (-Fx * R32 - kd * sp * ve) / mass_now;
    dy[5] = (-Fx * R33 - kd * sp * vd) / mass_now + kDefaultTandemVecParams.g;

    // q̇ = ½·q⊗ω
    dy[6] = 0.5f * (-wx * qx - wy * qy - wz * qz);
    dy[7] = 0.5f * (wx * qw + wz * qy - wy * qz);
    dy[8] = 0.5f * (wy * qw - wz * qx + wx * qz);
    dy[9] = 0.5f * (wz * qw + wy * qx - wx * qy);

    // 完整欧拉方程（对角惯量）
    const TandemVecParams &p = kDefaultTandemVecParams;
    dy[10] = (Mb[0] - (p.Iz - p.Iy) * wy * wz) / p.Ix;
    dy[11] = (Mb[1] - (p.Ix - p.Iz) * wz * wx) / p.Iy;
    dy[12] = (Mb[2] - (p.Iy - p.Ix) * wx * wy) / p.Iz;
  }

  void step(const SixDOFWrench &w, float dt)
  {
    // computeWrench 模型系 -> 机体 FRD。
    // ★ 这里的 Mx 分量是 −w.Mz 而不是纯几何的 +w.Mz，**不是笔误**：
    //   2026-11-07 实测把本行换成 {w.Mz, w.My, -w.Mx}（纯几何）后，纵列 SIL
    //   手动阶跃在 t=11 s 打到 −84°、t=12 s −142°、t=15 s 发散成 NaN；
    //   恢复 −w.Mz 后回到 5.86° 稳定。原因是固件 mix 层约定
    //   `Mx' = -Ix·alpha_roll`（模型系 Mx' 带负号），plant 这侧的 −w.Mz
    //   与之配对构成完整的符号链。几何归 FrameConventions（纯旋转），
    //   控制通道符号归 mix 层，这一行是两个语义的合成点，改任一处都会翻转滚转。
    const float Mb[3] = {-w.Mz, w.My, -w.Mx};
    float y[13] = {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d,
                   q.w, q.x, q.y, q.z, omega[0], omega[1], omega[2]};
    float k1[13], k2[13], k3[13], k4[13], tmp[13];

    deriv(y, w.Fx, Mb, k1);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k1[i];
    deriv(tmp, w.Fx, Mb, k2);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + 0.5f * dt * k2[i];
    deriv(tmp, w.Fx, Mb, k3);
    for (int i = 0; i < 13; ++i) tmp[i] = y[i] + dt * k3[i];
    deriv(tmp, w.Fx, Mb, k4);

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

#endif // BFS_SIL_PLANT_6DOF_H