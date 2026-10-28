/**
 * @file    missile_plant.h
 * @module  固件/宿主研究（fw-tests：6DOF 刚体导弹弹体，L1）
 * @brief   给真实固件导航任务提供高动态导弹真值：平移/转动刚体、推力、
 *          气动力/力矩与姿态自动驾驶；不参与固件控制。
 * @details 状态：NED 位置/速度、机体系→NED 四元数、机体系角速度、质量。
 *          力：body x 推力、阻力/升力/侧力（q̄、α/β 线性气动）；力矩：
 *          气动阻尼/静稳定 + 自动驾驶理想舵力矩（限幅）。自动驾驶跟踪
 *          预设的水平助推-爬升-转弯-俯冲欧拉角剖面，仅用于生成真实轨迹。
 *          机体轴 FRD（x 前/y 右/z 下），NED（x 北/y 东/z 下）。
 * @theory  刚体六自由度方程：
 *          ṗ=v^n，v̇^n=C_b^n F^b/m+g^n，
 *          q̇=½q⊗ω^b，ω̇=I⁻¹(M^b−ω×Iω)；
 *          气动 q̄=½ρV²，α=atan2(w,u)，β=asin(v/V)。
 *          @ref REF-120 §2.3（比力/重力）、§9.2（四元数姿态）；导弹气动为
 *          L1 线性模型，不对应具体型号。
 * @frame   NED / FRD
 * @units   m、m/s、rad、s、N、N·m、kg、kg·m²
 * @note    参数为 MODEL-DEFAULT（120 kg、8 kN、S=0.05 m²），不是实弹数据；
 *          自动驾驶是理想力矩，不含舵机/气动舵面动态。
 */
#ifndef FW_SIL_MISSILE_PLANT_H_
#define FW_SIL_MISSILE_PLANT_H_

#include <algorithm>
#include <cmath>
#include "Quat4f.h"
#include "QuaternionMath.h"

namespace sil {

struct MissileParams {
  float mass_kg = 120.0f;
  float inertia_xx = 2.0f;
  float inertia_yy = 60.0f;
  float inertia_zz = 60.0f;
  float area_m2 = 0.05f;
  float ref_len_m = 0.20f;
  float thrust_n = 5000.0f;
  float burn_s = 10.0f;
  float cd0 = 0.28f;
  float induced_k = 0.55f;
  float cl_alpha = 3.2f;      // 1/rad
  float cy_beta = -2.4f;      // 1/rad（侧力方向）
  float cm_alpha = -0.6f;     // 1/rad（静稳定，负；小静稳定度便于自动驾驶配平）
  float cn_beta = -0.6f;      // 1/rad（航向静稳定，负）
  float cl_p = -0.35f;        // 滚转阻尼
  float cm_q = -3.0f;         // 俯仰阻尼
  float cn_r = -2.5f;         // 偏航阻尼
  float max_ang_acc = 35.0f;  // rad/s² 自动驾驶力矩限幅
  float g = 9.80665f;
};

struct MissileState {
  float pos[3] = {0.0f, 0.0f, -50.0f};  // NED，50 m 起始高度
  float vel[3] = {0.0f, 0.0f, 0.0f};
  Quat4f q{1.0f, 0.0f, 0.0f, 0.0f};    // body→NED
  float omega[3] = {0.0f, 0.0f, 0.0f};
  float mass_kg = 120.0f;
  float force_ned[3] = {0.0f, 0.0f, 0.0f};
  float acc_ned[3] = {0.0f, 0.0f, 0.0f};
  bool impacted = false;
  float impact_t = -1.0f;
  float max_load_g = 0.0f;
  float max_speed_mps = 0.0f;
  float max_alt_m = 0.0f;
  float max_rate_dps = 0.0f;
  float max_qbar_pa = 0.0f;
  float alpha_rad = 0.0f;
  float beta_rad = 0.0f;
  float qbar_pa = 0.0f;
  float thrust_n = 0.0f;
  float time_s = 0.0f;
};

inline Quat4f QuatFromEuler(const float roll, const float pitch, const float yaw) {
  const float cr = std::cos(0.5f * roll), sr = std::sin(0.5f * roll);
  const float cp = std::cos(0.5f * pitch), sp = std::sin(0.5f * pitch);
  const float cy = std::cos(0.5f * yaw), sy = std::sin(0.5f * yaw);
  Quat4f q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return qNorm(q);
}

inline void BodyToNed(const Quat4f& q, const float v[3], float out[3]) {
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  out[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] +
           2 * (x * z + w * y) * v[2];
  out[1] = 2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] +
           2 * (y * z - w * x) * v[2];
  out[2] = 2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] +
           (1 - 2 * (x * x + y * y)) * v[2];
}

inline void NedToBody(const Quat4f& q, const float v[3], float out[3]) {
  const float w = q.w, x = q.x, y = q.y, z = q.z;
  out[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y + w * z) * v[1] +
           2 * (x * z - w * y) * v[2];
  out[1] = 2 * (x * y - w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] +
           2 * (y * z + w * x) * v[2];
  out[2] = 2 * (x * z + w * y) * v[0] + 2 * (y * z - w * x) * v[1] +
           (1 - 2 * (x * x + y * y)) * v[2];
}

inline float WrapPi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

// 水平助推-爬升-转弯-俯冲剖面（欧拉角，ZYX，rad）。
inline void MissileCommand(float t, float* roll, float* pitch, float* yaw) {
  auto smooth = [](float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
  };
  const float d2r = 0.0174532925f;
  if (t < 1.0f) {
    (*roll) = 0.0f; *pitch = 0.0f; *yaw = 0.0f;
  } else if (t < 2.0f) {
    (*roll) = 0.0f; *pitch = 15.0f * d2r * smooth((t - 1.0f) / 1.0f);
    *yaw = 0.0f;
  } else if (t < 6.0f) {
    (*roll) = 0.0f; *pitch = d2r * (15.0f + 10.0f * smooth((t - 2.0f) / 4.0f));
    *yaw = 0.0f;
  } else if (t < 12.0f) {
    (*roll) = 0.0f;
    *pitch = d2r * (25.0f + 10.0f * smooth((t - 6.0f) / 6.0f));
    *yaw = 0.0f;
  } else if (t < 20.0f) {
    (*roll) = 0.0f;
    *pitch = d2r * (35.0f - 20.0f * smooth((t - 12.0f) / 8.0f));
    *yaw = d2r * (0.0f + 85.0f * smooth((t - 12.0f) / 8.0f));
  } else if (t < 28.0f) {
    (*roll) = 0.0f;
    *pitch = d2r * (15.0f + 12.0f * smooth((t - 20.0f) / 8.0f));
    *yaw = d2r * (85.0f - 145.0f * smooth((t - 20.0f) / 8.0f));
  } else if (t < 36.0f) {
    (*roll) = 0.0f;
    *pitch = d2r * (27.0f - 52.0f * smooth((t - 28.0f) / 8.0f));
    *yaw = d2r * (-60.0f);
  } else {
    (*roll) = 0.0f; *pitch = d2r * (-25.0f); *yaw = d2r * (-60.0f);
  }
}

class MissilePlant {
 public:
  explicit MissilePlant(const MissileParams& p = MissileParams()) : p_(p) {
    s_.mass_kg = p_.mass_kg;
  }

  const MissileState& state() const { return s_; }

  // att_est/rate_est 非空时，自动驾驶用外部姿态/角速率估计（导航在环）；
  // 为空时保持原行为（真值闭环），默认路径不受影响。
  void Step(const float dt, const Quat4f* att_est = nullptr,
            const float* rate_est = nullptr) {
    if (s_.impacted) return;
    const float t = t_;
    // 1 s 地面支撑：固件 EKF 在纯重力比力下完成静基座对准；推力未点火。
    if (t < 1.0f) {
      s_.pos[0] = 0.0f; s_.pos[1] = 0.0f; s_.pos[2] = -50.0f;
      s_.vel[0] = s_.vel[1] = s_.vel[2] = 0.0f;
      s_.q = Quat4f(1.0f, 0.0f, 0.0f, 0.0f);
      s_.omega[0] = s_.omega[1] = s_.omega[2] = 0.0f;
      s_.force_ned[0] = 0.0f; s_.force_ned[1] = 0.0f;
      s_.force_ned[2] = -s_.mass_kg * p_.g;
      s_.acc_ned[0] = s_.acc_ned[1] = s_.acc_ned[2] = 0.0f;
      t_ += dt;
      return;
    }
    // 1 s 地面支撑（推力未点火），让固件 EKF 在重力下完成静基座对准。
    const float thrust = (t >= 1.0f && t < 1.0f + p_.burn_s) ? p_.thrust_n : 0.0f;
    if (t >= 1.0f) {
      s_.mass_kg = std::max(0.5f * p_.mass_kg,
                            p_.mass_kg * (1.0f - 0.5f * (t - 1.0f) / p_.burn_s));
    }
    // 自动驾驶期望欧拉角/角速率（中心差分）。
    const float h = 0.0025f;
    float r0, p0, y0, r1, p1, y1, rm, pm, ym;
    MissileCommand(t, &r0, &p0, &y0);
    MissileCommand(t + h, &r1, &p1, &y1);
    MissileCommand(t - h, &rm, &pm, &ym);
    const float rd = WrapPi(r1 - rm) / (2.0f * h);
    const float pd = WrapPi(p1 - pm) / (2.0f * h);
    const float yd = WrapPi(y1 - ym) / (2.0f * h);
    // 期望欧拉角速率 → 机体系角速率（ZYX）。
    const float q_des = pd * std::cos(r0) + yd * std::sin(r0) * std::cos(p0);
    const float r_des = -pd * std::sin(r0) + yd * std::cos(r0) * std::cos(p0);
    const float p_des = rd - yd * std::sin(p0);
    // 当前欧拉角（导航在环时用估计姿态，否则用真值）。
    const Quat4f q_fb = att_est ? *att_est : s_.q;
    float rc, pc, yc;
    Quaternion q_cur{q_fb.w, q_fb.x, q_fb.y, q_fb.z};
    quaternionToEuler(q_cur, rc, pc, yc);
    const float p_fb = rate_est ? rate_est[0] : s_.omega[0];
    const float q_fb_rate = rate_est ? rate_est[1] : s_.omega[1];
    const float r_fb = rate_est ? rate_est[2] : s_.omega[2];
    // 姿态 PD（理想舵力矩，限幅）。
    const float kp = 35.0f, kd = 10.0f;
    float mx = p_.inertia_xx * (kp * WrapPi(r0 - rc) + kd * (p_des - p_fb));
    float my = p_.inertia_yy * (kp * WrapPi(p0 - pc) + kd * (q_des - q_fb_rate));
    float mz = p_.inertia_zz * (kp * WrapPi(y0 - yc) + kd * (r_des - r_fb));
    const float mlim_x = p_.inertia_xx * p_.max_ang_acc;
    const float mlim_y = p_.inertia_yy * p_.max_ang_acc;
    const float mlim_z = p_.inertia_zz * p_.max_ang_acc;
    mx = std::clamp(mx, -mlim_x, mlim_x);
    my = std::clamp(my, -mlim_y, mlim_y);
    mz = std::clamp(mz, -mlim_z, mlim_z);
    // 气动/推力（机体系 FRD）。
    float vb[3];
    NedToBody(s_.q, s_.vel, vb);
    const float V = std::sqrt(vb[0] * vb[0] + vb[1] * vb[1] + vb[2] * vb[2]);
    const float alt = -s_.pos[2];
    const float rho = 1.225f * std::exp(-std::max(0.0f, alt) / 8500.0f);
    const float qbar = 0.5f * rho * V * V;
    const float alpha = (V > 1.0f) ? std::atan2(vb[2], std::max(0.5f, vb[0])) : 0.0f;
    const float beta = (V > 1.0f) ? std::asin(std::clamp(vb[1] / V, -1.0f, 1.0f)) : 0.0f;
    s_.alpha_rad = alpha; s_.beta_rad = beta; s_.qbar_pa = qbar; s_.thrust_n = thrust;
    const float cx = p_.cd0 + p_.induced_k * (alpha * alpha + beta * beta);
    const float fx = thrust - qbar * p_.area_m2 * cx;
    const float fy = qbar * p_.area_m2 * p_.cy_beta * beta;
    const float fz = -qbar * p_.area_m2 * p_.cl_alpha * alpha;
    const float mx_aero = qbar * p_.area_m2 * p_.ref_len_m *
                          p_.cl_p * (s_.omega[0] * p_.ref_len_m / std::max(1.0f, V));
    const float my_aero = qbar * p_.area_m2 * p_.ref_len_m *
                          (p_.cm_alpha * alpha +
                           p_.cm_q * (s_.omega[1] * p_.ref_len_m / std::max(1.0f, V)));
    const float mz_aero = qbar * p_.area_m2 * p_.ref_len_m *
                          (p_.cn_beta * beta +
                           p_.cn_r * (s_.omega[2] * p_.ref_len_m / std::max(1.0f, V)));
    const float Fb[3] = {fx, fy, fz};
    const float Mb[3] = {mx + mx_aero, my + my_aero, mz + mz_aero};
    // 刚体动力学。
    const float p_rate = s_.omega[0], q_rate = s_.omega[1], r_rate = s_.omega[2];
    s_.omega[0] += (Mb[0] - (p_.inertia_zz - p_.inertia_yy) * q_rate * r_rate) /
                   p_.inertia_xx * dt;
    s_.omega[1] += (Mb[1] - (p_.inertia_xx - p_.inertia_zz) * r_rate * p_rate) /
                   p_.inertia_yy * dt;
    s_.omega[2] += (Mb[2] - (p_.inertia_yy - p_.inertia_xx) * p_rate * q_rate) /
                   p_.inertia_zz * dt;
    const float wx = s_.omega[0], wy = s_.omega[1], wz = s_.omega[2];
    Quat4f dq{-0.5f * (s_.q.x * wx + s_.q.y * wy + s_.q.z * wz),
              0.5f * (s_.q.w * wx + s_.q.y * wz - s_.q.z * wy),
              0.5f * (s_.q.w * wy - s_.q.x * wz + s_.q.z * wx),
              0.5f * (s_.q.w * wz + s_.q.x * wy - s_.q.y * wx)};
    s_.q.w += dq.w * dt; s_.q.x += dq.x * dt;
    s_.q.y += dq.y * dt; s_.q.z += dq.z * dt;
    s_.q = qNorm(s_.q);
    float Fn[3];
    BodyToNed(s_.q, Fb, Fn);
    const float m = s_.mass_kg;
    const float ax = Fn[0] / m;
    const float ay = Fn[1] / m;
    const float az = Fn[2] / m + p_.g;
    s_.acc_ned[0] = ax; s_.acc_ned[1] = ay; s_.acc_ned[2] = az;
    s_.force_ned[0] = Fn[0]; s_.force_ned[1] = Fn[1]; s_.force_ned[2] = Fn[2];
    s_.vel[0] += ax * dt; s_.vel[1] += ay * dt; s_.vel[2] += az * dt;
    s_.pos[0] += s_.vel[0] * dt;
    s_.pos[1] += s_.vel[1] * dt;
    s_.pos[2] += s_.vel[2] * dt;
    // 地面/撞击。
    if (t > 1.0f && s_.pos[2] >= 0.0f) {
      s_.pos[2] = 0.0f;
      s_.impacted = true;
      s_.impact_t = t;
      s_.vel[0] = s_.vel[1] = s_.vel[2] = 0.0f;
    }
    // 诊断量（真值过载按比力模长/g，不含重力）。
    const float load = std::sqrt(fx * fx + fy * fy + fz * fz) / (m * p_.g);
    s_.max_load_g = std::max(s_.max_load_g, load);
    s_.max_speed_mps = std::max(s_.max_speed_mps, V);
    s_.max_alt_m = std::max(s_.max_alt_m, alt);
    s_.max_qbar_pa = std::max(s_.max_qbar_pa, qbar);
    const float rate_dps =
        std::sqrt(wx * wx + wy * wy + wz * wz) * 57.29578f;
    s_.max_rate_dps = std::max(s_.max_rate_dps, rate_dps);
    t_ += dt;
    s_.time_s = t_;
  }

 private:
  MissileParams p_;
  MissileState s_;
  float t_ = 0.0f;
};

}  // namespace sil

#endif  // FW_SIL_MISSILE_PLANT_H_
