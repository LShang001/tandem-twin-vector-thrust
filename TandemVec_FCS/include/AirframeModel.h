#pragma once

#include <cmath>
#include <cstdint>

/*
 * AirframeModel.h — 机型数据驱动的执行器模型与控制效能矩阵（2026-08-10）
 * =====================================================================
 * 通用层抽象：任何多电机飞行器 = 电机几何表 + 控制输入映射 + 动力学参数。
 *
 * 核心设计（2026-08-10 二次迭代修正）：
 *   执行器（ActuatorDef）= 物理电机（kRotor 固定 / kGimbal 摆座）；
 *   控制输入 u[] = 控制通道（转速/摆角/联动差速），数量可 ≠ 电机数；
 *   ActuatorDef 通过 u_spd/u_angle 索引把电机连接到输入通道。
 *   ——差速 Δω 是两电机的"联动转速输入"（w=w0√(1±Δω)），不是独立执行器，
 *     避免把同一电机建模两次（初版双重计数 bug，T11 等价测试抓出）。
 *
 * 坐标系：模型系（x'=推力轴朝机头、y'=右、z'=上摆轴；纵列双发约定，
 * 与 TandemVec_Propulsion.h computeWrench 一致）。
 *
 * 用法：
 *   - 正向映射 computeWrenchGeneric：u → 模型系六维力/力矩
 *   - 数值中心差分 Jacobian computeJacobianNumeric：∂M/∂u（机型无关，
 *     任意电机/输入映射自动正确——换机型 = 填几何+映射表，不写算法）
 *   - 与解析 computeEffectMatrix 逐元素 1e-3 内等价
 *     （test_tandemvec_allocation T11 回归锁定）
 */

// 电机类型
enum class ActuatorKind : uint8_t
{
  kRotor = 0,   // 固定转子（转速直接来自 u）
  kGimbal = 1,  // 摆座（摆角来自 u_angle；转速来自 u_spd 或恒 w0）
};

// 物理电机：几何 + 输入映射
struct ActuatorDef
{
  ActuatorKind kind;
  float r[3];           // 位置向量（力臂，m）
  float thrust_axis[3]; // 推力基向（未摆时推力方向单位向量）
  float gimbal_axis[3]; // 摆动轴（kGimbal 用）
  float spin;           // 反扭符号（+1/-1，CW/CCW）
  int8_t u_spd;         // 转速控制输入索引：
                        //   kRotor：u[u_spd] 直接为转速（rad/s）
                        //   kGimbal：u_spd<0 → 恒 w0；否则 w=w0√(1+spd_sign·u[u_spd])
  float spd_sign;       // 差速联动符号：+1 → w=w0√(1+u)、-1 → w=w0√(1−u)；kRotor 忽略
  int8_t u_angle;       // 摆角控制输入索引（kGimbal；-1 = 固定 0）
};

// 机型模型 = 电机表（几何+映射；动力学参数 kT/kQ 由调用方传入）
struct AirframeModel
{
  const ActuatorDef *acts;  // 电机几何/映射表
  uint8_t n_acts;           // 电机数量
};

/*
 * 通用正向映射：控制输入 u[] → 模型系六维力/力矩。
 *
 * @param acts  电机表（AirframeModel 内）
 * @param n     电机数量
 * @param u     控制输入（按各电机的 u_spd/u_angle 索引解释）
 * @param w0    差速基准转速（rad/s；恒转速电机/差速联动用）
 * @param kT    推力系数（N·s²）
 * @param kQ    反扭系数（N·m·s²）
 * @param out   输出 {Fx,Fy,Fz, Mx,My,Mz}（模型系）
 */
inline void computeWrenchGeneric(const ActuatorDef *acts, uint8_t n,
                                 const float *u, float w0,
                                 float kT, float kQ, float out[6])
{
  float F[3] = {0.f, 0.f, 0.f};
  float M[3] = {0.f, 0.f, 0.f};
  for (uint8_t i = 0; i < n; ++i)
  {
    const ActuatorDef &a = acts[i];
    // ---- 转速平方 ω² ----
    //   kRotor：u[u_spd] 直接为 ω²（推力 ∝ ω²，mixer 对 ω² 线性）
    //   kGimbal：转速由差速联动 w=w0√(1+spd_sign·u[u_spd])（实际转速，w²）
    float w_sq;
    if (a.kind == ActuatorKind::kRotor)
    {
      w_sq = (a.u_spd >= 0) ? u[a.u_spd] : (w0 * w0);
    }
    else  // kGimbal
    {
      const float w = (a.u_spd < 0)
                          ? w0
                          : w0 * sqrtf(fmaxf(0.f, 1.f + a.spd_sign * u[a.u_spd]));
      w_sq = w * w;
    }
    // ---- 摆角 ----
    const float d = (a.kind == ActuatorKind::kGimbal && a.u_angle >= 0)
                        ? u[a.u_angle]
                        : 0.f;
    // ---- 推力/反扭向量 ----
    const float T = kT * w_sq;
    const float Q = a.spin * kQ * w_sq;
    float tv[3];
    if (a.kind == ActuatorKind::kGimbal && d != 0.f)
    {
      // 绕 gimbal_axis 旋转推力基向（Rodrigues，gimbal_axis 为单位轴）
      const float c = cosf(d), s = sinf(d);
      const float gx = a.gimbal_axis[0], gy = a.gimbal_axis[1], gz = a.gimbal_axis[2];
      const float tx = a.thrust_axis[0], ty = a.thrust_axis[1], tz = a.thrust_axis[2];
      const float dot = gx * tx + gy * ty + gz * tz;
      tv[0] = tx * c + (gy * tz - gz * ty) * s + gx * dot * (1.f - c);
      tv[1] = ty * c + (gz * tx - gx * tz) * s + gy * dot * (1.f - c);
      tv[2] = tz * c + (gx * ty - gy * tx) * s + gz * dot * (1.f - c);
    }
    else
    {
      tv[0] = a.thrust_axis[0];
      tv[1] = a.thrust_axis[1];
      tv[2] = a.thrust_axis[2];
    }
    // ---- 力矩贡献 M = r×F + Q ----
    const float Fv[3] = {T * tv[0], T * tv[1], T * tv[2]};
    const float Qv[3] = {Q * tv[0], Q * tv[1], Q * tv[2]};
    F[0] += Fv[0]; F[1] += Fv[1]; F[2] += Fv[2];
    M[0] += a.r[1] * Fv[2] - a.r[2] * Fv[1] + Qv[0];
    M[1] += a.r[2] * Fv[0] - a.r[0] * Fv[2] + Qv[1];
    M[2] += a.r[0] * Fv[1] - a.r[1] * Fv[0] + Qv[2];
  }
  out[0] = F[0]; out[1] = F[1]; out[2] = F[2];
  out[3] = M[0]; out[4] = M[1]; out[5] = M[2];
}

/*
 * 数值中心差分 Jacobian：B[i][j] = ∂M_i/∂u_j（模型系力矩对控制输入）。
 * 机型无关：任意电机/输入映射自动正确（扰动 u[j] 自动传播到所有
 * 引用它的电机，含差速联动的转速+摆角链式耦合）。
 * 输出行序 [Mx,My,Mz]、列序 = u 数组顺序。
 */
template <uint8_t N>
inline void computeJacobianNumeric(const AirframeModel &model,
                                   const float *u, float w0,
                                   float kT, float kQ, float B[3][N])
{
  const float h = 1e-4f;  // 中心差分相对步长（步长 = h·max(1,|u[j]|)，
                          // 对 ω² 这类大量级输入自适应，避免差值被 float 精度吞没）
  for (uint8_t j = 0; j < N; ++j)
  {
    const float step = h * fmaxf(1.f, fabsf(u[j]));
    float up[N], um[N];
    for (uint8_t k = 0; k < N; ++k) { up[k] = u[k]; um[k] = u[k]; }
    up[j] = u[j] + step;
    um[j] = u[j] - step;
    float wp[6], wm[6];
    computeWrenchGeneric(model.acts, model.n_acts, up, w0, kT, kQ, wp);
    computeWrenchGeneric(model.acts, model.n_acts, um, w0, kT, kQ, wm);
    for (int i = 0; i < 3; ++i)
    {
      B[i][j] = (wp[3 + i] - wm[3 + i]) / (2.f * step);
    }
  }
}
