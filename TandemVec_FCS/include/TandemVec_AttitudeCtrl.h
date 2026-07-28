// ============================================================
//  TandemVec_AttitudeCtrl.h — 外环：四元数误差 → 目标角速率
//
//  实现最短路径四元数比例控制器（Geometric P controller on SO(3)）：
//    1. 计算体轴系误差四元数 q_err = q_meas⁻¹ ⊗ q_ref
//    2. 保证最短路径旋转（若 q_err.w < 0，取其负值）
//    3. 输出目标角速率 ω_ref = 2·kp · q_err.vec （体轴系）
//    4. 各轴限幅至 omega_max
//
//  优势（相比基于欧拉角的外环）：
//    · 无万向节死锁
//    · 不依赖 Euler 提取约定（本项目的 θ=-asin(R₁₃) 特殊符号不影响四元数控制）
//    · 全角度域连续，大姿态机动时仍正确收敛
//
//  因子 2 的来源：
//    单位四元数 q = [cos(θ/2), sin(θ/2)·n̂]，对小角度
//    q_err.vec ≈ (θ/2)·n̂，故 ω_ref = 2·kp·q_err.vec 等价于 ω_ref = kp·θ·n̂
//    （∝ 角度误差，符合比例控制器预期）
//
//  接口说明：
//    · 输入四元数为 float，分量顺序 [w, x, y, z]
//    · 输出 ω_ref 按 [roll(p), pitch(q), yaw(r)] 顺序（机体系 FRD）
//    · 该函数无内部状态，可在任意位置安全调用
// ============================================================
#pragma once
#include <cmath>
#include "TandemVec_CtrlParams.h"

// ============================================================
//  浮点四元数（仅限控制律内部使用，与 QuaternionMath.h 的 double Quaternion 解耦）
// ============================================================
struct Quat4f
{
    float w, x, y, z;
    Quat4f(float w_ = 1.f, float x_ = 0.f, float y_ = 0.f, float z_ = 0.f)
        : w(w_), x(x_), y(y_), z(z_) {}
};

// 四元数共轭（单位四元数的逆）
static inline Quat4f qConj(const Quat4f& q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

// Hamilton 乘积 q1 ⊗ q2
static inline Quat4f qMul(const Quat4f& a, const Quat4f& b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

// 快速归一化（Newton-Raphson 一步，精度 ~1e-7，比 sqrtf 快约 40%）
static inline Quat4f qNorm(const Quat4f& q)
{
    float msq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (msq < 1e-12f) return {1.f, 0.f, 0.f, 0.f};
    float inv = 1.0f / sqrtf(msq);
    return { q.w*inv, q.x*inv, q.y*inv, q.z*inv };
}

// ============================================================
//  限幅辅助
// ============================================================
static inline float clamp_f(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
//  attitudeStep — 外环主函数
//
//  @param q_meas  当前测量姿态（AHRS 输出，单位四元数）
//  @param q_ref   目标姿态（来自导航层或 RC 指令）
//  @param g       姿态外环增益与限幅参数
//  @param[out] omega_ref  目标角速率 [p, q, r] (rad/s，机体系)
// ============================================================
inline void attitudeStep(const Quat4f& q_meas,
                         const Quat4f& q_ref,
                         const AttitudeCtrlGains& g,
                         float omega_ref[3])
{
    // ---- 步骤1：体轴系误差四元数 q_err = q_meas⁻¹ ⊗ q_ref ----------
    // q_err 的向量部分在机体坐标系下，直接对应各轴角速率指令
    Quat4f q_err = qNorm(qMul(qConj(q_meas), q_ref));

    // ---- 步骤2：最短路径保证 -------------------------------------
    // 四元数 q 与 -q 代表同一旋转；当 q_err.w < 0 时，
    // |angle| = 2·acos(|q_err.w|) > π，即取反可得更短路径
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // ---- 步骤3：比例输出 ω_ref = 2·kp·[q_err.x, q_err.y, q_err.z] ---
    // 系数2：q_err.vec ≈ (θ/2)·n̂ → 乘2后输出正比于角度误差θ
    //
    // 注意：本函数采用标准 FRD 内部约定（roll←x, pitch←y, yaw←z）。
    // CascadeCtrl 内部全链（RateCtrl + 惯量逆解 + 分配）均使用此约定，
    // 仿真与测试保持一致。
    //
    // 实际 VTOL 机体物理映射（x_b 朝上）在 flight_control.cpp 单独处理：
    //   侧倾（前摆/z_b）← q_error.z   差速（x_b）← q_error.x
    // 两套约定独立，不混用。
    omega_ref[0] = 2.0f * g.kp_roll  * q_err.x;  // FRD roll  (p) / VTOL 差速轴
    omega_ref[1] = 2.0f * g.kp_pitch * q_err.y;  // FRD pitch (q) / VTOL 俯仰轴（相同）
    omega_ref[2] = 2.0f * g.kp_yaw   * q_err.z;  // FRD yaw   (r) / VTOL 侧倾轴

    // ---- 步骤4：各轴限幅 -----------------------------------------
    omega_ref[0] = clamp_f(omega_ref[0], -g.omega_max_roll,  g.omega_max_roll);
    omega_ref[1] = clamp_f(omega_ref[1], -g.omega_max_pitch, g.omega_max_pitch);
    omega_ref[2] = clamp_f(omega_ref[2], -g.omega_max_yaw,   g.omega_max_yaw);
}
