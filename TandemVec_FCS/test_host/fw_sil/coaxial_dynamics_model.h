/**
 * @file    coaxial_dynamics_model.h
 * @module  SIL / 共轴双桨 VTVL 动力学（2026-10-31）
 * @brief   共轴双桨 VTVL 的 L2 刚体 + 双旋翼 + TVC 摆座模型（纯函数、无 Arduino 依赖）
 *
 * @details 相对旧 SIL 的 L1 集总 wrench，本模型补齐四件事：
 *          ① 推力平方律 T = kT·ω²（旧模型是平均转速的线性映射）；
 *          ② 反扭力矩 τ = kQ·ω² + Jp·ω̇（含加减速惯性项，不是固定系数×差速）；
 *          ③ 双转子角动量陀螺耦合 M = −Ω×(Jp·(ω1−ω2)·n)（等速反转时严格抵消）；
 *          ④ 独立 6DOF 积分器（半隐式、子步），含可选气动阻尼与地面法向/摩擦接触。
 *
 * @note    坐标系：机体系 FRD（x 前、y 右、z 下）；NED 惯性系（z 下）。
 *          推力轴在零摆角时沿 −z_b（向上）。摆座角定义与固件一致：
 *            F_b = T·[sin δt, sin δf, −cos δf·cos δt]ᵀ
 *          δt 绕机体系 y（俯仰通道）、δf 绕 x（滚转通道）；力矩符号沿用
 *          SIL/固件已标定的通道方向（δf→Mx、δt→My、差速→−Mz），并附
 *          物理来源注释，避免"公式对了符号反了"的假验证。
 *
 * @warning 参数默认值来自 TandemVec 存档 L1（mass 0.84 kg、maxT 14 N、
 *          wMax 1150 rad/s、lever 0.15 m、I=(0.05,0.05,0.08) kg·m²），
 *          **不是共轴实机辨识值**。kQ/kT=0.027 沿用纵列实机经验值；
 *          Jp=2e-4 kg·m² 为同量级估计。上机前必须台架辨识 kT/kQ/Jp/τ_m。
 *
 * @ref THY-002 §3–§6（反扭、电机一阶滞后、陀螺力矩）、MOD-005（待建）
 * @test test_host/test_coaxial_dynamics_model.cpp
 */

#ifndef BFS_COAXIAL_DYNAMICS_MODEL_H
#define BFS_COAXIAL_DYNAMICS_MODEL_H

#include <cmath>
#include <cstring>

namespace bfs
{
namespace coax
{

struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quat
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline float clampf(const float v, const float lo, const float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

inline bool finite3(const Vec3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
    return Vec3{a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}

inline float dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Quat quatMul(const Quat &a, const Quat &b)
{
    return Quat{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Quat quatNormalize(const Quat &q)
{
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0.0f) || !std::isfinite(n))
    {
        return Quat{};
    }
    const float inv = 1.0f / n;
    return Quat{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

inline Vec3 bodyToNed(const Quat &q, const Vec3 &v)
{
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    return Vec3{
        (1.0f - 2.0f * (y * y + z * z)) * v.x +
            2.0f * (x * y - w * z) * v.y + 2.0f * (x * z + w * y) * v.z,
        2.0f * (x * y + w * z) * v.x +
            (1.0f - 2.0f * (x * x + z * z)) * v.y +
            2.0f * (y * z - w * x) * v.z,
        2.0f * (x * z - w * y) * v.x + 2.0f * (y * z + w * x) * v.y +
            (1.0f - 2.0f * (x * x + y * y)) * v.z};
}

/** @brief 模型参数（SI；默认值见文件头 warning） */
struct Params
{
    float mass_kg = 0.84f;
    float inertia_kgm2[3] = {0.05f, 0.05f, 0.08f}; // Ix, Iy, Iz
    float lever_m = 0.15f;                          // CG → 推力作用点（沿 z_b）
    float kT_n_per_radps2 = 5.2930e-6f;             // 单旋翼 T = kT·ω²
    float kQ_nm_per_radps2 = 1.4291e-7f;            // 单旋翼 Q = kQ·ω²
    float rotor_inertia_kgm2 = 2.0e-4f;             // Jp（转子+桨）
    float w_max_radps = 1150.0f;
    float motor_tau_s = 0.28f;
    float servo_tau_s = 0.05f;
    float servo_rate_dps = 300.0f;
    float gravity_mps2 = 9.80665f;
    float drag_linear_n_per_mps = 0.0f;   // 默认 0：VTVL 低速不做气动（用户口径）
    float drag_quadratic_n_per_mps2 = 0.0f;
    float ground_stiffness_n_per_m = 2.0e5f;
    float ground_damping_n_per_mps = 5.0e3f;
    float ground_mu = 0.5f;
    bool enable_gyro = true;
    bool enable_ground = true;
};

/** @brief 由最大总推力反标定 kT；kQ 按比例给出（沿用纵列 0.027 经验） */
inline Params makeParams(const float max_total_thrust_n,
                         const float w_max_radps,
                         const float kq_over_kt = 0.027f)
{
    Params p;
    p.w_max_radps = w_max_radps;
    const float w2 = w_max_radps * w_max_radps;
    p.kT_n_per_radps2 = (w2 > 0.0f) ? max_total_thrust_n / (2.0f * w2) : 0.0f;
    p.kQ_nm_per_radps2 = p.kT_n_per_radps2 * kq_over_kt;
    return p;
}

/** @brief 悬停所需单旋翼转速：2·kT·ω² = m·g */
inline float hoverRotorSpeed(const Params &p)
{
    const float denom = 2.0f * p.kT_n_per_radps2;
    if (!(denom > 0.0f))
    {
        return 0.0f;
    }
    return std::sqrt(p.mass_kg * p.gravity_mps2 / denom);
}

/** @brief 悬停所需油门比例（0..1，线性 ω 指令口径） */
inline float hoverThrottle(const Params &p)
{
    if (!(p.w_max_radps > 0.0f))
    {
        return 0.0f;
    }
    return clampf(hoverRotorSpeed(p) / p.w_max_radps, 0.0f, 1.0f);
}

/** @brief 模型输出：机体系力/力矩 + 可观测分项（N、N·m） */
struct Wrench
{
    Vec3 force_body;   // N，FRD
    Vec3 moment_body;  // N·m，FRD
    float thrust_n = 0.0f;
    float reaction_nm = 0.0f; // 双旋翼反扭（Mz 分量，含 ω̇ 惯性项）
    Vec3 gyro_moment_nm;
    bool valid = false;
};

/**
 * @brief 双旋翼 + TVC 摆座的 wrench（不含重力/气动/地面）
 * @param wf/wt 当前左右（前/后）旋翼转速 rad/s
 * @param wdot_f/wdot_t 当前转速导数 rad/s²（由一阶电机模型给出）
 * @param servo_f/servo_t 实际摆座角 rad
 * @param omega_body 机体系角速率 rad/s（陀螺项用）
 */
inline Wrench computeWrench(const Params &p, const float wf, const float wt,
                            const float wdot_f, const float wdot_t,
                            const float servo_f, const float servo_t,
                            const Vec3 &omega_body)
{
    Wrench out;
    if (!std::isfinite(wf) || !std::isfinite(wt) || !std::isfinite(wdot_f) ||
        !std::isfinite(wdot_t) || !std::isfinite(servo_f) ||
        !std::isfinite(servo_t) || !finite3(omega_body))
    {
        return out;
    }

    const float wf2 = wf * wf;
    const float wt2 = wt * wt;
    const float thrust = p.kT_n_per_radps2 * (wf2 + wt2);
    const float cf = std::cos(servo_f);
    const float sf = std::sin(servo_f);
    const float ct = std::cos(servo_t);
    const float st = std::sin(servo_t);

    // 推力方向：δt→+x_b、δf→+y_b（与固件/SIL 已标定的通道方向一致）。
    out.force_body = Vec3{thrust * st, thrust * sf, -thrust * cf * ct};

    // 摆座力臂力矩：δf→+Mx、δt→+My（旧 SIL 的标定方向）。
    out.moment_body.x = p.lever_m * thrust * sf;
    out.moment_body.y = p.lever_m * thrust * st;

    // 反扭：两旋翼反转，净反扭 ∝ (ω1²−ω2²) 与转速导数差；符号与旧 SIL
    // 的“wf>wt → −Mz”一致。
    const float reaction =
        p.kQ_nm_per_radps2 * (wf2 - wt2) +
        p.rotor_inertia_kgm2 * (wdot_f - wdot_t);
    out.reaction_nm = reaction;
    out.moment_body.z = -reaction;

    if (p.enable_gyro)
    {
        // 双转子净角动量沿推力轴（−z_b）；等速反转时 (wf−wt)=0，严格抵消。
        const Vec3 h{0.0f, 0.0f, -p.rotor_inertia_kgm2 * (wf - wt)};
        out.gyro_moment_nm = Vec3{-cross(omega_body, h).x,
                                  -cross(omega_body, h).y,
                                  -cross(omega_body, h).z};
        out.moment_body.x += out.gyro_moment_nm.x;
        out.moment_body.y += out.gyro_moment_nm.y;
        out.moment_body.z += out.gyro_moment_nm.z;
    }

    out.thrust_n = thrust;
    out.valid = std::isfinite(thrust) && finite3(out.force_body) &&
                finite3(out.moment_body);
    return out;
}

/** @brief 6DOF 状态（NED 位置速度、机体系姿态角速率、旋翼/摆座状态） */
struct State
{
    Quat q;          // body→NED
    Vec3 omega;      // body rad/s
    Vec3 vel_ned;    // m/s
    Vec3 pos_ned;    // m（z 向下，地面 z=0）
    float wf = 0.0f; // rad/s
    float wt = 0.0f;
    float servo_f = 0.0f; // rad
    float servo_t = 0.0f;
    bool on_ground = false;
};

/** @brief 开环指令：旋翼目标转速与摆座目标角 */
struct Command
{
    float wf_cmd = 0.0f;
    float wt_cmd = 0.0f;
    float servo_f_cmd = 0.0f;
    float servo_t_cmd = 0.0f;
};

inline float mechanicalEnergy(const Params &p, const State &s)
{
    const float v2 = dot(s.vel_ned, s.vel_ned);
    const float w2 = s.omega.x * s.omega.x * p.inertia_kgm2[0] +
                     s.omega.y * s.omega.y * p.inertia_kgm2[1] +
                     s.omega.z * s.omega.z * p.inertia_kgm2[2];
    // 势能以地面为 0；NED z 向下 → h = −z。
    return 0.5f * p.mass_kg * v2 + 0.5f * w2 -
           p.mass_kg * p.gravity_mps2 * s.pos_ned.z;
}

/**
 * @brief 推进一个控制周期（半隐式 + 子步）
 * @note  摆座/旋翼均为一阶跟随 + 速率限幅；地面为弹簧-阻尼法向 + 库仑摩擦，
 *        仅作 SIL 用（不是接触力学高保真模型）。
 */
/**
 * @brief 状态导数（RK4 各阶段复用）
 * @note  旋翼/舵机一阶跟随；舵机速率限幅通过导数钳位实现（不再事后夹位置）。
 *        地面法向弹簧-阻尼 + 库仑摩擦只在 z>=0 时进入导数；接触是事件，
 *        RK4 在穿越瞬间只保证子步精度，靠 substeps 收敛（见 §5 说明）。
 */
struct Deriv
{
    float qdot[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float omegadot[3] = {0.0f, 0.0f, 0.0f};
    float veldot[3] = {0.0f, 0.0f, 0.0f};
    float posdot[3] = {0.0f, 0.0f, 0.0f};
    float wfdot = 0.0f;
    float wtdot = 0.0f;
    float servofdot = 0.0f;
    float servotdot = 0.0f;
    bool valid = false;
};

inline Deriv derivative(const Params &p, const State &s, const Command &c)
{
    Deriv d;
    const float tau_m = (p.motor_tau_s > 0.0f) ? p.motor_tau_s : 0.0f;
    d.wfdot = (tau_m > 0.0f) ? (c.wf_cmd - s.wf) / tau_m : 0.0f;
    d.wtdot = (tau_m > 0.0f) ? (c.wt_cmd - s.wt) / tau_m : 0.0f;
    // 舵机：一阶跟随，角速率按 servo_rate_dps 钳位（rad/s）。
    const float rate = (p.servo_rate_dps > 0.0f)
                           ? p.servo_rate_dps * 0.017453292f
                           : 1.0e9f;
    const float tau_s = (p.servo_tau_s > 0.0f) ? p.servo_tau_s : 0.0f;
    d.servofdot = clampf((tau_s > 0.0f) ? (c.servo_f_cmd - s.servo_f) / tau_s
                                        : 0.0f,
                         -rate, rate);
    d.servotdot = clampf((tau_s > 0.0f) ? (c.servo_t_cmd - s.servo_t) / tau_s
                                        : 0.0f,
                         -rate, rate);

    const Vec3 omega{s.omega.x, s.omega.y, s.omega.z};
    const Wrench w = computeWrench(p, s.wf, s.wt, d.wfdot, d.wtdot, s.servo_f,
                                   s.servo_t, omega);
    if (!w.valid)
    {
        return d;
    }

    Vec3 f_body = w.force_body;
    Vec3 m_body = w.moment_body;
    (void)m_body;

    Vec3 f_ned = bodyToNed(s.q, f_body);
    f_ned.z += p.mass_kg * p.gravity_mps2;
    const float speed = std::sqrt(dot(s.vel_ned, s.vel_ned));
    if (speed > 1.0e-6f)
    {
        const float fd = p.drag_linear_n_per_mps * speed +
                         p.drag_quadratic_n_per_mps2 * speed * speed;
        f_ned.x -= fd * s.vel_ned.x / speed;
        f_ned.y -= fd * s.vel_ned.y / speed;
        f_ned.z -= fd * s.vel_ned.z / speed;
    }
    if (p.enable_ground && s.pos_ned.z >= 0.0f)
    {
        float normal = p.ground_stiffness_n_per_m * s.pos_ned.z +
                       p.ground_damping_n_per_mps * s.vel_ned.z;
        if (normal < 0.0f)
        {
            normal = 0.0f;
        }
        f_ned.z -= normal;
        const float vh =
            std::sqrt(s.vel_ned.x * s.vel_ned.x + s.vel_ned.y * s.vel_ned.y);
        if (vh > 1.0e-6f)
        {
            const float fmax = p.ground_mu * normal;
            const float acc = std::fmin(fmax / p.mass_kg, vh / 0.001f);
            f_ned.x -= acc * p.mass_kg * s.vel_ned.x / vh;
            f_ned.y -= acc * p.mass_kg * s.vel_ned.y / vh;
        }
    }

    d.veldot[0] = f_ned.x / p.mass_kg;
    d.veldot[1] = f_ned.y / p.mass_kg;
    d.veldot[2] = f_ned.z / p.mass_kg;
    d.posdot[0] = s.vel_ned.x;
    d.posdot[1] = s.vel_ned.y;
    d.posdot[2] = s.vel_ned.z;

    // 欧拉方程：I·ω̇ = M − ω×(I·ω)
    const float p_rate = s.omega.x, q_rate = s.omega.y, r_rate = s.omega.z;
    const float Ix = p.inertia_kgm2[0];
    const float Iy = p.inertia_kgm2[1];
    const float Iz = p.inertia_kgm2[2];
    d.omegadot[0] = (w.moment_body.x - (Iz - Iy) * q_rate * r_rate) / Ix;
    d.omegadot[1] = (w.moment_body.y - (Ix - Iz) * r_rate * p_rate) / Iy;
    d.omegadot[2] = (w.moment_body.z - (Iy - Ix) * p_rate * q_rate) / Iz;

    // q̇ = ½ q ⊗ (0, ω)
    d.qdot[0] = -0.5f * (s.q.x * p_rate + s.q.y * q_rate + s.q.z * r_rate);
    d.qdot[1] = 0.5f * (s.q.w * p_rate + s.q.y * r_rate - s.q.z * q_rate);
    d.qdot[2] = 0.5f * (s.q.w * q_rate - s.q.x * r_rate + s.q.z * p_rate);
    d.qdot[3] = 0.5f * (s.q.w * r_rate + s.q.x * q_rate - s.q.y * p_rate);
    d.valid = true;
    return d;
}

inline State stageState(const State &s, const Deriv &d, const float scale)
{
    State o = s;
    o.q.w += d.qdot[0] * scale;
    o.q.x += d.qdot[1] * scale;
    o.q.y += d.qdot[2] * scale;
    o.q.z += d.qdot[3] * scale;
    o.q = quatNormalize(o.q);
    o.omega.x += d.omegadot[0] * scale;
    o.omega.y += d.omegadot[1] * scale;
    o.omega.z += d.omegadot[2] * scale;
    o.vel_ned.x += d.veldot[0] * scale;
    o.vel_ned.y += d.veldot[1] * scale;
    o.vel_ned.z += d.veldot[2] * scale;
    o.pos_ned.x += d.posdot[0] * scale;
    o.pos_ned.y += d.posdot[1] * scale;
    o.pos_ned.z += d.posdot[2] * scale;
    o.wf += d.wfdot * scale;
    o.wt += d.wtdot * scale;
    o.servo_f += d.servofdot * scale;
    o.servo_t += d.servotdot * scale;
    return o;
}

inline State combineRK4(const State &s, const Deriv &k1, const Deriv &k2,
                        const Deriv &k3, const Deriv &k4, const float h)
{
    State o = s;
    const float w1 = h / 6.0f, w2 = h / 3.0f;
    o.q.w += w1 * (k1.qdot[0] + 2.0f * k2.qdot[0] + 2.0f * k3.qdot[0] +
                   k4.qdot[0]);
    o.q.x += w1 * (k1.qdot[1] + 2.0f * k2.qdot[1] + 2.0f * k3.qdot[1] +
                   k4.qdot[1]);
    o.q.y += w1 * (k1.qdot[2] + 2.0f * k2.qdot[2] + 2.0f * k3.qdot[2] +
                   k4.qdot[2]);
    o.q.z += w1 * (k1.qdot[3] + 2.0f * k2.qdot[3] + 2.0f * k3.qdot[3] +
                   k4.qdot[3]);
    o.q = quatNormalize(o.q);
    for (int i = 0; i < 3; ++i)
    {
        const float* a = &k1.omegadot[0];
        const float* b = &k2.omegadot[0];
        const float* c = &k3.omegadot[0];
        const float* d = &k4.omegadot[0];
        (&o.omega.x)[i] += w1 * (a[i] + 2.0f * b[i] + 2.0f * c[i] + d[i]);
        (&o.vel_ned.x)[i] += w1 * (k1.veldot[i] + 2.0f * k2.veldot[i] +
                                   2.0f * k3.veldot[i] + k4.veldot[i]);
        (&o.pos_ned.x)[i] += w1 * (k1.posdot[i] + 2.0f * k2.posdot[i] +
                                   2.0f * k3.posdot[i] + k4.posdot[i]);
    }
    o.wf += w1 * (k1.wfdot + 2.0f * k2.wfdot + 2.0f * k3.wfdot + k4.wfdot);
    o.wt += w1 * (k1.wtdot + 2.0f * k2.wtdot + 2.0f * k3.wtdot + k4.wtdot);
    o.servo_f += w1 * (k1.servofdot + 2.0f * k2.servofdot +
                       2.0f * k3.servofdot + k4.servofdot);
    o.servo_t += w1 * (k1.servotdot + 2.0f * k2.servotdot +
                       2.0f * k3.servotdot + k4.servotdot);
    (void)w2;
    return o;
}

/**
 * @brief 推进一个控制周期（经典四阶 Runge-Kutta，子步 dt/substeps）
 * @note  执行器/舵机/旋翼/地面全部进入导数；舵机速率限幅在导数里钳位。
 *        地面接触为事件型约束，RK4 在穿越瞬间由子步收敛；落地后仍做
 *        位置投影（z>=0、vz>=0）以防止数值穿透。默认 substeps=10。
 */
inline void step(const Params &p, State &s, const Command &cmd, const float dt,
                 const int substeps = 10)
{
    if (!(dt > 0.0f) || substeps <= 0)
    {
        return;
    }
    const float h = dt / static_cast<float>(substeps);
    for (int k = 0; k < substeps; ++k)
    {
        const Deriv k1 = derivative(p, s, cmd);
        if (!k1.valid)
        {
            return;
        }
        const Deriv k2 = derivative(p, stageState(s, k1, 0.5f * h), cmd);
        const Deriv k3 = derivative(p, stageState(s, k2, 0.5f * h), cmd);
        const Deriv k4 = derivative(p, stageState(s, k3, h), cmd);
        s = combineRK4(s, k1, k2, k3, k4, h);
        if (p.enable_ground && s.pos_ned.z > 0.0f)
        {
            s.pos_ned.z = 0.0f;
            if (s.vel_ned.z > 0.0f)
            {
                s.vel_ned.z = 0.0f;
            }
            s.on_ground = true;
        }
        else
        {
            s.on_ground = false;
        }
    }
}

} // namespace coax
} // namespace bfs

#endif // BFS_COAXIAL_DYNAMICS_MODEL_H
