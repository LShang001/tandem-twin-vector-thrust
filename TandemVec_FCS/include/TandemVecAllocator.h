/**
 * @file    TandemVecAllocator.h
 * @module  固件/控制（纵列双发机型分配器：角加速度 → 执行器）
 * @brief   机型无关控制层输出的 α_cmd + 油门，经惯量逆解、轴置换、BTRUE、
 *          差速开方后得到两摆角/两电机；这是当前 TandemVec 的机型逆解边界
 * @details 输入：α_roll/pitch/yaw（存档系）、油门、陀螺（已减零偏）、上一拍
 *          PropulsionState、τm 观测状态、惯量前馈掩码、dt；
 *          处理：w_hover/w0_eff 调度 → I·α + M_ff → 轴置换 → BTRUE →
 *          差速开方 → 电机百分比；输出：摆角 rad、电机 %、M_cmd/M_ff、
 *          饱和标记、下一拍状态。
 *          **不写硬件、不读全局**，是控制律与 GPIO 输出之间的机型层。
 * @theory  M = I·α + ω×(Iω) + ω×h；轴置换 x′=−z_b、y′=+y_b、z′=+x_b；
 *          u = B_true⁻¹ M；ωf,t = w0√(1±Δω)
 *          @ref THY-003 §2–§3；CTL-001 §7/§8.6；CTL-003 §2
 * @frame   α/gyro 为存档系（x=滚转、y=俯仰、z=推力轴）；M 为模型系
 * @units   α rad/s²、ω rad/s、M N·m、摆角 rad、电机 %
 * @note    ★ 本文件只搬运当前已验证的 TandemVec 机型逆解，行为红线（差速开方、
 *          τm 观测、零油门归中、BTRUE 退化、轴置换）不得在此“顺手改公式”。
 *          共轴/火箭需要各自的 allocator；它们的控制层仍可复用 α_cmd 接口。
 * @test    test_host/test_tandemvec_allocator.cpp（与 flight_control 旧公式/SIL 对照）
 * @see     TandemVec_ControlAllocation.h、InertiaDecoupling.h、ActuatorOutputMap.h
 */

#ifndef TANDEMVEC_ALLOCATOR_H
#define TANDEMVEC_ALLOCATOR_H

#include <cmath>
#include <cstdint>

#include "InertiaDecoupling.h"
#include "TandemVec_Config.h"
#include "TandemVec_ControlAllocation.h"
#include "TandemVec_Propulsion.h"

struct TandemVecAllocatorState
{
    PropulsionState prev{}; // 上一拍实际指令工作点（BTRUE/观测器）
    float wf_est = 0.0f;
    float wt_est = 0.0f;
};

struct TandemVecAllocatorInput
{
    float alpha_roll_radps2 = 0.0f;
    float alpha_pitch_radps2 = 0.0f;
    float alpha_yaw_radps2 = 0.0f;
    float throttle_pct = 0.0f;
    float gyro_body_radps[3] = {0.0f, 0.0f, 0.0f}; // 已减 EKF 零偏的存档系
    float dt_s = 0.005f;
    uint8_t inertia_comp_mask = 0x03;
};

struct TandemVecAllocatorOutput
{
    bool valid = false;
    float delta_f_rad = 0.0f;
    float delta_t_rad = 0.0f;
    float motor1_pct = 0.0f;
    float motor2_pct = 0.0f;
    float dw = 0.0f;
    float M_cmd[3] = {0.0f, 0.0f, 0.0f};
    float M_ff[3] = {0.0f, 0.0f, 0.0f};
    float w0_eff = 0.0f;
    float yaw_gain_sched = 1.0f;
    bool sat_delta_f = false;
    bool sat_delta_t = false;
    bool sat_dw = false;
    TandemVecAllocatorState next{};
};

inline float tandemVecAllocatorClamp(const float v, const float lo,
                                     const float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief 纵列双发机型分配：α_cmd + 油门 → 两摆角/两电机
 * @param[in] in α/油门/陀螺/掩码/dt
 * @param[in] state 上一拍 PropulsionState 与 τm 观测器
 * @param[in] p 机型参数
 * @return 分配结果；任何非有限输入/输出置 valid=false 并回零执行器
 * @formula ① M_ff = InertiaComp(ω,I,h,mask)；M = I·α + M_ff（轴置换后）
 *          ② Mx *= min((w0_eff/w_hover)²,1)
 *          ③ u = B_true⁻¹ M；④ ωf,t = w0√(1±Δω)；⑤ motor%=ω/wMax×100
 * @ref THY-003 §3；CTL-001 §8.6；CTL-003 §2
 * @note 零油门 <5% 时 δ/Δω 归中；B 工作点用观测转速 + w0_floor，但差速
 *       分配用真实 w0——这是 2026-08/09 差速震荡修复的行为红线。
 */
inline TandemVecAllocatorOutput allocateTandemVec(
    const TandemVecAllocatorInput &in, const TandemVecAllocatorState &state,
    const TandemVecParams &p)
{
    TandemVecAllocatorOutput out;
    out.next = state;

    if (!std::isfinite(in.alpha_roll_radps2) ||
        !std::isfinite(in.alpha_pitch_radps2) ||
        !std::isfinite(in.alpha_yaw_radps2) ||
        !std::isfinite(in.throttle_pct) || !(in.dt_s > 0.0f) ||
        !std::isfinite(in.dt_s) || !std::isfinite(p.kT) || !(p.kT > 0.0f) ||
        !(p.wMax > 0.0f) || !(p.tauM > 0.0f))
    {
        return out;
    }

    const float w_hover = std::sqrt(0.5f * p.m * p.g / p.kT);
    const float w0_floor = 0.6f * w_hover;
    out.next.wf_est += (state.prev.wf - out.next.wf_est) * (in.dt_s / p.tauM);
    out.next.wt_est += (state.prev.wt - out.next.wt_est) * (in.dt_s / p.tauM);
    const float w0_est = std::sqrt(
        0.5f * (out.next.wf_est * out.next.wf_est +
                out.next.wt_est * out.next.wt_est));
    out.w0_eff = std::fmax(w0_est, w0_floor);

    // 层1：惯量逆解 + 轴置换 + 交叉耦合前馈（全部在机型层内部）
    float Mx = -p.Ix * in.alpha_yaw_radps2;
    float My = p.Iy * in.alpha_pitch_radps2;
    float Mz = -p.Iz * in.alpha_roll_radps2;
    if (inertiaCompEnabled(in.inertia_comp_mask))
    {
        const float h_along_thrust =
            p.Jp * (out.next.wf_est * std::cos(state.prev.delta_f) -
                    out.next.wt_est * std::cos(state.prev.delta_t));
        const InertiaCompResult ff = computeInertiaCompensationFromArchive(
            in.gyro_body_radps, p.Iz, p.Iy, p.Ix, h_along_thrust,
            in.inertia_comp_mask);
        Mx += ff.Mx_ff;
        My += ff.My_ff;
        Mz += ff.Mz_ff;
        out.M_ff[0] = ff.Mx_ff;
        out.M_ff[1] = ff.My_ff;
        out.M_ff[2] = ff.Mz_ff;
    }

    const float w0 = (in.throttle_pct / 100.0f) * p.wMax;
    out.yaw_gain_sched =
        (w_hover > 1.0f) ? (out.w0_eff * out.w0_eff) / (w_hover * w_hover)
                         : 1.0f;
    out.yaw_gain_sched = std::fmin(out.yaw_gain_sched, 1.0f);
    Mx *= out.yaw_gain_sched;
    out.M_cmd[0] = Mx;
    out.M_cmd[1] = My;
    out.M_cmd[2] = Mz;

    AllocationInput ai;
    ai.Mx_cmd = Mx;
    ai.My_cmd = My;
    ai.Mz_cmd = Mz;
    ai.w0 = w0;
    ai.current_state = state.prev;
    ai.current_state.wf = std::fmax(out.next.wf_est, w0_floor);
    ai.current_state.wt = std::fmax(out.next.wt_est, w0_floor);
    const AllocationOutput ao =
        allocateMoments(ai, p, AllocationStrategy::BTRUE);

    float delta_f = ao.delta_f;
    float delta_t = ao.delta_t;
    float dw = ao.dw;
    if (in.throttle_pct < 5.0f)
    {
        delta_f = 0.0f;
        delta_t = 0.0f;
        dw = 0.0f;
    }

    const DiffAllocResult diff = allocateDifferential(w0, dw, p);
    const float m1 = tandemVecAllocatorClamp(
        (diff.wf_target * 100.0f) / p.wMax, 0.0f, 100.0f);
    const float m2 = tandemVecAllocatorClamp(
        (diff.wt_target * 100.0f) / p.wMax, 0.0f, 100.0f);

    out.delta_f_rad = delta_f;
    out.delta_t_rad = delta_t;
    out.dw = dw;
    out.motor1_pct = m1;
    out.motor2_pct = m2;
    out.sat_delta_f = ao.sat_delta_f;
    out.sat_delta_t = ao.sat_delta_t;
    out.sat_dw = ao.sat_dw;
    out.next.prev.delta_f = delta_f;
    out.next.prev.delta_t = delta_t;
    out.next.prev.wf = diff.wf_target;
    out.next.prev.wt = diff.wt_target;
    out.valid = std::isfinite(out.delta_f_rad) &&
                std::isfinite(out.delta_t_rad) &&
                std::isfinite(out.motor1_pct) &&
                std::isfinite(out.motor2_pct) &&
                std::isfinite(out.dw) && std::isfinite(out.M_cmd[0]) &&
                std::isfinite(out.M_cmd[1]) && std::isfinite(out.M_cmd[2]);
    if (!out.valid)
    {
        out.delta_f_rad = out.delta_t_rad = 0.0f;
        out.motor1_pct = out.motor2_pct = 0.0f;
        out.dw = 0.0f;
    }
    return out;
}

#endif // TANDEMVEC_ALLOCATOR_H
