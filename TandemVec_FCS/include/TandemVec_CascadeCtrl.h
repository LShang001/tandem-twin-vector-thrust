// ============================================================
//  TandemVec_CascadeCtrl.h — 级联控制律顶层编排
//
//  将四个物理层串联为单次调用：
//
//    q_ref, q_meas  →  [AttitudeCtrl]  →  ω_ref
//    ω_ref, ω_meas  →  [RateCtrl]      →  α_ref
//    α_ref          →  [I·α]           →  M_cmd
//    M_cmd, w0      →  [ControlAlloc]  →  δ_f, δ_t, Δω
//
//  每步的中间状态均保存在 CascadeTelemetry，供地面站日志、
//  遥测和调参时实时观测。
//
//  使用方式：
//    CascadeCtrl ctrl;
//    ctrl.reset();
//    // 200 Hz 循环内：
//    CascadeOutput out = ctrl.step(input, params, alloc_params, dt);
//    // 读取执行器指令
//    float delta_f = out.alloc.delta_f;
//    float delta_t = out.alloc.delta_t;
//    float dw      = out.alloc.dw;
//    // 读取遥测（地面站/黑匣子）
//    const CascadeTelemetry& tel = out.tel;
// ============================================================
#pragma once
#include <cmath>
#include "TandemVec_Config.h"

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.14159265f)
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.14159265f / 180.0f)
#endif
#include "TandemVec_CtrlParams.h"
#include "TandemVec_AttitudeCtrl.h"
#include "TandemVec_RateCtrl.h"
#include "TandemVec_ControlAllocation.h"

// ============================================================
//  顶层输入
// ============================================================
struct CascadeInput
{
    // ---- 目标姿态 ----
    Quat4f q_ref;   // 由导航层/RC映射产生的目标四元数

    // ---- 反馈量（来自AHRS/IMU） ----
    Quat4f q_meas;          // 当前姿态四元数
    float  omega_meas[3];   // 当前角速率 [p,q,r] rad/s（机体系 FRD）

    // ---- 推进状态 ----
    float  thr;             // 归一化油门 [0,1]（用于计算 w0 = thr·wMax）
};

// ============================================================
//  遥测结构体（每层中间输出，用于调参/日志/MAVLink）
// ============================================================
struct CascadeTelemetry
{
    // 外环
    float q_err[4];         // 姿态误差四元数 [w,x,y,z]
    float omega_ref[3];     // 外环输出目标角速率 [p,q,r] rad/s
    bool  omega_sat[3];     // 目标角速率是否触达 omega_max

    // 内环
    float omega_err[3];     // 速率误差 [p,q,r] rad/s
    float alpha_ref[3];     // 目标角加速度 [p,q,r] rad/s²
    bool  alpha_sat[3];     // 角加速度是否触达 alpha_max
    bool  int_frozen[3];    // 积分器是否冻结（抗积分饱和状态）

    // 力矩映射
    float M_cmd[3];         // 期望力矩 [Mx,My,Mz] N·m

    // 分配层（含摆角/差速饱和标记）
    AllocationOutput alloc;

    // 工作点
    float w0;               // 基准转速 rad/s
};

// ============================================================
//  顶层输出
// ============================================================
struct CascadeOutput
{
    // 快速路径：常用执行器指令直接引用
    float delta_f;   // rad  前摆角（偏航）
    float delta_t;   // rad  尾摆角（俯仰）
    float dw;        //  -   差速指令 [-dwMax, +dwMax]

    // 完整遥测（供黑匣子 / 地面站）
    CascadeTelemetry tel;
};

// ============================================================
//  CascadeCtrl — 主控制器类
// ============================================================
struct CascadeCtrl
{
    RateCtrl rate_ctrl;  // 内环PID（含积分状态）

    // ---- 重置所有状态 ----
    // 模式切换、解锁瞬间调用，防止旧积分值引起冲击
    void reset() { rate_ctrl.reset(); }

    // ---- 单步推进 ----
    //
    // @param in      当前传感器反馈 + 目标姿态
    // @param cp      控制律增益参数（外环+内环）
    // @param ap      分配层机型参数（kT/kQ/a/b/dMax/…）
    // @param ac_stg  控制分配策略（默认 FULL_B）
    // @param dt      控制周期 s（典型 0.005 s @ 200 Hz）
    // @return        执行器指令 + 全层遥测
    CascadeOutput step(const CascadeInput&    in,
                       const CascadeCtrlParams& cp,
                       const TandemVecParams&   ap,
                       AllocationStrategy       ac_stg,
                       float dt)
    {
        CascadeOutput out{};
        CascadeTelemetry& tel = out.tel;

        // ============================================================
        //  层1：外环 — 四元数误差 → 目标角速率
        // ============================================================
        attitudeStep(in.q_meas, in.q_ref, cp.att, tel.omega_ref);  // 输出 rad/s

        // 转换为 deg/s（rateCtrl 和 in.omega_meas 均按 deg/s 约定）
        for (int i = 0; i < 3; ++i) tel.omega_ref[i] *= RAD_TO_DEG;

        // 记录误差四元数（供调参查看收敛情况）
        {
            Quat4f qe = qNorm(qMul(qConj(in.q_meas), in.q_ref));
            if (qe.w < 0.f) { qe.w=-qe.w; qe.x=-qe.x; qe.y=-qe.y; qe.z=-qe.z; }
            tel.q_err[0] = qe.w; tel.q_err[1] = qe.x;
            tel.q_err[2] = qe.y; tel.q_err[3] = qe.z;
        }

        // 记录角速率是否饱和
        for (int i = 0; i < 3; ++i) {
            const float lim[3] = { cp.att.omega_max_roll,
                                   cp.att.omega_max_pitch,
                                   cp.att.omega_max_yaw };
            tel.omega_sat[i] = (tel.omega_ref[i] >=  lim[i]) ||
                               (tel.omega_ref[i] <= -lim[i]);
        }

        // ============================================================
        //  层2：内环 — 目标角速率 → 目标角加速度（位置式 PID）
        // ============================================================
        for (int i = 0; i < 3; ++i)
            tel.omega_err[i] = tel.omega_ref[i] - in.omega_meas[i];

        rate_ctrl.step(tel.omega_ref, in.omega_meas, cp.rate, dt, tel.alpha_ref);

        // 记录内环饱和与积分冻结状态
        for (int i = 0; i < 3; ++i) {
            tel.alpha_sat[i]  = (tel.alpha_ref[i] >= cp.rate.alpha_max[i]) ||
                                (tel.alpha_ref[i] <= -cp.rate.alpha_max[i]);
            tel.int_frozen[i] = rate_ctrl.state.int_frozen[i];
        }

        // ============================================================
        //  层3：惯量逆解 — α_ref → M_cmd = I · α_ref
        // ============================================================
        tel.M_cmd[0] = ap.Ix * tel.alpha_ref[0];   // Mx = Ix·α_roll
        tel.M_cmd[1] = ap.Iy * tel.alpha_ref[1];   // My = Iy·α_pitch
        tel.M_cmd[2] = ap.Iz * tel.alpha_ref[2];   // Mz = Iz·α_yaw

        // ============================================================
        //  层4：控制分配 — M_cmd → δ_f, δ_t, Δω
        // ============================================================
        tel.w0 = in.thr * ap.wMax;

        AllocationInput ai;
        ai.Mx_cmd = tel.M_cmd[0];
        ai.My_cmd = tel.M_cmd[1];
        ai.Mz_cmd = tel.M_cmd[2];
        ai.w0     = tel.w0;

        tel.alloc = allocateMoments(ai, ap, ac_stg);

        // ============================================================
        //  快速路径输出
        // ============================================================
        out.delta_f = tel.alloc.delta_f;
        out.delta_t = tel.alloc.delta_t;
        out.dw      = tel.alloc.dw;

        return out;
    }

    // ---- 便捷重载：使用默认参数 ----
    CascadeOutput step(const CascadeInput& in, float dt)
    {
        return step(in,
                    kDefaultCascadeCtrlParams,
                    kDefaultTandemVecParams,
                    AllocationStrategy::FULL_B,
                    dt);
    }
};
