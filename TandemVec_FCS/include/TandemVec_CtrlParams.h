// ============================================================
//  TandemVec_CtrlParams.h — 级联控制律全局调参结构体
//
//  所有飞控增益集中在此文件，便于：
//    · 一眼纵览全套参数，不必翻多个文件
//    · 地面站解析 / MAVLink参数同步时有统一入口
//    · 单独调参时按层隔离（姿态外环独立于角速率内环）
//
//  调参步骤（从内到外）：
//    Step 1  先关闭外环（kp_att全部置0），仅运行角速率环：
//              调整 rate.kp 使阶跃响应无明显超调，带宽 ~30Hz；
//    Step 2  打开外环，逐轴增大 att.kp，直到姿态收敛够快但不振荡；
//    Step 3  如有稳态误差，增大 rate.ki（慎用，先调积分限幅 int_max）；
//    Step 4  如微分噪声放大，减小 rate.kd 或在陀螺仪后加低通滤波。
//
//  单位约定（均为 SI）：
//    att.kp     [rad/s / rad]   — 角度误差 1 rad → 命令角速率 kp rad/s
//    rate.kp    [rad/s² / (rad/s)] — 速率误差 1 rad/s → 命令角加速度 kp rad/s²
//    rate.ki    [rad/s² / (rad)]   — 积分项，单位 rad/s² 每 rad·s
//    rate.kd    [rad/s² / (rad/s²)] — 微分先行，无量纲乘以角速率导数
//    omega_max  [rad/s]   — 外环输出的角速率饱和值
//    alpha_max  [rad/s²]  — 内环输出的角加速度饱和值
//    int_max    [rad/s]   — 积分项的等效角速率上限，防止饱和
// ============================================================
#pragma once

#include <cstdint>

// ============================================================
//  外环：姿态角（四元数误差 → 目标角速率）
// ============================================================
struct AttitudeCtrlGains
{
    // 比例增益（各轴可独立调节）
    float kp_roll;   // rad/s / rad   滚转：差速反扭，低油门效能衰减
    float kp_pitch;  // rad/s / rad   俯仰：尾摆，中等带宽
    float kp_yaw;    // rad/s / rad   偏航：前摆，中等带宽

    // 外环输出的角速率饱和值（保护内环不过驱动）
    float omega_max_roll;   // rad/s
    float omega_max_pitch;  // rad/s
    float omega_max_yaw;    // rad/s
};

// ============================================================
//  内环：角速率（误差 → 目标角加速度，位置式PID + 抗积分饱和）
// ============================================================
struct RateCtrlGains
{
    // 比例/积分/微分增益（各轴独立）
    float kp[3];  // [roll, pitch, yaw]  rad/s² / (rad/s)
    float ki[3];  // [roll, pitch, yaw]  rad/s² / rad（积分项）
    float kd[3];  // [roll, pitch, yaw]  微分增益（微分先行）

    // 角加速度指令饱和（同时限制 M_cmd 间接限幅）
    float alpha_max[3];  // [roll, pitch, yaw]  rad/s²

    // 积分器软限幅（防止积分 windup）
    float int_max[3];    // [roll, pitch, yaw]  rad/s（积分项等效速率）
};

// ============================================================
//  完整级联参数（一个结构体传遍所有层）
// ============================================================
struct CascadeCtrlParams
{
    AttitudeCtrlGains att;
    RateCtrlGains     rate;
};

// ============================================================
//  默认值（MODEL-DEFAULT，适合 ~0.7 kg VTOL 初飞调参起点；
//  惯量与增益均待台架标定精化，原 2.6 kg 注释已更正）
//
//  设计依据：
//    · 角速率内环带宽目标 ~25 Hz：kp ≈ 2π·25 / (∂M/∂u·1/I)
//      但在未知标定值下，保守取小值，由实飞逐步增大。
//    · 外环带宽比内环低 5–8×：kp_att ≈ inner_bandwidth / 6
//    · 积分上限 int_max：约外环稳态期望速率的 20%，防止长时漂移
// ============================================================
static const CascadeCtrlParams kDefaultCascadeCtrlParams = {
    // ---- 外环 ---------------------------------------------------
    /* att */ {
        /* kp_roll  */ 4.0f,   // rad/s / rad
        /* kp_pitch */ 4.0f,
        /* kp_yaw   */ 3.0f,   // 偏航带宽稍低（前摆权限有限）

        /* omega_max_roll  */ 2.0f,   // rad/s  ~115°/s
        /* omega_max_pitch */ 1.5f,   // rad/s  ~85°/s
        /* omega_max_yaw   */ 1.0f,   // rad/s  ~57°/s（偏航较慢）
    },
    // ---- 内环 ---------------------------------------------------
    /* rate */ {
        /* kp[3]       */ { 8.0f,  12.0f,  10.0f  },   // [roll, pitch, yaw]
        /* ki[3]       */ { 1.5f,   2.0f,   1.5f  },
        /* kd[3]       */ { 0.05f,  0.08f,  0.05f },
        /* alpha_max[3]*/ { 40.0f,  25.0f,  20.0f },   // rad/s²
        /* int_max[3]  */ {  5.0f,   4.0f,   3.0f },   // rad/s 等效
    },
};
