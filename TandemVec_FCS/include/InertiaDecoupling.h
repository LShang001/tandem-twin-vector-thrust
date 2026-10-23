/**
 * @file    InertiaDecoupling.h
 * @module  固件/控制（惯量逆解交叉耦合前馈，通用层，2026-08-10）
 * @brief   计算 M_ff = ω×(I·ω) + ω×h_rotor，补偿刚体转动交叉耦合与转子陀螺力矩
 * @details 输入：模型系角速度 ω、主轴惯量 Ix/Iy/Iz、转子角动量 h、使能掩码；
 *          处理：按位展开欧拉方程交叉项（对角惯量假设）；
 *          输出：模型系前馈力矩 M_ff [N·m]，由 mix 层直接加到 I·α 上。
 *          固件 gyro/控制律在存档系，必须经 `computeInertiaCompensationFromArchive()`
 *          做轴置换后再调用本原语（2026-10-23 C-02 修正）。
 *          被 flight_control.cpp 的层1 惯量逆解调用（inertia_comp_mask 默认 0x03）。
 *
 * 完整刚体转动动力学（与仿真 dynamics.mjs 同构，红线同源）：
 *     I·ω̇ = M − ω×(I·ω) − ω×h_rotor
 * 惯量逆解前馈（补偿项，加到 I·α 上得到期望力矩）：
 *     M_ff = ω×(I·ω) + ω×h_rotor
 *
 * 约定：
 *   - **模型系**（x′=推力轴、y′=俯仰、z′=滚转）是本原语的输入/输出坐标系；
 *     存档系（x=滚转、y=俯仰、z=推力轴）由 `...FromArchive()` 封装换算。
 *     两套轴名固定置换 x′=−z_b、y′=+y_b、z′=+x_b（det=+1）。
 *   - Ix/Iy/Iz = 绕模型系 x′/y′/z′ 的惯量（本机 Ix=轴向 0.0021、Iy=Iz=横向 0.022；
 *     陀螺耦合 gx 分量 (Iz−Iy)·ωy·ωz 在 Iy=Iz 时恒 0，这是细长杆布局的自然结果）。
 *   - h_rotor = 转子角动量合计（模型系；纵列双发前后转子反向，净角动量沿
 *     推力轴 = 模型系 x′ 分量，见仿真 hv.x = Jp(wf·cf − wt·ct)）。
 *     ★ 符号约定：模型系 x′ = −存档系 z_b，故 h_model=(+h,0,0) 对应
 *     h_archive=(0,0,−h)；写存档系公式时勿把 h 直接放 +z。
 *   - 使能掩码按位开关（默认全开；悬停 ω≈0 时交叉项≈0 无副作用，
 *     机动时直接受益；符号错误=正反馈风险 → host 测试对照刚体模型锁定，
 *     实机 A/B 可用参数 inertia_comp_mask 在线关闭）。
 *
 * @theory  欧拉方程 I·ω̇ = M − ω×(I·ω) 与转子陀螺力矩 −ω×h
 * @ref     THY-003 §1.2（欧拉方程交叉项推导）、§5（陀螺力矩/进动）；
 *          见 PROP-001（转子角动量来源）；AGENTS.md（前馈使能掩码约定）
 * @frame   模型系（x′=推力轴）；存档系包装见 `...FromArchive()`，rad/s
 * @units   ω [rad/s]；Ix/Iy/Iz [kg·m²]；h [kg·m²/s]；输出 [N·m]
 * @redline 惯量逆解前馈符号与坐标系（写反/错系 = 正反馈；host 测试锁定）
 * @units   ω [rad/s]；Ix/Iy/Iz [kg·m²]；h [kg·m²/s]；输出 [N·m]
 * @redline 惯量逆解前馈符号（写反 = 正反馈；host 测试 test_inertia_decoupling 锁定）
 * @note    本头文件为纯函数、零 Arduino 依赖，可 g++ 宿主机直接回归
 *          （沿用 ins_gnss_dynamic_weight.h 模式）。
 * @test    test_host/test_inertia_decoupling.cpp（含符号反例回归）
 */
#pragma once

#include <cmath>
#include <cstdint>

// 前馈使能位
enum InertiaCompBits : uint8_t
{
  kInertiaCompGyro  = 0x01,  // bit0: ω×(I·ω) 陀螺耦合（对角惯量展开）
  kInertiaCompRotor = 0x02,  // bit1: ω×h 转子角动量陀螺力矩
};

struct InertiaCompResult
{
  float Mx_ff = 0.0f;  // 前馈补偿力矩（机体系，N·m）
  float My_ff = 0.0f;
  float Mz_ff = 0.0f;
};

/**
 * @brief 计算交叉耦合前馈力矩 M_ff（**模型系**：x=推力轴、y=俯仰、z=滚转）
 * @param[in] omega   当前角速度（模型系，rad/s；来自 gyro，须已减 EKF 零偏）
 * @param[in] Ix/Iy/Iz 绕模型系 x'/y'/z' 轴的惯量（kg·m²；本机 Ix=轴向 0.0021）
 * @param[in] h_rotor 转子角动量合计（模型系，kg·m²/s；沿推力轴放 x 分量）
 * @param[in] mask    使能掩码（kInertiaCompBits）
 * @return 前馈力矩（模型系，N·m），与 Mx/My/Mz（分配器模型系）直接相加
 * @formula Mx_ff = (Iz−Iy)·ωy·ωz + (ωy·hz − ωz·hy)；
 *          My_ff = (Ix−Iz)·ωz·ωx + (ωz·hx − ωx·hz)；
 *          Mz_ff = (Iy−Ix)·ωx·ωy + (ωx·hy − ωy·hx)
 * @ref THY-003 §1.2（欧拉方程）、§5（陀螺进动 M = −Ω×h → 补偿取 +ω×h）
 * @pre  mask 的位定义与调用方一致；ω 与 h **必须同为本函数约定的模型系**
 * @note 交叉项符号以仿真 dynamics.mjs 为锚：g += ω×h、M_ff = +g。
 *       2026-08-10 host 闭环测试曾暴露写成 h×ω 的符号反 bug，勿再"顺直觉"改写。
 *       ★ 固件 gyro/控制律在**存档系**（x=滚转、y=俯仰、z=推力轴），调用本函数
 *       必须走 `computeInertiaCompensationFromArchive()` 做轴置换；直接传
 *       icm_gyro_* 会把 ω 当成模型系，属于 2026-10-23 修正的 C-02 缺陷。
 * @test test_host/test_inertia_decoupling.cpp
 */
inline InertiaCompResult computeInertiaCompensation(
    const float omega[3],
    const float Ix, const float Iy, const float Iz,
    const float h_rotor[3],
    const uint8_t mask)
{
  InertiaCompResult r;
  const float wx = omega[0], wy = omega[1], wz = omega[2];
  if (mask & kInertiaCompGyro)
  {
    // ω×(I·ω) 对角惯量展开（与仿真 dynamics.mjs gx/gy/gz 逐项同构）
    r.Mx_ff = (Iz - Iy) * wy * wz;
    r.My_ff = (Ix - Iz) * wz * wx;
    r.Mz_ff = (Iy - Ix) * wx * wy;
  }
  if (mask & kInertiaCompRotor)
  {
    // ω×h：转子角动量与机体角速度的交叉耦合（陀螺进动效应）。
    // ★ 符号 = 仿真 dynamics.mjs 同构（g += ω×h，M_ff = +g）——
    //   2026-08-10 host 闭环测试曾暴露写成 h×ω 的符号反 bug。
    r.Mx_ff += omega[1] * h_rotor[2] - omega[2] * h_rotor[1];
    r.My_ff += omega[2] * h_rotor[0] - omega[0] * h_rotor[2];
    r.Mz_ff += omega[0] * h_rotor[1] - omega[1] * h_rotor[0];
  }
  return r;
}

/**
 * @brief 由固件**存档系**输入计算**模型系**前馈力矩（轴置换封装，C-02 修正）
 * @param[in] omega_archive 存档系角速度 [rad/s]：x=滚转、y=俯仰、z=推力轴/偏航
 * @param[in] I_roll 绕存档系 x_b 的惯量 [kg·m²]（横向，本机 0.022）
 * @param[in] I_pitch 绕存档系 y_b 的惯量 [kg·m²]（横向，本机 0.022）
 * @param[in] I_yaw_axial 绕存档系 z_b（推力轴）的惯量 [kg·m²]（轴向，本机 0.0021）
 * @param[in] h_along_thrust 转子净角动量 [kg·m²/s]，沿**模型系 +x′（推力方向）**；
 *       等价于存档系 `(0,0,−h)`（因 x′=−z_b），勿在存档系里直接放 +z
 * @param[in] mask 使能掩码
 * @return 模型系前馈力矩（N·m），可直接加到分配器的 Mx/My/Mz
 * @formula 轴置换：ω_x′ = −ω_z、ω_y′ = +ω_y、ω_z′ = +ω_x；
 *          h′ = (h, 0, 0)；I_x′ = I_yaw_axial、I_y′ = I_pitch、I_z′ = I_roll；
 *          M′ = computeInertiaCompensation(ω′, I′, h′, mask)
 * @ref flight_control.cpp mix 层注释（x′=−z_b、y′=+y_b、z′=+x_b，det=+1）
 * @pre 输入必须是存档系（与 icm_gyro_x/y/z、alpha_roll/pitch/yaw 同一套轴名）
 * @note ★ C-02 缺陷根因：旧调用把存档系 ω 直接传给模型系函数，且 h 放在
 *       下标 0；悬停 ω≈0 时无感，机动时前馈落到错误通道（差速/滚转同量级）。
 *       本封装把置换固定在一处，flight_control 不再手写映射；回归见
 *       test_inertia_decoupling.cpp T6（旧直调在此用例下恒 0，新封装非 0）。
 */
inline InertiaCompResult computeInertiaCompensationFromArchive(
    const float omega_archive[3],
    const float I_roll, const float I_pitch, const float I_yaw_axial,
    const float h_along_thrust,
    const uint8_t mask)
{
  // 极矢量轴置换：x′=−z_b、y′=+y_b、z′=+x_b
  const float omega_model[3] = {-omega_archive[2], omega_archive[1], omega_archive[0]};
  // 转子净角动量沿推力方向（模型系 +x′）
  const float h_model[3] = {h_along_thrust, 0.0f, 0.0f};
  // 模型系惯量：x′=轴向、y′=俯仰、z′=滚转
  return computeInertiaCompensation(omega_model, I_yaw_axial, I_pitch, I_roll,
                                    h_model, mask);
}

/* 全部关闭时的快捷判断（mix 层可跳过前馈路径） */
inline bool inertiaCompEnabled(const uint8_t mask)
{
  return (mask & (kInertiaCompGyro | kInertiaCompRotor)) != 0;
}
