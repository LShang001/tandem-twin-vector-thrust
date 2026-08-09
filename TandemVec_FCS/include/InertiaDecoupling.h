#pragma once

#include <cmath>
#include <cstdint>

/*
 * InertiaDecoupling.h — 惯量逆解交叉耦合前馈（通用层，2026-08-10）
 * =================================================================
 * 完整刚体转动动力学（与仿真 dynamics.mjs 同构，红线同源）：
 *     I·ω̇ = M − ω×(I·ω) − ω×h_rotor
 * 惯量逆解前馈（补偿项，加到 I·α 上得到期望力矩）：
 *     M_ff = ω×(I·ω) + ω×h_rotor
 *
 * 约定：
 *   - 全部输入/输出为机体系（NED：x 前 / y 右 / z 下）
 *   - Ix/Iy/Iz = 绕对应机体系轴的惯量（与仿真/回归基线一致；
 *     本机 Ix=轴向小惯量 0.0021、Iy=Iz=横向 0.022 → gx 项恒 0，细长杆自然结果）
 *   - h_rotor = 转子角动量合计（机体系分量；纵列双发前后转子反向，
 *     净角动量沿推力轴 = 机体系 x 分量，见仿真 hv.x = Jp(wf·cf − wt·ct)）
 *   - 使能掩码按位开关（默认全开；悬停 ω≈0 时交叉项≈0 无副作用，
 *     机动时直接受益；符号错误=正反馈风险 → host 测试对照刚体模型锁定，
 *     实机 A/B 可用参数 inertia_comp_mask 在线关闭）
 *
 * 本头文件为纯函数、零 Arduino 依赖，可 g++ 宿主机直接回归
 * （沿用 ins_gnss_dynamic_weight.h 模式）。
 */

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

/*
 * 计算交叉耦合前馈力矩。
 *
 * @param omega   当前角速度（机体系，rad/s；来自 gyro）
 * @param Ix/Iy/Iz 绕机体系 x/y/z 轴的惯量（kg·m²）
 * @param h_rotor 转子角动量合计（机体系，kg·m²/s；无转子时全 0）
 * @param mask    使能掩码（kInertiaCompBits）
 * @return 前馈力矩（机体系，N·m）；调用方负责轴置换到分配器坐标系
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

/* 全部关闭时的快捷判断（mix 层可跳过前馈路径） */
inline bool inertiaCompEnabled(const uint8_t mask)
{
  return (mask & (kInertiaCompGyro | kInertiaCompRotor)) != 0;
}
