/**
 * @file    Quat4f.h
 * @module  固件/姿态数学（宿主测试用浮点四元数工具）
 * @brief   提供 float 四元数的 Hamilton 乘积、共轭与归一化，供 test_host 刚体仿真使用
 * @details 输入/输出均为 {w,x,y,z} 顺序的单位四元数或普通四元数；
 *          处理：纯内联数学，无 Arduino/STM32 依赖；不读写任何全局状态。
 *          不变量：qNorm 结果模长为 1（模长平方 <1e-12 时回退单位四元数）。
 * @theory  Hamilton 乘积 q1⊗q2 与共轭 q*（单位四元数下 q*=q⁻¹）
 *          @ref REF-120 §9.2.1 (9.2.2)(9.2.5) p.253；
 *          分量顺序与右乘语义 @ref MOD-002 §3.1/§3.2
 * @frame   四元数语义由调用方约定（本文件不自带 B/N 系假设），分量顺序 [w,x,y,z]
 * @units   N/A（无量纲），角度隐含在四元数内
 * @note    2026-08-08 从 `TandemVec_AttitudeCtrl.h` 抽出：CascadeCtrl 半成品架构
 *          已废弃（实机调参唯一入口为 `FlightCtrlParams.h`），仅保留工具供测试。
 *          与 `QuaternionMath.h` 的 double 版并存，勿混用两者类型。
 * @see     `QuaternionMath.h`、`test_host/` 刚体仿真测试
 */

#pragma once
#include <cmath>

// 浮点四元数（分量顺序 [w, x, y, z]）
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
