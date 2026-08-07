// ============================================================
//  Quat4f.h — 浮点四元数工具（纯平台无关，宿主机测试用）
//
//  2026-08-08 从 TandemVec_AttitudeCtrl.h 抽出：
//  CascadeCtrl 半成品架构已废弃（实机调参唯一入口为
//  include/FlightCtrlParams.h），保留 Quat4f 工具供
//  test_host 刚体仿真测试使用。
// ============================================================
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
