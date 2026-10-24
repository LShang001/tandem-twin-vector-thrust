/**
 * @file    GenericControlAllocation.h
 * @module  固件/控制（N 执行器通用控制分配，平台无关）
 * @brief   三轴力矩 + N 个控制输入：加权最小范数分配、限幅与秩亏检测；
 *          为多旋翼 VTVL、液体火箭（摆座/RCS/多发动机）提供通用分配内核
 * @details 输入：3×N 效率矩阵 B、期望力矩 M、每输入上下限/权重、正则化系数；
 *          处理：A = B·W·Bᵀ，解 u = W·Bᵀ·A⁻¹·M，逐通道限幅；
 *          输出：N 个控制量、饱和标记、秩亏标记、残差。
 *          不依赖 Arduino/Eigen，无动态内存；当前固件 legacy 路径仍用
 *          `TandemVec_ControlAllocation.h`，本模块先作为新机型扩展内核。
 * @theory  冗余/过驱动控制分配：加权最小范数解
 *          u = W Bᵀ (B W Bᵀ + λI)⁻¹ M；N=3 且 B 可逆时退化为 B⁻¹M。
 *          @ref THY-003 §3（控制分配、伪逆/奇异）；CTL-001 §2（本构型旧路径为何
 *          不使用 B⁺）；REF-105（过驱动 eVTOL 分配）思路。
 * @frame   模型系三轴力矩（行序 Mx,My,Mz）；输入 u 为各执行器归一化量
 * @units   B 单位随输入而定（N·m/单位输入），M 为 N·m，u 为执行器单位
 * @note    ★ 本模块只做“数学分配”，不替代机型专属的差速开方、τm 观测、
 *          零油门归中、轴置换等行为红线；接入前必须用旧路径做等价回归。
 *          秩亏/饱和只报告不自动再分配；后续可加 active-set。
 * @test    test_host/test_generic_allocation.cpp
 * @see     TandemVec_ControlAllocation.h、AirframeModel.h、VehicleProfile.h
 */

#ifndef TANDEMVEC_GENERIC_CONTROL_ALLOCATION_H
#define TANDEMVEC_GENERIC_CONTROL_ALLOCATION_H

#include <cmath>
#include <cstdint>

namespace bfs
{

constexpr uint8_t kMaxAllocationInputs = 12;

struct GenericAllocationInput
{
    float B[3][kMaxAllocationInputs] = {}; // 行 Mx/My/Mz，列输入
    float moment[3] = {0.0f, 0.0f, 0.0f};
    float min_u[kMaxAllocationInputs] = {};
    float max_u[kMaxAllocationInputs] = {};
    float weight[kMaxAllocationInputs] = {};
    uint8_t n = 0;
    float regularization = 1.0e-6f;
};

struct GenericAllocationResult
{
    bool valid = false;
    bool rank_deficient = false;
    uint8_t n = 0;
    float u[kMaxAllocationInputs] = {};
    bool saturated[kMaxAllocationInputs] = {};
    float residual_norm = 0.0f;
};

inline bool invert3x3(const float a[3][3], float inv[3][3], float *det_out)
{
    const float det =
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
        a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
        a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (det_out != nullptr)
    {
        *det_out = det;
    }
    if (!std::isfinite(det) || std::fabs(det) < 1.0e-20f)
    {
        return false;
    }
    const float inv_det = 1.0f / det;
    inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
    inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;
    return true;
}

/**
 * @brief 通用加权最小范数分配
 * @param[in] in 3×N 效率矩阵、力矩、限幅/权重、n、正则化
 * @return 分配结果；输入非法时 valid=false
 * @formula A = Σ_j w_j·b_j·b_jᵀ；u_j = w_j·b_jᵀ·(A+λI)⁻¹·M；
 *          然后 u_j ← clamp(u_j, min,max)
 * @ref THY-003 §3；REF-105（过驱动分配）
 * @note 饱和只标记，不做二次再分配；需要严格可达性时上层应降级/限幅目标力矩。
 *       输入 n<1 或非有限/权重≤0 时返回无效，不写输出。
 */
inline GenericAllocationResult allocateGeneric(const GenericAllocationInput &in)
{
    GenericAllocationResult out;
    if (in.n == 0U || in.n > kMaxAllocationInputs ||
        !std::isfinite(in.moment[0]) || !std::isfinite(in.moment[1]) ||
        !std::isfinite(in.moment[2]))
    {
        return out;
    }
    out.n = in.n;
    out.valid = true;

    float A[3][3] = {};
    for (uint8_t j = 0; j < in.n; ++j)
    {
        if (!(in.weight[j] > 0.0f) || !std::isfinite(in.weight[j]) ||
            !(in.min_u[j] <= in.max_u[j]) ||
            !std::isfinite(in.min_u[j]) || !std::isfinite(in.max_u[j]))
        {
            out.valid = false;
            return out;
        }
        for (uint8_t r = 0; r < 3; ++r)
        {
            if (!std::isfinite(in.B[r][j]))
            {
                out.valid = false;
                return out;
            }
            for (uint8_t c = 0; c < 3; ++c)
            {
                A[r][c] += in.weight[j] * in.B[r][j] * in.B[c][j];
            }
        }
    }

    // 3×3 A 的行列式用于秩亏诊断；正则化只用于数值稳定，不改变主解。
    const float det =
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    out.rank_deficient = !std::isfinite(det) || std::fabs(det) < 1.0e-12f;

    const float lambda = std::fmax(in.regularization, 0.0f);
    for (uint8_t i = 0; i < 3; ++i)
    {
        A[i][i] += lambda;
    }
    float invA[3][3];
    if (!invert3x3(A, invA, nullptr))
    {
        out.valid = false;
        return out;
    }

    // rhs = A⁻¹ M；u_j = w_j·b_jᵀ·rhs
    float rhs[3] = {};
    for (uint8_t r = 0; r < 3; ++r)
    {
        for (uint8_t c = 0; c < 3; ++c)
        {
            rhs[r] += invA[r][c] * in.moment[c];
        }
    }

    float residual[3] = {};
    for (uint8_t j = 0; j < in.n; ++j)
    {
        float u = 0.0f;
        for (uint8_t r = 0; r < 3; ++r)
        {
            u += in.B[r][j] * rhs[r];
        }
        u *= in.weight[j];
        if (!std::isfinite(u))
        {
            out.valid = false;
            return out;
        }
        out.u[j] = std::fmin(std::fmax(u, in.min_u[j]), in.max_u[j]);
        out.saturated[j] = (out.u[j] != u);
    }
    for (uint8_t r = 0; r < 3; ++r)
    {
        float achieved = 0.0f;
        for (uint8_t j = 0; j < in.n; ++j)
        {
            achieved += in.B[r][j] * out.u[j];
        }
        residual[r] = achieved - in.moment[r];
    }
    out.residual_norm = std::sqrt(residual[0] * residual[0] +
                                  residual[1] * residual[1] +
                                  residual[2] * residual[2]);
    return out;
}

} // namespace bfs

#endif // TANDEMVEC_GENERIC_CONTROL_ALLOCATION_H
