/**
 * @file    SingleEdf4VaneAllocator.h
 * @module  固件/控制（单涵道 + X 型四燃气舵机型的控制分配内核，平台无关）
 * @brief   三轴角加速度指令 → 三轴力矩 → 四片燃气舵偏角；含联合等比限幅、
 *          偏航配平、手动标定直通与推进联锁判定
 * @details 输入：机体系三轴角加速度 α（存档系 x/y/z = 滚转/俯仰/偏航）、
 *          各轴转动惯量、单舵舵效系数（N·m/deg）、满偏角、逐舵方向符号、
 *          偏航配平角、手动直通四舵角；
 *          处理：τ = I·α → δ = Hᵀ·τ/(4k) → 逐舵方向符号 → 联合等比限幅；
 *          输出：四舵有向偏角 deg、归一化量 [-1,1]、实际可达力矩与残差、饱和位。
 *          纯函数、无 Arduino/Eigen/动态内存，可 g++ 宿主机直接回归。
 * @theory  力矩需求与舵偏的线性化关系：M = k·H·δ，H 为 3×4 分配矩阵（±1、行正交）；
 *          最小范数解 δ = Hᵀ·τ/(4k)（因 H·Hᵀ = 4I，伪逆退化为转置除以 4）；
 *          联合限幅取 max|δi| 等比缩放，保证力矩方向不畸变。
 *          @ref THY-003 §3（控制分配、伪逆/奇异）；CTL-001 §2（分配策略边界）；
 *          源机型实现见 Universal-EDF `src/airframe_mixer_single_edf_4vane.cpp`。
 * @frame  机体系存档系（x_b=滚转轴、y_b=俯仰轴、z_b=推力轴=偏航/航向轴）；
 *         与源机型 Universal-EDF 的 FRD（x=推力轴）差一次纯旋转置换：
 *         x_EDF = z_b、y_EDF = y_b、z_EDF = −x_b（det=+1），分配矩阵已按此置换重构。
 *         ★ **物理口径 = 标准 NED/FRD**：导航解算与姿态都是 NED/FRD；悬停时机头朝天，
 *         机体 x_FRD 朝天、推力沿 **+x_FRD 朝上**。固件的"存档系"只是把机头方向
 *         命名为 z_b 的负向，固定对应关系：z_b = −x_FRD、x_b = z_FRD、y_b = y_FRD。
 *         因此三轴力矩的两种叫法一一对应（悬停语义）：
 *           绕 x_FRD（推力轴，世界航向）= 绕 z_b（存档偏航）→ 矩阵行 Mz → 舵 {1,3}vs{2,4}
 *           绕 y_FRD（右）        = 绕 y_b（存档俯仰）→ 矩阵行 My → 舵 {1,2}vs{3,4}
 *           绕 z_FRD（下）        = 绕 x_b（存档滚转）→ 矩阵行 Mx → 舵 {1,4}vs{2,3}
 *         倾角判据 acos|R33| 就是"推力轴（x_FRD）偏离天向的夹角"：竖直=0°、倒扣=180°
 *         （R33 = 1−2(qx²+qy²) 是 z_b 在天向的投影，|R33| 与 x_FRD 的天向投影同值）。
 * @units  α rad/s²、惯量 kg·m²、力矩 N·m、舵效 N·m/deg、舵偏角 deg
 * @note   ★ 舵效系数 k 是"额定推力点"的线性化斜率，真实燃气舵力矩 ∝ 喷流动压 ∝ 推力，
 *          低推力时舵效弱、高推力时强；本内核按固定 k 实现（与源机型一致的已知简化），
 *          标定后如需补偿，应由调用方按当前推力比例缩放后传入 k。
 *         ★ 分配矩阵的符号来自轴置换推导，实物仍必须按标定流程逐舵核对
 *          （VANE_DIR 为逐舵修正入口）；符号错误的后果是"越纠偏越偏"。
 * @test   test_host/test_single_edf_4vane.cpp
 * @see    include/ActuatorOutputMap.h、include/VehicleProfile.h、
 *         include/FlightCtrlParams.h、src/flight_control.cpp
 */

#ifndef TANDEMVEC_SINGLE_EDF_4VANE_ALLOCATOR_H
#define TANDEMVEC_SINGLE_EDF_4VANE_ALLOCATOR_H

#include <cmath>
#include <cstdint>

namespace bfs
{

/** @brief 单涵道机型的燃气舵数量（X 型四舵） */
constexpr uint8_t kSingleEdfVaneCount = 4;

/**
 * @brief 分配矩阵 H（行 = 存档系三轴力矩 Mx/My/Mz，列 = 舵1..舵4）
 * @details 由源机型 Universal-EDF 的实测矩阵（其轴序 roll/pitch/yaw = 绕 x/y/z_EDF）
 *          经轴置换 x_EDF→z_b、y_EDF→y_b、z_EDF→−x_b 得到：
 *            Mx（滚转，绕 x_b） = −M_yaw_EDF  → [−1, +1, −1, +1]
 *            My（俯仰，绕 y_b） = +M_pitch_EDF → [+1, +1, −1, −1]
 *            Mz（偏航，绕 z_b） = +M_roll_EDF  → [−1, +1, +1, −1]
 *          行两两内积为 0（行正交），故三轴可独立分配、互不耦合。
 *          四舵在存档系的方位：舵1 (+x_b,+y_b)、舵2 (−x_b,+y_b)、舵3 (−x_b,−y_b)、
 *          舵4 (+x_b,−y_b)——飞控与传感器安装与共轴 VTVL 完全一致（用户 2026-11-05
 *          确认，同机体同安装）。对应分组：俯仰 {1,2}vs{3,4}、滚转 {1,4}vs{2,3}、
 *          偏航 {1,3}vs{2,4}（源机型台架实测 2026-09-23）。
 *          ★ 该置换只保证"轴语义对齐"，不保证实物符号；实物符号由 vane_dir 标定。
 * @ref   Universal-EDF `docs/修改方案-单涵道四燃气舵构型.md` §4.1（台架实测矩阵）；
 *        TandemVec_FCS `AGENTS.md` §坐标系与执行器（存档系 z_b=推力轴）
 */
static constexpr float kSingleEdf4VaneMatrix[3][kSingleEdfVaneCount] = {
    {-1.0f, +1.0f, -1.0f, +1.0f}, // Mx 滚转 = −M_yaw_EDF
    {+1.0f, +1.0f, -1.0f, -1.0f}, // My 俯仰 = +M_pitch_EDF
    {-1.0f, +1.0f, +1.0f, -1.0f}, // Mz 偏航 = +M_roll_EDF
};

/**
 * @brief 结构体说明：单涵道机型的可标定参数（由 FlightCtrlParams 提供实例）
 * @note  字段默认值只作结构体自洽示例，真实默认值在 FlightCtrlParams.h 的
 *        SingleEdf4VaneParams 中锁定（static_assert 逐位比对）。
 */
struct SingleEdf4VaneAllocationParams
{
    float vane_max_deg = 20.0f;             // 单舵机械满偏角 [deg]
    float vane_moment_nm_per_deg = 0.02f;   // 额定推力点舵效 [N·m/deg]（MODEL-DEFAULT）
    float inertia_roll_kgm2 = 0.02f;        // 滚转轴惯量 [kg·m²]
    float inertia_pitch_kgm2 = 0.02f;       // 俯仰轴惯量 [kg·m²]
    float inertia_yaw_kgm2 = 0.03f;         // 偏航轴惯量 [kg·m²]
    float yaw_trim_deg = 0.0f;              // 涵道反扭矩配平（舵角域）[deg]
    float vane_dir[kSingleEdfVaneCount] = {+1.0f, +1.0f, +1.0f, +1.0f}; // 逐舵物理方向
};

/** @brief 分配输入 */
struct SingleEdf4VaneAllocationInput
{
    // 控制层输出：机体系三轴角加速度（存档系，rad/s²）
    float alpha_roll_radps2 = 0.0f;
    float alpha_pitch_radps2 = 0.0f;
    float alpha_yaw_radps2 = 0.0f;
    // 手动/地面标定直通：true 时忽略 α，直接使用 direct_vane_deg（单位 deg）
    bool direct_vane_enabled = false;
    float direct_vane_deg[kSingleEdfVaneCount] = {0.0f, 0.0f, 0.0f, 0.0f};
};

/** @brief 分配输出 */
struct SingleEdf4VaneAllocationOutput
{
    bool valid = false;        // 输入有限性/参数合法性通过
    bool saturated = false;    // 触发联合等比限幅（控制权限不足）
    float vane_deg[kSingleEdfVaneCount] = {0.0f, 0.0f, 0.0f, 0.0f};  // 有向舵偏角
    float vane_norm[kSingleEdfVaneCount] = {0.0f, 0.0f, 0.0f, 0.0f}; // 归一化 [-1,1]
    float torque_cmd_nm[3] = {0.0f, 0.0f, 0.0f}; // 期望力矩 Mx/My/Mz
    float torque_achieved_nm[3] = {0.0f, 0.0f, 0.0f}; // 限幅后实际可达力矩
    float residual_nm = 0.0f;  // 三轴残差最大值（饱和时 >0）
};

inline bool singleEdfVaneFinite(const float v)
{
    return std::isfinite(v);
}

inline float singleEdfVaneClamp(const float v, const float lo, const float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/**
 * @brief 求解四舵偏角：α → 力矩 → 分配 → 逐舵方向 → 联合等比限幅
 * @param[in]  in  三轴角加速度或手动直通四舵角
 * @param[in]  p   机型可标定参数（满偏角/舵效/惯量/配平/逐舵方向）
 * @param[out] out 四舵偏角与归一化量、可达力矩、饱和位
 * @formula τ_j = I_j·α_j（j = 滚转/俯仰/偏航）；
 *          d_j = τ_j / (4k)（k = 单舵舵效 N·m/deg）；
 *          δ_i = dir_i · (Σ_j H[j][i]·d_j)；
 *          max|δ_i| > δ_max 时全体乘 δ_max/max|δ_i|（方向不变、幅值减小）
 * @ref THY-003 §3；Universal-EDF 混控器（3×4 分配与联合限幅）
 * @note ① 直通模式下 δ_i 直接取 direct_vane_deg，仅做满偏角与我方向符号的限幅；
 *       ② 配平角只叠加到偏航（Mz）通道，等价于在 δ 上叠加 Hᵀ·[0,0,trim]/(4k)；
 *       ③ 任何输入非有限或参数非法时 valid=false 且四舵输出为 0（回中位），
 *          不保留上一拍值——避免故障时舵面停在偏置位置。
 */
inline SingleEdf4VaneAllocationOutput
allocateSingleEdf4Vane(const SingleEdf4VaneAllocationInput &in,
                       const SingleEdf4VaneAllocationParams &p)
{
    SingleEdf4VaneAllocationOutput out;

    // ---- 参数合法性（满偏角/舵效/惯量必须为正；方向符号只能取 ±1）----
    if (!singleEdfVaneFinite(p.vane_max_deg) || !(p.vane_max_deg > 0.0f) ||
        !singleEdfVaneFinite(p.vane_moment_nm_per_deg) ||
        !(p.vane_moment_nm_per_deg > 0.0f) ||
        !singleEdfVaneFinite(p.inertia_roll_kgm2) || !(p.inertia_roll_kgm2 > 0.0f) ||
        !singleEdfVaneFinite(p.inertia_pitch_kgm2) || !(p.inertia_pitch_kgm2 > 0.0f) ||
        !singleEdfVaneFinite(p.inertia_yaw_kgm2) || !(p.inertia_yaw_kgm2 > 0.0f) ||
        !singleEdfVaneFinite(p.yaw_trim_deg))
    {
        return out;
    }

    // ---- 手动直通：直接给四舵角，只做方向符号与满偏角限幅 ----
    // 语义统一：out.vane_deg = 实际下发给舵机的有向角（已含逐舵方向符号），
    //           out.vane_norm = vane_deg / 满偏角，供输出层换占空比。
    if (in.direct_vane_enabled)
    {
        float peak = 0.0f;
        for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
        {
            if (!singleEdfVaneFinite(in.direct_vane_deg[i]) ||
                !singleEdfVaneFinite(p.vane_dir[i]))
            {
                return out;
            }
            out.vane_deg[i] = in.direct_vane_deg[i] *
                              ((p.vane_dir[i] >= 0.0f) ? 1.0f : -1.0f);
            const float magnitude = std::fabs(out.vane_deg[i]);
            if (magnitude > peak)
            {
                peak = magnitude;
            }
        }
        if (peak > p.vane_max_deg)
        {
            const float scale = p.vane_max_deg / peak;
            out.saturated = true;
            for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
            {
                out.vane_deg[i] *= scale;
            }
        }
        for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
        {
            out.vane_norm[i] = out.vane_deg[i] / p.vane_max_deg;
        }
        out.valid = true;
        out.residual_nm = 0.0f;
        return out;
    }

    // ---- α 有限性检查 ----
    if (!singleEdfVaneFinite(in.alpha_roll_radps2) ||
        !singleEdfVaneFinite(in.alpha_pitch_radps2) ||
        !singleEdfVaneFinite(in.alpha_yaw_radps2))
    {
        return out;
    }

    // ---- 1) α → 三轴力矩（存档系，τ = I·α）----
    const float torque[3] = {
        p.inertia_roll_kgm2 * in.alpha_roll_radps2,
        p.inertia_pitch_kgm2 * in.alpha_pitch_radps2,
        p.inertia_yaw_kgm2 * in.alpha_yaw_radps2,
    };
    out.torque_cmd_nm[0] = torque[0];
    out.torque_cmd_nm[1] = torque[1];
    out.torque_cmd_nm[2] = torque[2];

    // ---- 2) 最小范数分配：δ = Hᵀ·τ/(4k)（H·Hᵀ = 4I）----
    // 偏航配平以"等效力矩"形式叠加到 Mz：τ_trim = 4k·trim/1（使 δ 的偏航分量恰为
    // trim 角）。这里直接在舵角域叠加更直观，见下方第 3 步。
    const float inv_scale = 1.0f / (4.0f * p.vane_moment_nm_per_deg);
    float delta[kSingleEdfVaneCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
    {
        float sum = 0.0f;
        for (uint8_t j = 0; j < 3; ++j)
        {
            sum += kSingleEdf4VaneMatrix[j][i] * torque[j];
        }
        delta[i] = sum * inv_scale;
    }

    // ---- 3) 偏航配平：只在偏航通道叠加一个常值舵角（与源机型 YAW_TRIM_DEG 同语义）----
    // 配平角按偏航行 H[2][i] 分布，使四舵的合成力矩只落在 Mz 上。
    if (p.yaw_trim_deg != 0.0f)
    {
        for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
        {
            delta[i] += kSingleEdf4VaneMatrix[2][i] * p.yaw_trim_deg;
        }
    }

    // ---- 4) 逐舵物理方向符号 ----
    float peak = 0.0f;
    for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
    {
        if (!singleEdfVaneFinite(delta[i]) || !singleEdfVaneFinite(p.vane_dir[i]))
        {
            return out;
        }
        delta[i] *= (p.vane_dir[i] >= 0.0f) ? 1.0f : -1.0f;
        const float magnitude = std::fabs(delta[i]);
        if (magnitude > peak)
        {
            peak = magnitude;
        }
    }

    // ---- 5) 四舵联合等比限幅（保持力矩方向不畸变）----
    float scale = 1.0f;
    if (peak > p.vane_max_deg)
    {
        scale = p.vane_max_deg / peak;
        out.saturated = true;
    }
    for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
    {
        out.vane_deg[i] = delta[i] * scale;
        out.vane_norm[i] = out.vane_deg[i] / p.vane_max_deg;
    }

    // ---- 6) 限幅后实际可达力矩与残差（供控制层抗饱和/遥测）----
    for (uint8_t j = 0; j < 3; ++j)
    {
        float achieved = 0.0f;
        for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
        {
            // 反算时把逐舵方向符号还原回矩阵坐标系
            const float signed_delta =
                out.vane_deg[i] * ((p.vane_dir[i] >= 0.0f) ? 1.0f : -1.0f);
            achieved += kSingleEdf4VaneMatrix[j][i] * signed_delta;
        }
        out.torque_achieved_nm[j] = achieved * p.vane_moment_nm_per_deg;
    }
    float residual_max = 0.0f;
    for (uint8_t j = 0; j < 3; ++j)
    {
        const float residual = std::fabs(out.torque_achieved_nm[j] - torque[j]);
        if (residual > residual_max)
        {
            residual_max = residual;
        }
    }
    out.residual_nm = residual_max;
    out.valid = true;
    return out;
}

/**
 * @brief 静态自检：分配矩阵三行两两正交（编译期证明，防止后续改动破坏三轴解耦）
 * @note 行内积必须为 0；H·Hᵀ = 4I 是"最小范数解退化为转置"的前提。
 */
constexpr bool singleEdfVaneRowsOrthogonal()
{
    for (uint8_t a = 0; a < 3; ++a)
    {
        for (uint8_t b = a + 1; b < 3; ++b)
        {
            float dot = 0.0f;
            for (uint8_t i = 0; i < kSingleEdfVaneCount; ++i)
            {
                dot += kSingleEdf4VaneMatrix[a][i] * kSingleEdf4VaneMatrix[b][i];
            }
            if (dot != 0.0f)
            {
                return false;
            }
        }
    }
    return true;
}

static_assert(singleEdfVaneRowsOrthogonal(),
              "单涵道四舵分配矩阵必须三行正交（H·Hᵀ = 4I），否则三轴耦合");

} // namespace bfs

#endif // TANDEMVEC_SINGLE_EDF_4VANE_ALLOCATOR_H
