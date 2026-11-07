/**
 * @file    FrameConventions.h
 * @module  固件/坐标系正本（全仓唯一的轴置换定义）
 * @brief   存档系 `{A}` 与模型系 `{M}` 之间的唯一转换实现，附编译期自检
 * @details 输入：三元素向量（力/力矩/角速度/位置/速度，同一旋转下同律变换）；
 *          处理：按 MOD-002 §0.2 的置换矩阵做一次基变换；
 *          输出：另一套坐标系下的同名分量。
 *          本文件是**代码正本**：任何模块需要跨系转换时调用这里的函数，
 *          不得各自手写 `-w.Mz, w.My, -w.Mx` 之类的就地表达式。
 * @theory  两套机体系是同一机械布局的纯旋转置换：
 *              x_A = +z_M、y_A = +y_M、z_A = −x_M
 *          写成矩阵 S_{M→A}（第 i 行第 j 列 = 新轴 i 在原轴 j 上的投影）：
 *              [ 0  0  +1 ]
 *              [ 0 +1   0 ]
 *              [−1  0   0 ]
 *          det(S) = +1 ⇒ 纯旋转、右手系保持；S⁻¹ = Sᵀ。
 * @ref     MOD-002 §0（全仓坐标系唯一正本）、§0.2（转换）、§0.3（逐量转换表）
 * @frame   本文件定义的就是这两套系本身：`{A}` 存档系 z_b=推力轴（固件口径）、
 *          `{M}` 模型系 x_b=推力轴（仿真/论文口径）
 * @units   N/A（纯线性变换，输入输出同量纲）
 * @note    ★ 为什么要有这个文件（2026-11-07 用户指令「坐标系一开始就严格确认规范」）：
 *          此前转换式散落在 plant、allocator、测试各处，写法各异（有的写在
 *          `const float Mb[3] = {-w.Mz, w.My, -w.Mx};`，有的内联进表达式），
 *          单涵道 SIL 就是因为把"源机型 roll 绕推力轴"平移到本固件而发散。
 *          统一到这里以后，转换只有一个出处，改一次全仓生效。
 *          ★ 本文件只做**坐标变换**，不做物理判断：调用方必须自己知道传进来的是
 *          哪套系的量、要的是哪套系。
 *          ★★ 重要边界（2026-11-07 实测，切勿"顺手统一"）：
 *          纯几何变换 modelToArchive 给出 Mx_A = +Mz_M，而两处 SIL plant 写的是
 *          `Mb[0] = -w.Mz`，与纯几何**差一个符号**。这个符号不能删——
 *          把 fw_sil_plant_6dof.h 临时改成纯几何 {+w.Mz, w.My, -w.Mx} 后，
 *          纵列 SIL 手动阶跃在 t=11 s 打到 −84°、t=12 s −142°、t=15 s 发散成 NaN；
 *          恢复 −w.Mz 后回到 5.86° 稳定。
 *          原因是固件 mix 层约定 `Mx' = -Ix·alpha_roll`（模型系 Mx' 自带负号），
 *          plant 侧的 −w.Mz 与它配对构成完整符号链。**几何**归本文件，
 *          **控制通道正负**归 mix 层，那一行是两个语义的合成点。
 *          所以：需要纯几何变换时调用本文件；构造"模型系力学量 → 机体力矩"
 *          的力链时，符号必须连同 mix 层约定一起核对，不能只看几何。
 * @see     TandemVec_FCS/tools/verify_frame_map.py（同一置换的独立推导校验）、
 *          docs/04-数学建模/MOD-002-坐标系与符号约定.md §0
 * @test    test_host/test_frame_conventions.cpp（S·Sᵀ=I、det=+1、往返恒等、
 *          与历史内联写法 `{-Mz, My, -Mx}` 逐项一致）
 */
#pragma once

#include <cmath>
#include <cstddef>

namespace bfs
{
namespace frames
{

/**
 * @brief 置换矩阵 S_{M→A}：模型系 → 存档系
 * @details S[i][j] = 新轴 i 在原轴 j 上的投影。
 *          行 0（x_A）= 原 z_M；行 1（y_A）= 原 y_M；行 2（z_A）= −原 x_M。
 */
constexpr float kModelToArchive[3][3] = {
    {0.0f, 0.0f, 1.0f},
    {0.0f, 1.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f},
};

/// 模型系 `{M}` 下的三分量向量（力/力矩/角速度/位置/速度均适用）
struct Vec3Model
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/// 存档系 `{A}` 下的三分量向量
struct Vec3Archive
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/**
 * @brief 模型系 → 存档系：x_A=+z_M、y_A=+y_M、z_A=−x_M
 */
inline Vec3Archive modelToArchive(const Vec3Model &m)
{
  Vec3Archive a;
  a.x = m.z;
  a.y = m.y;
  a.z = -m.x;
  return a;
}

/**
 * @brief 存档系 → 模型系：x_M=−z_A、y_M=+y_A、z_M=+x_A
 */
inline Vec3Model archiveToModel(const Vec3Archive &a)
{
  Vec3Model m;
  m.x = -a.z;
  m.y = a.y;
  m.z = a.x;
  return m;
}

// ============================================================
//  推力轴的竖直投影（倾角余弦）
// ============================================================

/**
 * @brief 推力轴在 NED 竖直方向的投影分量，即姿态矩阵元素 R33
 * @details 存档系里推力轴是 z_b，所以推力轴竖直投影就是 R33：
 *              R33 = 1 − 2(qx² + qy²)
 *          四元数分量顺序 {w,x,y,z}（与 QuaternionMath.h、AHRS_Packet 一致）。
 *          **模型系**里推力轴是 x_b，对应的是 R13（= 2(xz + wy)），两者是同一个
 *          物理量在两套轴名下的表达——见 MOD-002 §1.2。
 * @param qx 四元数 x 分量；@param qy 四元数 y 分量
 * @return 投影余弦，范围 [−1, 1]；悬停（机头朝天）时接近 ±1
 * @note 本函数只做几何计算，不含任何钳位或绝对值——调用方按需处理。
 */
inline float thrustAxisVerticalCos(float qx, float qy)
{
  return 1.0f - 2.0f * (qx * qx + qy * qy);
}

/**
 * @brief 推力轴竖直投影的绝对值，并钳到下限（供高度/斜距换算的除法使用）
 * @details 倾角接近 90° 时 R33→0，斜距换算会出现除零；固件侧统一钳 0.1
 *          （对应约 84° 倾角）。把「取绝对值 + 钳位」收在一处，避免每个调用点
 *          各写一遍 clamp 阈值。
 * @param floor 钳位下限，默认 0.1（与 sensor_peripheral.cpp 的历史取值一致）
 */
inline float thrustAxisVerticalCosClamped(float qx, float qy, float floor = 0.1f)
{
  const float c = std::fabs(thrustAxisVerticalCos(qx, qy));
  return (c < floor) ? floor : c;
}

// ============================================================
//  编译期自检：置换必须是纯旋转（det=+1）且正交（S·Sᵀ=I）
// ============================================================
namespace detail
{

constexpr float det3(const float m[3][3])
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

constexpr float dotRow(const float m[3][3], int i, int j)
{
  return m[i][0] * m[j][0] + m[i][1] * m[j][1] + m[i][2] * m[j][2];
}

} // namespace detail

static_assert(detail::det3(kModelToArchive) > 0.999f &&
                  detail::det3(kModelToArchive) < 1.001f,
              "S_{M→A} 必须是纯旋转（det=+1），否则会翻转手性");

static_assert(detail::dotRow(kModelToArchive, 0, 0) > 0.999f &&
                  detail::dotRow(kModelToArchive, 0, 1) > -0.001f &&
                  detail::dotRow(kModelToArchive, 0, 1) < 0.001f &&
                  detail::dotRow(kModelToArchive, 0, 2) > -0.001f &&
                  detail::dotRow(kModelToArchive, 0, 2) < 0.001f,
              "S_{M→A} 必须正交（行两两正交、模长为 1）");

} // namespace frames
} // namespace bfs
