/**
 * @file    test_frame_conventions.cpp
 * @module  固件/宿主测试（坐标系正本自检）
 * @brief   验证 FrameConventions.h 的置换是纯旋转，且两端互逆
 * @details 被测：`TandemVec_FCS/include/FrameConventions.h`
 *          输入：枚举的三分量向量；处理：往返变换 / 基向量映射 / 与解析式比对；
 *          输出：`require()` 失败即 exit(1)。
 * @theory  S_{M→A}：x_A=+z_M、y_A=+y_M、z_A=−x_M；det=+1、S⁻¹=Sᵀ。
 * @ref     MOD-002 §0（全仓坐标系唯一正本）、§0.2（转换）、§0.3（逐量转换表）
 * @frame   同时涉及 {M} 模型系（x_b=推力轴）与 {A} 存档系（z_b=推力轴）
 * @units   N/A（无量纲变换）
 * @note    ★ 本测试**不**断言"与 plant 里的 `{-w.Mz, w.My, -w.Mx}` 相等"。
 *          那个表达式是「几何置换 × 控制通道符号约定」的合成结果：mix 层把
 *          alpha_roll 映射成 Mz'=-Iz·alpha_roll（带负号是控制约定），几何变换再把
 *          Mz' 送到存档系。FrameConventions 只负责几何那一半，两者本来就不该相等。
 *          把控制符号混进几何变换，正是"每处各自决定符号"的老问题。
 * @test    T1 基向量映射（三个轴逐一对上）；T2 往返恒等；T3 正交性；
 *          T4 行列式为 +1；T5 力矩与角速度同律（同一旋转）
 * @see     TandemVec_FCS/include/FrameConventions.h、tools/verify_frame_map.py
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "FrameConventions.h"

namespace
{

void require(bool cond, const char *what)
{
  if (!cond)
  {
    std::fprintf(stderr, "[FAIL] %s\n", what);
    std::exit(1);
  }
}

bool near(float a, float b, float tol = 1.0e-6f)
{
  return std::fabs(a - b) <= tol;
}

} // namespace

int main()
{
  using namespace bfs::frames;

  // T1 基向量映射：模型系各轴在存档系下的像（按 MOD-002 §0.2）
  //     x_M ↦ −z_A、y_M ↦ +y_A、z_M ↦ +x_A
  //     注意 x_M 这一条：z_A 轴定义为 −x_M 方向，所以 x_M 在存档系里是 −z_A。
  {
    const Vec3Archive ex = modelToArchive({1.0f, 0.0f, 0.0f}); // x_M
    require(near(ex.x, 0.0f) && near(ex.y, 0.0f) && near(ex.z, -1.0f),
            "T1: x_M 应映射为 −z_A（z_A 轴定义为 −x_M 方向）");

    const Vec3Archive ey = modelToArchive({0.0f, 1.0f, 0.0f}); // y_M
    require(near(ey.x, 0.0f) && near(ey.y, 1.0f) && near(ey.z, 0.0f),
            "T1: y_M 应映射为 +y_A");

    const Vec3Archive ez = modelToArchive({0.0f, 0.0f, 1.0f}); // z_M
    require(near(ez.x, 1.0f) && near(ez.y, 0.0f) && near(ez.z, 0.0f),
            "T1: z_M 应映射为 +x_A");
  }

  // T2 往返恒等：A→M→A 与 M→A→M 都必须还原原向量
  {
    const Vec3Model m{1.5f, -2.5f, 3.5f};
    const Vec3Model back = archiveToModel(modelToArchive(m));
    require(near(back.x, m.x) && near(back.y, m.y) && near(back.z, m.z),
            "T2: 模型系→存档系→模型系 应还原");

    const Vec3Archive a{-4.0f, 5.5f, -6.25f};
    const Vec3Archive backA = modelToArchive(archiveToModel(a));
    require(near(backA.x, a.x) && near(backA.y, a.y) && near(backA.z, a.z),
            "T2: 存档系→模型系→存档系 应还原");
  }

  // T3 正交性：S · Sᵀ = I（逐元素查，容差 1e-6）
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        float dot = 0.0f;
        for (int k = 0; k < 3; ++k)
        {
          dot += kModelToArchive[i][k] * kModelToArchive[j][k];
        }
        const float expect = (i == j) ? 1.0f : 0.0f;
        require(near(dot, expect), "T3: S·Sᵀ 必须是单位矩阵");
      }
    }
  }

  // T4 行列式 = +1（纯旋转，保持手性）
  {
    const float m00 = kModelToArchive[0][0], m01 = kModelToArchive[0][1],
                m02 = kModelToArchive[0][2];
    const float m10 = kModelToArchive[1][0], m11 = kModelToArchive[1][1],
                m12 = kModelToArchive[1][2];
    const float m20 = kModelToArchive[2][0], m21 = kModelToArchive[2][1],
                m22 = kModelToArchive[2][2];
    const float det = m00 * (m11 * m22 - m12 * m21) -
                      m01 * (m10 * m22 - m12 * m20) +
                      m02 * (m10 * m21 - m11 * m20);
    require(near(det, 1.0f), "T4: det(S) 必须为 +1（否则镜像翻转手性）");
  }

  // T5 同一旋转下，力矩与角速度走同一变换（不是各写一套）
  //    取一个纯"绕模型系 z 轴"的量，变换后必须整体落到存档系 x 轴
  {
    const Vec3Model torque_z{0.0f, 0.0f, 2.0f};   // 模型系纯 Mz
    const Vec3Archive as_archive = modelToArchive(torque_z);
    require(near(as_archive.x, 2.0f) && near(as_archive.y, 0.0f) &&
                near(as_archive.z, 0.0f),
            "T5: 模型系纯 Mz 应整体落到存档系 Mx");

    const Vec3Model omega_z{0.0f, 0.0f, -0.5f};   // 模型系纯 ωz
    const Vec3Archive om_archive = modelToArchive(omega_z);
    require(near(om_archive.x, -0.5f) && near(om_archive.y, 0.0f) &&
                near(om_archive.z, 0.0f),
            "T5: 模型系纯 ωz 应整体落到存档系 ωx（与力矩同律）");
  }

  std::printf("[PASS] test_frame_conventions: 5 组几何性质断言全部通过\n");
  return 0;
}
