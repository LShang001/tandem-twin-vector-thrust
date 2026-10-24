/**
 * @file    test_generic_allocation.cpp
 * @module  固件/宿主回归测试（fw-tests：N 执行器通用控制分配）
 * @brief   验证 3×N 加权最小范数分配、限幅、冗余权重、秩亏与输入拒绝
 * @details 被测：`include/GenericControlAllocation.h`（平台无关）。
 *          输入：合成 B/M/限幅/权重；处理：allocateGeneric；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_generic_allocation.cpp
 *                -o bin/ga
 * @theory  u = W Bᵀ (B W Bᵀ + λI)⁻¹ M，逐通道限幅；N=3 可逆时退化为 B⁻¹M
 *          @ref THY-003 §3；CTL-001 §2；REF-105（过驱动分配）
 * @frame   模型系三轴力矩；输入为执行器归一化量
 * @units   B 随输入单位、M N·m、u 执行器单位
 * @note    这是分配数学层测试；未接入固件 legacy BTRUE 路径，不能替代
 *          test_tandemvec_allocation/SIL 的行为回归。
 * @test    A1 对角 3×3；A2 4 输入冗余；A3 权重；A4 饱和；A5 秩亏；
 *          A6 非法输入；A7 通用 3×3≡legacy BTRUE；A8 3×3 逆
 * @see     include/GenericControlAllocation.h、TandemVec_ControlAllocation.h
 */
#include <cmath>
#include <cstdio>

#include "GenericControlAllocation.h"
#include "TandemVec_ControlAllocation.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

static bfs::GenericAllocationInput makeInput(uint8_t n)
{
  bfs::GenericAllocationInput in;
  in.n = n;
  for (uint8_t i = 0; i < n; ++i)
  {
    in.min_u[i] = -10.0f;
    in.max_u[i] = 10.0f;
    in.weight[i] = 1.0f;
  }
  in.regularization = 1.0e-6f;
  return in;
}

int main()
{
  // A1 对角 3×3：u = B⁻¹M。
  {
    auto in = makeInput(3);
    in.B[0][0] = 1.0f;
    in.B[1][1] = 2.0f;
    in.B[2][2] = 3.0f;
    in.moment[0] = 1.0f;
    in.moment[1] = 2.0f;
    in.moment[2] = 3.0f;
    const auto r = bfs::allocateGeneric(in);
    CHECK(r.valid, "A1 有效");
    CHECK(!r.rank_deficient, "A1 非秩亏");
    CHECK(std::fabs(r.u[0] - 1.0f) < 1e-4 && std::fabs(r.u[1] - 1.0f) < 1e-4 &&
              std::fabs(r.u[2] - 1.0f) < 1e-4,
          "A1 解为 (1,1,1)");
    CHECK(r.residual_norm < 1e-5f, "A1 残差≈0");
  }

  // A2 4 输入冗余：期望力矩可由多组输入实现，残差应很小。
  {
    auto in = makeInput(4);
    in.B[0][0] = 1.0f; in.B[0][3] = 1.0f;
    in.B[1][1] = 1.0f; in.B[1][3] = 1.0f;
    in.B[2][2] = 1.0f; in.B[2][3] = 1.0f;
    in.moment[0] = 2.0f; in.moment[1] = 3.0f; in.moment[2] = 4.0f;
    const auto r = bfs::allocateGeneric(in);
    CHECK(r.valid && !r.rank_deficient, "A2 4 输入有效");
    CHECK(r.residual_norm < 2e-3f, "A2 冗余解残差小");
    CHECK(std::isfinite(r.u[0]) && std::isfinite(r.u[3]), "A2 解有限");
  }

  // A3 权重：两路完全相同执行器，权重 1:4 → 指令量 1:4。
  {
    auto in = makeInput(2);
    in.B[0][0] = 1.0f;
    in.B[0][1] = 1.0f;
    in.weight[0] = 1.0f;
    in.weight[1] = 4.0f;
    in.moment[0] = 3.0f;
    const auto r = bfs::allocateGeneric(in);
    CHECK(r.valid, "A3 有效");
    CHECK(std::fabs(r.u[0] - 0.6f) < 1e-3f, "A3 低权重分配 0.6");
    CHECK(std::fabs(r.u[1] - 2.4f) < 1e-3f, "A3 高权重分配 2.4");
    CHECK(r.residual_norm < 1e-4f, "A3 合力=3");
  }

  // A4 饱和：u0 上限 2，期望 10 → 饱和且残差 8。
  {
    auto in = makeInput(3);
    in.B[0][0] = 1.0f;
    in.B[1][1] = 1.0f;
    in.B[2][2] = 1.0f;
    in.max_u[0] = 2.0f;
    in.moment[0] = 10.0f;
    const auto r = bfs::allocateGeneric(in);
    CHECK(r.valid, "A4 有效");
    CHECK(r.saturated[0], "A4 通道 0 饱和");
    CHECK(std::fabs(r.u[0] - 2.0f) < 1e-5f, "A4 输出被限幅");
    CHECK(std::fabs(r.residual_norm - 8.0f) < 1e-3f, "A4 残差=8");
  }

  // A5 秩亏：两列共线，期望仍在可达空间内；应标记 rank_deficient。
  {
    auto in = makeInput(2);
    in.B[0][0] = 1.0f;
    in.B[0][1] = 2.0f;
    in.moment[0] = 3.0f;
    const auto r = bfs::allocateGeneric(in);
    CHECK(r.valid, "A5 有效");
    CHECK(r.rank_deficient, "A5 标记秩亏");
    CHECK(std::isfinite(r.u[0]) && std::isfinite(r.u[1]), "A5 解有限");
    CHECK(r.residual_norm < 1e-2f, "A5 共线输入仍可达期望");
  }

  // A6 非法输入：n=0、非有限权重。
  {
    bfs::GenericAllocationInput zero;
    CHECK(!bfs::allocateGeneric(zero).valid, "A6 n=0 拒绝");
    auto in = makeInput(3);
    in.B[0][0] = 1.0f;
    in.B[1][1] = 1.0f;
    in.B[2][2] = 1.0f;
    in.weight[1] = std::nanf("");
    CHECK(!bfs::allocateGeneric(in).valid, "A6 非有限权重拒绝");
  }

  // A7 通用 3×3 ≡ legacy BTRUE（同一工作点、无限幅触发）。
  {
    const TandemVecParams &p = kDefaultTandemVecParams;
    AllocationInput legacy_in;
    legacy_in.Mx_cmd = 0.010f;
    legacy_in.My_cmd = 0.025f;
    legacy_in.Mz_cmd = 0.040f;
    legacy_in.w0 = 800.0f;
    legacy_in.current_state = PropulsionState{800.0f, 800.0f, 0.0f, 0.0f};
    const AllocationOutput legacy =
        allocateMoments(legacy_in, p, AllocationStrategy::BTRUE);

    const EffectMatrix effect = computeEffectMatrix(legacy_in.current_state, p,
                                                    legacy_in.w0);
    bfs::GenericAllocationInput generic;
    generic.n = 3;
    for (uint8_t r = 0; r < 3; ++r)
    {
      for (uint8_t c = 0; c < 3; ++c)
      {
        generic.B[r][c] = effect.M[r][c];
      }
    }
    generic.moment[0] = legacy_in.Mx_cmd;
    generic.moment[1] = legacy_in.My_cmd;
    generic.moment[2] = legacy_in.Mz_cmd;
    generic.min_u[0] = -p.dwMax;
    generic.max_u[0] = p.dwMax;
    generic.min_u[1] = -p.dMax;
    generic.max_u[1] = p.dMax;
    generic.min_u[2] = -p.dMax;
    generic.max_u[2] = p.dMax;
    generic.weight[0] = generic.weight[1] = generic.weight[2] = 1.0f;
    generic.regularization = 0.0f;
    const auto r = bfs::allocateGeneric(generic);
    CHECK(r.valid, "A7 legacy 工作点有效");
    CHECK(std::fabs(r.u[0] - legacy.dw) < 1e-4f, "A7 Δω ≡ legacy");
    CHECK(std::fabs(r.u[1] - legacy.delta_t) < 1e-4f, "A7 δ_t ≡ legacy");
    CHECK(std::fabs(r.u[2] - legacy.delta_f) < 1e-4f, "A7 δ_f ≡ legacy");
  }

  // A8 3×3 逆：单位阵与奇异阵。
  {
    float I[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float inv[3][3];
    float det = 0.0f;
    CHECK(bfs::invert3x3(I, inv, &det), "A7 单位阵可逆");
    CHECK(std::fabs(det - 1.0f) < 1e-6f, "A7 det=1");
    float zero[3][3] = {};
    CHECK(!bfs::invert3x3(zero, inv, nullptr), "A7 奇异阵拒绝");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_generic_allocation: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_generic_allocation: %d 项失败 ===\n", g_fail);
  return 1;
}
