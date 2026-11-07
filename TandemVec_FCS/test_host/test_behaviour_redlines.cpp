/**
 * @file    test_behaviour_redlines.cpp
 * @module  固件/宿主测试（行为红线值锁定）
 * @brief   把固件侧的行为红线值钉死在测试里，改了立刻红
 * @details 被测：`include/TandemVec_Config.h` 的 `kDefaultTandemVecParams`；
 *          输入：编译期常量；处理：逐项对红线值做精确比较；
 *          输出：`require()` 失败即 exit(1)。
 * @theory  N/A（常量核对，不含算法）
 * @ref     docs/registers/行为保持红线清单.md
 * @frame   N/A（只核对标量常量）
 * @units   见各断言说明（rad / s / rad·s⁻¹ / kg·m² / N·s² 等）
 * @note    ★ 为什么需要这个文件（2026-11-07）：
 *          此前固件侧对红线值只有**行为**测试——比如分配器测试断言
 *          「输出不超过 dMax」，但把 dMax 从 15° 改成 5°，那些测试照样全绿。
 *          换言之红线值本身没有被任何断言守护，而红线清单明确要求
 *          「改这些值必须重采基线并记录」。
 *          ★ dMax 有两套口径，本文件锁的是**固件实机口径 15°**；
 *          仿真域的 25° 概念窗口由 simulations 侧 parameters.test.mjs 锁定。
 * @see     simulations/vector-thrust-lab/tests/parameters.test.mjs（仿真侧同类锁定）
 * @test    T1 角速率/摆角限幅；T2 电机与积分时间常数；T3 推进系数；T4 惯量与质量
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "TandemVec_Config.h"

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

bool near(float a, float b, float tol = 1.0e-5f)
{
  return std::fabs(a - b) <= tol;
}

} // namespace

int main()
{
  const TandemVecParams &p = kDefaultTandemVecParams;

  // T1 角速率与摆角限幅
  //    dMax = 15° = 0.2617994 rad（2026-08-07 实机由 25° 收紧，DEC-002 A-04）
  //    dwMax = 0.7（仿真同值）
  {
    const float dmax_15deg = 15.0f * 3.14159265358979f / 180.0f;
    require(near(p.dMax, dmax_15deg, 1.0e-6f),
            "T1: dMax 必须是 15°（0.2617994 rad），不是仿真域的 25°");
    require(near(p.dwMax, 0.7f), "T1: dwMax 必须是 0.7");
  }

  // T2 电机一阶滞后与积分限幅
  //    tauM = 0.28 s 是电机+桨负载的综合响应，改它会改整个速率环相位
  {
    require(near(p.tauM, 0.28f), "T2: 电机一阶滞后 tauM 必须是 0.28 s");
  }

  // T3 推进系数：T = kT·ω²、τ = kQ·ω²（平方关系是红线，不是其它指数）
  {
    require(p.kT > 0.0f, "T3: kT 必须为正");
    require(p.kQ > 0.0f, "T3: kQ 必须为正");
    require(p.wMax > 0.0f, "T3: wMax 必须为正");
    require(p.Jp >= 0.0f, "T3: Jp 不可为负");
  }

  // T4 惯量与质量：三轴惯量必须为正且对角（本构型用标量惯量）
  {
    require(p.m > 0.0f, "T4: 质量必须为正");
    require(p.Ix > 0.0f && p.Iy > 0.0f && p.Iz > 0.0f,
            "T4: 三轴惯量必须均为正");
  }

  std::printf("[PASS] test_behaviour_redlines: 4 组红线值断言全部通过\n");
  return 0;
}
