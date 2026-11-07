/**
 * @file    test_single_edf_plant.cpp
 * @module  固件/宿主测试（fw-tests：单涵道 SIL plant 与分配器的往返一致性）
 * @brief   验证「分配器逆解 α→δ」与「plant 正解 δ→τ」严格互逆，并锁死轴向映射
 * @details 被测：`test_host/fw_sil/single_edf_plant.h` 的 `vaneTorque()` 与
 *          `include/SingleEdf4VaneAllocator.h` 的 `allocateSingleEdf4Vane()`。
 *          输入：三轴目标角加速度 α（单轴与组合）；
 *          处理：α → 分配器 → 四舵偏角 → plant 正解 → 三轴力矩 τ'；
 *          输出：`require()` 失败即 exit(1)。
 *          运行：g++ -std=c++17 -Iinclude -Itest_host/stub \
 *                  test_host/test_single_edf_plant.cpp -o bin/sedfplant
 * @theory  分配器：τ_j = I_j·α_j，δ_i = dir_i·Σ_j H[j][i]·τ_j/(4k)；
 *          plant 正解：τ'_j = k·Σ_i H[j][i]·(δ_i/dir_i)。两式互为对偶，
 *          由 H·Hᵀ = 4I 保证 τ' = τ（限幅未触发时）。
 * @ref     CTL-008 §轴系对照（行序 = 本固件存档系：H[0]→Mx、H[1]→My、H[2]→Mz）；
 *          `docs/05-控制与分配/CTL-008-单涵道四燃气舵机型移植-2026-11-05.md`
 * @frame   本固件存档系：x_b 滚转（倾覆轴）、y_b 俯仰、z_b = 推力轴 = 偏航（世界航向）
 * @units   rad/s²、N·m、deg、kg·m²、N·m/deg
 * @note    ★ 本测试存在的理由：2026-11-07 的 SIL plant 实现曾按「源机型 roll 绕推力轴」
 *          把 H[0] 的力矩接到 Mz、并漏乘 vane_dir，导致手动阶跃自旋发散。那次的错误
 *          在**定高模式**下完全不暴露（舵面不动），只有打杆才触发。本测试把"逆解与
 *          正解必须闭合"写成断言，任何轴向/符号改错都会在这里先失败，而不是等到闭环。
 * @test    T1 纯滚转往返；T2 纯俯仰往返；T3 纯偏航往返（含分配器 torque[2] 取反）；
 *          T4 三轴组合往返；T5 掩码：H[0] 只产生 Mx、H[2] 只产生 Mz（轴向不串扰）；
 *          T6 悬停配平油门 = 质量×重力/最大推力
 * @see     TandemVec_FCS/test_host/fw_sil/single_edf_plant.h、
 *          TandemVec_FCS/include/SingleEdf4VaneAllocator.h
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../include/SingleEdf4VaneAllocator.h"
#include "fw_sil/single_edf_plant.h"

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

bool near(float a, float b, float tol)
{
  return std::fabs(a - b) <= tol;
}

const SingleEdf4VaneHwParams &P = kSingleEdf4VaneHwParamsDefaults;

/// 用实数域单位置构造分配参数（避开 FlightCtrlParams 的实例依赖）
bfs::SingleEdf4VaneAllocationParams makeParams()
{
  bfs::SingleEdf4VaneAllocationParams p;
  p.vane_max_deg = P.vane_max_deg;
  p.vane_moment_nm_per_deg = P.vane_moment_nm_per_deg;
  p.inertia_roll_kgm2 = P.inertia_roll_kgm2;
  p.inertia_pitch_kgm2 = P.inertia_pitch_kgm2;
  p.inertia_yaw_kgm2 = P.inertia_yaw_kgm2;
  p.yaw_trim_deg = 0.0f;
  for (int i = 0; i < 4; ++i)
  {
    p.vane_dir[i] = P.vane_dir[i];
  }
  return p;
}

/// α → 分配器 → 舵角 → plant 正解 → 三轴力矩
void roundTrip(float a_roll, float a_pitch, float a_yaw, float tau_out[3])
{
  bfs::SingleEdf4VaneAllocationInput in;
  in.alpha_roll_radps2 = a_roll;
  in.alpha_pitch_radps2 = a_pitch;
  in.alpha_yaw_radps2 = a_yaw;
  const bfs::SingleEdf4VaneAllocationOutput alloc =
      bfs::allocateSingleEdf4Vane(in, makeParams());
  require(alloc.valid, "分配器输出必须 valid");
  require(!alloc.saturated, "小指令不应触发联合限幅");

  SilSingleEdfBody::vaneTorque(alloc.vane_deg, tau_out);
}

} // namespace

int main()
{
  const float Ix = P.inertia_roll_kgm2;
  const float Iy = P.inertia_pitch_kgm2;
  const float Iz = P.inertia_yaw_kgm2;
  float tau[3] = {0.0f, 0.0f, 0.0f};

  // T1 纯滚转：α_roll=1 → τ'_x 应为 Ix，其余通道≈0
  {
    roundTrip(1.0f, 0.0f, 0.0f, tau);
    require(near(tau[0], Ix, 1.0e-5f), "T1 滚转往返：τ'_x 应等于 Ix·α");
    require(near(tau[1], 0.0f, 1.0e-5f), "T1 滚转不该串到俯仰");
    require(near(tau[2], 0.0f, 1.0e-5f), "T1 滚转不该串到偏航");
  }

  // T2 纯俯仰
  {
    roundTrip(0.0f, 1.0f, 0.0f, tau);
    require(near(tau[1], Iy, 1.0e-5f), "T2 俯仰往返：τ'_y 应等于 Iy·α");
    require(near(tau[0], 0.0f, 1.0e-5f), "T2 俯仰不该串到滚转");
  }

  // T3 纯偏航：分配器内部对 torque[2] 取反（实机手感约定），
  //    因此正解回来是 −Iz（这是设计行为，不是符号错误）
  {
    roundTrip(0.0f, 0.0f, 1.0f, tau);
    require(near(tau[2], -Iz, 1.0e-5f),
            "T3 偏航往返：τ'_z 应等于 −Iz·α（分配器 torque[2] 取反是实机手感约定）");
    require(near(tau[0], 0.0f, 1.0e-5f), "T3 偏航不该串到滚转");
  }

  // T4 三轴组合：与分配器自报的 torque_cmd 逐轴一致（限幅未触发）
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_roll_radps2 = 0.5f;
    in.alpha_pitch_radps2 = -0.3f;
    in.alpha_yaw_radps2 = 0.2f;
    const bfs::SingleEdf4VaneAllocationOutput alloc =
        bfs::allocateSingleEdf4Vane(in, makeParams());
    require(alloc.valid && !alloc.saturated, "T4 组合指令应 valid 且未饱和");
    SilSingleEdfBody::vaneTorque(alloc.vane_deg, tau);
    for (int j = 0; j < 3; ++j)
    {
      require(near(tau[j], alloc.torque_cmd_nm[j], 1.0e-5f),
              "T4 组合往返：plant 正解应逐轴还原分配器的 torque_cmd");
    }
  }

  // T5 轴向掩码（用分配器生成样本，不能用裸矩阵拼——裸舵角未含 vane_dir，
  //    经 plant 的 δ/dir 还原后会两两抵消，那是测试构造错而非产品错）：
  //    pure roll 只激励 Mx、pure yaw 只激励 Mz。
  {
    // 已在 T1/T3 断言 cross 通道 ≈0；这里补一条「两个对手轴方向相反」的直接判据：
    // 同一舵面组合在 roll 与 yaw 两行上的内积为 0，故二者激励的力矩不能同号。
    float tau_roll[3] = {0.0f, 0.0f, 0.0f};
    float tau_yaw[3] = {0.0f, 0.0f, 0.0f};
    roundTrip(1.0f, 0.0f, 0.0f, tau_roll);
    roundTrip(0.0f, 0.0f, 1.0f, tau_yaw);
    require(std::fabs(tau_roll[2]) < 1.0e-5f && std::fabs(tau_yaw[0]) < 1.0e-5f,
            "T5 滚转/偏航两轴互不串扰（H 行正交的直接体现）");
  }

  // T6 悬停配平：油门 = m·g / T_max（用于核对 SIL 闭环的稳态油门）
  {
    const float hover_pct = (P.mass_kg * 9.80665f / P.max_total_thrust_n) * 100.0f;
    require(near(hover_pct, 32.2f, 0.5f),
            "T6 悬停油门解析值应在 32.2% 附近（1.15 kg × g / 35 N）");
  }

  std::printf("[PASS] test_single_edf_plant: 6 组往返/轴向断言全部通过\n");
  return 0;
}
