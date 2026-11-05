/**
 * @file    test_single_edf_4vane.cpp
 * @module  固件/宿主回归测试（fw-tests：单涵道 + X 型四燃气舵机型的分配与输出）
 * @brief   验证 3×4 分配矩阵、α→力矩→舵偏换算、联合等比限幅、手动直通、
 *          偏航配平、推进联锁与四舵/单油门输出映射
 * @details 被测：`include/SingleEdf4VaneAllocator.h`、`include/ActuatorOutputMap.h`
 *          的 `mapSingleEdf4VaneOutputs()` 与 `singleEdf4VaneThrusterEnable()`、
 *          `include/VehicleProfile.h` 的 `kSingleEdf4VaneProfile`。
 *          输入：三轴角加速度、满舵角/舵效/惯量/方向符号、手动直通角；
 *          处理：allocateSingleEdf4Vane / mapSingleEdf4VaneOutputs；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_single_edf_4vane.cpp -o bin/sedf
 * @theory  M = k·H·δ 且 H·Hᵀ=4I ⇒ δ = Hᵀ·τ/(4k)；联合限幅取 max|δ| 等比缩放
 *          @ref THY-003 §3；CTL-001 §2
 * @frame   固件存档系（x_b=滚转、y_b=俯仰、z_b=推力轴=偏航）
 * @units   α rad/s²、惯量 kg·m²、舵效 N·m/deg、舵偏角 deg、输出 %
 * @note    本测试只证明"数学分配与输出映射自洽"，不证明实物舵向正确——
 *          实物符号必须按标定流程逐舵核对（VANE 方向与 VANE_DIR）。
 * @test    P1 分配矩阵正交；P2 单轴映射解析值；P3 联合等比限幅；P4 手动直通；
 *          P5 逐舵方向符号；P6 偏航配平只落在 Mz；P7 推进联锁三条件；
 *          P8 四舵/油门输出映射；P9 非有限输入回中位
 * @see     include/SingleEdf4VaneAllocator.h、include/ActuatorOutputMap.h
 */
#include <cmath>
#include <cstdio>
#include <limits>

#include "ActuatorOutputMap.h"
#include "FlightCtrlParams.h"
#include "SingleEdf4VaneAllocator.h"

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

static bool near(float a, float b, float tol = 1.0e-4f)
{
  return std::fabs(a - b) <= tol;
}

int main()
{
  // ---- P1：分配矩阵三行两两正交（H·Hᵀ = 4I）----
  {
    for (int a = 0; a < 3; ++a)
    {
      for (int b = 0; b < 3; ++b)
      {
        float dot = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
          dot += bfs::kSingleEdf4VaneMatrix[a][i] * bfs::kSingleEdf4VaneMatrix[b][i];
        }
        const float expect = (a == b) ? 4.0f : 0.0f;
        CHECK(near(dot, expect), "P1 分配矩阵行正交性（H·Hᵀ=4I）");
      }
    }
    CHECK(bfs::singleEdfVaneRowsOrthogonal(), "P1 编译期正交自检与运行期一致");
  }

  bfs::SingleEdf4VaneAllocationParams p;
  p.vane_max_deg = 20.0f;
  p.vane_moment_nm_per_deg = 0.02f;
  p.inertia_roll_kgm2 = 0.02f;
  p.inertia_pitch_kgm2 = 0.02f;
  p.inertia_yaw_kgm2 = 0.03f;

  // ---- P2：单轴 α → 四舵偏角解析值 ----
  // 滚转：τx=+Ix·α=0.02 ⇒ d=τ/(4k)=0.25° ⇒ δ=roll 行·0.25 = (−,+,+,−)
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_roll_radps2 = 1.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    CHECK(out.valid, "P2 滚转输入有效");
    CHECK(near(out.vane_deg[0], -0.25f) && near(out.vane_deg[1], 0.25f) &&
              near(out.vane_deg[2], 0.25f) && near(out.vane_deg[3], -0.25f),
          "P2 滚转 α=1 的四舵偏角解析值");
    CHECK(!out.saturated, "P2 小指令不触发联合限幅");
    CHECK(near(out.torque_cmd_nm[0], 0.02f), "P2 滚转力矩 = Ix·α");
  }
  // 俯仰：τy=0.02 ⇒ d=0.25° ⇒ δ=(+,+,−,−)
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_pitch_radps2 = 1.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    CHECK(near(out.vane_deg[0], 0.25f) && near(out.vane_deg[1], 0.25f) &&
              near(out.vane_deg[2], -0.25f) && near(out.vane_deg[3], -0.25f),
          "P2 俯仰 α=1 的四舵偏角解析值");
  }
  // 偏航：τz=−Iz·α=−0.03（机型知识：yaw 取反）⇒ d=−0.375° ⇒ δ=yaw 行·(−0.375) = (−,+,−,+)
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_yaw_radps2 = 1.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    CHECK(near(out.vane_deg[0], -0.375f) && near(out.vane_deg[1], 0.375f) &&
              near(out.vane_deg[2], -0.375f) && near(out.vane_deg[3], 0.375f),
          "P2 偏航 α=1 的四舵偏角解析值（对角分组）");
    CHECK(near(out.torque_cmd_nm[2], -0.03f), "P2 偏航力矩 = −Iz·α（yaw 轴取反）");
  }

  // ---- P3：联合等比限幅（三轴同时给大指令，方向不畸变、幅值到满偏）----
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_roll_radps2 = 40.0f;
    in.alpha_pitch_radps2 = 40.0f;
    in.alpha_yaw_radps2 = 40.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    float peak = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
      peak = std::fmax(peak, std::fabs(out.vane_deg[i]));
    }
    // 解析：τ=(0.8, 0.8, −1.2)；d=τ/(4k)=(10,10,−15)；
    // δ_raw = (−15, +35, −15, −5)；peak=35 ⇒ scale=20/35
    const float scale = 20.0f / 35.0f;
    CHECK(out.saturated, "P3 大指令触发联合限幅");
    CHECK(near(peak, 20.0f, 1.0e-3f), "P3 限幅后最大舵角恰为满偏角");
    CHECK(near(out.vane_deg[0], -15.0f * scale, 1.0e-3f) &&
              near(out.vane_deg[1], 35.0f * scale, 1.0e-3f) &&
              near(out.vane_deg[2], -15.0f * scale, 1.0e-3f) &&
              near(out.vane_deg[3], -5.0f * scale, 1.0e-3f),
          "P3 联合等比缩放后的四舵解析值");
    CHECK(out.residual_nm > 0.0f, "P3 饱和时力矩残差 > 0");
  }

  // ---- P4：手动直通（地面标定）——只做满偏限幅与方向符号 ----
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.direct_vane_enabled = true;
    in.direct_vane_deg[0] = 30.0f;
    in.direct_vane_deg[1] = 0.0f;
    in.direct_vane_deg[2] = -10.0f;
    in.direct_vane_deg[3] = 0.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    CHECK(out.valid && out.saturated, "P4 直通超限触发限幅");
    CHECK(near(out.vane_deg[0], 20.0f), "P4 直通满偏钳位到 +20°");
    CHECK(near(out.vane_deg[2], -20.0f / 3.0f), "P4 直通等比缩放（-10 → -6.67）");
    CHECK(near(out.vane_norm[0], 1.0f), "P4 归一化量满偏 = 1");
  }

  // ---- P5：逐舵方向符号（VANE_DIR）翻转输出舵角与归一化量 ----
  {
    bfs::SingleEdf4VaneAllocationParams flipped = p;
    flipped.vane_dir[0] = -1.0f;
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_pitch_radps2 = 1.0f;
    const auto out = bfs::allocateSingleEdf4Vane(in, flipped);
    CHECK(near(out.vane_deg[0], -0.25f), "P5 方向符号翻转输出舵角");
    CHECK(near(out.vane_norm[0], -0.25f / 20.0f), "P5 归一化量随输出舵角同向翻转");
    CHECK(near(out.vane_norm[1], 0.25f / 20.0f), "P5 其它舵不受影响");
  }

  // ---- P6：偏航配平只落在 Mz（绕推力轴）----
  {
    bfs::SingleEdf4VaneAllocationParams trimmed = p;
    trimmed.yaw_trim_deg = 5.0f;
    bfs::SingleEdf4VaneAllocationInput in; // 三轴 α 全 0
    const auto out = bfs::allocateSingleEdf4Vane(in, trimmed);
    CHECK(near(out.vane_deg[0], 5.0f) && near(out.vane_deg[1], -5.0f) &&
              near(out.vane_deg[2], 5.0f) && near(out.vane_deg[3], -5.0f),
          "P6 配平角按偏航行分布到四舵");
    CHECK(near(out.torque_cmd_nm[0], 0.0f) && near(out.torque_cmd_nm[1], 0.0f),
          "P6 配平不产生滚转/俯仰力矩指令");
    CHECK(near(out.torque_achieved_nm[2], 4.0f * p.vane_moment_nm_per_deg * 5.0f),
          "P6 配平产生的偏航力矩 = 4k·trim");
  }

  // ---- P7：推进联锁三条件（未解锁 / 使能关 / 大倾角），倾角用"偏离天向角"----
  {
    CHECK(!bfs::singleEdf4VaneThrusterEnable(false, true, 0.0f, 45.0f),
          "P7 未解锁 → 切断推进");
    CHECK(!bfs::singleEdf4VaneThrusterEnable(true, false, 0.0f, 45.0f),
          "P7 使能开关关 → 切断推进");
    CHECK(!bfs::singleEdf4VaneThrusterEnable(true, true, 60.0f, 45.0f),
          "P7 偏离天向 60° → 切断推进");
    CHECK(!bfs::singleEdf4VaneThrusterEnable(true, true, 180.0f, 45.0f),
          "P7 倒扣 180° → 切断推进");
    CHECK(!bfs::singleEdf4VaneThrusterEnable(true, true, -60.0f, 45.0f),
          "P7 反向倾覆同样切断");
    CHECK(bfs::singleEdf4VaneThrusterEnable(true, true, 44.0f, 45.0f),
          "P7 阈值内 → 允许推进");
    CHECK(bfs::singleEdf4VaneThrusterEnable(true, true, 0.5f, 45.0f),
          "P7 竖直（0°）→ 允许推进");
    CHECK(!bfs::singleEdf4VaneThrusterEnable(true, true,
                                             std::numeric_limits<float>::quiet_NaN(),
                                             45.0f),
          "P7 倾角非有限 → 切断推进");
  }

  // ---- P8：四舵 + 单油门输出映射 ----
  {
    CHECK(bfs::validateVehicleProfile(bfs::kSingleEdf4VaneProfile) ==
              bfs::VehicleProfileError::Ok,
          "P8 单涵道 profile 校验通过");
    CHECK(bfs::kSingleEdf4VaneProfile.output_count == 5U,
          "P8 输出通道数 = 4 舵 + 1 油门");
    CHECK(bfs::kSingleEdf4VaneProfile.control_architecture ==
              bfs::ControlArchitecture::FourVaneMatrix,
          "P8 控制架构 = 四舵矩阵分配");

    bfs::SingleEdf4VaneOutputCalibration cal;
    bfs::SingleEdf4VaneOutputInput in;
    in.thruster_cut = false;
    in.throttle_pct = 50.0f;
    const auto mid = bfs::mapSingleEdf4VaneOutputs(in, cal);
    CHECK(mid.count == 5U, "P8 输出 5 路命令");
    CHECK(near(mid.commands[0].value, 50.0f), "P8 中位舵 = 50%（1500µs）");
    CHECK(near(mid.commands[4].value, 50.0f), "P8 油门 50% 直通");

    in.vane_norm[0] = 1.0f;
    in.vane_norm[1] = -1.0f;
    const auto full = bfs::mapSingleEdf4VaneOutputs(in, cal);
    CHECK(near(full.commands[0].value, 66.67f, 0.02f), "P8 满舵正偏 → 66.67%");
    CHECK(near(full.commands[1].value, 33.33f, 0.02f), "P8 满舵反偏 → 33.33%");

    in.thruster_cut = true;
    const auto cut = bfs::mapSingleEdf4VaneOutputs(in, cal);
    CHECK(near(cut.commands[4].value, 0.0f), "P8 联锁切断 → 油门 0%");
    CHECK(near(cut.commands[0].value, 66.67f, 0.02f), "P8 联锁只切油门，舵仍受控");

    // 硬限位：把 span 放大到超出行程，输出必须被 clamp 在 [min,max]
    bfs::SingleEdf4VaneOutputCalibration wide = cal;
    wide.vane_span_pct = 40.0f;
    const auto clamped = bfs::mapSingleEdf4VaneOutputs(in, wide);
    CHECK(clamped.commands[0].value <= 66.66f + 1.0e-3f,
          "P8 舵机占空比硬上限生效");
    CHECK(clamped.commands[1].value >= 33.33f - 1.0e-3f,
          "P8 舵机占空比硬下限生效");
  }

  // ---- P10：固件默认参数与源机型台架结论对齐 ----
  {
    const SingleEdf4VaneHwParams &hw = kSingleEdf4VaneHwParamsDefaults;
    CHECK(hw.vane_max_deg == 20.0f,
          "P10 默认满舵角 20°（源文档口径；源代码 45° 与之冲突，取保守值）");
    CHECK(near(hw.vane_dir[0], 1.0f) && near(hw.vane_dir[1], -1.0f) &&
              near(hw.vane_dir[2], 1.0f) && near(hw.vane_dir[3], -1.0f),
          "P10 默认舵方向 = 源机型 2026-10-12 台架确认值 (+1,-1,+1,-1)");
    CHECK(hw.attitude_limit_deg == 45.0f, "P10 默认大倾角阈值 45°");
    CHECK(hw.vane_center_pct == 50.0f && near(hw.vane_span_pct, 16.67f) &&
              near(hw.vane_min_pct, 33.33f) && near(hw.vane_max_pct, 66.66f),
          "P10 默认舵机中位/半行程/硬限位与源机型一致");
    // 默认方向符号下的纯俯仰执行量：舵1/4 同号、舵2/3 同号且相反
    bfs::SingleEdf4VaneAllocationParams p10;
    for (int i = 0; i < 4; ++i)
    {
      p10.vane_dir[i] = hw.vane_dir[i];
    }
    bfs::SingleEdf4VaneAllocationInput in10;
    in10.alpha_pitch_radps2 = 1.0f;
    const auto out10 = bfs::allocateSingleEdf4Vane(in10, p10);
    CHECK(out10.vane_deg[0] > 0.0f && out10.vane_deg[3] > 0.0f &&
              out10.vane_deg[1] < 0.0f && out10.vane_deg[2] < 0.0f,
          "P10 默认方向符号下纯俯仰的四舵执行量符号");
  }

  // ---- P11：手动直通矩阵 = 源机型混控器逐字一致 ----
  // 源公式（Universal-EDF airframe_mixer_single_edf_4vane.cpp）：
  //   defl1 = −roll + pitch + yaw
  //   defl2 = +roll + pitch − yaw
  //   defl3 = +roll − pitch + yaw
  //   defl4 = −roll − pitch − yaw
  {
    const float expect_pure_roll[4] = {-1.0f, +1.0f, +1.0f, -1.0f};
    const float expect_pure_pitch[4] = {+1.0f, +1.0f, -1.0f, -1.0f};
    const float expect_pure_yaw[4] = {+1.0f, -1.0f, +1.0f, -1.0f};
    for (int i = 0; i < 4; ++i)
    {
      CHECK(near(bfs::kSingleEdf4VaneManualMatrix[0][i], expect_pure_roll[i]),
            "P11 手动矩阵 roll 行 = 源 defl 公式");
      CHECK(near(bfs::kSingleEdf4VaneManualMatrix[1][i], expect_pure_pitch[i]),
            "P11 手动矩阵 pitch 行 = 源 defl 公式");
      CHECK(near(bfs::kSingleEdf4VaneManualMatrix[2][i], expect_pure_yaw[i]),
            "P11 手动矩阵 yaw 行 = 源 defl 公式");
    }
    CHECK(bfs::singleEdfVaneManualRowsOrthogonal(), "P11 手动矩阵行正交");
  }

  // ---- P9：非有限输入 → 分配无效、四舵回中位 ----
  {
    bfs::SingleEdf4VaneAllocationInput in;
    in.alpha_pitch_radps2 = std::numeric_limits<float>::quiet_NaN();
    const auto out = bfs::allocateSingleEdf4Vane(in, p);
    CHECK(!out.valid, "P9 NaN 输入 → 分配无效");
    for (int i = 0; i < 4; ++i)
    {
      CHECK(near(out.vane_deg[i], 0.0f), "P9 无效分配时四舵输出为 0（中位）");
    }
    // 参数非法（满偏角 0）也必须无效
    bfs::SingleEdf4VaneAllocationParams bad = p;
    bad.vane_max_deg = 0.0f;
    bfs::SingleEdf4VaneAllocationInput ok_in;
    ok_in.alpha_roll_radps2 = 1.0f;
    CHECK(!bfs::allocateSingleEdf4Vane(ok_in, bad).valid,
          "P9 满偏角 0 → 参数非法");
  }

  if (g_fail == 0)
  {
    std::printf("ALL PASS: test_single_edf_4vane\n");
    return 0;
  }
  std::printf("FAILED: %d checks\n", g_fail);
  return 1;
}
