/**
 * @file    test_coaxial_reference_equivalence.cpp
 * @module  固件/宿主回归测试（fw-tests：共轴控制律输出级参考等价）
 * @brief   在确定性输入序列上逐拍比较统一固件 CoaxialVtvControl 与
 *          实飞共轴工程参考实现（外部 PositionPID/ComplementaryFilter）的
 *          两轴修正角与偏航差速
 * @details 被测：`include/CoaxialVtvControl.h`；参考：
 *          `test_host/reference/coaxial/coax_reference.cpp`（外部实飞工程
 *          PositionPID/ComplementaryFilter 原头文件）。
 *          序列覆盖：ATTITUDE / RATE / MANUAL_TVC / INACTIVE 四种状态，
 *          含角速率、偏航目标与手动修正变化。
 *          编译：g++ -std=c++17 -Iinclude -Itest_host/reference/coaxial \
 *                -Itest_host/stub test_host/test_coaxial_reference_equivalence.cpp \
 *                test_host/reference/coaxial/coax_reference.cpp -o bin/cxreq
 * @theory  ω_ref=PID_att(e,−ω)；u=PID_rate(ω_ref,ω)；yaw=PID_yaw(ω_ref,ω_z)
 *          @ref CTL-003 §2.2；实飞工程 VTVL_ElectricDualRotor_FCS
 * @frame   存档系 x=滚转、y=俯仰、z=推力轴
 * @units   deg、deg/s、deg、us、s
 * @note    参考实现与统一固件共享同款 PID/滤波公式；本测试锁定后续重构不得
 *          改变输出。它不能替代实飞日志回放（没有外部飞行日志）。
 * @test    E1 姿态模式逐拍等价；E2 RATE 模式；E3 手动 TVC；E4 失效回中；
 *          E5 全序列最大误差
 * @see     include/CoaxialVtvControl.h、test_host/reference/coaxial/
 */
#include <cmath>
#include <cstdio>

#include "CoaxialVtvControl.h"
#include "coax_reference.h"

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

int main()
{
  bfs::CoaxialVtvController controller;
  coax_ref_reset();

  float max_roll_err = 0.0f;
  float max_pitch_err = 0.0f;
  float max_yaw_err = 0.0f;
  float mode_roll_err[4] = {0, 0, 0, 0};
  float mode_pitch_err[4] = {0, 0, 0, 0};
  float mode_yaw_err[4] = {0, 0, 0, 0};

  for (int i = 0; i < 600; ++i)
  {
    int mode = 0; // 0 attitude / 1 rate / 2 manual / 3 inactive
    if (i >= 200 && i < 250) mode = 1;
    if (i >= 250 && i < 300) mode = 2;
    if (i >= 300 && i < 350) mode = 3;

    const float t = static_cast<float>(i) * 0.005f;
    const float error_roll = 8.0f * std::sin(1.1f * t) +
                             3.0f * std::sin(7.0f * t);
    const float error_pitch = -5.0f * std::cos(0.7f * t) +
                              2.0f * std::sin(5.0f * t);
    const float rate_roll = 20.0f * std::sin(3.0f * t);
    const float rate_pitch = 15.0f * std::cos(2.0f * t);
    const float rate_yaw = 10.0f * std::sin(1.5f * t);

    bfs::CoaxialVtvInput in;
    in.error_roll_deg = error_roll;
    in.error_pitch_deg = error_pitch;
    in.rate_roll_dps = rate_roll;
    in.rate_pitch_dps = rate_pitch;
    in.rate_yaw_dps = rate_yaw;
    in.yaw_rate_target_dps = 12.0f * std::sin(0.9f * t);
    in.rate_mode = (mode == 1);
    in.roll_rate_target_dps = 25.0f * std::sin(1.3f * t);
    in.pitch_rate_target_dps = -20.0f * std::cos(1.7f * t);
    in.attitude_active = (mode == 0 || mode == 1);
    in.yaw_active = (mode != 3);
    in.manual_tvc = (mode == 2);
    in.manual_roll_deg = 4.0f * std::sin(2.0f * t);
    in.manual_pitch_deg = -3.0f * std::cos(2.2f * t);
    in.dt_s = 0.005f;

    CoaxRefInput ref_in{};
    ref_in.error_roll_deg = in.error_roll_deg;
    ref_in.error_pitch_deg = in.error_pitch_deg;
    ref_in.rate_roll_dps = in.rate_roll_dps;
    ref_in.rate_pitch_dps = in.rate_pitch_dps;
    ref_in.rate_yaw_dps = in.rate_yaw_dps;
    ref_in.yaw_rate_target_dps = in.yaw_rate_target_dps;
    ref_in.rate_mode = in.rate_mode ? 1 : 0;
    ref_in.roll_rate_target_dps = in.roll_rate_target_dps;
    ref_in.pitch_rate_target_dps = in.pitch_rate_target_dps;
    ref_in.attitude_active = in.attitude_active ? 1 : 0;
    ref_in.yaw_active = in.yaw_active ? 1 : 0;
    ref_in.manual_tvc = in.manual_tvc ? 1 : 0;
    ref_in.manual_roll_deg = in.manual_roll_deg;
    ref_in.manual_pitch_deg = in.manual_pitch_deg;
    ref_in.dt_s = in.dt_s;

    const bfs::CoaxialVtvOutput out = controller.update(in);
    CoaxRefOutput ref{};
    coax_ref_update(&ref_in, &ref);

    const float er = std::fabs(out.roll_correction_deg -
                               ref.roll_correction_deg);
    const float ep = std::fabs(out.pitch_correction_deg -
                               ref.pitch_correction_deg);
    const float ey = std::fabs(out.yaw_diff_us - ref.yaw_diff_us);
    max_roll_err = std::fmax(max_roll_err, er);
    max_pitch_err = std::fmax(max_pitch_err, ep);
    max_yaw_err = std::fmax(max_yaw_err, ey);
    mode_roll_err[mode] = std::fmax(mode_roll_err[mode], er);
    mode_pitch_err[mode] = std::fmax(mode_pitch_err[mode], ep);
    mode_yaw_err[mode] = std::fmax(mode_yaw_err[mode], ey);
  }

  std::printf("CX-REF max_err roll=%.6f pitch=%.6f yaw=%.6f\n",
              max_roll_err, max_pitch_err, max_yaw_err);
  CHECK(max_roll_err < 1.0e-4f, "E1/E2/E3 滚转逐拍等价");
  CHECK(max_pitch_err < 1.0e-4f, "E1/E2/E3 俯仰逐拍等价");
  CHECK(max_yaw_err < 1.0e-4f, "E1/E2/E3 偏航逐拍等价");
  CHECK(mode_roll_err[0] < 1.0e-4f && mode_pitch_err[0] < 1.0e-4f,
        "E1 ATTITUDE 模式等价");
  CHECK(mode_roll_err[1] < 1.0e-4f && mode_pitch_err[1] < 1.0e-4f,
        "E2 RATE 模式等价");
  CHECK(mode_roll_err[2] < 1.0e-4f && mode_pitch_err[2] < 1.0e-4f,
        "E3 手动 TVC 等价");
  CHECK(mode_roll_err[3] < 1.0e-4f && mode_pitch_err[3] < 1.0e-4f &&
            mode_yaw_err[3] < 1.0e-4f,
        "E4 INACTIVE 回中");
  CHECK(max_roll_err < 1.0e-4f && max_pitch_err < 1.0e-4f &&
            max_yaw_err < 1.0e-4f,
        "E5 全序列最大误差");

  if (g_fail == 0)
  {
    std::printf("=== test_coaxial_reference_equivalence: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_coaxial_reference_equivalence: %d 项失败 ===\n", g_fail);
  return 1;
}
