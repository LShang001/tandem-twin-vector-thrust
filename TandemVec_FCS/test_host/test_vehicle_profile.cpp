/**
 * @file    test_vehicle_profile.cpp
 * @module  固件/宿主回归测试（fw-tests：机型输出描述与执行器映射）
 * @brief   验证 VehicleProfile 校验、纵列双发旧 mix 等价映射，以及 VTVL/
 *          液体火箭输出表的最小扩展性
 * @details 被测：`include/VehicleProfile.h`、`include/ActuatorOutputMap.h`。
 *          输入：编译期 profile、归一化输出、旧舵机标定；
 *          处理：validateVehicleProfile / mapTandemVecOutputs / mapNormalizedOutputs；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_vehicle_profile.cpp -o bin/vp
 * @theory  执行器输出 = 零点 + 归一化指令×量程；机型差异由数据表描述
 *          @ref THY-003 §2–§3；CTL-001 §8.6
 * @frame   N/A（输出通道量纲；姿态/位置不在此层）
 * @units   输出百分比 %、点火 0/1、舵机角 deg
 * @note    本测试锁定“通用输出表与旧固件公式等价”；新机型 profile 只证明表结构
 *          可表达，不代表固件控制链/制导已支持该机型。
 * @test    P1 profile 校验；P2 旧 mix 等价（含中位/方向）；P3 钳位；
 *          P4 多旋翼四电调；P5 液体火箭阀门/点火/摆座/RCS；
 *          P6 共轴双桨舵机方向/PWM差速/大倾角保护；P7 非有限输入
 * @see     include/VehicleProfile.h、include/ActuatorOutputMap.h
 */
#include <cmath>
#include <cstdio>
#include <limits>

#include "ActuatorOutputMap.h"

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

static double old_servo_pct(double deg, double gear, double dir, double half,
                            double zero_pct)
{
  return 50.0 + zero_pct + (deg * gear * dir / half) * 50.0;
}

int main()
{
  // P1 profile 校验。
  {
    CHECK(bfs::validateVehicleProfile(bfs::kTandemVecProfile) ==
              bfs::VehicleProfileError::Ok,
          "P1 TandemVec profile 有效");
    CHECK(bfs::validateVehicleProfile(bfs::kQuadVtvProfile) ==
              bfs::VehicleProfileError::Ok,
          "P1 QuadVTVL profile 有效");
    CHECK(bfs::validateVehicleProfile(bfs::kLiquidRocketProfile) ==
              bfs::VehicleProfileError::Ok,
          "P1 LiquidRocket profile 有效");
    CHECK(bfs::validateVehicleProfile(bfs::kCoaxialVtvProfile) ==
              bfs::VehicleProfileError::Ok,
          "P1 CoaxialVTVL profile 有效");
    CHECK(bfs::kCoaxialVtvProfile.control_architecture ==
              bfs::ControlArchitecture::DirectTvcRate &&
              bfs::kCoaxialVtvProfile.yaw_strategy ==
                  bfs::YawMixStrategy::DifferentialPwmUs,
          "P1 CoaxialVTVL 控制架构=DirectTvcRate/差速PWM");

    bfs::VehicleProfile bad = bfs::kTandemVecProfile;
    bad.name = "";
    CHECK(bfs::validateVehicleProfile(bad) ==
              bfs::VehicleProfileError::NullName,
          "P1 空名称拒绝");
    bad = bfs::kTandemVecProfile;
    bad.outputs = nullptr;
    CHECK(bfs::validateVehicleProfile(bad) ==
              bfs::VehicleProfileError::NoOutputs,
          "P1 空输出表拒绝");
    bfs::OutputChannelSpec dup[2] = {
        {bfs::OutputKind::ServoPwmPct, 0U, 0, 100, 50, 50},
        {bfs::OutputKind::ServoPwmPct, 0U, 0, 100, 50, 50},
    };
    bad.outputs = dup;
    bad.output_count = 2;
    CHECK(bfs::validateVehicleProfile(bad) ==
              bfs::VehicleProfileError::DuplicateSlot,
          "P1 重复槽位拒绝");
  }

  // P2 旧纵列双发 mix 等价：方向/中位/齿轮/行程全部带入。
  {
    bfs::TandemVecOutputCalibration cal;
    cal.gear_ratio = 40.0f / 30.0f;
    cal.half_travel_deg = 45.0f;
    cal.dir_roll = -1.0f;
    cal.dir_pitch = -1.0f;
    cal.zero_roll_pct = 3.0f;
    cal.zero_pitch_pct = -2.0f;
    const float upper = 10.0f, lower = -5.0f;
    const float m1 = 60.0f, m2 = 40.0f;
    const bfs::OutputCommandSet out =
        bfs::mapTandemVecOutputs(upper, lower, m1, m2, cal);
    CHECK(out.count == 4U, "P2 四路输出");
    CHECK(out.commands[0].kind == bfs::OutputKind::ServoPwmPct,
          "P2 roll 为舵机");
    CHECK(out.commands[2].kind == bfs::OutputKind::EscPwmPct,
          "P2 motor1 为电调");
    CHECK(out.commands[0].valid && out.commands[1].valid &&
              out.commands[2].valid && out.commands[3].valid,
          "P2 输出均有效");
    CHECK(std::fabs(out.commands[0].value -
                    old_servo_pct(upper, 4.0 / 3.0, -1.0, 45.0, 3.0)) < 1e-4,
          "P2 roll 舵机与旧公式一致");
    CHECK(std::fabs(out.commands[1].value -
                    old_servo_pct(lower, 4.0 / 3.0, -1.0, 45.0, -2.0)) < 1e-4,
          "P2 pitch 舵机与旧公式一致");
    CHECK(std::fabs(out.commands[2].value - m1) < 1e-4,
          "P2 motor1 直通");
    CHECK(std::fabs(out.commands[3].value - m2) < 1e-4,
          "P2 motor2 直通");
  }

  // P3 输出钳位：舵机 0–100%，电调 0–100%。
  {
    bfs::TandemVecOutputCalibration cal;
    const bfs::OutputCommandSet out =
        bfs::mapTandemVecOutputs(100.0f, -100.0f, 150.0f, -10.0f, cal);
    CHECK(std::fabs(out.commands[0].value - 100.0f) < 1e-4,
          "P3 roll 舵机上限钳位");
    CHECK(std::fabs(out.commands[1].value - 0.0f) < 1e-4,
          "P3 pitch 舵机下限钳位");
    CHECK(std::fabs(out.commands[2].value - 100.0f) < 1e-4,
          "P3 motor1 上限钳位");
    CHECK(std::fabs(out.commands[3].value - 0.0f) < 1e-4,
          "P3 motor2 下限钳位");
  }

  // P4 多旋翼 VTVL：四路电调，无舵机。
  {
    float norm[4] = {0.25f, 0.5f, 0.75f, 1.0f};
    const bfs::OutputCommandSet out =
        bfs::mapNormalizedOutputs(bfs::kQuadVtvProfile, norm, 4U);
    CHECK(out.count == 4U, "P4 四路电调输出");
    bool all_esc = true;
    for (uint8_t i = 0; i < out.count; ++i)
    {
      all_esc = all_esc && out.commands[i].kind == bfs::OutputKind::EscPwmPct;
    }
    CHECK(all_esc, "P4 全部为电调");
    CHECK(std::fabs(out.commands[3].value - 100.0f) < 1e-4,
          "P4 满油门=100%");
  }

  // P5 液体火箭：阀门/点火/双摆座/RCS 输出表可表达。
  {
    float norm[6] = {0.0f, 1.0f, 0.0f, 0.0f, 0.5f, 0.5f};
    const bfs::OutputCommandSet out =
        bfs::mapNormalizedOutputs(bfs::kLiquidRocketProfile, norm, 6U);
    CHECK(out.count == 6U, "P5 六路输出");
    CHECK(out.commands[0].kind == bfs::OutputKind::ValvePwmPct,
          "P5 阀门通道");
    CHECK(std::fabs(out.commands[1].value - 1.0f) < 1e-4,
          "P5 点火=1");
    CHECK(out.commands[2].kind == bfs::OutputKind::ServoPwmPct,
          "P5 摆座通道");
    CHECK(out.commands[4].kind == bfs::OutputKind::RcsPwmPct,
          "P5 RCS 通道");
  }

  // P6 共轴双桨 VTVL：旧 mix 的舵机方向/电机 PWM 差速/大倾角保护。
  {
    bfs::CoaxialVtvOutputInput in;
    in.roll_correction_deg = 10.0f;
    in.pitch_correction_deg = -5.0f;
    in.throttle_pct = 50.0f;
    in.yaw_diff_us = 0.0f;
    in.unlocked = true;
    const bfs::CoaxialVtvOutputCalibration cal;
    const bfs::OutputCommandSet out = bfs::mapCoaxialVtvOutputs(in, cal);
    CHECK(out.count == 4U, "P6 共轴四路输出");
    CHECK(std::fabs(out.commands[0].value - 61.111f) < 1e-2f,
          "P6 roll 舵机方向/行程");
    CHECK(std::fabs(out.commands[1].value - 55.556f) < 1e-2f,
          "P6 pitch 舵机反向/行程");
    CHECK(std::fabs(out.commands[2].value - 50.0f) < 1e-2f &&
              std::fabs(out.commands[3].value - 50.0f) < 1e-2f,
          "P6 零差速双电机 50%");

    in.yaw_diff_us = 600.0f;
    const bfs::OutputCommandSet yaw_out = bfs::mapCoaxialVtvOutputs(in, cal);
    CHECK(std::fabs(yaw_out.commands[2].value - 0.0f) < 1e-2f &&
              std::fabs(yaw_out.commands[3].value - 100.0f) < 1e-2f,
          "P6 yaw diff 联合限幅到 0/100%");

    in.yaw_diff_us = 0.0f;
    in.attitude_roll_deg = 50.0f;
    const bfs::OutputCommandSet tilt_out =
        bfs::mapCoaxialVtvOutputs(in, cal);
    CHECK(std::fabs(tilt_out.commands[2].value) < 1e-4f &&
              std::fabs(tilt_out.commands[3].value) < 1e-4f,
          "P6 大倾角保护锁电机");

    in.attitude_roll_deg = 0.0f;
    in.unlocked = false;
    const bfs::OutputCommandSet lock_out =
        bfs::mapCoaxialVtvOutputs(in, cal);
    CHECK(std::fabs(lock_out.commands[2].value) < 1e-4f &&
              std::fabs(lock_out.commands[3].value) < 1e-4f,
          "P6 未解锁锁电机");
  }

  // P7 非有限输入：不丢通道，回落零点并置 valid=false。
  {
    bfs::TandemVecOutputCalibration cal;
    const bfs::OutputCommandSet out = bfs::mapTandemVecOutputs(
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 50.0f, 50.0f, cal);
    CHECK(out.count == 4U, "P6 非有限输入仍保留四通道");
    CHECK(!out.commands[0].valid, "P6 非有限通道 valid=false");
    CHECK(std::fabs(out.commands[0].value - 50.0f) < 1e-4,
          "P6 非有限通道回落舵机零点");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_vehicle_profile: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_vehicle_profile: %d 项失败 ===\n", g_fail);
  return 1;
}
