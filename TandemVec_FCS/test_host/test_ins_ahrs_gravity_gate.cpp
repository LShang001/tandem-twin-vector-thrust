/**
 * @file    test_ins_ahrs_gravity_gate.cpp
 * @module  固件/宿主测试（fw-tests：AHRS 重力方向门）
 * @brief   验证 ins_ahrs_gravity_gate.h：比力方向与预测重力夹角门限、
 *          VTOL 包线内不启用/导弹高倾角或高过载启用、零输入 fail-closed。
 * @details 10° 方向门；15° 倾角/1.5 g 包线判据；纯算法输入输出。
 * @frame   N/A（输入为机体系方向矢量）
 * @units   rad、m/s²、g
 */
#include "ins_ahrs_gravity_gate.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s\n", msg);                                            \
      ++g_fail;                                                                 \
    }                                                                           \
    else                                                                        \
    {                                                                           \
      std::printf("PASS: %s\n", msg);                                            \
    }                                                                           \
  } while (0)

int main()
{
  const float deg = 0.0174532925f;
  const float limit = 10.0f * deg;
  {
    const Eigen::Vector3f expected(0.0f, 0.0f, -1.0f);
    const Eigen::Vector3f accel(0.0f, 0.0f, -9.8f);
    CHECK(InsAhrsGravityDirectionAllowed(expected, accel, limit),
          "T1 水平静止：比力=重力方向，放行");
  }
  {
    const Eigen::Vector3f expected(0.0f, 0.0f, -1.0f);
    const Eigen::Vector3f accel(std::sin(8.0f * deg) * 9.8f, 0.0f,
                                -std::cos(8.0f * deg) * 9.8f);
    CHECK(InsAhrsGravityDirectionAllowed(expected, accel, limit),
          "T2 8° 小倾角：仍在门限内，放行");
  }
  {
    // 无坡度转弯：升力沿机体系 z，但 EKF 姿态 20° 俯仰，预测重力方向
    // 与比力方向差 20° → 拒绝（否则 AHRS 会把机体拉平）。
    const Eigen::Vector3f expected(std::sin(20.0f * deg), 0.0f,
                                   -std::cos(20.0f * deg));
    const Eigen::Vector3f accel(0.0f, 0.0f, -9.8f);
    CHECK(!InsAhrsGravityDirectionAllowed(expected, accel, limit),
          "T3 20° 俯仰无坡度转弯：方向不符，拒绝");
  }
  {
    const Eigen::Vector3f expected(0.0f, 0.0f, -1.0f);
    const Eigen::Vector3f accel(std::sin(15.0f * deg) * 9.8f, 0.0f,
                                -std::cos(15.0f * deg) * 9.8f);
    CHECK(!InsAhrsGravityDirectionAllowed(expected, accel, limit),
          "T4 15° 倾角：超过 10° 门限，拒绝");
  }
  {
    CHECK(!InsAhrsGravityGateRequired(10.0f * deg, 14.0f * deg, 1.2f),
          "T5 VTOL 包线内（roll 10°/pitch 14°/1.2 g）：不启用方向门");
    CHECK(InsAhrsGravityGateRequired(0.0f, 20.0f * deg, 1.0f),
          "T6 导弹俯仰 20°：启用方向门");
    CHECK(InsAhrsGravityGateRequired(0.0f, 0.0f, 6.0f),
          "T7 6 g 过载：启用方向门");
  }
  {
    const Eigen::Vector3f expected(0.0f, 0.0f, -1.0f);
    CHECK(!InsAhrsGravityDirectionAllowed(expected, Eigen::Vector3f::Zero(),
                                          limit),
          "T8 零比力：拒绝");
    CHECK(!InsAhrsGravityDirectionAllowed(Eigen::Vector3f::Zero(),
                                          Eigen::Vector3f(0.0f, 0.0f, -9.8f),
                                          limit),
          "T6 零参考方向：拒绝");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_ins_ahrs_gravity_gate: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_ins_ahrs_gravity_gate: %d 项失败 ===\n", g_fail);
  return 1;
}
