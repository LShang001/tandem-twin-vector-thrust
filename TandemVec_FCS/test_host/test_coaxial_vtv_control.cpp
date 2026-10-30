/**
 * @file    test_coaxial_vtv_control.cpp
 * @module  固件/宿主回归测试（fw-tests：共轴双桨 VTVL DirectTvcRate）
 * @brief   验证共轴控制律的通道极性、限幅/抗饱和、手动 TVC 旁路、偏航差速，
 *          并用简化 6DOF 刚体 plant 做姿态/角速率闭环收敛验证
 * @details 被测：`include/CoaxialVtvControl.h`。
 *          输入：合成姿态误差/角速率/偏航目标；
 *          处理：CoaxialVtvController::update；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Isrc test_host/test_coaxial_vtv_control.cpp
 *                -o bin/cvc
 * @theory  ω_ref=Kp_att·e；u=Kp_rate(ω_ref−ω)+Ki_rate∫；yaw_diff=PI(ω_ref−ω_z)
 *          @ref CTL-003 §2.2；CTL-001 §4–§5
 * @frame   存档系 x=滚转、y=俯仰、z=推力轴
 * @units   deg、deg/s、us、s
 * @note    6DOF plant 是简化 L1 模型（无气动/无电机滞后/无传感器），只验证
 *          控制律极性与闭环稳定性，不代表实机/台架结果。
 * @test    T1 零输入；T2 姿态 P 极性；T3 速率阻尼；T4 手动 TVC（α0.25 滤波）；
 *          T5 饱和；T6 偏航差速；T7 非法输入；T8 6DOF 闭环；T9 杆量映射；
 *          T10 RATE 内环积分累积
 * @see     include/CoaxialVtvControl.h、include/ActuatorOutputMap.h、CTL-003 §2.2
 */
#include <cmath>
#include <cstdio>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif

#include "CoaxialVtvControl.h"
#include "QuaternionMath.h"

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

static bool nearf(const float a, const float b, const float tol = 1e-3f)
{
  return std::fabs(a - b) <= tol;
}

static bfs::CoaxialVtvInput baseInput()
{
  bfs::CoaxialVtvInput in;
  in.attitude_active = true;
  in.attitude_integral_active = true;
  in.rate_integral_active = true;
  in.yaw_active = true;
  in.yaw_integral_active = true;
  in.dt_s = 0.005f;
  return in;
}

int main()
{
  // T1 零误差/零速率：输出 0。
  {
    bfs::CoaxialVtvController c;
    const auto out = c.update(baseInput());
    CHECK(out.valid, "T1 有效");
    CHECK(nearf(out.roll_correction_deg, 0.0f) &&
              nearf(out.pitch_correction_deg, 0.0f) &&
              nearf(out.yaw_diff_us, 0.0f),
          "T1 零输入零输出");
  }

  // T2 正滚转误差 → 正修正；正俯仰误差 → 正修正。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = 5.0f;
    in.error_pitch_deg = 5.0f;
    const auto out = c.update(in);
    CHECK(out.roll_correction_deg > 0.0f, "T2 roll 极性");
    CHECK(out.pitch_correction_deg > 0.0f, "T2 pitch 极性");
  }

  // T2b RATE_MODE：直接使用速率目标，不经过姿态 P。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.rate_mode = true;
    in.roll_rate_target_dps = 30.0f;
    in.pitch_rate_target_dps = -20.0f;
    const auto out = c.update(in);
    CHECK(out.roll_correction_deg > 0.0f, "T2b rate roll 极性");
    CHECK(out.pitch_correction_deg < 0.0f, "T2b rate pitch 极性");
  }

  // T3 速率阻尼：零误差、正角速率 → 负修正。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.rate_roll_dps = 20.0f;
    in.rate_pitch_dps = 20.0f;
    const auto out = c.update(in);
    CHECK(out.roll_correction_deg < 0.0f, "T3 roll 速率阻尼");
    CHECK(out.pitch_correction_deg < 0.0f, "T3 pitch 速率阻尼");
  }

  // T4 手动 TVC：摇杆目标角映射后过 α0.25 输出滤波（原始 else 分支语义），
  // 首拍 1.0/-0.75，持续输入收敛到 4.0/-3.0；偏航差速 0。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.manual_tvc = true;
    in.manual_roll_deg = 4.0f;
    in.manual_pitch_deg = -3.0f;
    const auto first = c.update(in);
    CHECK(nearf(first.roll_correction_deg, 1.0f), "T4 手动 roll 首拍 1.0");
    CHECK(nearf(first.pitch_correction_deg, -0.75f), "T4 手动 pitch 首拍 -0.75");
    bfs::CoaxialVtvOutput out = first;
    for (int i = 0; i < 40; ++i) out = c.update(in);
    CHECK(nearf(out.roll_correction_deg, 4.0f), "T4 手动 roll 收敛");
    CHECK(nearf(out.pitch_correction_deg, -3.0f), "T4 手动 pitch 收敛");
    CHECK(nearf(out.yaw_diff_us, 0.0f), "T4 手动 yaw=0");
    auto normal = baseInput();
    normal.error_roll_deg = 5.0f;
    const auto out2 = c.update(normal);
    CHECK(out2.roll_correction_deg >= 0.0f, "T4 切回自动可恢复");
  }

  // T5 饱和：大误差最终被 ±MAX_CORRECTION 限幅并置饱和。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = 1000.0f;
    in.error_pitch_deg = 1000.0f;
    bfs::CoaxialVtvOutput out;
    for (int i = 0; i < 100; ++i) out = c.update(in);
    CHECK(nearf(out.roll_correction_deg, c.params().max_correction_deg, 1e-3f),
          "T5 roll 限幅");
    CHECK(nearf(out.pitch_correction_deg, c.params().max_correction_deg, 1e-3f),
          "T5 pitch 限幅");
    CHECK(out.saturated, "T5 饱和标记");
  }

  // T6 偏航差速：正目标速率 → 正差速；负目标 → 负差速；大误差限幅。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.yaw_rate_target_dps = 30.0f;
    bfs::CoaxialVtvOutput out;
    for (int i = 0; i < 20; ++i) out = c.update(in);
    CHECK(out.yaw_diff_us > 0.0f, "T6 正偏航目标");
    in.yaw_rate_target_dps = -30.0f;
    for (int i = 0; i < 20; ++i) out = c.update(in);
    CHECK(out.yaw_diff_us < 0.0f, "T6 负偏航目标");
    in.yaw_rate_target_dps = 1000.0f;
    for (int i = 0; i < 100; ++i) out = c.update(in);
    CHECK(std::fabs(out.yaw_diff_us) <= c.params().max_yaw_diff_us + 1e-3f,
          "T6 偏航限幅");
  }

  // T7 非有限输入：invalid 且输出回零。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = std::nanf("");
    const auto out = c.update(in);
    CHECK(!out.valid, "T7 NaN 无效");
    CHECK(nearf(out.roll_correction_deg, 0.0f) &&
              nearf(out.yaw_diff_us, 0.0f),
          "T7 NaN 回零");
  }

  // T8 简化 6DOF 闭环：5°/3° 初始姿态 + 10°/s 初始偏航速率。
  {
    bfs::CoaxialVtvController c;
    const double dt = 0.005;
    const double m = 1.5;
    const double g = 9.80665;
    Quaternion q = eulerToQuaternion(5.0 * DEG_TO_RAD, 3.0 * DEG_TO_RAD,
                                      0.0);
    Vector3 omega{0.0, 0.0, 10.0 * DEG_TO_RAD};
    Vector3 vel{0.0, 0.0, 0.0};
    Vector3 pos{0.0, 0.0, 0.0};
    const double Ix = 0.05, Iy = 0.05, Iz = 0.08;
    const double lever = 0.15;
    const double thrust_n = m * g;
    const double k_yaw = 0.0005;
    const Quaternion q_target; // identity

    double max_abs_roll = 0.0, max_abs_pitch = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
      // 姿态误差：q_err = q_current⁻¹ ⊗ q_target
      const Quaternion q_err = quaternionMultiply(quaternionConjugate(q),
                                                  q_target);
      const float sign_qw = (q_err.w >= 0.0) ? 1.0f : -1.0f;
      const float qv = std::sqrt(static_cast<float>(q_err.x * q_err.x +
                                                   q_err.y * q_err.y +
                                                   q_err.z * q_err.z));
      const float scale = (qv > 0.25f)
                              ? 2.0f * std::atan2(qv, std::fabs(static_cast<float>(q_err.w))) /
                                    qv * static_cast<float>(RAD_TO_DEG)
                              : 2.0f * static_cast<float>(RAD_TO_DEG);
      bfs::CoaxialVtvInput in;
      in.attitude_active = true;
      in.attitude_integral_active = true;
      in.rate_integral_active = true;
      in.yaw_active = true;
      in.yaw_integral_active = true;
      in.dt_s = static_cast<float>(dt);
      in.error_roll_deg = sign_qw * static_cast<float>(q_err.x) * scale;
      in.error_pitch_deg = sign_qw * static_cast<float>(q_err.y) * scale;
      in.rate_roll_dps = static_cast<float>(omega.x * RAD_TO_DEG);
      in.rate_pitch_dps = static_cast<float>(omega.y * RAD_TO_DEG);
      in.rate_yaw_dps = static_cast<float>(omega.z * RAD_TO_DEG);
      in.yaw_rate_target_dps = 0.0f;
      const auto out = c.update(in);
      if (!out.valid) { std::printf("FAIL: T8 controller invalid\n"); g_fail++; break; }

      const double mx = lever * thrust_n *
                        std::sin(out.roll_correction_deg * DEG_TO_RAD);
      const double my = lever * thrust_n *
                        std::sin(out.pitch_correction_deg * DEG_TO_RAD);
      const double mz = k_yaw * out.yaw_diff_us;
      const double wx = omega.x, wy = omega.y, wz = omega.z;
      const double wdot_x = (mx - (Iz - Iy) * wy * wz) / Ix;
      const double wdot_y = (my - (Ix - Iz) * wz * wx) / Iy;
      const double wdot_z = (mz - (Iy - Ix) * wx * wy) / Iz;
      omega.x += wdot_x * dt;
      omega.y += wdot_y * dt;
      omega.z += wdot_z * dt;

      const Quaternion dq = quaternionMultiply(
          q, Quaternion(0.0, omega.x * 0.5 * dt, omega.y * 0.5 * dt,
                        omega.z * 0.5 * dt));
      q.w += dq.w; q.x += dq.x; q.y += dq.y; q.z += dq.z;
      q = normalizeQuaternion(q);

      // 推力：机体 -z，旋转到 NED；重力 +NED z。
      Vector3 f_body{0.0, 0.0, -thrust_n};
      Vector3 f_ned = rotateVector(q, f_body);
      Vector3 acc{f_ned.x / m, f_ned.y / m, f_ned.z / m + g};
      vel.x += acc.x * dt; vel.y += acc.y * dt; vel.z += acc.z * dt;
      pos.x += vel.x * dt; pos.y += vel.y * dt; pos.z += vel.z * dt;

      float roll, pitch, yaw;
      quaternionToEuler(q, roll, pitch, yaw);
      max_abs_roll = std::fmax(max_abs_roll, std::fabs(roll * RAD_TO_DEG));
      max_abs_pitch = std::fmax(max_abs_pitch, std::fabs(pitch * RAD_TO_DEG));
    }
    float roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    std::printf("T8 final roll=%.3f pitch=%.3f yawrate=%.3f dps pos=(%.2f,%.2f,%.2f)\n",
                roll * RAD_TO_DEG, pitch * RAD_TO_DEG, omega.z * RAD_TO_DEG,
                pos.x, pos.y, pos.z);
    CHECK(std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw),
          "T8 有限");
    CHECK(std::fabs(roll * RAD_TO_DEG) < 2.0f, "T8 滚转收敛");
    CHECK(std::fabs(pitch * RAD_TO_DEG) < 2.0f, "T8 俯仰收敛");
    CHECK(std::fabs(omega.z * RAD_TO_DEG) < 5.0f, "T8 偏航速率收敛");
    CHECK(max_abs_roll < 30.0f && max_abs_pitch < 30.0f, "T8 姿态有界");
    CHECK(std::fabs(pos.z) < 3.0f, "T8 高度有界");
  }

  // T9 杆量→目标角速率线性映射（原始 mapFloat(raw,988,2012,∓80,±80)）。
  {
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(988.0f, 80.0f, false), -80.0f),
          "T9 988→-80 dps");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2012.0f, 80.0f, false), 80.0f),
          "T9 2012→+80 dps");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(1500.0f, 80.0f, false), 0.0f),
          "T9 中位→0");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2012.0f, 80.0f, true), -80.0f),
          "T9 俯仰反向");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(1900.0f, 80.0f, false),
                400.0f / 512.0f * 80.0f),
          "T9 线性中点");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2100.0f, 80.0f, false), 80.0f),
          "T9 越界钳位");
  }

  // T10 RATE 内环积分跨拍累积：两模式都使能；断开积分时输出停在 P 分量。
  {
    bfs::CoaxialVtvController a;
    bfs::CoaxialVtvController b;
    auto in = baseInput();
    in.rate_mode = true;
    in.roll_rate_target_dps = 30.0f;
    in.rate_roll_dps = 0.0f;
    bfs::CoaxialVtvOutput oa;
    bfs::CoaxialVtvOutput ob;
    for (int i = 0; i < 60; ++i)
    {
      oa = a.update(in);
      in.rate_integral_active = false;
      ob = b.update(in);
      in.rate_integral_active = true;
    }
    CHECK(oa.roll_correction_deg > ob.roll_correction_deg + 0.5f,
          "T10 RATE 内环积分持续累积");
    CHECK(oa.roll_correction_deg > 7.5f && oa.roll_correction_deg < 10.2f,
          "T10 积分累积叠加 P 分量（未饱和）");
    CHECK(ob.roll_correction_deg > 0.0f, "T10 无积分仍有 P 响应");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_coaxial_vtv_control: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_coaxial_vtv_control: %d 项失败 ===\n", g_fail);
  return 1;
}
