/**
 * @file    test_liquid_rocket_sil.cpp
 * @module  固件/宿主回归测试（fw-tests：液体火箭闭环 SIL，L1）
 * @brief   用 LiquidRocketPropulsion + mapNozzleGimbal 驱动简化 6DOF plant，
 *          验证点火、变质量、推力/阀门限幅、喷管姿态稳定与垂直回收闭环
 * @details 被测：`include/LiquidRocketModel.h`（推进/质量/喷管几何）。
 *          本文件自包含一个 6DOF plant 与简单垂直回收 PD 制导；它不是固件
 *          飞行模式，也不复制外部 CCAEGT 制导库。
 *          输入：CCAEGT 示例参数（干重 13.5 kg、初始推进剂 3 kg、Tmax 200 N、
 *          Tmin 80 N、Isp 200 s）；
 *          处理：30 m 高度点火下降 → 速度剖面跟踪 → 触地关机；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Isrc test_host/test_liquid_rocket_sil.cpp
 *                -o bin/lrsil
 * @theory  T=u·Tmax；m_dot=T/(g0·Isp)；v_target=−√(2·a·h)；喷管两轴 asin 几何
 *          @ref CTL-003 §3.3；PROP-001 §2；外部 VTVL_LiquidRocket_FC（仅参考）
 * @frame   NED 位置/速度 + 机体系四元数；推力沿机体 −z
 * @units   m、m/s、N、kg、s、deg
 * @note    ★ 参数不是本机标定值；无气动、无泵压/室压动态、无 RCS/多级、
 *          无风/传感器噪声。通过只证明“模型+几何+简单制导逻辑闭环有界”，
 *          不是液体火箭实机/着陆安全证据。
 * @test    L1 点火与质量消耗；L2 垂直回收收敛；L3 姿态有界；L4 触地关机
 * @see     include/LiquidRocketModel.h、CTL-003 §3.3
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif

#include "LiquidRocketModel.h"
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

int main(int argc, char **argv)
{
  double h0 = 30.0;
  double prop0 = 3.0;
  double roll0_deg = 3.0;
  double pitch0_deg = -2.0;
  for (int i = 1; i + 1 < argc; i += 2)
  {
    if (std::strcmp(argv[i], "--h0") == 0) h0 = std::atof(argv[i + 1]);
    else if (std::strcmp(argv[i], "--prop") == 0) prop0 = std::atof(argv[i + 1]);
    else if (std::strcmp(argv[i], "--roll0") == 0) roll0_deg = std::atof(argv[i + 1]);
    else if (std::strcmp(argv[i], "--pitch0") == 0) pitch0_deg = std::atof(argv[i + 1]);
  }

  bfs::LiquidRocketParams params;
  params.dry_mass_kg = 13.5f;
  params.initial_propellant_kg = static_cast<float>(prop0);
  params.max_thrust_n = 200.0f;
  params.min_thrust_n = 80.0f;
  params.isp_s = 200.0f;
  params.g0_mps2 = 9.80665f;
  params.thrust_rate_limit_nps = 1800.0f;
  params.valve_rate_limit_per_s = 10.0f;

  bfs::LiquidRocketPropulsion engine;
  CHECK(engine.init(params), "L1 初始化");

  const double dt = 0.005;
  const double g = params.g0_mps2;
  Quaternion q = eulerToQuaternion(roll0_deg * DEG_TO_RAD,
                                    pitch0_deg * DEG_TO_RAD, 0.0);
  Vector3 omega{0.0, 0.0, 0.0};
  Vector3 vel{0.0, 0.0, 0.0};
  Vector3 pos{0.0, 0.0, -h0}; // NED down：-h0 m 高度
  const double Ix = 0.8, Iy = 0.8, Iz = 0.5;
  const double lever = 0.4;
  const double a_brake = 1.5;
  bool landed = false;
  double landing_v = 0.0;
  double max_roll = 0.0, max_pitch = 0.0;
  double min_prop = params.initial_propellant_kg;
  bool ignited_seen = false;
  bool flameout_seen = false;

  for (int i = 0; i < 8000; ++i) // 40 s
  {
    const double h = -pos.z;
    const double v_up = -vel.z;

    float roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    max_roll = std::fmax(max_roll, std::fabs(roll * RAD_TO_DEG));
    max_pitch = std::fmax(max_pitch, std::fabs(pitch * RAD_TO_DEG));

    // 简单垂直回收制导：速度剖面 + 速度 PD → 推力需求。
    const double v_target =
        (h > 0.5) ? -std::fmin(20.0, std::sqrt(2.0 * a_brake * h)) : -0.5;
    const double v_err = v_target - v_up;
    const double cos_tilt = std::fmax(0.5, std::cos(roll) * std::cos(pitch));
    double thrust_req = engine.state().mass_kg * (g + 2.0 * v_err) / cos_tilt;
    bool engine_on = h > 0.2 && !landed;
    float throttle = 0.0f;
    if (engine_on)
    {
      thrust_req = std::fmax(static_cast<double>(params.min_thrust_n),
                             std::fmin(static_cast<double>(params.max_thrust_n),
                                       thrust_req));
      throttle = static_cast<float>(thrust_req / params.max_thrust_n);
    }

    bfs::LiquidRocketCommand cmd;
    cmd.throttle = throttle;
    cmd.ignition_enabled = true;
    cmd.unlocked = true;
    cmd.dt_s = static_cast<float>(dt);
    const auto eng = engine.update(cmd);
    if (!eng.valid) { std::printf("FAIL: L1 模型无效\n"); g_fail++; break; }
    ignited_seen = ignited_seen || eng.ignited;
    flameout_seen = flameout_seen || eng.flameout;
    min_prop = std::fmin(min_prop, eng.propellant_kg);

    // 姿态 PD：目标水平；输出归一化横向推力 → 双轴喷管角。
    const float roll_err = -static_cast<float>(roll * RAD_TO_DEG);
    const float pitch_err = -static_cast<float>(pitch * RAD_TO_DEG);
    const float roll_cmd = std::fmax(-1.0f, std::fmin(1.0f,
        roll_err / 20.0f - static_cast<float>(omega.x * RAD_TO_DEG) * 0.05f));
    const float pitch_cmd = std::fmax(-1.0f, std::fmin(1.0f,
        pitch_err / 20.0f - static_cast<float>(omega.y * RAD_TO_DEG) * 0.05f));
    const auto gimbal = bfs::mapNozzleGimbal(roll_cmd, pitch_cmd, 10.0f);

    // 推力矢量：机体 −z + 两轴摆角；摆角产生力矩。
    const double T = eng.thrust_n;
    const double gx = gimbal.valid ? gimbal.x_deg * DEG_TO_RAD : 0.0;
    const double gy = gimbal.valid ? gimbal.y_deg * DEG_TO_RAD : 0.0;
    Vector3 f_body{T * std::sin(gy), T * std::sin(gx),
                   -T * std::cos(gx) * std::cos(gy)};
    Vector3 f_ned = rotateVector(q, f_body);
    const double m = eng.mass_kg;
    Vector3 acc{f_ned.x / m, f_ned.y / m, f_ned.z / m + g};

    const double mx = lever * T * std::sin(gx);
    const double my = lever * T * std::sin(gy);
    const double mz = 0.0; // 无 RCS/差速偏航
    const double wx = omega.x, wy = omega.y, wz = omega.z;
    omega.x += (mx - (Iz - Iy) * wy * wz) / Ix * dt;
    omega.y += (my - (Ix - Iz) * wz * wx) / Iy * dt;
    omega.z += (mz - (Iy - Ix) * wx * wy) / Iz * dt;

    const Quaternion dq = quaternionMultiply(
        q, Quaternion(0.0, omega.x * 0.5 * dt, omega.y * 0.5 * dt,
                      omega.z * 0.5 * dt));
    q.w += dq.w; q.x += dq.x; q.y += dq.y; q.z += dq.z;
    q = normalizeQuaternion(q);

    vel.x += acc.x * dt; vel.y += acc.y * dt; vel.z += acc.z * dt;
    pos.x += vel.x * dt; pos.y += vel.y * dt; pos.z += vel.z * dt;

    if (pos.z >= 0.0)
    {
      // 触地：位置钳地，记录接地速度；若仍在下沉则视为硬着陆。
      pos.z = 0.0;
      landing_v = vel.z; // NED down 为正的接地下沉速度
      landed = true;
      break;
    }
  }

  float roll, pitch, yaw;
  quaternionToEuler(q, roll, pitch, yaw);
  std::printf("L-SIL landed=%d h=%.2f v=%.2f m=%.2f prop=%.2f "
              "roll=%.2f pitch=%.2f max_roll=%.2f max_pitch=%.2f\n",
              landed ? 1 : 0, -pos.z, landing_v, engine.state().mass_kg,
              engine.state().propellant_kg, roll * RAD_TO_DEG,
              pitch * RAD_TO_DEG, max_roll, max_pitch);

  CHECK(ignited_seen, "L1 点火锁存");
  CHECK(engine.state().mass_kg < params.dry_mass_kg +
                                     params.initial_propellant_kg,
        "L1 变质量消耗");
  CHECK(min_prop < params.initial_propellant_kg, "L1 推进剂下降");
  CHECK(landed, "L2 触地");
  CHECK(landing_v > 0.0 && landing_v < 4.0, "L2 触地速度有界");
  CHECK(std::fabs(roll * RAD_TO_DEG) < 5.0, "L3 滚转收敛");
  CHECK(std::fabs(pitch * RAD_TO_DEG) < 5.0, "L3 俯仰收敛");
  CHECK(max_roll < 20.0 && max_pitch < 20.0, "L3 姿态有界");
  CHECK(!flameout_seen, "L4 未意外熄火");
  CHECK(std::isfinite(engine.state().mass_kg) &&
            std::isfinite(engine.state().thrust_n),
        "L4 状态有限");

  if (g_fail == 0)
  {
    std::printf("=== test_liquid_rocket_sil: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_liquid_rocket_sil: %d 项失败 ===\n", g_fail);
  return 1;
}
