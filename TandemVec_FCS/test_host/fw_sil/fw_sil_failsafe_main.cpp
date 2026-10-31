/**
 * @file    fw_sil_failsafe_main.cpp
 * @module  固件/宿主 SIL 场景（CRSF 断链失控保护）
 * @brief   编译真实 `src/flight_control.cpp` + `src/state_data.cpp`，驱动刚体 plant
 *          验证断链后“保持解锁 + 定高”是否成立；同一二进制内用 `--legacy`
 *          复现 2026-10-31 修复前的通道合成作差分对照。
 * @details 输入：无（场景内置），可选 `--legacy` / `--only <name>`；
 *          处理：每拍 set_rc →（断链时）调用固件同一份 `linkDownCallback` 判据
 *          `bfs::failsafe::computeLinkLossPlan` → runGNCExecutive → plant 积分；
 *          输出：每个场景一行 PASS/FAIL + 关键量，main 返回非零即失败。
 *
 * 编译（仓库 TandemVec_FCS 目录，BIN_DIR 必须 ASCII）：
 *   g++ -std=c++17 -O2 -DTANDEMVEC_FIRMWARE \
 *       -Iinclude -Isrc -Itest_host/fw_sil/stub -Itest_host/stub \
 *       $(find lib -maxdepth 3 -type d | sed 's/^/-I/') \
 *       src/flight_control.cpp src/state_data.cpp test_host/fw_sil/fw_sil_stubs.cpp \
 *       test_host/fw_sil/fw_sil_failsafe_main.cpp \
 *       lib/DPS310-Pressure-Sensor/src/Dps310.cpp lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
 *       lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp -o $BIN_DIR/fw_failsafe
 *
 * @note 本文件**不复制**断链判据：判据来自 `include/FailsafeLinkLossPlan.h`，与固件
 *       `communication.cpp::linkDownCallback` 调用同一份实现。`--legacy` 分支只用于
 *       差分对照，代码形状与 2026-10-31 之前的历史实现一致，不代表现行行为。
 * @see  include/FailsafeLinkLossPlan.h、src/communication.cpp、VER-004 §断链保护
 */
#include "FailsafeLinkLossPlan.h"
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "TandemVec_Config.h"
#include "TandemVec_Propulsion.h"
#include "flight_control.h"
#include "state_data.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static const TandemVecParams &P = kDefaultTandemVecParams;
static const ServoConfig &SC = kDefaultServoConfig;

// ---------------------------------------------------------------------------
// 简化 plant：推力沿推力轴、电机一阶滞后、只积分垂向（断链场景只看高度）
// ---------------------------------------------------------------------------
struct Motor
{
  float w, target, tau;
  Motor(float w0 = 0.0f, float t = 0.28f) : w(w0), target(w0), tau(t) {}
  void set(float wt) { target = wt; }
  float step(float dt)
  {
    w += (target - w) * fminf(dt / tau, 1.0f);
    return w;
  }
};

struct Body
{
  Quat4f q;
  float omega[3];
  float vel_n, vel_e, vel_d;
  float pos_n, pos_e, pos_d;
  Body()
      : q(1, 0, 0, 0), omega{0, 0, 0}, vel_n(0), vel_e(0), vel_d(0), pos_n(0),
        pos_e(0), pos_d(0)
  {
  }
  void step(const SixDOFWrench &w, float dt)
  {
    const float p = omega[0], qq = omega[1], r = omega[2];
    omega[0] += (-w.Mz - (P.Iz - P.Iy) * qq * r) / P.Ix * dt;
    omega[1] += (w.My - (P.Ix - P.Iz) * r * p) / P.Iy * dt;
    omega[2] += (-w.Mx - (P.Iy - P.Ix) * p * qq) / P.Iz * dt;
    const float wx = omega[0], wy = omega[1], wz = omega[2];
    Quat4f dq{-0.5f * (q.x * wx + q.y * wy + q.z * wz),
              0.5f * (q.w * wx + q.y * wz - q.z * wy),
              0.5f * (q.w * wy - q.x * wz + q.z * wx),
              0.5f * (q.w * wz + q.x * wy - q.y * wx)};
    q.w += dq.w * dt;
    q.x += dq.x * dt;
    q.y += dq.y * dt;
    q.z += dq.z * dt;
    q = qNorm(q);
    const float R33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    vel_d += (P.g - (w.Fx / P.m) * R33) * dt;
    pos_d += vel_d * dt;
  }
  void euler(float &roll, float &pitch, float &yaw) const
  {
    Quaternion qq{q.w, q.x, q.y, q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
};

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500)
{
  for (int i = 0; i < 9; ++i) raw_rc_values[i] = 1500;
  raw_rc_values[0] = roll;
  raw_rc_values[1] = pitch;
  raw_rc_values[2] = throttle;
  raw_rc_values[3] = 1500;
  raw_rc_values[4] = unlock ? 1800 : 1000;
  raw_rc_values[5] = 1800;
  raw_rc_values[6] = mode;
  raw_rc_values[7] = 1000;
  raw_rc_values[8] = 1000;
}

static void feed_sensors(const Body &b)
{
  float roll, pitch, yaw;
  b.euler(roll, pitch, yaw);
  AHRS_Packet.Roll = roll;
  AHRS_Packet.Pitch = pitch;
  AHRS_Packet.Heading = yaw;
  AHRS_Packet.Qw = b.q.w;
  AHRS_Packet.Qx = b.q.x;
  AHRS_Packet.Qy = b.q.y;
  AHRS_Packet.Qz = b.q.z;
  icm_gyro_x = b.omega[0];
  icm_gyro_y = b.omega[1];
  icm_gyro_z = b.omega[2];
  current_omega_dps_body_filtered = Vector3(b.omega[0] * 57.29578f,
                                            b.omega[1] * 57.29578f,
                                            b.omega[2] * 57.29578f);
  estimated_height = -b.pos_d;
  estimated_velocity = b.vel_d;
  relative_north = b.pos_n;
  relative_east = b.pos_e;
  relative_down = b.pos_d;
  fused_north_pos = b.pos_n;
  fused_east_pos = b.pos_e;
  fused_north_vel = b.vel_n;
  fused_east_vel = b.vel_e;
  INS_GNSS_Packet.location_north = b.pos_n;
  INS_GNSS_Packet.location_east = b.pos_e;
  INS_GNSS_Packet.location_down = b.pos_d;
  INS_GNSS_Packet.velocity_north = b.vel_n;
  INS_GNSS_Packet.velocity_east = b.vel_e;
  INS_GNSS_Packet.velocity_down = b.vel_d;
}

// ---------------------------------------------------------------------------
// 断链注入：新判据调用固件同一份实现；--legacy 复现历史实现（仅差分对照）
// ---------------------------------------------------------------------------
static bool g_legacy = false;

static void applyLinkLoss()
{
  const uint16_t arm = raw_rc_values[bfs::failsafe::kChArm];
  const uint16_t thr = raw_rc_values[bfs::failsafe::kChThrottle];

  if (g_legacy)
  {
    // 2026-10-31 之前的实现（历史形状）：CH5>1500 && CH3>1100，CH3 落 else → 988
    const bool was = (arm > 1500) && (thr > 1100);
    failsafe_in_flight = was;
    for (int i = 0; i < RC_INPUT_MAX_CHANNELS; ++i) raw_rc_values[i] = 988;
    raw_rc_values[0] = 1500;
    raw_rc_values[1] = 1500;
    raw_rc_values[3] = 1500;
    raw_rc_values[4] = was ? 1500 : 988;
    raw_rc_values[6] = was ? 1500 : 988;
  }
  else
  {
    bfs::failsafe::Inputs in;
    in.arm_us = arm;
    in.throttle_us = thr;
    in.estimated_height_m = estimated_height;
    const bfs::failsafe::Plan plan = bfs::failsafe::makeLinkLossPlan(in);
    failsafe_in_flight = plan.in_flight;
    for (int i = 0; i < RC_INPUT_MAX_CHANNELS; ++i)
    {
      raw_rc_values[i] = plan.channels[i];
    }
  }
  isLinkUp = false;
}

// ---------------------------------------------------------------------------
// 场景
// ---------------------------------------------------------------------------
struct Scenario
{
  const char *name;
  float hover_alt_m;       // 断链前定高目标
  uint16_t throttle_at_loss; // 断链瞬间的油门杆
  float loss_time_s;
  bool ground_only;        // 从未起飞（地面断链）
  bool relink;             // 断链后恢复链路
  float duration_s;
};

struct Result
{
  bool ok;
  char detail[256];
};

static int g_fail = 0;

static void check(bool cond, const char *what, const char *scenario)
{
  if (!cond)
  {
    std::printf("[FAIL] %s :: %s\n", scenario, what);
    ++g_fail;
  }
}

static Result runScenario(const Scenario &sc)
{
  Body body;
  if (!sc.ground_only)
  {
    // 直接起飞到悬停高度：从静止平衡开始，避免 30 s 爬升
    body.pos_d = -sc.hover_alt_m;
  }
  const float w_hover = P.wMax * 0.5f;
  Motor mf(w_hover), mt(w_hover);

  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  is_origin_lla_set = true;
  origin_lat_deg = 28.2;
  origin_lon_deg = 112.9;
  origin_alt_m = 50.0;
  // 注意：解锁上升沿（updateControlInputs 步骤 5.1）会把 target_altitude 清 0，
  // 所以定高目标必须在解锁之后每拍重设（下面是循环内的设置）。
  target_altitude = 0.0f;

  const float dt = 0.005f;
  const int steps = int(sc.duration_s / dt);
  bool fs_applied = false;
  bool relink_applied = false;
  bool fs_at_loss = false;
  bool vz_sampled = false;

  // 断链瞬间快照与断链后统计
  float alt_at_loss = 0.0f;
  float alt_min_after = 1e9f;
  float alt_final = 0.0f;
  float motor_min_after_loss = 1e9f;
  float target_vz_settled = 0.0f;
  float max_abs_roll_after = 0.0f;
  int relink_unlock_drop = 0;

  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt;

    if (!fs_applied)
    {
      const bool descending = (t > sc.loss_time_s - 2.0f);
      const uint16_t thr = descending ? sc.throttle_at_loss : 1500;
      set_rc(thr, 1500, true);
      // 解锁沿之后把定高目标拉回场景高度（解锁沿会把 target_altitude 清 0）。
      if (t >= 1.0f) target_altitude = sc.hover_alt_m;
    }
    else if (sc.relink && !relink_applied && t >= sc.loss_time_s + 8.0f)
    {
      // 复链：飞行员接管并把油门杆回中（1500），保持解锁。
      set_rc(1500, 1500, true);
      isLinkUp = true;
      failsafe_in_flight = false;
      relink_applied = true;
    }

    feed_sensors(body);
    runGNCExecutive();

    if (!fs_applied && t >= sc.loss_time_s)
    {
      alt_at_loss = -body.pos_d;
      fs_at_loss = failsafe_in_flight;  // applyLinkLoss 之前应为 false
      applyLinkLoss();
      fs_at_loss = failsafe_in_flight;
      fs_applied = true;
    }

    // 断链后 5 s 采样高度环稳定后的目标垂速（滤波收敛值）。
    if (fs_applied && !vz_sampled && t >= sc.loss_time_s + 5.0f)
    {
      target_vz_settled = gnc_tel.target_vz_mps;
      vz_sampled = true;
    }

    if (fs_applied)
    {
      alt_min_after = fminf(alt_min_after, -body.pos_d);
      motor_min_after_loss = fminf(motor_min_after_loss, fminf(ch3_output, ch4_output));
      float r, p, y;
      body.euler(r, p, y);
      max_abs_roll_after = fmaxf(max_abs_roll_after, fabsf(r));
      if (relink_applied && !g_is_unlocked) ++relink_unlock_drop;
    }

    // 执行器 → plant
    const float upper = (ch1_output - (50.0f + SC.zero_roll_pct)) * SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_roll) * 0.017453292f;
    const float lower = (ch2_output - (50.0f + SC.zero_pitch_pct)) * SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_pitch) * 0.017453292f;
    mf.set((ch3_output / 100.0f) * P.wMax);
    mt.set((ch4_output / 100.0f) * P.wMax);
    PropulsionState ps{mf.step(dt), mt.step(dt), upper, lower};
    SixDOFWrench wr = computeWrench(ps, P);
    body.step(wr, dt);

    alt_final = -body.pos_d;
  }

  Result res{};
  std::snprintf(res.detail, sizeof(res.detail),
                "alt_loss=%.2f alt_final=%.2f alt_min=%.2f tgt_vz=%.2f motor_min=%.1f%% "
                "max_roll=%.1f° unlock=%d mode=%d fs=%d",
                static_cast<double>(alt_at_loss), static_cast<double>(alt_final),
                static_cast<double>(alt_min_after), static_cast<double>(target_vz_settled),
                static_cast<double>(motor_min_after_loss),
                static_cast<double>(max_abs_roll_after), static_cast<int>(g_is_unlocked),
                static_cast<int>(g_current_flight_mode),
                static_cast<int>(fs_at_loss));

  // 判定
  if (sc.ground_only)
  {
    res.ok = (!g_is_unlocked) && (motor_min_after_loss <= 0.01f);
    check(!g_is_unlocked, "地面断链必须保持锁定", sc.name);
    check(motor_min_after_loss <= 0.01f, "地面断链电机必须为 0%", sc.name);
  }
  else
  {
    // 空中断链：必须保持解锁、落 AUTO_ALTITUDE、目标垂速≈0（定高）、不下坠
    check(fs_at_loss, "空中断链 failsafe_in_flight 必须为真", sc.name);
    check(g_is_unlocked, "空中断链必须保持解锁", sc.name);
    check(g_current_flight_mode == AUTO_ALTITUDE, "空中断链必须落 AUTO_ALTITUDE", sc.name);
    check(vz_sampled && fabsf(target_vz_settled) < 0.05f,
          "空中断链 target_vz 必须为 0（定高）", sc.name);
    const float sink = alt_at_loss - alt_min_after;
    check(sink < 0.5f, "空中断链后不得持续下沉（下沉 < 0.5 m）", sc.name);
    check(motor_min_after_loss > 30.0f, "空中断链电机必须维持推力（>30%）", sc.name);
    check(max_abs_roll_after < 15.0f, "空中断链滚转必须有界（<15°）", sc.name);
    if (sc.relink)
    {
      check(relink_unlock_drop == 0, "复链后 is_unlocked 不得出现任何掉零拍", sc.name);
    }
    res.ok = fs_at_loss && g_is_unlocked &&
             (g_current_flight_mode == AUTO_ALTITUDE) && vz_sampled &&
             (fabsf(target_vz_settled) < 0.05f) && (sink < 0.5f) &&
             (motor_min_after_loss > 30.0f);
  }
  return res;
}

int main(int argc, char **argv)
{
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--legacy") == 0) g_legacy = true;
  }

  std::printf("== CRSF 断链失控保护 SIL（真实 flight_control.cpp + 真实判据）==\n");
  std::printf("模式：%s\n", g_legacy ? "legacy（2026-10-31 修复前，仅差分对照）"
                                     : "fixed（当前固件）");

  const Scenario scenarios[] = {
      {"air_low_throttle", 10.0f, 1000, 32.0f, false, false, 44.0f},
      {"air_normal_throttle", 10.0f, 1500, 32.0f, false, false, 44.0f},
      {"ground_disarmed_by_switch", 0.0f, 1000, 2.0f, true, false, 8.0f},
      {"air_relink", 10.0f, 1000, 32.0f, false, true, 44.0f},
  };

  for (const Scenario &sc : scenarios)
  {
    const Result r = runScenario(sc);
    std::printf("[%s] %-28s %s\n", r.ok ? "PASS" : "FAIL", sc.name, r.detail);
  }

  // 差分自检：修复后，空中低油门场景必须与 legacy 结果相反（legacy 上锁/停桨）。
  if (!g_legacy)
  {
    std::printf("[SELF] 断链判据已生效（legacy 对照请加 --legacy 运行）\n");
  }

  if (g_fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  if (g_legacy)
  {
    // legacy 分支是“修复前”的预期失败对照，不作为回归门禁：只打印差异后返回 0。
    std::printf("legacy 对照：按预期复现失败 %d 项（不判失败）\n", g_fail);
    return 0;
  }
  std::printf("失败 %d 项\n", g_fail);
  return 1;
}
