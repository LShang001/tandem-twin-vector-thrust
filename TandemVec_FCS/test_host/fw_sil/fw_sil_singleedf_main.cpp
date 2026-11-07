/**
 * @file    fw_sil_singleedf_main.cpp
 * @module  固件/宿主 SIL（单涵道四燃气舵：真实固件闭环入口）
 * @brief   链接真实 flight_control.cpp + state_data.cpp，跑单涵道构型的定高/手动闭环
 * @details 被测：真实固件 `runGNCExecutive()` 在 `BFS_SINGLE_EDF_4VANE` 下的控制链
 *          （速率环 α 域 → SingleEdf4VaneAllocator → 4 舵 + 推进器）。本文件不复制
 *          控制律，只提供单涵道 6DOF plant、传感器直注与执行器反解。
 *          输入：`--manual` 时给 roll/pitch 阶跃，否则给 AUTO_ALTITUDE 10 m 目标；
 *          处理：固件照常跑 200 Hz；每拍把 ch1–ch4（舵 PWM%）与
 *          `outputs.throttle_percent` 反解成物理舵角/油门送进 plant；
 *          输出：每 0.2 s 打印高度/姿态/四舵/油门，结束打印汇总。
 *          运行（TandemVec_FCS 目录）：bash test_host/fw_sil/build_and_run_singleedf.sh
 * @theory  闭环即真实固件的姿态环/定高环；plant 侧 Wrench 见 single_edf_plant.h
 *          （τ_j = k·Σ_i H[j][i]·δ_i，是分配器逆解的严格对偶）。@ref CTL-008 §3
 * @frame   plant 的 NED 位置/速度 + 机体系角速率；单涵道沿用"roll 绕推力轴"语义，
 *          不做存档系轴置换（与 SingleEdf4VaneAllocator 一致）
 * @units   高度 m，姿态 deg，舵机 PWM%，油门 %，时间 s
 * @note    与实机差异：无气动、无风、无传感器噪声与延迟、舵机理想瞬时；
 *          是"控制律与分配链能否收敛"的最小闭环证据，**不是**飞行品质证据。
 * @test    S1 AUTO_ALTITUDE 定高从 0 收敛到 10 m；S2 --manual 下 roll/pitch 阶跃响应收敛
 * @see     test_host/fw_sil/README.md、single_edf_plant.h、
 *          docs/05-控制与分配/CTL-008-单涵道四燃气舵机型移植-2026-11-05.md
 */
#include "state_data.h"
#include "flight_control.h"
#include "TandemVec_Config.h"
#include "single_edf_plant.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static const SingleEdf4VaneHwParams &P = kSingleEdf4VaneHwParamsDefaults;

using Body = SilSingleEdfBody;

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500)
{
  for (int i = 0; i < 9; ++i) raw_rc_values[i] = 1500;
  raw_rc_values[0] = roll;
  raw_rc_values[1] = pitch;
  raw_rc_values[2] = throttle;
  raw_rc_values[3] = 1500;
  raw_rc_values[4] = unlock ? 1800 : 1000;
  raw_rc_values[5] = 1800; // CH6 推进使能
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
  AHRS_Packet.Qw = b.q.w; AHRS_Packet.Qx = b.q.x;
  AHRS_Packet.Qy = b.q.y; AHRS_Packet.Qz = b.q.z;

  icm_gyro_x = b.omega[0];
  icm_gyro_y = b.omega[1];
  icm_gyro_z = b.omega[2];
  current_omega_dps_body_filtered = Vector3(b.omega[0] * 57.29578f,
                                            b.omega[1] * 57.29578f,
                                            b.omega[2] * 57.29578f);

  estimated_height = -b.pos_d;
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

/// 把固件下发的舵机 PWM% 反解成物理舵角（与 mapSingleEdf4VaneOutputs 严格互逆）
static SingleEdfActuators decode_actuators(const float ch_pct[4], float throttle_pct)
{
  SingleEdfActuators act;
  constexpr float kZeroPct = 50.0f;   // kSingleEdf4VaneOutputs[0].zero_value
  constexpr float kSpanPct = 16.67f;  // kSingleEdf4VaneOutputs[0].span
  for (int i = 0; i < 4; ++i)
  {
    const float norm = (ch_pct[i] - kZeroPct) / kSpanPct;
    act.vane_deg[i] = norm * P.vane_max_deg;
  }
  act.throttle_pct = throttle_pct;
  return act;
}

int main(int argc, char **argv)
{
  const bool manual = (argc > 1 && std::strcmp(argv[1], "--manual") == 0);
  Body body;

  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  // 首次解锁门需要 EKF 收敛标志：SIL 用注入传感器直接构造飞行状态，
  // 语义上等价于"EKF 已收敛"（同 fw_sil_main.cpp 的 2026-11-07 修复）。
  g_ekf_converged = true;
  g_ekf_converged_s = 10.0f;
  is_origin_lla_set = true;
  origin_lat_deg = 28.2; origin_lon_deg = 112.9; origin_alt_m = 50.0;

  const float dt = 0.005f;
  const int steps = int(40.0f / dt);
  bool alt_cmd_sent = false;

  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt;
    if (manual)
    {
      const uint16_t roll = (t > 10.0f && t < 15.0f) ? 1600 : 1500;
      const uint16_t pitch = (t > 20.0f && t < 25.0f) ? 1600 : 1500;
      set_rc((t < 1.0f) ? 1000 : 1500, 1000, t >= 0.5f, roll, pitch);
    }
    else
    {
      set_rc((t < 1.0f) ? 1000 : 1500, (t < 2.0f) ? 1300 : 1500, t >= 0.5f);
    }
    feed_sensors(body);
    runGNCExecutive();

    if (!manual && !alt_cmd_sent && t >= 2.0f)
    {
      target_altitude = 10.0f; // 等价于一次定高指令
      alt_cmd_sent = true;
    }

    const float ch_pct[4] = {ch1_output, ch2_output, ch3_output, ch4_output};
    // throttlePercent 是固件在单涵道 mix 出口回填的最终油门（含联锁后的值），
    // 与下发到推进器的那一路同源（见 flight_control.cpp 单涵道分支末尾）。
    const SingleEdfActuators act = decode_actuators(ch_pct, throttlePercent);
    body.step(act, dt);

    if ((k % 200) == 0)
    {
      float r, p, y;
      body.euler(r, p, y);
      std::printf("t=%5.1f alt=%6.2f roll=%7.2f pitch=%7.2f yaw=%7.2f "
                  "vane=(%.1f,%.1f,%.1f,%.1f) thr=%.1f%% mode=%d unlock=%d\n",
                  t, -body.pos_d, r * 57.29578f, p * 57.29578f, y * 57.29578f,
                  ch_pct[0], ch_pct[1], ch_pct[2], ch_pct[3],
                  throttlePercent,
                  (int)g_current_flight_mode, (int)g_is_unlocked);
    }
  }
  return 0;
}
