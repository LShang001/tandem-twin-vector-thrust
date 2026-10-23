/**
 * @file    fw_sil_main.cpp
 * @module  固件/宿主 SIL（fw-tests：真实固件控制链的闭环入口）
 * @brief   链接真实 flight_control.cpp + state_data.cpp，跑“定高 10 m”或手动阶跃闭环
 * @details 被测：真实固件 `runGNCExecutive()` 及其控制链（本文件不复制控制律）。
 *          输入：`--manual` 时给 roll/pitch 阶跃，否则给 AUTO_ALTITUDE 10 m 目标；
 *          处理：本文件提供单刚体 plant、电机一阶滞后、传感器直注与执行器解算，
 *          固件代码照常跑 200 Hz；
 *          输出：每 0.2 s 打印高度/姿态/舵机/电机，运行结束打印汇总。
 *          运行（仓库根目录，需先设 BIN_DIR 避开中文路径）：
 *          BIN_DIR=<临时目录> bash test_host/fw_sil/build_and_run.sh [--manual]
 * @theory  闭环即真实固件的姿态环/定高环；plant 侧用
 *          `computeWrench()`（T = kT·ω²、Q = kQ·ω²+J_p·ω̇）。@ref CTL-001 §4
 * @frame   plant 的 NED 位置/速度 + 机体系角速率；固件侧沿用存档系→模型系置换
 * @units   高度 m，姿态 deg，舵机 PWM 百分比，电机转速 rad/s，时间 s
 * @note    与实机差异：无气动、无风、无传感器噪声与延迟，电机/舵机为理想一阶模型，
 *          是“控制律能否收敛”的最小闭环证据，**不是**飞行品质或鲁棒性证据。
 * @test    S1 AUTO_ALTITUDE 定高从 0 收敛到 10 m；S2 --manual 下 roll/pitch 阶跃响应收敛
 * @see     TandemVec_FCS/test_host/fw_sil/README.md、fw_sil_env_main.cpp（6DOF/传感器版）
 */
// ============================================================
// 仅替换 STM32/Arduino 外设、传感器、执行器反馈，控制链仍是固件原码。
// ============================================================
#include "state_data.h"
#include "flight_control.h"
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "Quat4f.h"
#include "QuaternionMath.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static const TandemVecParams &P = kDefaultTandemVecParams;
static const ServoConfig &SC = kDefaultServoConfig;

struct Motor {
  float w, target, tau;
  Motor(float w0 = 0.0f, float t = 0.28f) : w(w0), target(w0), tau(t) {}
  void set(float wt) { target = wt; }
  float step(float dt) { w += (target - w) * fminf(dt / tau, 1.0f); return w; }
};

struct Body {
  Quat4f q;
  float omega[3];
  float vel_n, vel_e, vel_d;
  float pos_n, pos_e, pos_d;

  Body() : q(1, 0, 0, 0), omega{0, 0, 0},
           vel_n(0), vel_e(0), vel_d(0),
           pos_n(0), pos_e(0), pos_d(0) {}

  void step(const SixDOFWrench &w, float dt) {
    const float p = omega[0], qq = omega[1], r = omega[2];
    // computeWrench 模型系 (x'=推力轴,y'=尾摆,z'=前摆) -> 机体 FRD:
    // M_body = (-Mz', My', -Mx')
    const float Mx_b = -w.Mz;
    const float My_b = w.My;
    const float Mz_b = -w.Mx;
    omega[0] += (Mx_b - (P.Iz - P.Iy) * qq * r) / P.Ix * dt;
    omega[1] += (My_b - (P.Ix - P.Iz) * r * p) / P.Iy * dt;
    omega[2] += (Mz_b - (P.Iy - P.Ix) * p * qq) / P.Iz * dt;

    const float wx = omega[0], wy = omega[1], wz = omega[2];
    Quat4f dq{-0.5f * (q.x * wx + q.y * wy + q.z * wz),
              0.5f * (q.w * wx + q.y * wz - q.z * wy),
              0.5f * (q.w * wy - q.x * wz + q.z * wx),
              0.5f * (q.w * wz + q.x * wy - q.y * wx)};
    q.w += dq.w * dt; q.x += dq.x * dt; q.y += dq.y * dt; q.z += dq.z * dt;
    q = qNorm(q);

    // 推力沿 -z_b；z_b 在 NED down 的投影 = R33。
    const float R33 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    const float acc_d = P.g - (w.Fx / P.m) * R33;
    vel_d += acc_d * dt;
    pos_d += vel_d * dt;
  }

  void euler(float &roll, float &pitch, float &yaw) const {
    Quaternion qq{q.w, q.x, q.y, q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
};

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500) {
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

static void feed_sensors(const Body &b) {
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

int main(int argc, char **argv) {
  const bool manual = (argc > 1 && std::strcmp(argv[1], "--manual") == 0);
  Body body;
  const float w_hover = P.wMax * 0.5f;
  Motor mf(w_hover), mt(w_hover);

  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  is_origin_lla_set = true;
  origin_lat_deg = 28.2; origin_lon_deg = 112.9; origin_alt_m = 50.0;

  const float dt = 0.005f;
  const int steps = int(40.0f / dt);
  bool alt_cmd_sent = false;

  for (int k = 0; k < steps; ++k) {
    const float t = k * dt;
    if (manual) {
      const uint16_t roll = (t > 10.0f && t < 15.0f) ? 1600 : 1500;
      const uint16_t pitch = (t > 20.0f && t < 25.0f) ? 1600 : 1500;
      set_rc((t < 1.0f) ? 1000 : 1500, 1000, t >= 0.5f, roll, pitch);
    } else {
      set_rc((t < 1.0f) ? 1000 : 1500, (t < 2.0f) ? 1300 : 1500, t >= 0.5f);
    }
    feed_sensors(body);
    runGNCExecutive();

    if (!manual && !alt_cmd_sent && t >= 2.0f) {
      target_altitude = 10.0f;   // 等价于一次定高指令
      alt_cmd_sent = true;
    }

    // ch1/ch2: 舵机百分比；ch3/ch4: 电机百分比。
    const float upper = (ch1_output - (50.0f + SC.zero_roll_pct)) *
                        SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_roll) *
                        0.017453292f;
    const float lower = (ch2_output - (50.0f + SC.zero_pitch_pct)) *
                        SC.half_travel_deg /
                        (50.0f * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_pitch) *
                        0.017453292f;

    mf.set((ch3_output / 100.0f) * P.wMax);
    mt.set((ch4_output / 100.0f) * P.wMax);
    PropulsionState ps{mf.step(dt), mt.step(dt), upper, lower};
    SixDOFWrench wr = computeWrench(ps, P);
    body.step(wr, dt);

    if ((k % 200) == 0) {
      float r, p, y;
      body.euler(r, p, y);
      std::printf("t=%5.1f alt=%6.2f roll=%7.2f pitch=%7.2f yaw=%7.2f "
                  "servo=(%.1f,%.1f) motor=(%.1f,%.1f) mode=%d unlock=%d\n",
                  t, -body.pos_d, r * 57.29578f, p * 57.29578f, y * 57.29578f,
                  ch1_output, ch2_output, ch3_output, ch4_output,
                  (int)g_current_flight_mode, (int)g_is_unlocked);
    }
  }
  return 0;
}
