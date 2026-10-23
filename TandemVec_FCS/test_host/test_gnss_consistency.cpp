/**
 * @file    test_gnss_consistency.cpp
 * @module  固件/宿主回归测试（fw-tests：GNSS 慢漂移/欺骗一致性）
 * @brief   验证加速度一致性、位置自洽、光流/气压交叉的 Ok/Suspect/锁存与 reset
 * @details 被测：`include/ins_gnss_consistency.h`（平台无关）。
 *          输入：合成 GNSS/IMU 速度增量/位置/光流/气压序列；
 *          处理：按 10 Hz epoch 调用 `update()`；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/eigen/src
 *                test_host/test_gnss_consistency.cpp -o bin/gnssc
 * @theory  1 s 窗口比力积分与 GNSS 速度增量比较；位置自洽/光流/气压做交叉；
 *          Suspect 锁存 5 s @ref REF-121 §7.3.1 p.255–259、
 *          REF-120 §7.4 (7.4.6) p.189
 * @frame   NED；flow_ned 由调用方从机体系旋转后传入
 * @units   m/s、m/s²、m、s
 * @note    0.02 m/s 级别慢漂移/恒定位置偏置+一致速度不在可检测范围；测试中
 *          明确写成“不承诺检测”，不是通过。
 * @test    C1 正常；C2 0.5 m/s² ramp；C3 20 m 位置跳变；C4 光流反证；
 *          C5 气压反证；C6 0.02 m/s 漂移不承诺；C7 reset
 * @see     TandemVec_FCS/include/ins_gnss_consistency.h、src/navigation_task.cpp
 */
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>

#include "ins_gnss_consistency.h"

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

static bfs::GnssConsistencyInput makeInput(float dt, const Eigen::Vector3f &vel,
                                           const Eigen::Vector3f &imu_dv,
                                           const Eigen::Vector3f &pos,
                                           bool flow_valid,
                                           const Eigen::Vector3f &flow_vel,
                                           bool baro_valid, float gnss_alt,
                                           float baro_alt)
{
  bfs::GnssConsistencyInput in;
  in.dt_s = dt;
  in.gnss_vel_ned = vel;
  in.imu_dv_ned = imu_dv;
  in.pos_valid = true;
  in.gnss_pos_ned = pos;
  in.flow_valid = flow_valid;
  in.flow_vel_ned = flow_vel;
  in.baro_valid = baro_valid;
  in.gnss_alt_up_m = gnss_alt;
  in.baro_alt_up_m = baro_alt;
  return in;
}

int main()
{
  const float dt = 0.1f;

  // C1 正常：IMU 速度增量与 GNSS 完全一致，位置/光流/气压一致。
  {
    bfs::InsGnssConsistencyMonitor mon;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    bfs::GnssConsistencyStatus s;
    for (int i = 0; i < 80; ++i)
    {
      const Eigen::Vector3f dv(0.02f, -0.01f, 0.0f);
      const Eigen::Vector3f old_vel = vel;
      vel += dv;
      pos += 0.5f * (old_vel + vel) * dt;
      s = mon.update(makeInput(dt, vel, dv, pos, true, vel, true, 10.0f,
                               10.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Ok, "C1 正常 Ok");
    CHECK(s.suspect_count == 0U, "C1 无 Suspect");
  }

  // C2 2.0 m/s² gross ramp：加速度残差单独触发 Suspect。
  {
    bfs::InsGnssConsistencyMonitor mon;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    bfs::GnssConsistencyStatus s;
    for (int i = 0; i < 25; ++i)
    {
      const Eigen::Vector3f dv(0.20f, 0.0f, 0.0f);
      const Eigen::Vector3f old_vel = vel;
      vel += dv;
      pos += 0.5f * (old_vel + vel) * dt;
      s = mon.update(makeInput(dt, vel, Eigen::Vector3f::Zero(), pos, true,
                               vel, true, 10.0f, 10.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Suspect, "C2 gross ramp Suspect");
    CHECK(s.suspect_count >= 1U, "C2 Suspect 计数");
    CHECK(s.accel_res_mps2 > 1.5f, "C2 加速度残差>Suspect 门限");
  }

  // C2b 0.5 m/s² 慢 ramp：单靠加速度不判，光流反证在 ~3 s 后触发 Suspect。
  {
    bfs::InsGnssConsistencyMonitor mon;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    bfs::GnssConsistencyStatus s;
    for (int i = 0; i < 45; ++i)
    {
      const Eigen::Vector3f dv(0.05f, 0.0f, 0.0f);
      const Eigen::Vector3f old_vel = vel;
      vel += dv;
      pos += 0.5f * (old_vel + vel) * dt;
      s = mon.update(makeInput(dt, vel, Eigen::Vector3f::Zero(), pos, true,
                               Eigen::Vector3f::Zero(), true, 10.0f, 10.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Suspect, "C2b ramp+flow Suspect");
    CHECK(s.flow_res_mps > 1.2f, "C2b 光流交叉残差>门限");
  }

  // C3 20 m 位置跳变：单帧位置异常，加速度不变。
  {
    bfs::InsGnssConsistencyMonitor mon;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    bfs::GnssConsistencyStatus s;
    for (int i = 0; i < 22; ++i)
    {
      if (i == 12)
      {
        pos.x() += 20.0f;
      }
      s = mon.update(makeInput(dt, vel, Eigen::Vector3f::Zero(), pos, false,
                               Eigen::Vector3f::Zero(), false, 0.0f, 0.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Suspect, "C3 位置跳变 Suspect");
    CHECK(s.pos_res_m > 5.0f, "C3 位置残差>门限");
  }

  // C4 光流反证：GNSS/IMU 一致 1.5 m/s，光流为 0。
  {
    bfs::InsGnssConsistencyMonitor mon;
    bfs::GnssConsistencyStatus s;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f(1.5f, 0.0f, 0.0f);
    for (int i = 0; i < 15; ++i)
    {
      pos += vel * dt;
      s = mon.update(makeInput(dt, vel, i == 0 ? vel : Eigen::Vector3f::Zero(),
                               pos, true, Eigen::Vector3f::Zero(), false, 0.0f,
                               0.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Suspect, "C4 光流反证 Suspect");
    CHECK(s.flow_res_mps > 1.2f, "C4 光流差>门限");
  }

  // C5 气压反证：GNSS 高度 5 m 阶跃，气压保持。
  {
    bfs::InsGnssConsistencyMonitor mon;
    bfs::GnssConsistencyStatus s;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    for (int i = 0; i < 15; ++i)
    {
      const float alt = (i >= 10) ? 5.0f : 0.0f;
      s = mon.update(makeInput(dt, Eigen::Vector3f::Zero(),
                               Eigen::Vector3f::Zero(), pos, false,
                               Eigen::Vector3f::Zero(), true, alt, 0.0f));
    }
    CHECK(s.state == bfs::GnssConsistencyState::Suspect, "C5 气压反证 Suspect");
    CHECK(s.baro_span_m > 4.0f, "C5 气压跨度>门限");
  }

  // C6 0.02 m/s 慢漂：加速度为 0、位置自洽，本层不承诺检测。
  {
    bfs::InsGnssConsistencyMonitor mon;
    bfs::GnssConsistencyStatus s;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    const Eigen::Vector3f vel(0.02f, 0.0f, 0.0f);
    for (int i = 0; i < 100; ++i)
    {
      pos += vel * dt;
      s = mon.update(makeInput(dt, vel, i == 0 ? vel : Eigen::Vector3f::Zero(),
                               pos, false, Eigen::Vector3f::Zero(), false, 0.0f,
                               0.0f));
    }
    CHECK(!mon.suspect(), "C6 0.02m/s 慢漂不被本层误报（不承诺检测）");
  }

  // C7 reset：Suspect 后清空锁存与窗口。
  {
    bfs::InsGnssConsistencyMonitor mon;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    for (int i = 0; i < 15; ++i)
    {
      vel.x() += 0.05f;
      pos += vel * dt;
      mon.update(makeInput(dt, vel, Eigen::Vector3f::Zero(), pos, false,
                           Eigen::Vector3f::Zero(), false, 0.0f, 0.0f));
    }
    bfs::GnssConsistencyInput reset_in = makeInput(
        dt, vel, Eigen::Vector3f::Zero(), pos, false, Eigen::Vector3f::Zero(),
        false, 0.0f, 0.0f);
    reset_in.reset = true;
    const auto s = mon.update(reset_in);
    CHECK(s.state == bfs::GnssConsistencyState::Ok, "C7 reset 后 Ok");
    CHECK(mon.status().suspect_hold_s == 0.0f, "C7 reset 清锁存");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_gnss_consistency: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_gnss_consistency: %d 项失败 ===\n", g_fail);
  return 1;
}
