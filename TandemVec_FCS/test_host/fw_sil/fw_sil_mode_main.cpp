/**
 * @file    fw_sil_mode_main.cpp
 * @module  固件/宿主 SIL 场景（模式重入与速率环积分）
 * @brief   编译真实 `src/flight_control.cpp` + `src/state_data.cpp`，用静态传感器
 *          验证两件事：
 *          ① AUTO_POSITION 重入必须重新锁当前点（P1-1，旧实现自主飞回旧点）；
 *          ② RATE_MODE 的 PID 复位只在模式沿（P1-4，旧实现每拍清零 → 纯 P）。
 * @details 输入：无；处理：每拍设 RC → runGNCExecutive，plant 不解算（只看控制量）；
 *          输出：每个场景一行 PASS/FAIL + 关键量，main 返回非零即失败。
 *
 * 编译命令同 `fw_sil_failsafe_main.cpp`，把主文件换成本文件。
 *
 * @note 本文件直接调用固件 `runGNCExecutive()`，不复制控制律；被测对象是
 *       `generate_attitude_target` 的沿检测与 `execute_attitude_controller` 的复位口径。
 * @see  src/flight_control.cpp::generate_attitude_target / execute_attitude_controller
 */
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "TandemVec_Config.h"
#include "flight_control.h"
#include "state_data.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int g_fail = 0;

static void check(bool cond, const char *what)
{
  if (!cond)
  {
    std::printf("[FAIL] %s\n", what);
    ++g_fail;
  }
}

static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500,
                   uint16_t att_mode = 1000)
{
  for (int i = 0; i < 9; ++i) raw_rc_values[i] = 1500;
  raw_rc_values[0] = roll;
  raw_rc_values[1] = pitch;
  raw_rc_values[2] = throttle;
  raw_rc_values[3] = 1500;
  raw_rc_values[4] = unlock ? 1800 : 1000;
  // 点火开关关闭：否则 CH7 高档 + 点火会让 getControlMode 返回 GUIDED（本场景不需要）。
  raw_rc_values[5] = 1000;
  raw_rc_values[6] = mode;
  raw_rc_values[7] = 1000;
  raw_rc_values[8] = att_mode;
}

// 静态传感器：姿态水平、角速率 0、高度 10 m、水平位置由测试给定。
static void feed_static(float height_m, float north_m, float east_m)
{
  AHRS_Packet.Roll = 0.0f;
  AHRS_Packet.Pitch = 0.0f;
  AHRS_Packet.Heading = 0.0f;
  AHRS_Packet.Qw = 1.0f;
  AHRS_Packet.Qx = 0.0f;
  AHRS_Packet.Qy = 0.0f;
  AHRS_Packet.Qz = 0.0f;
  icm_gyro_x = 0.0f;
  icm_gyro_y = 0.0f;
  icm_gyro_z = 0.0f;
  current_omega_dps_body_filtered = Vector3(0.0f, 0.0f, 0.0f);
  estimated_height = height_m;
  estimated_velocity = 0.0f;
  relative_north = north_m;
  relative_east = east_m;
  relative_down = -height_m;
  fused_north_pos = 0.0f;
  fused_east_pos = 0.0f;
  fused_north_vel = 0.0f;
  fused_east_vel = 0.0f;
  INS_GNSS_Packet.location_north = north_m;
  INS_GNSS_Packet.location_east = east_m;
  INS_GNSS_Packet.location_down = -height_m;
  INS_GNSS_Packet.velocity_north = 0.0f;
  INS_GNSS_Packet.velocity_east = 0.0f;
  INS_GNSS_Packet.velocity_down = 0.0f;
}

static void tick(float height_m, float north_m, float east_m)
{
  feed_static(height_m, north_m, east_m);
  runGNCExecutive();
}

// 场景 A：定点 → 定高 → 漂移 3 m → 回定点，目标点必须重锁到当前点。
static void testPositionReentry()
{
  std::printf("-- A. AUTO_POSITION 重入重锁 --\n");
  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  is_origin_lla_set = true;
  origin_lat_deg = 28.2;
  origin_lon_deg = 112.9;
  origin_alt_m = 50.0;
  // 航向可观（导航层会把该标志清零）；默认值 true 是“未证明可观即降级”的安全向默认，
  // 本场景要验证定点模式本身，所以显式模拟导航层已放行。
  g_pos_hold_degraded = false;
  target_altitude = 10.0f;

  const float dt = 0.005f;
  float north = 0.0f;

  // 0.0–0.5 s 未解锁；0.5 s 起解锁（触发解锁上升沿，会清目标点与 PID）
  for (float t = 0.0f; t < 2.0f; t += dt)
  {
    set_rc(1500, 988, t >= 0.5f);
    tick(10.0f, north, 0.0f);
  }

  // 2.0–4.0 s 进入 AUTO_POSITION（旧点 = 0 m）
  for (float t = 0.0f; t < 2.0f; t += dt)
  {
    set_rc(1500, 2012, true);
    tick(10.0f, north, 0.0f);
  }
  const float target_first = targetNorth;

  // 4.0–6.0 s 切 AUTO_ALTITUDE（定点分支不再被调用）
  for (float t = 0.0f; t < 2.0f; t += dt)
  {
    set_rc(1500, 1500, true);
    tick(10.0f, north, 0.0f);
  }

  // 漂移 3 m（位置估计变化），仍在定高模式
  north = 3.0f;
  for (float t = 0.0f; t < 0.5f; t += dt)
  {
    set_rc(1500, 1500, true);
    tick(10.0f, north, 0.0f);
  }

  // 6.5–7.5 s 切回 AUTO_POSITION：必须把目标重锁到当前点 3 m
  for (float t = 0.0f; t < 1.0f; t += dt)
  {
    set_rc(1500, 2012, true);
    tick(10.0f, north, 0.0f);
  }

  std::printf("   首段目标=%.4f m 重入后目标=%.4f m 当前=%.4f m mode=%d\n",
              static_cast<double>(target_first), static_cast<double>(targetNorth),
              static_cast<double>(north), static_cast<int>(g_current_flight_mode));
  check(g_current_flight_mode == AUTO_POSITION, "场景 A 必须处于 AUTO_POSITION");
  check(std::fabs(target_first) < 1e-3f, "场景 A 首段目标应为 0 m");
  check(std::fabs(targetNorth - 3.0f) < 0.05f,
        "场景 A 重入后目标点必须重锁到当前点（3 m），不得沿用旧点 0 m");
}

// 场景 B：RATE_MODE 下速率环积分必须累积（旧实现每拍清零 → 恒 0）。
static void testRateIntegral()
{
  std::printf("-- B. RATE_MODE 速率环积分 --\n");
  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;
  is_origin_lla_set = true;
  target_altitude = 10.0f;

  const float dt = 0.005f;

  // 解锁 + 角度模式，给滚转杆制造持续误差，先累积一段积分
  for (float t = 0.0f; t < 1.0f; t += dt)
  {
    set_rc(1500, 1500, t >= 0.3f, 1600, 1500, 1000);
    tick(10.0f, 0.0f, 0.0f);
  }
  const float integral_attitude = rollRatePID.getIntegral();

  // 切 RATE_MODE（CH9 高位），保持滚转杆偏置，让内环误差持续存在
  for (float t = 0.0f; t < 0.2f; t += dt)
  {
    set_rc(1500, 1500, true, 1600, 1500, 2012);
    tick(10.0f, 0.0f, 0.0f);
  }
  const float integral_rate_short = rollRatePID.getIntegral();

  for (float t = 0.0f; t < 1.0f; t += dt)
  {
    set_rc(1500, 1500, true, 1600, 1500, 2012);
    tick(10.0f, 0.0f, 0.0f);
  }
  const float integral_rate_long = rollRatePID.getIntegral();

  std::printf("   角度段积分=%.6f 切 RATE 后 0.2s=%.6f 再 1s=%.6f mode=%d\n",
              static_cast<double>(integral_attitude),
              static_cast<double>(integral_rate_short),
              static_cast<double>(integral_rate_long),
              static_cast<int>(g_current_flight_mode));

  // 旧实现（每拍 reset）两个 RATE 采样都恒为 0。
  check(std::fabs(integral_rate_short) > 1e-6f || std::fabs(integral_rate_long) > 1e-6f,
        "RATE_MODE 下速率环积分必须累积（不得每拍清零）");
  check(std::fabs(integral_rate_long) > std::fabs(integral_rate_short),
        "RATE_MODE 持续偏差下积分应继续增长");
}

int main()
{
  std::printf("== 模式重入 / RATE 积分 SIL（真实 flight_control.cpp）==\n");
  testPositionReentry();
  testRateIntegral();

  if (g_fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  std::printf("失败 %d 项\n", g_fail);
  return 1;
}
