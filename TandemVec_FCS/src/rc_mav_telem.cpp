/**
 * @file rc_mav_telem.cpp
 * @module 飞控/通信（RC 侧 MAVLink 遥测下行）
 * @brief `rc_mav_telem.h` 的实现：按 ELRS 翻译白名单组帧，发往 RC 串口。
 *
 * @note 只发帧、不改全局状态；数据源全部只读（EKF / UBX / 电池 / RC 通道）。
 * @note 用 `lib/MAVLink` 官方编码器组帧，不改供应商树。
 */

#include "rc_mav_telem.h"

#include <Arduino.h>

#include "MAVLink.h"
#include "state_data.h"

namespace {

Stream* s_out = nullptr;
uint32_t s_frames = 0;
uint32_t s_last_ms[8] = {};   ///< 每条消息的上次发送时刻（按 kMsg* 索引）
uint32_t s_now_ms = 0;

/// 本机在 MAVLink 网络中的身份：与 mavlink_bridge / mavlink_rc 保持一致
constexpr uint8_t kSysId = 1;
constexpr uint8_t kCompId = MAV_COMP_ID_AUTOPILOT1;

/**
 * @brief 组帧并写出（串口写满时丢弃本帧，绝不阻塞控制循环）
 */
void emit(const mavlink_message_t& msg) {
  if (s_out == nullptr) {
    return;
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  if (len == 0 || s_out->availableForWrite() < static_cast<int>(len)) {
    return;  // 端口忙：本帧丢弃，下一拍再发（遥测允许丢帧，控制不允许阻塞）
  }
  s_out->write(buf, len);
  ++s_frames;
}

/// 节拍闸门：返回 true 表示这一拍该发这条消息
bool due(uint8_t slot, uint32_t period_ms) {
  if (s_last_ms[slot] != 0 && (s_now_ms - s_last_ms[slot]) < period_ms) {
    return false;
  }
  s_last_ms[slot] = s_now_ms;
  return true;
}

/**
 * @brief 本项目控制模式 → ArduPilot Copter 的 custom_mode
 *
 * ELRS 用 `ap_flight_mode_name4(mode, ap_vehicle_from_mavtype(type), custom_mode)`
 * 查表出模式名，所以必须填 Copter 模式号，不能填自定义枚举值。
 */
uint32_t modeToApCopter(ControlMode mode) {
  switch (mode) {
    case MANUAL:
      return 0;   // STABILIZE
    case AUTO_POSITION:
      return 5;   // LOITER
    case AUTO_ALTITUDE:
      return 2;   // ALT_HOLD
    case GUIDED:
      return 4;   // GUIDED
    default:
      return 0;
  }
}

/// 地速（m/s）：由 UBX 的 NED 速度水平分量合成
float groundSpeedMps() {
  const float vn = ubx.north_vel_mps();
  const float ve = ubx.east_vel_mps();
  return sqrtf(vn * vn + ve * ve);
}

/// 航向（cdeg，0..35999）：用 EKF 偏航角，负角回绕到 0..2π
uint16_t headingCdeg() {
  float yaw = nav_ekf.yaw_rad();
  while (yaw < 0.0f) {
    yaw += 6.283185307f;
  }
  while (yaw >= 6.283185307f) {
    yaw -= 6.283185307f;
  }
  return static_cast<uint16_t>(yaw * 57.29578f * 100.0f) % 36000U;
}

// ---- 白名单消息 1/6：HEARTBEAT（模式名 + 解锁标记 `*`），1 Hz ----
void sendHeartbeat() {
  uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  if (g_is_unlocked) {
    base |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(kSysId, kCompId, &m, MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_ARDUPILOTMEGA, base,
                             modeToApCopter(g_current_flight_mode),
                             g_is_unlocked ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
  emit(m);
}

// ---- 白名单消息 2/6：ATTITUDE（遥控器姿态页），20 Hz ----
void sendAttitude() {
  mavlink_message_t m;
  mavlink_msg_attitude_pack(kSysId, kCompId, &m, s_now_ms,
                            nav_ekf.roll_rad(), nav_ekf.pitch_rad(),
                            nav_ekf.yaw_rad(), 0.0f, 0.0f, 0.0f);
  emit(m);
}

// ---- 白名单消息 3/6：GPS_RAW_INT（坐标/星数/地速/航向），2 Hz ----
void sendGpsRaw() {
  // Ubx::Fix 的 1..6 与 MAVLink GPS_FIX_TYPE 数值一致（1=NO_FIX … 6=RTK_FIXED）
  const int8_t fix = ubx.fix();
  const uint32_t hacc_mm =
      static_cast<uint32_t>(constrain(g_ekf_pos_sigma_m * 1000.0f, 0.0f, 4.0e9f));
  mavlink_message_t m;
  mavlink_msg_gps_raw_int_pack(
      kSysId, kCompId, &m, static_cast<uint64_t>(s_now_ms) * 1000ULL,
      static_cast<uint8_t>(fix > 0 ? fix : 0),
      static_cast<int32_t>(ubx.lat_deg() * 1e7), static_cast<int32_t>(ubx.lon_deg() * 1e7),
      static_cast<int32_t>(ubx.alt_msl_m() * 1000.0f),
      static_cast<uint16_t>(constrain(g_ekf_pos_sigma_m * 100.0f, 0.0f, 65535.0f)),
      static_cast<uint16_t>(constrain(g_ekf_pos_sigma_m * 100.0f, 0.0f, 65535.0f)),
      static_cast<uint16_t>(constrain(groundSpeedMps() * 100.0f, 0.0f, 65535.0f)),
      headingCdeg(), static_cast<uint8_t>(constrain(ubx.num_sv(), 0, 255)),
      0 /*alt_ellipsoid*/, hacc_mm, hacc_mm, 0 /*vel_acc*/, 0 /*hdg_acc*/,
      0 /*yaw*/);
  emit(m);
}

// ---- 白名单消息 4/6：GLOBAL_POSITION_INT（相对高度 → VARIO），5 Hz ----
void sendGlobalPosition() {
  mavlink_message_t m;
  mavlink_msg_global_position_int_pack(
      kSysId, kCompId, &m, s_now_ms,
      static_cast<int32_t>(ubx.lat_deg() * 1e7), static_cast<int32_t>(ubx.lon_deg() * 1e7),
      static_cast<int32_t>((estimated_height + static_cast<float>(origin_alt_m)) * 1000.0f),
      static_cast<int32_t>(estimated_height * 1000.0f),
      static_cast<int16_t>(constrain(ubx.north_vel_mps() * 100.0f, -32768.0f, 32767.0f)),
      static_cast<int16_t>(constrain(ubx.east_vel_mps() * 100.0f, -32768.0f, 32767.0f)),
      static_cast<int16_t>(constrain(ubx.down_vel_mps() * 100.0f, -32768.0f, 32767.0f)),
      headingCdeg());
  emit(m);
}

// ---- 白名单消息 5/6：BATTERY_STATUS（id 必须为 0 才会被翻译），1 Hz ----
void sendBattery() {
  uint16_t volts[10] = {};
  volts[0] = static_cast<uint16_t>(constrain(bat_voltage_mv, 0.0f, 65535.0f));
  mavlink_message_t m;
  mavlink_msg_battery_status_pack(
      kSysId, kCompId, &m, 0 /*id: 翻译器只认 0*/,
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX,
      volts, -1 /*电流无传感器*/, -1 /*已消耗能量*/, -1 /*已消耗容量*/,
      -1 /*剩余百分比*/, 0 /*剩余时间*/, 0 /*充电状态*/,
      nullptr /*扩展单体电压*/, 0 /*工作模式*/, 0 /*故障位*/);
  emit(m);
}

// ---- 白名单消息 6/6：VFR_HUD（油门/地速/升降率），5 Hz ----
void sendVfrHud() {
  // 油门百分比由 CH3（油门通道）换算：988..2012 µs → 0..100 %
  const uint16_t thr_us = raw_rc_values[2];
  float thr_pct = 0.0f;
  if (thr_us > 988U) {
    thr_pct = (static_cast<float>(thr_us) - 988.0f) * 100.0f / 1024.0f;
  }
  thr_pct = constrain(thr_pct, 0.0f, 100.0f);

  const float gs = groundSpeedMps();
  mavlink_message_t m;
  mavlink_msg_vfr_hud_pack(
      kSysId, kCompId, &m, gs /*空速暂用地速*/, gs, static_cast<int16_t>(headingCdeg() / 100U),
      static_cast<uint16_t>(thr_pct), static_cast<float>(estimated_height),
      -ubx.down_vel_mps() /*MAVLink 向上为正*/);
  emit(m);
}

}  // namespace

void rcMavTelemBegin(Stream& out) {
  s_out = &out;
  rcMavTelemReset();
}

void rcMavTelemReset() {
  for (uint32_t& v : s_last_ms) {
    v = 0;
  }
  s_frames = 0;
}

void rcMavTelemLoop(uint32_t now_ms) {
  if (s_out == nullptr) {
    return;
  }
  s_now_ms = now_ms;
  // 节拍表：合计约 1.3 kB/s（白名单消息的包长：HEARTBEAT 21B / ATTITUDE 40B /
  // GPS_RAW_INT 52B / GLOBAL_POSITION_INT 40B / BATTERY_STATUS 48B / VFR_HUD 28B）
  if (due(0, 1000)) { sendHeartbeat(); }        // 21 B/s
  if (due(1, 50))   { sendAttitude(); }         // 40 B × 20 Hz = 800 B/s
  if (due(2, 500))  { sendGpsRaw(); }           // 52 B × 2 Hz  = 104 B/s
  if (due(3, 200))  { sendGlobalPosition(); }   // 40 B × 5 Hz  = 200 B/s
  if (due(4, 1000)) { sendBattery(); }          // 48 B/s
  if (due(5, 200))  { sendVfrHud(); }           // 28 B × 5 Hz  = 140 B/s
}

uint32_t rcMavTelemFrames() { return s_frames; }

bool rcMavTelemReady() { return s_out != nullptr; }