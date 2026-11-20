/**
 * @file    fs_telemetry_main.cpp
 * @module  固件/全链路数字孪生（fs：遥测帧字节级验证）
 * @brief   让真实 `communication.cpp` 构造机载链路帧，导出字节并按协议解析回原值
 * @details 被测：`src/communication.cpp` 的 `sendPositionVelocityData()`（18 字段 /
 *          74 字节帧，`UPLINK_18FLOAT=1` 默认），**一行不改**地链接进来。
 *          输入：直接赋值 `AHRS_Packet`、`IMU_Packet`、`relative_north/east`、
 *                `estimated_height/velocity` 等固件全局量；
 *          处理：调用真实组帧函数，从 `ONBOARD_LINK_UART` 捕获写出的字节；
 *          输出：帧头/帧尾/长度/逐字段值，与注入值比对。
 * @theory  N/A（帧契约验证）
 * @ref     src/communication.cpp:1940 sendPositionVelocityData、
 *          docs（机载链路 0xAA…0x55 小端 float 帧）
 * @frame   机载链路机体系/相对坐标系字段见函数内注释（[3] p_n、[5] p_u 等）
 * @units   位姿字段单位见字段注释（m / m/s / rad / rad/s / m/s²）
 * @note    ★这条链只能证明"固件写出的字节符合它自己声明的布局"，不能证明
 *          上位机按同一布局解析——后者要在 GCS 侧另做一次解析对拍。
 *          字节序是小端（memcpy 直拷 STM32 内存），不是网络序。
 * @test    本文件即测试：退出码 0 = 通过。
 */

#include "Arduino.h"
#include "state_data.h"
#include "communication.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}

/// 按固件声明的布局取第 i 个 float（小端）。
float fieldAt(const uint8_t *buf, int i)
{
  float v = 0.0f;
  std::memcpy(&v, &buf[1 + i * 4], sizeof(float));
  return v;
}
}  // namespace

int main()
{
  std::printf("== 真实 communication.cpp 遥测帧字节验证 ==\n");

  // ---------- 注入已知状态 ----------
  g_sil_time_us = 12'345'000U;  // millis() = 12345 s
  AHRS_Packet.Roll = 0.25f;
  AHRS_Packet.Pitch = -0.5f;
  AHRS_Packet.Heading = 1.5f;
  IMU_Packet.gyroscope_x = 0.01f;
  IMU_Packet.gyroscope_y = -0.02f;
  IMU_Packet.gyroscope_z = 0.03f;
  relative_north = 12.5f;
  relative_east = -3.25f;
  estimated_height = 7.75f;
  estimated_velocity = 0.5f;
  INS_GNSS_Packet.velocity_north = 1.25f;
  INS_GNSS_Packet.velocity_east = -1.5f;
  g_onboard_link_proto = 0;  // 走固件自有帧，不走 MAVLink
  g_nav_cmd_accel_valid = true;
  g_nav_cmd_accel_up_mps2 = 0.11f;
  g_nav_cmd_accel_e_mps2 = 0.22f;
  g_nav_cmd_accel_n_mps2 = 0.33f;

  // ---------- 调用真实组帧函数 ----------
  Serial5.silClearTx();
  Serial8.silClearTx();
  sendPositionVelocityData();

  const uint8_t *src = nullptr;
  size_t len = 0;
  const char *which = "";
  if (Serial8.silTxLen() > 0) { src = Serial8.silTxBuf(); len = Serial8.silTxLen(); which = "Serial8"; }
  else if (Serial5.silTxLen() > 0) { src = Serial5.silTxBuf(); len = Serial5.silTxLen(); which = "Serial5"; }

  std::printf("  帧从 %s 写出，长度 %u 字节\n", which, static_cast<unsigned>(len));
  check(src != nullptr, "组帧函数确实写出了字节");

  if (src == nullptr)
  {
    std::printf("\n=== 存在失败 ===（失败 %d 项）\n", g_fail);
    return 1;
  }

  // ---------- 帧结构 ----------
  check(len == 74, "帧长 74 字节（1 + 18×4 + 1）");
  check(src[0] == 0xAA, "帧头 0xAA");
  if (len == 74) check(src[73] == 0x55, "帧尾 0x55");

  // ---------- 逐字段回读 ----------
  struct { int idx; float expect; const char *name; } cases[] = {
      {2, 12345.0f, "时间戳 = millis()/1000"},
      {3, 12.5f, "p_n = relative_north"},
      {4, -3.25f, "p_e = relative_east"},
      {5, 7.75f, "p_u = estimated_height"},
      {6, 1.25f, "v_n = INS_GNSS velocity_north"},
      {7, -1.5f, "v_e = INS_GNSS velocity_east"},
      {8, 0.5f, "v_u = estimated_velocity"},
      {9, 0.25f, "roll = AHRS_Packet.Roll"},
      {10, -0.5f, "pitch = AHRS_Packet.Pitch"},
      {11, 1.5f, "heading = AHRS_Packet.Heading"},
      {12, 0.01f, "gx = IMU gyroscope_x"},
      {13, -0.02f, "gy = IMU gyroscope_y"},
      {14, 0.03f, "gz = IMU gyroscope_z"},
      {15, 0.11f, "acc_u"},
      {16, 0.22f, "acc_e"},
      {17, 0.33f, "acc_n"},
  };
  for (const auto &c : cases)
  {
    const float got = fieldAt(src, c.idx);
    const bool ok = std::fabs(got - c.expect) < 1e-6f;
    std::printf("  [%2d] %-28s 期望 %10.4f  实测 %10.4f  %s\n", c.idx, c.name,
                static_cast<double>(c.expect), static_cast<double>(got), ok ? "OK" : "MISMATCH");
    check(ok, c.name);
  }

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}