/**
 * @file    test_ubx_epoch_queue_clear.cpp
 * @module  固件/宿主回归测试（fw-tests：UBX epoch 队列清理）
 * @brief   验证 Pump() 入队后 ClearEpochQueue() 清空、PopEpoch() 不再返回过期历元
 * @details 被测：`lib/ublox-main/src/ubx.h` 的 Pump/PopEpoch/ClearEpochQueue。
 *          输入：合成的 UBX-NAV-PVT(92B) + NAV-EOE(4B)，iTOW 相同；
 *          处理：喂入 fake HardwareSerial → Pump → 检查队列 → 清理 → 再 Pop；
 *          输出：PASS/FAIL，main 返回非零即失败。
 * @theory  UBX epoch = 一对 iTOW 相同的 NAV-PVT + NAV-EOE；队列只保存完整历元。
 * @frame   N/A
 * @units   N/A
 * @note    该回归对应 2026-11-03 实机上电 queue_overflow=16：DETA100 检测/
 *          GNSS 配置期间 Pump 已入队，但导航任务尚未启动，初始化后必须清空。
 * @see     src/main.cpp（ubxAutoConfig 后 ClearEpochQueue）、ubx.cpp
 */
#include <cstdio>
#include <cstring>
#include <vector>

#include "Arduino.h"
#include "ubx.h"

uint32_t g_sil_time_us = 0;

static void append_u8(std::vector<uint8_t> &v, uint8_t b) { v.push_back(b); }
static void append_u16(std::vector<uint8_t> &v, uint16_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}

static void append_frame(std::vector<uint8_t> &v, uint8_t cls, uint8_t id,
                         const std::vector<uint8_t> &payload)
{
  const size_t base = v.size();
  v.push_back(0xB5);
  v.push_back(0x62);
  v.push_back(cls);
  v.push_back(id);
  append_u16(v, static_cast<uint16_t>(payload.size()));
  v.insert(v.end(), payload.begin(), payload.end());
  uint8_t a = 0, b = 0;
  for (size_t i = base + 2; i < base + 6 + payload.size(); ++i)
  {
    a = static_cast<uint8_t>(a + v[i]);
    b = static_cast<uint8_t>(b + a);
  }
  v.push_back(a);
  v.push_back(b);
}

static std::vector<uint8_t> make_epoch(uint32_t itow)
{
  std::vector<uint8_t> pvt(92, 0);
  pvt[0] = static_cast<uint8_t>(itow & 0xFF);
  pvt[1] = static_cast<uint8_t>((itow >> 8) & 0xFF);
  pvt[2] = static_cast<uint8_t>((itow >> 16) & 0xFF);
  pvt[3] = static_cast<uint8_t>((itow >> 24) & 0xFF);

  std::vector<uint8_t> eoe(4, 0);
  eoe[0] = pvt[0];
  eoe[1] = pvt[1];
  eoe[2] = pvt[2];
  eoe[3] = pvt[3];

  std::vector<uint8_t> frames;
  append_frame(frames, 0x01, 0x07, pvt); // NAV-PVT
  append_frame(frames, 0x01, 0x61, eoe); // NAV-EOE
  return frames;
}

static int g_fail = 0;
static void check(bool ok, const char *what)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++g_fail;
}

int main()
{
  HardwareSerial serial;
  bfs::Ubx ubx(&serial);

  // 1) 一个完整历元入队
  {
    const std::vector<uint8_t> f = make_epoch(1000);
    serial.feed(f.data(), f.size());
    g_sil_time_us = 1000;
    (void)ubx.Pump(4096, g_sil_time_us);
    check(ubx.pending_epoch_count() == 1, "完整历元入队后 pending=1");
    check(ubx.nav_pvt_count() == 1 && ubx.nav_eoe_count() == 1,
          "PVT/EOE 各计数一次");
  }

  // 2) 清空后队列为空、Pop 失败；统计计数保留
  {
    ubx.ClearEpochQueue();
    check(ubx.pending_epoch_count() == 0, "ClearEpochQueue 后 pending=0");
    bfs::UbxEpoch epoch{};
    check(!ubx.PopEpoch(&epoch), "清空后 PopEpoch 返回 false");
    check(ubx.nav_pvt_count() == 1, "清队列不抹掉 PVT 统计");
  }

  // 3) 清理后再来一个历元仍能正常入队/出队
  {
    const std::vector<uint8_t> f = make_epoch(1100);
    serial.feed(f.data(), f.size());
    g_sil_time_us = 2000;
    (void)ubx.Pump(4096, g_sil_time_us);
    check(ubx.pending_epoch_count() == 1, "清理后新历元可入队");
    bfs::UbxEpoch epoch{};
    check(ubx.PopEpoch(&epoch), "清理后 PopEpoch 返回 true");
    check(ubx.pending_epoch_count() == 0, "Pop 后队列为空");
  }

  std::printf(g_fail == 0 ? "=== 全部通过 ===\n" : "=== 失败 %d ===\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
