// nmea_host_test.cpp — 库级双协议宿主机回归测试
// 验证 lib/ublox-main 的 UBX+NMEA 双协议（GpsProtocol）：
//   - kNmea 模式：GGA/RMC 句子切分、minmea 解析、快照→UbxEpoch 换算、
//     fix 映射（quality→Fix 枚举）、伪 tow、去重、坏校验和
//   - kUbx 模式：UBX 帧照常解析入队
//   - kAuto 模式：UBX 优先（backoff 窗口内不合成 NMEA），UBX 失效后 NMEA 兜底
//   - SetProtocol/SwitchProtocol 独立切换
// 编译（Windows Git Bash / Linux）：
//   gcc  -c -I. -I../../lib/ublox-main/src ../../lib/ublox-main/src/minmea.c -o minmea.o
//   g++  -std=gnu++17 -I. -I../../lib/ublox-main/src nmea_host_test.cpp \
//         ../../lib/ublox-main/src/ubx.cpp minmea.o -o nmea_test
#include "ubx.h"

#include <cmath>
#include <cstdio>
#include <cstring>

uint32_t g_sim_millis = 0;
uint32_t g_sim_micros = 0;

// Windows 无 timegm（minmea_gettime 引用，测试未调用但符号需解析）
#if defined(_WIN32)
#include <time.h>
extern "C" time_t timegm(struct tm *tm) { return _mkgmtime(tm); }
#endif

static int g_fail = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (cond) {                                                                \
      printf("  PASS %s\n", msg);                                              \
    } else {                                                                   \
      printf("  FAIL %s (line %d)\n", msg, __LINE__);                          \
      g_fail++;                                                                \
    }                                                                          \
  } while (0)

static void advance_ms(uint32_t ms)
{
  g_sim_millis += ms;
  g_sim_micros += static_cast<uint32_t>(ms) * 1000U;
}

// ---- UBX 帧构造（Pump 字节级验证用）：B5 62 cls id lenL lenH payload CK_A CK_B ----
static void makeUbxFrame(uint8_t cls, uint8_t id, const uint8_t *payload,
                         uint16_t len, uint8_t *out)
{
  size_t pos = 0;
  out[pos++] = 0xB5;
  out[pos++] = 0x62;
  out[pos++] = cls;
  out[pos++] = id;
  out[pos++] = static_cast<uint8_t>(len & 0xFF);
  out[pos++] = static_cast<uint8_t>(len >> 8);
  uint8_t ck_a = 0, ck_b = 0;
  // 校验范围含 cls/id/len
  for (int i = 2; i < 6; i++)
  {
    ck_a += out[i];
    ck_b += ck_a;
  }
  for (uint16_t i = 0; i < len; i++)
  {
    const uint8_t b = payload[i];
    out[pos++] = b;
    ck_a += b;
    ck_b += ck_a;
  }
  out[pos++] = ck_a;
  out[pos++] = ck_b;
}

// 最小 NAV-PVT（92B，iTOW=100000，3D fix，8 星）+ NAV-EOE 配对
static void makeUbxEpochFrames(uint8_t *out, size_t out_cap, size_t *len)
{
  uint8_t pvt[92] = {0};
  pvt[0] = 0xA0;  // iTOW=100000ms 小端（0x000186A0 → A0 86 01 00）
  pvt[1] = 0x86;
  pvt[2] = 0x01;
  pvt[3] = 0x00;
  pvt[20] = 3;    // fix_type = 3D
  pvt[21] = 0x01; // flags: gnssFixOK
  pvt[23] = 8;    // num_sv
  const int32_t lat = static_cast<int32_t>(31.5e7);   // 31.5° ×1e7
  const int32_t lon = static_cast<int32_t>(121.5e7);  // 121.5° ×1e7
  memcpy(pvt + 28, &lat, 4);
  memcpy(pvt + 24, &lon, 4);

  uint8_t eoe[4] = {0xA0, 0x86, 0x01, 0x00};  // iTOW 与 PVT 一致

  size_t pos = 0;
  makeUbxFrame(0x01, 0x07, pvt, 92, out + pos);
  pos += 6 + 92 + 2;
  makeUbxFrame(0x01, 0x61, eoe, 4, out + pos);
  pos += 6 + 4 + 2;
  *len = pos;
}

// ---- NMEA 句子：手动 XOR 校验和 ----
static void makeNmea(const char *body, char *out, size_t out_len)
{
  uint8_t ck = 0;
  for (const char *p = body; *p; p++)
  {
    ck ^= static_cast<uint8_t>(*p);
  }
  snprintf(out, out_len, "$%s*%02X", body, ck);
}

static void feedNmea(bfs::Ubx &ubx, HardwareSerial &serial, const char *sentence)
{
  serial.clear();
  serial.feed(sentence);
  serial.feed("\r\n");
  ubx.Pump(0, g_sim_micros);
}

int main()
{
  HardwareSerial serial;
  bfs::Ubx ubx;
  ubx.Config(&serial);
  char sent[160];

  // ================= 用例 1：kNmea 模式基础解析 =================
  // 真实 1Hz 流语义：GGA 触发合成（位置/高程），RMC 同批刷新速度字段，
  // 800ms 节流保证 GGA+RMC 双句只产生 1Hz epoch；合成时快照已合并两者。
  printf("== 用例1 kNmea：GGA+RMC → epoch 换算（节流合并）==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kNmea);
  ubx.Reset();
  makeNmea("GPGGA,092204.999,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // GGA → 合成 epoch#1（无速度历史）
  makeNmea("GPRMC,092205.000,A,4250.5589,S,14718.5084,E,10.5,89.68,200806,,,A", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // RMC 刷新速度 → 800ms 节流拒绝
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "kNmea：epoch#1 已入队");
    // 42°50.5589'S / 147°18.5084'E
    const double exp_lat = -42.84264833 * DEG_TO_RAD;
    const double exp_lon = 147.30847333 * DEG_TO_RAD;
    CHECK(fabs(ep.lat_rad - exp_lat) < 1e-6, "纬度换算（度→rad，minmea_tocoord）");
    CHECK(fabs(ep.lon_rad - exp_lon) < 1e-6, "经度换算");
    CHECK(fabs(ep.alt_wgs84_m - 43.4f) < 0.01f, "椭球高 = MSL 33.5 + geoid 9.9");
    CHECK(ep.num_sv == 8, "卫星数 8");
    CHECK(fabs(ep.horz_acc_m - 2.575f) < 0.05f, "h_acc = HDOP 1.03 × 2.5 = 2.575");
    CHECK(ep.spd_acc_mps == 3.0f, "spd_acc 保守上限 3.0（弱垂直约束）");
    CHECK(ep.fix == static_cast<int8_t>(bfs::Ubx::FIX_3D), "quality=1 → FIX_3D");
    CHECK(ep.pvt_tow_ms == 33724999U, "伪 tow = UTC 09:22:04.999 → 33724999ms（minmea 微秒精度）");
    CHECK(ep.pvt_tow_ms == ep.eoe_tow_ms, "pvt/eoe tow 一致");
    CHECK(ubx.PopEpoch(&ep) == false, "RMC 同批被 800ms 节流 → 无第二个 epoch");
  }
  // 节流窗口过后：RMC 已合并进快照 → 合成 epoch#2（速度来自 RMC）
  advance_ms(900);
  ubx.Pump(0, g_sim_micros);
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "节流过后合成 epoch#2");
    // 速度：RMC 10.5kt / 89.68° → vn≈0.03 ve≈5.40
    CHECK(fabs(ep.east_vel_mps - 5.40f) < 0.05f, "东向速度 ≈5.40m/s（RMC 合并）");
    CHECK(fabs(ep.north_vel_mps) < 0.1f, "北向速度 ≈0");
    CHECK(ep.pvt_tow_ms == 33725000U, "伪 tow = UTC 09:22:05.000 → 33725000ms");
  }

  // ================= 用例 2：fix 映射（quality→Fix） =================
  printf("== 用例2 fix 映射 ==\n");
  {
    struct
    {
      int quality;
      bfs::Ubx::Fix expect;
    } cases[] = {{1, bfs::Ubx::FIX_3D},
                 {2, bfs::Ubx::FIX_DGNSS},
                 {4, bfs::Ubx::FIX_RTK_FIXED},
                 {5, bfs::Ubx::FIX_RTK_FLOAT}};
    for (auto &c : cases)
    {
      char body[128];
      snprintf(body, sizeof(body),
               "GPGGA,092206.000,4250.5589,S,14718.5084,E,%d,08,1.03,33.5,M,9.9,M,,", c.quality);
      makeNmea(body, sent, sizeof(sent));
      feedNmea(ubx, serial, sent);
      char msg[64];
      snprintf(msg, sizeof(msg), "quality=%d → Fix(%d)", c.quality, static_cast<int>(c.expect));
      CHECK(ubx.nmea_fix() == static_cast<int8_t>(c.expect), msg);
    }
  }

  // ================= 用例 3：无效句保留快照 + 坏校验和计数 =================
  printf("== 用例3 无效句/坏校验和 ==\n");
  {
    makeNmea("GPGGA,092207.000,4250.5589,S,14718.5084,E,0,00,99.99,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_valid(), "quality=0 不覆盖有效快照");
    const uint32_t before = ubx.nmea_bad_checksum_count();
    serial.clear();
    serial.feed("$GPGGA,092208.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,*00\r\n");
    ubx.Pump(0, g_sim_micros);
    CHECK(ubx.nmea_bad_checksum_count() == before + 1, "坏校验和句 → bad_ck +1");
  }

  // ================= 用例 4：北纬西经（负 geoid 分离） =================
  printf("== 用例4 北纬西经 ==\n");
  advance_ms(1000);  // 越过上一合成的 800ms 节流窗口
  makeNmea("GPGGA,092208.000,3945.1234,N,10515.6789,W,1,08,1.03,100.0,M,-30.0,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // GGA 北纬西经 → 合成 epoch（位置）
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "北纬西经 epoch");
    const double exp_lat = 39.75205667 * DEG_TO_RAD;
    const double exp_lon = -105.26131500 * DEG_TO_RAD;
    CHECK(fabs(ep.lat_rad - exp_lat) < 1e-6, "北纬为正");
    CHECK(fabs(ep.lon_rad - exp_lon) < 1e-6, "西经为负");
    CHECK(fabs(ep.alt_wgs84_m - 70.0f) < 0.01f, "椭球高 = MSL 100 + geoid(-30) = 70m");
    CHECK(ubx.PopEpoch(&ep) == false, "无第二个 epoch（800ms 节流）");
  }
  makeNmea("GPRMC,092208.500,A,3945.1234,N,10515.6789,W,5.0,180.0,200806,,,A", sent, sizeof(sent));   // 与 GGA 同秒（092208）
  feedNmea(ubx, serial, sent);   // RMC 刷新速度 → 秒键去重拒绝（同秒）
  advance_ms(900);               // 无新句：不重复合成同一快照
  ubx.Pump(0, g_sim_micros);
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) == false, "无新句不重复合成（秒键去重）");
  }
  // 跨秒新句触发合成：位置=新 GGA，速度=同秒 RMC 已合并进快照（滞后 ≤1s）
  makeNmea("GPGGA,092209.000,3945.1234,N,10515.6789,W,1,08,1.03,100.0,M,-30.0,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "跨秒 epoch（RMC 速度合并）");
    CHECK(fabs(ep.north_vel_mps + 5.0f * 0.514444f) < 1e-3f, "航向180° → 纯南向");
    CHECK(fabs(ep.east_vel_mps) < 1e-3f, "航向180° → 东向≈0");
    const double exp_lat = 39.75205667 * DEG_TO_RAD;
    CHECK(fabs(ep.lat_rad - exp_lat) < 1e-6, "位置=跨秒新 GGA");
  }

  // ================= 用例 5：同秒多句去重 =================
  printf("== 用例5 同秒多句去重 ==\n");
  advance_ms(900);  // 越过节流窗口，隔离出本用例的合成时序
  {
    const uint32_t before = ubx.nmea_epoch_count();
    // 同秒（092210）连发 GGA+RMC：秒键去重只合成 1 次
    makeNmea("GPGGA,092210.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    makeNmea("GPRMC,092210.500,A,4250.5589,S,14718.5084,E,5.0,90.0,200806,,,A", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_epoch_count() == before + 1, "同秒去重：只合成 1 次");
  }

  // ================= 用例 6：kUbx 模式 UBX 帧解析 =================
  printf("== 用例6 kUbx：UBX 帧解析 ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kUbx);
  ubx.Reset();
  {
    uint8_t frames[256];
    size_t flen = 0;
    makeUbxEpochFrames(frames, sizeof(frames), &flen);
    serial.clear();
    serial.feed(frames, flen);
    const bool pushed = ubx.Pump(0, g_sim_micros);
    CHECK(pushed, "UBX PVT+EOE → epoch 入队（Pump 返回 true）");
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "PopEpoch 取到 UBX epoch");
    CHECK(ep.pvt_tow_ms == 100000U, "UBX iTOW=100000ms 保留");
    CHECK(ep.num_sv == 8, "UBX num_sv=8");
    CHECK(fabs(ep.lat_deg - 31.5) < 1e-4, "UBX lat=31.5°");
    CHECK(fabs(ep.lon_deg - 121.5) < 1e-4, "UBX lon=121.5°");
    // NMEA 句子在 kUbx 模式下应被忽略（UBX 状态机把它们当噪声丢弃）
    makeNmea("GPGGA,092210.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_sentence_count() == 0, "kUbx 模式不解析 NMEA 句");
  }

  // ================= 用例 7：kAuto 模式（UBX 优先 + 失效兜底） =================
  printf("== 用例7 kAuto：UBX 优先，失效后 NMEA 兜底 ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kAuto);
  ubx.Reset();
  // 7a：UBX epoch 到达（backoff 窗口内）→ NMEA 不合成
  {
    uint8_t frames[256];
    size_t flen = 0;
    makeUbxEpochFrames(frames, sizeof(frames), &flen);
    serial.clear();
    serial.feed(frames, flen);
    ubx.Pump(0, g_sim_micros);
    CHECK(ubx.nmea_epoch_count() == 0, "backoff 窗口内 UBX 存活 → NMEA 不合成");
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 100000U, "UBX epoch 正常消费");
  }
  // 7b：UBX 失效 300ms 后 → NMEA 兜底合成
  {
    makeNmea("GPGGA,092211.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);  // last_ubx_epoch_us 刚更新 → backoff 内，不合成
    CHECK(ubx.nmea_epoch_count() == 0, "UBX 刚入队 0ms → NMEA 不合成（backoff）");
    advance_ms(500);              // 推进 500ms > 300ms backoff
    ubx.Pump(0, g_sim_micros);
    CHECK(ubx.nmea_epoch_count() == 1, "UBX 失效 500ms → NMEA 兜底合成");
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "NMEA 兜底 epoch 可消费");
    CHECK(ep.pvt_tow_ms == 33731000U, "兜底 epoch 伪 tow = 09:22:11.000 → 33731000ms");
  }

  // ================= 用例 8：SwitchProtocol（含波特率） =================
  printf("== 用例8 SwitchProtocol ==\n");
  CHECK(ubx.SwitchProtocol(bfs::Ubx::GpsProtocol::kNmea, 38400), "切换 kNmea+38400");
  CHECK(serial.begin_baud() == 38400, "串口波特率已重设 38400");
  CHECK(ubx.SwitchProtocol(bfs::Ubx::GpsProtocol::kUbx, 921600), "切换 kUbx+921600");
  CHECK(serial.begin_baud() == 921600, "串口波特率已重设 921600");
  CHECK(ubx.SwitchProtocol(bfs::Ubx::GpsProtocol::kNmea, -1) == false, "非法波特率防护（-1）");
  CHECK(ubx.protocol() == bfs::Ubx::GpsProtocol::kNmea, "非法波特率仍完成协议切换");

  // ================= 用例 9：GSA → PDOP（组合解优先） =================
  printf("== 用例9 GSA→PDOP：GN 组合解优先 ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kNmea);
  ubx.Reset();
  makeNmea("GNGSA,A,3,02,05,09,12,15,18,23,27,,,,,1.8,1.0,1.5", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // GNGSA：组合解 PDOP=1.8
  CHECK(fabs(ubx.nmea_pdop() - 1.8f) < 0.01f, "GNGSA → PDOP 1.8");
  CHECK(fabs(ubx.nmea_vdop() - 1.5f) < 0.01f, "GNGSA → VDOP 1.5");
  makeNmea("GPGSA,A,3,02,05,09,12,,,,,,,,,1.5,0.9,1.2", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // GPGSA 后到：单星座不覆盖组合解
  CHECK(fabs(ubx.nmea_pdop() - 1.8f) < 0.01f, "GPGSA 后到不覆盖 GN 组合解 PDOP");
  // GSA PDOP 进入合成 epoch（GGA+RMC+GSA 完整流）
  makeNmea("GPGGA,092212.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep), "GGA 触发合成");
    CHECK(fabs(ep.pvt_pdop - 1.8f) < 0.01f, "epoch.pvt_pdop = GSA 组合 PDOP 1.8");
  }
  printf("== 用例10 GSA→PDOP：单星座兜底 ==\n");
  ubx.Reset();   // 清快照（含 PDOP）
  makeNmea("GPGSA,A,3,02,05,09,12,,,,,,,,,1.5,0.9,1.2", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // 只有单星座 GSA
  CHECK(fabs(ubx.nmea_pdop() - 1.5f) < 0.01f, "单星座 GSA 兜底 PDOP 1.5");

  // ================= 用例 11：句子统计计数 =================
  printf("== 用例11 句子统计 ==\n");
  ubx.Reset();
  makeNmea("GPGGA,092213.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  makeNmea("GPRMC,092213.000,A,4250.5589,S,14718.5084,E,5.0,90.0,200806,,,A", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  makeNmea("GNGSA,A,3,02,05,09,12,15,18,23,27,,,,,1.8,1.0,1.5", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  makeNmea("GPGLL,4250.5589,S,14718.5084,E,092213.000,A,A", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // GLL 未处理 → unknown
  CHECK(ubx.nmea_gga_count() == 1, "GGA 计数 1");
  CHECK(ubx.nmea_rmc_count() == 1, "RMC 计数 1");
  CHECK(ubx.nmea_gsa_count() == 1, "GSA 计数 1");
  CHECK(ubx.nmea_unknown_count() == 1, "GLL → unknown 计数 1");
  CHECK(ubx.nmea_sentence_count() == 4, "总句子计数 4");

  // ================= 用例 12：超长句溢出 + 恢复 =================
  printf("== 用例12 超长句溢出与恢复 ==\n");
  ubx.Reset();
  {
    const uint32_t before_overflow = ubx.nmea_overflow_count();
    char long_sent[160];
    memset(long_sent, 'A', sizeof(long_sent) - 8);  // 超长 body（>缓冲 96B）
    long_sent[sizeof(long_sent) - 8] = '\0';
    makeNmea(long_sent, sent, sizeof(sent));        // 生成超长句（含校验和）
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_overflow_count() == before_overflow + 1, "超长句 → overflow +1");
    // 溢出后正常句恢复解析
    makeNmea("GPGGA,092214.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_gga_count() == 1, "溢出后 GGA 正常解析");
  }

  // ================= 用例 13：句子分片跨 Pump =================
  printf("== 用例13 句子分片跨 Pump ==\n");
  ubx.Reset();
  makeNmea("GPGGA,092215.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  serial.clear();
  serial.feed(sent, 20);             // 前半段
  ubx.Pump(0, g_sim_micros);
  serial.feed(sent + 20, strlen(sent) - 20);  // 后半段
  serial.feed("\r\n");
  ubx.Pump(0, g_sim_micros);
  CHECK(ubx.nmea_gga_count() == 1, "分片 GGA 完整解析");
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 33735000U, "分片后 epoch 正常合成（09:22:15）");
  }

  // ================= 用例 14：串扰/乱码重同步 =================
  printf("== 用例14 串扰/乱码重同步 ==\n");
  ubx.Reset();
  serial.clear();
  serial.feed("$$$$GARBAGE\x01\x02\x03");          // 乱码 + 连续 $
  serial.feed("$GPGGA,092216.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,*XX\r\n");  // 坏校验
  makeNmea("GPGGA,092217.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  serial.feed(sent);
  serial.feed("\r\n");
  ubx.Pump(0, g_sim_micros);
  CHECK(ubx.nmea_gga_count() == 1, "乱码/坏校验后 GGA 正常解析（重同步）");
  CHECK(ubx.nmea_bad_checksum_count() == 1, "坏校验句计数 1");
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 33737000U, "重同步后 epoch（09:22:17）");
  }

  // ================= 用例 15：UBX 帧分片跨 Pump =================
  printf("== 用例15 UBX 帧分片跨 Pump ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kUbx);
  ubx.Reset();
  {
    uint8_t frames[256];
    size_t flen = 0;
    makeUbxEpochFrames(frames, sizeof(frames), &flen);
    serial.clear();
    serial.feed(frames, flen / 2);   // 前一半（跨 PVT/EOE 边界）
    ubx.Pump(0, g_sim_micros);
    serial.feed(frames + flen / 2, flen - flen / 2);
    ubx.Pump(0, g_sim_micros);
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 100000U, "UBX 帧分片跨 Pump 正常入队");
  }

  // ================= 用例 16：kAuto 混合流（NMEA 先到 → UBX 接入 → NMEA 兜底） =================
  printf("== 用例16 kAuto 混合流 ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kAuto);
  ubx.Reset();
  // 16a：从未见过 UBX → NMEA 直接可用（backoff 闸门不挡首段）
  makeNmea("GPGGA,092218.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);
  CHECK(ubx.nmea_epoch_count() == 1, "从未见 UBX → NMEA 直接合成");
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 33738000U, "16a NMEA epoch（09:22:18）");
  }
  // 16b：UBX epoch 接入 → backoff 窗口内 NMEA 不合成
  {
    uint8_t frames[256];
    size_t flen = 0;
    makeUbxEpochFrames(frames, sizeof(frames), &flen);
    serial.clear();
    serial.feed(frames, flen);
    ubx.Pump(0, g_sim_micros);
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 100000U, "UBX epoch 正常入队");
    makeNmea("GPGGA,092219.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
    feedNmea(ubx, serial, sent);
    CHECK(ubx.nmea_epoch_count() == 1, "UBX 存活窗口内 NMEA 不合成");
  }
  // 16c：UBX 失效（>300ms backoff）且节流过（>800ms）后 NMEA 兜底
  advance_ms(900);
  ubx.Pump(0, g_sim_micros);
  CHECK(ubx.nmea_epoch_count() == 2, "UBX 失效后 NMEA 兜底合成");

  // ================= 用例 17：跨日 tow 回绕 =================
  printf("== 用例17 跨日 tow 回绕 ==\n");
  ubx.SetProtocol(bfs::Ubx::GpsProtocol::kNmea);
  ubx.Reset();
  makeNmea("GPGGA,235959.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // tow=86399000
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 86399000U, "23:59:59 → tow 86399000");
  }
  advance_ms(1100);              // 跨日（越过节流）
  makeNmea("GPGGA,000000.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // tow=0
  {
    bfs::UbxEpoch ep;
    CHECK(ubx.PopEpoch(&ep) && ep.pvt_tow_ms == 0U, "00:00:00 → tow 0（跨日回绕不误判去重）");
  }

  // ================= 用例 18：RMC 速度/航向缺省 =================
  printf("== 用例18 RMC 缺省字段 ==\n");
  ubx.Reset();
  makeNmea("GPGGA,092220.000,4250.5589,S,14718.5084,E,1,08,1.03,33.5,M,9.9,M,,", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // 先有位置（速度=0）
  makeNmea("GPRMC,092220.000,A,4250.5589,S,14718.5084,E,,,200806,,,A", sent, sizeof(sent));  // speed/course 空
  feedNmea(ubx, serial, sent);   // 缺省速度不刷新（NaN 防护）
  CHECK(ubx.nmea_vel_n_mps() == 0.0f && ubx.nmea_vel_e_mps() == 0.0f, "缺省速度不刷新快照");
  makeNmea("GPRMC,092221.000,A,4250.5589,S,14718.5084,E,5.0,90.0,200806,,,A", sent, sizeof(sent));
  feedNmea(ubx, serial, sent);   // 正常速度恢复
  CHECK(fabs(ubx.nmea_vel_e_mps() - 5.0f * 0.514444f) < 1e-3f, "正常速度恢复刷新");

  printf("\n%s (失败 %d)\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
  return g_fail;
}
