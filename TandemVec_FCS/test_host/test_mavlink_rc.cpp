/**
 * @file    test_mavlink_rc.cpp
 * @module  固件/宿主回归测试（fw-tests：MAVLink RC 输入解析）
 * @brief   验证 RC_CHANNELS_OVERRIDE 的通道语义、寻址/来源过滤、超时与 reset
 * @details 被测：`src/mavlink_rc.cpp` 的 `MavlinkRc`
 *          输入：用 lib/MAVLink 官方编码器构造的 RC_CHANNELS_OVERRIDE 帧
 *          处理：假 Stream 逐字节喂入（虚拟时间注入，不用 millis）
 *          输出：CHECK 失败计数，main 返回非零即失败
 *          运行：g++ -std=c++17 -Iinclude -Isrc -Itest_host/stub -Ilib/MAVLink
 *                test_host/test_mavlink_rc.cpp src/mavlink_rc.cpp -o bin/mavrc
 * @theory  N/A（协议语义实现规约）；对端实现见
 *          @ref docs/reference/elrs-official/src/src/rx-serial/SerialMavlink.cpp
 * @frame   N/A
 * @units   通道值 µs（988-2012）
 * @note    T2/T3/T4 是安全项：0 值不覆盖、异机寻址拒绝、非接收机来源拒绝。
 *          任何一条失效都可能让空中油门归零或被地面站抢占遥控。
 * @test    T1 正常帧 16 通道 + 建链；T2 chanN=0 保持上一帧；T3 target 非本机丢弃；
 *          T4 来源 sysid 非接收机丢弃；T5 CRC 损坏不改通道；T6 超时断链；T7 reset 回中位
 * @see     TandemVec_FCS/src/mavlink_rc.h、src/communication.cpp（rcInputPoll）
 */
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

#include <Arduino.h>
#include "mavlink_rc.h"

static int g_fail = 0;
static int g_checks = 0;
#define CHECK(cond, msg) \
  do { ++g_checks; if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail++; } } while (0)

// ---- 假串口：按字节喂入，只实现 stub Stream 的三个纯虚函数 ----
class FakeSerial : public Stream {
public:
  void inject(const uint8_t* data, size_t len) { _buf.insert(_buf.end(), data, data + len); }
  void clear() { _buf.clear(); _pos = 0; }
  int available() override { return static_cast<int>(_buf.size() - _pos); }
  int read() override { return _pos < _buf.size() ? _buf[_pos++] : -1; }
  size_t write(const uint8_t*, size_t len) override { return len; }

private:
  std::vector<uint8_t> _buf;
  size_t _pos = 0;
};

// ---- 回调记录（验证通信层收到的值，而不是解析器内部状态）----
static uint16_t g_last[16] = {};
static int g_ch_calls = 0;
static int g_link_ups = 0;
static int g_link_downs = 0;

static void onChannels(const uint16_t us[16]) {
  std::memcpy(g_last, us, sizeof(g_last));
  ++g_ch_calls;
}
static void onLink(bool up) {
  if (up) { ++g_link_ups; } else { ++g_link_downs; }
}

static void resetRecorders() {
  std::memset(g_last, 0, sizeof(g_last));
  g_ch_calls = 0;
  g_link_ups = 0;
  g_link_downs = 0;
}

// ---- 帧构造：一律用官方编码器，CRC/CRC_EXTRA 由库保证 ----
static void buildOverride(std::vector<uint8_t>& out, uint8_t sysid,
                          uint8_t tgt_sys, uint8_t tgt_comp,
                          const uint16_t ch[16]) {
  mavlink_rc_channels_override_t m{};
  m.target_system = tgt_sys;
  m.target_component = tgt_comp;
  m.chan1_raw = ch[0];
  m.chan2_raw = ch[1];
  m.chan3_raw = ch[2];
  m.chan4_raw = ch[3];
  m.chan5_raw = ch[4];
  m.chan6_raw = ch[5];
  m.chan7_raw = ch[6];
  m.chan8_raw = ch[7];
  m.chan9_raw = ch[8];
  m.chan10_raw = ch[9];
  m.chan11_raw = ch[10];
  m.chan12_raw = ch[11];
  m.chan13_raw = ch[12];
  m.chan14_raw = ch[13];
  m.chan15_raw = ch[14];
  m.chan16_raw = ch[15];

  mavlink_message_t msg;
  mavlink_msg_rc_channels_override_encode(sysid, MAV_COMP_ID_TELEMETRY_RADIO, &msg, &m);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  out.assign(buf, buf + len);
}

static void feed(FakeSerial& s, MavlinkRc& rc, const std::vector<uint8_t>& frame, uint32_t now_ms) {
  s.inject(frame.data(), frame.size());
  rc.loop(now_ms);
}

int main() {
  FakeSerial serial;
  MavlinkRc rc;
  rc.setHandlers(onChannels, onLink);
  rc.begin(serial, 255);  // 只接受 ELRS 接收机的 sysid=255

  // 基准通道表：CH1-4 摇杆、CH5 解锁 2012、CH6 推进 2012、CH7 模式 1500、CH8 低、CH9 高
  uint16_t ch[16] = {1500, 1500, 1000, 1500, 2012, 2012, 1500, 988, 2012,
                     1500, 1500, 1500, 1500, 1500, 1500, 1500};
  std::vector<uint8_t> frame;

  // ---- T1 正常帧：16 通道全部落到回调 + 建链一次 ----
  resetRecorders();
  buildOverride(frame, 255, 0, 0, ch);
  feed(serial, rc, frame, 1000);
  CHECK(g_ch_calls == 1, "T1 应收到 1 次通道回调");
  CHECK(g_link_ups == 1, "T1 首帧应触发一次建链");
  CHECK(rc.linkUp(), "T1 建链后 linkUp 应为 true");
  CHECK(g_last[0] == 1500 && g_last[2] == 1000 && g_last[4] == 2012 && g_last[8] == 2012,
        "T1 通道值应与输入一致");
  CHECK(rc.channelFrames() == 1, "T1 通道帧计数应为 1");

  // ---- T2 chan3_raw=0 表示"本帧不覆盖"：必须保持上一帧 1000，不能归零 ----
  uint16_t ch_zero[16];
  std::memcpy(ch_zero, ch, sizeof(ch_zero));
  ch_zero[2] = 0;   // 油门通道留空
  ch_zero[0] = 1600;  // 其他通道照常更新
  buildOverride(frame, 255, 0, 0, ch_zero);
  feed(serial, rc, frame, 1010);
  CHECK(g_last[2] == 1000, "T2 油门 0 值必须保持上一帧（安全项）");
  CHECK(g_last[0] == 1600, "T2 其他通道仍应更新");

  // ---- T3 寻址过滤：target_system 指向别的机体，应丢弃 ----
  const int calls_before = g_ch_calls;
  const uint32_t dropped_before = rc.droppedFrames();
  buildOverride(frame, 255, 9, 0, ch);
  feed(serial, rc, frame, 1020);
  CHECK(g_ch_calls == calls_before, "T3 异机 target 不应触发通道回调");
  CHECK(rc.droppedFrames() == dropped_before + 1, "T3 应计入一次丢弃");

  // ---- T4 来源过滤：sysid=1（不是接收机 255），应丢弃 ----
  buildOverride(frame, 1, 0, 0, ch);
  feed(serial, rc, frame, 1030);
  CHECK(g_ch_calls == calls_before, "T4 GCS 来源不应触发通道回调");
  CHECK(rc.droppedFrames() == dropped_before + 2, "T4 应再计入一次丢弃");

  // ---- T5 CRC 损坏：帧被库丢弃，通道保持 ----
  buildOverride(frame, 255, 0, 0, ch);
  frame.back() ^= 0xFF;  // 打坏 CRC
  feed(serial, rc, frame, 1040);
  CHECK(g_ch_calls == calls_before, "T5 校验失败的帧不应改变通道");

  // ---- T6 超时：300 ms 无有效帧即断链 ----
  feed(serial, rc, {}, 1350);  // 1350 - 1010 > 300
  CHECK(!rc.linkUp(), "T6 超时后 linkUp 应为 false");
  CHECK(g_link_downs == 1, "T6 应触发一次断链回调");

  // ---- T7 reset：通道回中位、链路失效 ----
  rc.reset();
  CHECK(rc.channel(0) == MavlinkRc::kCenterUs, "T7 reset 后通道应回中位");
  CHECK(!rc.linkUp(), "T7 reset 后链路应失效");

  // ---- 收尾：恢复一帧后时间戳前进，超时判据不应误触发 ----
  resetRecorders();
  rc.setHandlers(onChannels, onLink);
  buildOverride(frame, 255, 0, 0, ch);
  feed(serial, rc, frame, 2000);
  rc.loop(2200);  // 仍在 300 ms 窗口内
  CHECK(rc.linkUp(), "收尾：200 ms 内不应超时");
  rc.loop(2400);
  CHECK(!rc.linkUp(), "收尾：400 ms 应超时");

  // 防假绿：用例被误删/循环未进入时，检查项数会塌下来
  CHECK(g_checks >= 18, "检查项数低于预期，疑似用例未执行");

  if (g_fail == 0) {
    printf("ALL PASS: mavlink_rc (%d checks)\n", g_checks);
  }
  return g_fail == 0 ? 0 : 1;
}