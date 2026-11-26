/**
 * @file    fs_ubxcfg_main.cpp
 * @module  固件/宿主 SIL 全链路（fw-tests：ubx_config 快路径与全量配置）
 * @brief   全链路数字孪生 P5：ubx_config 的开机快路径与全量配置路径验证
 *
 * @details 被测对象是真实 `src/ubx_config.cpp`（一行未改）。串口两侧角色对调：
 *          固件写出的每一帧被「替身接收机」按 UBX 协议解析，替身按真实的
 *          u-blox 行为回 ACK-ACK、回 poll 响应、维护 CFG 配置数据库
 *          （CFG-VALSET/VALGET 的键值表），并按配置状态决定是否输出
 *          NAV-PVT/NAV-EOE——这正是快路径判据要看的现象。
 *
 *          三个场景，覆盖「换接收机 / 已固化 / 半配置被改过」三类上电情况：
 *            S1 冷机：9600 + 只出 NMEA + 空配置库 → 必须走全量并配到目标；
 *            S2 已配置：921600 + PVT/EOE 在出 + 键值全对 → 必须命中快路径
 *               且发送帧数为 0（快路径的意义就是不重发配置）；
 *            S3 半配置：921600 + PVT/EOE 在出，但 UART1OUTPROT-NMEA=1、
 *               dynModel=7 → 快路径必须拒绝，落回全量并改正。
 *
 * @note  ① 虚拟时钟：`delay()` 只有装了 `g_sil_delay_hook` 才推进，而
 *          `ubxAutoConfig` 的所有等待都基于 millis()，所以必须安装。
 *        ② 替身是惰性的：固件调用 `serial.available()` 时才处理已写出的字节，
 *          不需要后台线程，也就没有时序不确定性。
 *        ③ 不链接 `lib/ublox-main`：ubx_config.cpp 自己组帧/解析，不经解析库。
 *
 * @frame N/A
 * @units ms / 字节 / bps
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

#include <Arduino.h>
#include "ubx_config.h"

// 虚拟时钟与 delay 钩子由本目标自己提供（不链接 fw_sil_stubs.cpp）。
uint32_t g_sil_time_us = 0;
SilDelayHook g_sil_delay_hook = nullptr;

namespace
{
int g_pass = 0;
int g_fail = 0;

void check(const char *what, bool ok, const char *detail = nullptr)
{
  if (ok)
  {
    ++g_pass;
    std::printf("[PASS] %s", what);
  }
  else
  {
    ++g_fail;
    std::printf("[FAIL] %s", what);
  }
  if (detail != nullptr)
  {
    std::printf("（%s）", detail);
  }
  std::printf("\n");
}

// ============================================================
//  UBX 组帧（供替身回包）
// ============================================================
void ubxAppend(std::vector<uint8_t> &out, uint8_t cls, uint8_t id,
               const uint8_t *payload, uint16_t len)
{
  out.push_back(0xB5);
  out.push_back(0x62);
  out.push_back(cls);
  out.push_back(id);
  out.push_back(static_cast<uint8_t>(len & 0xFF));
  out.push_back(static_cast<uint8_t>(len >> 8));
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  auto feed = [&ck_a, &ck_b](uint8_t b) {
    ck_a = static_cast<uint8_t>(ck_a + b);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  };
  feed(cls);
  feed(id);
  feed(static_cast<uint8_t>(len & 0xFF));
  feed(static_cast<uint8_t>(len >> 8));
  for (uint16_t i = 0; i < len; ++i)
  {
    out.push_back(payload[i]);
    feed(payload[i]);
  }
  out.push_back(ck_a);
  out.push_back(ck_b);
}

void putU2(uint8_t *p, uint16_t off, uint16_t v)
{
  p[off] = static_cast<uint8_t>(v & 0xFF);
  p[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void putU4(uint8_t *p, uint16_t off, uint32_t v)
{
  p[off] = static_cast<uint8_t>(v & 0xFF);
  p[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[off + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[off + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

uint16_t getU2(const uint8_t *p, uint16_t off)
{
  return static_cast<uint16_t>(p[off] | (static_cast<uint16_t>(p[off + 1]) << 8));
}

uint32_t getU4(const uint8_t *p, uint16_t off)
{
  return static_cast<uint32_t>(p[off]) |
         (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) |
         (static_cast<uint32_t>(p[off + 3]) << 24);
}

// 配置键（与固件 ubx_config.cpp 使用的是同一批 ID）
constexpr uint32_t kKeyGpsEna = 0x1031001FU;
constexpr uint32_t kKeySbasEna = 0x10310020U;
constexpr uint32_t kKeyGalEna = 0x10310021U;
constexpr uint32_t kKeyBdsEna = 0x10310022U;
constexpr uint32_t kKeyInProtNmea = 0x10730002U;
constexpr uint32_t kKeyOutProtUbx = 0x10740001U;
constexpr uint32_t kKeyOutProtNmea = 0x10740002U;
constexpr uint32_t kKeyMsgPvt = 0x20910007U;
constexpr uint32_t kKeyMsgEoe = 0x20910160U;
constexpr uint32_t kKeyOutfilPdop = 0x301100B1U;
constexpr uint32_t kKeyUart1Baud = 0x40520001U;

/** @brief 配置项的值长度（L=1 / U2=2 / U4=4），按一期用到的键判定 */
uint8_t kvSize(uint32_t key)
{
  if (key == kKeyUart1Baud)
  {
    return 4U;
  }
  if (key == kKeyOutfilPdop)
  {
    return 2U;
  }
  return 1U;
}

// ============================================================
//  替身接收机
// ============================================================
class SimReceiver
{
public:
  // ---- 可观测状态（真机上只能通过 VALGET/PRT poll 看到的那部分）----
  uint32_t baud = 9600U;
  bool ubx_stream = false;   ///< 是否在输出 NAV-PVT/NAV-EOE
  bool nmea_stream = true;   ///< 出厂态：只有 NMEA 句
  uint8_t prt_in_mask = 0x0001;
  uint8_t prt_out_mask = 0x0001;
  uint8_t nav5[36] = {};
  uint8_t rate[6] = {0x64, 0x00, 0x01, 0x00, 0x01, 0x00};  // 100ms / navRate 1 / GPS
  uint8_t gnss[4 + 5 * 8] = {};
  uint16_t gnss_len = 0;

  // ---- 统计（用于断言「快路径真的没发配置帧」）----
  uint32_t frames_in = 0;
  uint32_t acks_out = 0;
  uint32_t naks_out = 0;
  uint32_t valset_writes = 0;
  uint32_t last_stream_emit_ms = 0;
  bool stream_started = false;

  std::map<uint32_t, std::vector<uint8_t>> kv;

  SimReceiver()
  {
    // 出厂默认值（F9P 近似）：NMEA 进/出都开、SBAS 关、GPS/GAL/GLO 开、
    // BDS/QZSS 关、pDOP 输出滤波默认 25.0(=250)。
    kv[kKeyGpsEna] = {1};
    kv[kKeySbasEna] = {0};
    kv[kKeyGalEna] = {1};
    kv[kKeyBdsEna] = {0};
    kv[kKeyInProtNmea] = {1};
    kv[kKeyOutProtUbx] = {1};
    kv[kKeyOutProtNmea] = {1};
    kv[kKeyMsgPvt] = {0};
    kv[kKeyMsgEoe] = {0};
    kv[kKeyOutfilPdop] = {0xFA, 0x00};  // 250
    kv[kKeyUart1Baud] = {0x80, 0x25, 0x00, 0x00};  // 9600

    nav5[2] = 0;    // dynModel = portable
    nav5[3] = 3;    // fixMode = auto
    nav5[12] = 5;   // minElev = 5°
    putU2(nav5, 14, 250U);
    gnss_len = 0;   // 由 setupGnss() 按需填充
  }

  /** @brief 预置 CFG-GNSS 的 5 个星座块（GPS/GLO/GAL/BDS/QZSS） */
  void setupGnss()
  {
    static const uint8_t ids[5] = {0, 6, 2, 3, 5};
    gnss[0] = 0x00;  // version
    gnss[1] = 32;    // numTrkChHw
    gnss[2] = 60;    // numTrkChUse
    gnss[3] = 5;     // numConfigBlocks
    for (uint8_t i = 0; i < 5; ++i)
    {
      uint8_t *blk = gnss + 4 + i * 8;
      blk[0] = ids[i];
      blk[1] = 4;
      blk[2] = 12;
      blk[3] = 0;
      const bool on = (ids[i] == 0) || (ids[i] == 6) || (ids[i] == 2);
      blk[4] = on ? 0x01 : 0x00;
    }
    gnss_len = static_cast<uint16_t>(4 + 5 * 8);
  }

  /** @brief 把已配置状态同步到可观测字段（便于场景构造） */
  void syncFromKv()
  {
    prt_in_mask = kv[kKeyInProtNmea][0];
    prt_out_mask = kv[kKeyOutProtNmea][0];
    putU4(kv[kKeyUart1Baud].data(), 0, baud);
  }

  void setKv(uint32_t key, uint8_t v) { kv[key] = {v}; }
  uint8_t getKvL(uint32_t key) const
  {
    const auto it = kv.find(key);
    return (it == kv.end() || it->second.empty()) ? 0U : it->second[0];
  }

  /** @brief 回 ACK-ACK / ACK-NAK */
  void sendAck(uint8_t cls, uint8_t id, bool ok, std::vector<uint8_t> &tx)
  {
    uint8_t p[2] = {cls, id};
    ubxAppend(tx, 0x05, ok ? 0x01 : 0x00, p, 2);
    if (ok)
    {
      ++acks_out;
    }
    else
    {
      ++naks_out;
    }
  }

  /**
   * @brief 处理一帧来自飞控的命令
   * @return 需要回给飞控的字节（真机行为：命令必有 ACK 或 poll 响应）
   */
  void handleFrame(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len,
                   std::vector<uint8_t> &tx)
  {
    ++frames_in;
    if (std::getenv("SIM_UBX_DEBUG") != nullptr)
    {
      std::fprintf(stderr, "[sim] cmd %02X/%02X len=%u @%ums\n", cls, id,
                   static_cast<unsigned>(len),
                   static_cast<unsigned>(g_sil_time_us / 1000U));
    }
    switch (cls)
    {
      case 0x06:  // CFG
        switch (id)
        {
          case 0x00:  // CFG-PRT
            if (len == 0)
            {
              uint8_t out[20] = {};
              out[0] = 1;
              putU4(out, 4, 0x000008D0U);
              putU4(out, 8, baud);
              putU2(out, 12, prt_in_mask);
              putU2(out, 14, prt_out_mask);
              ubxAppend(tx, 0x06, 0x00, out, sizeof(out));
            }
            else if (len >= 20)
            {
              baud = getU4(p, 8);
              prt_in_mask = static_cast<uint8_t>(getU2(p, 12));
              prt_out_mask = static_cast<uint8_t>(getU2(p, 14));
              putU4(kv[kKeyUart1Baud].data(), 0, baud);
              sendAck(0x06, 0x00, true, tx);
            }
            else
            {
              sendAck(0x06, 0x00, false, tx);
            }
            break;

          case 0x01:  // CFG-MSG
            if (len >= 8)
            {
              const uint8_t mcls = p[0];
              const uint8_t mid = p[1];
              const uint8_t mrate = p[3];  // UART1 速率
              if (mcls == 0x01U && mid == 0x07U) kv[kKeyMsgPvt] = {mrate};
              if (mcls == 0x01U && mid == 0x61U) kv[kKeyMsgEoe] = {mrate};
              sendAck(0x06, 0x01, true, tx);
            }
            else
            {
              sendAck(0x06, 0x01, false, tx);
            }
            break;

          case 0x08:  // CFG-RATE
            if (len == 0)
            {
              ubxAppend(tx, 0x06, 0x08, rate, sizeof(rate));
            }
            else if (len >= 6)
            {
              std::memcpy(rate, p, 6);
              sendAck(0x06, 0x08, true, tx);
            }
            else
            {
              sendAck(0x06, 0x08, false, tx);
            }
            break;

          case 0x16:  // CFG-SBAS
            if (len == 0)
            {
              uint8_t cur[8] = {0};
              ubxAppend(tx, 0x06, 0x16, cur, sizeof(cur));
            }
            else
            {
              sendAck(0x06, 0x16, true, tx);
            }
            break;

          case 0x24:  // CFG-NAV5
            if (len == 0)
            {
              ubxAppend(tx, 0x06, 0x24, nav5, sizeof(nav5));
            }
            else if (len >= 36)
            {
              const uint16_t mask = getU2(p, 0);
              if (mask & 0x0001U) nav5[2] = p[2];              // dynModel
              if (mask & 0x0004U) nav5[3] = p[3];              // fixMode
              if (mask & 0x0002U) nav5[12] = p[12];            // minElev
              if (mask & 0x0010U) putU2(nav5, 14, getU2(p, 14));  // pDOP
              putU2(nav5, 0, mask);
              sendAck(0x06, 0x24, true, tx);
            }
            else
            {
              sendAck(0x06, 0x24, false, tx);
            }
            break;

          case 0x3E:  // CFG-GNSS
            if (len == 0)
            {
              if (gnss_len == 0) setupGnss();
              ubxAppend(tx, 0x06, 0x3E, gnss, gnss_len);
            }
            else
            {
              if (len <= sizeof(gnss))
              {
                std::memcpy(gnss, p, len);
                gnss_len = len;
              }
              sendAck(0x06, 0x3E, true, tx);
            }
            break;

          case 0x09:  // CFG-CFG 固化
            sendAck(0x06, 0x09, true, tx);
            break;

          case 0x04:  // CFG-RST
            sendAck(0x06, 0x04, true, tx);
            break;

          case 0x8A:  // CFG-VALSET：version/layers/transaction + [key(4) + value(N)]
          {
            if (len < 4)
            {
              sendAck(0x06, 0x8A, false, tx);
              break;
            }
            uint16_t off = 4;
            bool ok = true;
            while (off + 4 <= len)
            {
              const uint32_t key = getU4(p, off);
              const uint8_t vs = kvSize(key);
              if (off + 4 + vs > len)
              {
                ok = false;
                break;
              }
              std::vector<uint8_t> val(p + off + 4, p + off + 4 + vs);
              kv[key] = val;
              if (key == kKeyUart1Baud && vs == 4U)
              {
                baud = getU4(p, off + 4);
                nmea_stream = false;  // 切 UBX-only 后不再发 NMEA（真机同理）
              }
              if (key == kKeyInProtNmea) prt_in_mask = val[0];
              if (key == kKeyOutProtNmea) prt_out_mask = val[0];
              ++valset_writes;
              off = static_cast<uint16_t>(off + 4 + vs);
            }
            sendAck(0x06, 0x8A, ok, tx);
            break;
          }

          case 0x8B:  // CFG-VALGET：version/layer/position + keys
          {
            if (len < 4)
            {
              sendAck(0x06, 0x8B, false, tx);
              break;
            }
            std::vector<uint8_t> resp;
            resp.push_back(0x00);
            resp.push_back(p[1]);  // 回显 layer
            resp.push_back(p[2]);
            resp.push_back(p[3]);
            bool ok = true;
            for (uint16_t off = 4; off + 4 <= len; off = static_cast<uint16_t>(off + 4))
            {
              const uint32_t key = getU4(p, off);
              const uint8_t vs = kvSize(key);
              const auto it = kv.find(key);
              if (it == kv.end() || it->second.size() != vs)
              {
                ok = false;  // 真机对未知键回 NAK
                break;
              }
              uint8_t keybuf[4];
              putU4(keybuf, 0, key);
              resp.insert(resp.end(), keybuf, keybuf + 4);
              resp.insert(resp.end(), it->second.begin(), it->second.end());
            }
            if (ok)
            {
              ubxAppend(tx, 0x06, 0x8B, resp.data(),
                        static_cast<uint16_t>(resp.size()));
            }
            else
            {
              sendAck(0x06, 0x8B, false, tx);
            }
            break;
          }

          default:
            sendAck(0x06, id, false, tx);
            break;
        }
        break;

      case 0x0A:  // MON
        if (id == 0x04U)
        {
          uint8_t ver[40] = {};
          std::memcpy(ver, "ROM CORE 3.01 (107888)", 22);
          std::memcpy(ver + 30, "00080000", 8);
          ubxAppend(tx, 0x0A, 0x04, ver, sizeof(ver));
        }
        else
        {
          sendAck(0x0A, id, false, tx);
        }
        break;

      default:
        sendAck(cls, id, false, tx);
        break;
    }

    // MSGOUT 键决定是否持续输出（真机把 CFG-MSG 与 MSGOUT 合成同一份配置）
    ubx_stream = (getKvL(kKeyMsgPvt) > 0U) && (getKvL(kKeyMsgEoe) > 0U);
  }

  /** @brief 按虚拟时间产生接收机输出（NMEA 或 NAV-PVT/EOE，均 10 Hz） */
  void emitStreamIfDue(std::vector<uint8_t> &tx)
  {
    const uint32_t now_ms = g_sil_time_us / 1000U;
    // 不能用 last==0 当"还没发过"的哨兵：虚拟时间在 0 时刻赋 0 等于没记，
    // 会让 available() 每次调用都喂一句 NMEA，固件内层读循环永不结束。
    const bool due = !stream_started || ((now_ms - last_stream_emit_ms) >= 100U);
    if (!due)
    {
      return;
    }
    if (ubx_stream)
    {
      stream_started = true;
      last_stream_emit_ms = now_ms;
      if (std::getenv("SIM_UBX_DEBUG") != nullptr)
      {
        std::fprintf(stderr, "[sim] PVT/EOE @ %ums\n", static_cast<unsigned>(now_ms));
      }
      uint8_t pvt[92] = {};  // 内容不参与判据，只要 class/id/长度正确
      ubxAppend(tx, 0x01, 0x07, pvt, sizeof(pvt));
      uint8_t eoe[8] = {};
      ubxAppend(tx, 0x01, 0x61, eoe, sizeof(eoe));
      return;
    }
    if (nmea_stream)
    {
      // 出厂态持续吐 NMEA——真机不会只吐一句；快路径监听与波特率探测都会
      // 先把这段流读掉，只发一次会让后续探测误判"没有接收机"。
      stream_started = true;
      last_stream_emit_ms = now_ms;
      static const char kGga[] = "$GPGGA,000000.00,,,,,,,,,,*00\r\n";
      tx.insert(tx.end(), kGga, kGga + sizeof(kGga) - 1U);
    }
  }
};

// ============================================================
//  串口替身：写出的字节 → 替身接收机 → 回包喂回读缓冲
// ============================================================
class SimUart : public HardwareSerial
{
public:
  SimReceiver sim;

  /// 固件当前请求的链路波特率：替身只在它与接收机自身波特率一致时才应答，
  /// 这就是真实设备"波特率不对就一个字节都收不到"的语义。
  void begin(unsigned long baud) override { host_baud_ = static_cast<uint32_t>(baud); }

  /// 处理固件写出的字节，并把替身的回应（含定时流）喂进读缓冲。
  void pump()
  {
    std::vector<uint8_t> tx;
    if (host_baud_ == sim.baud)
    {
      sim.emitStreamIfDue(tx);
    }

    const size_t n = silTxLen();
    while (consumed_ < n)
    {
      frame_[fpos_++] = silTxBuf()[consumed_++];
      // 帧头同步：只认 B5 62 起点
      if (fpos_ == 1U && frame_[0] != 0xB5U)
      {
        fpos_ = 0;
        continue;
      }
      if (fpos_ == 2U && frame_[1] != 0x62U)
      {
        fpos_ = 0;
        continue;
      }
      if (fpos_ < 6U)
      {
        continue;
      }
      const uint16_t len = static_cast<uint16_t>(frame_[4] | (frame_[5] << 8));
      const size_t total = static_cast<size_t>(6U + len + 2U);
      if (total > sizeof(frame_))
      {
        fpos_ = 0;
        continue;
      }
      if (fpos_ < total)
      {
        continue;
      }
      sim.handleFrame(frame_[2], frame_[3], &frame_[6], len, tx);
      fpos_ = 0;
    }

    if (!tx.empty())
    {
      feed(tx.data(), tx.size());
    }
  }

  int available() override
  {
    pump();
    return HardwareSerial::available();
  }
  int read() override { return HardwareSerial::read(); }
  int peek() override { return HardwareSerial::peek(); }

  /** @brief 清写缓冲后必须同步复位解析游标，否则新场景会漏掉前几帧 */
  void resetFraming()
  {
    consumed_ = 0;
    fpos_ = 0;
  }

private:
  size_t consumed_ = 0;
  uint8_t frame_[600] = {};
  size_t fpos_ = 0;
  uint32_t host_baud_ = 0;
};

/** @brief 构造一个"已配置到位"的接收机（S2/S3 用） */
void makeConfigured(SimReceiver &sim, bool good_outprot_nmea, uint8_t dyn_model)
{
  sim.baud = 921600U;
  sim.setupGnss();
  sim.setKv(kKeyInProtNmea, 0);
  sim.setKv(kKeyOutProtUbx, 1);
  sim.setKv(kKeyOutProtNmea, good_outprot_nmea ? 0 : 1);
  sim.setKv(kKeySbasEna, 0);
  sim.setKv(kKeyMsgPvt, 1);
  sim.setKv(kKeyMsgEoe, 1);
  // pDOP 是 U2 型键，必须写满 2 字节：值长度与 VALGET 的 kvSize 不一致
  // 会被替身按"未知键"回 NAK，快路径随之失败（曾因此误判成"已配置不成立"）。
  sim.kv[kKeyOutfilPdop] = {0xFA, 0x00};
  sim.kv[kKeyUart1Baud] = {0x00, 0x0E, 0x00, 0x00};  // 921600
  sim.nav5[2] = dyn_model;
  sim.nav5[3] = 3;
  sim.nav5[12] = 5;
  putU2(sim.nav5, 14, 250U);
  sim.ubx_stream = true;
  sim.nmea_stream = false;
  sim.syncFromKv();
}

/**
 * @brief 跑一次 ubxAutoConfig 并返回结果
 * @param sim 预置好的接收机状态
 * @param frames_sent_seen 回填替身收到的配置帧数
 */
UbxCfgResult runOnce(SimUart &uart, const UbxConfigOptions &opt, uint32_t *frames_seen)
{
  uart.sim.frames_in = 0;
  uart.silClearTx();
  uart.resetFraming();
  const UbxCfgResult r = ubxAutoConfig(uart, &opt);
  if (frames_seen != nullptr)
  {
    *frames_seen = uart.sim.frames_in;
  }
  return r;
}

}  // namespace

int main()
{
  // stdout 可能是管道（全缓冲）：不设无缓冲的话，卡住时看不到已经跑过的场景，
  // 排查死循环会误判成"什么都没执行"。
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  // 所有等待都基于 millis()：必须让 delay() 推进虚拟时钟。
  g_sil_delay_hook = [](uint32_t us) { g_sil_time_us += us; };

  const UbxConfigOptions opt = kUbxDefaultCfg;

  // ---------------- S1：冷接收机（9600 / 只有 NMEA / 空配置库）----------------
  std::printf("\n== S1 冷接收机：必须走全量配置 ==\n");
  {
    static SimUart uart;  // 静态：避免 4KB 读缓冲压在栈上
    g_sil_time_us = 0;
    uint32_t frames_seen = 0;
    const UbxCfgResult r = runOnce(uart, opt, &frames_seen);

    check("S1 探测到接收机", r.detected);
    check("S1 探测到出厂 9600", r.found_baud == 9600U,
          r.found_baud == 9600U ? nullptr : "波特率不符");
    check("S1 未命中快路径", !r.fast_path);
    check("S1 CFG-RATE 生效", r.rate_ok);
    check("S1 NAV-PVT 使能", r.msg_pvt_ok);
    check("S1 NAV-EOE 使能", r.msg_eoe_ok);
    check("S1 VALSET 被确认", r.valset_ok);
    check("S1 CFG-NAV5 写入", r.nav5_ok);
    check("S1 NAV5 回读 dynModel=8（Airborne<4g>）", r.nav5_dyn_readback == 8U,
          r.nav5_dyn_readback == 8U ? nullptr : "回读不是 8");
    check("S1 星座优化执行", r.gnss_ok);
    check("S1 CFG-PRT 回读波特率=921600", r.verify_ok);
    check("S1 发送配置帧数 > 10", r.cfg_frames_sent > 10U);
    check("S1 收到 ACK 数与发送帧数同量级", r.cfg_acks_received >= 6U);

    // 接收机侧最终状态：这才证明"配置真的落到目标字段"，而不只是 ACK
    check("S1 接收机侧波特率 = 921600", uart.sim.baud == 921600U);
    check("S1 接收机侧 UART1OUTPROT-NMEA = 0",
          uart.sim.getKvL(kKeyOutProtNmea) == 0U);
    check("S1 接收机侧 UART1INPROT-NMEA = 0",
          uart.sim.getKvL(kKeyInProtNmea) == 0U);
    check("S1 接收机侧 MSGOUT PVT/EOE 均 = 1",
          uart.sim.getKvL(kKeyMsgPvt) == 1U && uart.sim.getKvL(kKeyMsgEoe) == 1U);
    check("S1 接收机侧 NAV5 dynModel = 8", uart.sim.nav5[2] == 8U,
          uart.sim.nav5[2] == 8U ? nullptr : "接收机里没改到");
    check("S1 接收机侧 pDOP 门限 = 250",
          getU2(uart.sim.nav5, 14) == 250U);
    check("S1 接收机侧 CFG-RATE = 100ms",
          getU2(uart.sim.rate, 0) == 100U);
    std::printf("      替身收到 %u 帧，回 ACK %u 个\n",
                static_cast<unsigned>(frames_seen),
                static_cast<unsigned>(uart.sim.acks_out));
  }

  // ---------------- S2：已配置（快路径命中）----------------
  std::printf("\n== S2 已配置接收机：必须命中快路径 ==\n");
  {
    static SimUart uart;
    g_sil_time_us = 0;
    makeConfigured(uart.sim, true, 8U);
    uint32_t frames_seen = 0;
    const UbxCfgResult r = runOnce(uart, opt, &frames_seen);

    check("S2 命中快路径", r.fast_path);
    check("S2 探测到 921600", r.found_baud == 921600U);
    check("S2 未发配置帧（快路径的意义）", r.cfg_frames_sent == 0U,
          r.cfg_frames_sent == 0U ? nullptr : "快路径仍发了配置帧");
    check("S2 NAV5 回读 dynModel=8", r.nav5_dyn_readback == 8U);
    check("S2 各 * _ok 为已验证而非空", r.prt_ok && r.rate_ok && r.valset_ok &&
                                            r.verify_ok && r.save_ok);
    check("S2 替身仅收到只读轮询（≤6 帧）", frames_seen <= 6U,
          frames_seen <= 6U ? nullptr : "轮询帧数异常");
  }

  // ---------------- S3：半配置（PVT/EOE 在出但 NMEA 输出未关、dynModel 低一档）----------------
  std::printf("\n== S3 半配置接收机：快路径必须拒绝并改正 ==\n");
  {
    static SimUart uart;
    g_sil_time_us = 0;
    makeConfigured(uart.sim, false, 7U);  // OUTPROT-NMEA=1、dynModel=<2g>
    uint32_t frames_seen = 0;
    const UbxCfgResult r = runOnce(uart, opt, &frames_seen);

    check("S3 未命中快路径", !r.fast_path);
    check("S3 回落到全量后 NAV5 dynModel 修正为 8", r.nav5_dyn_readback == 8U,
          r.nav5_dyn_readback == 8U ? nullptr : "没纠正动态模型");
    check("S3 接收机侧 UART1OUTPROT-NMEA 被改回 0",
          uart.sim.getKvL(kKeyOutProtNmea) == 0U);
    check("S3 接收机侧 NAV5 dynModel = 8", uart.sim.nav5[2] == 8U);
    check("S3 发送了配置帧", r.cfg_frames_sent > 0U);
  }

  std::printf("\n=== ubx_config 快路径/全量路径：PASS %d, FAIL %d ===\n", g_pass,
              g_fail);
  return g_fail == 0 ? 0 : 1;
}