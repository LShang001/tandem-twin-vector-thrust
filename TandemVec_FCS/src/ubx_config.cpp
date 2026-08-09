// ============================================================
//  ubx_config.cpp — u-blox UBX 接收机自动配置实现（扩展版）
//
//  参考：SparkFun u-blox GNSS Arduino Library v2.2.29。
//  功能：波特率探测（目标优先/已固化快路径）→ CFG-PRT（波特率/协议掩码）
//  → CFG-RATE → CFG-MSG（NAV-PVT/NAV-EOE/任意消息）→ CFG-NAV5（动态模型，
//  读改写）→ CFG-GNSS（星座使能，读改写）→ CFG-SBAS → CFG-CFG（固化）
//  → 回读校验；另含 CFG-RST 软复位。全部走经典 CFG-* 消息（M8/M9/M10 兼容）。
// ============================================================
#include "ubx_config.h"

// 探测窗口/ACK 超时（uint32_t：此前误用 uint8_t 把 300/500ms 截断成 44/244ms）
static const uint32_t kDetectWindowMs = 300;   // 每档探测窗口
static const uint32_t kAckTimeoutMs = 500;     // ACK 等待超时

static UbxCfgResult s_last_cfg = {};

const UbxCfgResult &ubxLastCfgResult() { return s_last_cfg; }

// ---------------- 帧构建与校验 ----------------
// UBX 帧: B5 62 | cls | id | lenL | lenH | payload | CK_A | CK_B（标准 sum0/sum1）
static uint8_t s_txbuf[6 + 40 + 2];   // 最大配置帧（CFG-GNSS/CFG-NAV5 36B）

static void _ubx_send(HardwareSerial &serial, uint8_t cls, uint8_t id,
                      const uint8_t *payload, uint16_t len)
{
  uint8_t *p = s_txbuf;
  p[0] = 0xB5; p[1] = 0x62; p[2] = cls; p[3] = id;
  p[4] = (uint8_t)(len & 0xFF); p[5] = (uint8_t)(len >> 8);
  if (len > 0 && payload != nullptr)
  {
    memcpy(p + 6, payload, len);
  }
  uint8_t ck_a = 0, ck_b = 0;
  for (uint16_t i = 2; i < 6U + len; i++)
  {
    ck_a = (uint8_t)(ck_a + p[i]);
    ck_b = (uint8_t)(ck_b + ck_a);
  }
  p[6 + len] = ck_a; p[6 + len + 1] = ck_b;
  serial.write(s_txbuf, 6U + len + 2U);
}

// ---------------- ACK-ACK 等待（带回显校验）----------------
static bool _ubx_wait_ack(HardwareSerial &serial, uint8_t cls, uint8_t id, uint32_t timeout_ms)
{
  const uint32_t t0 = millis();
  uint8_t state = 0;
  uint8_t r_cls = 0, r_id = 0, ack_cls = 0, ack_id = 0;
  uint16_t r_len = 0;
  uint8_t ck_a = 0, ck_b = 0;
  while (millis() - t0 < timeout_ms)
  {
    while (serial.available())
    {
      const uint8_t b = serial.read();
      switch (state)
      {
        case 0: if (b == 0xB5) state = 1; break;
        case 1: if (b == 0x62) { state = 2; ck_a = ck_b = 0; } else state = 0; break;
        case 2: r_cls = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 3; break;
        case 3: r_id = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 4; break;
        case 4: r_len = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 5; break;
        case 5: r_len = (uint16_t)(r_len | ((uint16_t)b << 8));
                ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 6; break;
        default:
          if ((uint16_t)(state - 6) < r_len)
          {
            if ((state - 6) == 0) ack_cls = b;
            if ((state - 6) == 1) ack_id = b;
            ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a);
            state++;
          }
          else if ((uint16_t)(state - 6) == r_len)
          {
            if (b != ck_a) { state = 0; break; }
            state++;
          }
          else
          {
            if (b == ck_b && r_cls == 0x05 && r_id == 0x01 && ack_cls == cls && ack_id == id)
            {
              return true;
            }
            state = 0;
          }
          break;
      }
    }
    delay(1);
  }
  return false;
}

// ---------------- 通用"轮询读取"：发 len=0 的 CFG-* → 收同 cls/id 响应 payload ----------------
// 返回实际收到的 payload 长度（0 = 超时/失败）；校验通过才返回
static uint16_t _ubx_poll_payload(HardwareSerial &serial, uint8_t cls, uint8_t id,
                                  uint8_t *out, uint16_t out_max, uint32_t timeout_ms)
{
  _ubx_send(serial, cls, id, nullptr, 0);
  const uint32_t t0 = millis();
  uint8_t state = 0;
  uint16_t r_len = 0, got = 0;
  uint8_t r_cls = 0, r_id = 0;
  uint8_t ck_a = 0, ck_b = 0;
  while (millis() - t0 < timeout_ms)
  {
    while (serial.available())
    {
      const uint8_t b = serial.read();
      switch (state)
      {
        case 0: if (b == 0xB5) state = 1; break;
        case 1: if (b == 0x62) { state = 2; ck_a = ck_b = 0; } else state = 0; break;
        case 2: r_cls = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 3; break;
        case 3: r_id = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 4; break;
        case 4: r_len = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 5; break;
        case 5: r_len = (uint16_t)(r_len | ((uint16_t)b << 8));
                ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a);
                if (r_len > out_max) { state = 0; break; }
                got = 0; state = 6; break;
        default:
          if (got < r_len)
          {
            out[got++] = b;
            ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a);
            if (got == r_len) state = 7;
          }
          else if (state == 7)
          {
            if (b != ck_a) { state = 0; break; }
            state = 8;
          }
          else
          {
            if (b == ck_b && r_cls == cls && r_id == id)
            {
              return got;
            }
            state = 0;
          }
          break;
      }
    }
    delay(1);
  }
  return 0;
}

// ---------------- 波特率探测 ----------------
enum DetectKind { DETECT_NONE, DETECT_UBX, DETECT_NMEA };

static DetectKind _probe_baud(HardwareSerial &serial, uint32_t baud, uint32_t window_ms)
{
  serial.begin(baud);
  const uint32_t t0 = millis();
  uint8_t state = 0;
  uint16_t f_len = 0, got = 0;
  uint8_t ck_a = 0, ck_b = 0;
  while (millis() - t0 < window_ms)
  {
    while (serial.available())
    {
      const uint8_t b = serial.read();
      if (b == '$') return DETECT_NMEA;
      switch (state)
      {
        case 0: if (b == 0xB5) state = 1; break;
        case 1: if (b == 0x62) { state = 2; got = 0; ck_a = ck_b = 0; } else state = 0; break;
        case 2: ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 3; break;
        case 3: ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 4; break;
        case 4: f_len = b; ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a); state = 5; break;
        case 5: f_len = (uint16_t)(f_len | ((uint16_t)b << 8));
                ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a);
                if (f_len > 256) { state = 0; break; }
                got = 0; state = 6; break;
        default:
          if (got < f_len)
          {
            ck_a = (uint8_t)(ck_a + b); ck_b = (uint8_t)(ck_b + ck_a);
            got++;
            if (got == f_len) state = 7;
          }
          else if (state == 7)
          {
            if (b != ck_a) { state = 0; break; }
            state = 8;
          }
          else
          {
            if (b == ck_b) return DETECT_UBX;
            state = 0;
          }
          break;
      }
    }
    delay(1);
  }
  return DETECT_NONE;
}

// ---------------- 配置帧构造 ----------------

// CFG-PRT（20B）：UART1 → 目标波特率 + 协议掩码（nmea_out=false 时 UBX-only）
static void _cfg_prt(HardwareSerial &serial, uint32_t baud, bool nmea_out)
{
  uint8_t p[20] = {0};
  p[0] = 1;                          // portID = UART1
  p[4] = 0xD0; p[5] = 0x08;          // mode = 0x000008D0 (8N1)
  p[8]  = (uint8_t)(baud & 0xFF);
  p[9]  = (uint8_t)((baud >> 8) & 0xFF);
  p[10] = (uint8_t)((baud >> 16) & 0xFF);
  p[11] = (uint8_t)((baud >> 24) & 0xFF);
  p[14] = 0x01;                      // inProtoMask = UBX
  p[16] = nmea_out ? 0x03 : 0x01;    // outProtoMask = UBX(+NMEA)
  _ubx_send(serial, 0x06, 0x00, p, 20);
}

// CFG-RATE（6B）：测量周期 / 导航频率 / 时间基准
static void _cfg_rate(HardwareSerial &serial, uint16_t meas_rate_ms)
{
  uint8_t p[6] = {0};
  p[0] = (uint8_t)(meas_rate_ms & 0xFF);
  p[1] = (uint8_t)(meas_rate_ms >> 8);
  p[2] = 1;                          // navRate
  p[5] = 1;                          // timeRef = GPS
  _ubx_send(serial, 0x06, 0x08, p, 6);
}

// CFG-MSG（8B）：UART1 上使能指定消息
static void _cfg_msg(HardwareSerial &serial, uint8_t cls, uint8_t id, uint8_t rate)
{
  uint8_t p[8] = {cls, id, rate, 0, 0, 0, 0, 0};
  _ubx_send(serial, 0x06, 0x01, p, 8);
}

// CFG-NAV5（36B）读改写：mask 按需置位，只改指定字段（-1/0 表示不改）
//   dyn_model  动态模型：2=Airborne<2g(6)，4=Airborne<4g(7)；0=不改
//   fix_mode   定位模式：-1 不改，0=2D，1=3D，2=auto
//   min_elev   最低仰角：-1 不改（0-90°）
//   pdop_thresh pDOP 定位门限：-1 不改（单位 0.1，如 50=5.0）
// ★ 2026-08-09 修复：dynModel 在 payload[2]、mask 位在 payload[0]（bit0）——
//   旧实现把 dynModel 枚举值写进 mask 字节，Airborne 配置从未生效过。
static bool _cfg_nav5_rw(HardwareSerial &serial, int dyn_model, int fix_mode,
                         int min_elev, int pdop_thresh)
{
  uint8_t cur[36];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x24, cur, sizeof(cur), 500);
  if (n != 36)
  {
    return false;
  }
  uint16_t mask = (uint16_t)(cur[0] | ((uint16_t)cur[1] << 8));
  if (dyn_model > 0)
  {
    mask |= 0x0001;                    // bit0 = dynModel
    cur[2] = (uint8_t)((dyn_model == 2) ? 6 : 7);  // 6=Airborne<2g, 7=Airborne<4g
  }
  if (fix_mode >= 0)
  {
    mask |= 0x0002;                    // bit1 = fixMode
    cur[3] = (uint8_t)fix_mode;
  }
  if (min_elev >= 0)
  {
    mask |= 0x0010;                    // bit4 = minElev
    cur[12] = (uint8_t)min_elev;
  }
  if (pdop_thresh >= 0)
  {
    mask |= 0x0020;                    // bit5 = pDOP
    cur[13] = (uint8_t)pdop_thresh;
  }
  cur[0] = (uint8_t)(mask & 0xFF);
  cur[1] = (uint8_t)(mask >> 8);
  _ubx_send(serial, 0x06, 0x24, cur, 36);
  return _ubx_wait_ack(serial, 0x06, 0x24, kAckTimeoutMs);
}

// CFG-GNSS（星座配置）——读改写：轮询当前 → 按掩码改各星座使能位 → 回写
// gnss_mask: bit0 GPS(0) bit1 GLONASS(6) bit2 Galileo(2) bit3 BeiDou(3)
static bool _cfg_gnss(HardwareSerial &serial, uint8_t gnss_mask)
{
  uint8_t cur[40];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x3E, cur, sizeof(cur), 500);
  if (n < 8)
  {
    return false;                      // 接收机不支持 CFG-GNSS（老型号）
  }
  const uint8_t num_blocks = cur[3];
  const uint8_t block_size = 8;        // 每块 8B: gnssId,resTrkCh,maxTrkCh,res,flags[4]
  bool any_patched = false;
  for (uint8_t i = 0; i < num_blocks && 8U + (uint32_t)i * block_size + 8U <= n; i++)
  {
    uint8_t *blk = cur + 8 + (uint32_t)i * block_size;
    const uint8_t gnss_id = blk[0];
    const bool want = (gnss_id == 0 && (gnss_mask & 0x01)) ||
                      (gnss_id == 6 && (gnss_mask & 0x02)) ||
                      (gnss_id == 2 && (gnss_mask & 0x04)) ||
                      (gnss_id == 3 && (gnss_mask & 0x08));
    if (gnss_id == 0 || gnss_id == 6 || gnss_id == 2 || gnss_id == 3)
    {
      const uint8_t flags0 = blk[4];
      const uint8_t new_flags0 = want ? (uint8_t)(flags0 | 0x01) : (uint8_t)(flags0 & ~0x01U);
      if (new_flags0 != flags0)
      {
        blk[4] = new_flags0;
        any_patched = true;
      }
    }
  }
  if (!any_patched)
  {
    return true;                       // 已是目标配置
  }
  _ubx_send(serial, 0x06, 0x3E, cur, (uint16_t)n);
  return _ubx_wait_ack(serial, 0x06, 0x3E, kAckTimeoutMs);
}

// CFG-SBAS（8B）：差分增强使能
static bool _cfg_sbas(HardwareSerial &serial)
{
  uint8_t p[8] = {1, 0, 0, 0, 0, 0, 0, 0};   // enabled=1
  _ubx_send(serial, 0x06, 0x16, p, 8);
  return _ubx_wait_ack(serial, 0x06, 0x16, kAckTimeoutMs);
}

// CFG-CFG（13B）：固化当前配置到接收机非易失存储
static void _cfg_save(HardwareSerial &serial)
{
  uint8_t p[13] = {0};
  p[4] = 0x1F;
  _ubx_send(serial, 0x06, 0x09, p, 13);
}

// ---------------- 回读校验：CFG-PRT 轮询 ----------------
static bool _ubx_verify_prt(HardwareSerial &serial, uint32_t expected_baud)
{
  uint8_t payload[20];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x00, payload, sizeof(payload), 500);
  if (n < 12)
  {
    return false;
  }
  const uint32_t baud = (uint32_t)payload[8] | ((uint32_t)payload[9] << 8) |
                        ((uint32_t)payload[10] << 16) | ((uint32_t)payload[11] << 24);
  return baud == expected_baud;
}

// ============================================================
//  主入口
// ============================================================
UbxCfgResult ubxAutoConfig(HardwareSerial &serial, const UbxConfigOptions *opt)
{
  const UbxConfigOptions &o = (opt != nullptr) ? *opt : kUbxDefaultCfg;
  UbxCfgResult r = {};
  memset(&s_last_cfg, 0, sizeof(s_last_cfg));

  // 1. 波特率探测：目标波特率优先（已固化接收机 300ms 快路径）
  const uint32_t scan_order[6] = {o.target_baud, 9600U, 38400U, 57600U, 115200U, 921600U};
  for (uint8_t i = 0; i < 6; i++)
  {
    const uint32_t b = scan_order[i];
    if (b == 0U || (i > 0 && b == o.target_baud)) continue;
    if (_probe_baud(serial, b, kDetectWindowMs) != DETECT_NONE)
    {
      r.detected = true;
      r.found_baud = b;
      break;
    }
  }
  if (!r.detected)
  {
    s_last_cfg = r;
    return r;
  }

  // ★ 快路径：已在目标波特率（= 已固化过）→ 跳过全部配置
  if (r.found_baud == o.target_baud)
  {
    r.prt_ok = r.rate_ok = r.msg_pvt_ok = r.msg_eoe_ok =
      r.nav5_ok = r.gnss_ok = r.sbas_ok = r.save_ok = r.verify_ok = true;
    s_last_cfg = r;
    return r;
  }

  // 2. CFG-PRT：切波特率 + 协议掩码（先等 ACK 再切 MCU 波特率）
  _cfg_prt(serial, o.target_baud, o.nmea_out);
  r.cfg_frames_sent++;
  if (_ubx_wait_ack(serial, 0x06, 0x00, kAckTimeoutMs))
  {
    r.cfg_acks_received++;
  }
  delay(60);
  serial.begin(o.target_baud);
  r.prt_ok = true;

  // 3. CFG-RATE
  if (o.nav_rate_ms > 0)
  {
    _cfg_rate(serial, o.nav_rate_ms);
    r.cfg_frames_sent++;
    r.rate_ok = _ubx_wait_ack(serial, 0x06, 0x08, kAckTimeoutMs);
    if (r.rate_ok) r.cfg_acks_received++;
  }
  else
  {
    r.rate_ok = true;
  }

  // 4. CFG-MSG：NAV-PVT + NAV-EOE
  _cfg_msg(serial, 0x01, 0x07, 1);
  r.cfg_frames_sent++;
  r.msg_pvt_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  if (r.msg_pvt_ok) r.cfg_acks_received++;

  _cfg_msg(serial, 0x01, 0x61, 1);
  r.cfg_frames_sent++;
  r.msg_eoe_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  if (r.msg_eoe_ok) r.cfg_acks_received++;

  // 5. CFG-NAV5 动态模型（读改写；2026-08-09 修复 mask/dynModel 错位）
  if (o.airborne_g == 2 || o.airborne_g == 4)
  {
    r.cfg_frames_sent += 2;
    r.nav5_ok = _cfg_nav5_rw(serial, o.airborne_g, -1, -1, -1);
    if (r.nav5_ok) r.cfg_acks_received++;
  }
  else
  {
    r.nav5_ok = true;
  }

  // 6. CFG-GNSS 星座（读改写）
  if (o.gnss_mask != 0)
  {
    r.cfg_frames_sent += 2;
    r.gnss_ok = _cfg_gnss(serial, o.gnss_mask);
    if (r.gnss_ok) r.cfg_acks_received++;
  }
  else
  {
    r.gnss_ok = true;
  }

  // 7. CFG-SBAS
  if (o.sbas_enable)
  {
    r.cfg_frames_sent++;
    r.sbas_ok = _cfg_sbas(serial);
    if (r.sbas_ok) r.cfg_acks_received++;
  }
  else
  {
    r.sbas_ok = true;
  }

  // 8. 固化（可选）
  if (o.persist)
  {
    _cfg_save(serial);
    r.cfg_frames_sent++;
    r.save_ok = _ubx_wait_ack(serial, 0x06, 0x09, kAckTimeoutMs);
    if (r.save_ok) r.cfg_acks_received++;
  }
  else
  {
    r.save_ok = true;
  }

  // 9. 回读校验
  r.verify_ok = _ubx_verify_prt(serial, o.target_baud);

  s_last_cfg = r;
  return r;
}

// CFG-RST：GNSS 仅软件复位（navBbrMask=0x02, resetMode=0x02）
void ubxResetReceiver(HardwareSerial &serial)
{
  uint8_t p[4] = {0x02, 0x02, 0, 0};
  _ubx_send(serial, 0x06, 0x04, p, 4);
}

// ============================================================
//  NMEA 输出配置扩展（2026-08-09，DBG `ubxcfg nmea` / `ubxcfg proto`）
//
//  接收机侧 NMEA 输出控制——与库内解析（lib/ublox-main GpsProtocol）配合：
//  纯 NMEA 模块/UBX 配置失败时切 kAuto + 开 NMEA 输出，双保险。
// ============================================================

// 消息输出速率（CFG-MSG；rate=0 关闭，1-127 为 1/n 秒）。
// cls=0xF0 为 NMEA 句、cls=0x01 为 UBX 消息（NAV-PVT/EOE/DOP/SAT...）。
bool ubxMsgRateConfig(HardwareSerial &serial, uint8_t cls, uint8_t msg_id,
                      uint8_t rate)
{
  if (rate > 127)
  {
    return false;
  }
  _cfg_msg(serial, cls, msg_id, rate);
  return _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
}

// CFG-NMEA（0x06 0x17，20B V1）读改写公共骨架：轮询当前 → 改 offset → 回写 → 等 ACK
static bool _cfg_nmea_read_modify_write(HardwareSerial &serial, uint8_t offset,
                                        uint8_t value)
{
  uint8_t cur[20];
  const uint16_t n =
      _ubx_poll_payload(serial, 0x06, 0x17, cur, sizeof(cur), 500);
  if (n != 20)
  {
    return false;  // 未收到完整 CFG-NMEA（老固件/无响应）
  }
  cur[offset] = value;
  _ubx_send(serial, 0x06, 0x17, cur, 20);
  return _ubx_wait_ack(serial, 0x06, 0x17, kAckTimeoutMs);
}

// NMEA 版本（offset 1）：编码 = (主<<4)|次，如 0x23=2.3 / 0x41=4.1 / 0x4A=4.10 / 0x4B=4.11
bool ubxNmeaVersionConfig(HardwareSerial &serial, uint8_t version)
{
  if (version == 0)
  {
    return true;  // 0 = 不改
  }
  return _cfg_nmea_read_modify_write(serial, 1, version);
}

// 主 talker ID（offset 8）：NMEA_TALKER_*（0=不覆盖接收机默认）
bool ubxNmeaTalkerConfig(HardwareSerial &serial, uint8_t talker_id)
{
  if (talker_id == 0)
  {
    return true;  // 0 = 不改
  }
  return _cfg_nmea_read_modify_write(serial, 8, talker_id);
}

// NMEA 输出滤波掩码（offset 0）：NMEA_FILTER_*（0=不过滤，全量输出）
bool ubxNmeaFilterConfig(HardwareSerial &serial, uint8_t filter_mask)
{
  return _cfg_nmea_read_modify_write(serial, 0, filter_mask);
}

// 串口输出协议掩码（CFG-PRT outProtoMask，读改写——不动波特率/模式）
bool ubxProtoOutputConfig(HardwareSerial &serial, uint8_t out_mask)
{
  if (out_mask == 0 || out_mask > 3)
  {
    return false;  // 1=UBX-only，2=NMEA-only，3=UBX+NMEA
  }
  uint8_t p[20];
  const uint16_t n =
      _ubx_poll_payload(serial, 0x06, 0x00, p, sizeof(p), 500);
  if (n < 17)
  {
    return false;
  }
  p[16] = out_mask;
  _ubx_send(serial, 0x06, 0x00, p, 20);
  return _ubx_wait_ack(serial, 0x06, 0x00, kAckTimeoutMs);
}

// 导航引擎参数（CFG-NAV5 读改写，见 _cfg_nav5_rw）
bool ubxNav5Config(HardwareSerial &serial, int dyn_model, int fix_mode,
                   int min_elev, int pdop_thresh)
{
  return _cfg_nav5_rw(serial, dyn_model, fix_mode, min_elev, pdop_thresh);
}

// CFG-ITFM（0x06 0x39，8B）：CW 干扰检测开关（读改写 bit0）
bool ubxItfmConfig(HardwareSerial &serial, bool enable)
{
  uint8_t cur[8];
  const uint16_t n =
      _ubx_poll_payload(serial, 0x06, 0x39, cur, sizeof(cur), 500);
  if (n != 8)
  {
    return false;
  }
  if (enable)
  {
    cur[0] |= 0x01;
  }
  else
  {
    cur[0] &= (uint8_t)~0x01;
  }
  _ubx_send(serial, 0x06, 0x39, cur, 8);
  return _ubx_wait_ack(serial, 0x06, 0x39, kAckTimeoutMs);
}

// CFG-ANT（0x06 0x13，4B）：天线状态查询（纯读，不改配置）
//   返回 true 且 out[0..1]=config、out[2..3]=status；
//   status 低 2 位：0=INIT 1=UNKNOWN 2=OK 3=SHORT/OPEN（短路或开路，看 config 启用了哪个检测）
bool ubxAntStatusQuery(HardwareSerial &serial, uint8_t out[4])
{
  const uint16_t n =
      _ubx_poll_payload(serial, 0x06, 0x13, out, 4, 500);
  return n == 4;
}
