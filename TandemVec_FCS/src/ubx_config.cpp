/**
 * @file    ubx_config.cpp
 * @module  固件/通信（GNSS 接收机 UBX 配置）
 * @brief   实现 UBX 接收机自动配置、消息速率/NMEA 选项与 CFG-RST 软复位
 *
 * @details 参考：SparkFun u-blox GNSS Arduino Library v2.2.29。
 *          功能：波特率探测（目标优先/已固化快路径）→ CFG-PRT（波特率/协议掩码）
 *          → CFG-RATE → CFG-MSG（NAV-PVT/NAV-EOE/任意消息）→ CFG-NAV5（动态模型，
 *          读改写）→ CFG-GNSS（星座使能，读改写）→ CFG-SBAS → CFG-CFG（固化）
 *          → 回读校验；另含 CFG-RST 软复位。全部走经典 CFG-* 消息（M8/M9/M10 兼容）。
 * @theory  N/A（外设配置协议，非理论模型）
 * @ref     docs/reference/sparkfun-ublox-gnss；UBX 帧格式见本文件 `_ubx_send()` 注释
 * @frame   N/A
 * @units   波特率 bps、探测/ACK 超时 ms、NAV5 参数按 u-blox 协议整数单位
 * @note    ① UBX 帧：`B5 62 | cls | id | lenL lenH | payload | CK_A CK_B`，
 *            校验为逐字节 sum0/sum1（与 AnoCom 的双校验算法不同，勿混用）；
 *          ② 探测窗口 300ms/档、ACK 超时 500ms，必须用 uint32_t 保存
 *            （曾误用 uint8_t 导致 300/500ms 被截断成 44/244ms）；
 *          ③ 配置函数是阻塞调用（约 1-4s），只能出现在 setup 或 DBG 命令中，
 *            绝不可放进 200Hz 任务。
 */
#include "ubx_config.h"

// 探测窗口/ACK 超时（uint32_t：此前误用 uint8_t 把 300/500ms 截断成 44/244ms）
static const uint32_t kDetectWindowMs = 300;   // 每档探测窗口
static const uint32_t kAckTimeoutMs = 500;     // ACK 等待超时
// 开机快路径的监听窗口：10 Hz 下 400 ms 约 4 个 PVT/EOE，阈值取 3 留一帧余量。
static const uint32_t kQuickVerifyMs = 400;
static const uint32_t kQuickMinFrames = 3;     // 窗口内至少见到的 PVT/EOE 帧数

static UbxCfgResult s_last_cfg = {};
static uint32_t s_gnss_serial_config_fail_count = 0U;

const UbxCfgResult &ubxLastCfgResult() { return s_last_cfg; }

uint32_t ubxGnssSerialConfigFailCount() { return s_gnss_serial_config_fail_count; }

// ---------------- GNSS 串口初始化收口 ----------------
// Uart::begin() 会重跑 uart_init：NVIC 恢复默认优先级，UART_SetConfig 也清 FIFOEN。
// 必须在每次 begin 之后恢复，不能只在上电 setup() 前段设一次。
bool ubxGnssSerialBegin(Uart &serial, uint32_t baud)
{
  serial.begin(baud);

  UART_HandleTypeDef *huart = serial.getHandle();
  if (huart == nullptr || huart->Instance != UART4)
  {
    ++s_gnss_serial_config_fail_count;
    return false;
  }

  // begin() 已给 Arduino 的 recv 字节启动 1 字节 IT 接收。先保存其目标地址，
  // 停止旧事务后用 HAL 正式配置 FIFO，再以原地址重启；不直接访问 Uart 私有成员。
  uint8_t *rx_target = huart->pRxBuffPtr;
  if (rx_target == nullptr || huart->RxState != HAL_UART_STATE_BUSY_RX)
  {
    ++s_gnss_serial_config_fail_count;
    return false;
  }

  HAL_NVIC_DisableIRQ(UART4_IRQn);
  HAL_StatusTypeDef st = HAL_UART_AbortReceive(huart);
  if (st == HAL_OK) st = HAL_UARTEx_EnableFifoMode(huart);
  if (st == HAL_OK) st = HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_2);
  if (st == HAL_OK) st = HAL_UART_Receive_IT(huart, rx_target, 1U);
  if (st != HAL_OK)
  {
    // FIFO 任一步失败不能留下关闭的接收通道；尽力恢复原逐字节接收。
    (void)HAL_UART_Receive_IT(huart, rx_target, 1U);
  }
  NVIC_SetPriority(UART4_IRQn, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  const bool ok = (st == HAL_OK) &&
                  ((UART4->CR1 & USART_CR1_FIFOEN) != 0U) &&
                  (huart->FifoMode == UART_FIFOMODE_ENABLE) &&
                  (huart->RxState == HAL_UART_STATE_BUSY_RX) &&
                  (NVIC_GetPriority(UART4_IRQn) == 0U);
  if (!ok)
  {
    ++s_gnss_serial_config_fail_count;
  }
  return ok;
}

// ---------------- 帧构建与校验 ----------------
// UBX 帧: B5 62 | cls | id | lenL | lenH | payload | CK_A | CK_B（标准 sum0/sum1）
static uint8_t s_txbuf[6 + 40 + 2];   // 最大配置帧（CFG-GNSS/CFG-NAV5 36B）

static void _ubx_send(Uart &serial, uint8_t cls, uint8_t id,
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
/**
 * @brief 等待 UBX ACK-ACK 并校验 cls/id 匹配
 * @param[in] cls        期望被确认的 class（ACK 载荷里的 clsID）
 * @param[in] id         期望被确认的 id（ACK 载荷里的 msgID）
 * @param[in] timeout_ms 超时 [ms]
 * @return true=收到匹配的 ACK；false=超时，或只收到 NAK/其他帧
 * @note ① 自带逐字节状态机解析，不复用库内解析器——那段数据还要喂给上层。
 *       ② 只认 ACK-ACK（0x05/0x01）；ACK-NAK（0x05/0x00）不单独区分，
 *         一律按"未确认"返回 false。
 */
static bool _ubx_wait_ack(Uart &serial, uint8_t cls, uint8_t id, uint32_t timeout_ms)
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

// ---------------- 通用"请求-响应"：发一帧 cls/id（可带请求 payload）→ 收同 cls/id 响应 ----------------
// 返回实际收到的 payload 长度（0 = 超时/失败）；校验通过才返回
static uint16_t _ubx_request_payload(Uart &serial, uint8_t cls, uint8_t id,
                                     const uint8_t *req, uint16_t req_len,
                                     uint8_t *out, uint16_t out_max, uint32_t timeout_ms)
{
  _ubx_send(serial, cls, id, req, req_len);
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

// ---------------- 空 payload 轮询（CFG-PRT/NAV5/GNSS 这类 Get 请求） ----------------
static uint16_t _ubx_poll_payload(Uart &serial, uint8_t cls, uint8_t id,
                                  uint8_t *out, uint16_t out_max, uint32_t timeout_ms)
{
  return _ubx_request_payload(serial, cls, id, nullptr, 0, out, out_max, timeout_ms);
}

// ---------------- CFG-VALGET / CFG-VALSET：新协议下的配置数据库访问 ----------------
// ★2026-11-19：CFG-GNSS(0x06 0x3E) 在协议版本 >23.01 已废弃（F9P HPG 1.32 即属此列），
//   官方要求改用 CFG-VALGET/VALSET 操作配置项。实测废弃接口对星座使能位只有部分
//   生效（能关不能开、GPS 关不掉），正是走错接口的表现。
//   VALGET 请求：version(0) layer(1) position(2-3 U2) keys(4+，每项 U4)
//   VALGET 响应：version(0) layer(1) position(2-3) 然后 key(U4)+value 依次拼接
//   VALSET 请求：version(0) layers(1) transaction(2-3) 然后 key(U4)+value 依次拼接

/**
 * @brief 用 CFG-VALGET 读取若干 L 型（单字节）配置项
 * @param[in]  keys    键 ID 数组
 * @param[in]  count   键数量（≤16）
 * @param[out] values  对应值（每项 1 字节）
 * @return true=全部读回成功
 * @note layer 固定 0（RAM，即当前生效值）；请求键数受限，超出返回 false。
 */
static bool _valget_l(Uart &serial, const uint32_t *keys, uint8_t count, uint8_t *values)
{
  if (keys == nullptr || values == nullptr || count == 0 || count > 16)
  {
    return false;
  }
  uint8_t req[4 + 16 * 4];
  req[0] = 0x00;                 // version
  req[1] = 0x00;                 // layer = RAM（当前生效值）
  req[2] = 0x00;                 // position
  req[3] = 0x00;
  for (uint8_t i = 0; i < count; ++i)
  {
    ubxLayout::putU4(req, (uint16_t)(4 + i * 4), keys[i]);
  }
  uint8_t resp[4 + 16 * 5];
  const uint16_t n = _ubx_request_payload(serial, 0x06, 0x8B, req,
                                          (uint16_t)(4 + count * 4), resp,
                                          sizeof(resp), 500);
  if (n < 4 + count * 5)
  {
    return false;
  }
  // 响应按请求顺序返回 key+value，逐项比对 key 后取值
  for (uint8_t i = 0; i < count; ++i)
  {
    const uint16_t off = (uint16_t)(4 + i * 5);
    if (ubxLayout::getU4(resp, off) != keys[i])
    {
      return false;
    }
    values[i] = resp[off + 4];
  }
  return true;
}

/**
 * @brief 用 CFG-VALSET 写入若干 L 型（单字节）配置项
 * @param[in] keys   键 ID 数组
 * @param[in] values 对应值
 * @param[in] count  键数量（≤16）
 * @return true=接收机回 ACK
 * @note layers=0x01 只写 RAM，掉电不保留，需 _cfg_save 固化。
 */
static bool _valset_l(Uart &serial, const uint32_t *keys, const uint8_t *values, uint8_t count)
{
  if (keys == nullptr || values == nullptr || count == 0 || count > 16)
  {
    return false;
  }
  uint8_t req[4 + 16 * 5];
  req[0] = 0x00;                 // version
  req[1] = 0x01;                 // layers = RAM
  req[2] = 0x00;                 // transaction
  req[3] = 0x00;
  for (uint8_t i = 0; i < count; ++i)
  {
    ubxLayout::putU4(req, (uint16_t)(4 + i * 5), keys[i]);
    req[4 + i * 5 + 4] = values[i];
  }
  _ubx_send(serial, 0x06, 0x8A, req, (uint16_t)(4 + count * 5));
  return _ubx_wait_ack(serial, 0x06, 0x8A, kAckTimeoutMs);
}

/** @brief 六星座使能位打包（bit0 GPS, bit1 SBAS, bit2 GAL, bit3 BDS, bit4 QZSS, bit5 GLO） */
static const uint32_t kSignalEnableKeys[6] = {
    0x1031001FU,  // CFG-SIGNAL-GPS_ENA
    0x10310020U,  // CFG-SIGNAL-SBAS_ENA
    0x10310021U,  // CFG-SIGNAL-GAL_ENA
    0x10310022U,  // CFG-SIGNAL-BDS_ENA
    0x10310024U,  // CFG-SIGNAL-QZSS_ENA
    0x10310025U,  // CFG-SIGNAL-GLO_ENA
};

// ---------------- 波特率探测 ----------------
enum DetectKind { DETECT_NONE, DETECT_UBX, DETECT_NMEA };

/**
 * @brief 在指定波特率下探测接收机输出类型
 * @param[in] baud      待探测波特率
 * @param[in] window_ms 监听窗口 [ms]
 * @return DETECT_UBX=收到校验通过的 UBX 帧；DETECT_NMEA=收到 '$' 起始的
 *         NMEA 句；DETECT_NONE=窗口内没有有效数据
 * @note ① 探测前统一走 ubxGnssSerialBegin() 重开串口——直接 begin() 会清掉
 *         FIFOEN 并复位中断优先级（见 GNSS 串口丢帧记录）。
 *       ② NMEA 判据只看首字节 '$'，不校验整句校验和，噪声大时可能误判。
 */
static DetectKind _probe_baud(Uart &serial, uint32_t baud, uint32_t window_ms)
{
  (void)ubxGnssSerialBegin(serial, baud);
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
/**
 * @brief 下发 CFG-PRT + CFG-VALSET 补写输出协议掩码
 * @param[in] baud     目标波特率
 * @param[in] nmea_out true=UBX+NMEA 输出，false=UBX-only
 * @note ★2026-11-19：CFG-PRT 在协议版本 >23.01 已废弃（F9P PROTVER=34.00）。
 *       实测它的 outProtoMask 写入不生效——配的是 UBX-only，用 CFG-VALGET 读
 *       CFG-UART1OUTPROT-NMEA 仍是 1，接收机一直在输出 NMEA。因此这里先发
 *       CFG-PRT（兼容老机型、且波特率那项实测有效），再用 CFG-VALSET 显式
 *       写 CFG-UART1OUTPROT-NMEA，把输出协议真正定死。
 *       配置项键值（同手册配置项表）：
 *         CFG-UART1OUTPROT-UBX  = 0x10740001（保持 1）
 *         CFG-UART1OUTPROT-NMEA = 0x10740002（按 nmea_out 置 0/1）
 */
static void _cfg_prt(Uart &serial, uint32_t baud, bool nmea_out)
{
  using namespace ubxLayout;
  uint8_t p[prt::kPayloadBytes] = {0};
  p[prt::kPortId] = 1;                                  // UART1
  putU4(p, prt::kMode, prt::kMode8N1);                  // 8N1
  putU4(p, prt::kBaudRate, baud);
  putU2(p, prt::kInProtoMask, 0x0001);                  // 只收 UBX（含 RTCM 的走 inRtcm3）
  putU2(p, prt::kOutProtoMask, nmea_out ? 0x0003 : 0x0001);  // 只发 UBX，可选加 NMEA
  _ubx_send(serial, 0x06, 0x00, p, prt::kPayloadBytes);
}

/**
 * @brief 用 CFG-VALSET 定死 UART1 的输入/输出协议掩码（NMEA 侧）
 * @param[in] nmea_out true=允许 NMEA 输出，false=只输出 UBX
 * @return 无（调用方按需回读验证）
 * @note ★2026-11-19：CFG-PRT 的 inProtoMask/outProtoMask 在 F9P(PROTVER=34.00)
 *       上都不生效。实测回读：
 *         CFG-UART1INPROT-NMEA   = 1  （配的是 0x0001，不含 NMEA）
 *         CFG-UART1INPROT-RTCM3X = 1  （配的是 0x0001，不含 RTCM3）
 *         CFG-UART1OUTPROT-NMEA  = 1  （配的是 UBX-only）
 *       inProtoMask 失效反而救了 RTK——若它真生效，RTCM3 差分就收不到。
 *       现改用配置项接口逐项写：
 *         CFG-UART1INPROT-NMEA  0x10730002 -> 0（不收 NMEA，只需要 UBX+RTCM3）
 *         CFG-UART1OUTPROT-NMEA 0x10740002 -> 按 nmea_out
 *       注意 RTCM3X 输入必须保持 1（RTK 差分改正数走它），不在此处改动。
 *
 *       失效规律（本轮实测归纳）：legacy 字段与配置项严格 1:1 的仍然有效
 *       （CFG-MSG.rate→CFG-MSGOUT、CFG-RATE.measRate→CFG-RATE-MEAS、
 *       CFG-NAV5 的 dynModel/fixMode/minElev/pDop），而一个字段要拆成多个
 *       配置项的（CFG-GNSS.gnssId→5 个 CFG-SIGNAL、CFG-PRT.mask→3 个
 *       CFG-UART1*PROT）在新协议下不落地。
 *       时序：不能在 _cfg_prt 里紧跟着发，CFG-PRT 会让接收机重配 UART，
 *       紧随其后的 VALSET 会被丢掉（实测 readback 仍为 1）。所以单独成步。
 */
static void _cfg_uart1_proto_masks(Uart &serial, bool nmea_out)
{
  const uint32_t keys[2] = {
      0x10730002U,   // CFG-UART1INPROT-NMEA
      0x10740002U,   // CFG-UART1OUTPROT-NMEA
  };
  const uint8_t vals[2] = {0, (uint8_t)(nmea_out ? 1 : 0)};
  (void)_valset_l(serial, keys, vals, 2);
}

// CFG-RATE（6B）：测量周期 / 导航频率 / 时间基准
/**
 * @brief 下发 CFG-RATE：设置测量周期与导航频率
 * @param[in] meas_rate_ms 测量周期 [ms]（100 → 10 Hz）
 * @note navRate 固定 1、timeRef 固定 GPS，即"测量周期 = 导航解算周期"；
 *       要改导航频率必须同时改 meas_rate，不存在单独的导航率。
 */
static void _cfg_rate(Uart &serial, uint16_t meas_rate_ms)
{
  using namespace ubxLayout;
  uint8_t p[rate::kPayloadBytes] = {0};
  putU2(p, rate::kMeasRate, meas_rate_ms);
  putU2(p, rate::kNavRate, 1);
  putU2(p, rate::kTimeRef, rate::kTimeRefGps);
  _ubx_send(serial, 0x06, 0x08, p, rate::kPayloadBytes);
}

// CFG-MSG（8B）：UART1 上使能指定消息
// CFG-MSG（8 字节版）：payload = cls, id, rateDDC, rateUART1, rateUART2,
// rateUSB, rateSPI, reserved。
// ★2026-11-03 修复：旧实现把 rate 放在 rateDDC、UART1/UART2 全 0，接收机
//   仍回 ACK，但串口（UART1/UART2）上根本不输出 NAV-PVT；现场表现为
//   ubxcfg status 全 OK 而 ubxstat 的 pvt 恒为 0。这里对全部端口同速率。
/**
 * @brief 下发 CFG-MSG（8 字节版）：设置某消息在全部端口上的输出速率
 * @param[in] cls  消息 class（0x01=UBX NAV，0xF0=NMEA 句）
 * @param[in] id   消息 id
 * @param[in] rate 0=关闭；1..127=每 n 个历元输出一次
 * @note 旧实现只填 rateDDC，UART1/UART2 留 0——接收机照样回 ACK，但串口上
 *       根本不输出 NAV-PVT，现场表现为 ubxcfg status 全 OK 而 ubxstat 的
 *       pvt 恒为 0（2026-11-03 修）。现在四个端口写同一速率。
 */
static void _cfg_msg(Uart &serial, uint8_t cls, uint8_t id, uint8_t rate)
{
  uint8_t p[8] = {cls, id, rate, rate, rate, rate, 0, 0};
  _ubx_send(serial, 0x06, 0x01, p, 8);
}

// CFG-VALSET（0x06 0x8A，F10 SPG 6.00 / protocol 40 推荐路径）：
// payload = version(1B)=0, layers(1B)=RAM(0x01), reserved(2B), 然后重复
// [key(4B LE) + value(1B)]。按键 CFG_MSGOUT-UBX_NAV_PVT_UART1=0x20910007、
// CFG_MSGOUT-UBX_NAV_EOE_UART1=0x20910160（F10 SPG 6.00 ID §4.9.10）。
/**
 * @brief 用 CFG-VALSET 配置 NAV-PVT / NAV-EOE 在 UART1 的输出速率
 * @param[in] rate 输出速率（0=关闭）
 * @note ① F10 SPG 6.00 / protocol 40 的推荐路径，键值见 ID §4.9.10：
 *         CFG_MSGOUT_UBX_NAV_PVT_UART1=0x20910007、
 *         CFG_MSGOUT_UBX_NAV_EOE_UART1=0x20910160。
 *       ② layers=0x01 只写 RAM，掉电不保留，需 _cfg_save 固化。
 */
static void _cfg_valset_pvt_eoe_uart1(Uart &serial, uint8_t rate)
{
  uint8_t p[14] = {0x00, 0x01, 0x00, 0x00};
  auto put = [&p](uint8_t off, uint32_t key, uint8_t value) {
    p[off + 0] = static_cast<uint8_t>(key & 0xFFU);
    p[off + 1] = static_cast<uint8_t>((key >> 8) & 0xFFU);
    p[off + 2] = static_cast<uint8_t>((key >> 16) & 0xFFU);
    p[off + 3] = static_cast<uint8_t>((key >> 24) & 0xFFU);
    p[off + 4] = value;
  };
  put(4, 0x20910007U, rate);  // CFG_MSGOUT_UBX_NAV_PVT_UART1
  put(9, 0x20910160U, rate);  // CFG_MSGOUT_UBX_NAV_EOE_UART1
  _ubx_send(serial, 0x06, 0x8A, p, sizeof(p));
}

// CFG-VALSET 改 UART1 波特率（F10 文档路径）：key=CFG-UART1-BAUDRATE
// (0x40520001, U4)，value 为 baud（4B LE）。F10 接口说明没有 CFG-PRT，
// legacy CFG-PRT 虽回 ACK 但不改波特率；实测从 460800 切 921600 会断链。
/**
 * @brief 用 CFG-VALSET 改 UART1 波特率（F10 的推荐路径）
 * @param[in] baud 目标波特率
 * @note ① key=CFG-UART1-BAUDRATE 0x40520001（U4，小端 4 字节值）。
 *       ② F10 接口说明里没有 CFG-PRT；legacy CFG-PRT 虽回 ACK 但不改波特率，
 *         实测从 460800 切到 921600 会直接断链。
 */
static void _cfg_valset_baud_uart1(Uart &serial, uint32_t baud)
{
  uint8_t p[12] = {0x00, 0x01, 0x00, 0x00,
                   0x01, 0x00, 0x52, 0x40,  // 0x40520001 LE
                   0, 0, 0, 0};
  p[8] = static_cast<uint8_t>(baud & 0xFFU);
  p[9] = static_cast<uint8_t>((baud >> 8) & 0xFFU);
  p[10] = static_cast<uint8_t>((baud >> 16) & 0xFFU);
  p[11] = static_cast<uint8_t>((baud >> 24) & 0xFFU);
  _ubx_send(serial, 0x06, 0x8A, p, sizeof(p));
}

// CFG-NAV5（36B）读改写：mask 按需置位，只改指定字段（-1/0 表示不改）
//   dyn_model  动态模型：2=Airborne<2g(7)，4=Airborne<4g(8)；0=不改
//   fix_mode   定位模式：-1 不改，0=2D，1=3D，2=auto
//   min_elev   最低仰角：-1 不改（0-90°）
//   pdop_thresh pDOP 定位门限：-1 不改（单位 0.1，如 50=5.0）
// ★ 2026-08-09 修复：dynModel 在 payload[2]、mask 位在 payload[0]（bit0）——
//   旧实现把 dynModel 枚举值写进 mask 字节，Airborne 配置从未生效过。
// ★ 2026-11-18 修复：u-blox dynModel 枚举是 6=airborne<1g、7=airborne<2g、
//   8=airborne<4g；旧实现写 (dyn_model==2)?6:7，实际选到的是 <1g>/<2g>，
//   比目标低一档。高机动必须落到 8，否则接收机内部平滑延迟偏大。
static bool _cfg_nav5_rw(Uart &serial, int dyn_model, int fix_mode,
                         int min_elev, int pdop_thresh)
{
  using namespace ubxLayout;
  uint8_t cur[nav5::kPayloadBytes];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x24, cur, sizeof(cur), 500);
  if (n != nav5::kPayloadBytes)
  {
    return false;
  }
  // 字段偏移与 mask 位一律取自 ubxLayout::nav5（官方 Byte offset 列）。
  // 历史实现把 fixMode 写到 cur[4]、minElev 写到 cur[3]、pDop 写到 cur[16]
  // 且用了 bit14，三项全部落空——ACK 正常但接收机侧没变。
  uint16_t mask = getU2(cur, nav5::kMask);
  if (dyn_model > 0)
  {
    mask |= nav5::kMaskDyn;                                  // bit0
    cur[nav5::kDynModel] = (uint8_t)((dyn_model == 2) ? 7 : 8);  // 7=<2g, 8=<4g
  }
  if (fix_mode >= 0)
  {
    mask |= nav5::kMaskFixMode;                              // bit2 posFixMode
    cur[nav5::kFixMode] = (uint8_t)fix_mode;
  }
  if (min_elev >= 0)
  {
    mask |= nav5::kMaskMinElev;                              // bit1 minEl
    cur[nav5::kMinElev] = (uint8_t)min_elev;
  }
  if (pdop_thresh >= 0)
  {
    mask |= nav5::kMaskPDop;                                 // bit4 posMask
    putU2(cur, nav5::kPDop, (uint16_t)pdop_thresh);          // 单位 0.1
  }
  putU2(cur, nav5::kMask, mask);
  _ubx_send(serial, 0x06, 0x24, cur, nav5::kPayloadBytes);
  return _ubx_wait_ack(serial, 0x06, 0x24, kAckTimeoutMs);
}

// CFG-GNSS（星座配置）——读改写：轮询当前 → 按掩码改各星座使能位 → 回写
// gnss_mask: bit0 GPS(0) bit1 GLONASS(6) bit2 Galileo(2) bit3 BeiDou(3)
/**
 * @brief 读改写 CFG-GNSS：按掩码启停各导航星座
 * @param[in] gnss_mask bit0 GPS、bit1 GLONASS、bit2 Galileo、bit3 BeiDou
 * @return true=已生效或本来就是目标配置；false=接收机不支持 CFG-GNSS
 * @note ① 必须先轮询原配置再回写——直接构造 payload 会把未列入掩码的星座
 *         及其跟踪通道数清零。
 *       ② 未列入掩码的星座是显式关闭使能位，不是"保持原样"。
 *       ③ 已是目标配置时直接返回，不发冗余写入。
 */
static bool _cfg_gnss(Uart &serial, uint8_t gnss_mask)
{
  uint8_t cur[40];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x3E, cur, sizeof(cur), 500);
  if (n < 8)
  {
    return false;                      // 接收机不支持 CFG-GNSS（老型号）
  }
  const uint8_t num_blocks = cur[3];
  // 布局走 ubxCfgGnss 常量，不手写裸偏移（2026-11-19 曾两次写成 8）
  bool any_patched = false;
  for (uint8_t i = 0; i < num_blocks && ubxCfgGnss::blockFits(i, n); i++)
  {
    uint8_t *blk = cur + ubxCfgGnss::blockOffset(i);
    const uint8_t gnss_id = blk[ubxCfgGnss::kOffGnssId];
    const bool want = (gnss_id == 0 && (gnss_mask & 0x01)) ||
                      (gnss_id == 6 && (gnss_mask & 0x02)) ||
                      (gnss_id == 2 && (gnss_mask & 0x04)) ||
                      (gnss_id == 3 && (gnss_mask & 0x08));
    if (gnss_id == 0 || gnss_id == 6 || gnss_id == 2 || gnss_id == 3)
    {
      const uint8_t flags0 = blk[ubxCfgGnss::kOffFlags];
      const uint8_t new_flags0 =
          want ? (uint8_t)(flags0 | ubxCfgGnss::kFlagEnableBit)
               : (uint8_t)(flags0 & ~ubxCfgGnss::kFlagEnableBit);
      if (new_flags0 != flags0)
      {
        blk[ubxCfgGnss::kOffFlags] = new_flags0;
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

/**
 * @brief 关闭 SBAS 星座（国内无覆盖）
 * @return true=已生效或无需改动
 * @note ★2026-11-19 改正：原来走 CFG-GNSS(0x06 0x3E) 改 flags。官方手册明确
 *       CFG-GNSS 在协议版本 >23.01 已废弃，要求改用 CFG-VALSET 操作配置项；
 *       F9P HPG 1.32 正属此列。实测废弃接口对使能位只有部分生效（BDS 能关
 *       不能开、GPS 关不掉），就是走错接口的表现。现改用
 *       CFG-SIGNAL-SBAS_ENA(0x10310020) 走 VALSET/VALGET。
 *
 *       依据（F9 HP 1.32 Interface Description）：
 *       - CFG-GNSS Comment：「deprecated in protocol versions greater than
 *         23.01. Use UBX-CFG-VALSET/VALGET/VALDEL instead」；
 *       - 字段映射表：UBX-CFG-GNSS.gnssId → CFG-SIGNAL-GPS_ENA / SBAS_ENA /
 *         BDS_ENA / QZSS_ENA / GLO_ENA；
 *       - CFG-GNSS 的配置约束同样适用于新接口：至少一个主星座启用、每个启用
 *         的主星座 maxTrkCh ≥ 4、通道总和不超硬件——所以只动 SBAS 这一路。
 *
 *       SBAS 的四套系统（EGNOS 欧洲、WAAS 美国、MSAS 日本、GAGAN 印度）都
 *       不覆盖中国，3 个跟踪通道白占；关掉后这 3 个通道回到总预算供其他星座
 *       动态调度。SBAS 非"主星座"，不受"至少启用一个主星座"约束。
 */
static bool _cfg_gnss_optimize(Uart &serial)
{
  uint8_t en = 0xFF;
  if (_valget_l(serial, &kSignalEnableKeys[1], 1, &en))
  {
    if (en == 0)
    {
      return true;                     // SBAS 已是关闭状态
    }
  }
  const uint8_t want = 0;
  return _valset_l(serial, &kSignalEnableKeys[1], &want, 1);
}

/**
 * @brief 使能 CFG-SBAS（星基增强系统）
 * @return true=接收机回 ACK
 * @note 国内 SBAS 几何覆盖有限，本条主要为兼容多星座接收机的默认行为，
 *       不代表定位精度会因此提升。
 */
static bool _cfg_sbas(Uart &serial)
{
  uint8_t p[8] = {1, 0, 0, 0, 0, 0, 0, 0};   // enabled=1
  _ubx_send(serial, 0x06, 0x16, p, 8);
  return _ubx_wait_ack(serial, 0x06, 0x16, kAckTimeoutMs);
}

// CFG-CFG（13B）：固化当前配置到接收机非易失存储
/**
 * @brief 下发 CFG-CFG 把 RAM 配置固化进接收机非易失存储
 * @note ★2026-08-10：掩码 0x1F→0x3F 补上 senConf(bit5)。原来的 0x1F 不含
 *       senConf，CFG-GNSS 的星座配置固化不进去；而"已固化"快路径又不再重配，
 *       于是掉电后星座配置丢失且再也回不来。
 */
static void _cfg_save(Uart &serial)
{
  uint8_t p[13] = {0};
  // ★ 2026-08-10 全面审查修复：0x1F→0x3F 补 senConf(bit5)——原 0x1F 不含
  //   senConf，CFG-GNSS 星座配置无法固化（掉电丢失且"已固化"快路径不再重配）
  p[4] = 0x3F;
  _ubx_send(serial, 0x06, 0x09, p, 13);
}

// ---------------- 回读校验：CFG-PRT 轮询 ----------------
/**
 * @brief 回读 CFG-PRT 并比对波特率
 * @param[in] expected_baud 期望波特率
 * @return true=回读值与期望一致
 * @note ACK 只证明"命令被接收"，不证明"配置生效"；本函数才是配置落地的判据，
 *       ubxcfg status 的 cfg_ok 就取自这里。
 */
static bool _ubx_verify_prt(Uart &serial, uint32_t expected_baud)
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

/**
 * @brief 开机快路径：验证接收机是否已是目标配置（不回写任何配置）
 * @param[in]  baud             期望波特率
 * @param[in]  expect_dyn       期望 dynModel（7=<2g>、8=<4g>；0=不检查）
 * @param[in]  require_sbas_off true=要求 SBAS 星座处于关闭状态
 * @param[out] dyn_readback     回读的 dynModel（0xFF=未读到；可为 nullptr）
 * @return true=PVT/EOE 都在出、星座与 dynModel 都匹配 → 可跳过全量配置
 * @note 判据只验现象、不逐项验配置：
 *       ① 目标波特率下 400 ms 内见到 ≥3 个 NAV-PVT 与 ≥3 个 NAV-EOE
 *          —— 波特率、CFG-RATE、CFG-MSG、VALSET 只要有一项错都不会有这个现象；
 *       ② CFG-NAV5 回读 payload[2] 与期望 dynModel 一致 —— 动态模型没被改过；
 *       ③ CFG-GNSS 回读里 SBAS 使能位为 0 —— 星座改动也纳入校验，否则
 *          「关 SBAS」这类改动会被快路径整段跳过、永远下发不下去。
 *       帧头计数不校验 payload/checksum：目的只是确认"这类消息在持续输出"，
 *       10 Hz 下连续 3 帧都校验失败的概率可忽略。
 *       任一项不过即返回 false，由调用方走全量配置（换接收机/被擦除/被改配置
 *       都会落到这条路，无需人工干预）。
 */
static bool _quick_verify_configured(Uart &serial, uint32_t baud, uint8_t expect_dyn,
                                     bool require_sbas_off, uint8_t *dyn_readback)
{
  (void)ubxGnssSerialBegin(serial, baud);
  const uint32_t t0 = millis();
  uint8_t state = 0, cls = 0, id = 0;
  uint16_t len = 0, remaining = 0;
  uint32_t pvt = 0, eoe = 0;
  while (millis() - t0 < kQuickVerifyMs)
  {
    while (serial.available())
    {
      const uint8_t b = serial.read();
      switch (state)
      {
        case 0: if (b == 0xB5) state = 1; break;
        case 1: state = (b == 0x62) ? 2 : 0; break;
        case 2: cls = b; state = 3; break;
        case 3: id = b; state = 4; break;
        case 4: len = b; state = 5; break;
        case 5:
          len = (uint16_t)(len | ((uint16_t)b << 8));
          if (len > 512) { state = 0; break; }
          if (cls == 0x01 && id == 0x07) ++pvt;
          else if (cls == 0x01 && id == 0x61) ++eoe;
          remaining = len;
          state = 6;
          break;
        case 6:
          if (remaining > 0) { --remaining; }
          else { state = 7; }        // 当前字节是校验和 A
          break;
        default: state = 0; break;    // 校验和 B，帧结束
      }
    }
    delay(1);
  }
  while (serial.available() > 0)
  {
    (void)serial.read();             // 清掉监听期间积压，避免污染后续解析
  }

  if (pvt < kQuickMinFrames || eoe < kQuickMinFrames)
  {
    return false;
  }
  // 星座与协议掩码校验（★2026-11-19）：SBAS 关闭、UART1 不收发 NMEA。
  // 不验这几项时，这类改动会被快路径整段跳过——配置永远下发不下去
  // （CFG-PRT 的 in/outProtoMask 实测都不生效，就是这里的证据）。
  // 三项都是 L 型，一次 CFG-VALGET 读完，约 50–80 ms。
  if (require_sbas_off)
  {
    const uint32_t keys[3] = {
        0x10310020U,  // CFG-SIGNAL-SBAS_ENA
        0x10730002U,  // CFG-UART1INPROT-NMEA
        0x10740002U,  // CFG-UART1OUTPROT-NMEA
    };
    uint8_t vals[3] = {0xFF, 0xFF, 0xFF};
    if (!_valget_l(serial, keys, 3, vals))
    {
      return false;                  // 读不到配置就别跳过
    }
    if (vals[0] != 0 || vals[1] != 0 || vals[2] != 0)
    {
      return false;                  // 有任一项不符 → 需全量配置
    }
  }
  if (expect_dyn == 0)
  {
    return true;                     // 不检查动态模型
  }
  uint8_t rb[36];
  if (_ubx_poll_payload(serial, 0x06, 0x24, rb, sizeof(rb), 500) != 36)
  {
    return false;
  }
  if (dyn_readback != nullptr)
  {
    *dyn_readback = rb[ubxLayout::nav5::kDynModel];
  }
  return rb[ubxLayout::nav5::kDynModel] == expect_dyn;
}

/**
 * @brief 读回接收机 CFG-GNSS 星座配置（只读，不改任何配置）
 * @param[out] payload 原始 payload（布局见 ubx_config.h 声明）
 * @param[in]  cap     缓冲容量
 * @param[out] len     实际长度（0 = 失败）
 * @return true=读回成功
 * @note 只解析到"能打印"的程度，不改 constellation；要改走 _cfg_gnss。
 */
bool ubxQueryGnssConfig(Uart &serial, uint8_t *payload, uint16_t cap, uint16_t *len)
{
  if (payload == nullptr || len == nullptr || cap < 8)
  {
    return false;
  }
  *len = 0;
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x3E, payload, cap, 500);
  if (n < 8)
  {
    return false;
  }
  *len = n;
  return true;
}

/**
 * @brief 单独设置某个星座的 maxTrkCh（CFG-GNSS 读改写）
 * @note 用于实测 F9P 对通道预算的钳制规则：ACK 只证明帧合法，
 *       是否被采纳要用 ubxQueryGnssConfig 回读确认。
 */
bool ubxSetGnssMaxChannel(Uart &serial, uint8_t gnss_id, uint8_t max_trk)
{
  uint8_t cur[64];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x3E, cur, sizeof(cur), 500);
  if (n < 8)
  {
    return false;
  }
  const uint8_t num_blocks = cur[3];
  bool found = false;
  for (uint8_t i = 0; i < num_blocks && ubxCfgGnss::blockFits(i, n); ++i)
  {
    uint8_t *blk = cur + ubxCfgGnss::blockOffset(i);
    if (blk[ubxCfgGnss::kOffGnssId] == gnss_id)
    {
      blk[ubxCfgGnss::kOffMaxTrkCh] = max_trk;
      found = true;
      break;
    }
  }
  if (!found)
  {
    return false;
  }
  _ubx_send(serial, 0x06, 0x3E, cur, n);
  return _ubx_wait_ack(serial, 0x06, 0x3E, kAckTimeoutMs);
}

/**
 * @brief 读回六星座使能状态（走 CFG-VALGET 读 CFG-SIGNAL-*_ENA）
 */
bool ubxQuerySignalEnable(Uart &serial, uint8_t enabled[6])
{
  if (enabled == nullptr)
  {
    return false;
  }
  return _valget_l(serial, kSignalEnableKeys, 6, enabled);
}

/**
 * @brief 设置某个星座的使能（走 CFG-VALSET 写 CFG-SIGNAL-*_ENA）
 * @note ★2026-11-19：改用配置数据库接口。CFG-GNSS 已废弃，且实测对使能位
 *       只有部分生效（BDS 能关不能开、GPS 关不掉）。
 */
bool ubxSetGnssEnable(Uart &serial, uint8_t gnss_id, bool enable)
{
  int idx = -1;
  switch (gnss_id)
  {
    case 0: idx = 0; break;   // GPS
    case 1: idx = 1; break;   // SBAS
    case 2: idx = 2; break;   // Galilean
    case 3: idx = 3; break;   // BeiDou
    case 5: idx = 4; break;   // QZSS
    case 6: idx = 5; break;   // GLONASS
    default: return false;    // IMES/NavIC 无对应 CFG-SIGNAL 键
  }
  const uint8_t value = enable ? 1 : 0;
  return _valset_l(serial, &kSignalEnableKeys[idx], &value, 1);
}

/**
 * @brief 读回 CFG-NAV5 的关键字段（只读）
 * @note 字段偏移全部取自 ubxLayout::nav5，与写入路径共用同一份常量，
 *       避免"写得对、读得错"或反过来的错配。
 */
bool ubxQueryNav5Config(Uart &serial, UbxNav5Status *status)
{
  if (status == nullptr)
  {
    return false;
  }
  status->valid = false;
  uint8_t rb[ubxLayout::nav5::kPayloadBytes];
  const uint16_t n = _ubx_poll_payload(serial, 0x06, 0x24, rb, sizeof(rb), 500);
  if (n != ubxLayout::nav5::kPayloadBytes)
  {
    return false;
  }
  status->dyn_model = rb[ubxLayout::nav5::kDynModel];
  status->fix_mode = rb[ubxLayout::nav5::kFixMode];
  status->min_elev = rb[ubxLayout::nav5::kMinElev];
  status->pdop = ubxLayout::getU2(rb, ubxLayout::nav5::kPDop);
  status->valid = true;
  return true;
}

/**
 * @brief 用 CFG-VALGET 读回单个配置项（L/U1 单字节型）
 */
bool ubxValGetByte(Uart &serial, uint32_t key, uint8_t *value)
{
  if (value == nullptr)
  {
    return false;
  }
  return _valget_l(serial, &key, 1, value);
}

/**
 * @brief 用 CFG-VALGET 读回单个 U2 型配置项
 * @note VALGET 响应里的值宽度由键的类型决定，U2 占 2 字节，所以响应长度是
 *       4（头部）+ 4（键）+ 2（值）= 10，不能复用 _valget_l（它按 1 字节取）。
 */
bool ubxValGetU2(Uart &serial, uint32_t key, uint16_t *value)
{
  if (value == nullptr)
  {
    return false;
  }
  uint8_t req[4 + 4];
  req[0] = 0x00;
  req[1] = 0x00;                 // RAM
  req[2] = 0x00;
  req[3] = 0x00;
  ubxLayout::putU4(req, 4, key);
  uint8_t resp[16];
  const uint16_t n = _ubx_request_payload(serial, 0x06, 0x8B, req, sizeof(req),
                                          resp, sizeof(resp), 500);
  if (n < 10 || ubxLayout::getU4(resp, 4) != key)
  {
    return false;
  }
  *value = ubxLayout::getU2(resp, 8);
  return true;
}

/**
 * @brief 用 CFG-VALSET 写入单个 L 型配置项
 */
bool ubxValSetByte(Uart &serial, uint32_t key, uint8_t value)
{
  return _valset_l(serial, &key, &value, 1);
}

// ============================================================
//  主入口
// ============================================================
/**
 * @brief 接收机自动配置主入口：探测波特率 → 逐项下发配置 → 回读校验
 * @param[in] opt 配置选项；nullptr 用 kUbxDefaultCfg
 * @return 各项结果与人读诊断（字段见 UbxCfgResult；nav5_dyn_readback 为 0xFF
 *         表示 CFG-NAV5 回读失败）
 * @pre  GNSS 串口已由 ubxGnssSerialBegin() 打开；本函数内部会按探测结果重开
 * @note ① 波特率扫描顺序：目标波特率优先，命中已固化接收机 300 ms 快路径；
 *         其余按 921600→9600 降序试，命中即停。
 *       ② 探测阶段用错波特率会产生大量帧错误，F10 手册 §3.2 p.21 规定
 *         1 s 内 >100 个帧错误会关闭 UART RX 约 1 s；这里探测后固定等待
 *         1200 ms 并清积压，否则后续配置会被整体丢弃（实测 ack=0、
 *         rate/msg/val 全 FAIL）。
 *       ③ 配置成功不等于生效——写后一律回读（PRT 波特率、NAV5 dynModel），
 *         回读值才是判据。
 */
UbxCfgResult ubxAutoConfig(Uart &serial, const UbxConfigOptions *opt)
{
  const UbxConfigOptions &o = (opt != nullptr) ? *opt : kUbxDefaultCfg;
  UbxCfgResult r = {};
  r.nav5_dyn_readback = 0xFF;   // 未读到的哨兵值（0 是合法的 portable 档位）
  memset(&s_last_cfg, 0, sizeof(s_last_cfg));

  // 0. 开机快路径（★2026-11-19）：接收机已 CFG-CFG 固化且未被改动时，
  //    直接验证「PVT/EOE 在出 + dynModel 匹配」就返回，不重发任何配置帧。
  //    省掉 1.2 s 防帧错误等待与约 20 帧配置往返（原全量流程约 1.5–2 s）。
  //    验证不过（换接收机 / 被擦除 / 被别人改过配置）自动落回全量流程，
  //    不是"信任接收机"，而是"验证它的当前状态"。
  {
    const uint8_t expect_dyn =
        (o.airborne_g == 2) ? 7 : ((o.airborne_g == 4) ? 8 : 0);
    if (_quick_verify_configured(serial, o.target_baud, expect_dyn,
                                 o.gnss_optimize, &r.nav5_dyn_readback))
    {
      r.detected = true;
      r.found_baud = o.target_baud;
      r.prt_ok = true;
      r.rate_ok = true;
      r.msg_pvt_ok = true;
      r.msg_eoe_ok = true;
      r.msg_rtcm_ok = true;
      r.valset_ok = true;
      r.nav5_ok = true;
      r.gnss_ok = (o.gnss_mask == 0);
      r.sbas_ok = !o.sbas_enable;
      r.save_ok = true;
      r.verify_ok = true;
      r.fast_path = true;
      s_last_cfg = r;
      return r;
    }
  }

  // 1. 波特率探测：目标波特率优先（已固化接收机 300ms 快路径）
  const uint32_t scan_order[8] = {o.target_baud, 921600U, 460800U,
                                  230400U, 115200U, 57600U, 38400U, 9600U};
  for (uint8_t i = 0; i < 8; i++)
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

  // F10 手册 §3.2 p.21：1 s 内 >100 个帧错误会关闭 UART RX 约 1 s。波特率
  // 探测阶段的错误波特率会触发该保护，若立刻发配置会被整体丢弃（实测
  // ack=0、rate/msg/val 全 FAIL）。检测到后先等保护窗口结束并清积压。
  delay(1200);
  while (serial.available() > 0)
  {
    (void)serial.read();
  }

  // 已在目标波特率 ≠ 已正确配置：接收机可能只固化了波特率，却缺
  // NAV-PVT/NAV-EOE 或速率不对，此时跳过 CFG-MSG 会永久 0 epoch。
  // 因此只跳过 CFG-PRT（改波特率），其余幂等配置每次启动都重发。
  const bool already_at_target = (r.found_baud == o.target_baud);

  // 2. 先用 F10 文档键 CFG-UART1-BAUDRATE 通过 VALSET 切目标波特率。
  //    不能先发 legacy CFG-PRT：部分 F10 固件收到 CFG-PRT 会重启接口，把
  //    紧随其后的 VALSET 吞掉（实测 ack=0、随后断链）。VALSET 成功即跳过
  //    CFG-PRT，失败才回退老路径（M8N 等）。
  if (!already_at_target)
  {
    _cfg_valset_baud_uart1(serial, o.target_baud);
    r.cfg_frames_sent++;
    bool baud_valset_ok = _ubx_wait_ack(serial, 0x06, 0x8A, kAckTimeoutMs);
    if (baud_valset_ok)
    {
      r.cfg_acks_received++;
    }
    else
    {
      _cfg_prt(serial, r.found_baud, o.nmea_out);
      r.cfg_frames_sent++;
      if (_ubx_wait_ack(serial, 0x06, 0x00, kAckTimeoutMs))
      {
        r.cfg_acks_received++;
      }
    }
    // 只有确认接收机收到切速命令才切 MCU 波特率；输入链路故障（实测
    // CFG-RST/CFG-MSG/CFG-VALSET 全无 ACK）时留在已检测波特率继续收数据，
    // 不能盲目切到目标速率把整条 GNSS 链路弄断。
    const bool baud_switch_confirmed =
        baud_valset_ok || (r.cfg_acks_received > 0);
    if (baud_switch_confirmed)
    {
      // F10 手册：改波特率后至少等约 100 ms，再按新速率通信。
      delay(120);
      (void)ubxGnssSerialBegin(serial, o.target_baud);
    }
    else
    {
      (void)ubxGnssSerialBegin(serial, r.found_baud);
    }
    while (serial.available() > 0)
    {
      (void)serial.read();
    }
  }
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

  // 3b. UART1 协议掩码（只能走 VALSET，见 _cfg_uart1_proto_masks 的说明）。
  //     放在 CFG-PRT 之后隔一步发：紧跟 CFG-PRT 的 VALSET 会被接收机重配
  //     UART 的动作丢掉（实测 readback 仍为 1）。
  _cfg_uart1_proto_masks(serial, o.nmea_out);
  r.cfg_frames_sent++;

  // 4. CFG-MSG：NAV-PVT + NAV-EOE
  _cfg_msg(serial, 0x01, 0x07, 1);
  r.cfg_frames_sent++;
  r.msg_pvt_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  if (r.msg_pvt_ok) r.cfg_acks_received++;

  _cfg_msg(serial, 0x01, 0x61, 1);
  r.cfg_frames_sent++;
  r.msg_eoe_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  if (r.msg_eoe_ok) r.cfg_acks_received++;

  // 4.2 RXM-RTCM：接收机把收到的差分改正数状态上报（0x02 0x32）。
  // 无 RTK 差分输入时接收机可能不回复，超时按 NA 处理、不阻断配置——
  // 一旦接入 RTK，这条是判断改正数是否真正到达的唯一通道。
  _cfg_msg(serial, 0x02, 0x32, 1);
  r.cfg_frames_sent++;
  r.msg_rtcm_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  if (r.msg_rtcm_ok) r.cfg_acks_received++;

  // 4.5 关闭接收机出厂默认的其余 UBX 消息（只留 NAV-PVT + NAV-EOE）。
  // ★ 2026-11-11 实测：不关时 Serial4 收 1822 B/s，而 PVT(92B)+EOE(8B)@8Hz 只需
  //   800 B/s —— 多出的约 56% 流量来自接收机默认开启的 NAV-* 系列。
  //   框架 uart.c 是 HANDLE 逐字节中断接收（每次 HAL_UART_Receive_IT(...,1)），
  //   921600 下每秒 9 万次中断，多余消息会持续抢占 UART 处理窗口，实测约 8.4%
  //   的历元 EOE 没有配到有效 PVT（ubxstat 的 eoe_no_pvt）。
  //   关掉后带宽减半，丢帧率随之下降；PVT 的 numSV 字段已含卫星数，不需要 NAV-SAT。
  //   注意：未开启的消息接收机可能直接 NAK/不回，超时只记录不阻断。
  {
    static const uint8_t kOffMsgs[][2] = {
        {0x01, 0x02}, // NAV-POSLLH
        {0x01, 0x03}, // NAV-STATUS
        {0x01, 0x04}, // NAV-DOP
        {0x01, 0x06}, // NAV-SOL
        {0x01, 0x12}, // NAV-VELNED
        {0x01, 0x21}, // NAV-TIMEUTC
        {0x01, 0x22}, // NAV-CLOCK
        {0x01, 0x30}, // NAV-SVINFO
        {0x01, 0x35}, // NAV-SAT
        {0x01, 0x43}, // NAV-SIG
    };
    for (size_t i = 0; i < sizeof(kOffMsgs) / sizeof(kOffMsgs[0]); ++i)
    {
      _cfg_msg(serial, kOffMsgs[i][0], kOffMsgs[i][1], 0);
      r.cfg_frames_sent++;
      (void)_ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
    }
  }

  // 5. CFG-VALSET（F10 SPG 6.00 / protocol 40）：CFG-MSG 已 deprecated，
  //    官方推荐按键配置。老接收机（M8N）不认此消息会 NAK/超时，不影响前面的
  //    CFG-MSG 兼容路径，因此失败只记录不阻断。
  _cfg_valset_pvt_eoe_uart1(serial, 1);
  r.cfg_frames_sent++;
  r.valset_ok = _ubx_wait_ack(serial, 0x06, 0x8A, kAckTimeoutMs);
  if (r.valset_ok) r.cfg_acks_received++;

  // 5. CFG-NAV5 动态模型（读改写；2026-08-09 修复 mask/dynModel 错位）
  if (o.airborne_g == 2 || o.airborne_g == 4)
  {
    r.cfg_frames_sent += 2;
    r.nav5_ok = _cfg_nav5_rw(serial, o.airborne_g, -1, -1, -1);
    if (r.nav5_ok) r.cfg_acks_received++;
    // ACK 只证明接收机收下了帧，不证明 dynModel 落到目标档位；写后回读 payload[2]。
    uint8_t nav5_rb[36];
    if (_ubx_poll_payload(serial, 0x06, 0x24, nav5_rb, sizeof(nav5_rb), 500) == 36)
    {
      r.nav5_dyn_readback = nav5_rb[2];
    }
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
  // 6b. 通道重排：关 SBAS、扩北斗（国内 SBAS 无覆盖，北斗通道偏少）
  if (o.gnss_optimize)
  {
    r.cfg_frames_sent += 2;
    r.gnss_ok = _cfg_gnss_optimize(serial) && r.gnss_ok;
    if (r.gnss_ok) r.cfg_acks_received++;
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
/**
 * @brief 下发 CFG-RST 让接收机做软件复位（不清非易失配置）
 * @note navBbrMask=0x02（保留 BBR 热启动数据）、resetMode=0x02（受控软件复位）。
 *       配置刚写完但没生效时，复位接收机比反复重写更可靠。
 */
void ubxResetReceiver(Uart &serial)
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
/**
 * @brief 从 MON-VER 的定长字段复制字符串并去掉尾部空格/CR
 * @param[in]  src 源字段起始地址
 * @param[in] n   源字段长度（定长，不以 NUL 结尾也可能）
 * @param[out] dst 目标缓冲（保证以 NUL 结尾）
 * @param[in]  cap 目标缓冲容量（含结尾 NUL）
 */
static void _copy_trim_monver(const uint8_t *src, size_t n, char *dst, size_t cap)
{
  if (dst == nullptr || cap == 0) return;
  size_t k = 0;
  for (size_t i = 0; i < n && k + 1 < cap; ++i)
  {
    const char c = static_cast<char>(src[i]);
    if (c == '\0') break;
    dst[k++] = c;
  }
  while (k > 0 && (dst[k - 1] == ' ' || dst[k - 1] == '\r')) --k;
  dst[k] = '\0';
}

bool ubxQueryMonVer(Uart &serial, char *sw, size_t sw_cap, char *hw, size_t hw_cap,
                    char *protver, size_t pv_cap, char *antsup, size_t ant_cap)
{
  // UBX-MON-VER payload: swVersion[30] + hwVersion[10] + 重复 extension[30]。
  uint8_t buf[300];
  const uint16_t n = _ubx_poll_payload(serial, 0x0A, 0x04, buf, sizeof(buf), 800);
  if (n < 40) return false;
  _copy_trim_monver(buf, 30, sw, sw_cap);
  _copy_trim_monver(buf + 30, 10, hw, hw_cap);
  if (protver != nullptr && pv_cap > 0) protver[0] = '\0';
  if (antsup != nullptr && ant_cap > 0) antsup[0] = '\0';
  for (uint16_t off = 40; off + 30 <= n; off += 30)
  {
    const uint8_t *e = buf + off;
    if (memcmp(e, "PROTVER=", 8) == 0)
      _copy_trim_monver(e + 8, 22, protver, pv_cap);
    else if (memcmp(e, "ANTSUPERV=", 10) == 0)
      _copy_trim_monver(e + 10, 20, antsup, ant_cap);
  }
  return true;
}

bool ubxMsgRateConfig(Uart &serial, uint8_t cls, uint8_t msg_id,
                      uint8_t rate)
{
  if (rate > 127)
  {
    return false;
  }
  // F10（protocol 40）文档路径：PVT/EOE 用 CFG-VALSET；两者同速率保证历元配对。
  // 同时保留 CFG-MSG 兼容 M8N 等老接收机；两条任一 ACK 即成功。
  bool valset_ok = false;
  if (cls == 0x01U && (msg_id == 0x07U || msg_id == 0x61U))
  {
    _cfg_valset_pvt_eoe_uart1(serial, rate);
    valset_ok = _ubx_wait_ack(serial, 0x06, 0x8A, kAckTimeoutMs);
  }
  _cfg_msg(serial, cls, msg_id, rate);
  const bool legacy_ok = _ubx_wait_ack(serial, 0x06, 0x01, kAckTimeoutMs);
  return valset_ok || legacy_ok;
}

// CFG-NMEA（0x06 0x17，20B V1）读改写公共骨架：轮询当前 → 改 offset → 回写 → 等 ACK
/**
 * @brief 读改写 CFG-NMEA 的单个字节字段（先轮询 20B 原值再回写）
 * @param[in] offset CFG-NMEA payload 内的字节偏移
 * @param[in] value  目标值
 * @return true=接收机回 ACK；false=未收到完整原配置（老固件/无响应）
 * @note 必须读改写：CFG-NMEA 的其余字段（版本、talker、滤波、兼容标志）
 *       直接构造 payload 会被清零。
 */
static bool _cfg_nmea_read_modify_write(Uart &serial, uint8_t offset,
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

/**
 * @brief 设置 NMEA 协议版本（CFG-NMEA offset 1）
 * @param[in] version 编码=(主<<4)|次，如 0x23=2.3、0x41=4.1、0x4A=4.10；0=不改
 * @return true=已生效或 version=0；false=读取/写入失败
 */
bool ubxNmeaVersionConfig(Uart &serial, uint8_t version)
{
  if (version == 0)
  {
    return true;  // 0 = 不改
  }
  return _cfg_nmea_read_modify_write(serial, 1, version);
}

// 主 talker ID（offset 8）：NMEA_TALKER_*（0=不覆盖接收机默认）
/**
 * @brief 设置 NMEA 主 talker ID（CFG-NMEA offset 8）
 * @param[in] talker_id NMEA_TALKER_*；0=不覆盖接收机默认
 * @return true=已生效或 talker_id=0；false=读取/写入失败
 */
bool ubxNmeaTalkerConfig(Uart &serial, uint8_t talker_id)
{
  if (talker_id == 0)
  {
    return true;  // 0 = 不改
  }
  return _cfg_nmea_read_modify_write(serial, 8, talker_id);
}

// NMEA 输出滤波掩码（offset 0）：NMEA_FILTER_*（0=不过滤，全量输出）
/**
 * @brief 设置 NMEA 输出滤波掩码（CFG-NMEA offset 0）
 * @param[in] filter_mask NMEA_FILTER_*；0=不过滤，全量输出
 * @return true=已生效；false=读取/写入失败
 */
bool ubxNmeaFilterConfig(Uart &serial, uint8_t filter_mask)
{
  return _cfg_nmea_read_modify_write(serial, 0, filter_mask);
}

/**
 * @brief 读改写 CFG-PRT 的输出协议掩码（不动波特率与串口模式）
 * @param[in] out_mask 1=UBX-only、2=NMEA-only、3=UBX+NMEA
 * @return true=接收机回 ACK；false=掩码非法或读取失败
 * @note 这里只改 outProtoMask；要改波特率用 _cfg_prt（它会重写整个 20B）。
 */
bool ubxProtoOutputConfig(Uart &serial, uint8_t out_mask)
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
bool ubxNav5Config(Uart &serial, int dyn_model, int fix_mode,
                   int min_elev, int pdop_thresh)
{
  return _cfg_nav5_rw(serial, dyn_model, fix_mode, min_elev, pdop_thresh);
}

/**
 * @brief 读改写 CFG-ITFM 的 CW 干扰检测开关
 * @param[in] enable true=开干扰检测（bit0），false=关
 * @return true=接收机回 ACK；false=读取的 payload 长度不对
 */
bool ubxItfmConfig(Uart &serial, bool enable)
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

/**
 * @brief 查询天线状态（CFG-ANT，纯读不改配置）
 * @param[out] out 4 字节：out[0..1]=config、out[2..3]=status
 * @return true=读到完整 4 字节；false=超时
 * @note status 低 2 位：0=INIT、1=UNKNOWN、2=OK、3=SHORT/OPEN
 *       （短路还是开路取决于 config 里启用了哪个检测）。
 *       没有源天线的大多数接收机恒回 UNKNOWN，不能据此判断天线好坏。
 */
bool ubxAntStatusQuery(Uart &serial, uint8_t out[4])
{
  const uint16_t n =
      _ubx_poll_payload(serial, 0x06, 0x13, out, 4, 500);
  return n == 4;
}
