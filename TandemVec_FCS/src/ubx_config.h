/**
 * @file    ubx_config.h
 * @module  固件/通信（GNSS 接收机 UBX 配置）
 * @brief   声明 u-blox 接收机自动配置（CFG-PRT/RATE/MSG/NAV5/GNSS/SBAS/CFG）接口
 * @details 输入：绑定到 Serial4 的接收机（出厂可能仍是 9600 + NMEA）；
 *          处理：波特率探测 → CFG-PRT 切 921600 与协议掩码 → CFG-RATE 10Hz →
 *                CFG-MSG 使能 NAV-PVT/NAV-EOE →（可选）CFG-NAV5 动态模型、CFG-GNSS 星座、
 *                CFG-SBAS、CFG-CFG 固化 → 回读 CFG-PRT 校验；
 *          输出：UbxCfgResult 各项成败（每步独立，失败不阻塞后续）。
 *          不变量：只发经典 CFG-* 消息（M8/M9/M10 兼容）；不改变解析库行为。
 * @theory  N/A（外设配置，非理论模型）
 * @ref     docs/reference/sparkfun-ublox-gnss（SparkFun u-blox GNSS Library v2.2.29 参考）、
 *          PROTOCOL-001 §二.3.1（0x30 GPS 帧的 FIX/SACC/VACC 语义取自 UBX NAV-PVT）
 * @frame   N/A（配置消息不含本项目坐标语义）
 * @units   波特率 bps、导航周期 ms、仰角/精度门限为协议整数单位（见各参数注释）
 * @note    Serial4 与 DETA100 共用——仅 `NavDataSource::INTERNAL` 路径才调用本模块；
 *          DETA100 模式下配置帧会把模块字节流打断。
 * @see     src/ubx_config.cpp、src/main.cpp（setup §1.6 调用点）、src/communication.cpp
 *          （DBG `ubxcfg`/`gpsproto` 命令）
 */
#pragma once
#include <Arduino.h>

// ============================================================
//  ubx_config.h — u-blox UBX 接收机自动配置
//
//  自实现（2026-08-09），参考 SparkFun u-blox GNSS Arduino Library v2.2.29
//  （docs/reference/sparkfun-ublox-gnss）。解决 BFS ublox 解析库"只解析不配置"：
//  出厂默认 NMEA/9600 的接收机插上即用——波特率自动探测 → CFG-PRT 切换 →
//  CFG-RATE 导航频率 → CFG-MSG 消息使能 →（可选）CFG-CFG 固化 → 回读校验。
// ============================================================

// 自动配置结果（DBG `ubxcfg status` 查询）
struct UbxCfgResult
{
  bool     detected;          // 探测到接收机（UBX 或 NMEA 响应）
  uint32_t found_baud;        // 探测到的当前波特率（0 = 未探测到）
  bool     prt_ok;            // CFG-PRT（端口/波特率/协议掩码）成功
  bool     rate_ok;           // CFG-RATE（导航频率）成功
  bool     msg_pvt_ok;        // CFG-MSG NAV-PVT 成功
  bool     msg_eoe_ok;        // CFG-MSG NAV-EOE 成功
  bool     msg_rtcm_ok;       // CFG-MSG RXM-RTCM 成功（改正数到达监视；无差分输入时超时不算失败）
  bool     valset_ok;         // CFG-VALSET（F10/M10 推荐路径，UART1 PVT+EOE）成功
  bool     nav5_ok;           // CFG-NAV5（动态模型，读改写）成功
  uint8_t  nav5_dyn_readback; // CFG-NAV5 写后回读的 dynModel（0xFF=未读到；7=<2g> 8=<4g>）
  bool     gnss_ok;           // CFG-GNSS（星座使能，读改写）成功
  bool     sbas_ok;           // CFG-SBAS（差分增强）成功
  bool     save_ok;           // CFG-CFG 固化闪存成功（未请求时 true）
  bool     verify_ok;         // 回读 CFG-PRT 校验成功
  uint32_t cfg_frames_sent;   // 发送的配置帧数
  uint32_t cfg_acks_received; // 收到的 ACK-ACK 数
  // ★2026-11-19：开机快路径命中标志。true=接收机已是目标配置，
  // 只跑了「测 PVT/EOE 是否在出 + 回读 dynModel」，未重发任何配置帧。
  // 此时上述 * _ok 字段全部置 true（那是"已验证"而非"刚配置成功"）。
  bool     fast_path;
};

// 配置选项（ubxAutoConfig 参数）
struct UbxConfigOptions
{
  uint32_t target_baud;      // 目标波特率（默认 921600）
  uint16_t nav_rate_ms;      // 导航测量周期 ms（100=10Hz；0=不改）
  uint8_t  airborne_g;       // 动态模型：0=不改，2=Airborne<2g（dynModel=7），4=Airborne<4g>（dynModel=8）
  uint8_t  gnss_mask;        // 星座使能掩码：bit0 GPS, bit1 GLONASS, bit2 Galileo, bit3 BeiDou（0=不改）
  bool     sbas_enable;      // 使能 SBAS 差分增强（EGNOS/WAAS）
  bool     nmea_out;         // false=UBX-only 输出；true=UBX+NMEA
  bool     persist;          // CFG-CFG 固化到接收机闪存
  // ★2026-11-19：星座通道重排（关 SBAS、扩北斗）。
  // SBAS 在国内无覆盖（EGNOS/WAAS/MSAS/GAGAN 都不覆盖中国），实测占 3 个跟踪通道；
  // 北斗在国内可用卫星最多而只分到 5 个通道。开启后按
  // GPS14 + GAL16 + BDS12 + QZSS4 + GLO12 = 58（trkUse=60）重排。
  bool     gnss_optimize;
  // ★2026-11-19：接收机输出滤波的 pDOP 门限（单位 0.1，0=不改）。
  // 手册（Integration Manual §4.9.3）：「A fix is only valid if it passes the
  // navigation output filters. In particular, both PDOP and accuracy values must
  // lie below the respective limits.」——超门限时定位解直接判无效，这是硬截断。
  // 飞控侧还有一层 GNSS_MAX_PDOP=3.0（只用于降级路径判定），主融合路径按
  // PDOP 动态缩放权重、不硬拒。本机实测接收机里残留 2.5，比飞控更严，
  // PDOP 2.5–3.0 的可用解被接收机先丢掉。改为 5.0，把判断权交给飞控。
  uint16_t pdop_thresh;
};

// 默认配置选项（main.cpp / DBG 使用）
static const UbxConfigOptions kUbxDefaultCfg = {
  /* target_baud: 目标波特率 */ 921600UL,
  /* nav_rate_ms: 导航周期 100ms=10Hz */ 100,
  /* airborne_g:  动态模型 4=Airborne<4g>（高机动，减少接收机内部位置/速度平滑延迟） */ 4,
  /* gnss_mask:   星座掩码 0=不改 */ 0,
  /* sbas_enable: SBAS 差分增强 */ false,
  /* nmea_out:    false=仅 UBX 输出 */ false,
  /* persist:     CFG-CFG 固化到接收机闪存 */ false,
  /* gnss_optimize: 关 SBAS + 扩北斗通道（国内 SBAS 无覆盖） */ true,
  /* pdop_thresh: 接收机输出滤波 pDOP 门限，50=5.0（放宽，交给飞控判断） */ 50,
};

/**
 * @brief 运行 UBX 接收机自动配置（阻塞，约 1-4s）
 *
 * 流程：波特率探测（目标优先，已固化走快路径）→ CFG-PRT（波特率/协议掩码）
 * → CFG-RATE → CFG-MSG（NAV-PVT/NAV-EOE）→ CFG-NAV5（动态模型，读改写）
 * → CFG-GNSS（星座，读改写）→ CFG-SBAS → CFG-CFG（可选固化）→ 回读校验。
 * 每步独立成败，失败不阻塞后续；无接收机响应时保持现状。
 *
 * @param serial 绑定的串口（本机 Serial4，与 DETA100 共用——仅 INTERNAL 路径调用）
 * @param opt    配置选项（见 UbxConfigOptions；传 nullptr = kUbxDefaultCfg）
 */
UbxCfgResult ubxAutoConfig(Uart &serial, const UbxConfigOptions *opt);

/**
 * @brief GNSS 串口统一起始入口：设置波特率后恢复接收 FIFO 与 UART4 高优先级。
 * @details `Uart::begin()` 会重新配置外设并清除 FIFOEN，还会恢复框架默认 IRQ 优先级；
 *          所有 GNSS 探测/切速都必须经此入口，不能只在上电开头配置一次。
 * @return true=HAL FIFO 配置、接收重启与读回核对均成功；false=至少一步失败。
 */
bool ubxGnssSerialBegin(Uart &serial, uint32_t baud);

/** @brief GNSS 串口接收配置失败累计次数（DBG `ubxstat` 显示）。 */
uint32_t ubxGnssSerialConfigFailCount();

/**
 * @brief UBX 配置消息的 payload 字段偏移（一手来源：u-blox F9 HPG 1.32
 *        Interface Description UBX-22008968 §UBX-CFG-*）
 * @details 全部按官方 "Byte offset" 列抄录。任何 CFG-* 组帧/解析都必须用
 *          这里的常量，禁止在调用点手写整数偏移——2026-11-19 曾出现
 *          CFG-PRT in/outProtoMask、CFG-RATE timeRef、CFG-NAV5 minElev/pDop
 *          三处偏移写错，功能"看似正常"但配置根本没落到目标字段。
 */
namespace ubxLayout
{
/** @brief 小端 U2 写入 */
inline void putU2(uint8_t *p, uint16_t off, uint16_t v)
{
  p[off] = static_cast<uint8_t>(v & 0xFFU);
  p[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
}
/** @brief 小端 U4 写入 */
inline void putU4(uint8_t *p, uint16_t off, uint32_t v)
{
  p[off] = static_cast<uint8_t>(v & 0xFFU);
  p[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFFU);
  p[off + 2] = static_cast<uint8_t>((v >> 16) & 0xFFU);
  p[off + 3] = static_cast<uint8_t>((v >> 24) & 0xFFU);
}
/** @brief 小端 U2 读取 */
inline uint16_t getU2(const uint8_t *p, uint16_t off)
{
  return static_cast<uint16_t>(p[off] | (static_cast<uint16_t>(p[off + 1]) << 8));
}
/** @brief 小端 U4 读取 */
inline uint32_t getU4(const uint8_t *p, uint16_t off)
{
  return static_cast<uint32_t>(p[off]) |
         (static_cast<uint32_t>(p[off + 1]) << 8) |
         (static_cast<uint32_t>(p[off + 2]) << 16) |
         (static_cast<uint32_t>(p[off + 3]) << 24);
}

/** @brief UBX-CFG-PRT（UART 口，payload 20 字节） */
namespace prt
{
constexpr uint16_t kPortId = 0;        // U1
constexpr uint16_t kTxReady = 2;       // X2
constexpr uint16_t kMode = 4;          // X4
constexpr uint16_t kBaudRate = 8;      // U4
constexpr uint16_t kInProtoMask = 12;  // X2
constexpr uint16_t kOutProtoMask = 14; // X2
constexpr uint16_t kFlags = 16;        // X2
constexpr uint16_t kPayloadBytes = 20;
constexpr uint32_t kMode8N1 = 0x000008D0UL;
}  // namespace prt

/** @brief UBX-CFG-RATE（payload 6 字节） */
namespace rate
{
constexpr uint16_t kMeasRate = 0;  // U2，ms
constexpr uint16_t kNavRate = 2;   // U2，cycles
constexpr uint16_t kTimeRef = 4;   // U2，0=UTC 1=GPS 2=GLO 3=BDS 4=GAL
constexpr uint16_t kPayloadBytes = 6;
constexpr uint16_t kTimeRefGps = 1;
}  // namespace rate

/** @brief UBX-CFG-NAV5（payload 36 字节） */
namespace nav5
{
constexpr uint16_t kMask = 0;       // X2，bit0=dyn  bit1=fixMode  bit4=minElev  bit2=pDop
constexpr uint16_t kDynModel = 2;   // E1
constexpr uint16_t kFixMode = 3;    // E1
constexpr uint16_t kMinElev = 12;   // I1，deg
constexpr uint16_t kPDop = 14;      // U2，0.1
constexpr uint16_t kPayloadBytes = 36;
constexpr uint16_t kMaskDyn = 0x0001;      // bit0 dyn
constexpr uint16_t kMaskMinElev = 0x0002;  // bit1 minEl
constexpr uint16_t kMaskFixMode = 0x0004;  // bit2 posFixMode
constexpr uint16_t kMaskPDop = 0x0010;     // bit4 posMask
}  // namespace nav5

/** @brief UBX-CFG-MSG（payload 8 字节） */
namespace msg
{
constexpr uint16_t kMsgClass = 0;
constexpr uint16_t kMsgId = 1;
constexpr uint16_t kRateUart1 = 3;
constexpr uint16_t kPayloadBytes = 8;
}  // namespace msg
}  // namespace ubxLayout

/**
 * @brief CFG-GNSS payload 字节布局（UBX 接口描述 UBX-CFG-GNSS）
 * @details 前 4 字节是 version/numTrkChHw/numTrkChUse/numConfigBlocks，
 *          配置块从偏移 4 开始，每块 8 字节：
 *          gnssId(0) resTrkCh(1) maxTrkCh(2) reserved(3) flags[4](4..7)，
 *          flags 的 bit0 是使能位。
 * @note 所有 CFG-GNSS 解析必须经由这里的常量与函数，不要在调用点手写裸偏移。
 *       2026-11-19 曾两次把块起点写成 8（应为 4），导致星座配置从未落到接收机。
 */
namespace ubxCfgGnss
{
constexpr uint16_t kHeaderBytes = 4;   // version + numTrkChHw + numTrkChUse + numConfigBlocks
constexpr uint16_t kBlockBytes = 8;    // 每个星座配置块
constexpr uint16_t kFlagEnableBit = 0x01U;

/** @brief 第 i 个配置块在 payload 内的起始偏移 */
constexpr uint16_t blockOffset(uint16_t i) { return kHeaderBytes + i * kBlockBytes; }

/** @brief 长度为 len 的 payload 是否完整包含第 i 个配置块 */
constexpr bool blockFits(uint16_t i, uint16_t len)
{
  return static_cast<uint32_t>(blockOffset(i)) + kBlockBytes <= len;
}

/** @brief 配置块内各字段相对块起点的偏移 */
constexpr uint16_t kOffGnssId = 0;
constexpr uint16_t kOffResTrkCh = 1;
constexpr uint16_t kOffMaxTrkCh = 2;
constexpr uint16_t kOffFlags = 4;
}  // namespace ubxCfgGnss

/** @brief 上次配置结果（DBG `ubxcfg status` 读取） */
const UbxCfgResult &ubxLastCfgResult();

/**
 * @brief 读回接收机 CFG-GNSS（星座使能与通道配置），不改任何配置
 * @param[out] payload 原始 CFG-GNSS payload：前 4 字节为
 *             version/numTrkChHw/numTrkChUse/numConfigBlocks，
 *             之后每 8 字节一块（gnssId/resTrkCh/maxTrkCh/reserved/flags[4]，
 *             flags bit0 = 使能）
 * @param[in]  cap     payload 缓冲容量
 * @param[out] len     实际读回长度（0 = 失败）
 * @return true=读回成功
 * @note 用于确认接收机实际启用哪些星座——它决定 PVT 更新率上限
 *       （F9P 数据手册 Table 1：四星座 8 Hz、三星座 10 Hz、双星座 15 Hz）
 *       以及定位几何/收敛速度。
 */
bool ubxQueryGnssConfig(Uart &serial, uint8_t *payload, uint16_t cap, uint16_t *len);

/**
 * @brief 读回六星座使能状态（走 CFG-VALGET 读 CFG-SIGNAL-*_ENA）
 * @param[out] enabled 长度 6：0=GPS 1=SBAS 2=Galileo 3=BeiDou 4=QZSS 5=GLONASS
 * @return true=全部读回成功
 * @note 这是新协议下的正确读法。CFG-GNSS(0x06 0x3E) 在协议版本 >23.01 已废弃，
 *       官方要求改用 CFG-VALGET/VALSET；F9P HPG 1.32 属此列。
 */
bool ubxQuerySignalEnable(Uart &serial, uint8_t enabled[6]);

/**
 * @brief 设置某个星座的使能（走 CFG-VALSET 写 CFG-SIGNAL-*_ENA）
 * @param[in] gnss_id 星座 id：0=GPS 1=SBAS 2=GAL 3=BDS 5=QZSS 6=GLO
 * @param[in] enable  目标使能状态
 * @return true=接收机回 ACK（是否被采纳须用 ubxQuerySignalEnable 回读）
 * @note 沿用 CFG-GNSS 的约束：至少一个主星座必须启用；每个启用的主星座
 *       maxTrkCh 不得低于 4；GPS 与 QZSS 建议同开同关（避免互相关）。
 *       ACK 不等于采纳——实测 SBAS 使能位写入会回 ACK 但不生效。
 */
bool ubxSetGnssEnable(Uart &serial, uint8_t gnss_id, bool enable);

/** @brief CFG-NAV5 关键字段读回结果（DBG `ubxcfg nav5 show`） */
struct UbxNav5Status
{
  uint8_t  dyn_model;   // 动态平台模型（6=<1g 7=<2g 8=<4g 等）
  uint8_t  fix_mode;    // 定位模式（1=2D 2=3D 3=auto）
  uint8_t  min_elev;    // 最低仰角 [deg]
  uint16_t pdop;        // pDOP 门限（单位 0.1）
  bool     valid;       // true=读回成功
};

/**
 * @brief 读回 CFG-NAV5 的关键字段（只读）
 * @return true=读回成功（status.valid 同步置位）
 * @note 用于证明 NAV5 写入真的落到目标字段，而不是只看 ACK。
 *       字段偏移全部取自 ubxLayout::nav5。
 */
bool ubxQueryNav5Config(Uart &serial, UbxNav5Status *status);

/**
 * @brief 用 CFG-VALGET 读回单个配置项（L/U1 单字节型）
 * @param[in]  key   配置键 ID（如 CFG-NAVSPG-DYNMODEL 0x20110021）
 * @param[out] value 读回值
 * @return true=读回成功
 * @note 用于交叉验证 legacy CFG-* 消息写入的值与新接口读回是否一致。
 *       F9P 的 PROTVER=34.00，legacy CFG-PRT/RATE/MSG/NAV5/GNSS/SBAS 官方
 *       均已废弃（>23.01），实测仍向后兼容，但存在字段部分失效的先例
 *       （CFG-GNSS 的使能位就是这样）。
 */
bool ubxValGetByte(Uart &serial, uint32_t key, uint8_t *value);

/**
 * @brief 用 CFG-VALGET 读回单个 U2 型配置项
 * @param[in]  key   配置键 ID（如 CFG-RATE-MEAS 0x30210001）
 * @param[out] value 读回值
 * @return true=读回成功
 */
bool ubxValGetU2(Uart &serial, uint32_t key, uint16_t *value);

/**
 * @brief 用 CFG-VALSET 写入单个 L 型（单字节）配置项
 * @param[in] key   配置键 ID
 * @param[in] value 目标值
 * @return true=接收机回 ACK（是否被采纳须用读回验证）
 * @note layers=RAM，掉电需 CFG-CFG 固化。
 */
bool ubxValSetByte(Uart &serial, uint32_t key, uint8_t value);

/**
 * @brief 轮询 UBX-MON-VER 并解析 SW/HW/PROTVER/ANTSUPERV
 * @return true=收到 ≥40B 的 MON-VER；各输出缓冲区允许为 nullptr/0
 */
bool ubxQueryMonVer(Uart &serial, char *sw, size_t sw_cap, char *hw, size_t hw_cap,
                    char *protver, size_t pv_cap, char *antsup, size_t ant_cap);

/** @brief CFG-RST 软复位接收机（DBG `ubxcfg rst`；GNSS 仅软件复位） */
void ubxResetReceiver(Uart &serial);

// ============================================================
//  NMEA 输出配置扩展（2026-08-09，DBG `ubxcfg nmea` / `ubxcfg proto`）
//
//  接收机侧 NMEA 输出控制——与库内解析（lib/ublox-main GpsProtocol）配合：
//  纯 NMEA 模块/UBX 配置失败时切 kAuto + 开 NMEA 输出，双保险。
// ============================================================

/** NMEA 消息 ID（CFG-MSG class 0xF0，u-blox 协议）——名称→ID 查表 */
enum
{
  UBX_NMEA_GGA = 0x00,
  UBX_NMEA_GLL = 0x01,
  UBX_NMEA_GSA = 0x02,
  UBX_NMEA_GSV = 0x03,
  UBX_NMEA_RMC = 0x04,
  UBX_NMEA_VTG = 0x05,
  UBX_NMEA_GRS = 0x06,
  UBX_NMEA_GST = 0x07,
  UBX_NMEA_ZDA = 0x08,
  UBX_NMEA_GBS = 0x09,
  UBX_NMEA_DTM = 0x0A,
  UBX_NMEA_GNS = 0x0D,
  UBX_NMEA_VLW = 0x0F,
  UBX_NMEA_TXT = 0x41,
};

/** 主 talker ID 编码（CFG-NMEA offset 8；0=不覆盖接收机默认） */
enum
{
  NMEA_TALKER_DEFAULT = 0x00,
  NMEA_TALKER_GP      = 0x01,
  NMEA_TALKER_GL      = 0x02,
  NMEA_TALKER_GN      = 0x03,
  NMEA_TALKER_GA      = 0x04,
  NMEA_TALKER_GB      = 0x05,
};

/** CFG-NMEA 输出滤波掩码（offset 0；仅常用位） */
enum
{
  NMEA_FILTER_POS  = 0x01,  // posFilt：GNSS 位置变化才输出
  NMEA_FILTER_MSK  = 0x02,  // mskPosFilt：掩码位置变化才输出
  NMEA_FILTER_TIME = 0x04,  // timeFilt：UTC 时间变化才输出
  NMEA_FILTER_DATE = 0x08,  // dateFilt：日期变化才输出
  NMEA_FILTER_TRK  = 0x10,  // trackFilt：航迹变化才输出
};

/**
 * @brief 设置消息输出速率（CFG-MSG；rate=0 关闭）
 * @param cls  class：0xF0=NMEA 句，0x01=UBX 消息（NAV-PVT/EOE/DOP/SAT/STATUS/SVIN）
 * @param msg_id 消息 ID（NMEA 用 UBX_NMEA_*，UBX 用协议 ID）
 * @param rate   输出速率（1-127；0 = 关闭）
 */
bool ubxMsgRateConfig(Uart &serial, uint8_t cls, uint8_t msg_id,
                      uint8_t rate);

/**
 * @brief 设置 NMEA 版本（CFG-NMEA 0x06 0x17 读改写，offset 1）
 * @param version 编码 = (主<<4)|次，如 0x23=2.3 / 0x41=4.1 / 0x4A=4.10 / 0x4B=4.11；0=不改
 */
bool ubxNmeaVersionConfig(Uart &serial, uint8_t version);

/**
 * @brief 设置主 talker ID（CFG-NMEA offset 8；NMEA_TALKER_*；0=不改）
 */
bool ubxNmeaTalkerConfig(Uart &serial, uint8_t talker_id);

/**
 * @brief 设置 NMEA 输出滤波掩码（CFG-NMEA offset 0；NMEA_FILTER_*；0=不过滤）
 */
bool ubxNmeaFilterConfig(Uart &serial, uint8_t filter_mask);

/**
 * @brief 设置串口输出协议掩码（CFG-PRT OutProtocolMask）
 * @param out_mask 1=UBX-only，2=NMEA-only，3=UBX+NMEA
 */
bool ubxProtoOutputConfig(Uart &serial, uint8_t out_mask);

/**
 * @brief 导航引擎参数（CFG-NAV5 0x06 0x24 读改写，mask 按需置位）
 * @param dyn_model    动态模型：2=Airborne<2g，4=Airborne<4g；0=不改
 * @param fix_mode     定位模式：-1 不改，0=2D，1=3D，2=auto
 * @param min_elev     最低仰角：-1 不改（0-90°；城市峡谷抬高可滤低仰角多径）
 * @param pdop_thresh  pDOP 定位门限：-1 不改（单位 0.1，如 50=5.0）
 * @note 2026-08-09 修复：旧实现把 dynModel 枚举写进 mask 字节致 Airborne 配置从未生效
 */
bool ubxNav5Config(Uart &serial, int dyn_model, int fix_mode,
                   int min_elev, int pdop_thresh);

/**
 * @brief CW 干扰检测开关（CFG-ITFM 0x06 0x39 读改写 bit0）
 * @note 城市/图传频段干扰导致定位漂移时可开启诊断（接收机自报干扰而非盲目调 EKF）
 */
bool ubxItfmConfig(Uart &serial, bool enable);

/**
 * @brief 天线状态查询（CFG-ANT 0x06 0x13，纯读）
 * @param out 4 字节出参：out[0..1]=config（bit3 短路检测/bit4 开路检测/bit5-6 供电），
 *            out[2..3]=status（低 2 位：0=INIT 1=UNKNOWN 2=OK 3=SHORT/OPEN）
 * @return 收到 4 字节响应
 */
bool ubxAntStatusQuery(Uart &serial, uint8_t out[4]);

/** @brief UBX 帧类型常量（供外部消息使能用） */
enum
{
  UBX_CLASS_NAV = 0x01,
  UBX_NAV_PVT   = 0x07,
  UBX_NAV_EOE   = 0x61,
  UBX_CLASS_CFG = 0x06,
  UBX_ACK_ACK   = 0x05,
};
