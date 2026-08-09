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
  bool     nav5_ok;           // CFG-NAV5（动态模型，读改写）成功
  bool     gnss_ok;           // CFG-GNSS（星座使能，读改写）成功
  bool     sbas_ok;           // CFG-SBAS（差分增强）成功
  bool     save_ok;           // CFG-CFG 固化闪存成功（未请求时 true）
  bool     verify_ok;         // 回读 CFG-PRT 校验成功
  uint32_t cfg_frames_sent;   // 发送的配置帧数
  uint32_t cfg_acks_received; // 收到的 ACK-ACK 数
};

// 配置选项（ubxAutoConfig 参数）
struct UbxConfigOptions
{
  uint32_t target_baud;      // 目标波特率（默认 921600）
  uint16_t nav_rate_ms;      // 导航测量周期 ms（100=10Hz；0=不改）
  uint8_t  airborne_g;       // 动态模型：0=不改，2=Airborne<2g，4=Airborne<4g
  uint8_t  gnss_mask;        // 星座使能掩码：bit0 GPS, bit1 GLONASS, bit2 Galileo, bit3 BeiDou（0=不改）
  bool     sbas_enable;      // 使能 SBAS 差分增强（EGNOS/WAAS）
  bool     nmea_out;         // false=UBX-only 输出；true=UBX+NMEA
  bool     persist;          // CFG-CFG 固化到接收机闪存
};

// 默认配置选项（main.cpp / DBG 使用）
static const UbxConfigOptions kUbxDefaultCfg = {
  /* target_baud */ 921600UL,
  /* nav_rate_ms */ 100,
  /* airborne_g  */ 0,
  /* gnss_mask   */ 0,
  /* sbas_enable */ false,
  /* nmea_out    */ false,
  /* persist     */ false,
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
UbxCfgResult ubxAutoConfig(HardwareSerial &serial, const UbxConfigOptions *opt);

/** @brief 上次配置结果（DBG `ubxcfg status` 读取） */
const UbxCfgResult &ubxLastCfgResult();

/** @brief CFG-RST 软复位接收机（DBG `ubxcfg rst`；GNSS 仅软件复位） */
void ubxResetReceiver(HardwareSerial &serial);

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
bool ubxMsgRateConfig(HardwareSerial &serial, uint8_t cls, uint8_t msg_id,
                      uint8_t rate);

/**
 * @brief 设置 NMEA 版本（CFG-NMEA 0x06 0x17 读改写，offset 1）
 * @param version 编码 = (主<<4)|次，如 0x23=2.3 / 0x41=4.1 / 0x4A=4.10 / 0x4B=4.11；0=不改
 */
bool ubxNmeaVersionConfig(HardwareSerial &serial, uint8_t version);

/**
 * @brief 设置主 talker ID（CFG-NMEA offset 8；NMEA_TALKER_*；0=不改）
 */
bool ubxNmeaTalkerConfig(HardwareSerial &serial, uint8_t talker_id);

/**
 * @brief 设置 NMEA 输出滤波掩码（CFG-NMEA offset 0；NMEA_FILTER_*；0=不过滤）
 */
bool ubxNmeaFilterConfig(HardwareSerial &serial, uint8_t filter_mask);

/**
 * @brief 设置串口输出协议掩码（CFG-PRT OutProtocolMask）
 * @param out_mask 1=UBX-only，2=NMEA-only，3=UBX+NMEA
 */
bool ubxProtoOutputConfig(HardwareSerial &serial, uint8_t out_mask);

/**
 * @brief 导航引擎参数（CFG-NAV5 0x06 0x24 读改写，mask 按需置位）
 * @param dyn_model    动态模型：2=Airborne<2g，4=Airborne<4g；0=不改
 * @param fix_mode     定位模式：-1 不改，0=2D，1=3D，2=auto
 * @param min_elev     最低仰角：-1 不改（0-90°；城市峡谷抬高可滤低仰角多径）
 * @param pdop_thresh  pDOP 定位门限：-1 不改（单位 0.1，如 50=5.0）
 * @note 2026-08-09 修复：旧实现把 dynModel 枚举写进 mask 字节致 Airborne 配置从未生效
 */
bool ubxNav5Config(HardwareSerial &serial, int dyn_model, int fix_mode,
                   int min_elev, int pdop_thresh);

/**
 * @brief CW 干扰检测开关（CFG-ITFM 0x06 0x39 读改写 bit0）
 * @note 城市/图传频段干扰导致定位漂移时可开启诊断（接收机自报干扰而非盲目调 EKF）
 */
bool ubxItfmConfig(HardwareSerial &serial, bool enable);

/**
 * @brief 天线状态查询（CFG-ANT 0x06 0x13，纯读）
 * @param out 4 字节出参：out[0..1]=config（bit3 短路检测/bit4 开路检测/bit5-6 供电），
 *            out[2..3]=status（低 2 位：0=INIT 1=UNKNOWN 2=OK 3=SHORT/OPEN）
 * @return 收到 4 字节响应
 */
bool ubxAntStatusQuery(HardwareSerial &serial, uint8_t out[4]);

/** @brief UBX 帧类型常量（供外部消息使能用） */
enum
{
  UBX_CLASS_NAV = 0x01,
  UBX_NAV_PVT   = 0x07,
  UBX_NAV_EOE   = 0x61,
  UBX_CLASS_CFG = 0x06,
  UBX_ACK_ACK   = 0x05,
};
