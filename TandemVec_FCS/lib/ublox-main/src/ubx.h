/*
 * ubx-gnss —— u-blox UBX 协议本地重构解析库
 *
 * 本文件源自 Bolder Flight Systems 的 ublox 库，但已在本仓库内做过较多本地重构，
 * 与官方原版不再一致。相对官方版本的主要本地改动：
 *   1. 新增 UbxEpoch 快照与固定环形队列（PopEpoch/Pump），面向 100/200 Hz INS 融合，
 *      让解析线程持续搬运串口、导航线程按自己节奏逐帧消费，避免新 PVT 覆盖未融合的旧观测；
 *   2. 新增 BeginConfigured()，只做通信探测、不重复调用 HardwareSerial::begin()，
 *      适配 ESP32-P4 等“引脚映射须由上层先指定”的平台；
 *   3. 新增大量链路健康统计计数器（校验失败/超长/EOE 缺 PVT/iTOW 不匹配/队列溢出等）；
 *   4. 新增 pvt_pdop()，仅凭 NAV-PVT 即可拿到 pDOP，无需额外开 NAV-DOP；
 *   5. 新增 NMEA 双协议支持（2026-08-09）：GpsProtocol 枚举（kUbx/kNmea/kAuto）、
 *      SetProtocol/SwitchProtocol 独立切换，内置 MicroNMEA 2.0.6（MIT）解析 GGA/RMC，
 *      NMEA 快照合成 UbxEpoch 入同一队列，kAuto 下 UBX 优先、失效后自动兜底。
 * 保留以下原始 MIT 版权声明仅为遵守许可证要求。
 *
 * 原始版权声明（MIT License）：
 * Brian R Taylor / brian.taylor@bolderflight.com
 * Copyright (c) 2022 Bolder Flight Systems Inc
 *
 * 特此免费授予任何获得本软件及相关文档文件（“软件”）副本的人士不受限制地处置
 * 本软件的权利，包括但不限于使用、复制、修改、合并、发布、分发、再许可和/或
 * 销售本软件副本的权利，并允许获得本软件的人士在满足以下条件的前提下这样做：
 *
 * 上述版权声明和本许可声明应包含在本软件的所有副本或主要部分中。
 *
 * 本软件按“原样”提供，不附带任何形式的明示或暗示担保，包括但不限于对适销性、
 * 特定用途适用性和非侵权性的担保。在任何情况下，作者或版权持有人均不对任何索赔、
 * 损害或其他责任负责，无论是合同诉讼、侵权诉讼还是其他诉讼。
 */

#ifndef UBX_SRC_UBX_H_ // NOLINT
#define UBX_SRC_UBX_H_

// Arduino 环境直接用 Arduino.h 提供的 HardwareSerial / 定宽整型；
// 非 Arduino（主机 CMake/单元测试）环境则手动引入定宽整型，并用 core/core.h 提供的
// HardwareSerial 兼容层（见 examples/cmake 与 test/ 的最小 Arduino shim）。
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "ubx_defs.h" // NOLINT  —— Class 常量与 U1/I2/X4 类型别名
#include "ubx_nav.h"  // NOLINT  —— 各 UBX-NAV 消息结构体
#include "minmea.h"   // NOLINT —— NMEA 备用解析（GGA/RMC），MIT 许可

namespace bfs
{
  /*
   * 一帧完整 GNSS epoch 的轻量快照。
   *
   * 该结构只保存 200 Hz INS 融合和诊断真正需要的字段，避免上层直接依赖 Ubx 内部那组
   * “最新消息”缓存。解析器可以继续吃 UART backlog，并把完整 NAV-PVT + NAV-EOE 结果放入
   * 固定队列；导航线程再按自己的节奏从队列里取（PopEpoch），互不抢占同一份解析缓存。
   *
   * 字段已做完单位换算（度/弧度/米/米每秒），上层拿到即可直接用，无需再乘缩放因子。
   */
  struct UbxEpoch
  {
    uint32_t pvt_tow_ms = 0;     // 本历元 NAV-PVT 的周内时刻【ms】，可做去重/时序诊断
    uint32_t eoe_tow_ms = 0;     // 本历元 NAV-EOE 的周内时刻【ms】，正常应等于 pvt_tow_ms
    uint32_t receive_time_us = 0; // 本历元完整 EOE 被 Pump 处理的 MCU 时间【us】
    double gps_tow_s = 0.0;      // GPS 周内秒【s】（有 NAV-TIMEGPS 时更精确，否则由 PVT.iTOW 推算）
    int8_t fix = 0;              // 定位类型，取值同 Ubx::Fix 枚举
    int8_t num_sv = 0;           // 参与解算卫星数
    double lat_deg = 0.0;        // 纬度【度】；必须保持 double，不能降成 float32，否则绝对经纬度会出现分米级量化
    double lon_deg = 0.0;        // 经度【度】；必须保持 double，不能降成 float32，否则 113 deg 附近经度约 0.75 m 一跳
    double lat_rad = 0.0;        // 纬度【弧度】（已预乘 DEG2RAD，省去上层换算）
    double lon_rad = 0.0;        // 经度【弧度】
    float alt_wgs84_m = 0.0f;    // WGS84 椭球高【m】
    float alt_msl_m = 0.0f;      // 海平面高(MSL)【m】
    float north_vel_mps = 0.0f;  // NED 北向速度【m/s】
    float east_vel_mps = 0.0f;   // NED 东向速度【m/s】
    float down_vel_mps = 0.0f;   // NED 地向速度【m/s】，正值向下
    float horz_acc_m = 0.0f;     // 水平位置精度估计【m】，融合质量门限常用
    float vert_acc_m = 0.0f;     // 垂直位置精度估计【m】
    float spd_acc_mps = 0.0f;    // 速度精度估计【m/s】
    float pvt_pdop = 0.0f;       // 来自 NAV-PVT 的位置 DOP，几何质量判据
  };

  class Ubx
  {
  public:
    // epoch 环形队列容量。导航线程偶尔落后时用它吸收抖动；长期落后会丢最旧帧（见 PushEpoch）。
    static constexpr uint8_t kEpochQueueCapacity = 4U;
    /*
     * 定位类型枚举。数值刻意与 u-blox NAV-PVT.fixType 体系区分（这里从 1 起按“质量递增”排列），
     * 便于上层用 fix >= FIX_3D 之类的比较做融合门限。映射逻辑见 ubx.cpp::ProcessNavData。
     */
    enum Fix : int8_t
    {
      FIX_NONE = 1,       // 无定位
      FIX_2D = 2,         // 2D 定位
      FIX_3D = 3,         // 3D 定位
      FIX_DGNSS = 4,      // 3D + 差分改正(DGNSS/SBAS)
      FIX_RTK_FLOAT = 5,  // 3D + RTK 浮点解（分米级）
      FIX_RTK_FIXED = 6   // 3D + RTK 固定解（厘米级）
    };
    /*
     * 协议模式（2026-08-09 双协议支持）：
     *   kUbx  — 纯 UBX（默认，向后兼容）
     *   kNmea — 纯 NMEA（接收机仅输出 NMEA 句，如 GGA/RMC）
     *   kAuto — 双流解析：UBX 优先，UBX 失效（nmea_ubx_backoff 窗口内无 UBX
     *            epoch）后自动用 NMEA 快照合成 epoch 兜底；两种源的 epoch
     *            混在同一队列，上层 PopEpoch 无感。
     * 切换解析模式不碰串口波特率；需要一并改波特率用 SwitchProtocol()。
     */
    enum class GpsProtocol : uint8_t
    {
      kUbx = 0,
      kNmea = 1,
      kAuto = 2
    };
    // 默认构造：未绑定串口，使用前须先 Config(bus)。构造即 Reset 清零所有状态。
    Ubx() { Reset(); }
    // 带串口构造：直接绑定 HardwareSerial 总线指针。
    explicit Ubx(HardwareSerial *bus) : bus_(bus) { Reset(); }
    // 设置/更换通信串口。用默认构造时必须调用本方法绑定总线。
    void Config(HardwareSerial *bus);
    /* 清空解析状态、统计和输出缓存；不改变已经配置好的串口指针。 */
    void Reset();
    /*
     * 标准初始化：内部调用 HardwareSerial::begin(baud) 设置波特率，再做通信探测。
     * 适合“库自己管串口”的平台（如 Teensy）。收到首帧有效 UBX 返回 true，超时返回 false。
     */
    bool Begin(const int32_t baud);
    /*
     * 只做通信探测，不重新调用 HardwareSerial::begin()。
     * ESP32-P4 等平台需要上层先指定 RX/TX 引脚，若库内部再次 begin(baud)
     * 可能把 UART 引脚恢复到默认映射，导致真实 GNSS 通信失败。
     * 用法：上层先 gpsSerial.begin(baud, SERIAL_8N1, rx, tx) 再 Config(&gpsSerial)，
     * 最后调 BeginConfigured() 仅探测链路。
     */
    bool BeginConfigured();
    /*
     * 阻塞式按历元读取（经典用法）。持续读串口直到收到一帧 NAV-EOE 即返回 true，
     * 表示当前历元数据已齐、可以读取各 getter。收到 EOE 后必须立即返回让上层消费，
     * 否则缓冲区里堆积的下一组 NAV-PVT 会覆盖当前数据。串口暂无完整历元时返回 false。
     * 注意：Read() 路径不入 epoch 队列，与 Pump()/PopEpoch() 是两套独立用法，不要混用。
     */
    bool Read();
    /*
     * 非阻塞泵入当前可读 UART 字节。
     * 无参版本会尽量清空当前串口 backlog；带 max_bytes 版本用于高频导航线程给
     * GNSS 解析设置单次工作预算，避免异常 backlog 把 200 Hz 主循环拖爆。
     * 与 Read() 不同，Pump() 会把每个组装好的完整 epoch 推入内部环形队列，
     * 供 PopEpoch() 按导航线程节奏消费。返回值表示本次调用是否至少入队了一个完整 epoch。
     */
    bool Pump();
    bool Pump(size_t max_bytes);
    bool Pump(size_t max_bytes, uint32_t receive_time_us);
    /* 从固定队列取出最早的一帧完整 epoch；队列空或入参为空时返回 false。 */
    bool PopEpoch(UbxEpoch *epoch);
    /*
     * 字节旁路回调：解析器从串口读入的每个字节都会镜像到该回调（若已注册）。
     * 用途：与 NMEA 等非 UBX 协议共享同一根串口——UBX 泵按字节预算读空缓冲时，
     * 把每个字节同步镜像给第二个解析器，避免两个解析器竞争读串口互相偷字节。
     * 注意：DETA100 模式下 ubx.Pump 不运行，回调不会被触发，NMEA 也无需工作。
     */
    using UbxByteTapFn = void (*)(uint8_t byte, void *context);
    void SetByteTap(UbxByteTapFn fn, void *context)
    {
      byte_tap_fn_ = fn;
      byte_tap_ctx_ = context;
    }

    /* ======================= 协议模式与 NMEA 配置（2026-08-09） ======================= */
    /* 切换解析协议（kUbx/kNmea/kAuto）。不改变串口波特率。 */
    void SetProtocol(GpsProtocol protocol) { protocol_ = protocol; }
    /*
     * 切换协议并同步重设串口波特率（serial.begin）。
     * 用于"独立切换"场景：NMEA 接收机常见 9600/38400/115200，与 UBX 的
     * 921600 不同，切换时必须一并 begin。返回 false 表示未绑定串口。
     */
    bool SwitchProtocol(GpsProtocol protocol, int32_t baud);
    inline GpsProtocol protocol() const { return protocol_; }
    /* NMEA 快照新鲜度超时【ms】：超过视为链路失效（GGA/RMC 标称 1Hz，默认 3000）。 */
    void SetNmeaFixTimeoutMs(uint32_t ms) { nmea_fix_timeout_ms_ = ms; }
    inline uint32_t nmea_fix_timeout_ms() const { return nmea_fix_timeout_ms_; }
    /* kAuto 下 UBX 失效判定窗口【ms】：该窗口内收过 UBX epoch 则跳过 NMEA 合成，
     * 避免混合输出时同一接收机的位置被双通道重复融合（默认 300）。 */
    void SetNmeaUbxBackoffMs(uint32_t ms) { nmea_ubx_backoff_ms_ = ms; }
    inline uint32_t nmea_ubx_backoff_ms() const { return nmea_ubx_backoff_ms_; }

    /* ---- NMEA 链路状态（诊断用；kNmea/kAuto 模式有效）---- */
    inline bool nmea_valid() const { return nmea_snapshot_valid_; }          // 快照是否有效
    inline int8_t nmea_fix() const { return nmea_snapshot_fix_; }            // 映射定位类型（Fix 枚举）
    inline int8_t nmea_num_sv() const { return nmea_snapshot_sv_; }          // 参与解算卫星数
    inline float nmea_hdop() const { return nmea_snapshot_hdop_; }           // 水平精度因子（GGA）
    inline float nmea_pdop() const { return nmea_snapshot_pdop_; }           // 位置精度因子（GSA，0=未提供）
    inline float nmea_vdop() const { return nmea_snapshot_vdop_; }           // 垂直精度因子（GSA，0=未提供）
    inline uint32_t nmea_last_fix_ms() const { return nmea_last_fix_ms_; }   // 最近有效 fix 的 MCU 时间
    inline uint32_t nmea_sentence_count() const { return nmea_sentence_count_; }      // 完整句子数
    inline uint32_t nmea_bad_checksum_count() const { return nmea_bad_checksum_count_; }  // 校验失败数
    inline uint32_t nmea_overflow_count() const { return nmea_overflow_count_; }  // 超长残句丢弃数
    inline uint32_t nmea_gga_count() const { return nmea_gga_count_; }       // GGA 句数（校验通过）
    inline uint32_t nmea_rmc_count() const { return nmea_rmc_count_; }       // RMC 句数
    inline uint32_t nmea_gsa_count() const { return nmea_gsa_count_; }       // GSA 句数
    inline uint32_t nmea_unknown_count() const { return nmea_unknown_count_; }  // 未处理句子数
    inline uint32_t nmea_fix_count() const { return nmea_fix_count_; }       // 有效定位句次数
    inline uint32_t nmea_epoch_count() const { return nmea_epoch_count_; }   // 合成并入队的 NMEA epoch 数
    inline double nmea_lat_rad() const { return nmea_snapshot_lat_rad_; }    // 纬度【rad】
    inline double nmea_lon_rad() const { return nmea_snapshot_lon_rad_; }    // 经度【rad】
    inline float nmea_alt_wgs84_m() const { return nmea_snapshot_alt_wgs84_m_; }  // 椭球高【m】
    inline float nmea_vel_n_mps() const { return nmea_snapshot_vel_n_mps_; } // 北向速度【m/s】
    inline float nmea_vel_e_mps() const { return nmea_snapshot_vel_e_mps_; } // 东向速度【m/s】

    /* ======================= 数据输出（getter） ======================= */
    /* 以下 getter 返回“最近一次成功解析历元”的数据，已完成单位换算。 */

    inline Fix fix() const { return fix_; }                  // 定位类型
    inline int8_t num_sv() const { return num_sv_; }         // 参与解算卫星数
    inline int16_t utc_year() const { return year_; }        // UTC 年（时间未完全确认时为 0）
    inline int8_t utc_month() const { return month_; }       // UTC 月
    inline int8_t utc_day() const { return day_; }           // UTC 日
    inline int8_t utc_hour() const { return hour_; }         // UTC 时
    inline int8_t utc_min() const { return min_; }           // UTC 分
    inline int8_t utc_sec() const { return sec_; }           // UTC 秒
    inline int32_t utc_nano() const { return nano_; }        // UTC 秒的小数部分【ns】，可为负
    inline double gps_tow_s() const { return tow_s_; }       // GPS 周内秒【s】
    // 直接读最近 NAV-PVT/NAV-EOE 的原始 iTOW【ms】，常用于诊断两者是否配对。
    inline uint32_t nav_pvt_tow_ms() const { return ubx_nav_pvt_.payload.i_tow; }
    inline uint32_t nav_eoe_tow_ms() const { return ubx_nav_eoe_.payload.i_tow; }

    /* ---- 链路健康统计计数器（本地新增，用于诊断 UART/UBX 解析质量）---- */
    inline uint32_t rx_byte_count() const { return rx_byte_count_; }              // 累计读入字节数
    inline uint32_t valid_msg_count() const { return valid_msg_count_; }          // 累计校验通过的 UBX 报文数
    inline uint32_t checksum_fail_count() const { return checksum_fail_count_; }  // 累计校验失败数（应保持 0）
    inline uint32_t oversize_msg_count() const { return oversize_msg_count_; }    // 累计超长(>1024B)被丢弃数
    inline uint32_t nav_pvt_count() const { return nav_pvt_count_; }              // 累计收到 NAV-PVT 数
    inline uint32_t nav_eoe_count() const { return nav_eoe_count_; }              // 累计收到 NAV-EOE 数
    inline uint32_t eoe_without_pvt_count() const { return eoe_without_pvt_count_; }  // EOE 到来但本历元无 PVT 的次数
    inline uint32_t pvt_duplicate_in_epoch_count() const { return pvt_duplicate_in_epoch_count_; }  // 同一历元收到多个 PVT 的次数
    inline uint32_t tow_mismatch_count() const { return tow_mismatch_count_; }    // PVT 与 EOE iTOW 不一致次数
    inline uint32_t latest_epoch_tow_ms() const { return latest_epoch_tow_ms_; }
    inline uint32_t latest_epoch_receive_time_us() const { return latest_epoch_receive_time_us_; }
    inline uint32_t queue_overflow_count() const { return queue_overflow_count_; }  // 队列满丢最旧帧次数
    inline uint32_t queued_epoch_count() const { return queued_epoch_count_; }    // 累计成功入队 epoch 数
    inline uint32_t pending_epoch_count() const { return epoch_queue_count_; }    // 当前队列中待消费 epoch 数
    inline uint8_t last_msg_cls() const { return last_msg_cls_; }                 // 最近一帧 UBX 的 Class
    inline uint8_t last_msg_id() const { return last_msg_id_; }                   // 最近一帧 UBX 的 ID
    inline uint16_t last_msg_len() const { return last_msg_len_; }                // 最近一帧 UBX 的 payload 长度

    inline int16_t gps_week() const { return week_; }            // GPS 周数（需 NAV-TIMEGPS 且 weekValid）
    inline int8_t leap_s() const { return leap_s_; }             // GPS 闰秒（需 NAV-TIMEGPS 且 leapSValid）
    inline uint32_t time_acc_ns() const { return t_acc_ns_; }    // 时间精度估计【ns】

    /* ---- 速度（NED / ECEF / 地速航向）---- */
    inline float north_vel_mps() const { return ned_vel_mps_[0]; }  // NED 北向速度【m/s】
    inline float east_vel_mps() const { return ned_vel_mps_[1]; }   // NED 东向速度【m/s】
    inline float down_vel_mps() const { return ned_vel_mps_[2]; }   // NED 地向速度【m/s】，正值向下
    inline float gnd_spd_mps() const { return gnd_spd_mps_; }       // 地速(2D)【m/s】
    inline float ecef_vel_x_mps() const { return ecef_vel_mps_[0]; }  // ECEF X 速度【m/s】
    inline float ecef_vel_y_mps() const { return ecef_vel_mps_[1]; }  // ECEF Y 速度【m/s】
    inline float ecef_vel_z_mps() const { return ecef_vel_mps_[2]; }  // ECEF Z 速度【m/s】
    inline float spd_acc_mps() const { return s_acc_mps_; }         // 速度精度估计【m/s】
    inline float track_deg() const { return track_deg_; }           // 运动航向(2D)【度】
    inline float track_rad() const { return track_deg_ * DEG2RADf_; }       // 运动航向【弧度】
    inline float track_acc_deg() const { return track_acc_deg_; }           // 运动航向精度【度】
    inline float track_acc_rad() const { return track_acc_deg_ * DEG2RADf_; }  // 运动航向精度【弧度】

    /* ---- 位置（经纬高，度/弧度两种单位）---- */
    inline double lat_deg() const { return llh_[0]; }                 // 纬度【度】，底层高精度输出，显示层可另行降精度
    inline double lat_rad() const { return llh_[0] * DEG2RADl_; }     // 纬度【弧度】
    inline double lon_deg() const { return llh_[1]; }                 // 经度【度】，底层高精度输出，显示层可另行降精度
    inline double lon_rad() const { return llh_[1] * DEG2RADl_; }     // 经度【弧度】
    inline float alt_wgs84_m() const { return static_cast<float>(llh_[2]); }  // WGS84 椭球高【m】
    inline float alt_msl_m() const { return alt_msl_m_; }             // 海平面高(MSL)【m】
    inline float horz_acc_m() const { return h_acc_m_; }              // 水平位置精度估计【m】
    inline float vert_acc_m() const { return v_acc_m_; }              // 垂直位置精度估计【m】

    /* ---- 位置（ECEF 地心地固坐标）---- */
    inline double ecef_pos_x_m() const { return ecef_m_[0]; }     // ECEF X 位置【m】
    inline double ecef_pos_y_m() const { return ecef_m_[1]; }     // ECEF Y 位置【m】
    inline double ecef_pos_z_m() const { return ecef_m_[2]; }     // ECEF Z 位置【m】
    inline float ecef_pos_acc_m() const { return p_acc_m_; }      // ECEF 位置精度估计【m】

    /* ---- DOP 精度衰减因子（需接收机开启 NAV-DOP，否则恒为 0）---- */
    inline float gdop() const { return gdop_; }  // 几何 DOP
    inline float pdop() const { return pdop_; }  // 位置 DOP（来自 NAV-DOP）
    /* pDOP from NAV-PVT（本地新增）：无需 NAV-DOP 消息，仅靠 NAV-PVT 即可拿到位置精度因子。
     * 接收机未配置 NAV-DOP 输出时 pdop() 恒为 0，应改用此 getter 做几何质量判据。 */
    inline float pvt_pdop() const { return pvt_pdop_; }
    inline float tdop() const { return tdop_; }  // 时间 DOP
    inline float vdop() const { return vdop_; }  // 垂直 DOP
    inline float hdop() const { return hdop_; }  // 水平 DOP
    inline float ndop() const { return ndop_; }  // 北向 DOP
    inline float edop() const { return edop_; }  // 东向 DOP

    /* ---- 相对定位数据（需 NAV-RELPOSNED，RTK 基线/双天线测向）---- */
    inline bool rel_pos_avail() const { return rel_pos_avail_; }                       // 相对定位是否可用
    inline bool rel_pos_moving_baseline() const { return rel_pos_moving_baseline_; }   // 是否移动基站模式
    inline bool rel_pos_ref_pos_miss() const { return rel_pos_ref_pos_miss_; }         // 本历元是否用了外推参考位置
    inline bool rel_pos_ref_obs_miss() const { return rel_pos_ref_obs_miss_; }         // 本历元是否用了外推参考观测
    inline bool rel_pos_heading_valid() const { return rel_pos_heading_valid_; }       // 相对矢量航向是否有效
    inline bool rel_pos_normalized() const { return rel_pos_norm_; }                   // 相对矢量是否已归一化
    inline double rel_pos_north_m() const { return rel_pos_ned_m_[0]; }                // 相对位置北向分量【m】
    inline double rel_pos_east_m() const { return rel_pos_ned_m_[1]; }                 // 相对位置东向分量【m】
    inline double rel_pos_down_m() const { return rel_pos_ned_m_[2]; }                 // 相对位置地向分量【m】
    inline float rel_pos_acc_north_m() const { return rel_pos_ned_acc_m_[0]; }         // 北向分量精度【m】
    inline float rel_pos_acc_east_m() const { return rel_pos_ned_acc_m_[1]; }          // 东向分量精度【m】
    inline float rel_pos_acc_down_m() const { return rel_pos_ned_acc_m_[2]; }          // 地向分量精度【m】
    inline double rel_pos_len_m() const { return rel_pos_len_m_; }                     // 基线长【m】
    inline float rel_pos_len_acc_m() const { return rel_pos_len_acc_m_; }              // 基线长精度【m】
    inline float rel_pos_heading_deg() const { return rel_pos_heading_deg_; }          // 相对矢量航向【度】
    inline float rel_pos_heading_acc_deg() const
    {
      return rel_pos_heading_acc_deg_;  // 相对矢量航向精度【度】
    }
    inline float rel_pos_heading_rad() const
    {
      return rel_pos_heading_deg_ * DEG2RADf_;  // 相对矢量航向【弧度】
    }
    inline float rel_pos_heading_acc_rad() const
    {
      return rel_pos_heading_acc_deg_ * DEG2RADf_;  // 相对矢量航向精度【弧度】
    }

    /* ---- Survey-in 定点勘测数据（需 NAV-SVIN，固定基站架设用）---- */
    inline bool svin_valid() const { return svin_valid_; }              // Survey-in 平均坐标是否已有效
    inline bool svin_in_progress() const { return svin_in_progress_; }  // Survey-in 是否进行中
    inline uint32_t svin_dur_s() const { return svin_dur_s_; }          // 已进行观测时长【s】
    inline double svin_ecef_pos_x_m() const { return svin_ecef_m_[0]; } // 平均位置 ECEF X【m】
    inline double svin_ecef_pos_y_m() const { return svin_ecef_m_[1]; } // 平均位置 ECEF Y【m】
    inline double svin_ecef_pos_z_m() const { return svin_ecef_m_[2]; } // 平均位置 ECEF Z【m】
    inline float svin_ecef_pos_acc_m() const { return svin_acc_m_; }    // 平均位置精度【m】
    inline uint32_t svin_num_obs() const { return svin_num_obs_; }      // 使用的观测次数

  private:
    /* 阻塞式读字节直到组完一帧有效 UBX 报文（BeginConfigured/Read 路径用；
     * 不带字节预算，读空串口返回 false）。Pump 路径走 ProcessUbxByte 逐字节分派。 */
    bool ParseMsg();
    /* 处理一帧有效报文：分发到对应 NAV 缓存。queue_output 为 true 时，完整历元会被推入队列。 */
    bool HandleValidMessage(bool queue_output);
    /* 把各 NAV 缓存里的原始整数换算成物理量，填入下方各数据成员。 */
    void ProcessNavData();
    /* 把当前已处理数据打包成一帧 UbxEpoch 快照。 */
    UbxEpoch MakeEpoch() const;
    /* 把一帧 epoch 压入环形队列；队列满时丢最旧帧并累加 queue_overflow_count_。 */
    void PushEpoch(const UbxEpoch &epoch);
    /* 单字节推进 UBX 状态机；组完一帧且校验通过返回 true（由 Pump 调 HandleValidMessage）。 */
    bool ProcessUbxByte(uint8_t c);
    /* 单字节喂 NMEA 句子切分器；完整句子交给 nmeaParseSentence。 */
    void nmeaProcessByte(uint8_t b);
    /* 解析一帧完整 NMEA 句子（校验通过后按 GGA/RMC 分发刷新快照）。 */
    void nmeaParseSentence(const char *sentence);
    /* GGA 有效句：刷新位置/精度/高程快照。 */
    void nmeaUpdateSnapshotFromGga(const struct minmea_sentence_gga &gga);
    /* RMC 有效句：刷新速度/时间快照。 */
    void nmeaUpdateSnapshotFromRmc(const struct minmea_sentence_rmc &rmc);
    /* GSA 句：刷新 PDOP/VDOP 快照。is_combined=true 表示主 talker（GN=组合解）优先。 */
    void nmeaUpdateSnapshotFromGsa(const struct minmea_sentence_gsa &gsa,
                                   bool is_combined);
    /* NMEA 快照 → UbxEpoch（换算逻辑经宿主机回归测试验证，tools/nmea-host-test）。 */
    UbxEpoch MakeNmeaEpoch() const;
    /* kAuto/kNmea 下的 NMEA 兜底合成：快照新鲜 + 伪 tow 去重 +（kAuto 还需 UBX 失效）→ 入队。 */
    bool TryPushNmeaEpoch();
    /* 清空 NMEA 解析状态与快照（Reset 调用；不改变协议模式与配置）。 */
    void ResetNmea();

    /* ---- 通信 ---- */
    HardwareSerial *bus_ = nullptr;                      // GNSS 串口总线指针
    UbxByteTapFn byte_tap_fn_ = nullptr;                 // 字节镜像回调（外部调试/分析用）
    void *byte_tap_ctx_ = nullptr;                       // 回调上下文透传
    int16_t comm_timeout_count_ = 0;                     // Begin 探测时的轮询计数
    static const int16_t COMM_TIMEOUT_TRIES_ = 1000;     // 探测最大轮询次数
    static const int16_t COMM_TIMEOUT_DELAY_MS_ = 10;    // 每轮探测无数据时的等待【ms】

    /* ---- 协议模式与 NMEA 解析（2026-08-09，minmea MIT）---- */
    GpsProtocol protocol_ = GpsProtocol::kUbx;           // 当前协议模式（默认纯 UBX）
    // minmea 解析完整句子（$..*CK），字节流由内部切分器组装。NMEA 最长句子约 82 字节。
    static constexpr uint8_t kNmeaSentenceBufLen = 96;
    char nmea_sentence_[kNmeaSentenceBufLen];            // 完整句子缓冲
    uint8_t nmea_sentence_len_ = 0;                      // 缓冲内有效字节数
    bool nmea_in_sentence_ = false;                      // 是否在句子收集状态
    uint32_t nmea_fix_timeout_ms_ = 3000;                // NMEA 快照新鲜度超时
    uint32_t nmea_ubx_backoff_ms_ = 300;                 // kAuto 下 UBX 失效判定窗口
    // NMEA epoch 合成最小间隔（GGA/RMC 双句 1Hz 流 → 合成率 1Hz）
    static constexpr uint32_t kNmeaEpochMinIntervalUs = 800000;
    bool nmea_ubx_seen_ = false;                         // 是否收到过 UBX epoch（backoff 前提）
    uint32_t last_ubx_epoch_us_ = 0;                     // 最近 UBX epoch 入队时刻【us】（kAuto 闸门）
    bool nmea_epoch_seen_ = false;                       // 是否合成过 NMEA epoch（节流前提）
    uint32_t nmea_last_epoch_us_ = 0;                    // 最近 NMEA epoch 合成时刻【us】（节流）
    // NMEA 有效 fix 快照（只在有效句刷新；无效句保留上一快照）
    bool nmea_snapshot_valid_ = false;
    bool nmea_snapshot_alt_valid_ = false;
    int8_t nmea_snapshot_fix_ = 0;                       // 映射定位类型（Fix 枚举，GGA quality 映射）
    int8_t nmea_snapshot_sv_ = 0;
    float nmea_snapshot_hdop_ = 0.0f;
    float nmea_snapshot_pdop_ = 0.0f;                    // 位置精度因子（GSA；0=未提供→合成不缩放）
    float nmea_snapshot_vdop_ = 0.0f;                    // 垂直精度因子（GSA）
    double nmea_snapshot_lat_rad_ = 0.0;
    double nmea_snapshot_lon_rad_ = 0.0;
    float nmea_snapshot_alt_wgs84_m_ = 0.0f;
    float nmea_snapshot_alt_msl_m_ = 0.0f;
    float nmea_snapshot_vel_n_mps_ = 0.0f;
    float nmea_snapshot_vel_e_mps_ = 0.0f;
    uint32_t nmea_snapshot_tow_ms_ = 0;                  // UTC 时间-of-day 伪 tow（毫秒）
    uint32_t nmea_last_fused_tow_ms_ = 0;                // 已合成消费的伪 tow（本地去重）
    uint32_t nmea_last_fix_ms_ = 0;                      // 最近有效 fix 的 MCU 时间【ms】
    uint32_t nmea_sentence_count_ = 0;                   // 完整句子数（含无效）
    uint32_t nmea_bad_checksum_count_ = 0;               // 校验失败句子数
    uint32_t nmea_overflow_count_ = 0;                   // 超长残句丢弃数
    uint32_t nmea_gga_count_ = 0;                        // GGA 句数（校验通过）
    uint32_t nmea_rmc_count_ = 0;                        // RMC 句数
    uint32_t nmea_gsa_count_ = 0;                        // GSA 句数
    uint32_t nmea_unknown_count_ = 0;                    // 未处理句子数
    uint32_t nmea_fix_count_ = 0;                        // 有效定位句次数
    uint32_t nmea_epoch_count_ = 0;                      // 合成并入队的 NMEA epoch 数

    /* ---- 解析参数与状态机 ---- */
    static constexpr size_t UBX_MAX_PAYLOAD_ = 1024;     // 支持的最大 payload 字节数（超过即判为异常丢弃）
    static constexpr uint8_t UBX_HEADER_[2] = {0xB5, 0x62};  // UBX 帧头同步字 0xB5 0x62
    static constexpr uint8_t UBX_CLS_POS_ = 2;           // 状态机中 Class 字节所处的位置索引
    static constexpr uint8_t UBX_ID_POS_ = 3;            // ID 字节位置索引
    static constexpr uint8_t UBX_LEN_POS_LSB_ = 4;       // 长度低字节位置索引
    static constexpr uint8_t UBX_LEN_POS_MSB_ = 5;       // 长度高字节位置索引
    static constexpr uint8_t UBX_HEADER_LEN_ = 6;        // 帧头总长（同步字2+Class+ID+长度2）
    // 校验和覆盖范围相对结构体起点的偏移：rx_msg_ 从 cls 开始即参与校验，故减去 2 字节同步字。
    static constexpr uint8_t UBX_CHK_OFFSET_ = UBX_HEADER_LEN_ - sizeof(UBX_HEADER_);
    uint8_t c_, len_, chk_rx_, chk_[2];                  // c_:当前字节 len_:长度低字节暂存 chk_rx_:收到的校验高字节前置
    uint16_t chk_cmp_rx_, chk_cmp_tx_;                   // 本地计算出的校验值（收/发）
    size_t parser_state_ = 0;                            // 状态机当前位置（0 起，跨调用保留以支持分块解析）

    /* ---- 链路健康统计（与上面同名 getter 对应）---- */
    uint32_t rx_byte_count_ = 0;
    uint32_t valid_msg_count_ = 0;
    uint32_t checksum_fail_count_ = 0;
    uint32_t oversize_msg_count_ = 0;
    uint32_t nav_pvt_count_ = 0;
    uint32_t nav_eoe_count_ = 0;
    uint32_t eoe_without_pvt_count_ = 0;
    uint32_t pvt_duplicate_in_epoch_count_ = 0;
    uint32_t tow_mismatch_count_ = 0;
    uint32_t pump_receive_time_us_ = 0;
    uint32_t latest_epoch_tow_ms_ = 0;
    uint32_t latest_epoch_receive_time_us_ = 0;
    uint32_t queue_overflow_count_ = 0;
    uint32_t queued_epoch_count_ = 0;
    uint8_t last_msg_cls_ = 0;
    uint8_t last_msg_id_ = 0;
    uint16_t last_msg_len_ = 0;

    // 角度换算常量：度↔弧度。float/double 两份，分别给 float 与 double 精度的换算用。
    static constexpr float DEG2RADf_ = 3.14159265358979323846264338327950288f / 180.0f;
    static constexpr double DEG2RADl_ = 3.14159265358979323846264338327950288 / 180.0;

    /* ---- 解析过程中的中间标志位 ---- */
    bool eoe_ = false;                   // 历元结束标志（内部状态）
    bool use_hp_pos_ = false;            // 是否已收到高精度位置报文（HPPOSLLH/HPPOSECEF）
    bool svin_data_ = false;             // 是否已收到 Survey-in 报文
    bool rel_pos_data_ = false;          // 是否已收到相对定位报文
    bool pvt_ready_this_epoch_ = false;  // 本历元是否已收到 NAV-PVT（EOE 到来时据此判断历元是否完整）
    Fix fix_ = FIX_NONE;                 // 当前定位类型
    bool gnss_fix_ok_ = false, diff_soln_ = false;  // gnssFixOK / 差分解标志（来自 PVT.flags）
    bool valid_date_ = false, valid_time_ = false, fully_resolved_ = false, validity_confirmed_ = false;  // PVT 时间有效性分解位
    bool tow_valid_ = false, week_valid_ = false, leap_valid_ = false;  // NAV-TIMEGPS 各有效性位
    bool confirmed_date_ = false, confirmed_time_ = false, valid_time_and_date_ = false;  // 时间确认位与综合判定
    bool invalid_llh_ = true, invalid_ecef_ = true;  // 经纬高/ECEF 无效标志（取自报文，默认无效）
    bool rel_pos_avail_ = false;
    bool rel_pos_moving_baseline_ = false;
    bool rel_pos_ref_pos_miss_ = false;
    bool rel_pos_ref_obs_miss_ = false;
    bool rel_pos_heading_valid_ = false;
    bool rel_pos_norm_ = false;
    bool svin_valid_ = false;
    bool svin_in_progress_ = false;
    int8_t carr_soln_ = 0;  // 载波相位解状态：0 无 / 1 浮点 / 2 固定（来自 PVT.flags 高 2 位）

    /* ---- 已换算的导航数据（与各 getter 一一对应）---- */
    int8_t num_sv_ = 0;
    int8_t month_ = 0, day_ = 0, hour_ = 0, min_ = 0, sec_ = 0;
    int8_t leap_s_ = 0;
    int16_t year_ = 0;
    int16_t week_ = 0;
    int32_t nano_ = 0;
    uint32_t t_acc_ns_ = 0;
    uint32_t svin_dur_s_ = 0, svin_num_obs_ = 0;
    float alt_msl_m_ = 0.0f;
    float gnd_spd_mps_ = 0.0f;
    float track_deg_ = 0.0f;
    float gdop_ = 0.0f, pdop_ = 0.0f, tdop_ = 0.0f, vdop_ = 0.0f, hdop_ = 0.0f, ndop_ = 0.0f, edop_ = 0.0f;
    float pvt_pdop_ = 0.0f;  // 本地新增：从 NAV-PVT 解析的 pDOP，独立于 NAV-DOP 的 pdop_。
    float h_acc_m_ = 0.0f, v_acc_m_ = 0.0f, p_acc_m_ = 0.0f, track_acc_deg_ = 0.0f, s_acc_mps_ = 0.0f;
    float rel_pos_heading_deg_ = 0.0f;
    float rel_pos_heading_acc_deg_ = 0.0f;
    float rel_pos_len_acc_m_ = 0.0f;
    float svin_acc_m_ = 0.0f;
    double rel_pos_len_m_ = 0.0;
    double tow_s_ = 0.0;
    float ecef_vel_mps_[3] = {};       // [0]=X [1]=Y [2]=Z
    float ned_vel_mps_[3] = {};        // [0]=N [1]=E [2]=D
    float rel_pos_ned_acc_m_[3] = {};  // [0]=N [1]=E [2]=D 精度
    double ecef_m_[3] = {};            // ECEF 位置 [0]=X [1]=Y [2]=Z
    double llh_[3] = {};               // [0]=纬度(度) [1]=经度(度) [2]=椭球高(m)
    double rel_pos_ned_m_[3] = {};     // 相对位置 [0]=N [1]=E [2]=D
    double svin_ecef_m_[3] = {};       // Survey-in 平均 ECEF [0]=X [1]=Y [2]=Z

    /* ---- epoch 环形队列 ---- */
    UbxEpoch epoch_queue_[kEpochQueueCapacity] = {};  // 固定容量环形缓冲
    uint8_t epoch_queue_head_ = 0;                    // 队头索引（下一个被 Pop 的位置）
    uint8_t epoch_queue_count_ = 0;                   // 当前队列中元素个数

    /*
     * UBX 8 位 Fletcher 校验和计算器。Compute 重置后一次性算完整段；
     * Update 在已有累加值上继续累加（本库目前用 Compute 一次算完）。
     * 返回值高字节为 CK_B(sum1)、低字节为 CK_A(sum0)。
     */
    class Checksum
    {
    public:
      uint16_t Compute(uint8_t const *const data, const size_t len)
      {
        if (!data)
        {
          return 0;
        }
        sum0_ = 0;
        sum1_ = 0;
        for (size_t i = 0; i < len; i++)
        {
          sum0_ += data[i];   // CK_A：逐字节累加
          sum1_ += sum0_;     // CK_B：对 CK_A 再累加
        }
        return static_cast<uint16_t>(sum1_) << 8 | sum0_;
      }
      uint16_t Update(uint8_t const *const data, const size_t len)
      {
        if (!data)
        {
          return 0;
        }
        for (size_t i = 0; i < len; i++)
        {
          sum0_ += data[i];
          sum1_ += sum0_;
        }
        return static_cast<uint16_t>(sum1_) << 8 | sum0_;
      }

    private:
      uint8_t sum0_, sum1_;  // CK_A / CK_B 累加器
    } chksum_rx_, chksum_tx_;  // 接收/发送各一份

    /* 通用接收缓冲：先按未知报文收进来，校验通过后再按 cls/id memcpy 到下方具体结构体。 */
    struct UbxMsg
    {
      uint8_t cls;
      uint8_t id;
      uint16_t len;
      uint8_t payload[UBX_MAX_PAYLOAD_];
    } UBX_PACKED rx_msg_;

    /* ---- 各 NAV 消息的最新一帧缓存（解析时按 cls/id 写入，组装 epoch 时读取）---- */
    UbxNavDop ubx_nav_dop_;                // 精度衰减因子
    UbxNavEoe ubx_nav_eoe_;                // 历元结束标志
    UbxNavHpposecef ubx_nav_hp_pos_ecef_;  // 高精度 ECEF 位置
    UbxNavHpposllh ubx_nav_hp_pos_llh_;    // 高精度经纬高位置
    UbxNavPosecef ubx_nav_pos_ecef_;       // 标准精度 ECEF 位置
    UbxNavRelposned ubx_nav_rel_pos_ned_;  // 相对定位 NED
    UbxNavVelecef ubx_nav_vel_ecef_;       // ECEF 速度
    UbxNavPvt ubx_nav_pvt_;                // 位置/速度/时间（核心）
    UbxNavTimegps ubx_nav_time_gps_;       // GPS 时间
    UbxNavSvin ubx_nav_svin_;              // Survey-in 状态
  };

} // namespace bfs

#endif // UBX_SRC_UBX_H_ NOLINT
