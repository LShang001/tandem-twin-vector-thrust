/*
 * ubx-gnss —— u-blox UBX 协议本地重构解析库
 *
 * 本文件源自 Bolder Flight Systems 的 ublox 库，但已在本仓库内做过较多本地重构，
 * 与官方原版不再一致。保留以下原始 MIT 版权声明仅为遵守许可证要求。
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

#ifndef SRC_UBX_NAV_H_
#define SRC_UBX_NAV_H_

#if !defined(ARDUINO)
#include <cstdint>
#include <cstddef>
#endif
#include "ubx_defs.h"  // NOLINT  —— U1/I2/X4 等类型别名与 UBX_NAV_CLS_

namespace bfs
{
  /*
   * ============================ UBX-NAV 消息定义 ============================
   *
   * 本文件按 u-blox《Interface Description》逐条定义 UBX-NAV（导航解算结果）报文的
   * 内存布局。每个结构体的约定：
   *   - cls / id：消息的 Class 与 ID，用于在解析时识别报文类型（见 ubx.cpp）；
   *   - len：协议规定的 payload 字节数。收到报文后先比对 len 再 memcpy，长度不符直接丢弃，
   *          这是防止把变长/异常报文拷进定长结构体导致字段错位的关键防线；
   *   - payload：与协议字节序逐字段对齐的结构体，串口 payload 会被原样 memcpy 进来。
   *
   * payload 内字段的注释统一标注【单位 / 缩放因子】。缩放因子表示“原始整数 × 因子 = 物理量”，
   * 例如经纬度 scale 1e-7 表示报文里的 int32 要乘 1e-7 才是“度”。实际换算见 ubx.cpp::ProcessNavData。
   *
   * 【本库实际解析】的消息：POSECEF / PVT / DOP / VELECEF / HPPOSECEF / HPPOSLLH /
   *   TIMEGPS / SVIN / RELPOSNED / EOE —— 它们在 Ubx 类里有对应成员并被组装成 epoch。
   * 其余结构体（CLOCK / GEOFENCE / ODO / ORB / POSLLH / SAT / SBAS / SIG / SLAS /
   *   STATUS / TIMEx / VELNED 等）仅照协议定义保留，便于诊断识别和将来扩展，当前不参与解析。
   */

  /* ---- UBX-NAV 各消息的 ID（Class 固定为 UBX_NAV_CLS_=0x01）---- */
  static constexpr uint8_t UBX_NAV_CLOCK_ID_ = 0x22;      // 接收机时钟解
  static constexpr uint8_t UBX_NAV_DOP_ID_ = 0x04;        // 精度衰减因子 DOP【本库解析】
  static constexpr uint8_t UBX_NAV_EOE_ID_ = 0x61;        // 历元结束 End-Of-Epoch【本库解析】
  static constexpr uint8_t UBX_NAV_GEOFENCE_ID_ = 0x39;   // 地理围栏状态
  static constexpr uint8_t UBX_NAV_HPPOSECEF_ID_ = 0x13;  // 高精度 ECEF 位置【本库解析】
  static constexpr uint8_t UBX_NAV_HPPOSLLH_ID_ = 0x14;   // 高精度经纬高位置【本库解析】
  static constexpr uint8_t UBX_NAV_ODO_ID_ = 0x09;        // 里程计
  static constexpr uint8_t UBX_NAV_ORB_ID_ = 0x34;        // 星历/历书概览
  static constexpr uint8_t UBX_NAV_POSECEF_ID_ = 0x01;    // ECEF 位置【本库解析】
  static constexpr uint8_t UBX_NAV_POSLLH_ID_ = 0x02;     // 经纬高位置（标准精度）
  static constexpr uint8_t UBX_NAV_PVT_ID_ = 0x07;        // 位置/速度/时间合一【本库核心】
  static constexpr uint8_t UBX_NAV_RELPOSNED_ID_ = 0x3c;  // 相对定位 NED（RTK/双天线）【本库解析】
  static constexpr uint8_t UBX_NAV_RESETODO_ID_ = 0x10;   // 复位里程计（命令）
  static constexpr uint8_t UBX_NAV_SAT_ID_ = 0x35;        // 逐卫星状态
  static constexpr uint8_t UBX_NAV_SBAS_ID_ = 0x32;       // SBAS 状态
  static constexpr uint8_t UBX_NAV_SIG_ID_ = 0x43;        // 逐信号状态
  static constexpr uint8_t UBX_NAV_SLAS_ID_ = 0x42;       // QZSS-SLAS 状态
  static constexpr uint8_t UBX_NAV_STATUS_ID_ = 0x03;     // 导航状态摘要
  static constexpr uint8_t UBX_NAV_SVIN_ID_ = 0x3b;       // 基站 Survey-in 状态【本库解析】
  static constexpr uint8_t UBX_NAV_TIMEBDS_ID_ = 0x24;    // 北斗时间
  static constexpr uint8_t UBX_NAV_TIMEGAL_ID_ = 0x25;    // Galileo 时间
  static constexpr uint8_t UBX_NAV_TIMEGLO_ID_ = 0x23;    // GLONASS 时间
  static constexpr uint8_t UBX_NAV_TIMEGPS_ID_ = 0x20;    // GPS 时间（周/周内秒/闰秒）【本库解析】
  static constexpr uint8_t UBX_NAV_TIMELS_ID_ = 0x26;     // 闰秒信息
  static constexpr uint8_t UBX_NAV_TIMEQZSS_ID_ = 0x27;   // QZSS 时间
  static constexpr uint8_t UBX_NAV_TIMEUTC_ID_ = 0x21;    // UTC 时间
  static constexpr uint8_t UBX_NAV_VELECEF_ID_ = 0x11;    // ECEF 速度【本库解析】
  static constexpr uint8_t UBX_NAV_VELNED_ID_ = 0x12;     // NED 速度（标准精度）

  /*
   * UBX-NAV-CLOCK：接收机本地时钟相对 GNSS 系统时间的偏差与漂移估计。
   * 用途：评估晶振质量、做授时；本库当前不解析。
   */
  struct UbxNavClock
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_CLOCK_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;  // GPS 周内时刻【ms】—— 同一历元各 NAV 报文共用，用于对齐
      I4 clk_b;  // 时钟偏差【ns】
      I4 clk_d;  // 时钟漂移【ns/s】
      U4 t_acc;  // 时间精度估计【ns】
      U4 f_acc;  // 频率精度估计【ps/s】
    } payload;
  };

  /*
   * UBX-NAV-DOP：精度衰减因子（Dilution Of Precision），反映卫星几何分布优劣。
   * 数值越小几何越好；所有 DOP 字段缩放因子 0.01（原始 u16 × 0.01 = 实际 DOP 值）。
   * 【本库解析】对应 Ubx::pdop()/hdop()/vdop() 等。注意：很多最小化输出只发 NAV-PVT
   * 而不发 NAV-DOP，此时这些 getter 恒为 0，应改用 Ubx::pvt_pdop()（从 PVT 直接拿 pDOP）。
   */
  struct UbxNavDop
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_DOP_ID_;
    static constexpr uint16_t len = 18;
    struct
    {
      U4 i_tow;  // GPS 周内时刻【ms】
      U2 g_dop;  // 几何 DOP【scale 0.01】
      U2 p_dop;  // 位置 DOP【scale 0.01】
      U2 t_dop;  // 时间 DOP【scale 0.01】
      U2 v_dop;  // 垂直 DOP【scale 0.01】
      U2 h_dop;  // 水平 DOP【scale 0.01】
      U2 n_dop;  // 北向 DOP【scale 0.01】
      U2 e_dop;  // 东向 DOP【scale 0.01】
    } payload;
  };

  /*
   * UBX-NAV-EOE：历元结束标志（End Of Epoch）。
   * u-blox 在一个导航历元的所有 NAV-* 报文发完后，最后补发一帧 EOE。本库以收到 EOE 作为
   * “该历元数据已齐”的同步点：此时才组装并入队一个完整 epoch（见 ubx.cpp::HandleValidMessage）。
   * payload 只有 iTOW，用于和本历元 NAV-PVT 的 iTOW 比对，确认是同一历元、未发生混帧。【本库解析】
   */
  struct UbxNavEoe
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_EOE_ID_;
    static constexpr uint16_t len = 4;
    struct
    {
      U4 i_tow;  // 本历元 GPS 周内时刻【ms】，应与同历元 NAV-PVT.i_tow 一致
    } payload;
  };

  /*
   * UBX-NAV-GEOFENCE：地理围栏状态。变长报文——尾部 fence[] 重复 num_fences 次，
   * 故用模板参数 N 指定最大围栏数，len 为运行期字段而非编译期常量。本库不解析。
   */
  template <size_t N>  // N = 支持的最大围栏数量
  struct UbxNavGeofence
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_GEOFENCE_ID_;
    uint16_t len;  // 变长：随围栏数量变化，运行期确定
    struct
    {
      U4 i_tow;       // GPS 周内时刻【ms】
      U1 version;     // 消息版本
      U1 status;      // 地理围栏功能状态
      U1 num_fences;  // 围栏数量（决定 fence[] 实际有效长度）
      U1 comb_state;  // 所有围栏的组合（逻辑或）状态
      struct
      {
        U1 state;  // 单个围栏状态
        U1 id;     // 围栏 ID
      } fence[N];  // 重复组（重复 num_fences 次）
    } payload;
  };

  /*
   * UBX-NAV-HPPOSECEF：高精度 ECEF 位置。把整数主分量(ecef_*)与高精度小数补偿(ecef_*_hp)
   * 相加可得 mm 级位置：(ecef_x + ecef_x_hp*0.1) cm。【本库解析】，仅当 flags 的 invalid 位为 0 时有效。
   */
  struct UbxNavHpposecef
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_HPPOSECEF_ID_;
    static constexpr uint16_t len = 28;
    struct
    {
      U1 version;       // 消息版本
      U1 reserved0[3];  // 保留对齐
      U4 i_tow;         // GPS 周内时刻【ms】
      I4 ecef_x;        // ECEF X 主分量【cm】
      I4 ecef_y;        // ECEF Y 主分量【cm】
      I4 ecef_z;        // ECEF Z 主分量【cm】
      I1 ecef_x_hp;     // ECEF X 高精度补偿【mm, scale 0.1】
      I1 ecef_y_hp;     // ECEF Y 高精度补偿【mm, scale 0.1】
      I1 ecef_z_hp;     // ECEF Z 高精度补偿【mm, scale 0.1】
      X1 flags;         // 标志位（bit0 = invalidEcef，置 1 表示本帧 ECEF 无效）
      U4 p_acc;         // 位置精度估计【mm, scale 0.1】
    } payload;
  };

  /*
   * UBX-NAV-HPPOSLLH：高精度经纬高位置。lat/lon 主分量 scale 1e-7（度），叠加 *_hp 小数补偿
   * scale 1e-9 可达 mm 级。【本库解析】，flags 的 invalidLlh 位为 0 时有效。
   */
  struct UbxNavHpposllh
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_HPPOSLLH_ID_;
    static constexpr uint16_t len = 36;
    struct
    {
      U1 version;       // 消息版本
      U1 reserved0[2];  // 保留对齐
      X1 flags;         // 标志位（bit0 = invalidLlh）
      U4 i_tow;         // GPS 周内时刻【ms】
      I4 lon;           // 经度主分量【deg, scale 1e-7】
      I4 lat;           // 纬度主分量【deg, scale 1e-7】
      I4 height;        // 椭球高主分量【mm】
      I4 h_msl;         // 海平面高(MSL)主分量【mm】
      I1 lon_hp;        // 经度高精度补偿【deg, scale 1e-9】
      I1 lat_hp;        // 纬度高精度补偿【deg, scale 1e-9】
      I1 height_hp;     // 椭球高高精度补偿【mm, scale 0.1】
      I1 h_msl_hp;      // MSL 高精度补偿【mm, scale 0.1】
      U4 h_acc;         // 水平精度估计【mm, scale 0.1】
      U4 v_acc;         // 垂直精度估计【mm, scale 0.1】
    } payload;
  };

  /*
   * UBX-NAV-ODO：基于多普勒积分的地面里程计。需接收机开启 ODO 功能，本库不解析。
   */
  struct UbxNavOdo
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_ODO_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U1 version;         // 消息版本
      U1 reserved0[3];    // 保留对齐
      U4 i_tow;           // GPS 周内时刻【ms】
      U4 distance;        // 上次复位以来的地面距离【m】
      U4 total_distance;  // 累计地面总距离【m】
      U4 distance_std;    // 地面距离精度(1σ)【m】
    } payload;
  };

  /*
   * UBX-NAV-ORB：各卫星星历/历书可用性概览。变长（sv[] 重复 num_sv 次），本库不解析。
   */
  template <size_t N>  // N = 支持的最大卫星数量
  struct UbxNavOrb
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_ORB_ID_;
    uint16_t len;  // 变长：随卫星数量变化
    struct
    {
      U4 i_tow;         // GPS 周内时刻【ms】
      U1 version;       // 消息版本
      U1 num_sv;        // 卫星数量（决定 sv[] 有效长度）
      U1 reserved0[2];  // 保留对齐
      struct
      {
        U1 gnss_id;    // GNSS 系统 ID
        U1 sv_id;      // 卫星 ID
        X1 sv_flag;    // 信息标志
        X1 eph;        // 星历数据状态
        X1 alm;        // 历书数据状态
        X1 other_orb;  // 其它轨道数据状态
      } sv[N];         // 重复组（重复 num_sv 次）
    } payload;
  };

  /*
   * UBX-NAV-POSECEF：标准精度 ECEF 位置。当未启用 HPPOSECEF 高精度报文时，本库用它取 ECEF 位置。
   * 【本库解析】p_acc 单位 cm（注意与 HPPOSECEF 的 0.1mm 不同）。
   */
  struct UbxNavPosecef
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_POSECEF_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;   // GPS 周内时刻【ms】
      I4 ecef_x;  // ECEF X 坐标【cm】
      I4 ecef_y;  // ECEF Y 坐标【cm】
      I4 ecef_z;  // ECEF Z 坐标【cm】
      U4 p_acc;   // 位置精度估计【cm】
    } payload;
  };

  /*
   * UBX-NAV-POSLLH：标准精度经纬高位置。NAV-PVT 已包含等价字段，本库统一用 PVT，故不解析此报文。
   */
  struct UbxNavPosllh
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_POSLLH_ID_;
    static constexpr uint16_t len = 28;
    struct
    {
      U4 i_tow;   // GPS 周内时刻【ms】
      I4 lon;     // 经度【deg, scale 1e-7】
      I4 lat;     // 纬度【deg, scale 1e-7】
      I4 height;  // 椭球高【mm】
      I4 h_msl;   // 海平面高【mm】
      U4 h_acc;   // 水平精度估计【mm】
      U4 v_acc;   // 垂直精度估计【mm】
    } payload;
  };

  /*
   * UBX-NAV-PVT：位置/速度/时间一体化解算结果，是本库的核心消息。
   * 单条 PVT 即可拿到定位类型、卫星数、UTC 时间、经纬高、NED 速度、地速航向、各类精度估计，
   * 以及位标志 flags/flags2/flags3 和 pDOP。【本库核心解析】，字段换算见 ubx.cpp::ProcessNavData。
   */
  struct UbxNavPvt
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_PVT_ID_;
    static constexpr uint16_t len = 92;
    struct
    {
      U4 i_tow;    // GPS 周内时刻【ms】
      U2 year;     // 年（UTC）
      U1 month;    // 月（UTC）
      U1 day;      // 日（UTC）
      U1 hour;     // 时（UTC）
      U1 min;      // 分（UTC）
      U1 sec;      // 秒（UTC）
      X1 valid;    // 有效性标志：bit0=validDate bit1=validTime bit2=fullyResolved bit3=validMag
      U4 t_acc;    // 时间精度估计(UTC)【ns】
      I4 nano;     // UTC 秒的小数部分【ns】，可为负
      U1 fix_type; // 定位类型：0=无 2=2D 3=3D 4=GNSS+DR 5=仅授时
      X1 flags;    // 定位状态标志：bit0=gnssFixOK bit1=diffSoln bit6..7=carrSoln(0无/1浮点/2固定)
      X1 flags2;   // 附加标志：bit5=confirmedAvai bit6=confirmedDate bit7=confirmedTime
      U1 num_sv;   // 参与解算的卫星数
      I4 lon;      // 经度【deg, scale 1e-7】
      I4 lat;      // 纬度【deg, scale 1e-7】
      I4 height;   // 椭球高【mm】
      I4 h_msl;    // 海平面高(MSL)【mm】
      U4 h_acc;    // 水平精度估计【mm】
      U4 v_acc;    // 垂直精度估计【mm】
      I4 vel_n;    // NED 北向速度【mm/s】
      I4 vel_e;    // NED 东向速度【mm/s】
      I4 vel_d;    // NED 地向(向下)速度【mm/s】，正值表示下降
      I4 g_speed;  // 地速(2D 水平合速度)【mm/s】
      I4 head_mot; // 运动航向(2D)【deg, scale 1e-5】，指运动方向而非机头朝向
      U4 s_acc;    // 速度精度估计【mm/s】
      U4 head_acc; // 航向精度估计【deg, scale 1e-5】
      U2 p_dop;    // 位置 DOP【scale 0.01】—— 无需 NAV-DOP 即可拿到，本库用作 pvt_pdop()
      X2 flags3;   // 附加标志（手册 offset 78, X2 = 2字节）：
                    //   bit0=invalidLlh（置 1 表示经纬高无效）
                    //   bits[4:1]=lastCorrectionAge（RTK 改正龄期）
      U1 reserved0[4];  // 保留对齐（手册 offset 80, U1[4]）
      I4 head_veh; // 车体航向(2D)【deg, scale 1e-5】，需车体航向功能
      I2 mag_dec;  // 磁偏角【deg, scale 1e-2】
      U2 mag_acc;  // 磁偏角精度【deg, scale 1e-2】
    } payload;
  };

  /*
   * UBX-NAV-RELPOSNED：相对定位（本机相对参考站/另一天线）的 NED 矢量，用于 RTK 基线与双天线测向。
   * 主分量(cm) + *_hp 高精度补偿(0.1mm) 叠加得高精度基线；flags 携带可用性/航向有效/移动基站等状态位。
   * 【本库解析】，仅当收到本报文且 flags 的 relPosValid 位有效时才更新 Ubx::rel_pos_* 系列输出。
   */
  struct UbxNavRelposned
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_RELPOSNED_ID_;
    static constexpr uint16_t len = 64;
    struct
    {
      U1 version;            // 消息版本
      U1 reserved0;          // 保留
      U2 ref_station_id;     // 参考站 ID
      U4 i_tow;              // GPS 周内时刻【ms】
      I4 rel_pos_n;          // 相对位置北向分量【cm】
      I4 rel_pos_e;          // 相对位置东向分量【cm】
      I4 rel_pos_d;          // 相对位置地向分量【cm】
      I4 rel_pos_length;     // 相对位置矢量长度(基线长)【cm】
      I4 rel_pos_heading;    // 相对位置矢量航向【deg, scale 1e-5】
      U1 reserved1[4];       // 保留对齐
      I1 rel_pos_hp_n;       // 北向高精度补偿【mm, scale 0.1】
      I1 rel_pos_hp_e;       // 东向高精度补偿【mm, scale 0.1】
      I1 rel_pos_hp_d;       // 地向高精度补偿【mm, scale 0.1】
      I1 rel_pos_hp_length;  // 基线长高精度补偿【mm, scale 0.1】
      U4 acc_n;              // 北向分量精度【mm, scale 0.1】
      U4 acc_e;              // 东向分量精度【mm, scale 0.1】
      U4 acc_d;              // 地向分量精度【mm, scale 0.1】
      U4 acc_length;         // 基线长精度【mm, scale 0.1】
      U4 acc_heading;        // 航向精度【deg, scale 1e-5】
      U1 reserved2[4];       // 保留对齐
      X4 flags;              // 标志位：bit2=relPosValid bit5=isMoving bit6=refPosMiss
                             //         bit7=refObsMiss bit8=relPosHeadingValid bit9=relPosNormalized
    } payload;
  };

  /*
   * UBX-NAV-RESETODO：复位里程计的命令报文（无 payload）。仅用于发送，本库不接收解析。
   */
  struct UbxNavResetodo
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_RESETODO_ID_;
    static constexpr uint16_t len = 0;
  };

  /*
   * UBX-NAV-SAT：逐卫星详细状态（信噪比、仰角、方位角、伪距残差等）。变长，调试/可视化用，本库不解析。
   */
  template <size_t N>  // N = 支持的最大卫星数量
  struct UbxNavSat
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_SAT_ID_;
    uint16_t len;  // 变长：随卫星数量变化
    struct
    {
      U4 i_tow;         // GPS 周内时刻【ms】
      U1 version;       // 消息版本
      U1 num_sv;        // 卫星数量（决定 sv[] 有效长度）
      U1 reserved0[2];  // 保留对齐
      struct
      {
        U1 gnss_id;  // GNSS 系统 ID
        U1 sv_id;    // 卫星 ID
        U1 cno;      // 载噪比 C/N0【dBHz】
        I1 elev;     // 仰角【deg】
        I2 azim;     // 方位角【deg】
        I2 pr_res;   // 伪距残差【m, scale 0.1】
        X4 flags;    // 标志位
      } sv[N];       // 重复组（重复 num_sv 次）
    } payload;
  };

  /*
   * UBX-NAV-SBAS：SBAS（星基增强）状态与改正信息。变长，本库不解析。
   */
  template <size_t N>  // N = 支持的最大卫星数量
  struct UbxNavSbas
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_SBAS_ID_;
    uint16_t len;  // 变长
    struct
    {
      U4 i_tow;         // GPS 周内时刻【ms】
      U1 geo;           // 改正数据来源 GEO 卫星的 PRN
      U1 mode;          // SBAS 工作模式
      I1 sys;           // SBAS 系统
      X1 service;       // 可用的 SBAS 服务
      U1 cnt;           // 后随的卫星数据条数
      U1 reserved0[3];  // 保留对齐
      struct
      {
        U1 sv_id;       // 卫星 ID
        U1 flags;       // 该卫星标志
        U1 udre;        // 监测状态(UDRE)
        U1 sv_sys;      // 系统
        U1 sv_service;  // 可用服务
        U1 reserved1;   // 保留
        I2 prc;         // 伪距改正【cm】
        U1 reserved2[2];// 保留
        I2 ic;          // 电离层改正【cm】
      } sv[N];          // 重复组（重复 cnt 次）
    } payload;
  };

  /*
   * UBX-NAV-SIG：逐信号状态（一颗卫星可有多个频点信号）。变长，本库不解析。
   */
  template <size_t N>  // N = 支持的最大信号数量
  struct UbxNavSig
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_SIG_ID_;
    uint16_t len;  // 变长
    struct
    {
      U4 i_tow;         // GPS 周内时刻【ms】
      U1 version;       // 消息版本
      U1 num_sigs;      // 信号数量（决定 sig[] 有效长度）
      U1 reserved0[2];  // 保留对齐
      struct
      {
        U1 gnss_id;      // GNSS 系统 ID
        U1 sv_id;        // 卫星 ID
        U1 sig_id;       // 信号 ID
        U1 freq_id;      // GLONASS 频隙号
        I2 pr_res;       // 伪距残差【m, scale 0.1】
        U1 cno;          // 载噪比 C/N0【dBHz】
        U1 quality_ind;  // 信号质量指示
        U1 corr_source;  // 改正来源
        U1 iono_model;   // 电离层模型
        X2 sig_flags;    // 信号相关标志
        U1 reserved1[4]; // 保留对齐
      } sig[N];          // 重复组（重复 num_sigs 次）
    } payload;
  };

  /*
   * UBX-NAV-SLAS：QZSS-SLAS（亚米级增强）状态。变长，本库不解析。
   */
  template <size_t N>  // N = 支持的最大改正数量
  struct UbxNavSlas
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_SLAS_ID_;
    uint16_t len;  // 变长
    struct
    {
      U4 i_tow;          // GPS 周内时刻【ms】
      U1 version;        // 消息版本
      U1 reserved0[3];   // 保留对齐
      I4 gms_lon;        // 地面监测站经度【deg, scale 1e-3】
      I4 gms_lat;        // 地面监测站纬度【deg, scale 1e-3】
      U1 gms_code;       // 地面监测站编码
      U1 qzss_sc_id;     // 所用 QZSS/GEO 改正数据的卫星 ID
      X1 service_flags;  // SLAS 服务标志
      U1 cnt;            // 后随的伪距改正条数
      struct
      {
        U1 gnss_id;       // GNSS 系统 ID
        U1 sv_id;         // 卫星 ID
        U1 reserved1;     // 保留
        U1 reserved2[3];  // 保留
        I2 prc;           // 伪距改正【cm】
      } corr[N];          // 重复组（重复 cnt 次）
    } payload;
  };

  /*
   * UBX-NAV-STATUS：导航状态摘要（定位类型、首次定位时间 TTFF 等）。本库改用 NAV-PVT 取状态，故不解析。
   */
  struct UbxNavStatus
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_STATUS_ID_;
    static constexpr uint16_t len = 16;
    struct
    {
      U4 i_tow;     // GPS 周内时刻【ms】
      U1 gps_fix;   // 定位类型
      X1 flags;     // 导航状态标志
      X1 fix_stat;  // 定位状态信息
      X1 flags2;    // 导航输出附加信息
      U4 ttff;      // 首次定位耗时(TTFF)【ms】
      U4 msss;      // 自启动以来的毫秒数【ms】
    } payload;
  };

  /*
   * UBX-NAV-SVIN：基站 Survey-in（定点自勘测）状态。固定基站架设时用来观测平均自身坐标，
   * 收敛后(valid=1)可作为 RTK 基准坐标。【本库解析】对应 Ubx::svin_* 系列输出。
   */
  struct UbxNavSvin
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_SVIN_ID_;
    static constexpr uint16_t len = 40;
    struct
    {
      U1 version;       // 消息版本
      U1 reserved0[3];  // 保留对齐
      U4 i_tow;         // GPS 周内时刻【ms】
      U4 dur;           // 已进行的 Survey-in 观测时长【s】
      I4 mean_x;        // 当前平均 ECEF X 主分量【cm】
      I4 mean_y;        // 当前平均 ECEF Y 主分量【cm】
      I4 mean_z;        // 当前平均 ECEF Z 主分量【cm】
      I1 mean_x_hp;     // ECEF X 高精度补偿【mm, scale 0.1】
      I1 mean_y_hp;     // ECEF Y 高精度补偿【mm, scale 0.1】
      I1 mean_z_hp;     // ECEF Z 高精度补偿【mm, scale 0.1】
      U1 reserved1;     // 保留
      U4 mean_acc;      // 当前 Survey-in 精度【mm, scale 0.1】
      U4 obs;           // Survey-in 期间使用的观测次数
      U1 valid;         // Survey-in 位置是否有效(1=有效)
      U1 active;        // Survey-in 是否进行中(1=进行中)
      U1 reserved2[2];  // 保留对齐
    } payload;
  };

  /*
   * UBX-NAV-TIMEBDS：北斗系统时间。本库不解析（统一用 NAV-TIMEGPS 取 GPS 时间）。
   */
  struct UbxNavTimebds
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEBDS_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;   // GPS 周内时刻【ms】
      U4 sow;     // 北斗周内秒【s】
      I4 f_sow;   // 周内秒小数部分【ns】
      I2 week;    // 北斗周数
      I1 leap_s;  // 北斗闰秒
      X1 valid;   // 有效性标志
      U4 t_acc;   // 时间精度估计【ns】
    } payload;
  };

  /*
   * UBX-NAV-TIMEGAL：Galileo 系统时间。本库不解析。
   */
  struct UbxNavTimegal
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEGAL_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;      // GPS 周内时刻【ms】
      U4 gal_tow;    // Galileo 周内秒【s】
      I4 f_gal_tow;  // 周内秒小数部分【ns】
      I2 gal_wno;    // Galileo 周数
      I1 leap_s;     // Galileo 闰秒
      X1 valid;      // 有效性标志
      U4 t_acc;      // 时间精度估计【ns】
    } payload;
  };

  /*
   * UBX-NAV-TIMEGLO：GLONASS 系统时间。本库不解析。
   */
  struct UbxNavTimeglo
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEGLO_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;  // GPS 周内时刻【ms】
      U4 tod;    // GLONASS 当日时刻【s】
      I4 f_tod;  // 当日时刻小数部分【ns】
      U2 nt;     // 四年周期内的日序（从 n4 的 1 月 1 日起，从 1 计）
      U1 n4;     // 四年周期编号
      X1 valid;  // 有效性标志
      U4 t_acc;  // 时间精度估计【ns】
    } payload;
  };

  /*
   * UBX-NAV-TIMEGPS：GPS 系统时间——周内时刻(iTOW)、周内秒小数(fTOW)、周数(week)与闰秒(leapS)。
   * 【本库解析】用于输出 gps_tow_s()/gps_week()/leap_s()。注意 valid 三个位分别标记 iTOW/week/leapS 有效性，
   * 若链路未开本报文，本库会退而用 NAV-PVT.iTOW 充当周内时刻（见 ubx.cpp::ProcessNavData）。
   */
  struct UbxNavTimegps
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEGPS_ID_;
    static constexpr uint16_t len = 16;
    struct
    {
      U4 i_tow;   // GPS 周内时刻【ms】
      I4 f_tow;   // 周内时刻小数部分【ns】，与 i_tow 相加得高精度 TOW
      I2 week;    // GPS 周数
      I1 leap_s;  // GPS 闰秒(GPS-UTC)
      X1 valid;   // 有效性标志：bit0=towValid bit1=weekValid bit2=leapSValid
      U4 t_acc;   // 时间精度估计【ns】
    } payload;
  };

  /*
   * UBX-NAV-TIMELS：闰秒详细信息（当前闰秒来源、下次闰秒变化时刻等）。本库不解析。
   */
  struct UbxNavTimels
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMELS_ID_;
    static constexpr uint16_t len = 24;
    struct
    {
      U4 i_tow;              // GPS 周内时刻【ms】
      U1 version;            // 消息版本
      U1 reserved0[3];       // 保留对齐
      U1 src_of_curr_ls;     // 当前闰秒数的信息来源
      I1 curr_ls;            // 当前闰秒数
      U1 src_of_ls_change;   // 未来闰秒变化的信息来源
      I1 ls_change;          // 未来闰秒变化量
      I4 time_to_ls_event;   // 距闰秒变化事件的秒数
      U2 date_of_ls_gps_wn;  // 闰秒变化所在 GPS 周数
      U2 date_of_ls_gps_dn;  // 闰秒变化所在 GPS 周内日
      U1 reserved1[3];       // 保留对齐
      X1 valid;              // 有效性标志
    } payload;
  };

  /*
   * UBX-NAV-TIMEQZSS：QZSS 系统时间。本库不解析。
   */
  struct UbxNavTimeqzss
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEQZSS_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;       // GPS 周内时刻【ms】
      U4 qzss_tow;    // QZSS 周内秒【s】
      I4 f_qzss_tow;  // 周内秒小数部分【ns】
      I2 qzss_wno;    // QZSS 周数
      I1 leap_s;      // QZSS 闰秒【s】
      X1 valid;       // 有效性标志
      U4 t_acc;       // 时间精度估计【ns】
    } payload;
  };

  /*
   * UBX-NAV-TIMEUTC：UTC 时间。NAV-PVT 已含 UTC 年月日时分秒，本库统一用 PVT，故不解析此报文。
   */
  struct UbxNavTimeutc
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_TIMEUTC_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;  // GPS 周内时刻【ms】
      U4 t_acc;  // 时间精度估计(UTC)【ns】
      I4 nano;   // UTC 秒的小数部分【ns】
      U2 year;   // 年（UTC）
      U1 month;  // 月（UTC）
      U1 day;    // 日（UTC）
      U1 hour;   // 时（UTC）
      U1 min;    // 分（UTC）
      U1 sec;    // 秒（UTC）
      X1 valid;  // 有效性标志
    } payload;
  };

  /*
   * UBX-NAV-VELECEF：ECEF 三轴速度。【本库解析】对应 ecef_vel_x/y/z_mps()，注意单位 cm/s。
   */
  struct UbxNavVelecef
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_VELECEF_ID_;
    static constexpr uint16_t len = 20;
    struct
    {
      U4 i_tow;     // GPS 周内时刻【ms】
      I4 ecef_v_x;  // ECEF X 速度【cm/s】
      I4 ecef_v_y;  // ECEF Y 速度【cm/s】
      I4 ecef_v_z;  // ECEF Z 速度【cm/s】
      U4 s_acc;     // 速度精度估计【cm/s】
    } payload;
  };

  /*
   * UBX-NAV-VELNED：NED 速度（标准精度）。NAV-PVT 已含 NED 速度与地速航向，本库统一用 PVT，故不解析。
   */
  struct UbxNavVelned
  {
    static constexpr uint8_t cls = UBX_NAV_CLS_;
    static constexpr uint8_t id = UBX_NAV_VELNED_ID_;
    static constexpr uint16_t len = 36;
    struct
    {
      U4 i_tow;    // GPS 周内时刻【ms】
      I4 vel_n;    // 北向速度【cm/s】
      I4 vel_e;    // 东向速度【cm/s】
      I4 vel_d;    // 地向(向下)速度【cm/s】
      U4 speed;    // 三维速度【cm/s】
      U4 g_speed;  // 地速(2D)【cm/s】
      I4 heading;  // 运动航向(2D)【deg, scale 1e-5】
      U4 s_acc;    // 速度精度估计【cm/s】
      U4 c_acc;    // 航向精度估计【deg, scale 1e-5】
    } payload;
  };

} // namespace bfs

#endif // SRC_UBX_NAV_H_
