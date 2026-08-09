/*
 * ubx-gnss —— u-blox UBX 协议本地重构解析库
 *
 * 本文件源自 Bolder Flight Systems 的 ublox 库，但已在本仓库内做过较多本地重构，
 * 与官方原版不再一致（epoch 队列、泵入式解析、链路统计、BeginConfigured 等均为本地新增）。
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

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include <cstring>     // memcpy / memset
#include <math.h>      // cosf / sinf（NMEA 速度分解）
#include "ubx.h"
#include "ubx_defs.h" // NOLINT
#include "ubx_nav.h"  // NOLINT

namespace bfs
{

  // UBX_HEADER_ 是类内 static constexpr 数组，C++14 下需要在此处给出脱离类的定义，
  // 以便对其取地址/按数组使用（ParseMsg 中按下标访问同步字时依赖该定义）。
  constexpr uint8_t Ubx::UBX_HEADER_[];

  // 绑定/更换通信串口。用默认构造后必须先调用本方法，否则 bus_ 为空指针。
  void Ubx::Config(HardwareSerial *bus)
  {
    bus_ = bus;
  }

  // 把全部解析状态、统计计数、导航数据缓存和 epoch 队列清零，但保留已配置的串口指针，
  // 使对象回到“刚绑定串口、尚未收到任何数据”的初始状态。构造函数与需要重新同步时调用。
  void Ubx::Reset()
  {
    // 暂存并写回串口指针：Reset 不应丢掉已经 Config 好的 bus_。
    HardwareSerial *const configured_bus = bus_;
    bus_ = configured_bus;
    // --- 通信探测与解析状态机 ---
    comm_timeout_count_ = 0;
    c_ = 0;
    len_ = 0;
    chk_rx_ = 0;
    chk_[0] = 0;
    chk_[1] = 0;
    chk_cmp_rx_ = 0;
    chk_cmp_tx_ = 0;
    parser_state_ = 0;
    // --- 链路健康统计计数器 ---
    rx_byte_count_ = 0;
    valid_msg_count_ = 0;
    checksum_fail_count_ = 0;
    oversize_msg_count_ = 0;
    nav_pvt_count_ = 0;
    nav_eoe_count_ = 0;
    eoe_without_pvt_count_ = 0;
    tow_mismatch_count_ = 0;
    pump_receive_time_us_ = 0;
    latest_epoch_tow_ms_ = 0;
    latest_epoch_receive_time_us_ = 0;
    queue_overflow_count_ = 0;
    queued_epoch_count_ = 0;
    last_msg_cls_ = 0;
    last_msg_id_ = 0;
    last_msg_len_ = 0;
    // --- 解析过程中间标志位 ---
    eoe_ = false;
    use_hp_pos_ = false;
    svin_data_ = false;
    rel_pos_data_ = false;
    pvt_ready_this_epoch_ = false;
    fix_ = FIX_NONE;
    gnss_fix_ok_ = false;
    diff_soln_ = false;
    valid_date_ = false;
    valid_time_ = false;
    fully_resolved_ = false;
    validity_confirmed_ = false;
    tow_valid_ = false;
    week_valid_ = false;
    leap_valid_ = false;
    confirmed_date_ = false;
    confirmed_time_ = false;
    valid_time_and_date_ = false;
    // 经纬高/ECEF 默认置为“无效”，等收到有效报文再翻转，避免初始就吐出 (0,0,0)。
    invalid_llh_ = true;
    invalid_ecef_ = true;
    rel_pos_avail_ = false;
    rel_pos_moving_baseline_ = false;
    rel_pos_ref_pos_miss_ = false;
    rel_pos_ref_obs_miss_ = false;
    rel_pos_heading_valid_ = false;
    rel_pos_norm_ = false;
    svin_valid_ = false;
    svin_in_progress_ = false;
    carr_soln_ = 0;
    // --- 已换算导航数据（标量）---
    num_sv_ = 0;
    month_ = 0;
    day_ = 0;
    hour_ = 0;
    min_ = 0;
    sec_ = 0;
    leap_s_ = 0;
    year_ = 0;
    week_ = 0;
    nano_ = 0;
    t_acc_ns_ = 0;
    svin_dur_s_ = 0;
    svin_num_obs_ = 0;
    alt_msl_m_ = 0.0f;
    gnd_spd_mps_ = 0.0f;
    track_deg_ = 0.0f;
    gdop_ = 0.0f;
    pdop_ = 0.0f;
    tdop_ = 0.0f;
    vdop_ = 0.0f;
    hdop_ = 0.0f;
    ndop_ = 0.0f;
    edop_ = 0.0f;
    pvt_pdop_ = 0.0f;
    h_acc_m_ = 0.0f;
    v_acc_m_ = 0.0f;
    p_acc_m_ = 0.0f;
    track_acc_deg_ = 0.0f;
    s_acc_mps_ = 0.0f;
    rel_pos_heading_deg_ = 0.0f;
    rel_pos_heading_acc_deg_ = 0.0f;
    rel_pos_len_acc_m_ = 0.0f;
    svin_acc_m_ = 0.0f;
    rel_pos_len_m_ = 0.0;
    tow_s_ = 0.0;
    // --- 已换算导航数据（数组/向量）---
    memset(ecef_vel_mps_, 0, sizeof(ecef_vel_mps_));
    memset(ned_vel_mps_, 0, sizeof(ned_vel_mps_));
    memset(rel_pos_ned_acc_m_, 0, sizeof(rel_pos_ned_acc_m_));
    memset(ecef_m_, 0, sizeof(ecef_m_));
    memset(llh_, 0, sizeof(llh_));
    memset(rel_pos_ned_m_, 0, sizeof(rel_pos_ned_m_));
    memset(svin_ecef_m_, 0, sizeof(svin_ecef_m_));
    // --- 通用接收缓冲与各 NAV 消息缓存 ---
    memset(&rx_msg_, 0, sizeof(rx_msg_));
    memset(&ubx_nav_dop_, 0, sizeof(ubx_nav_dop_));
    memset(&ubx_nav_eoe_, 0, sizeof(ubx_nav_eoe_));
    memset(&ubx_nav_hp_pos_ecef_, 0, sizeof(ubx_nav_hp_pos_ecef_));
    memset(&ubx_nav_hp_pos_llh_, 0, sizeof(ubx_nav_hp_pos_llh_));
    memset(&ubx_nav_pos_ecef_, 0, sizeof(ubx_nav_pos_ecef_));
    memset(&ubx_nav_rel_pos_ned_, 0, sizeof(ubx_nav_rel_pos_ned_));
    memset(&ubx_nav_vel_ecef_, 0, sizeof(ubx_nav_vel_ecef_));
    memset(&ubx_nav_pvt_, 0, sizeof(ubx_nav_pvt_));
    memset(&ubx_nav_time_gps_, 0, sizeof(ubx_nav_time_gps_));
    memset(&ubx_nav_svin_, 0, sizeof(ubx_nav_svin_));
    // --- epoch 环形队列 ---
    // UbxEpoch 带默认成员初始化器（= 0）属 non-trivial 类型，不能用 memset 清零
    // （会触发 -Wclass-memaccess）。改用值初始化逐个重置，语义等价且类型安全。
    for (uint8_t i = 0; i < kEpochQueueCapacity; i++)
    {
      epoch_queue_[i] = UbxEpoch{};
    }
    epoch_queue_head_ = 0;
    epoch_queue_count_ = 0;
    // --- NMEA 解析状态与快照（协议模式与配置保持不变）---
    ResetNmea();
  }

  // 标准初始化：自己设波特率并清空发送缓冲，再做通信探测。
  // 适合“库自己管串口”的平台；ESP32-P4 这类需上层固定引脚的平台请改用 BeginConfigured()。
  bool Ubx::Begin(int32_t baud)
  {
    bus_->begin(baud);
    bus_->flush();
    return BeginConfigured();
  }

  // 仅做通信探测：在 COMM_TIMEOUT_TRIES_ 次轮询内只要解析出一帧有效 UBX 即认为链路通，
  // 处理该帧后立即返回 true；始终收不到则返回 false。不调用 begin()，不改 UART 引脚映射。
  bool Ubx::BeginConfigured()
  {
    comm_timeout_count_ = 0;
    while (comm_timeout_count_++ < COMM_TIMEOUT_TRIES_)
    {
      if (ParseMsg())
      {
        HandleValidMessage(true);
        return true;
      }
      delay(COMM_TIMEOUT_DELAY_MS_);
    }
    return false;
  }

  // 阻塞式按历元读取（经典用法，不入队列）。
  bool Ubx::Read()
  {
    /*
     * 实时导航按 epoch 处理 GNSS。串口缓冲区里可能已经堆了下一组 NAV-PVT，
     * 因此收到当前组 NAV-EOE 后必须立刻返回，让上层先融合当前 epoch。
     * 若继续把缓冲区读空，下一组 NAV-PVT 会覆盖当前数据，导致漏帧或混帧。
     */
    while (bus_->available())
    {
      if (ParseMsg())
      {
        // queue_output=false：Read() 路径只更新各 getter，不往 epoch 队列推。
        if (HandleValidMessage(false))
        {
          return true;  // 收到 EOE 且本历元完整：立即返回让上层消费。
        }
      }
    }
    return false;
  }

  // Pump 无参版：不限单次字节预算，尽量清空当前串口 backlog。
  bool Ubx::Pump()
  {
    return Pump(0, 0U);
  }

  // Pump 带预算版：本次最多消费约 max_bytes 字节，把组装好的完整 epoch 推入队列。
  // 用于高频导航线程限制单次解析耗时，避免异常 backlog 把固定周期主循环拖爆。
  bool Ubx::Pump(size_t max_bytes)
  {
    return Pump(max_bytes, 0U);
  }

  bool Ubx::Pump(size_t max_bytes, uint32_t receive_time_us)
  {
    pump_receive_time_us_ = receive_time_us;
    const uint32_t before_count = queued_epoch_count_;  // 记录入队前的累计计数，用于判断本次是否新入队
    const uint32_t before_bytes = rx_byte_count_;       // 记录入队前的累计字节，用于核算本次预算用量
    // 协议分派（2026-08-09 双协议）：kNmea 跳过 UBX 状态机，kUbx 跳过 NMEA，kAuto 双流并行。
    const bool ubx_enabled = (protocol_ != GpsProtocol::kNmea);
    const bool nmea_enabled = (protocol_ != GpsProtocol::kUbx);
    while (bus_->available() &&
           (max_bytes == 0U ||
            (rx_byte_count_ - before_bytes) < max_bytes))
    {
      const int raw_byte = bus_->read();
      if (raw_byte < 0)
      {
        break;  // 读到 -1：当前无更多可读字节
      }
      const uint8_t b = static_cast<uint8_t>(raw_byte);
      rx_byte_count_++;
      if (ubx_enabled)
      {
        if (ProcessUbxByte(b))
        {
          if (HandleValidMessage(true))  // 完整历元入队
          {
            nmea_ubx_seen_ = true;      // kAuto 闸门：标记见过 UBX epoch
            last_ubx_epoch_us_ = micros();  // 记录最近 UBX epoch 时刻
          }
        }
      }
      if (nmea_enabled)
      {
        nmeaProcessByte(b);  // NMEA 解析（有效句刷新快照）
      }
      if (byte_tap_fn_)
      {
        byte_tap_fn_(b, byte_tap_ctx_);  // 字节镜像：外部调试/分析用
      }
    }
    // NMEA 兜底合成：kNmea 直接按快照合成；kAuto 需 UBX 失效窗口（内部判断）。
    if (nmea_enabled)
    {
      TryPushNmeaEpoch();
    }
    return queued_epoch_count_ != before_count;  // 本次是否至少入队了一个新 epoch
  }

  // 从环形队列取出最早的一帧 epoch（FIFO）。队列空或入参为空返回 false。
  bool Ubx::PopEpoch(UbxEpoch *epoch)
  {
    if (!epoch || epoch_queue_count_ == 0U)
    {
      return false;
    }
    *epoch = epoch_queue_[epoch_queue_head_];
    // 队头后移一格（环形回绕），元素数减一。
    epoch_queue_head_ = static_cast<uint8_t>((epoch_queue_head_ + 1U) %
                                             kEpochQueueCapacity);
    epoch_queue_count_--;
    return true;
  }

  // 处理一帧已通过校验的 UBX 报文：按 cls/id 分发到对应 NAV 缓存。
  // 仅当本帧是 NAV-EOE 且本历元已收到配对的 NAV-PVT 时返回 true（表示一个完整历元就绪）。
  // queue_output 为 true 时，完整历元还会被打包入 epoch 队列。
  bool Ubx::HandleValidMessage(bool queue_output)
  {
    // 记录最近一帧报文的 cls/id/len，供诊断 getter 观察当前在收什么。
    last_msg_cls_ = rx_msg_.cls;
    last_msg_id_ = rx_msg_.id;
    last_msg_len_ = rx_msg_.len;
    // 只关心 NAV 类，其余 Class 直接忽略。
    if (rx_msg_.cls != UBX_NAV_CLS_)
    {
      return false;
    }
    switch (rx_msg_.id)
    {
    case UBX_NAV_POSECEF_ID_:
    {
      // 长度需与结构体声明完全一致才拷贝，杜绝错位。下同。
      if (rx_msg_.len == ubx_nav_pos_ecef_.len)
      {
        memcpy(&ubx_nav_pos_ecef_.payload, rx_msg_.payload, rx_msg_.len);
      }
      break;
    }
    case UBX_NAV_PVT_ID_:
    {
      if (rx_msg_.len == ubx_nav_pvt_.len)
      {
        if (pvt_ready_this_epoch_)
        {
          pvt_duplicate_in_epoch_count_++;  // 同一历元收到多个 PVT，覆盖前计数
        }
        memcpy(&ubx_nav_pvt_.payload, rx_msg_.payload, rx_msg_.len);
        pvt_ready_this_epoch_ = true;  // 标记本历元已收到 PVT，供 EOE 到来时判定历元完整
        nav_pvt_count_++;
      }
      break;
    }
    case UBX_NAV_DOP_ID_:
    {
      if (rx_msg_.len == ubx_nav_dop_.len)
      {
        memcpy(&ubx_nav_dop_.payload, rx_msg_.payload, rx_msg_.len);
      }
      break;
    }
    case UBX_NAV_VELECEF_ID_:
    {
      if (rx_msg_.len == ubx_nav_vel_ecef_.len)
      {
        memcpy(&ubx_nav_vel_ecef_.payload, rx_msg_.payload, rx_msg_.len);
      }
      break;
    }
    case UBX_NAV_HPPOSECEF_ID_:
    {
      if (rx_msg_.len == ubx_nav_hp_pos_ecef_.len)
      {
        memcpy(&ubx_nav_hp_pos_ecef_.payload, rx_msg_.payload, rx_msg_.len);
        use_hp_pos_ = true;  // 收到高精度位置：ProcessNavData 改用 HP 字段换算
      }
      break;
    }
    case UBX_NAV_HPPOSLLH_ID_:
    {
      if (rx_msg_.len == ubx_nav_hp_pos_llh_.len)
      {
        memcpy(&ubx_nav_hp_pos_llh_.payload, rx_msg_.payload, rx_msg_.len);
        use_hp_pos_ = true;
      }
      break;
    }
    case UBX_NAV_TIMEGPS_ID_:
    {
      if (rx_msg_.len == ubx_nav_time_gps_.len)
      {
        memcpy(&ubx_nav_time_gps_.payload, rx_msg_.payload, rx_msg_.len);
      }
      break;
    }
    case UBX_NAV_SVIN_ID_:
    {
      if (rx_msg_.len == ubx_nav_svin_.len)
      {
        memcpy(&ubx_nav_svin_.payload, rx_msg_.payload, rx_msg_.len);
        svin_data_ = true;  // 标记已收到 Survey-in，ProcessNavData 才会更新 svin_* 输出
      }
      break;
    }
    case UBX_NAV_RELPOSNED_ID_:
    {
      if (rx_msg_.len == ubx_nav_rel_pos_ned_.len)
      {
        memcpy(&ubx_nav_rel_pos_ned_.payload, rx_msg_.payload, rx_msg_.len);
        rel_pos_data_ = true;  // 标记已收到相对定位，ProcessNavData 才会更新 rel_pos_* 输出
      }
      break;
    }
    case UBX_NAV_EOE_ID_:
    {
      // NAV-EOE 是历元同步点：收到它意味着本历元所有 NAV-* 报文已发完。
      if (rx_msg_.len == ubx_nav_eoe_.len)
      {
        memcpy(&ubx_nav_eoe_.payload, rx_msg_.payload, rx_msg_.len);
        nav_eoe_count_++;
        eoe_ = false;
        // 缺 PVT 的孤立 EOE：本历元不完整，记一笔并丢弃，不更新输出也不入队。
        if (!pvt_ready_this_epoch_)
        {
          eoe_without_pvt_count_++;
          return false;
        }
        pvt_ready_this_epoch_ = false;  // 复位，准备下一历元重新累计
        ProcessNavData();               // 把各 NAV 缓存换算成物理量
        // PVT 与 EOE 的 iTOW 必须相等，否则说明发生了混帧（当前 PVT 不属于这个 EOE）。
        if (ubx_nav_pvt_.payload.i_tow != ubx_nav_eoe_.payload.i_tow)
        {
          tow_mismatch_count_++;
          return false;
        }
        if (queue_output)
        {
          PushEpoch(MakeEpoch());  // Pump 路径：完整历元入队
        }
        return true;  // 历元完整且一致：通知上层可消费
      }
      break;
    }
    default:
    {
      // 其它 NAV 子类型（STATUS/SAT/SIG…）本库不处理，忽略。
      break;
    }
    }
    return false;
  }

  // 把当前已处理的导航数据打包成一帧 UbxEpoch 快照（单位已换算好），供入队/出队使用。
  UbxEpoch Ubx::MakeEpoch() const
  {
    UbxEpoch epoch = {};
    epoch.pvt_tow_ms = ubx_nav_pvt_.payload.i_tow;
    epoch.eoe_tow_ms = ubx_nav_eoe_.payload.i_tow;
    epoch.receive_time_us = pump_receive_time_us_;
    epoch.gps_tow_s = tow_s_;
    epoch.fix = static_cast<int8_t>(fix_);
    epoch.num_sv = num_sv_;
    epoch.lat_deg = llh_[0];
    epoch.lon_deg = llh_[1];
    epoch.lat_rad = llh_[0] * DEG2RADl_;  // 预乘弧度，省去导航线程换算
    epoch.lon_rad = llh_[1] * DEG2RADl_;
    epoch.alt_wgs84_m = static_cast<float>(llh_[2]);
    epoch.alt_msl_m = alt_msl_m_;
    epoch.north_vel_mps = ned_vel_mps_[0];
    epoch.east_vel_mps = ned_vel_mps_[1];
    epoch.down_vel_mps = ned_vel_mps_[2];
    epoch.horz_acc_m = h_acc_m_;
    epoch.vert_acc_m = v_acc_m_;
    epoch.spd_acc_mps = s_acc_mps_;
    epoch.pvt_pdop = pvt_pdop_;
    return epoch;
  }

  // 把一帧 epoch 压入环形队列尾部。
  void Ubx::PushEpoch(const UbxEpoch &epoch)
  {
    // 队列已满：丢弃最旧一帧（队头前移），并累加溢出计数。
    // 设计取舍——导航线程长期落后时，宁可保留较新观测，也不阻塞解析线程。
    if (epoch_queue_count_ >= kEpochQueueCapacity)
    {
      epoch_queue_head_ = static_cast<uint8_t>((epoch_queue_head_ + 1U) %
                                               kEpochQueueCapacity);
      epoch_queue_count_--;
      queue_overflow_count_++;
    }
    // 计算尾部写入位置（环形）并放入新帧。
    const uint8_t tail = static_cast<uint8_t>((epoch_queue_head_ + epoch_queue_count_) %
                                             kEpochQueueCapacity);
    epoch_queue_[tail] = epoch;
    latest_epoch_tow_ms_ = epoch.pvt_tow_ms;
    latest_epoch_receive_time_us_ = epoch.receive_time_us;
    epoch_queue_count_++;
    queued_epoch_count_++;
  }

  /* ======================= 协议切换与 NMEA 解析（2026-08-09） ======================= */

  // 切换协议并同步重设串口波特率（serial.begin）。未绑定串口或波特率非法时
  // 仅切换协议并返回 false（防止负值/零值波特率把 UART 配置成非法状态）。
  bool Ubx::SwitchProtocol(GpsProtocol protocol, int32_t baud)
  {
    protocol_ = protocol;
    if (bus_ == nullptr || baud <= 0)
    {
      return false;
    }
    bus_->begin(baud);
    bus_->flush();
    return true;
  }

  // 单字节喂 NMEA 句子切分器：'$' 起始、'\r'/'\n' 结束、超长/串扰 '$' 重新同步。
  // 完整句子（含 *CK 校验）交给 nmeaParseSentence。
  void Ubx::nmeaProcessByte(uint8_t b)
  {
    if (!nmea_in_sentence_)
    {
      if (b == '$')
      {
        nmea_in_sentence_ = true;
        nmea_sentence_len_ = 0;
        nmea_sentence_[nmea_sentence_len_++] = static_cast<char>(b);  // '$' 入缓冲
      }
      return;
    }
    if (b == '\r' || b == '\n')
    {
      // 句子结束：交给解析（切分器只负责组句，校验/有效性由 nmeaParseSentence 判定）
      nmea_in_sentence_ = false;
      if (nmea_sentence_len_ > 0)
      {
        nmea_sentence_[nmea_sentence_len_] = '\0';
        nmeaParseSentence(nmea_sentence_);
      }
      return;
    }
    if (b == '$')
    {
      // 串扰/残留：重新同步到新句子起点
      nmea_sentence_len_ = 0;
      nmea_sentence_[nmea_sentence_len_++] = static_cast<char>(b);
      return;
    }
    if (nmea_sentence_len_ >= (sizeof(nmea_sentence_) - 1))
    {
      nmea_in_sentence_ = false;  // 超长残句：丢弃（防溢出）
      nmea_overflow_count_++;
      return;
    }
    nmea_sentence_[nmea_sentence_len_++] = static_cast<char>(b);
  }

  // 解析一帧完整 NMEA 句子：先过 minmea_check（格式+校验和），再按句子类型分发。
  void Ubx::nmeaParseSentence(const char *sentence)
  {
    nmea_sentence_count_++;
    if (nmea_sentence_len_ <= 4)
    {
      return;  // 空句/残句（如仅 "$"）：不构成坏校验，静默忽略
    }
    if (!minmea_check(sentence, false))
    {
      nmea_bad_checksum_count_++;
      return;
    }
    switch (minmea_sentence_id(sentence, false))
    {
      case MINMEA_SENTENCE_GGA:
      {
        nmea_gga_count_++;
        struct minmea_sentence_gga gga;
        if (minmea_parse_gga(&gga, sentence) && gga.fix_quality >= 1)
        {
          nmeaUpdateSnapshotFromGga(gga);
        }
        break;
      }
      case MINMEA_SENTENCE_RMC:
      {
        // 注意：不用官方 minmea_parse_rmc——u-blox 等真实接收机的 RMC 尾部为
        // `,,A`（variation 空 + FAA mode 'A'），官方格式 "tTcfdfdffDfd" 会把
        // 'A' 当 variation 方向字符而 parse_error（'A' 不在 N/E/S/W 中），整句丢弃。
        // 本库只用到 date 之前的字段，自实现时截断尾部（NMEA 2.3 后字段皆可选）。
        nmea_rmc_count_++;
        char valid = '\0';
        int lat_dir = 0;
        int lon_dir = 0;
        struct minmea_sentence_rmc rmc = {};
        if (minmea_scan(sentence, "tTcfdfdffD", &rmc.type, &rmc.time, &valid,
                        &rmc.latitude, &lat_dir, &rmc.longitude, &lon_dir,
                        &rmc.speed, &rmc.course, &rmc.date) &&
            memcmp(rmc.type.sentence_id, "RMC", 3) == 0)
        {
          rmc.valid = (valid == 'A');
          rmc.latitude.value *= lat_dir;
          rmc.longitude.value *= lon_dir;
          if (rmc.valid)
          {
            nmeaUpdateSnapshotFromRmc(rmc);
          }
        }
        break;
      }
      case MINMEA_SENTENCE_GSA:
      {
        // 多星座接收机（u-blox M8/M9/M10）每个星座各输出一条 GSA（GPGSA/GLGSA…），
        // 只有主 talker GN（组合解）的 PDOP 才是整体位置解的几何质量。
        // 单星座接收机只有一条 GSA（talker=GP 等），作兜底采用。
        nmea_gsa_count_++;
        struct minmea_sentence_gsa gsa;
        char talker[3] = {0};
        const bool got_talker = minmea_talker_id(talker, sentence);
        if (minmea_parse_gsa(&gsa, sentence))
        {
          nmeaUpdateSnapshotFromGsa(gsa, got_talker && talker[0] == 'G' && talker[1] == 'N');
        }
        break;
      }
      default:
        nmea_unknown_count_++;  // 其它句子（GSV/VTG/GLL…）本库不处理
        break;
    }
  }

  // GGA 有效句（fix_quality>=1）：刷新位置/精度/高程快照。
  void Ubx::nmeaUpdateSnapshotFromGga(const struct minmea_sentence_gga &gga)
  {
    const float lat_deg = minmea_tocoord(&gga.latitude);
    const float lon_deg = minmea_tocoord(&gga.longitude);
    const float hdop = minmea_tofloat(&gga.hdop);
    const float alt_msl_m = minmea_tofloat(&gga.altitude);
    const float geoid_m = minmea_tofloat(&gga.height);
    if (isnan(lat_deg) || isnan(lon_deg) || isnan(alt_msl_m))
    {
      return;  // 坐标/高程缺省（空字段）：本句不足以刷新位置快照
    }
    // GGA quality → Fix 枚举（quality 3=PPS 罕见，按 3D 保守处理）：
    //   1=GPS → FIX_3D；2=DGPS → FIX_DGNSS；4=RTK Fixed → FIX_RTK_FIXED；5=RTK Float → FIX_RTK_FLOAT
    int8_t fix = FIX_3D;
    if (gga.fix_quality == 2)
    {
      fix = FIX_DGNSS;
    }
    else if (gga.fix_quality == 4)
    {
      fix = FIX_RTK_FIXED;
    }
    else if (gga.fix_quality == 5)
    {
      fix = FIX_RTK_FLOAT;
    }

    nmea_snapshot_fix_ = fix;
    nmea_snapshot_sv_ = static_cast<int8_t>(gga.satellites_tracked);
    // HDOP 为 float（minmea 精度高于 MicroNMEA 的十分之一整数）。
    nmea_snapshot_hdop_ = hdop;
    nmea_snapshot_lat_rad_ = static_cast<double>(lat_deg) * DEG_TO_RAD;
    nmea_snapshot_lon_rad_ = static_cast<double>(lon_deg) * DEG_TO_RAD;
    // 高程：GGA 给 MSL 高 + 大地水准面高 → WGS84 椭球高 = MSL + geoid。
    nmea_snapshot_alt_valid_ = true;
    nmea_snapshot_alt_msl_m_ = alt_msl_m;
    nmea_snapshot_alt_wgs84_m_ = alt_msl_m + (isnan(geoid_m) ? 0.0f : geoid_m);
    // UTC 时间-of-day → 伪 tow【ms】（含百分秒精度；跨日回绕由下游周内差处理兜底）。
    nmea_snapshot_tow_ms_ =
        ((static_cast<uint32_t>(gga.time.hours) * 3600U +
          static_cast<uint32_t>(gga.time.minutes) * 60U +
          static_cast<uint32_t>(gga.time.seconds)) *
         1000U) +
        static_cast<uint32_t>(gga.time.microseconds) / 1000U;

    nmea_snapshot_valid_ = true;
    nmea_fix_count_++;
    nmea_last_fix_ms_ = millis();
  }

  // RMC 有效句（valid=true）：刷新速度/时间快照（位置 RMC 也带，但以 GGA 为准）。
  void Ubx::nmeaUpdateSnapshotFromRmc(const struct minmea_sentence_rmc &rmc)
  {
    const float spd_knots = minmea_tofloat(&rmc.speed);
    const float course_deg = minmea_tofloat(&rmc.course);
    if (isnan(spd_knots) || isnan(course_deg))
    {
      return;  // 速度/航向缺省：不足以刷新速度快照
    }
    // 对地速度（knots）→ m/s；航向（度，自北顺时针）→ NED 分量。
    // 垂直速度 NMEA 不报告，保持 0（合成 epoch 时 down_vel=0 作弱约束，见 MakeNmeaEpoch）。
    const float spd_mps = spd_knots * 0.514444f;
    const float course_rad = course_deg * DEG_TO_RAD;
    nmea_snapshot_vel_n_mps_ = spd_mps * cosf(course_rad);
    nmea_snapshot_vel_e_mps_ = spd_mps * sinf(course_rad);
    nmea_snapshot_tow_ms_ =
        ((static_cast<uint32_t>(rmc.time.hours) * 3600U +
          static_cast<uint32_t>(rmc.time.minutes) * 60U +
          static_cast<uint32_t>(rmc.time.seconds)) *
         1000U) +
        static_cast<uint32_t>(rmc.time.microseconds) / 1000U;

    nmea_snapshot_valid_ = true;
    nmea_fix_count_++;
    nmea_last_fix_ms_ = millis();
  }

  // NMEA 快照 → UbxEpoch。换算逻辑经宿主机回归测试验证（tools/nmea-host-test）。
  UbxEpoch Ubx::MakeNmeaEpoch() const
  {
    UbxEpoch epoch = {};
    epoch.pvt_tow_ms = nmea_snapshot_tow_ms_;
    epoch.eoe_tow_ms = nmea_snapshot_tow_ms_;
    epoch.receive_time_us = micros();
    epoch.gps_tow_s = static_cast<double>(nmea_snapshot_tow_ms_) * 1e-3;
    epoch.fix = nmea_snapshot_fix_;
    epoch.num_sv = nmea_snapshot_sv_;
    epoch.lat_deg = nmea_snapshot_lat_rad_ * RAD_TO_DEG;
    epoch.lon_deg = nmea_snapshot_lon_rad_ * RAD_TO_DEG;
    epoch.lat_rad = nmea_snapshot_lat_rad_;
    epoch.lon_rad = nmea_snapshot_lon_rad_;
    epoch.alt_wgs84_m = nmea_snapshot_alt_wgs84_m_;
    epoch.alt_msl_m = nmea_snapshot_alt_msl_m_;
    epoch.north_vel_mps = nmea_snapshot_vel_n_mps_;
    epoch.east_vel_mps = nmea_snapshot_vel_e_mps_;
    epoch.down_vel_mps = 0.0f;  // NMEA 无垂直速度
    // 精度估计（保守且诚实，由下游动态权重与 NIS 门控兜底）：
    // - 水平位置：HDOP × UERE（C/A 码单频 ≈2.5m/HDOP 单位）
    // - 垂直/速度精度：0 → 下游按 h_acc×1.5 回退
    // - spd_acc 取动态权重上限 3.0：让 down_vel=0 只作弱约束（若取 0.3 会强拉垂直速度，
    //   爬升阶段垂直通道被 0 观测拉向地平）；水平速度信任度偏保守但诚实
    epoch.horz_acc_m = nmea_snapshot_hdop_ * 2.5f;
    epoch.vert_acc_m = 0.0f;
    epoch.spd_acc_mps = 3.0f;
    // PDOP 来自 GSA（组合解优先，见 nmeaUpdateSnapshotFromGsa）；0=未提供 → 动态权重不缩放
    epoch.pvt_pdop = nmea_snapshot_pdop_;
    return epoch;
  }

  // GSA 句：刷新 PDOP/VDOP 快照（用于合成 epoch 的 pvt_pdop，激活动态权重 DOP 缩放）。
  // 多星座接收机每星座各输出一条 GSA——is_combined（talker=GN，组合解）优先覆盖；
  // 单星座 GSA 仅在尚无任何 PDOP 时兜底（防 GPGSA 覆盖 GNGSA 的组合值）。
  void Ubx::nmeaUpdateSnapshotFromGsa(const struct minmea_sentence_gsa &gsa,
                                      bool is_combined)
  {
    const float pdop = minmea_tofloat(&gsa.pdop);
    const float vdop = minmea_tofloat(&gsa.vdop);
    if (isnan(pdop) || pdop <= 0.0f)
    {
      return;  // 未提供或无效
    }
    if (!is_combined && nmea_snapshot_pdop_ > 0.0f)
    {
      return;  // 已有组合解 PDOP，单星座 GSA 不覆盖
    }
    nmea_snapshot_pdop_ = pdop;
    nmea_snapshot_vdop_ = isnan(vdop) ? 0.0f : vdop;
  }

  // NMEA 兜底合成入队：kNmea 直接合成；kAuto 需 UBX 失效窗口（防混合输出双通道重复融合）。
  // 合成节流语义（2026-08-09 实测校准）：
  //   - 秒键去重：快照伪 tow 未跨秒 → 不合成（防无新句时重复合成同一快照）
  //   - 800ms 节流：GGA/RMC 分属两句、时戳错开（u-blox 同秒、部分模块跨秒），
  //     双句若都触发合成会 2Hz 重复融合同一位置；节流后合成率 = 1Hz，
  //     且合成时刻快照已合并最近 GGA（位置）+ 最近 RMC（速度滞后 ≤1s）
  bool Ubx::TryPushNmeaEpoch()
  {
    if (!nmea_snapshot_valid_)
    {
      return false;
    }
    if (millis() - nmea_last_fix_ms_ > nmea_fix_timeout_ms_)
    {
      return false;  // 链路失效（超时无有效句）
    }
    if (nmea_snapshot_tow_ms_ / 1000U == nmea_last_fused_tow_ms_ / 1000U)
    {
      return false;  // 同秒去重：无新句重复合成同一快照 / 同秒双句只合成一次
    }
    if (nmea_epoch_seen_ &&
        (micros() - nmea_last_epoch_us_) < kNmeaEpochMinIntervalUs)
    {
      return false;  // 800ms 合成节流：GGA+RMC 双句只产生 1Hz epoch
    }
    if (!nmea_snapshot_alt_valid_)
    {
      return false;  // 无 GGA 高程：位置融合缺少垂直信息，拒绝合成（宁缺毋滥）
    }
    if (protocol_ == GpsProtocol::kAuto && nmea_ubx_seen_ &&
        (micros() - last_ubx_epoch_us_) < (nmea_ubx_backoff_ms_ * 1000U))
    {
      return false;  // UBX 链路存活：避免同一接收机的位置被 UBX+NMEA 双通道重复融合
    }
    nmea_last_fused_tow_ms_ = nmea_snapshot_tow_ms_;
    nmea_epoch_seen_ = true;  // 先置标志再记时刻：micros()==0 时 `!=0` 判断会失效
    nmea_last_epoch_us_ = micros();
    PushEpoch(MakeNmeaEpoch());
    nmea_epoch_count_++;
    return true;
  }

  // 清空 NMEA 解析状态与快照（Reset 调用；不改变协议模式与配置）。
  void Ubx::ResetNmea()
  {
    nmea_sentence_len_ = 0;
    nmea_in_sentence_ = false;
    nmea_snapshot_valid_ = false;
    nmea_snapshot_alt_valid_ = false;
    nmea_snapshot_fix_ = 0;
    nmea_snapshot_sv_ = 0;
    nmea_snapshot_hdop_ = 0.0f;
    nmea_snapshot_pdop_ = 0.0f;
    nmea_snapshot_vdop_ = 0.0f;
    nmea_snapshot_lat_rad_ = 0.0;
    nmea_snapshot_lon_rad_ = 0.0;
    nmea_snapshot_alt_wgs84_m_ = 0.0f;
    nmea_snapshot_alt_msl_m_ = 0.0f;
    nmea_snapshot_vel_n_mps_ = 0.0f;
    nmea_snapshot_vel_e_mps_ = 0.0f;
    nmea_snapshot_tow_ms_ = 0;
    nmea_last_fused_tow_ms_ = 0;
    nmea_last_fix_ms_ = 0;
    nmea_sentence_count_ = 0;
    nmea_bad_checksum_count_ = 0;
    nmea_overflow_count_ = 0;
    nmea_gga_count_ = 0;
    nmea_rmc_count_ = 0;
    nmea_gsa_count_ = 0;
    nmea_unknown_count_ = 0;
    nmea_fix_count_ = 0;
    nmea_epoch_count_ = 0;
    nmea_ubx_seen_ = false;
    last_ubx_epoch_us_ = 0;
    nmea_epoch_seen_ = false;
    nmea_last_epoch_us_ = 0;
  }

  // 把各 NAV 缓存里的原始整数按协议缩放因子换算成物理量，写入各数据成员（即各 getter 的返回源）。
  void Ubx::ProcessNavData()
  {
    /* 定位类型：综合 PVT.flags 的 gnssFixOK/diffSoln/carrSoln 与 fixType，映射到本库 Fix 枚举。 */
    gnss_fix_ok_ = ubx_nav_pvt_.payload.flags & 0x01;  // bit0：解算是否在精度范围内
    diff_soln_ = ubx_nav_pvt_.payload.flags & 0x02;    // bit1：是否应用了差分改正
    carr_soln_ = ubx_nav_pvt_.payload.flags >> 6;      // bit6..7：载波相位解 0无/1浮点/2固定
    if (gnss_fix_ok_)
    {
      switch (ubx_nav_pvt_.payload.fix_type)
      {
      case 2:
      {
        fix_ = FIX_2D;
        break;
      }
      case 3:
      {
        // 3D 解再按差分/RTK 状态细分等级（质量递增）。
        fix_ = FIX_3D;
        if (diff_soln_)
        {
          fix_ = FIX_DGNSS;
        }
        if (carr_soln_ == 1)
        {
          fix_ = FIX_RTK_FLOAT;
        }
        if (carr_soln_ == 2)
        {
          fix_ = FIX_RTK_FIXED;
        }
        break;
      }
      default:
      {
        fix_ = FIX_NONE;
        break;
      }
      }
    }
    else
    {
      // gnssFixOK 未置位：即便 fixType 非零也不可信，一律按无定位处理。
      fix_ = FIX_NONE;
    }
    /* 卫星数 */
    num_sv_ = ubx_nav_pvt_.payload.num_sv;
    /* 日期与时间：只有 valid/flags2 的全部确认位都置位，才认为 UTC 时间可信并输出，否则清零。 */
    valid_date_ = ubx_nav_pvt_.payload.valid & 0x01;          // 日期有效
    valid_time_ = ubx_nav_pvt_.payload.valid & 0x02;          // 时间有效
    fully_resolved_ = ubx_nav_pvt_.payload.valid & 0x04;      // UTC 完全解算
    validity_confirmed_ = ubx_nav_pvt_.payload.flags2 & 0x20; // 确认信息可用
    confirmed_date_ = ubx_nav_pvt_.payload.flags2 & 0x40;     // 日期已确认
    confirmed_time_ = ubx_nav_pvt_.payload.flags2 & 0x80;     // 时间已确认
    valid_time_and_date_ = valid_date_ && valid_time_ && fully_resolved_ &&
                           validity_confirmed_ && confirmed_date_ &&
                           confirmed_time_;
    if (valid_time_and_date_)
    {
      year_ = ubx_nav_pvt_.payload.year;
      month_ = ubx_nav_pvt_.payload.month;
      day_ = ubx_nav_pvt_.payload.day;
      hour_ = ubx_nav_pvt_.payload.hour;
      min_ = ubx_nav_pvt_.payload.min;
      sec_ = ubx_nav_pvt_.payload.sec;
      nano_ = ubx_nav_pvt_.payload.nano;
    }
    else
    {
      // 时间未完全确认：清零，避免上层误用半成品时间戳。
      year_ = 0;
      month_ = 0;
      day_ = 0;
      hour_ = 0;
      min_ = 0;
      sec_ = 0;
      nano_ = 0;
    }
    t_acc_ns_ = ubx_nav_pvt_.payload.t_acc;
    /* GPS 时间：来自 NAV-TIMEGPS，各分量按 valid 位逐项判定有效性。 */
    tow_valid_ = ubx_nav_time_gps_.payload.valid & 0x01;   // 周内时刻有效
    week_valid_ = ubx_nav_time_gps_.payload.valid & 0x02;  // 周数有效
    leap_valid_ = ubx_nav_time_gps_.payload.valid & 0x04;  // 闰秒有效
    if (tow_valid_)
    {
      // iTOW(ms) + fTOW(ns) 合成高精度周内秒。
      tow_s_ = static_cast<double>(ubx_nav_time_gps_.payload.i_tow) * 1e-3 +
               static_cast<double>(ubx_nav_time_gps_.payload.f_tow) * 1e-9;
    }
    else
    {
      /*
       * F10N 等实际接入时常见配置只输出 NAV-PVT + NAV-EOE，不一定打开
       * NAV-TIMEGPS。此时 PVT.iTOW 仍是该导航解的 GPS 周内毫秒，足够用于
       * epoch 去重和上层时序诊断，不能把 gps_tow_s() 留成 0。
       */
      tow_s_ = static_cast<double>(ubx_nav_pvt_.payload.i_tow) * 1e-3;
    }
    if (week_valid_)
    {
      week_ = ubx_nav_time_gps_.payload.week;
    }
    else
    {
      week_ = 0;
    }
    if (leap_valid_)
    {
      leap_s_ = ubx_nav_time_gps_.payload.leap_s;
    }
    else
    {
      leap_s_ = 0;
    }
    /* DOP：来自 NAV-DOP，原始 u16 × 0.01 = 实际 DOP 值。接收机未开 NAV-DOP 时这些恒为 0。 */
    gdop_ = static_cast<float>(ubx_nav_dop_.payload.g_dop) * 0.01f;
    pdop_ = static_cast<float>(ubx_nav_dop_.payload.p_dop) * 0.01f;
    tdop_ = static_cast<float>(ubx_nav_dop_.payload.t_dop) * 0.01f;
    vdop_ = static_cast<float>(ubx_nav_dop_.payload.v_dop) * 0.01f;
    hdop_ = static_cast<float>(ubx_nav_dop_.payload.h_dop) * 0.01f;
    ndop_ = static_cast<float>(ubx_nav_dop_.payload.n_dop) * 0.01f;
    edop_ = static_cast<float>(ubx_nav_dop_.payload.e_dop) * 0.01f;
    /* pDOP from NAV-PVT (本地新增)：
     * NAV-PVT payload 自带 p_dop（scale 0.01），与 NAV-DOP 的 pdop_ 同义但无需额外消息。
     * 上面的 pdop_ 仅在接收机配置了 NAV-DOP 输出时才有效；很多最小输出配置只发 NAV-PVT，
     * 此处单独解析 pvt_pdop_，让只有 NAV-PVT 的链路也能拿到几何质量指标做融合质量门限。 */
    pvt_pdop_ = static_cast<float>(ubx_nav_pvt_.payload.p_dop) * 0.01f;
    /* NED 速度：PVT 原始 mm/s → m/s。 */
    ned_vel_mps_[0] = static_cast<float>(ubx_nav_pvt_.payload.vel_n) / 1000.0f;
    ned_vel_mps_[1] = static_cast<float>(ubx_nav_pvt_.payload.vel_e) / 1000.0f;
    ned_vel_mps_[2] = static_cast<float>(ubx_nav_pvt_.payload.vel_d) / 1000.0f;
    s_acc_mps_ = static_cast<float>(ubx_nav_pvt_.payload.s_acc) / 1000.0f;
    /* ECEF 速度：NAV-VELECEF 原始 cm/s → m/s。 */
    ecef_vel_mps_[0] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_x) /
                       100.0f;
    ecef_vel_mps_[1] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_y) /
                       100.0f;
    ecef_vel_mps_[2] = static_cast<float>(ubx_nav_vel_ecef_.payload.ecef_v_z) /
                       100.0f;
    /* 地速与运动航向：g_speed mm/s → m/s；head_mot / head_acc scale 1e-5 → 度。 */
    gnd_spd_mps_ = static_cast<float>(ubx_nav_pvt_.payload.g_speed) / 1000.0f;
    track_deg_ = static_cast<float>(ubx_nav_pvt_.payload.head_mot) / 100000.0f;
    track_acc_deg_ = static_cast<float>(ubx_nav_pvt_.payload.head_acc) /
                     100000.0f;
    /* LLH 经纬高位置：flags3.bit0=invalidLlh 为 1 时本帧无效，保持上一帧值不动。 */
    invalid_llh_ = ubx_nav_pvt_.payload.flags3 & 0x01;
    if (!invalid_llh_)
    {
      if (use_hp_pos_)
      {
        // 高精度：主分量 + *_hp 补偿，再统一缩放到 度 / 米。
        // lat/lon：(1e-7度主分量 + 1e-9度补偿) → double 度；禁止中途转 float，
        // 否则绝对经纬度会被 float32 ULP 量化，轨迹会出现经度台阶。
        // 写成 (lat + lat_hp*1e-2)*1e-7 与 lat*1e-7 + lat_hp*1e-9 等价。
        llh_[0] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.lat) +
                   static_cast<double>(ubx_nav_hp_pos_llh_.payload.lat_hp) *
                       1e-2) *
                  1e-7;
        llh_[1] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.lon) +
                   static_cast<double>(ubx_nav_hp_pos_llh_.payload.lon_hp) *
                       1e-2) *
                  1e-7;
        // 椭球高：(mm主分量 + 0.1mm补偿) → m。
        llh_[2] = (static_cast<double>(ubx_nav_hp_pos_llh_.payload.height) +
                   static_cast<double>(ubx_nav_hp_pos_llh_.payload.height_hp) *
                       0.1) *
                  1e-3;
        alt_msl_m_ = (static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_msl) +
                      static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_msl_hp) *
                          0.1f) /
                     1000.0f;
        // 高精度精度估计单位是 0.1mm，故除以 10000 得 m。
        h_acc_m_ = static_cast<float>(ubx_nav_hp_pos_llh_.payload.h_acc) /
                   10000.0f;
        v_acc_m_ = static_cast<float>(ubx_nav_hp_pos_llh_.payload.v_acc) /
                   10000.0f;
      }
      else
      {
        // 标准精度：直接用 PVT 的 lat/lon(1e-7度) / height(mm) / h_acc(mm)。
        // NAV-PVT 虽然没有 HP 补偿，也必须转成 double 保存，避免显示/传输层 float32 反向污染解析层。
        llh_[0] = static_cast<double>(ubx_nav_pvt_.payload.lat) * 1e-7;
        llh_[1] = static_cast<double>(ubx_nav_pvt_.payload.lon) * 1e-7;
        llh_[2] = static_cast<double>(ubx_nav_pvt_.payload.height) * 1e-3;
        alt_msl_m_ = static_cast<float>(ubx_nav_pvt_.payload.h_msl) / 1000.0f;
        h_acc_m_ = static_cast<float>(ubx_nav_pvt_.payload.h_acc) / 1000.0f;
        v_acc_m_ = static_cast<float>(ubx_nav_pvt_.payload.v_acc) / 1000.0f;
      }
    }
    /* ECEF 位置：invalidEcef 为 1 时本帧无效。HPPOSECEF 单位 0.1mm，POSECEF 单位 cm。 */
    invalid_ecef_ = ubx_nav_hp_pos_ecef_.payload.flags & 0x01;
    if (!invalid_ecef_)
    {
      if (use_hp_pos_)
      {
        // 高精度：(cm主分量 + 0.1mm补偿) → m，即 (ecef + ecef_hp*1e-2) cm * 1e-2。
        ecef_m_[0] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_x) +
                      static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_x_hp) * 1e-2) *
                     1e-2;
        ecef_m_[1] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_y) +
                      static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_y_hp) * 1e-2) *
                     1e-2;
        ecef_m_[2] = (static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_z) +
                      static_cast<double>(ubx_nav_hp_pos_ecef_.payload.ecef_z_hp) * 1e-2) *
                     1e-2;
        p_acc_m_ = static_cast<float>(ubx_nav_hp_pos_ecef_.payload.p_acc) /
                   10000.0f;
      }
      else
      {
        // 标准精度：POSECEF 主分量 cm → m，p_acc cm → m。
        ecef_m_[0] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_x) * 1e-2;
        ecef_m_[1] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_y) * 1e-2;
        ecef_m_[2] = static_cast<double>(ubx_nav_pos_ecef_.payload.ecef_z) * 1e-2;
        p_acc_m_ = static_cast<float>(ubx_nav_pos_ecef_.payload.p_acc) / 100.0f;
      }
    }
    /* 相对定位：仅在收到过 NAV-RELPOSNED 后处理；先取状态位，再在 relPosValid 时换算矢量。 */
    if (rel_pos_data_)
    {
      rel_pos_avail_ = ubx_nav_rel_pos_ned_.payload.flags & 0x04;          // bit2 relPosValid
      rel_pos_moving_baseline_ = ubx_nav_rel_pos_ned_.payload.flags & 0x20;// bit5 isMoving
      rel_pos_ref_pos_miss_ = ubx_nav_rel_pos_ned_.payload.flags & 0x40;   // bit6 refPosMiss
      rel_pos_ref_obs_miss_ = ubx_nav_rel_pos_ned_.payload.flags & 0x80;   // bit7 refObsMiss
      rel_pos_heading_valid_ = ubx_nav_rel_pos_ned_.payload.flags & 0x100; // bit8 headingValid
      rel_pos_norm_ = ubx_nav_rel_pos_ned_.payload.flags & 0x200;          // bit9 normalized
      if (rel_pos_avail_)
      {
        // NED 各分量：(cm主分量 + 0.1mm补偿) → m。
        rel_pos_ned_m_[0] =
            (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_n) +
             static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_n) * 1e-2) *
            1e-2;
        rel_pos_ned_m_[1] =
            (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_e) +
             static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_e) * 1e-2) *
            1e-2;
        rel_pos_ned_m_[2] =
            (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_d) +
             static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_d) * 1e-2) *
            1e-2;
        // 基线长：同样 (cm + 0.1mm补偿) → m。
        rel_pos_len_m_ =
            (static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_length) +
             static_cast<double>(ubx_nav_rel_pos_ned_.payload.rel_pos_hp_length) *
                 1e-2) *
            1e-2;
        // 航向 scale 1e-5 → 度。
        rel_pos_heading_deg_ =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.rel_pos_heading) /
            100000.0f;
        // 各精度单位 0.1mm → m。
        rel_pos_ned_acc_m_[0] =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_n) / 10000.0f;
        rel_pos_ned_acc_m_[1] =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_e) / 10000.0f;
        rel_pos_ned_acc_m_[2] =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_d) / 10000.0f;
        rel_pos_len_acc_m_ =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_length) / 10000.0f;
        rel_pos_heading_acc_deg_ =
            static_cast<float>(ubx_nav_rel_pos_ned_.payload.acc_heading) /
            100000.0f;
      }
    }
    /* Survey-in：仅在收到过 NAV-SVIN 后处理，换算平均 ECEF 坐标与状态。 */
    if (svin_data_)
    {
      svin_dur_s_ = ubx_nav_svin_.payload.dur;
      // 平均 ECEF：(cm主分量 + 0.1mm补偿) → m。
      svin_ecef_m_[0] = (static_cast<double>(ubx_nav_svin_.payload.mean_x) +
                         static_cast<double>(ubx_nav_svin_.payload.mean_x_hp) * 1e-2) *
                        1e-2;
      svin_ecef_m_[1] = (static_cast<double>(ubx_nav_svin_.payload.mean_y) +
                         static_cast<double>(ubx_nav_svin_.payload.mean_y_hp) * 1e-2) *
                        1e-2;
      svin_ecef_m_[2] = (static_cast<double>(ubx_nav_svin_.payload.mean_z) +
                         static_cast<double>(ubx_nav_svin_.payload.mean_z_hp) * 1e-2) *
                        1e-2;
      svin_acc_m_ = static_cast<float>(ubx_nav_svin_.payload.mean_acc) /
                    10000.0f;
      svin_valid_ = ubx_nav_svin_.payload.valid;
      svin_in_progress_ = ubx_nav_svin_.payload.active;
      svin_num_obs_ = ubx_nav_svin_.payload.obs;
    }
  }

  // UBX 报文解析状态机：从串口逐字节喂入，按 [同步字×2][Class][ID][Len×2][Payload][CK_A][CK_B]
  // 推进 parser_state_。组完整帧且校验通过返回 true（数据在 rx_msg_）；否则继续读，读空/达预算返回 false。
  // parser_state_ 跨调用保留，因此支持把一帧拆到多次调用里解析（配合 Pump 的字节预算）。
  // 阻塞式读字节直到组完一帧有效 UBX 报文（BeginConfigured/Read 路径用）。
  // 不带字节预算、不触发字节镜像（tap 仅 Pump 路径生效）。
  bool Ubx::ParseMsg()
  {
    while (bus_->available())
    {
      const int raw_byte = bus_->read();
      if (raw_byte < 0)
      {
        break;  // 读到 -1：当前无更多可读字节
      }
      rx_byte_count_++;
      if (ProcessUbxByte(static_cast<uint8_t>(raw_byte)))
      {
        return true;  // 一帧有效报文就绪
      }
    }
    return false;  // 串口读空，尚未组成完整有效帧
  }

  // 单字节推进 UBX 解析状态机：按 [同步字×2][Class][ID][Len×2][Payload][CK_A][CK_B]
  // 逐字节推进 parser_state_。组完一帧且校验通过返回 true（数据在 rx_msg_，
  // 由 Pump 调 HandleValidMessage 分发）；否则返回 false。
  // parser_state_ 跨调用保留，因此支持把一帧拆到多次调用里解析（配合 Pump 的字节预算）。
  bool Ubx::ProcessUbxByte(uint8_t c)
  {
    c_ = c;
    /* 阶段 1：同步字 0xB5 0x62。任一字节不匹配即把状态机重置回 0 重新找帧头。 */
    if (parser_state_ < sizeof(UBX_HEADER_))
    {
      if (c_ == UBX_HEADER_[parser_state_])
      {
        parser_state_++;
      }
      else
      {
        parser_state_ = 0;
      }
      /* 阶段 2：Class 字节 */
    }
    else if (parser_state_ == UBX_CLS_POS_)
    {
      rx_msg_.cls = c_;
      parser_state_++;
      /* 阶段 3：ID 字节 */
    }
    else if (parser_state_ == UBX_ID_POS_)
    {
      rx_msg_.id = c_;
      parser_state_++;
      /* 阶段 4：长度低字节（小端，先低后高），暂存到 len_ */
    }
    else if (parser_state_ == UBX_LEN_POS_LSB_)
    {
      len_ = c_;
      parser_state_++;
    }
    /* 阶段 5：长度高字节，与 len_ 合成 16 位 payload 长度 */
    else if (parser_state_ == UBX_LEN_POS_MSB_)
    {
      rx_msg_.len = static_cast<uint16_t>(c_) << 8 | len_;
      parser_state_++;
      /* 防溢出：payload 超过接收缓冲容量则丢弃整帧并复位状态机。 */
      if (rx_msg_.len > UBX_MAX_PAYLOAD_)
      {
        oversize_msg_count_++;
        parser_state_ = 0;
      }
      /* 阶段 6：payload 主体，按偏移写入 rx_msg_.payload。 */
    }
    else if (parser_state_ < (static_cast<size_t>(rx_msg_.len) + UBX_HEADER_LEN_))
    {
      rx_msg_.payload[parser_state_ - UBX_HEADER_LEN_] = c_;
      parser_state_++;
      /* 阶段 7：校验和第一字节 CK_A，先暂存。 */
    }
    else if (parser_state_ == (static_cast<size_t>(rx_msg_.len) + UBX_HEADER_LEN_))
    {
      chk_rx_ = c_;
      parser_state_++;
    }
    /* 阶段 8：校验和第二字节 CK_B。此时本地重算校验并与收到的比对。 */
    else
    {
      // 校验范围：从 rx_msg_.cls 起，长度 = payload 长度 + (Class+ID+Len 共 4 字节)。
      // UBX_CHK_OFFSET_ = 头长6 - 同步字2 = 4，正是 cls..len 这段需纳入校验的字节数。
      chk_cmp_rx_ =
          chksum_rx_.Compute(reinterpret_cast<const uint8_t *>(&rx_msg_),
                             rx_msg_.len + UBX_CHK_OFFSET_);
      parser_state_ = 0;  // 无论校验成败，本帧结束，状态机复位准备下一帧。
      /* 比对本地校验(高字节 sum1<<8 | 低字节 sum0) 与收到的 (CK_B<<8 | CK_A)。 */
      if (chk_cmp_rx_ == (static_cast<uint16_t>(c_) << 8 | chk_rx_))
      {
        valid_msg_count_++;
        return true;  // 一帧有效报文就绪，交给 HandleValidMessage。
      }
      checksum_fail_count_++;  // 校验失败：丢弃本帧（应保持为 0，非 0 多为接线/波特率问题）。
    }
    return false;  // 本字节未组完整帧。
  }

} // namespace bfs
