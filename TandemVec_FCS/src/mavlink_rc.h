/**
 * @file mavlink_rc.h
 * @module 飞控/通信（MAVLink RC 输入解析）
 * @brief 解析 ELRS 接收机在 MAVLink 模式下输出的 RC 通道与链路状态。
 *
 * @details 输入：Serial1 字节流（ELRS RX，420000 8N1，MAVLink 模式）
 *          输出：① 16 通道 PWM(988-2012 µs)；② 链路有效/失效事件；③ LQ 百分比。
 *
 * @note 与 CRSF 的差异（本模块存在的理由，详见 docs/memory 的 RC 双协议记录）：
 *       1. `RC_CHANNELS_OVERRIDE` 的 `chanN_raw == 0` 表示"本帧不覆盖该通道"，
 *          必须保持上一帧值；若直接写入 0 会让油门瞬间归零。
 *       2. 报文带 target_system/target_component 寻址，且来源 sysid 可能是
 *          GCS 而不是接收机，必须双向过滤，否则地面站可抢遥控。
 *       3. ELRS 侧把 RC 帧限流到 100 Hz（`MAVLINK_RC_PACKET_INTERVAL`），
 *          低于 CRSF 的逐包输出，控制环会重复读到同一帧 —— 这是协议上限，
 *          固件侧不补偿，只保证语义正确。
 *       4. 超时由飞控自己判定：MAVLink 只给"覆盖指令"，不给心跳。本模块沿用
 *          与 CRSF 一致的 300 ms 判据，不依赖对端栈的 override 超时。
 *
 * @note 安全约束：本模块只写通道与链路状态，不做模式/解锁判定；
 *       所有开关量语义仍由 flight_control.cpp 按 `isLinkUp` 门控解释。
 *
 * @see src/communication.cpp（分发与回调）、lib/CrsfSerial（CRSF 路径对照）、
 *      docs/reference/elrs-official/src/src/rx-serial/SerialMavlink.cpp（对端实现）
 */

#pragma once

#include <Arduino.h>

#include "MAVLink.h"

/**
 * @brief RC 输入协议选择（与 `rcInputProto()` 的 0/1 一一对应）
 */
enum class RcProto : uint8_t {
  Crsf = 0,    ///< CRSF（默认，逐包输出，11-bit 打包）
  Mavlink = 1, ///< MAVLink RC_CHANNELS_OVERRIDE（ELRS MAVLink 模式）
};

/**
 * @brief MAVLink RC 输入解析器
 *
 * 生命周期：`begin()` 一次 → 主循环每轮 `loop(now_ms)` → `reset()` 在协议切换时调用。
 * 所有硬件访问都经过 `Stream`，宿主测试用假 Stream 喂字节。
 */
class MavlinkRc {
public:
  /// 通道更新回调：参数为 16 通道 µs 值，调用时值已通过过滤
  using ChannelsFn = void (*)(const uint16_t us[16]);
  /// 链路状态回调：true=恢复，false=失效（超时或 reset）
  using LinkFn = void (*)(bool up);

  /// 通道数（与 `RC_INPUT_MAX_CHANNELS` 一致）
  static constexpr uint8_t kChannels = 16;
  /// 链路超时：与 `CRSF_FAILSAFE_STAGE1_MS` 对齐（300 ms）
  static constexpr uint16_t kLinkTimeoutMs = 300;
  /// 通道初始/无效值：中位 1500 µs（避免上电 0 被误判为最低油门）
  static constexpr uint16_t kCenterUs = 1500;

  /**
   * @brief 绑定串口并复位解析状态
   * @param serial  ELRS 接收机所在串口（Serial1）
   * @param rx_sysid 只接受该 sysid 的 RC 帧（ELRS 默认 255；0=不过滤）
   */
  void begin(Stream& serial, uint8_t rx_sysid = 255);

  /// 注册回调（在 begin 之前调用均可）
  void setHandlers(ChannelsFn on_channels, LinkFn on_link);

  /// 设置接收机 SysID（0 表示关闭来源过滤，仅调试用）
  void setRxSysId(uint8_t rx_sysid) { _rx_sysid = rx_sysid; }

  /**
   * @brief 主循环轮询：读串口字节、推进解析状态机、判超时
   * @param now_ms `millis()` 当前值（宿主测试可注入虚拟时间）
   */
  void loop(uint32_t now_ms);

  /// 清空解析状态、通道回中位、链路置失效（协议切换时调用）
  void reset();

  bool linkUp() const { return _link_up; }
  const uint16_t* channels() const { return _ch; }
  uint16_t channel(uint8_t idx) const { return idx < kChannels ? _ch[idx] : kCenterUs; }

  /// RADIO_STATUS 的 rssi(0..255) 换算成 0..100，未收到时为 100
  uint8_t lqPercent() const;

  uint32_t channelFrames() const { return _ch_frames; }  ///< 有效通道帧计数
  uint32_t droppedFrames() const { return _dropped; }    ///< 被过滤的帧计数
  uint32_t lastChannelsMs() const { return _last_ch_ms; }
  /// 串口累计收到字节数：区分"串口无数据"与"有数据但不是合法 MAVLink"
  uint32_t bytesIn() const { return _bytes_in; }
  /// 最近 16 个原始字节（诊断用，看首字节是 0xFD(MAVLink v2) 还是 0xC8(CRSF)）
  const uint8_t* sniff() const { return _sniff; }
  uint8_t sniffLen() const { return _sniff_len; }

private:
  /// 处理一条已通过 CRC 的 MAVLink 帧（仅 RC_CHANNELS_OVERRIDE / RADIO_STATUS 被采纳）
  void handleMessage(const mavlink_message_t& msg);

  Stream* _serial = nullptr;
  ChannelsFn _on_channels = nullptr;
  LinkFn _on_link = nullptr;

  uint16_t _ch[kChannels] = {};
  bool _link_up = false;
  bool _seen_any = false;   ///< 是否收到过任何有效通道帧（区分"从未连上"与"断链"）
  uint8_t _rx_sysid = 255;
  uint8_t _rssi_raw = 255;  ///< RADIO_STATUS.rssi，255 表示未收到

  uint32_t _ch_frames = 0;
  uint32_t _dropped = 0;
  uint32_t _last_ch_ms = 0;
  uint32_t _now_ms = 0;  ///< 本轮注入时间（宿主测试用虚拟时间，不能用 millis()）
  uint32_t _bytes_in = 0;
  uint8_t _sniff[16] = {};   ///< 最近 16 字节环形快照
  uint8_t _sniff_len = 0;
};

/// 全局实例（协议切换由 communication.cpp 的 rcInputSetProto() 驱动）
extern MavlinkRc mavlinkRc;