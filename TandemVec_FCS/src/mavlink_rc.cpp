/**
 * @file mavlink_rc.cpp
 * @module 飞控/通信（MAVLink RC 输入解析）
 * @brief `MavlinkRc` 的实现：MAVLink 帧状态机 + RC_CHANNELS_OVERRIDE/RADIO_STATUS 语义。
 *
 * @note 独立使用 `MAVLINK_COMM_1`，与 Serial6 上的参数桥（`mavlink_bridge.cpp`，
 *       使用库默认通道）隔离状态机，互不干扰。
 *
 * @note 解析器与协议库的分工：`lib/MAVLink` 只负责帧同步与 CRC 校验（官方生成代码，
 *       CRC_EXTRA 随消息定义走），本文件只做"哪条消息、怎么过滤、写哪个数组"。
 */

#include "mavlink_rc.h"

MavlinkRc mavlinkRc;

namespace {

/// 本机在 MAVLink 网络中的身份，与 mavlink_bridge.cpp 的下行编码保持一致
constexpr uint8_t kOwnSysId = 1;
constexpr uint8_t kOwnCompId = MAV_COMP_ID_AUTOPILOT1;

/// 解析通道号：与 Serial6 参数桥隔离
constexpr uint8_t kChannel = MAVLINK_COMM_1;

/**
 * @brief 每轮最多消费的字节数
 *
 * 420000 8N1 下 100 Hz RC 帧约 5.3 kB/s，主循环 1 kHz 量级，64 B/轮足够；
 * 设上限是为了避免串口洪水时把整轮主循环耗在解析上（与 GNSS 轮询同一理由）。
 */
constexpr int kMaxBytesPerLoop = 64;

}  // namespace

void MavlinkRc::begin(Stream& serial, uint8_t rx_sysid) {
  _serial = &serial;
  _rx_sysid = rx_sysid;
  mavlink_reset_channel_status(kChannel);
  reset();
}

void MavlinkRc::setHandlers(ChannelsFn on_channels, LinkFn on_link) {
  _on_channels = on_channels;
  _on_link = on_link;
}

void MavlinkRc::reset() {
  for (uint8_t i = 0; i < kChannels; ++i) {
    _ch[i] = kCenterUs;
  }
  _link_up = false;
  _seen_any = false;
  _rssi_raw = 255;
}

void MavlinkRc::loop(uint32_t now_ms) {
  _now_ms = now_ms;
  if (_serial != nullptr) {
    int budget = kMaxBytesPerLoop;
    while (budget-- > 0 && _serial->available() > 0) {
      const uint8_t c = static_cast<uint8_t>(_serial->read());
      ++_bytes_in;
      // 环形快照：始终保留最近 16 字节，用于判断串口上到底是哪种协议
      if (_sniff_len < sizeof(_sniff)) {
        _sniff[_sniff_len++] = c;
      } else {
        for (uint8_t i = 1; i < sizeof(_sniff); ++i) {
          _sniff[i - 1] = _sniff[i];
        }
        _sniff[sizeof(_sniff) - 1] = c;
      }
      mavlink_message_t msg;
      mavlink_status_t status;
      if (mavlink_parse_char(kChannel, c, &msg, &status)) {
        handleMessage(msg);
      }
    }
  }

  // 超时判定：MAVLink 没有心跳概念，链路有效性完全由"多久没收到通道帧"决定。
  // 判据与 CRSF 路径的 CRSF_FAILSAFE_STAGE1_MS 对齐，切换协议不改变安全时序。
  if (_link_up && (now_ms - _last_ch_ms) > kLinkTimeoutMs) {
    _link_up = false;
    if (_on_link != nullptr) {
      _on_link(false);
    }
  }
}

void MavlinkRc::handleMessage(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
      mavlink_rc_channels_override_t m;
      mavlink_msg_rc_channels_override_decode(&msg, &m);
      (void)mavlink_msg_rc_channels_override_decode;

      // ① 寻址过滤：0 表示广播；否则必须是本机
      if (m.target_system != 0 && m.target_system != kOwnSysId) {
        ++_dropped;
        return;
      }
      if (m.target_component != 0 && m.target_component != kOwnCompId) {
        ++_dropped;
        return;
      }
      // ② 来源过滤：只认接收机（默认 255）。GCS 的 override 会被这条挡掉，
      //    否则地面站可以在飞行中抢夺遥控，属于安全红线。
      if (_rx_sysid != 0 && msg.sysid != _rx_sysid) {
        ++_dropped;
        return;
      }

      // ③ 通道语义：0 = 本帧不覆盖该通道，必须保持上一帧值。
      //    ELRS 实际会填满 16 通道，但 GCS/MANUAL_CONTROL 路径可能留 0，
      //    直接写入会让油门归零。
      const uint16_t raw[kChannels] = {
          m.chan1_raw,  m.chan2_raw,  m.chan3_raw,  m.chan4_raw,
          m.chan5_raw,  m.chan6_raw,  m.chan7_raw,  m.chan8_raw,
          m.chan9_raw,  m.chan10_raw, m.chan11_raw, m.chan12_raw,
          m.chan13_raw, m.chan14_raw, m.chan15_raw, m.chan16_raw,
      };

      bool any = false;
      for (uint8_t i = 0; i < kChannels; ++i) {
        if (raw[i] != 0) {
          _ch[i] = raw[i];
          any = true;
        }
      }
      if (!any) {
        // 全 0 帧等价于"什么都没说"，按丢弃计数，不刷新链路时间戳
        ++_dropped;
        return;
      }

      ++_ch_frames;
      _last_ch_ms = _now_ms;
      _seen_any = true;
      if (_on_channels != nullptr) {
        _on_channels(_ch);
      }
      if (!_link_up) {
        _link_up = true;
        if (_on_link != nullptr) {
          _on_link(true);
        }
      }
      return;
    }

    case MAVLINK_MSG_ID_RADIO_STATUS: {
      mavlink_radio_status_t m;
      mavlink_msg_radio_status_decode(&msg, &m);
      if (_rx_sysid != 0 && msg.sysid != _rx_sysid) {
        ++_dropped;
        return;
      }
      // ELRS 用 rssi = LQ×2.55 编码（见 SerialMavlink.cpp），这里只取上行 LQ
      _rssi_raw = m.rssi;
      return;
    }

    default:
      // 其余消息（HEARTBEAT/PARAM 等）归 Serial6 参数桥，本模块不消费
      return;
  }
}

uint8_t MavlinkRc::lqPercent() const {
  if (_rssi_raw == 255) {
    return 100;  // 未收到统计帧时按满信号处理，避免灯效误报
  }
  return static_cast<uint8_t>((static_cast<uint16_t>(_rssi_raw) * 100U + 127U) / 255U);
}