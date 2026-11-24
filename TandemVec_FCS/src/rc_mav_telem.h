/**
 * @file rc_mav_telem.h
 * @module 飞控/通信（RC 侧 MAVLink 遥测下行）
 * @brief RC 输入走 MAVLink 时，把遥测改写为 MAVLink 帧发给 ELRS 接收机。
 *
 * @details 为什么必须有这个模块：
 *          ELRS 的 RX↔飞控是**一根 UART 双向**，协议二选一。CRSF 模式下飞控发
 *          CRSF 遥测帧（0xC8/0xEE），RX 转发给 TX 再进遥控器；切到 MAVLink 后
 *          RX 只认 MAVLink 帧（`SerialMavlink::processBytes`），CRSF 帧被直接丢弃，
 *          遥控器上就没有任何回传数据。
 *
 * @note 消息集**严格限定**在 ELRS 的翻译白名单内（`lib/MAVLink/MAVLink.cpp`
 *       `convert_mavlink_to_crsf_telem()`），发白名单外的消息遥控器不显示：
 *       HEARTBEAT / GPS_RAW_INT / ATTITUDE / GLOBAL_POSITION_INT /
 *       BATTERY_STATUS / VFR_HUD / ALTITUDE / SCALED_PRESSURE /
 *       HOME_POSITION / STATUSTEXT / SYSTEM_TIME / HIGH_LATENCY2。
 *
 * @note TX 侧有硬过滤：`if (msg.compid != MAV_COMP_ID_AUTOPILOT1) continue;`
 *       → 本模块统一用 compid=AUTOPILOT1、sysid=1。
 *
 * @note 带宽：ELRS MAVLink 模式强制 Telem Ratio 1:2。2.4G F1000（OTA4，5B/包）
 *       下回传约 500 包/s × 5B ≈ 2.5 kB/s；本模块按 ≈1.3 kB/s 配置（约 52% 占用），
 *       留出余量给 RC 与协议开销。频率改动前先按这张预算表核算。
 *
 * @see src/mavlink_rc.cpp（同协议的反方向：RC 输入解析）、
 *      docs/memory/2026-11-25-ELRS-MAVLink模式RC接入.md
 */

#pragma once

#include <Arduino.h>

/**
 * @brief 绑定输出串口（与 RC 输入同一个 Serial1）
 * @param out ELRS 接收机所在串口
 */
void rcMavTelemBegin(Stream& out);

/**
 * @brief 周期发送：按内部节拍表挑这一拍要发的消息
 * @param now_ms `millis()` 当前值
 */
void rcMavTelemLoop(uint32_t now_ms);

/// @brief 清空节拍与统计（协议切换时调用）
void rcMavTelemReset();

/// @brief 已发出的 MAVLink 帧数（DBG `rcproto` 展示，用于确认真的在发）
uint32_t rcMavTelemFrames();

/// @brief 是否已绑定输出口
bool rcMavTelemReady();