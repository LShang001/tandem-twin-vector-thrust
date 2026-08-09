// ============================================================
//  mavlink_bridge.h — MAVLink 双向功能（QGC 桥接）对外接口
//
//  2026-08-10 双向扩展（docs/mavlink-ref/ 官方协议参考）：
//    - 上行：PARAM_REQUEST_LIST/READ/SET（QGC 直接读写 121 参数）、
//      COMMAND_LONG（重启支持，ARM/SET_MODE 安全拒绝）
//    - 下行：PARAM_VALUE 广播状态机、COMMAND_ACK、STATUSTEXT 队列
//  协议语义/枚举见 docs/mavlink-ref/MAVLink-*。
// ============================================================
#pragma once
#include "MAVLink.h"

// 上行消息分发（handleAnoCom 的 mavlink 分支调用，帧已通过 CRC 校验）
void mavlinkHandleRx(const mavlink_message_t &msg);

// 下行周期发送（mavlinkSendTelemetry 内调用）：
//   - PARAM_VALUE 广播状态机（REQUEST_LIST 触发后逐条发，200Hz 每拍 1 条）
//   - STATUSTEXT 队列轮发（1Hz）
void mavlinkBridgeTick();

// STATUSTEXT 入队（severity 见 MAV_SEVERITY 枚举；文本 ≤50B 截断）
void mavlinkSendStatustext(uint8_t severity, const char *text);
