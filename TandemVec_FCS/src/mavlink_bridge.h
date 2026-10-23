/**
 * @file    mavlink_bridge.h
 * @module  固件/通信（MAVLink 双向桥：QGC / Mission Planner）
 * @brief   声明 MAVLink 上行分发、下行参数广播/STATUSTEXT 队列接口
 * @details MAVLink 与 AnoCom 共用 Serial6，由 `s_mav_active`（DBG `proto mavlink|anocom`）
 *          在运行时二选一，切换不重烧固件。
 *          上行：PARAM_REQUEST_LIST/READ/SET（QGC 直接读写全部参数）、COMMAND_LONG
 *          （246 重启；400 ARM / 176 SET_MODE 安全拒绝）、其余回 UNSUPPORTED；
 *          下行：PARAM_VALUE 广播状态机、COMMAND_ACK、STATUSTEXT 队列。
 * @theory  N/A（协议桥接实现）
 * @ref     docs/mavlink-ref/MAVLink-参数服务.md（param_id char[16] 截断/枚举语义）、
 *          docs/mavlink-ref/MAVLink-命令与状态.md；AGENTS.md 踩坑记录（MAVLink 切换）
 * @frame   N/A（MAVLink 消息字段自带坐标系，本项目透传 AHRS/INS 值）
 * @units   消息字段单位由 MAVLink 定义（rad、m、m/s、mV、cA）；参数一律 float
 * @note    参数表 = `ano_params.cpp` 的 kAnoParams 注册表（数量以 ANO_PARAMS_COUNT 为准），
 *          写入后同样调用 `applyFlightCtrlParams()`；ARM/SET_MODE 由 RC CH5 硬判定，
 *          地面站远程解锁属于行为红线，必须保持拒绝。
 * @see     src/mavlink_bridge.cpp、src/ano_params.h、src/communication.cpp（handleAnoCom）
 */
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
