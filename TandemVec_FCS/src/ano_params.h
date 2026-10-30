/**
 * @file    ano_params.h
 * @module  固件/通信（AnoCom 参数在线读写 + MAVLink 参数桥）
 * @brief   声明参数注册表访问接口（0xE0/0xE1 读写、0xE2 信息、MAVLink float 桥）
 *
 * 2026-08-08 新增：完善原占位实现（校验值恒 0），参数表 =
 * kFlightCtrlParams（FlightCtrlParams.h）全部字段：
 *   12 个 PID 环 × 9 字段（kp/ki/kd/out_min/out_max/int_limit/
 *   threshold/filter_alpha/enabled）+ 9 个控制滤波器 alpha + 3 个二级滤波 alpha
 *   + inertia_comp_mask + 7 个 FPV 摇杆曲线参数；总数以 ANO_PARAMS_COUNT 为准。
 * 写入后调用 applyFlightCtrlParams() 无扰同步到全部 PID/滤波器实例。
 *
 * ★ 参数线上名 ≤20B（协议 PAR_NAME 定长 20B，超限截断致上位机按名匹配失败）：
 *   filter_alpha 的线上名 = "{loop}.falpha"；滤波数组 = spd_alpha/ang_alpha/out_alpha/
 *   spd2_alpha + rc_expo/rc_super。
 *   命名/顺序须与 GCS/server/params.py 的 FILTER_WIRE / FIELD_META / expected_names() 一致。
 *
 * @details 协议（匿名通信协议 v1，PROTOCOL-001 §四）：
 *    0xE0 DATA=[CMD, VAL...]
 *      CMD 0x00 读设备信息 → 回 0xE3（由 communication.cpp onAnoRxFrame 处理）
 *      CMD 0x01 读参数个数 → 回 0xE0 [0x01, count u32 LE]
 *      CMD 0x02 读参数值 (VAL=ID u16) → 回 0xE1 [ID u16, PAR_VAL]
 *      CMD 0x03 读参数信息 (VAL=ID u16) → 回 0xE2 [ID u16, TYPE u8, NAME 20B]
 *      CMD 0x10 VAL=0xAA 恢复默认值 / 0xAB 保存（无持久化，仅确认）
 *    0xE1 DATA=[ID u16, PAR_VAL（类型定长）] → 写入 + apply
 *  校验帧（0x00 回传）由 communication.cpp onAnoRxFrame 统一回传。
 * @theory  N/A（协议实现）
 * @ref     PROTOCOL-001 §四（0xE0~0xE3 参数链路，已实机逐项对齐）、
 *          GCS-001 §2（参数在线读写方案）；AGENTS.md 关键路径（注册表 ID 顺序同步约束）
 * @frame   N/A
 * @units   协议层无缩放（PAR_VAL 为定长原值二进制）；物理单位由 kFlightCtrlParams 注释给出
 * @note    参数 ID = 注册表下标（0..ANO_PARAMS_COUNT-1），**顺序变更 = 协议变更**：
 *          改顺序必须同步 GCS/server/params.py::expected_names()，否则在线调参会写错参数。
 * @see     src/ano_params.cpp、include/FlightCtrlParams.h、test_host/test_anocom_packing.cpp
 */
#pragma once

#include "AnoComProtocol.h"

// 参数总数（12 环 × 9 字段 + 12 滤波 alpha + 前馈掩码 + 二级滤波 + FPV 摇杆曲线 7 参数
// + 航向源 2 + GNSS 杆臂 3 + ★2026-10-30 共轴双桨 25 项 = 158）
// ★ 唯一真值：改动注册表必须同时改这里与 GCS/server/params.py::expected_names()
#define ANO_PARAMS_COUNT 158

/**
 * @brief 处理 AnoCom 参数帧（0xE0 参数命令 / 0xE1 参数写入）
 * @param ano   AnoCom 通信实例（发送回传帧用）
 * @param funcCode 功能码（ANO_FUNC_PARAM_CMD / ANO_FUNC_PARAM_WRITE_READ）
 * @param data  DATA 区指针
 * @param len   DATA 长度
 * @return true = 帧已消费（非参数帧返回 false，供调用方继续处理）
 */
bool anoParamsHandleRx(AnoComProtocol &ano, uint8_t funcCode,
                       const uint8_t *data, uint16_t len);

// ---- MAVLink 参数桥统一 float 接口（2026-08-10，供 mavlink_bridge.cpp 复用）----
// 参数表 = 同一 kAnoParams 注册表（数量 = ANO_PARAMS_COUNT，顺序即 ID）。
// u8（enabled/inertia_comp_mask）统一转 float 上报；写后调
// applyFlightCtrlParams() 无扰同步。★ MAVLink param_id char[16] 限制：
// 线上名 >16B 时按截断 15B 匹配（唯一性已核实，见 docs/mavlink-ref/参数服务）。

// 参数总数（= ANO_PARAMS_COUNT）
uint16_t anoParamCount();

// 参数线上名（≤20B，MAVLink 侧截断 15B 使用）
const char *anoParamNameAt(uint16_t id);

// 按线上名查 ID（返回 -1 未找到；MAVLink 用截断名匹配）
int16_t anoParamIdByName(const char *name);

// 读参数值（统一 float；无效 ID 返回 0 且 ok=false）
bool anoParamReadFloat(uint16_t id, float *out);

// 写参数值（统一 float；写后 applyFlightCtrlParams；无效 ID/类型不匹配返回 false）
bool anoParamWriteFloat(uint16_t id, float value);
