// ============================================================
//  ano_params.h — AnoCom 参数在线读写（0xE0 参数命令 / 0xE1 参数写入）
//
//  2026-08-08 新增：完善原占位实现（校验值恒 0），参数表 = 
//  kFlightCtrlParams（FlightCtrlParams.h）全部字段：
//    12 个 PID 环 × 9 字段（kp/ki/kd/out_min/out_max/int_limit/
//    threshold/filter_alpha/enabled）+ 9 个控制滤波器 alpha = 117 参数。
//  写入后调用 applyFlightCtrlParams() 无扰同步到全部 PID/滤波器实例。
//
//  ★ 参数线上名 ≤20B（协议 PAR_NAME 定长 20B，超限截断致上位机按名匹配失败）：
//    filter_alpha 的线上名 = "{loop}.falpha"；滤波数组 = spd_alpha/ang_alpha/out_alpha。
//    命名须与 GCS/server/params.py 的 FILTER_WIRE / FIELD_META 一致。
//
//  协议（匿名通信协议 v1）：
//    0xE0 DATA=[CMD, VAL...]
//      CMD 0x01 读参数个数 → 回 0xE0 [0x01, count u32 LE]
//      CMD 0x02 读参数值 (VAL=ID u16) → 回 0xE1 [ID u16, PAR_VAL]
//      CMD 0x03 读参数信息 (VAL=ID u16) → 回 0xE2 [ID u16, TYPE u8, NAME 20B]
//      CMD 0x10 VAL=0xAA 恢复默认值 / 0xAB 保存（无持久化，仅确认）
//    0xE1 DATA=[ID u16, PAR_VAL（类型定长）] → 写入 + apply
//  校验帧（0x00 回传）由 communication.cpp onAnoRxFrame 统一回传。
// ============================================================
#pragma once

#include "AnoComProtocol.h"

// 参数总数（12 环 × 9 字段 + 12 滤波 alpha；★2026-08-09 +3 二级滤波 spd2_alpha）
#define ANO_PARAMS_COUNT 121

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
// 参数表 = 同一 kAnoParams 注册表（121 项，顺序即 ID 0..120）。
// u8（enabled/inertia_comp_mask）统一转 float 0.0/1.0 上报；写后调
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
