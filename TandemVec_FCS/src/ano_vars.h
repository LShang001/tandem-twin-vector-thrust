// ============================================================
//  ano_vars.h — 通用变量上报（AnoVars）对外接口
//  注册表/序列化/watch 集合定义见 include/AnoVars.h（平台无关）
//  本文件：固件侧接线函数声明（0xF2 发送 / 0xF3 应答 / DBG 命令）
// ============================================================
#pragma once
#include "Arduino.h"
#include "AnoVars.h"

class AnoComProtocol;

// 变量 ID → 表项（越界返回 nullptr；MAVLink 调试通道共用）
const AnoVarEntry *anoVarAt(uint16_t id);

// 0xF2 值帧发送（handleAnoCom 200Hz 调用，内部节流 + watch 轮转）
void anoVarsSendTick(AnoComProtocol &ano);

// 上行分派（communication.cpp onAnoRxFrame 调用，0xF3 清单请求）
void anoVarsHandleRx(AnoComProtocol &ano, uint8_t funcCode,
                     const uint8_t *data, uint16_t len);

// DBG vars 命令（handleDebugConsole 分派）
void anoVarsDebugCommand(HardwareSerial &serial, char *args);
