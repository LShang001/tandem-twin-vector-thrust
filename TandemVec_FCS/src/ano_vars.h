/**
 * @file    ano_vars.h
 * @module  固件/通信（AnoVars 通用变量上报，固件侧接线）
 * @brief   声明 0xF2 值帧发送、0xF3 清单应答与 DBG `vars` 命令入口
 * @details 注册表/序列化/watch 集合等纯逻辑见 include/AnoVars.h（平台无关、host 可测）；
 *          本文件只做固件侧接线：把 watch 集合按频率轮转成 0xF2 帧、应答 0xF3 清单、
 *          解析 DBG 命令。输入：`g_ano_var_watch`（DBG 配置）；输出：AnoCom 帧 / 串口文本。
 *          不变量：0xF2 = `[id u16 LE] + [float LE]`（6B DATA）；0xF3 应答 20B；
 *          watch 上限 16、速率 1-200Hz。
 * @theory  N/A（诊断遥测接线，无理论模型）
 * @ref     PROTOCOL-001 §六（0xF1~0xFA 灵活格式帧的用户自定义语义）、
 *          AGENTS.md 踩坑记录（AnoVars 通用变量上报 + 三坑）
 * @frame   变量值沿用其原始语义（姿态 deg、角加速度 rad/s² 或 deg/s² 见变量名后缀）
 * @units   帧内统一 float32，无缩放；单位由具体变量决定
 * @note    watch 为空时 0xF2 零开销返回；与双环遥测相互独立，不占组缓冲。
 * @see     include/AnoVars.h、src/ano_vars.cpp、src/communication.cpp（handleAnoCom）
 */
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
void anoVarsDebugCommand(Uart &serial, char *args);
