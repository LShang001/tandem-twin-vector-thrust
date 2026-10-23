/**
 * @file    Stream.h
 * @module  固件/宿主 SIL 桩（fw-tests：Stream 基类转发）
 * @brief   把固件源码的 #include <Stream.h> 转发到本目录的 Arduino.h 替身
 * @details 被测：无（替身层）。固件只经此头拿到 Stream 基类声明，真实串口行为
 *          由 SIL 主程序注入，不经过本文件。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 Stream 的差异：无缓冲、无超时、无 available/read 语义。
 * @see     TandemVec_FCS/test_host/fw_sil/stub/Arduino.h
 */
#pragma once
#include "Arduino.h"
