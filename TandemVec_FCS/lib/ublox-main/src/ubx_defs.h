/*
 * ubx-gnss —— u-blox UBX 协议本地重构解析库
 *
 * 本文件源自 Bolder Flight Systems 的 ublox 库，但已在本仓库内做过较多本地重构，
 * 与官方原版不再一致（裁剪了不用的消息族、改写了 epoch 队列与泵入式解析等）。
 * 保留以下原始 MIT 版权声明仅为遵守许可证要求，不代表本文件与官方版本等价。
 *
 * 原始版权声明（MIT License）：
 * Brian R Taylor / brian.taylor@bolderflight.com
 * Copyright (c) 2022 Bolder Flight Systems Inc
 *
 * 特此免费授予任何获得本软件及相关文档文件（“软件”）副本的人士不受限制地处置
 * 本软件的权利，包括但不限于使用、复制、修改、合并、发布、分发、再许可和/或
 * 销售本软件副本的权利，并允许获得本软件的人士在满足以下条件的前提下这样做：
 *
 * 上述版权声明和本许可声明应包含在本软件的所有副本或主要部分中。
 *
 * 本软件按“原样”提供，不附带任何形式的明示或暗示担保，包括但不限于对适销性、
 * 特定用途适用性和非侵权性的担保。在任何情况下，作者或版权持有人均不对任何索赔、
 * 损害或其他责任负责，无论是合同诉讼、侵权诉讼还是其他诉讼。
 */

#ifndef SRC_UBX_DEFS_H_
#define SRC_UBX_DEFS_H_

// 非 Arduino（如主机 CMake / 单元测试）环境下没有 Arduino.h，手动引入定宽整型与 size_t。
// Arduino 环境由上层（ubx.h）先包含 Arduino.h，这里不重复包含以避免污染编译单元。
#if !defined(ARDUINO)
#include <cstdint>
#include <cstddef>
#endif

/*
 * UBX 协议把多字节字段按“小端、紧凑、无填充”方式排布在报文里。本库的接收结构体
 * （见 ubx_nav.h）直接用 memcpy 把串口收到的 payload 原样拷进 struct，因此 struct
 * 的内存布局必须和报文字节序完全一致——任何编译器自动插入的对齐填充都会让字段错位、
 * 进而让 sizeof(payload) 与协议规定的 len 对不上，最终破坏长度校验和数值解析。
 *
 * 当前用到的 NAV 结构体只含 uint8/uint16/uint32/int* 等自然对齐字段，多数编译器
 * 默认布局已经无填充；但显式标注 packed 是一道保险：将来若新增含 double/嵌套体的
 * 结构，也不会被静默填充破坏。GCC/Clang 用 __attribute__((packed))，其它编译器
 * 留空（MSVC 等需要时可改用 #pragma pack，本库目标平台是 GCC/Clang 故无需处理）。
 */
#if defined(__GNUC__) || defined(__clang__)
#define UBX_PACKED __attribute__((packed))
#else
#define UBX_PACKED
#endif

namespace bfs {
/*
 * u-blox 在《Interface Description》里用 U1/I2/X4/R8 这类简写描述每个字段的类型。
 * 这里把它们映射成 C++ 定宽类型，照着接口手册抄结构体时可以一一对应、减少出错：
 *   U = 无符号整数, I = 有符号整数, X = 位域/标志(按无符号整数收), R = 浮点, CH = 字符。
 *   末尾数字是字节数：1=8bit, 2=16bit, 4=32bit, 8=64bit。
 */
using U1 = uint8_t;   // 无符号 8 位
using I1 = int8_t;    // 有符号 8 位
using X1 = uint8_t;   // 8 位标志/位域
using U2 = uint16_t;  // 无符号 16 位
using I2 = int16_t;   // 有符号 16 位
using X2 = uint16_t;  // 16 位标志/位域
using U4 = uint32_t;  // 无符号 32 位
using I4 = int32_t;   // 有符号 32 位
using X4 = uint32_t;  // 32 位标志/位域
using R4 = float;     // 32 位浮点
using R8 = double;    // 64 位浮点
using CH = char;      // 字符

/*
 * UBX 报文头之后的第 1 个字节是 Class（消息大类）。本库只解析 NAV 类（见 ubx.cpp
 * 的 HandleValidMessage），其余 Class 常量仅作完整性备查，方便诊断时识别报文归属。
 */
static constexpr uint8_t UBX_ACK_CLS_ = 0x05;  // ACK/NAK：配置命令的应答
static constexpr uint8_t UBX_CFG_CLS_ = 0x06;  // CFG：接收机配置
static constexpr uint8_t UBX_INF_CLS_ = 0x04;  // INF：信息/调试文本
static constexpr uint8_t UBX_LOG_CLS_ = 0x21;  // LOG：数据记录
static constexpr uint8_t UBX_MGA_CLS_ = 0x13;  // MGA：多 GNSS 辅助（星历注入）
static constexpr uint8_t UBX_MON_CLS_ = 0x0a;  // MON：接收机监控/状态
static constexpr uint8_t UBX_NAV_CLS_ = 0x01;  // NAV：导航解算结果（本库主要解析对象）
static constexpr uint8_t UBX_RXM_CLS_ = 0x02;  // RXM：原始测量/接收管理
static constexpr uint8_t UBX_SEC_CLS_ = 0x27;  // SEC：安全相关
static constexpr uint8_t UBX_TIM_CLS_ = 0x0d;  // TIM：授时
static constexpr uint8_t UBX_UPD_CLS_ = 0x09;  // UPD：固件/Flash 更新

/*
 * 接收机的物理端口编号。配置 CFG-* 报文（指定某端口启用哪些协议）时会用到；
 * 本解析库本身不发配置，仅在需要构造配置报文的上层代码里作参照。
 */
static constexpr uint8_t UBX_COM_PORT_I2C_ = 0;    // I2C(DDC) 端口
static constexpr uint8_t UBX_COM_PORT_UART1_ = 1;  // UART1（本项目实链路所用）
static constexpr uint8_t UBX_COM_PORT_UART2_ = 2;  // UART2
static constexpr uint8_t UBX_COM_PORT_USB_ = 3;    // USB
static constexpr uint8_t UBX_COM_PORT_SPI_ = 4;    // SPI

/*
 * 端口协议使能位掩码：CFG-PRT 等报文用它指定某端口收发哪些协议，可按位或组合。
 * 例如只保留 UBX、关掉 NMEA 可减小串口带宽占用，是高频导航链路常用配置。
 */
static constexpr uint8_t UBX_COM_PROT_UBX_ = 0x01;    // UBX 二进制协议
static constexpr uint8_t UBX_COM_PROT_NMEA_ = 0x02;   // NMEA 文本协议
static constexpr uint8_t UBX_COM_PROT_RTCM_ = 0x04;   // RTCM2（差分）
static constexpr uint8_t UBX_COM_PROT_RTCM3_ = 0x08;  // RTCM3（差分/RTK）

}  // namespace bfs

#endif  // SRC_UBX_DEFS_H_
