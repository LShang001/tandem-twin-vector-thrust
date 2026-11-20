/**
 * @file    ubx.h
 * @module  固件/宿主 SIL 桩（fw-tests：u-blox 驱动替身）
 * @brief   提供 bfs::Ubx 的类型与 Fix 枚举声明，禁掉真实串口 transport
 * @details 被测：无（替身层）。SIL 直接把 GNSS 观测写入 `state_data`，不经过
 *          UBX 二进制解析，因此 begin/update 为空实现。
 * @theory  N/A（驱动替身）
 * @frame   N/A
 * @units   N/A
 * @note    与真实驱动的差异：无帧同步、无校验、无 PVT 解码——UBX 解析正确性
 *          只能由 GCS/固件侧解析测试覆盖，不能由 SIL 证明。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_env_main.cpp
 */
#pragma once

// ★全链路数字孪生（2026-11-20）：定义 BFS_REAL_UBX 时改用**真实 UBX 驱动**
//   （lib/ublox-main/src/ubx.h），否则沿用本文件的极简桩。
//
// 为什么需要这个开关：本桩只提供 Fix 枚举与空 begin/update，没有 nav_pvt_count /
// num_sv / lat_deg / north_vel_mps 等遥测与导航要用的字段。通信层一接进来就会
// 因缺字段编译失败——继续给桩加字段等于在本文件里重抄一遍真实驱动的数据成员，
// 那既不"同一份源码"也必然漂移。正确做法是换回真实头。
//
// 既有 SIL（fw_sil_env_main 等）不定义该宏，行为与本文件原先完全一致。
#ifdef BFS_REAL_UBX
#include "../../../lib/ublox-main/src/ubx.h"
#else

// 宿主机 SIL：u-blox 二进制 transport 不需要，固定返回空实现。
namespace bfs {
class Ubx {
public:
  enum Fix { FIX_NONE = 0, FIX_DEAD_RECKONING = 1, FIX_2D = 2, FIX_3D = 3,
             FIX_GNSS_DEAD_RECKONING = 4, FIX_TIME_ONLY = 5 };
  enum class GpsProtocol { kAuto = 0, kUbx = 1, kNmea = 2 };
  Ubx() {}
  Ubx(void*) {}
  void begin() {}
  void update() {}
  void Config(void*) {}
  void SetProtocol(GpsProtocol) {}
  bool Pump(uint32_t, uint32_t) { return false; }
  void Reset() {}
  void ClearEpochQueue() {}
};
}

#endif  // BFS_REAL_UBX
