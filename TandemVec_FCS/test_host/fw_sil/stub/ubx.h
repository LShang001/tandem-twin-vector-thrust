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
// 宿主机 SIL：u-blox 二进制 transport 不需要，固定返回空实现。
namespace bfs {
class Ubx {
public:
  enum Fix { FIX_NONE = 0, FIX_DEAD_RECKONING = 1, FIX_2D = 2, FIX_3D = 3,
             FIX_GNSS_DEAD_RECKONING = 4, FIX_TIME_ONLY = 5 };
  Ubx() {}
  Ubx(void*) {}
  void begin() {}
  void update() {}
};
}
