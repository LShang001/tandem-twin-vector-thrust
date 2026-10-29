/*
 * @file    NavImuIncrementSelect.h
 * @brief   导航 IMU 增量源选择器：高动态段用未滤波增量提带宽，惯导健康 Failed 期退回滤波值
 * @module  TandemVec_FCS/include（平台无关导航设施）
 * @details 背景（报告《导弹6DOF-导航SIL-2026-10-27.md》§4.24h/§4.24k-终/§4.24m）：
 *          1) 生产 2 kHz 链路在 `sensor_imu.cpp` 步骤 4 对加计/陀螺做一阶软件低通，
 *             步骤 5 轴映射后写入导航增量缓冲 `imu_delta_*`。低通的时变群延迟会被
 *             自动 GNSS 延时估计器当成"接收机延迟"吸收后回放，大过载段因此群体级退化
 *             （§4.24：30.5 g 生产路径双峰、58% 落入 22–32 m 失效吸引子）。
 *          2) 让导航增量绕过该低通（"提带宽"）可把全矩阵位置误差压低 20–66%，
 *             但在 **IMU 冻结** 故障格上代价 +67.5%（40 seeds 确认）。
 *          3) 同一个故障锁存装在"自动延时应用点"收益为 0（§4.24k-终，已撤回）；
 *             装在**增量源头**（本模块）则完全消除代价（§4.24m：`imu_stuck`
 *             57.62→34.26 m 窗键 / **2.92 m** 健康键，47 格矩阵 41/47 逐位不变、
 *             无一格变差 >1%）。
 *          4) 触发源必须用**需要确认时间**的健康状态（`ins_imu_health` 的 `Failed`：
 *             非有限/确认冻结/无样本），而不是逐帧即时信号——§4.24m-W 组显示
 *             收益取决于回退起点落点：跳过故障前沿、覆盖后半段与恢复期最好，
 *             在故障第一帧就切只拿到默认水平（34.20 vs 2.52 m）。
 *             释放侧不敏感（后持 0.1 s 与 5 s 逐位相同）。
 * @units   时间以"调用节拍数"计（固件 2 kHz、SIL 同频），hold_ticks 默认 10 000 = 5 s
 * @frame   选源本身无坐标系语义（只做"用哪一路增量"的布尔决策）；同文件的
 *          `NavImuSensorRubToBodyFrd()` 承载传感器系 RUB → 机体系 FRD 的定常置换，
 *          供两条路径共用同一份映射（避免内联表达式各写一遍）。
 * @note    总开关关闭时 `select()` 恒返回 Filtered，且不改任何计数语义 ⇒ 默认行为等价现网。
 *          接线级不变量（宿主 T10–T13）：raw / filtered 必须走同一置换，且六轴
 *          一次性取齐，不允许逐轴混选。
 */
#pragma once

#include <cstdint>

namespace bfs
{

/// 导航增量来源。Filtered = 现网生产口径（控制链软件低通之后）。
enum class NavImuSource : uint8_t
{
  Filtered = 0,
  Raw = 1,
};

/// 机体系六轴样本（加速度 g、角速度 dps，尚未换算 SI）。
struct NavImuBodySample
{
  float accel_g[3];
  float gyro_dps[3];
};

/// 传感器系 RUB（右-上-后）→ 机体系 FRD 的定常带符号置换。
/// 与 `sensor_imu.cpp` 步骤 5 逐字一致（2026-08-07 轴置换红线）：
///   bX=-sZ、bY=+sX、bZ=-sY；加速度与角速度使用同一置换。
/// 把该置换放进平台无关头，是为了让"raw 与 filtered 走同一映射"可被宿主测试锁住，
/// 而不是只在两处内联表达式上靠人工比对。
inline NavImuBodySample NavImuSensorRubToBodyFrd(const float accel_rub_g[3],
                                                 const float gyro_rub_dps[3])
{
  NavImuBodySample out{};
  out.accel_g[0] = -accel_rub_g[2];
  out.accel_g[1] = accel_rub_g[0];
  out.accel_g[2] = -accel_rub_g[1];
  out.gyro_dps[0] = -gyro_rub_dps[2];
  out.gyro_dps[1] = gyro_rub_dps[0];
  out.gyro_dps[2] = -gyro_rub_dps[1];
  return out;
}

/// 按选源**一次性取齐六轴**。逐轴混选（部分 raw、部分 filtered）是把两条路径
/// 拼成不存在的第三条，任何情况下都不允许；整结构返回让该约束由类型系统保证。
inline NavImuBodySample NavImuSelectIncrement(const NavImuBodySample &filtered,
                                              const NavImuBodySample &raw,
                                              NavImuSource source)
{
  return (source == NavImuSource::Raw) ? raw : filtered;
}

/// 增量源选择器 + 故障后持锁存。
///
/// 语义（与 §4.24m 实测的"健康键回退"一致）：
/// - `imu_failed` 为真的那一拍：锁存置满 `hold_ticks`，本拍用 Filtered；
/// - 之后每拍递减，未减到 0 之前仍用 Filtered（覆盖恢复瞬态）；
/// - 锁存为 0 且总开关打开：用 Raw（提带宽）；
/// - 总开关关闭：恒 Filtered。
class NavImuIncrementSelector
{
public:
  /// 默认 10 000 拍 @2 kHz = 5 s。取值依据见 §4.24m：释放侧时长不敏感
  /// （H=0.1 s 与 H=5 s 逐位相同），保留 5 s 只为覆盖恢复瞬态。
  static constexpr uint32_t kDefaultHoldTicks = 10000U;

  explicit NavImuIncrementSelector(bool enabled,
                                   uint32_t hold_ticks = kDefaultHoldTicks)
      : enabled_(enabled), hold_ticks_(hold_ticks == 0U ? 1U : hold_ticks)
  {
  }

  /// 本拍应使用的增量源。`imu_failed` 应来自需要确认时间的健康状态
  /// （`ImuHealthState::Failed`），不要传逐帧非有限判据（见 §4.24m-W 组）。
  NavImuSource select(bool imu_failed)
  {
    if (imu_failed)
    {
      hold_ = hold_ticks_;
    }
    else if (hold_ > 0U)
    {
      --hold_;
    }

    if (!enabled_)
    {
      return NavImuSource::Filtered;
    }
    const bool fallback = (hold_ > 0U);
    if (fallback)
    {
      ++fallback_ticks_;
      return NavImuSource::Filtered;
    }
    ++raw_ticks_;
    return NavImuSource::Raw;
  }

  /// 诊断：当前是否处于故障回退中（与 `select()` 返回值解耦，可独立上报）。
  bool inFallback() const { return hold_ > 0U; }
  /// 诊断：累计处于回退状态的拍数（SIL/固件自证据，等价于 §4.24m 的 `nrfb`）。
  uint32_t fallbackTicks() const { return fallback_ticks_; }
  /// 诊断：累计使用未滤波增量的拍数。
  uint32_t rawTicks() const { return raw_ticks_; }
  /// 诊断：当前锁存剩余拍数。
  uint32_t holdRemainingTicks() const { return hold_; }
  bool enabled() const { return enabled_; }
  uint32_t holdTicks() const { return hold_ticks_; }

  void reset()
  {
    hold_ = 0U;
    fallback_ticks_ = 0U;
    raw_ticks_ = 0U;
  }

private:
  bool enabled_ = false;
  uint32_t hold_ticks_ = kDefaultHoldTicks;
  uint32_t hold_ = 0U;
  uint32_t fallback_ticks_ = 0U;
  uint32_t raw_ticks_ = 0U;
};

}  // namespace bfs
