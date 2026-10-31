/**
 * @file    FailsafeLinkLossPlan.h
 * @module  固件/链路与失效保护（纯平台无关决策函数）
 * @brief   CRSF 断链瞬间的通道合成：判定“是否在空中”并生成 988/1500 通道表
 *
 * 设计动机（2026-10-31 审计 P1-2/P1-3）：
 *   旧实现用“油门杆 > 1100 µs”代理“在空中”。在 `AUTO_ALTITUDE` 下油门杆是
 *   升降速率指令（中心 50%、死区 ±10%，<40% 即下降，0% = -1 m/s），空中主动
 *   下降时杆位与地面待机同形，于是断链被误判成地面 → CH5 写 988 → `is_unlocked`
 *   变假 → 混控走锁定分支 → 电机 0% → 空中上锁坠落。
 *
 *   本模块把判据收敛成单一事实源，固件回调与宿主测试调用**同一份**代码：
 *     在空中 = 已解锁 AND (油门有动力 OR 已离地)。
 *   必要条件用臂状态（唯一可靠的“人是否在飞”信号），佐证用两条互相兜底的
 *   物理证据：低油门下降仍有油门动力；悬停/滑翔低油门还有离地高度。
 *
 * 不变量：
 *   - 未解锁（CH5 ≤ 1500）时任何输入组合都不得判为在空中；
 *   - 空中时 CH1/CH2/CH4 归中、CH3 归中（AUTO 下 = 定高）、CH5 保持解锁、
 *     CH7 落 AUTO_ALTITUDE 中档、其余通道压最低（含 CH6 点火、CH8 手动 TVC）；
 *   - 地面时全 988（电机锁定）；
 *   - 输出通道值只可能是 988 或 1500，不产生中间值。
 *
 * @ref  CTL-001 §8.6（模式与安全联锁）、VER-004（共轴机型原始固件对照）
 * @note 本文件不得包含 Arduino/串口/全局状态依赖，宿主测试直接编译。
 */
#ifndef BFS_FAILSAFE_LINK_LOSS_PLAN_H
#define BFS_FAILSAFE_LINK_LOSS_PLAN_H

#include <cstdint>

namespace bfs {
namespace failsafe {

/** 固件 RC 通道数组长度（state_data.h: RC_INPUT_MAX_CHANNELS 16） */
constexpr int kMaxChannels = 16;

/** 通道压最低值（CRSF_to_US 之后的低端；固件全链路统一该量纲） */
constexpr uint16_t kPwmLow = 988;
/** 通道中位值 */
constexpr uint16_t kPwmMid = 1500;

/** 解锁阈值：CH5 严格大于该值视为已解锁（与 getControlMode 的判据同口径） */
constexpr uint16_t kArmUnlockedUs = 1500;
/** “有动力”阈值：与 manage_pid_integrals 的 throttle_active 同口径 */
constexpr uint16_t kThrottlePoweredUs = 1100;
/** 离地证据阈值：EKF 相对起飞点高度（向上为正）超过该值视为曾离地 */
constexpr float kAirborneHeightM = 0.5f;

/** 通道索引（0-based；与 raw_rc_values 一致） */
constexpr int kChRoll = 0;
constexpr int kChPitch = 1;
constexpr int kChThrottle = 2;
constexpr int kChYaw = 3;
constexpr int kChArm = 4;
constexpr int kChIgnition = 5;
constexpr int kChMode = 6;
constexpr int kChTvcManual = 7;
constexpr int kChAttMode = 8;

/** 断链决策输入（全部取断链瞬间的最后一帧有效值 + 当前导航高度） */
struct Inputs {
  uint16_t arm_us;              /**< CH5 解锁开关原始 PWM */
  uint16_t throttle_us;         /**< CH3 油门杆原始 PWM */
  float estimated_height_m;     /**< EKF 相对起飞点高度，向上为正（m） */
};

/** 断链后的通道计划 */
struct Plan {
  bool in_flight;                       /**< 是否按“飞行中失控”处理 */
  uint16_t channels[kMaxChannels];      /**< 合成后的 16 通道 PWM */
};

/**
 * @brief 判定断链瞬间是否在空中
 *
 * 三个输入都参与，缺一不可退化：
 *   - 去掉 arm_us 必要条件 → 未解锁的地面状态也会保持解锁；
 *   - 只用 arm_us → 地面已武装时断链仍保持武装，高度环会在地面把推力积到悬停附近；
 *   - 只用 throttle_us（旧实现）→ 空中主动下降被判成地面，空中上锁；
 *   - 加上 estimated_height_m → 覆盖“低油门但已离地”的窗口。
 *
 * @pre  调用点在链路超时回调内，raw_rc_values 仍是断链前最后一帧有效值
 */
inline bool isInFlightAtLinkLoss(uint16_t arm_us, uint16_t throttle_us,
                                 float estimated_height_m)
{
  const bool armed = (arm_us > kArmUnlockedUs);
  const bool powered = (throttle_us > kThrottlePoweredUs);
  const bool airborne = (estimated_height_m > kAirborneHeightM);
  return armed && (powered || airborne);
}

/**
 * @brief 生成断链后的 16 通道计划
 *
 * 输出被写入就地的 `Plan`，不读全局量，纯函数，可在宿主机上穷举真值表。
 */
inline void computeLinkLossPlan(const Inputs &in, Plan &out)
{
  out.in_flight = isInFlightAtLinkLoss(in.arm_us, in.throttle_us,
                                       in.estimated_height_m);

  for (int i = 0; i < kMaxChannels; ++i)
  {
    out.channels[i] = kPwmLow;
  }

  // 姿态三轴归中：断链后姿态由自稳环接管，摇杆不得残留断链前的杆位。
  out.channels[kChRoll] = kPwmMid;
  out.channels[kChPitch] = kPwmMid;
  out.channels[kChYaw] = kPwmMid;

  // 油门：空中归中。AUTO_ALTITUDE 下 50% 落在中心死区内 → 目标垂速 0 → 定高；
  // 旧实现把该通道压到 988 即 0%，被高度环解释成“以 1 m/s 下降”。
  out.channels[kChThrottle] = out.in_flight ? kPwmMid : kPwmLow;

  // 解锁开关：空中保持解锁（绝不允许空中上锁）；地面保持锁定。
  out.channels[kChArm] = out.in_flight ? kPwmMid : kPwmLow;

  // 模式开关：空中落 AUTO_ALTITUDE 中档；地面压最低。
  out.channels[kChMode] = out.in_flight ? kPwmMid : kPwmLow;

  // 其余通道（点火 CH6、手动 TVC CH8、姿态模式 CH9 及 10..16）保持压最低：
  // 断链时不点火、不进入手动 TVC 旁路、不改姿态模式。
}

/** 便捷重载：直接返回 Plan，值语义，便于测试与临时对象。 */
inline Plan makeLinkLossPlan(const Inputs &in)
{
  Plan plan{};
  computeLinkLossPlan(in, plan);
  return plan;
}

}  // namespace failsafe
}  // namespace bfs

#endif  // BFS_FAILSAFE_LINK_LOSS_PLAN_H
