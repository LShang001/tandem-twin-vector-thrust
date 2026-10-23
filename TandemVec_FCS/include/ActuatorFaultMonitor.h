/**
 * @file    ActuatorFaultMonitor.h
 * @module  固件/控制（执行器故障只读监测，平台无关）
 * @brief   F3 单发推力损失 / F4 舵机卡死的检测器，只输出标志与策略建议
 * @details 输入：roll/pitch 姿态误差 [deg]、上/下摆指令占空比（|cmd|/dMax）、
 *          油门 [0,1]、垂直速度误差 [m/s]、dt [s]；
 *          处理：连续超阈值计时（双计时器互不影响）；
 *          输出：servo_jam()/thrust_loss()/action()。**本类不写执行器**。
 *
 * 边界（F3/F4 判据不同，能力上限也不同）：
 *   F3 推力损失：油门 > 75% 且垂直速度误差 < −1.0 m/s 持续 1.0 s
 *     → 建议 ThrustBoost；定高环可补偿约 30% 推力损失，但不等于故障重构。
 *     ★ 2026-10-23 SIL 观察：本机 SIL 悬停油门约 45–50%，自动定高在
 *     12.5 m/s² 限幅下最大指令约 57%，75% 门限在 SIL 中不可达——但实飞
 *     PID/门限是飞行验证基线，**不按未标定仿真改门限**；是否参数化/下调
 *     须等实机悬停油门与故障日志。当前保持 0.75。
 *   F4 舵机卡死：|姿态误差| > 25° 且 |摆角指令| ≥ 90% dMax 持续 0.5 s
 *     → 建议 ControlledDescent。前摆 δ_f 是滚转唯一主控，卡死是硬件单点，
 *       无法靠控制律恢复（能力边界见 notes/08）。
 *
 * @theory  基于"误差 + 指令饱和 + 持续时间"的证据式故障检测（无传感器位置反馈）
 * @ref     CTL-002 §3（失效模式表：单电机失效/失控）；见
 *          docs/08-参考资料/notes/08-F4单舵机卡死能力边界.md；
 *          AGENTS.md（低速/悬停闭环仿真口径：F4 只能检测+有界下降）
 * @frame   N/A（标量判据；姿态误差沿用控制器角度域定义）
 * @units   角度 deg、占空比 0–1、油门 0–1、速度 m/s、时间 s
 * @note    ① 阈值都是**仿真值**，实机需按噪声/机动包线重新标定；
 *          ② 本类刻意不依赖 Arduino，可在宿主机直接回归；
 *          ③ 检测结果由 flight_control.cpp 转成 g_actuator_* 全局标志，
 *            当前没有自动执行路径（飞行员可超控）。
 * @test    test_host/test_actuator_fault_monitor.cpp
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace bfs
{

/* 执行器故障动作建议：只做监测/策略输出，不直接改控制分配。 */
enum class ActuatorFaultAction : uint8_t
{
  None = 0,
  ControlledDescent, // 舵机卡死/姿态不可控：切油门/有界坠落
  ThrustBoost,       // 单发推力损失：抬油门/配平，由控制层决定是否执行
};

struct ActuatorFaultConfig
{
  float servo_err_deg = 25.0f;    // 姿态误差门限
  float servo_cmd_frac = 0.90f;   // 舵机指令饱和比例（相对 dMax）
  float servo_time_s = 0.50f;     // 持续时间
  float thrust_throttle = 0.75f;  // 油门门限（0-1）；实飞基线，不按 SIL 下调
  float thrust_vel_err_mps = -1.0f; // 垂直速度误差（下沉为负）
  float thrust_time_s = 1.0f;
};

/**
 * @brief 计算 F3 判据的垂直速度误差：测得 − 目标（向上为正，下沉为负）
 * @param[in] vertical_vel_mps 测得垂直速度 [m/s]（向上为正）
 * @param[in] target_vertical_vel_mps 目标垂直速度 [m/s]（向上为正）
 * @return 误差 [m/s]；下沉时为负
 * @formula e_v = v_up,meas − v_up,target
 * @ref CTL-002 §3（单发推力损失判据：油门高且持续下沉）
 * @note ★ 2026-10-23 修正：调用方曾传“目标−测得”，与监测器约定相反，导致
 *       F3 在真实下沉时 e_v>0 永不触发（漏报）。此函数把符号契约收口到一处，
 *       调用方不得自行相减；回归见 test_actuator_fault_monitor.cpp T8。
 */
inline float thrustLossVerticalVelocityError(const float vertical_vel_mps,
                                             const float target_vertical_vel_mps)
{
  return vertical_vel_mps - target_vertical_vel_mps;
}

/**
 * 平台无关的执行器故障监测器。
 *
 * 输入由飞控主循环提供：
 * - roll/pitch 姿态误差（deg）
 * - 上摆/下摆指令占空比（|cmd|/dMax，0-1）
 * - 油门（0-1）
 * - 垂直速度误差（m/s，**测得-目标**，下沉为负；用
 *   `thrustLossVerticalVelocityError()` 计算，禁止调用方自行相减）
 * - 是否启用 F3（手动模式无有效垂直目标时应传 false，避免陈旧目标造成误判）
 *
 * 输出：
 * - servo_jam(): 舵机卡死/姿态持续不可控
 * - thrust_loss(): 单发推力损失/油门饱和仍下沉
 * - action(): ControlledDescent / ThrustBoost / None
 *
 * 设计边界：只做检测与策略建议，不直接写执行器；是否切油门/抬油门由
 * flight_control 层决定。没有舵机位置反馈时，用“姿态误差+指令饱和+持续
 * 时间”作为卡死证据；单发推力损失用“油门高+仍下沉”作为证据。
 */
class ActuatorFaultMonitor
{
public:
  ActuatorFaultMonitor() = default;

  void reset()
  {
    servo_timer_s_ = 0.0f;
    thrust_timer_s_ = 0.0f;
    servo_jam_ = false;
    thrust_loss_ = false;
  }

  void update(const float roll_err_deg,
              const float pitch_err_deg,
              const float df_cmd_frac,
              const float dt_cmd_frac,
              const float throttle_frac,
              const float vertical_vel_err_mps,
              const float dt_s,
              const bool thrust_detection_enabled = true)
  {
    if (!(dt_s > 0.0f) || !std::isfinite(dt_s))
    {
      return;
    }

    const bool roll_bad =
        std::fabs(roll_err_deg) > cfg_.servo_err_deg &&
        std::fabs(df_cmd_frac) >= cfg_.servo_cmd_frac;
    const bool pitch_bad =
        std::fabs(pitch_err_deg) > cfg_.servo_err_deg &&
        std::fabs(dt_cmd_frac) >= cfg_.servo_cmd_frac;
    const bool servo_bad = roll_bad || pitch_bad;
    servo_timer_s_ = servo_bad ? (servo_timer_s_ + dt_s) : 0.0f;
    if (servo_timer_s_ >= cfg_.servo_time_s)
    {
      servo_jam_ = true;
    }

    const bool thrust_bad =
        thrust_detection_enabled &&
        throttle_frac > cfg_.thrust_throttle &&
        vertical_vel_err_mps < cfg_.thrust_vel_err_mps;
    thrust_timer_s_ = thrust_bad ? (thrust_timer_s_ + dt_s) : 0.0f;
    if (thrust_timer_s_ >= cfg_.thrust_time_s)
    {
      thrust_loss_ = true;
    }
  }

  bool servo_jam() const { return servo_jam_; }
  bool thrust_loss() const { return thrust_loss_; }

  ActuatorFaultAction action() const
  {
    if (servo_jam_)
    {
      return ActuatorFaultAction::ControlledDescent;
    }
    if (thrust_loss_)
    {
      return ActuatorFaultAction::ThrustBoost;
    }
    return ActuatorFaultAction::None;
  }

  float servo_timer_s() const { return servo_timer_s_; }
  float thrust_timer_s() const { return thrust_timer_s_; }
  const ActuatorFaultConfig &config() const { return cfg_; }
  void set_config(const ActuatorFaultConfig &cfg) { cfg_ = cfg; }

private:
  ActuatorFaultConfig cfg_{};
  float servo_timer_s_ = 0.0f;
  float thrust_timer_s_ = 0.0f;
  bool servo_jam_ = false;
  bool thrust_loss_ = false;
};

} // namespace bfs
