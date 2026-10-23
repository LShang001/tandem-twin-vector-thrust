/**
 * @file    test_actuator_fault_monitor.cpp
 * @module  固件/宿主回归测试（fw-tests：执行器故障监测）
 * @brief   验证舵机卡死/单发推力损失检测的判据、去抖时长与优先级
 * @details 被测：`include/ActuatorFaultMonitor.h`（`bfs::ActuatorFaultMonitor::update()`）。
 *          输入：roll/pitch 姿态角（deg）、δt/δf 摆角（deg）、油门、垂直误差，
 *          以 10 ms 步长连续送入；
 *          处理：监测器按“误差+指令饱和+持续时间”判定故障，输出建议动作；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          运行：g++ -std=c++17 -Iinclude test_host/test_actuator_fault_monitor.cpp -o bin/afm
 * @theory  故障判据来自 CTL-002 可控性与失效模式：单摆（单个执行器）失效后
 *          该轴仍有残余控制能力，故策略是“检测 + 有界下降”而不是恢复。
 *          @ref CTL-002 §3、@ref AGENTS.md（F4 单前摆=滚转唯一主控，硬件单点）
 * @frame   机体系（上摆 δf→滚转、下摆 δt→俯仰的构型语义）
 * @units   角度 deg，油门 0–1，垂直速度误差 m/s，时间 s
 * @note    去抖时长 0.5 s 的量级来自舵机/推力响应时间与 τm=0.28 s 的叠加；
 *          短于该值的尖峰不得触发，否则正常机动会被误判成故障。
 * @test    T1 无故障不触发；T2 滚转舵机卡死；T3 俯仰舵机卡死；
 *          T4 单发推力损失；T5 短时尖峰不误报；T6 reset 清除；
 *          T7 舵机故障优先于推力损失；T4b 油门门限 75% 边界；
 *          T8 F3 垂直速度误差符号契约；
 *          T9 F3 使能开关（手动模式禁用）；T10 旧反向符号会漏报（回归）
 * @see     TandemVec_FCS/include/ActuatorFaultMonitor.h、docs/05-控制与分配/CTL-002-可控性与失效模式.md
 */
// 2026-10 F3/F4 故障矩阵配套：判据为“误差 + 指令饱和 + 持续时间”。
#include <cmath>
#include <cstdio>

#include "ActuatorFaultMonitor.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(const char *name, const bool ok)
{
  if (ok)
  {
    ++g_pass;
    std::printf("[PASS] %s\n", name);
  }
  else
  {
    ++g_fail;
    std::printf("[FAIL] %s\n", name);
  }
}

static void step_for(bfs::ActuatorFaultMonitor &m,
                     const float roll, const float pitch,
                     const float df, const float dt,
                     const float thr, const float verr,
                     const float seconds)
{
  const float h = 0.01f;
  for (float t = 0.0f; t < seconds; t += h)
  {
    m.update(roll, pitch, df, dt, thr, verr, h);
  }
}

/** 只驱动 F3 通道的便捷入口（姿态误差/摆角固定为无故障值）。 */
static void step_for_f3(bfs::ActuatorFaultMonitor &m,
                        const float thr, const float verr,
                        const float seconds, const bool enabled)
{
  const float h = 0.01f;
  for (float t = 0.0f; t < seconds; t += h)
  {
    m.update(2.0f, 2.0f, 0.2f, 0.2f, thr, verr, h, enabled);
  }
}

int main()
{
  using bfs::ActuatorFaultAction;

  // T1 无故障：短时正常输入不触发
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 2.0f, 1.0f, 0.2f, 0.2f, 0.5f, 0.0f, 0.6f);
    check("T1 无故障不触发", !m.servo_jam() && !m.thrust_loss() &&
                                 m.action() == ActuatorFaultAction::None);
  }

  // T2 滚转舵机卡死：大误差 + 指令饱和 0.6 s
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 30.0f, 2.0f, 0.97f, 0.1f, 0.6f, 0.0f, 0.6f);
    std::printf("[INFO] T2 servo_t=%.2f action=%d\n",
                m.servo_timer_s(), static_cast<int>(m.action()));
    check("T2 滚转卡死检测", m.servo_jam());
    check("T2 动作=受控下降", m.action() == ActuatorFaultAction::ControlledDescent);
  }

  // T3 俯仰舵机卡死
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 2.0f, 30.0f, 0.1f, 0.97f, 0.6f, 0.0f, 0.6f);
    check("T3 俯仰卡死检测", m.servo_jam());
  }

    // T4 单发推力损失：油门高 + 持续下沉 1.1 s
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 2.0f, 2.0f, 0.2f, 0.2f, 0.90f, -2.0f, 1.1f);
    std::printf("[INFO] T4 thrust_t=%.2f action=%d\n",
                m.thrust_timer_s(), static_cast<int>(m.action()));
    check("T4 推力损失检测", m.thrust_loss());
    check("T4 动作=抬油门", m.action() == ActuatorFaultAction::ThrustBoost);
  }

  // T4b 门限边界：0.75（实飞基线；SIL 因悬停油门较低可能达不到，
  //     但不在未标定仿真上改门限）
  {
    bfs::ActuatorFaultMonitor low;
    step_for(low, 2.0f, 2.0f, 0.2f, 0.2f, 0.74f, -2.0f, 1.1f);
    check("T4b 油门低于 75% 不触发", !low.thrust_loss());
    bfs::ActuatorFaultMonitor hi;
    step_for(hi, 2.0f, 2.0f, 0.2f, 0.2f, 0.76f, -2.0f, 1.1f);
    check("T4b 油门高于 75% 触发", hi.thrust_loss());
  }

  // T5 短时尖峰不误报（0.3 s < 0.5 s）
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 30.0f, 2.0f, 0.97f, 0.1f, 0.6f, 0.0f, 0.3f);
    check("T5 短时尖峰不误报", !m.servo_jam());
  }

  // T6 reset 清除
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 30.0f, 2.0f, 0.97f, 0.1f, 0.6f, 0.0f, 0.6f);
    m.reset();
    check("T6 reset 清除", !m.servo_jam() && !m.thrust_loss() &&
                               m.action() == ActuatorFaultAction::None);
  }

  // T7 舵机优先于推力：同时满足时先受控下降
  {
    bfs::ActuatorFaultMonitor m;
    step_for(m, 30.0f, 2.0f, 0.97f, 0.1f, 0.95f, -2.0f, 1.1f);
    check("T7 舵机优先级", m.servo_jam() && m.thrust_loss() &&
                               m.action() == ActuatorFaultAction::ControlledDescent);
  }

  // T8 F3 垂直速度误差符号契约：测得−目标，下沉为负
  //    历史缺陷：调用方写成“目标−测得”，真实下沉时误差为正 → 永不触发。
  {
    const float sink_err = bfs::thrustLossVerticalVelocityError(-2.0f, 0.0f);
    const float climb_err = bfs::thrustLossVerticalVelocityError(1.0f, 0.0f);
    const float target_desc = bfs::thrustLossVerticalVelocityError(-2.0f, -0.5f);
    const float legacy_inverted = 0.0f - (-2.0f); // 旧写法
    check("T8 下沉时 e_v<0（测得−目标）", sink_err < -1.0f);
    check("T8 爬升时 e_v>0", climb_err > 0.0f);
    check("T8 目标下降时仍按相对误差", target_desc < -1.0f);
    check("T8 旧反向写法会得到正误差（漏报根因）", legacy_inverted > 0.0f);
  }

  // T9 F3 使能开关：手动模式禁用时，即使高油门+持续下沉也不触发
  {
    bfs::ActuatorFaultMonitor off;
    step_for_f3(off, 0.90f, -2.0f, 1.1f, false);
    check("T9 禁用 F3 不触发", !off.thrust_loss());

    bfs::ActuatorFaultMonitor on;
    step_for_f3(on, 0.90f, -2.0f, 1.1f, true);
    check("T9 启用 F3 触发", on.thrust_loss());
  }

  // T10 回归：若按旧符号传入 +2.0（爬升方向），高油门也不得触发
  {
    bfs::ActuatorFaultMonitor m;
    step_for_f3(m, 0.95f, 2.0f, 1.1f, true);
    check("T10 正误差（爬升）不触发 ThrustBoost", !m.thrust_loss());
  }

  std::printf("\n=== 结果: %d 通过, %d 失败 ===\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
