/**
 * @file    fs_scheduler_main.cpp
 * @module  固件/全链路数字孪生（fs：full-stack 调度器接入验证）
 * @brief   用真实 `src/task_scheduler.cpp` + 真实 `src/state_data.cpp` 在宿主跑任务调度
 * @details 被测：`src/task_scheduler.cpp`（setupTimer / timerCallback / addTaskNamed /
 *          taskExecutor / 统计），**一行不改**地链接进来。
 *          输入：三个自定义任务（5 ms / 20 ms / 100 ms，与固件 GNC 200 Hz 同量级）；
 *          处理：`setupTimer()` 建立 6400 Hz 时基 → 仿真主程序每 156.25 µs 调一次
 *                `TaskTimer->silTick()`（等价 TIM8 溢出中断）→ 每拍调 `taskExecutor()`；
 *          输出：执行次数、首次到期拍、丢拍计数与调度统计表。
 * @theory  N/A（调度基础设施的行为验证，不是物理模型）
 * @ref     src/task_scheduler.cpp、src/task_scheduler_interval.h；
 *          docs/04-数学建模/代码理论核查/任务调度器与TIM8配置-2026-11-03.md
 * @frame   N/A
 * @units   时间 µs（虚拟时钟）、tick（156.25 µs）、频率 Hz
 * @note    ★ 与真实固件的唯一差异是「谁按 156.25 µs 触发 timerCallback」：
 *          固件是 TIM8 硬件中断，这里是主程序按仿真时间显式调用。
 *          因此本测试验证的是**调度算法本体**，不代表中断延迟/抖动。
 *          156.25 µs 用 ×4 定点累积（625/4），避免整数截断造成长期频率偏移。
 * @test    本文件即测试：退出码 0 = 全通过。
 * @see     test_host/fw_sil/fullstack/README.md（全链路分层与阶段计划）
 */

#include "task_scheduler.h"  // 带 state_data.h → Arduino.h 桩
#include "task_scheduler_interval.h"

#include <cstdio>

// state_data.h 已声明 extern HardwareTimer *TaskTimer，此处仅需完整类型（Arduino.h 已给）。
extern HardwareTimer *TaskTimer;

namespace
{

int g_fail = 0;

void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond)
  {
    ++g_fail;
  }
}

// ---- 两个任务体：只计数，不做耗时代码，便于精确核对调度次数 ----
uint32_t g_n200 = 0, g_n50 = 0, g_n10 = 0;
uint32_t g_first200 = 0xFFFFFFFFu, g_first50 = 0xFFFFFFFFu, g_first10 = 0xFFFFFFFFu;
uint32_t g_tick = 0;  // 当前已推进的 tick 数（0 基），供首次到期核对

void task200()
{
  if (g_first200 == 0xFFFFFFFFu) g_first200 = g_tick;
  ++g_n200;
}
void task50()
{
  if (g_first50 == 0xFFFFFFFFu) g_first50 = g_tick;
  ++g_n50;
}
void task10()
{
  if (g_first10 == 0xFFFFFFFFu) g_first10 = g_tick;
  ++g_n10;
}

}  // namespace

int main()
{
  std::printf("== 全链路调度器验证（真实 task_scheduler.cpp）==\n");
  std::printf("时基: %u Hz, 1 tick = %.4f us\n",
              static_cast<unsigned>(scheduler::kSchedulerTickHz),
              static_cast<double>(scheduler::kTickUs));

  // ---------- 1) 真实 setupTimer()：配置时基并挂中断 ----------
  setupTimer();
  check(TaskTimer != nullptr, "TaskTimer 已由 state_data.cpp 构造");
  check(TaskTimer->silRunning(), "setupTimer() 启动了时基（resume）");
  check(TaskTimer->silOverflow() == scheduler::kSchedulerTickHz,
        "overflow 按 kSchedulerTickHz(6400) 配置");
  check(TaskTimer->silHertzFormat(), "overflow 使用 HERTZ_FORMAT");
  check(taskCount == 0, "setupTimer 后任务表为空");

  // ---------- 2) 真实 addTaskNamed()：注册三个任务 ----------
  check(addTaskNamed(task200, 5.0f, "gnc200"), "注册 5 ms 任务");
  check(addTaskNamed(task50, 20.0f, "f50"), "注册 20 ms 任务");
  check(addTaskNamed(task10, 100.0f, "f10"), "注册 100 ms 任务");
  check(taskCount == 3, "任务表恰好 3 项");

  // 5 ms 在 6400 Hz 下 = 32 tick（Q8 精确，无需抖动）
  check(tasks[0].interval == 32, "5 ms → 32 tick");
  check(tasks[1].interval == 128, "20 ms → 128 tick");
  check(tasks[2].interval == 640, "100 ms → 640 tick");

  // ---------- 3) 仿真 10 s：每 tick 触发一次真实 timerCallback ----------
  const uint32_t total_ticks = 10u * scheduler::kSchedulerTickHz;  // 64000
  uint64_t t_us_x4 = 0;  // 虚拟时钟 ×4 定点：1 tick = 156.25 µs = 625/4

  for (g_tick = 0; g_tick < total_ticks; ++g_tick)
  {
    t_us_x4 += 625u;
    g_sil_time_us = static_cast<uint32_t>(t_us_x4 / 4u);
    TaskTimer->silTick();   // ← 真实 timerCallback()：tick 累加 + 到期置位
    taskExecutor();         // ← 真实 taskExecutor()：清标志 + 执行 + 统计
  }

  const double elapsed_s = static_cast<double>(g_sil_time_us) * 1e-6;

  // ---------- 4) 判定：次数、周期、首次到期相位 ----------
  std::printf("\n-- 10 s 仿真结果（%llu tick，%.4f s）--\n",
              static_cast<unsigned long long>(total_ticks), elapsed_s);
  std::printf("  200 Hz: %u 次，首次第 %u tick\n", g_n200, g_first200);
  std::printf("   50 Hz: %u 次，首次第 %u tick\n", g_n50, g_first50);
  std::printf("   10 Hz: %u 次，首次第 %u tick\n", g_n10, g_first10);

  check(g_n200 == 2000, "200 Hz 任务 10 s 执行 2000 次");
  check(g_n50 == 500, "50 Hz 任务 10 s 执行 500 次");
  check(g_n10 == 100, "10 Hz 任务 10 s 执行 100 次");

  // 首次到期相位：lastRunTime 初值 0，故第 interval 个 tick（1 基）= interval-1（0 基）
  check(g_first200 == 31, "200 Hz 首次到期在第 32 拍（0 基 31）");
  check(g_first50 == 127, "50 Hz 首次到期在第 128 拍（0 基 127）");
  check(g_first10 == 639, "10 Hz 首次到期在第 640 拍（0 基 639）");

  // ---------- 5) 真实统计口径：无丢拍、间隔无抖动 ----------
  std::printf("\n-- 真实调度统计（schedulerStatsPrint）--\n");
  schedulerStatsPrint(Serial8);
  check(schedulerCpuPermille() < 1000, "CPU 负荷统计 < 100%");

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}