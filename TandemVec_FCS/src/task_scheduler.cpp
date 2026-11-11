/**
 * @file    task_scheduler.cpp
 * @module  固件/基础设施（任务调度器：TIM8 6.4kHz 时基）
 * @brief   实现标志位式协作调度、任务注册与调度统计
 *
 * 本文件实现了一个基于硬件定时器中断的协作式任务调度系统。
 * 核心设计思想：定时器中断只负责设置标志位，实际任务在主循环中执行，
 * 避免在中断上下文中执行耗时操作（如串口通信、浮点运算）。
 *
 * 时间基准：TIM8 硬件定时器，scheduler::kSchedulerTickHz（当前 6400Hz / 156.25µs）
 * 最大任务数：MAX_TASKS = 30
 * 调度精度：scheduler::kTickUs（一个滴答，6400Hz 时 156.25µs）
 *
 * @details 输入：`addTaskNamed()` 注册表、`BFS_TASK_FRACTIONAL_INTERVALS` 下的 Q8 目标间隔；
 *          处理：ISR 累加 tick → 到期置 flag 并更新 lastRunTime（可选按 Q8 误差累积
 *                在 N/N+1 tick 间抖动，见 task_scheduler_interval.h）→ 主循环执行；
 *          输出：任务执行计数、耗时、间隔抖动、迟到计数、CPU 负荷（常开累积，
 *                DBG `tasks` 打印后清零，形成观测窗口）。
 *          不变量：先清 flag 再执行（执行期间新到期请求不丢）；标志读改写临界区用
 *          `noInterrupts()/interrupts()` 保护；count 为 uint32 溢出周期约 7.8 天（6400Hz）。
 * @theory  N/A（调度基础设施）
 * @ref     COMM-001 §3（200Hz 节奏）、GOV-004 §5（L1 文档要求）
 * @frame   N/A
 * @units   间隔 ms、tick = scheduler::kTickUs、耗时 µs、频率 Hz、负荷 %
 * @note    统计口径即“软超时”：间隔 > 1.5×名义周期记一次迟到（漏拍）。硬件看门狗
 *          （IWDG 3.0s）在 main.cpp，本文件不涉及。
 */
#include "task_scheduler.h"
#include "sensor_imu.h"
#include "task_scheduler_interval.h"
#include "imu_odr_config.h"   // IMU 采样率唯一真源（ISR 抽点频率由它派生）

#ifdef BFS_PROFILE_TO_ANOCOM_SERIAL
#define BFS_PROFILE_SERIAL Serial6
#else
#define BFS_PROFILE_SERIAL Serial8
#endif

// DBG 模式访问器（communication.cpp 定义）：画像输出仅 DBG 模式下打印，
// 避免 [TASK]/[LOOP] 文本污染正常 AnoCom 遥测流。
bool isDebugModeActive();

// 可选：同周期任务相位错开（默认关，见 platformio.ini [env:TandemVec_FCS_PHASE]）。
// 动机：5ms 任务注册相位几乎相同，会在同一 tick 集中到期；主循环一次 pass
// 若超过 5ms，标志位被下一次到期覆盖 → 实际频率下降（实测 200Hz→约155Hz）。
// 错开只改首次到期相位，不改名义周期；需上板用 tasks 的迟到列 A/B。
#ifndef BFS_TASK_PHASE_STAGGER
#define BFS_TASK_PHASE_STAGGER 0
#endif

// ====================================================================
// 任务调度统计（常开累积，DBG `tasks` 命令消费）
// 每次执行累积：次数 / 耗时 / 相邻执行间隔 / 迟到次数；`tasks` 打印后清零。
// 开销：每任务每次执行约 20-30 周期（480MHz 下可忽略）。
// ====================================================================
struct TaskStats
{
  uint32_t run_count;          // 窗口内执行次数
  uint64_t exec_sum_us;        // 执行总耗时
  uint32_t exec_max_us;        // 单次最大耗时
  uint32_t last_run_us;        // 上次执行 micros()
  uint32_t gap_sum_us;         // 相邻执行间隔累计
  uint32_t gap_max_us;         // 最大执行间隔（抖动上界）
  uint32_t gap_min_us;         // 最小执行间隔（抖动下界）
  uint32_t late_count;         // 迟到次数（间隔 > 1.5×名义周期 = 漏拍）
  uint32_t missed_count;       // 上次 flag 未消费又被 ISR 置位的次数（真正丢执行）
  uint32_t nominal_period_us;  // 名义周期
};
static TaskStats s_stats[MAX_TASKS] = {};
static uint32_t s_stats_window_start_ms = 0;  // 观测窗口起点（毫秒）
// ★ 2026-11-08 修复：窗口基准从 micros() 改为 millis()。
//   micros() 在 32 位上 71.6 分钟就回绕一次，长航时后「窗口」被算小、
//   而 exec_sum_us 是 u64 累加不受回绕影响，于是 CPU 负载被放大成
//   不可能的值（实测 93 分钟后 hp_permille=4090 即 409%）。
//   改用 millis()（49.7 天回绕）后该失真是数量级上的改善；
//   窗口精度降到 1 ms，对几百秒的窗口完全够用。
//   注意：这只是统计口径，不参与任何调度决策，不影响控制时序。
// 单轮执行预算：超过后只推迟 deferrable 任务；控制/导航/IMU 不受影响。
#ifndef BFS_TASK_PASS_BUDGET_US
#define BFS_TASK_PASS_BUDGET_US 1200U
#endif
static uint32_t s_loop_max_pass_us = 0;       // taskExecutor 单轮最大耗时

void schedulerStatsReset()
{
  memset(s_stats, 0, sizeof(s_stats));
  for (uint8_t i = 0; i < taskCount; i++)
  {
    // 1 tick = kTickUs（6400Hz 时 156.25µs）；用 ×16 定点避免整数截断。
    s_stats[i].nominal_period_us = static_cast<uint32_t>(
        (static_cast<uint64_t>(tasks[i].interval) * scheduler::kTickUsTimes16) / 16ULL);
    s_stats[i].gap_min_us = 0xFFFFFFFFu;
    s_stats[i].last_run_us = micros();
  }
  s_stats_window_start_ms = millis();
  s_loop_max_pass_us = 0;
}

void schedulerStatsPrint(arduino::Print &out)
{
  const uint32_t window_ms = millis() - s_stats_window_start_ms;
  const uint64_t window_us = static_cast<uint64_t>(window_ms) * 1000ULL;
  uint64_t total_exec_us = 0;

  out.println(F("[DBG] == 任务调度统计 =="));
  out.println(F("[DBG] 名称         名义Hz 实际Hz 平均us 最大us 间隔min 间隔avg 间隔max 迟到 丢执行 负荷%"));
  for (uint8_t i = 0; i < taskCount; i++)
  {
    const TaskStats &st = s_stats[i];
    if (st.run_count == 0)
    {
      continue;
    }
    const char *name = tasks[i].name != nullptr ? tasks[i].name : "-";
    const uint32_t nominal_hz = st.nominal_period_us > 0 ? 1000000U / st.nominal_period_us : 0U;
    const uint32_t actual_hz = window_ms > 0 ? st.run_count * 1000U / window_ms : 0U;
    const uint32_t avg_us = st.run_count > 0 ? static_cast<uint32_t>(st.exec_sum_us / st.run_count) : 0U;
    const uint32_t avg_gap = st.run_count > 0 ? st.gap_sum_us / st.run_count : 0U;
    const float load_pct = window_us > 0 ? static_cast<float>(st.exec_sum_us) * 100.0f / static_cast<float>(window_us) : 0.0f;
    total_exec_us += st.exec_sum_us;

    out.print(F("[DBG] "));
    out.print(name);
    for (uint8_t pad = static_cast<uint8_t>(strlen(name)); pad < 12; pad++)
    {
      out.print(' ');
    }
    out.print(nominal_hz);
    out.print(F("   "));
    out.print(actual_hz);
    out.print(F("   "));
    out.print(avg_us);
    out.print(F("   "));
    out.print(st.exec_max_us);
    out.print(F("   "));
    out.print(st.gap_min_us);
    out.print(F("   "));
    out.print(avg_gap);
    out.print(F("   "));
    out.print(st.gap_max_us);
    out.print(F("   "));
    out.print(st.late_count);
    out.print(F("   "));
    out.print(st.missed_count);
    out.print(F("   "));
    out.print(load_pct, 1);
    out.println();
  }
  out.print(F("[DBG] TIM8 clk="));
  out.print(TaskTimer->getTimerClkFreq());
  out.print(F("Hz psc="));
  out.print(TaskTimer->getPrescaleFactor());
  out.print(F(" arr="));
  out.print(TaskTimer->getOverflow(TICK_FORMAT));
  out.print(F(" tick_hz="));
  out.println(TaskTimer->getOverflow(HERTZ_FORMAT));
  const float cpu_pct = window_us > 0 ? static_cast<float>(total_exec_us) * 100.0f / static_cast<float>(window_us) : 0.0f;
  out.print(F("[DBG] LOOP 窗口="));
  out.print(window_ms);
  out.print(F("ms CPU="));
  out.print(cpu_pct, 1);
  out.print(F("% 最大单轮="));
  out.print(s_loop_max_pass_us);
  out.println(F("us"));
  schedulerStatsReset();
}

uint32_t schedulerWindowMs()
{
  return millis() - s_stats_window_start_ms;
}

uint32_t schedulerLoopMaxPassUs()
{
  return s_loop_max_pass_us;
}

uint32_t schedulerCpuPermille()
{
  const uint64_t window_us = static_cast<uint64_t>(millis() - s_stats_window_start_ms) * 1000ULL;
  if (window_us == 0U) return 0U;
  uint64_t total = 0;
  for (uint8_t i = 0; i < taskCount; ++i)
  {
    total += s_stats[i].exec_sum_us;
  }
  return static_cast<uint32_t>((total * 1000ULL) / window_us);
}

/**
 * @brief 定时器中断回调函数 (Timer Callback Function)
 *
 * 由 TIM8 硬件定时器以 scheduler::kSchedulerTickHz（当前 6400Hz）频率触发。
 * 功能：遍历已注册的任务列表，检查每个任务是否到达预定执行时间点。
 * 如果到达，则设置该任务的执行标志位 (flag = true)，但不执行实际任务函数。
 *
 * 设计原因：中断服务程序(ISR)应尽可能短小，避免阻塞其他中断。
 * 实际的任务函数在主循环的 taskExecutor() 中执行，保证了：
 *   1. 任务函数可以安全使用串口、I2C、SPI 等阻塞式外设
 *   2. 任务函数之间不会相互中断
 *   3. 高优先级任务（如IMU采集）不会被低优先级任务阻塞
 *
 * 调用频率：scheduler::kSchedulerTickHz（当前 6400Hz）
 */
// 定时器自由运行 tick（6400 Hz）；相位错开需要读取当前 tick。
static volatile uint32_t s_timerTicks = 0;

void timerCallback()
{
  s_timerTicks++; // 每次中断递增，溢出周期约 7.8 天 (2^32 / 6400 / 86400)
#ifdef BFS_IMU_ISR_RING
  static uint16_t s_imu_phase = 0;
  // ★ 2026-11-09：45686 走 FIFO 批量读，ISR 1kHz 每拍取约 3 帧（ODR 3.2k）；
  //   42688 保持 2kHz 每拍 1 帧。生产速率由芯片 ODR 决定，ISR 只决定取数节奏。
  // ★ 2026-11-10：ISR 抽点频率直接等于采样率，由唯一真源派生（src/imu_odr_config.h）。
  //   原来这里按型号手写 3200/2000，与 ODR 分开维护：45686 改到 1600Hz 后 ISR 仍跑
  //   3200Hz，一半中断是空转（实测 isr_empty 约等于 isr_f），既浪费又容易掩盖失配。
  //   现在 ISR 频率 == ODR，一拍一包；若两侧不等，health 的 isr_empty 会立刻暴露。
  s_imu_phase += bfs_imu_odr::kIsrPollHz;
  if (s_imu_phase >= scheduler::kSchedulerTickHz)
  {
    s_imu_phase = static_cast<uint16_t>(s_imu_phase - scheduler::kSchedulerTickHz);
    imuSampleIsr();
  }
#endif
  const uint32_t currentCount = s_timerTicks;

  // 遍历所有已注册的任务
  for (uint8_t i = 0; i < taskCount; i++)
  {
    // 检查条件：
    //   1. tasks[i].enabled: 任务是否处于启用状态
    //   2. (currentCount - tasks[i].lastRunTime >= tasks[i].interval):
    //      距离上次执行是否已过去足够多的滴答数
    //      使用无符号减法，即使 currentCount 溢出也能正确计算时间差
    if (tasks[i].enabled && (currentCount - tasks[i].lastRunTime >= tasks[i].interval))
    {
      if (tasks[i].flag)
      {
        s_stats[i].missed_count++;  // 上一拍还没被主循环消费：本次到期被合并
      }
      tasks[i].flag = true;                // 设置任务可执行标志，通知主循环执行该任务
      tasks[i].lastRunTime = currentCount; // 更新上次执行时间戳，为下次调度做准备
#ifdef BFS_TASK_FRACTIONAL_INTERVALS
      tasks[i].interval = scheduler::nextIntervalTicks(tasks[i].interval_q8,
                                                       tasks[i].interval_error_q8);
#endif
    }
  }
}

/**
 * @brief 初始化硬件定时器 (TIM8)
 *
 * 配置 STM32H743 的 TIM8 定时器产生 scheduler::kSchedulerTickHz 的周期性溢出中断。
 * 时基选择依据（2026-11-01 由 2000Hz 提升到 6400Hz）：
 *   - 必须能精确表达 3.2kHz：ICM-45686 无 2000Hz ODR 档，换型后需 312.5µs
 *     周期，在 2kHz 时基下不足 1 tick（会被抬到 500µs），物理不可达。
 *   - 整除性：6400Hz 下 3.2kHz=2 tick、2kHz=3.2 tick(Q8)、200Hz=32 tick、
 *     500Hz=12.8 tick(Q8)、DPS310 7.735ms=49.504 tick(Q8)，全部准确表达。
 *   - 中断开销：ISR 只遍历任务表并置标志；6400Hz 下理论占用约 1-2%，
 *     ★ 上板必须用 DBG `tasks` 复核主循环余量与迟到计数。
 */
void setupTimer()
{
  TaskTimer->setOverflow(scheduler::kSchedulerTickHz, HERTZ_FORMAT); // 时基见 task_scheduler_interval.h
  TaskTimer->attachInterrupt(timerCallback);  // 将 timerCallback 附加到溢出中断
  TaskTimer->resume();                        // 启动定时器，开始产生中断
  schedulerStatsReset();                      // 调度统计观测窗口起点（任务随后注册）
}

/**
 * @brief 添加任务到调度器
 *
 * 将一个任务函数注册到调度器的任务列表中。
 * 任务注册后默认处于启用状态 (enabled = true)，首次执行将在 interval 滴答后触发。
 *
 * @param function 任务函数指针，函数签名必须为 void func(void)
 * @param interval 任务执行间隔 (毫秒)，会被转换为定时器滴答数
 *                  例如: 05ms -> 1 滴答, 5ms -> 10 滴答, 20ms -> 40 滴答
 * @return true  任务添加成功
 * @return false 任务添加失败 (已达到最大任务数 MAX_TASKS)
 */
bool addTaskNamed(void (*function)(), float interval, const char *name)
{
  // 检查任务列表是否已满
  if (taskCount >= MAX_TASKS)
  {
    return false; // 任务数已达上限，添加失败
  }

  // 填充任务结构体各字段
  tasks[taskCount].function = function; // 绑定任务函数指针
  tasks[taskCount].name = name;
  // 将毫秒间隔转换为定时器滴答数
  // 计算公式：滴答数 = 间隔(ms) × kSchedulerTickHz/1000
  // 时基 6400Hz 时 1ms = 6.4 滴答；例如 5ms -> 32 滴答，0.3125ms -> 2 滴答
#ifdef BFS_TASK_FRACTIONAL_INTERVALS
  tasks[taskCount].interval_q8 = scheduler::intervalMsToTicksQ8(interval);
  tasks[taskCount].interval = scheduler::initialIntervalTicks(tasks[taskCount].interval_q8);
  tasks[taskCount].interval_error_q8 =
      scheduler::initialIntervalErrorQ8(tasks[taskCount].interval_q8);
#else
  tasks[taskCount].interval = static_cast<uint32_t>(
      interval * (static_cast<float>(scheduler::kSchedulerTickHz) / 1000.0f) + 0.5f);
#endif
#if BFS_TASK_PHASE_STAGGER
  // 仅对 >=5ms 的任务错开首次相位，避免高数率任务（IMU/500Hz）被扰动。
  static uint8_t s_phase_seq = 0;
  const uint32_t period_ticks = tasks[taskCount].interval;
  if (period_ticks >= 32U)
  {
    const uint32_t spread = (period_ticks < 64U) ? 8U : 16U;
    const uint32_t offset = static_cast<uint32_t>(s_phase_seq % spread);
    s_phase_seq++;
    tasks[taskCount].lastRunTime = s_timerTicks - offset;
  }
  else
  {
    tasks[taskCount].lastRunTime = 0;
  }
#else
  tasks[taskCount].lastRunTime = 0; // 初始化上次执行时间为 0 (首次执行在 interval 后)
#endif
  tasks[taskCount].enabled = true;  // 默认启用该任务
  tasks[taskCount].deferrable = false;
  tasks[taskCount].flag = false;    // 初始标志位为 false (等待定时器置位)
  // 调度统计基线（首个间隔≈名义周期，不污染抖动统计）
  s_stats[taskCount].nominal_period_us = static_cast<uint32_t>(interval * 1000.0f);
  s_stats[taskCount].gap_min_us = 0xFFFFFFFFu;
  s_stats[taskCount].last_run_us = micros();
  taskCount++;                      // 已注册任务计数递增

  return true; // 任务添加成功
}

bool setTaskDeferrable(const char *name, bool deferrable)
{
  if (name == nullptr)
  {
    return false;
  }
  for (uint8_t i = 0; i < taskCount; ++i)
  {
    if (tasks[i].name != nullptr && strcmp(tasks[i].name, name) == 0)
    {
      tasks[i].deferrable = deferrable;
      return true;
    }
  }
  return false;
}

bool addTask(void (*function)(), float interval)
{
  return addTaskNamed(function, interval, nullptr);
}

/**
 * @brief 任务执行器
 *
 * 在主循环 (loop()) 中被调用，是调度系统的核心执行函数。
 * 遍历任务列表，检查每个任务的执行标志位：
 *   - 如果 flag == true: 清除标志位并执行任务函数
 *   - 如果 flag == false: 跳过该任务
 *
 * 关键设计：
 *   1. 使用 noInterrupts()/interrupts() 保护标志位读取和清除的原子性
 *   2. 先清除标志再执行任务：如果任务执行期间中断再次置位，新的执行请求不会丢失
 *   3. 任务按注册顺序执行 (FIFO)，无优先级抢占机制
 *   4. 如果某个任务执行时间过长，会延迟后续任务的执行
 */
void taskExecutor()
{
  const uint32_t t_pass0 = micros(); // 单轮耗时统计起点
#ifdef BFS_TASK_PROFILE
  // 每 2 秒输出一次各任务执行耗时统计, 用于分析延迟瓶颈。
  static uint32_t s_profile_last_ms = 0;
  static uint32_t s_task_max_us[MAX_TASKS] = {};
  static uint64_t s_task_sum_us[MAX_TASKS] = {};
  static uint32_t s_task_count[MAX_TASKS] = {};
  const uint32_t now_ms = millis();
#endif

  // 遍历所有已注册的任务
  for (uint8_t i = 0; i < taskCount; i++)
  {
    bool should_run = false;

    noInterrupts();
    if (tasks[i].flag)
    {
      tasks[i].flag = false;
      should_run = true;
    }
    interrupts();

    // 本轮预算：超过预算后，把遥测/灯效/日志等 deferrable 任务推迟到下一轮。
    // 控制/导航/IMU 任务不标记，优先级由注册顺序 + 本预算共同保证。
    if (should_run && tasks[i].deferrable &&
        (micros() - t_pass0) > static_cast<uint32_t>(BFS_TASK_PASS_BUDGET_US))
    {
      noInterrupts();
      tasks[i].flag = true;
      interrupts();
      continue;
    }

    if (should_run)
    {
      TaskStats &st = s_stats[i];
      const uint32_t t0 = micros();
      // 间隔统计在执行前采样：任务内部可能触发 reset（DBG `tasks` 命令
      // 在 Debug 任务内处理），若任务返回后再算 gap，t0 早于 reset 时刻
      // → 无符号下溢 → 巨大间隔（实测 4294961991us）
      const uint32_t gap = t0 - st.last_run_us;
      st.last_run_us = t0;
      st.gap_sum_us += gap;
      if (gap > st.gap_max_us)
      {
        st.gap_max_us = gap;
      }
      if (gap < st.gap_min_us)
      {
        st.gap_min_us = gap;
      }
      if (st.nominal_period_us > 0 && gap > (st.nominal_period_us * 3) / 2)
      {
        st.late_count++;
      }

      tasks[i].function();
      const uint32_t t1 = micros();
      const uint32_t exec_us = t1 - t0;
      st.run_count++;
      st.exec_sum_us += exec_us;
      if (exec_us > st.exec_max_us)
      {
        st.exec_max_us = exec_us;
      }

#ifdef BFS_TASK_PROFILE
      const uint32_t elapsed = t1 - t0;

      // 累计统计
      if (elapsed > s_task_max_us[i]) s_task_max_us[i] = elapsed;
      s_task_sum_us[i] += elapsed;
      s_task_count[i]++;
#endif
    }
  }

  // 单轮最大耗时（主循环一个完整 pass 的执行时间，含所有到期任务）
  const uint32_t pass_us = micros() - t_pass0;
  if (pass_us > s_loop_max_pass_us)
  {
    s_loop_max_pass_us = pass_us;
  }

#ifdef BFS_TASK_PROFILE
  // 周期性打印（仅 DBG 模式，避免污染遥测流）
  if (isDebugModeActive() && now_ms - s_profile_last_ms >= 2000)
  {
    const uint32_t window_ms = now_ms - s_profile_last_ms;
    s_profile_last_ms = now_ms;
    uint64_t total_exec_us = 0;
    for (uint8_t i = 0; i < taskCount; i++)
    {
      if (s_task_count[i] == 0) continue;
      const uint32_t avg = static_cast<uint32_t>(s_task_sum_us[i] / s_task_count[i]);
      total_exec_us += s_task_sum_us[i];
      BFS_PROFILE_SERIAL.print("[TASK ");
      BFS_PROFILE_SERIAL.print(i);
      BFS_PROFILE_SERIAL.print(" ");
      BFS_PROFILE_SERIAL.print(tasks[i].name != nullptr ? tasks[i].name : "-");
      BFS_PROFILE_SERIAL.print("] max=");
      BFS_PROFILE_SERIAL.print(s_task_max_us[i]);
      BFS_PROFILE_SERIAL.print("us avg=");
      BFS_PROFILE_SERIAL.print(avg);
      BFS_PROFILE_SERIAL.print("us cnt=");
      BFS_PROFILE_SERIAL.print(s_task_count[i]);
      BFS_PROFILE_SERIAL.println();
    }
    const uint32_t window_us = window_ms * 1000UL;
    const uint32_t load_x10 = window_us > 0
                                  ? static_cast<uint32_t>((total_exec_us * 1000ULL) / window_us)
                                  : 0U;
    BFS_PROFILE_SERIAL.print("[LOOP] cpu=");
    BFS_PROFILE_SERIAL.print(load_x10 / 10U);
    BFS_PROFILE_SERIAL.print(".");
    BFS_PROFILE_SERIAL.print(load_x10 % 10U);
    BFS_PROFILE_SERIAL.print("% task0_period=");
    // 任务 0 实测周期 = 窗口时长 / 执行次数（与时基无关；原写死 2kHz 换算）。
    BFS_PROFILE_SERIAL.print(window_us / (s_task_count[0] > 0 ? s_task_count[0] : 1));
    BFS_PROFILE_SERIAL.print("us task0_cnt=");
    BFS_PROFILE_SERIAL.print(s_task_count[0]);
    BFS_PROFILE_SERIAL.println();
    BFS_PROFILE_SERIAL.println();
    // 清零
    memset(s_task_max_us, 0, sizeof(s_task_max_us));
    memset(s_task_sum_us, 0, sizeof(s_task_sum_us));
    memset(s_task_count, 0, sizeof(s_task_count));
  }
#endif
}
