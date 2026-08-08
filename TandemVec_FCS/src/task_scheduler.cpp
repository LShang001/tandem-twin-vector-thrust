/**
 * @file task_scheduler.cpp
 * @brief 轻量级实时任务调度器实现
 *
 * 本文件实现了一个基于硬件定时器中断的协作式任务调度系统。
 * 核心设计思想：定时器中断只负责设置标志位，实际任务在主循环中执行，
 * 避免在中断上下文中执行耗时操作（如串口通信、浮点运算）。
 *
 * 时间基准：TIM8 硬件定时器，2kHz (05ms 周期)
 * 最大任务数：MAX_TASKS = 30
 * 调度精度：05ms (一个定时器滴答)
 */
#include "task_scheduler.h"
#include "task_scheduler_interval.h"

#ifdef BFS_PROFILE_TO_ANOCOM_SERIAL
#define BFS_PROFILE_SERIAL Serial6
#else
#define BFS_PROFILE_SERIAL Serial8
#endif

// DBG 模式访问器（communication.cpp 定义）：画像输出仅 DBG 模式下打印，
// 避免 [TASK]/[LOOP] 文本污染正常 AnoCom 遥测流。
bool isDebugModeActive();

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
  uint32_t nominal_period_us;  // 名义周期
};
static TaskStats s_stats[MAX_TASKS] = {};
static uint32_t s_stats_window_start_us = 0;  // 观测窗口起点
static uint32_t s_loop_max_pass_us = 0;       // taskExecutor 单轮最大耗时

void schedulerStatsReset()
{
  memset(s_stats, 0, sizeof(s_stats));
  for (uint8_t i = 0; i < taskCount; i++)
  {
    s_stats[i].nominal_period_us = tasks[i].interval * 500U; // tick=0.5ms
    s_stats[i].gap_min_us = 0xFFFFFFFFu;
    s_stats[i].last_run_us = micros();
  }
  s_stats_window_start_us = micros();
  s_loop_max_pass_us = 0;
}

void schedulerStatsPrint(Print &out)
{
  const uint32_t window_us = micros() - s_stats_window_start_us;
  const uint32_t window_ms = window_us / 1000U;
  uint64_t total_exec_us = 0;

  out.println(F("[DBG] == 任务调度统计 =="));
  out.println(F("[DBG] 名称         名义Hz 实际Hz 平均us 最大us 间隔min 间隔avg 间隔max 迟到 负荷%"));
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
    out.print(load_pct, 1);
    out.println();
  }
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

/**
 * @brief 定时器中断回调函数 (Timer Callback Function)
 *
 * 由 TIM8 硬件定时器以 2kHz (每 05ms) 频率触发。
 * 功能：遍历已注册的任务列表，检查每个任务是否到达预定执行时间点。
 * 如果到达，则设置该任务的执行标志位 (flag = true)，但不执行实际任务函数。
 *
 * 设计原因：中断服务程序(ISR)应尽可能短小，避免阻塞其他中断。
 * 实际的任务函数在主循环的 taskExecutor() 中执行，保证了：
 *   1. 任务函数可以安全使用串口、I2C、SPI 等阻塞式外设
 *   2. 任务函数之间不会相互中断
 *   3. 高优先级任务（如IMU采集）不会被低优先级任务阻塞
 *
 * 调用频率：2kHz (每 05ms 执行一次)
 */
void timerCallback()
{
  static uint32_t currentCount = 0; // 静态中断计数器，作为全局时间基准 (单位: 定时器滴答)
  currentCount++;                   // 每次中断递增，溢出周期约 248 天 (2^32 / 2000 / 86400)

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
 * 配置 STM32H743 的 TIM8 定时器产生 2kHz 的周期性溢出中断。
 * 2kHz 频率的选择依据：
 *   - 足够高：可以精确调度 1ms 级别的任务 (如 IMU 2kHz 采集)
 *   - 足够低：中断开销可接受 (每次中断约 1-2us，CPU 占用约 02%-04%)
 *   - 整数倍：200Hz 任务 = 每 10 个滴答执行一次，50Hz = 每 40 个滴答
 */
void setupTimer()
{
  TaskTimer->setOverflow(2000, HERTZ_FORMAT); // 设置溢出频率为 2000Hz (2kHz)
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
  // 计算公式：滴答数 = 间隔(ms) * 频率(kHz) = 间隔(ms) * 2
  // 例如：5ms -> 5 * 2 = 10 滴答
#ifdef BFS_TASK_FRACTIONAL_INTERVALS
  tasks[taskCount].interval_q8 = scheduler::intervalMsToTicksQ8(interval);
  tasks[taskCount].interval = scheduler::initialIntervalTicks(tasks[taskCount].interval_q8);
  tasks[taskCount].interval_error_q8 =
      scheduler::initialIntervalErrorQ8(tasks[taskCount].interval_q8);
#else
  tasks[taskCount].interval = static_cast<uint32_t>(interval * 2.0f);
#endif
  tasks[taskCount].lastRunTime = 0; // 初始化上次执行时间为 0 (首次执行在 interval 后)
  tasks[taskCount].enabled = true;  // 默认启用该任务
  tasks[taskCount].flag = false;    // 初始标志位为 false (等待定时器置位)
  // 调度统计基线（首个间隔≈名义周期，不污染抖动统计）
  s_stats[taskCount].nominal_period_us = static_cast<uint32_t>(interval * 1000.0f);
  s_stats[taskCount].gap_min_us = 0xFFFFFFFFu;
  s_stats[taskCount].last_run_us = micros();
  taskCount++;                      // 已注册任务计数递增

  return true; // 任务添加成功
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
    BFS_PROFILE_SERIAL.print(2000000UL / (s_task_count[0] > 0 ? s_task_count[0] : 1));
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
