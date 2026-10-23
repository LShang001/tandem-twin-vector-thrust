/**
 * @file    task_scheduler.h
 * @module  固件/基础设施（任务调度器：TIM8 2kHz 时基）
 * @brief   声明协作式任务调度器的注册、执行与统计接口
 * @details 定时器中断（2kHz）只置任务标志位，任务函数全部在主循环 `taskExecutor()`
 *          中执行——串口/SPI/I2C 等阻塞外设可在任务内安全使用。
 *          输入：`addTaskNamed()` 注册的函数指针 + 间隔（ms，浮点可表达 7.735ms 这类
 *          非整数 tick 周期，需 `BFS_TASK_FRACTIONAL_INTERVALS`）；
 *          处理：tick 计数取模判定到期 → 主循环 FIFO 执行；
 *          输出：任务执行统计（次数/耗时/间隔抖动/迟到/CPU 负荷）。
 *          不变量：无抢占、无优先级；注册顺序即执行顺序。
 * @theory  N/A（调度基础设施）
 * @ref     COMM-001 §3（200Hz 任务节奏来源）、AGENTS.md 踩坑记录（任务调度统计）
 * @frame   N/A
 * @units   间隔 ms、耗时 µs、频率 Hz；硬件 tick = 0.5ms（2kHz）
 * @note    ★ 超时/看门狗语义：调度器本身不检测超时、也不喂狗——
 *          ① “超时”只体现为统计口径：相邻执行间隔 > 1.5×名义周期记为“迟到”
 *            （`late_count`），供 DBG `tasks` 判读主循环是否过载；
 *          ② 真正的硬件兜底是 main.cpp 的 IWDG 3.0s：主循环任一轮被阻塞超过该窗口
 *            即整机复位，因此长阻塞任务（flash 导出、wsseq）必须自行喂狗或压时限；
 *          ③ 任务函数内部不得长时间关中断，否则 TIM8 置位会丢拍（表现为迟到）。
 * @see     src/task_scheduler.cpp、src/task_scheduler_interval.h、src/main.cpp（任务注册段）
 */
#pragma once

#include "state_data.h"

class Print; // Arduino 输出流（HardwareSerial 基类），供 schedulerStatsPrint 参数

/**
 * @brief 定时器中断回调函数 (2kHz)
 *
 * 遍历任务列表，根据预设执行间隔设置任务执行标志位。
 * 实际任务函数在主循环的 taskExecutor() 中执行。
 */
void timerCallback();

/**
 * @brief 初始化硬件定时器 (TIM8, 2kHz)
 */
void setupTimer();

/**
 * @brief 添加任务到调度器
 *
 * @param function 任务函数指针
 * @param interval 执行间隔 (毫秒)
 * @return true 添加成功, false 达到最大任务数
 */
bool addTask(void (*function)(), float interval);

/**
 * @brief 添加带诊断名称的任务到调度器
 *
 * @param function 任务函数指针
 * @param interval 执行间隔 (毫秒)
 * @param name 任务名，供 BFS_TASK_PROFILE 输出识别
 * @return true 添加成功, false 达到最大任务数
 */
bool addTaskNamed(void (*function)(), float interval, const char *name);

/**
 * @brief 任务执行器
 *
 * 在主循环中调用，遍历任务列表执行到期任务。
 */
void taskExecutor();

/**
 * @brief 清零任务调度统计（观测窗口起点）
 *
 * 累积量：执行次数 / 耗时(avg,max) / 相邻执行间隔(min,avg,max) /
 * 迟到次数（间隔 > 1.5×名义周期）/ 任务执行器单轮最大耗时。
 * 统计常开累积（开销≈每次执行 20-30 周期，可忽略）。
 */
void schedulerStatsReset();

/**
 * @brief 打印任务调度统计报告（DBG `tasks` 命令入口）
 *
 * 输出后自动清零，形成观测窗口——连续执行 `tasks` 两次即可对比
 * 前后窗口的实际频率/抖动/迟到/CPU 负荷。实际Hz 与名义Hz 的偏差
 * 直接反映调度健康度（欠频=主循环过载或定时器时钟偏差）。
 *
 * @param out 输出流（DBG 控制台 Serial6）
 */
void schedulerStatsPrint(Print &out);
