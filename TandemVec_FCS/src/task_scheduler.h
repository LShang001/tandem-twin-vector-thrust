/**
 * @file task_scheduler.h
 * @brief 轻量级实时任务调度器
 *
 * 基于2kHz硬件定时器驱动的任务调度系统。
 * 定时器中断设置任务标志位，主循环中执行实际任务函数。
 */
#pragma once

#include "state_data.h"

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
