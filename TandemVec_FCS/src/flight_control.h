/**
 * @file flight_control.h
 * @brief 飞行控制模块：制导律、控制律、TVC混控输出
 *
 * 本模块实现完整的 GNC (制导-导航-控制) 执行流程：
 *
 *   制导律 (Guidance):
 *   - 垂直通道: 高度/速度串级PID -> 垂直推力加速度
 *   - 水平通道: 位置/速度串级PID -> 水平加速度矢量
 *   - 物理模型逆解: 3轴加速度 -> 推力方向矢量 -> 目标姿态四元数
 *
 *   控制律 (Control):
 *   - 姿态外环: 四元数误差 -> 目标角速率 (PID)
 *   - 角速率内环: 角速率误差 -> TVC摆角/电机差速 (PID, 微分先行)
 *   - 偏航环: 摇杆 -> 目标角速率 -> 电机差速 (PID)
 *
 *   混控输出 (Mixer):
 *   - TVC舵机: roll/pitch修正量 -> PWM百分比
 *   - 共轴电机: 油门 ± 偏航差速 -> 双电机PWM
 *
 *   飞行模式:
 *   - MANUAL: 手动油门 + 姿态自稳 (角度/角速率模式)
 *   - AUTO_ALTITUDE: 自动高度 + 手动水平
 *   - AUTO_POSITION: 自动高度 + 自动定点 (光流/KF/GNSS)
 *   - GUIDED: 机载计算机加速度指令制导
 */
#pragma once
#include "state_data.h"

/**
 * @brief GNC 核心调度执行器 (200Hz)
 *
 * 飞行控制主任务，按以下步骤执行完整的制导-导航-控制流程：
 * 1. 处理遥控输入、开关状态、解锁原点设置
 * 2. 决定飞行模式 (MANUAL/AUTO_ALT/AUTO_POS/GUIDED)
 * 3. 管理PID积分项使能
 * 4. 计算推力控制 (油门百分比)
 * 5. 生成目标姿态四元数
 * 6. 执行Roll/Pitch姿态控制器 (TVC摆角)
 * 7. 执行Yaw偏航控制器 (电机差速)
 * 8. 混控输出到执行机构
 */
void runGNCExecutive();

/**
 * @brief 初始化水平位置保持参数
 *
 * 配置位置环和速度环PID的限幅、滤波系数、积分限幅。
 * 重置水平卡尔曼滤波器和推力补偿滤波器。
 * 在首次进入 AUTO_POSITION/AUTO_ALTITUDE 模式时调用。
 */
void initPositionHold();

/**
 * @brief 水平位置控制处理
 *
 * 实现完整的水平定点控制链：
 * 1. Loiter状态机: 悬停(位置环) <-> 移动(速度环)
 * 2. 悬停模式: 物理刹车预测 + 位置PID -> 目标速度
 * 3. 移动模式: 摇杆 -> 机体系速度 -> NED系速度
 * 4. 速度环: 速度PID -> 目标加速度 (圆形限幅)
 * 5. 物理模型逆解: 加速度矢量 -> 推力方向矢量 (全比力归一化)
 * 6. 滤波输出: 低通滤波推力补偿分量
 *
 * @param roll_rc_raw  滚转通道原始值 (988-2012)
 * @param pitch_rc_raw 俯仰通道原始值 (988-2012)
 */
void handlePositionControl(float roll_rc_raw, float pitch_rc_raw);
