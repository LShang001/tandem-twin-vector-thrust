/**
 * @file CCAEGT_Types.h
 * @module 固件/宿主参考（CC-AEGT 实飞制导库，逐字复制，仅测试用）
 * @brief CC-AEGT 算法数据类型定义
 * @details 定义四组 POD 结构：CCAEGT_State_t（飞行器状态）、CCAEGT_Output_t（制导输出）、
 *          CCAEGT_Internal_t（阶段/积分/延迟队列等需持久化内部量）、CCAEGT_Stats_t（统计），
 *          以及阶段与预测结果两个枚举。第三方参考库（箭载机智导）逐字复制，
 *          **仅测试用、不参与固件构建**；结构只承载 1D 标量，不含坐标变换，
 *          不可直接当作固件控制链的数据契约。
 * @frame   1D 竖直剖面：h 为离地高度（向上为正），v 沿同一竖直轴（向上为正、下降为负，
 *          CCAEGT_Output_t::v_lim 是负值给出的速度包络下限）；无姿态/横侧向字段，
 *          不涉及机体三轴轴名
 * @units   h m、v/v_lim/v_err m/s、a_cap m/s²、m/prop kg、T/T_ff/T_fb/T_cmd N、
 *          t s、CCAEGT_Stats_t::last_compute_time_us µs、compute_count/ignite_count 计数、
 *          max_a_cap m/s²、min_altitude m；delay_steps/delay_idx 为步数/索引
 */

#ifndef CCAEGT_TYPES_H
#define CCAEGT_TYPES_H

#include <stdint.h>

/**
 * @brief 飞行阶段枚举
 */
typedef enum
{
    CCAEGT_PHASE_IDLE = 0,     ///< 待机状态
    CCAEGT_PHASE_ASCENT = 1,   ///< 动力上升
    CCAEGT_PHASE_FREEFALL = 2, ///< 自由落体
    CCAEGT_PHASE_BRAKE = 3,    ///< 动力制动
    CCAEGT_PHASE_LANDED = 4,   ///< 已着陆
    CCAEGT_PHASE_CRASHED = 5   ///< 坠毁
} CCAEGT_Phase_t;

/**
 * @brief 预测器结果枚举
 */
typedef enum
{
    CCAEGT_PRED_SAFE = 0,    ///< 安全，可继续自由落体
    CCAEGT_PRED_CRITICAL = 1 ///< 临界，必须立即点火
} CCAEGT_PredictResult_t;

/**
 * @brief 飞行器状态结构体
 */
typedef struct
{
    float h;    ///< 高度 (m)
    float v;    ///< 速度 (m/s)
    float m;    ///< 质量 (kg)
    float prop; ///< 剩余燃料 (kg)
    float T;    ///< 当前推力 (N)
    float t;    ///< 任务时间 (s)
} CCAEGT_State_t;

/**
 * @brief 制导输出结构体
 */
typedef struct
{
    float T_cmd; ///< 推力指令 (N)
    float v_lim; ///< 速度限制 (m/s)
    float a_cap; ///< 制动能力 (m/s^2)
    float T_ff;  ///< 前馈推力 (N)
    float T_fb;  ///< 反馈推力 (N)
    float v_err; ///< 速度误差 (m/s)
} CCAEGT_Output_t;

/**
 * @brief 算法内部状态结构体 (需持久化)
 */
typedef struct
{
    CCAEGT_Phase_t phase; ///< 当前飞行阶段
    float integrator;     ///< PID 积分项
    float *delay_queue;   ///< 延迟队列指针
    uint16_t delay_steps; ///< 延迟队列步数
    uint16_t delay_idx;   ///< 延迟队列索引
    bool initialized;     ///< 初始化标志
} CCAEGT_Internal_t;

/**
 * @brief 算法统计信息结构体
 */
typedef struct
{
    uint32_t compute_count;        ///< 计算次数
    uint32_t ignite_count;         ///< 点火次数
    float max_a_cap;               ///< 最大制动能力
    float min_altitude;            ///< 最低高度
    uint32_t last_compute_time_us; ///< 上次计算耗时 (微秒)
} CCAEGT_Stats_t;

#endif // CCAEGT_TYPES_H