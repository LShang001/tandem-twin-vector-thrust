/**
 * @file    ControlContracts.h
 * @module  固件/控制（控制层与机型层之间的显式契约）
 * @brief   定义制导→姿态→分配→执行器的数据契约与饱和位，不含任何机型公式
 * @details 本文件只放"层与层之间传什么"，不放"某个机型怎么算"：
 *            GuidanceCommand     制导层输出（NED 目标加速度 + 航向）
 *            AttitudeSetpoint    目标姿态 + 总推力（由动力分配层给出）
 *            AlphaCommand        控制层输出（机体系三轴角加速度，FRD）
 *            AllocatorStatus     分配器回报（力矩指令/实际/残差/饱和位）
 *            ActuatorCommand     执行器归一化命令（由机型输出表映射到槽位）
 *          **坐标系约定**：机体系 FRD — x=滚转轴、y=俯仰轴、z=偏航/推力轴
 *          （悬停时 z 指地）。固件历史文档里的"悬停存档系"与本 FRD 的
 *          x/y/z 轴名一致；TandemVec 分配器内部的模型系置换只发生在
 *          分配器内部，不得泄漏到契约里。
 * @units  加速度 m/s²、角加速度 rad/s²、力矩 N·m、推力 N、角度 rad、时间 s
 * @note    契约是"行为边界"，不是配置表：所有字段的坐标系/单位/饱和责任
 *          必须写在这里；机型参数（力臂、惯量、增益）放各机型 allocator。
 * @see     CTL-003 §2（多机型分层）、TandemVecAllocator.h、CoaxialAllocator.h
 * @test    test_host/test_allocator_contracts.cpp（契约与轴序）
 */
#ifndef BFS_CONTROL_CONTRACTS_H
#define BFS_CONTROL_CONTRACTS_H

#include <cmath>
#include <cstdint>

namespace bfs
{
namespace control
{

/** @brief 机体系三轴索引（FRD） */
enum ControlAxis : uint8_t
{
    kAxisRoll = 0,
    kAxisPitch = 1,
    kAxisYaw = 2,
    kAxisCount = 3
};

/**
 * @brief 饱和/失效位。分配器负责置位并回报，控制层据此做抗饱和。
 * @note 这不是故障码；故障码仍走 ActuatorFaultMonitor。
 */
enum ControlSaturationBits : uint16_t
{
    kSatNone = 0u,
    kSatAlphaRoll = 1u << 0,  /**< 滚转角加速度被物理权限钳位 */
    kSatAlphaPitch = 1u << 1, /**< 俯仰角加速度被物理权限钳位 */
    kSatAlphaYaw = 1u << 2,   /**< 偏航角加速度被物理权限钳位 */
    kSatTvcRoll = 1u << 3,    /**< 滚转舵机指令被机械行程钳位 */
    kSatTvcPitch = 1u << 4,   /**< 俯仰舵机指令被机械行程钳位 */
    kSatYawDiff = 1u << 5,    /**< 差速指令被 ±max_yaw_diff 钳位 */
    kSatLowThrust = 1u << 6,  /**< 推力过低，TVC 权限按 0 处理 */
    kSatMotorClamp = 1u << 7, /**< 电机 µs 被行程钳位 */
    kSatInvalid = 1u << 8,    /**< 输入非有限/参数非法，输出已回零 */
    kSatRateCoupling = 1u << 9 /**< 陀螺耦合项被限幅（保留位） */
};

/** @brief 机体系三维向量（FRD；单位见使用处） */
struct ControlVec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief 制导层输出：NED 目标加速度 + 航向目标
 * @note 动力分配层负责 a_cmd → (q_target, T_cmd)；本契约不指定倾角策略。
 */
struct GuidanceCommand
{
    ControlVec3 a_cmd_mps2_ned{};
    float yaw_cmd_rad = 0.0f;
    bool yaw_valid = false;
    bool valid = false;
};

/**
 * @brief 目标姿态 + 总推力（动力分配层输出，姿态控制层输入）
 * @note thrust_n 是沿机体 -z（FRD 下"上"）的总推力，不含重力。
 */
struct AttitudeSetpoint
{
    float q_w = 1.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float thrust_n = 0.0f;
    bool valid = false;
};

/**
 * @brief 控制层输出：机体系三轴目标角加速度（FRD，rad/s²）
 * @note 控制层不知道惯量；分配器负责 τ = I·α + ω×(I·ω)。
 */
struct AlphaCommand
{
    ControlVec3 alpha_body_radps2{};
    bool valid = false;
};

/**
 * @brief 分配器回报：力矩指令/实际/残差 + 饱和位
 * @note torque_residual_nm 取三轴残差的最大绝对值（N·m）；
 *       饱和时 >0，控制层可据此冻结积分或降低外环指令。
 */
struct AllocatorStatus
{
    ControlVec3 torque_cmd_nm{};
    ControlVec3 torque_achieved_nm{};
    float torque_residual_nm = 0.0f;
    uint16_t sat_mask = kSatNone;
    bool valid = false;
};

/**
 * @brief 执行器归一化命令（0–1 或 µs 由 OutputChannelSpec 解释）
 * @note 该结构只描述"槽位 + 归一化值"，不描述物理引脚；
 *       引脚由 AirframeSpec/OutputChannelSpec 唯一拥有。
 */
struct ActuatorCommand
{
    float channel[8] = {0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t count = 0;
    bool valid = false;
};

inline bool controlFinite(const float v)
{
    return std::isfinite(v);
}

inline bool controlFinite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline float controlClamp(const float v, const float lo, const float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

} // namespace control
} // namespace bfs

#endif // BFS_CONTROL_CONTRACTS_H
