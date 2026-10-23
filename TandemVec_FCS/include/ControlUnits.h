/**
 * @file    ControlUnits.h
 * @module  固件/控制（单位边界）
 * @brief   控制器角度域（deg/s²）与刚体动力学 SI 域（rad/s²）的唯一换算点
 * @details 输入：控制器输出的角加速度 [deg/s²]；输出：物理层角加速度 [rad/s²]。
 *          约定：所有角加速度只允许在本文件做换算——控制器内部一律角度域，
 *          惯量逆解/力矩分配一律 SI 域，避免"混合域"再次出现（2026-08-11 事故）。
 *          头部注释"控制器人机接口采用角度制、刚体动力学采用 SI 制"是设计意图，
 *          与 @ref AGENTS.md 速率环量纲踩坑记录一致。
 * @theory  纯单位换算，无动力学模型：1 deg = π/180 rad
 * @frame  N/A（标量单位换算，与坐标系无关）
 * @units  kDegPerRad = 180/π ≈ 57.29578；kRadPerDeg = π/180 ≈ 0.0174533
 * @redline 速率环量纲（改动会让所有增益的物理意义差 57.3 倍）
 * @note    constexpr 实现：编译期可折叠，零运行时开销；仅用于标量，不做量纲检查。
 * @see     FlightCtrlParams.h（增益单位）、flight_control.cpp（调用点）
 */
#pragma once

// 控制器人机接口采用角度制，刚体动力学采用 SI 制。
// 所有角加速度只允许在控制器/物理层边界通过本文件转换。
namespace ControlUnits
{
static constexpr float kDegPerRad = 57.29577951308232f;
static constexpr float kRadPerDeg = 0.017453292519943295f;

/**
 * @brief 角加速度 deg/s² → rad/s²
 * @param[in] alpha_dps2 角加速度 [deg/s²]
 * @return 角加速度 [rad/s²]
 * @formula α[rad/s²] = α[deg/s²] · π/180
 * @ref AGENTS.md 踩坑记录（速率环量纲已统一，2026-08-11）
 * @note 控制器输出侧唯一出口；反向换算见 radps2ToDps2（遥测/日志用）
 */
constexpr float dps2ToRadps2(float alpha_dps2)
{
    return alpha_dps2 * kRadPerDeg;
}

/**
 * @brief 角加速度 rad/s² → deg/s²（遥测/离线复算用）
 * @param[in] alpha_radps2 角加速度 [rad/s²]
 * @return 角加速度 [deg/s²]
 * @formula α[deg/s²] = α[rad/s²] · 180/π
 * @ref AGENTS.md 踩坑记录；见 TandemVec_OnlineID.h（i_term 换算）
 * @note 注意 0x21/CAN 帧仍以 rad/s² 为兼容口径，adps2_* 变量才是角度域
 */
constexpr float radps2ToDps2(float alpha_radps2)
{
    return alpha_radps2 * kDegPerRad;
}
} // namespace ControlUnits
