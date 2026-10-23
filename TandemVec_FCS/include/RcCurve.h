/**
 * @file    RcCurve.h
 * @module  固件/控制（FPV 摇杆曲线，平台无关，可宿主机回归）
 * @brief   FPV 摇杆曲线（Betaflight 风格，平台无关 host 可测）
 * @details 输入：归一化杆量 rc ∈ [-1,1]（已含死区）与三参数；
 *          处理：expo 压中心 → ×rcRate×200 得基准角速率 → 除以
 *          (1−|rcCmd|·super) 双曲放大边缘 → 限幅；
 *          输出：目标角速率 [deg/s]。被 flight_control.cpp 的 RATE_MODE
 *          （roll/pitch）与 execute_yaw_controller（yaw）调用。
 *
 * 三参数模型（2026-08-11 接入 RATE_MODE）：
 *   rc_rate  全局灵敏度（0-2.5，默认 1.0）——线性缩放整条曲线
 *   rc_expo  输入侧中心曲线（0-1）——压中心斜率（rc=0 处增益 1-expo，满杆增益 1）
 *   rc_super 输出侧边缘曲线（0-1，默认 roll/pitch 0.7、yaw 0.55）——
 *            双曲增益 1/(1-|rcCmd|·super)，只抬边缘，满杆增益 1/(1-super)
 *
 * 满杆角速度 = rcRate × 200 / (1 - super)（200 = Betaflight 基准 deg/s）
 * 极点保护：|rcCmd|·super ≥ 0.9 时钳制（super≥1 会落入杆量范围，故 super 限 [0,1)）
 * 死区：归一化 |rc| < RC_DEADBAND_NORM（≈10us）归零
 *
 * @theory  Betaflight 三参数摇杆曲线
 * @ref     TandemVec-Paper/ch/07-control.tex (eq:rc_curve)、§"摇杆曲线"；
 *          @ref CTL-001 §8.6（固件对照表的 FPV 曲线行）；
 *          AGENTS.md 踩坑记录"FPV 摇杆曲线（2026-08-11，RATE_MODE）"
 * @frame   N/A（标量映射：杆量 → 目标角速率，坐标系由调用方决定）
 * @units   杆量归一化无量纲；输出 deg/s；200 为 Betaflight 基准 deg/s；
 *          RC_DEADBAND_NORM=0.02 对应 ±10 µs（半程 512 µs）
 * @test    test_host/test_rc_curve.cpp
 */
#pragma once
#include <cmath>
#include <algorithm>

#define RC_DEADBAND_NORM 0.02f   // 归一化死区（±10us @512us 半程）

// 归一化摇杆（PWM µs → [-1,1]；988–2012 映射到 ±1）
// @formula rc = (us − 1500) / 512，半程 512 µs 对应满杆 ±1
// @note 输入超出 988–2012 时不额外钳位（调用方/曲线末级负责限幅）
static inline float rcNormFromUs(float us)
{
    return (us - 1500.0f) / 512.0f;
}

// Expo 输入曲线（Betaflight：rc·(expo·rc² + (1−expo))）
// @formula rcCmd = rc·(expo·rc² + 1 − expo)
// @note rc=0 处斜率 = 1−expo（中心更细腻），|rc|=1 处增益恰为 1（满杆不放大）
static inline float rcExpoCurve(float rc, float expo)
{
    if (expo <= 0.0f)
        return rc;
    return rc * (expo * rc * rc + (1.0f - expo));
}

/**
 * @brief 完整摇杆曲线：归一化杆量 → 目标角速率 (deg/s)
 * @param rc        归一化杆量 [-1,1]（死区在本函数内处理）
 * @param rcRate    全局灵敏度（0-2.5）
 * @param expo      输入曲线（0-1）
 * @param super     边缘曲线（0-1，<1 防极点）
 * @param maxRate   角速率上限（限幅，固件 RATE_MODE 传 600 deg/s）
 * @return 目标角速率 [deg/s]，|值| ≤ maxRate
 * @formula ω_ref = maxRate 限幅后的 [ 200·rcRate·rcExpoCurve(rc,expo) /
 *          (1 − |rcExpoCurve(rc,expo)|·super) ]
 * @ref TandemVec-Paper/ch/07-control.tex (eq:rc_curve)；
 *      AGENTS.md 踩坑记录（限幅 600 与姿态外环 80 解耦）
 * @pre  maxRate > 0；super ∈ [0,1)；expo ∈ [0,1]（越界由调用方参数表约束）
 * @note ① 死区用 RC_DEADBAND_NORM（0.02）整体归零，不做平滑过渡；
 *       ② 极点保护把分母钳到 0.1（|rcCmd|·super ≥ 0.9 时），避免 super→1 爆炸；
 *       ③ maxRate 必须大于曲线满杆值，否则曲线在半杆就被削平（2026-08-11 事故：
 *          误用姿态外环 80 deg/s 限幅，57 deg/s 处即被截断）。
 */
static inline float rcRateCurve(float rc, float rcRate, float expo, float super,
                                float maxRate)
{
    if (fabsf(rc) < RC_DEADBAND_NORM)
        rc = 0.0f;
    float rcCmd = rcExpoCurve(rc, expo);
    float rate = rcCmd * rcRate * 200.0f;                 // Betaflight 基准 200°/s
    if (super > 0.0f)
    {
        float denom = 1.0f - fabsf(rcCmd) * super;
        if (denom < 0.1f)
            denom = 0.1f;                                 // 极点保护（super≥0.9 时钳制）
        rate = rate / denom;
    }
    return std::max(-maxRate, std::min(maxRate, rate));
}
