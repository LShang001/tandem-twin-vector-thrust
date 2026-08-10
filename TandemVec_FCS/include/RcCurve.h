/**
 * @file RcCurve.h
 * @brief FPV 摇杆曲线（Betaflight 风格，平台无关 host 可测）
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
 */
#pragma once
#include <cmath>
#include <algorithm>

#define RC_DEADBAND_NORM 0.02f   // 归一化死区（±10us @512us 半程）

// 归一化摇杆（PWM us → [-1,1]；988-2012 映射到 ±1）
static inline float rcNormFromUs(float us)
{
    return (us - 1500.0f) / 512.0f;
}

// Expo 输入曲线（Betaflight：rc·(expo·rc² + (1-expo))）
static inline float rcExpoCurve(float rc, float expo)
{
    if (expo <= 0.0f)
        return rc;
    return rc * (expo * rc * rc + (1.0f - expo));
}

/**
 * @brief 完整摇杆曲线：归一化杆量 → 目标角速率 (deg/s)
 * @param rc        归一化杆量 [-1,1]（已含死区处理）
 * @param rcRate    全局灵敏度（0-2.5）
 * @param expo      输入曲线（0-1）
 * @param super     边缘曲线（0-1，<1 防极点）
 * @param maxRate   角速率上限（限幅，如 450 deg/s）
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
