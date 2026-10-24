/**
 * @file    LiquidRocketModel.h
 * @module  固件/推进（液体火箭变质量推进与喷管几何，通用层，2026-10-24）
 * @brief   提供 L1 液体火箭推进模型：点火/阀门/推力曲线/变质量/比冲流量，
 *          以及双轴喷管几何逆解；不接入飞行模式，不写硬件。
 * @details 输入：归一化油门、点火/解锁门控、dt；处理：点火锁存、阀门速率限幅、
 *          最小/最大推力限幅、推力速率限幅、m_dot = T/(g0·Isp)、推进剂耗尽；
 *          输出：推力、质量、推进剂余量、质量流率、阀门位置、火焰熄灭标记。
 *          喷管几何：roll/pitch 归一化力矩指令 → 双轴摆角（外部工程
 *          `VTVL_LiquidRocket_FC` 的 CCAEGT/几何 TVC 口径，未复制其制导算法）。
 *          纯函数/类、零 Arduino/Eigen 依赖，宿主可直接回归。
 * @theory  T = u·T_max；m_dot = T/(g0·Isp)；m = m_dry + m_prop；
 *          Δm = −m_dot·Δt；双轴摆角由归一化横向推力分量经 asin 几何分解
 *          @ref CTL-003 §3.3；PROP-001 §2（推进集总模型）；THY-003 §2
 * @frame   推力沿箭体轴；摆角为喷管相对箭体轴的两轴转角（deg）
 * @units   N、kg、s、kg/s、deg
 * @note    ★ 本模型是 L1 仿真/台架模型：T_max/T_min/Isp/干重来自外部液体火箭
 *          工程的 CCAEGT 参数示例，**不是本机标定值**，不能用于实机推力预测。
 *          阀门/点火/RCS 的真实安全联锁、推进剂密度/晃荡、泵压/室压动态均未建模。
 *          该模块目前只用于模型/分配层验证，未接入 runGNCExecutive。
 * @redline 点火/解锁门控、推进剂耗尽后推力必须为 0、质量不得为负
 * @test    test_host/test_liquid_rocket_model.cpp
 * @see     include/VehicleProfile.h、include/ActuatorOutputMap.h、CTL-003 §3.3
 */

#ifndef TANDEMVEC_LIQUID_ROCKET_MODEL_H
#define TANDEMVEC_LIQUID_ROCKET_MODEL_H

#include <cmath>
#include <cstdint>

namespace bfs
{

struct LiquidRocketParams
{
    float dry_mass_kg = 13.5f;          // 结构干重
    float initial_propellant_kg = 3.0f; // 初始推进剂
    float max_thrust_n = 200.0f;        // 最大推力
    float min_thrust_n = 80.0f;         // 最小稳定工作推力
    float isp_s = 200.0f;               // 比冲
    float g0_mps2 = 9.80665f;           // 标准重力
    float thrust_rate_limit_nps = 1800.0f;
    float valve_rate_limit_per_s = 10.0f;
    float min_throttle = 0.0f;          // 低于该油门视为关机
};

struct LiquidRocketState
{
    float propellant_kg = 0.0f;
    float mass_kg = 0.0f;
    float valve_position = 0.0f; // 实际阀门/油门位置 0–1
    float thrust_n = 0.0f;
    bool ignited = false;
    bool flameout = false;
    bool valid = false;
};

struct LiquidRocketCommand
{
    float throttle = 0.0f;      // 归一化 0–1
    bool ignition_enabled = false;
    bool unlocked = false;
    float dt_s = 0.005f;
};

struct LiquidRocketOutput
{
    bool valid = false;
    float thrust_n = 0.0f;
    float mass_kg = 0.0f;
    float propellant_kg = 0.0f;
    float mass_flow_kgps = 0.0f;
    float valve_position = 0.0f;
    bool ignited = false;
    bool flameout = false;
    bool saturated = false;
};

struct NozzleGimbalOutput
{
    bool valid = false;
    float x_deg = 0.0f;
    float y_deg = 0.0f;
    bool saturated = false;
};

inline float liquidRocketClamp(const float v, const float lo, const float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

inline bool liquidRocketParamsValid(const LiquidRocketParams &p)
{
    return std::isfinite(p.dry_mass_kg) && p.dry_mass_kg > 0.0f &&
           std::isfinite(p.initial_propellant_kg) &&
           p.initial_propellant_kg >= 0.0f &&
           std::isfinite(p.max_thrust_n) && p.max_thrust_n > 0.0f &&
           std::isfinite(p.min_thrust_n) && p.min_thrust_n >= 0.0f &&
           p.min_thrust_n <= p.max_thrust_n && std::isfinite(p.isp_s) &&
           p.isp_s > 0.0f && std::isfinite(p.g0_mps2) && p.g0_mps2 > 0.0f &&
           std::isfinite(p.thrust_rate_limit_nps) &&
           p.thrust_rate_limit_nps > 0.0f &&
           std::isfinite(p.valve_rate_limit_per_s) &&
           p.valve_rate_limit_per_s > 0.0f &&
           std::isfinite(p.min_throttle) && p.min_throttle >= 0.0f &&
           p.min_throttle < 1.0f;
}

class LiquidRocketPropulsion
{
public:
    bool init(const LiquidRocketParams &params)
    {
        params_ = params;
        if (!liquidRocketParamsValid(params_))
        {
            state_ = LiquidRocketState{};
            return false;
        }
        reset();
        return true;
    }

    void reset()
    {
        state_.propellant_kg = params_.initial_propellant_kg;
        state_.mass_kg = params_.dry_mass_kg + state_.propellant_kg;
        state_.valve_position = 0.0f;
        state_.thrust_n = 0.0f;
        state_.ignited = false;
        state_.flameout = false;
        state_.valid = liquidRocketParamsValid(params_);
    }

    LiquidRocketOutput update(const LiquidRocketCommand &cmd)
    {
        LiquidRocketOutput out;
        if (!state_.valid || !std::isfinite(cmd.throttle) ||
            !std::isfinite(cmd.dt_s) || !(cmd.dt_s > 0.0f) ||
            cmd.throttle < 0.0f || cmd.throttle > 1.0f)
        {
            return out;
        }

        // 点火锁存：必须解锁 + 点火使能 + 有推进剂；一旦锁存，后续由油门控制。
        if (cmd.unlocked && cmd.ignition_enabled &&
            state_.propellant_kg > 1.0e-6f)
        {
            state_.ignited = true;
        }
        if (!cmd.unlocked)
        {
            state_.ignited = false;
        }

        const bool can_fire =
            state_.ignited && !state_.flameout &&
            state_.propellant_kg > 1.0e-6f &&
            cmd.throttle > params_.min_throttle;
        float valve_target = can_fire ? cmd.throttle : 0.0f;
        out.saturated = can_fire && (cmd.throttle >= 1.0f);

        // 阀门/油门速率限幅（真实阀门不会瞬开/瞬关）。
        const float valve_step =
            params_.valve_rate_limit_per_s * cmd.dt_s;
        state_.valve_position += liquidRocketClamp(
            valve_target - state_.valve_position, -valve_step, valve_step);
        state_.valve_position =
            liquidRocketClamp(state_.valve_position, 0.0f, 1.0f);

        // 阀门→推力：最小稳定工作推力（泵压/燃烧室稳定下限）。
        float thrust_target = state_.valve_position * params_.max_thrust_n;
        if (state_.valve_position > 1.0e-4f &&
            thrust_target < params_.min_thrust_n)
        {
            thrust_target = params_.min_thrust_n;
        }
        if (!can_fire && state_.valve_position <= 1.0e-4f)
        {
            thrust_target = 0.0f;
        }

        // 推力速率限幅；关机时允许按同一速率下降，避免阶跃。
        const float thrust_step =
            params_.thrust_rate_limit_nps * cmd.dt_s;
        state_.thrust_n += liquidRocketClamp(
            thrust_target - state_.thrust_n, -thrust_step, thrust_step);
        state_.thrust_n = liquidRocketClamp(state_.thrust_n, 0.0f,
                                            params_.max_thrust_n);

        // 推进剂消耗：T = m_dot·g0·Isp → m_dot = T/(g0·Isp)。
        float mass_flow = 0.0f;
        if (state_.thrust_n > 0.0f && state_.propellant_kg > 0.0f)
        {
            mass_flow = state_.thrust_n / (params_.g0_mps2 * params_.isp_s);
            float consumed = mass_flow * cmd.dt_s;
            if (consumed >= state_.propellant_kg)
            {
                consumed = state_.propellant_kg;
                mass_flow = (cmd.dt_s > 0.0f) ? consumed / cmd.dt_s : 0.0f;
                state_.propellant_kg = 0.0f;
                state_.thrust_n = 0.0f;
                state_.valve_position = 0.0f;
                state_.flameout = true;
            }
            else
            {
                state_.propellant_kg -= consumed;
            }
        }
        if (state_.propellant_kg <= 1.0e-6f && state_.ignited &&
            !state_.flameout)
        {
            state_.propellant_kg = 0.0f;
            state_.thrust_n = 0.0f;
            state_.valve_position = 0.0f;
            state_.flameout = true;
        }

        state_.mass_kg = params_.dry_mass_kg + state_.propellant_kg;
        out.valid = std::isfinite(state_.mass_kg) &&
                    std::isfinite(state_.thrust_n) &&
                    std::isfinite(state_.propellant_kg) &&
                    state_.mass_kg > 0.0f;
        if (!out.valid)
        {
            state_.thrust_n = 0.0f;
            state_.valve_position = 0.0f;
        }
        out.thrust_n = state_.thrust_n;
        out.mass_kg = state_.mass_kg;
        out.propellant_kg = state_.propellant_kg;
        out.mass_flow_kgps = mass_flow;
        out.valve_position = state_.valve_position;
        out.ignited = state_.ignited;
        out.flameout = state_.flameout;
        return out;
    }

    const LiquidRocketState &state() const { return state_; }
    const LiquidRocketParams &params() const { return params_; }

private:
    LiquidRocketParams params_{};
    LiquidRocketState state_{};
};

/**
 * @brief 双轴喷管几何：归一化 roll/pitch 横向推力 → 两轴摆角
 * @param[in] roll_cmd 归一化滚转横向推力分量 [-1,1]
 * @param[in] pitch_cmd 归一化俯仰横向推力分量 [-1,1]
 * @param[in] max_angle_deg 单轴机械限幅（deg）
 * @return 两轴摆角；非法输入 valid=false、输出 0
 * @formula gx=(roll−pitch)/√2，gy=(roll+pitch)/√2；θx=asin(gx)，
 *          θy=asin(gy/cosθx)；gx/gy 先钳 [-1,1]，θ 再按机械限幅
 * @ref CTL-003 §3.3；外部 VTVL_LiquidRocket_FC 的几何 TVC 口径（仅参考）
 * @note 该分解假设两轴摆角小、横向推力可线性叠加；大角度/耦合需按真实喷管
 *       运动学标定。max_angle_deg=0 时返回无效。
 * @test test_host/test_liquid_rocket_model.cpp
 */
inline NozzleGimbalOutput mapNozzleGimbal(const float roll_cmd,
                                          const float pitch_cmd,
                                          const float max_angle_deg)
{
    NozzleGimbalOutput out;
    if (!std::isfinite(roll_cmd) || !std::isfinite(pitch_cmd) ||
        !std::isfinite(max_angle_deg) || !(max_angle_deg > 0.0f))
    {
        return out;
    }
    constexpr float kSqrt2Over2 = 0.7071067811865476f;
    float gx = liquidRocketClamp((roll_cmd - pitch_cmd) * kSqrt2Over2,
                                 -1.0f, 1.0f);
    float gy = liquidRocketClamp((roll_cmd + pitch_cmd) * kSqrt2Over2,
                                 -1.0f, 1.0f);
    const float gx_raw = (roll_cmd - pitch_cmd) * kSqrt2Over2;
    const float gy_raw = (roll_cmd + pitch_cmd) * kSqrt2Over2;
    out.saturated = (gx != gx_raw) || (gy != gy_raw);

    const float x_rad = std::asin(gx);
    const float cos_x = std::fmax(std::fabs(std::cos(x_rad)), 1.0e-6f);
    const float y_rad = std::asin(liquidRocketClamp(gy / cos_x, -1.0f, 1.0f));

    float x_deg = x_rad * 57.29577951308232f;
    float y_deg = y_rad * 57.29577951308232f;
    if (std::fabs(x_deg) > max_angle_deg)
    {
        out.saturated = true;
        x_deg = (x_deg > 0.0f) ? max_angle_deg : -max_angle_deg;
    }
    if (std::fabs(y_deg) > max_angle_deg)
    {
        out.saturated = true;
        y_deg = (y_deg > 0.0f) ? max_angle_deg : -max_angle_deg;
    }
    out.x_deg = x_deg;
    out.y_deg = y_deg;
    out.valid = std::isfinite(out.x_deg) && std::isfinite(out.y_deg);
    return out;
}

} // namespace bfs

#endif // TANDEMVEC_LIQUID_ROCKET_MODEL_H
