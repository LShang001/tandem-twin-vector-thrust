/**
 * @file    HeadingAidingGate.h
 * @module  固件/导航（绝对航向辅助可用性与位置保持降级门控，2026-10-24）
 * @brief   监视磁矢量/GNSS 双矢量航向的最近成功融合时间，决定 AUTO_POSITION
 *          是否仍具备可观测的绝对航向；失效时要求上层降级为定高，避免
 *          位置保持追逐不可观航向估计导致姿态-位置耦合发散。
 * @details 输入：Mag/YawGsf 距上次成功融合的时间、水平速度、dt；
 *          处理：新鲜度判定（**仅 Mag ≤2 s 可作为位置保持/heading hold 的
 *          绝对航向源**；YawGsf 是航迹−侧滑的伪航向，只在有速度时成立，
 *          仅作诊断，不解除门控）+ 降级锁存（失效立即降级；恢复后需连续
 *          3 s 磁新鲜才允许重新位置保持）；无磁模式 20 s 保持窗口
 *          **只在离地后计时**（上电/地面等待不消耗，落地复位）；
 *          输出：source_fresh / position_hold_degraded / best_age_s。
 *          纯函数、零 Arduino/Eigen 依赖，宿主可直接回归。
 * @theory  悬停时航向不可观（无绝对航向源），EKF 航向误差会 1:1 进入姿态/
 *          位置保持回路；位置保持必须以可观测的绝对航向为前提，否则应
 *          降级为不含水平位置闭环的定高/手动姿态模式。
 *          @ref REF-120 §9.8.14 p.320–322（航向弱可观性）；CTL-001 §8.4；
 *          AGENTS.md「EKF 进控制闭环 10/10 FAIL 根因」
 * @frame   N/A（只处理辅助源时间戳与速度标量）
 * @units   时间 s、速度 m/s
 * @note    ★ YawGsf 是“GNSS 航迹 − 光流侧滑”的航向观测，只在有速度时成立；
 *          悬停/低速不能替代磁航向。更关键的是：有速度时它仍可能被风/侧滑
 *          偏置污染，且一旦速度降回悬停就失效——用它会重新打开不可观位置
 *          保持。因此本门控只认 Mag 为绝对航向源；YawGsf 年龄仅上报，
 *          不作为解除降级的依据。Mag 年龄来自
 *          `aid_tracker.status(AidSource::Mag).age_s`，只有 EKF **成功融合**
 *          才清零，门控拒绝/无样本都会让年龄增长。
 * @redline 航向失效时不得继续 AUTO_POSITION/GUIDED 水平位置闭环
 * @test    test_host/test_heading_aiding_gate.cpp
 * @see     src/navigation_task.cpp、src/flight_control.cpp、AGENTS.md
 */

#ifndef TANDEMVEC_HEADING_AIDING_GATE_H
#define TANDEMVEC_HEADING_AIDING_GATE_H

#include <cmath>
#include <cfloat>
#include <cstdint>

#include "YawOscillationDetector.h"

namespace bfs
{

struct HeadingAidingInput
{
    float mag_age_s = FLT_MAX;         // 距 Mag 最近一次成功融合的时间
    float yaw_gsf_age_s = FLT_MAX;     // 距 YawGsf 最近一次成功融合的时间
    float horizontal_speed_mps = 0.0f; // EKF/GNSS 水平速度（YawGsf 有效性）
    float yaw_rate_dps = 0.0f;         // 机体系 z 角速度（偏航振荡检测）
    bool monitor_yaw_oscillation = false; // 默认关闭，保持旧行为
    float yaw_sigma_deg = 0.0f;        // EKF 航向 1σ 不确定度
    bool monitor_yaw_sigma = false;    // 默认关闭，保持旧行为
    // 无磁/磁仅初始对准模式：航向由运动 EKF 观测，悬停靠陀螺短时维持。
    uint8_t heading_mode = 0;          // 0=MAG_ONLY / 1=MAG_INIT / 2=NO_MAG
    bool gnss_motion_valid = false;    // GNSS 速度足以观测航向（>2 m/s）
    bool motion_excitation_valid = false; // 速度 + 倾斜同时存在，航向可观测
    float yaw_drift_dps = 0.0f;        // EKF 航向变化率 − 陀螺 z（无磁一致性）
    bool airborne = false;             // 已离地（解锁且油门上量），无磁窗口从离地起算
    bool calibrated = true;            // 静止校准已完成（无磁模式的保持前置条件）
    float dt_s = 0.005f;
};

struct HeadingAidingOutput
{
    bool source_fresh = false;             // 磁航向是否新鲜（唯一绝对源）
    bool heading_hold_allowed = false;     // 是否允许 ratchet heading hold
    bool yaw_gsf_fresh = false;            // YawGsf 诊断标志，不解除门控
    bool yaw_oscillating = false;          // 偏航角速度持续超限（航向环振荡）
    bool yaw_sigma_high = false;           // EKF 航向不确定度持续超限
    bool position_hold_degraded = true;    // true=禁止水平位置保持/heading hold
    float best_age_s = FLT_MAX;            // 两源中较新的年龄，用于遥测
    float valid_for_s = 0.0f;              // 恢复计时（饱和到重新使能门限）
};

class HeadingAidingGate
{
public:
    static constexpr float kMagMaxAgeS = 2.0f;
    static constexpr float kYawGsfMaxAgeS = 2.0f;
    static constexpr float kYawGsfMinSpeedMps = 2.0f;
    static constexpr float kReenableHoldS = 3.0f;
    static constexpr float kMotionCoastS = 3.0f;
    static constexpr float kNoMagHoldMaxS = 20.0f;

    void reset()
    {
        degraded_ = true;
        valid_accum_s_ = 0.0f;
        drift_high_s_ = 0.0f;
        drift_quiet_s_ = 0.0f;
        drift_degraded_ = false;
        motion_coast_s_ = 0.0f;
        no_mag_hold_s_ = 0.0f;
        yaw_osc_.reset();
        sigma_high_accum_s_ = 0.0f;
        sigma_quiet_accum_s_ = 0.0f;
        sigma_degraded_ = false;
    }

    HeadingAidingOutput update(const HeadingAidingInput &in)
    {
        HeadingAidingOutput out;
        const bool dt_ok = std::isfinite(in.dt_s) && in.dt_s > 0.0f;
        const float dt_s = dt_ok ? in.dt_s : 0.0f;

        const bool mag_fresh =
            std::isfinite(in.mag_age_s) && in.mag_age_s >= 0.0f &&
            in.mag_age_s <= kMagMaxAgeS;
        // 无磁/磁仅初始对准：悬停没有绝对航向，靠陀螺短时维持；用 EKF
    // 航向 1σ 限制保持时长，并用“EKF 航向变化率 − 陀螺 z”识别纯平移时
    // 估计器自漂（风推、无倾斜，航向不可观）。两者都满足才允许保持。
    const bool no_mag_mode = in.heading_mode >= 1U;
    const bool sigma_valid =
        std::isfinite(in.yaw_sigma_deg) && in.yaw_sigma_deg >= 0.0f;
    const bool sigma_ok =
        sigma_valid && in.yaw_sigma_deg <= 10.0f;
    if (no_mag_mode && std::isfinite(in.yaw_drift_dps))
    {
        const float drift_abs = std::fabs(in.yaw_drift_dps);
        if (drift_abs > 5.0f)
        {
            drift_high_s_ += dt_s;
            drift_quiet_s_ = 0.0f;
            if (drift_high_s_ >= 0.5f)
            {
                drift_degraded_ = true;
            }
        }
        else if (drift_degraded_)
        {
            if (drift_abs < 2.5f)
            {
                drift_quiet_s_ += dt_s;
                if (drift_quiet_s_ >= 3.0f)
                {
                    drift_degraded_ = false;
                    drift_quiet_s_ = 0.0f;
                }
            }
            else
            {
                drift_quiet_s_ = 0.0f;
            }
        }
        else
        {
            drift_high_s_ = 0.0f;
        }
    }
    else if (!no_mag_mode)
    {
        drift_high_s_ = 0.0f;
        drift_quiet_s_ = 0.0f;
        drift_degraded_ = false;
    }
    const bool motion_ok =
        no_mag_mode && in.motion_excitation_valid && sigma_ok &&
        !drift_degraded_;
    if (motion_ok)
    {
        motion_coast_s_ = kMotionCoastS;
    }
    else if (dt_s > 0.0f && motion_coast_s_ > 0.0f)
    {
        motion_coast_s_ = std::fmax(0.0f, motion_coast_s_ - dt_s);
    }
    const bool motion_coast_ok =
        no_mag_mode && motion_coast_s_ > 0.0f && sigma_ok &&
        !drift_degraded_;
    const bool no_mag_ok = motion_ok || motion_coast_ok;
    // YawGsf 只做诊断，不解除门控：详见头文件 @note。
    const bool yaw_gsf_fresh =
        std::isfinite(in.yaw_gsf_age_s) && in.yaw_gsf_age_s >= 0.0f &&
        in.yaw_gsf_age_s <= kYawGsfMaxAgeS &&
        std::isfinite(in.horizontal_speed_mps) &&
        in.horizontal_speed_mps >= kYawGsfMinSpeedMps;
    out.yaw_gsf_fresh = yaw_gsf_fresh;
    out.source_fresh = mag_fresh || no_mag_ok;
    // 无磁悬停不允许 heading hold：只允许位置保持需要运动激励。
    out.heading_hold_allowed = mag_fresh || motion_ok;

        out.best_age_s = FLT_MAX;
        if (std::isfinite(in.mag_age_s) && in.mag_age_s >= 0.0f)
        {
            out.best_age_s = in.mag_age_s;
        }
        if (std::isfinite(in.yaw_gsf_age_s) && in.yaw_gsf_age_s >= 0.0f)
        {
            out.best_age_s = std::fmin(out.best_age_s, in.yaw_gsf_age_s);
        }

        // 无磁/磁仅初始对准：绝对航向不可观。SIL 证明悬停/前飞后切定点都会
        // 因航向估计漂移而发散（229–239 m），故禁用 AUTO_POSITION 与 heading
        // hold，只保留定高/手动姿态；位置保持必须等绝对航向源可用。
        if (in.heading_mode >= 1U)
        {
            // 无磁/磁仅初始：短时允许位置保持与 heading hold。陀螺零偏在
            // 数十秒内有限；时间窗结束或航向 1σ 过大时必须降级定高。
            // 窗口只在离地后计时：上电/地面等待不消耗保持时间，落地复位。
            if (in.airborne)
            {
                no_mag_hold_s_ += dt_s;
            }
            else
            {
                no_mag_hold_s_ = 0.0f;
            }
            const bool time_ok = no_mag_hold_s_ <= kNoMagHoldMaxS;
            const bool sigma_ok20 =
                std::isfinite(in.yaw_sigma_deg) && in.yaw_sigma_deg <= 20.0f;
            // 静止校准（陀螺零偏/水平）未完成前不得进入位置保持：否则
            // 零偏/姿态初值误差会直接进入位置环（用户要求先静后动）。
            const bool hold_ok = in.calibrated && time_ok && sigma_ok20;
            out.source_fresh = hold_ok;
            out.heading_hold_allowed = hold_ok;
            out.position_hold_degraded = !hold_ok;
            out.valid_for_s = kNoMagHoldMaxS - no_mag_hold_s_;
            return out;
        }

        if (!out.source_fresh)
        {
            // 失效立即降级：不允许“先飞一会儿再看”。
            degraded_ = true;
            valid_accum_s_ = 0.0f;
        }
        else if (degraded_)
        {
            valid_accum_s_ += dt_s;
            if (valid_accum_s_ >= kReenableHoldS)
            {
                degraded_ = false;
            }
        }
        else
        {
            valid_accum_s_ = kReenableHoldS;
        }

        if (in.monitor_yaw_oscillation)
        {
            out.yaw_oscillating = yaw_osc_.update(in.yaw_rate_dps, dt_s);
        }
        else
        {
            yaw_osc_.reset();
            out.yaw_oscillating = false;
        }
        if (in.monitor_yaw_sigma && std::isfinite(in.yaw_sigma_deg) &&
            in.yaw_sigma_deg >= 0.0f && dt_s > 0.0f)
        {
            if (in.yaw_sigma_deg > 10.0f)
            {
                sigma_high_accum_s_ += dt_s;
                sigma_quiet_accum_s_ = 0.0f;
                if (sigma_high_accum_s_ >= 0.5f)
                {
                    sigma_degraded_ = true;
                }
            }
            else
            {
                sigma_high_accum_s_ = 0.0f;
                if (in.yaw_sigma_deg < 6.0f)
                {
                    sigma_quiet_accum_s_ += dt_s;
                    if (sigma_quiet_accum_s_ >= 3.0f)
                    {
                        sigma_degraded_ = false;
                        sigma_quiet_accum_s_ = 0.0f;
                    }
                }
                else
                {
                    sigma_quiet_accum_s_ = 0.0f;
                }
            }
            out.yaw_sigma_high = sigma_degraded_;
        }
        else
        {
            sigma_high_accum_s_ = 0.0f;
            sigma_quiet_accum_s_ = 0.0f;
            sigma_degraded_ = false;
            out.yaw_sigma_high = false;
        }
        out.position_hold_degraded =
            degraded_ || out.yaw_oscillating || out.yaw_sigma_high;
        out.valid_for_s = std::fmin(valid_accum_s_, kReenableHoldS);
        return out;
    }

    bool degraded() const { return degraded_; }

private:
    bool degraded_ = true;
    float valid_accum_s_ = 0.0f;
    float drift_high_s_ = 0.0f;
    float drift_quiet_s_ = 0.0f;
    bool drift_degraded_ = false;
    float motion_coast_s_ = 0.0f;
    float no_mag_hold_s_ = 0.0f;
    float sigma_high_accum_s_ = 0.0f;
    float sigma_quiet_accum_s_ = 0.0f;
    bool sigma_degraded_ = false;
    YawOscillationDetector yaw_osc_;
};

} // namespace bfs

#endif // TANDEMVEC_HEADING_AIDING_GATE_H
