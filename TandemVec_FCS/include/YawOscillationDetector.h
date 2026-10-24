/**
 * @file    YawOscillationDetector.h
 * @module  固件/导航（偏航振荡检测，位置保持降级候选）
 * @brief   监测持续高偏航角速度，判断航向环是否进入极限环/振荡
 * @details 输入：偏航角速度（deg/s，机体系 z）与 dt；处理：|r| 超过
 *          rate_threshold_dps 连续 trigger_s 秒 → oscillating=true；
 *          |r| 低于 quiet_threshold_dps 连续 recover_s 秒 → 解除。
 *          纯类、零 Arduino/Eigen 依赖；**未接入默认导航路径**。
 * @theory  悬停/位置保持时航向指令≈0，持续高偏航角速度意味着航向环
 *          在振荡或航向估计翻转；此时位置环会把 NED 修正投到错误机体
 *          方向，形成位置 runaway。检测到后应降级定高/停止水平位置环。
 *          @ref VER-003 §3.14；AGENTS.md「seed7 航向根因」
 * @frame   机体系 z 轴角速度（FRD）
 * @units   deg/s、s
 * @note    ★ 2026-10-24：seed7 轨迹显示 yaw +42°→+172°→−82°、位置 132 m，
 *          而高度/NIS 正常；本模块只做检测，不替代磁新息门控。
 * @test    test_host/test_yaw_oscillation_detector.cpp
 * @see     HeadingAidingGate.h、navigation_task.cpp、VER-003
 */
#ifndef TANDEMVEC_YAW_OSCILLATION_DETECTOR_H
#define TANDEMVEC_YAW_OSCILLATION_DETECTOR_H

#include <cmath>

namespace bfs
{

struct YawOscillationParams
{
    float rate_threshold_dps = 35.0f;
    float quiet_threshold_dps = 10.0f;
    float trigger_s = 1.0f;
    float recover_s = 3.0f;
};

class YawOscillationDetector
{
public:
    void setParams(const YawOscillationParams &p) { params_ = p; }
    const YawOscillationParams &params() const { return params_; }

    void reset()
    {
        high_accum_s_ = 0.0f;
        quiet_accum_s_ = 0.0f;
        oscillating_ = false;
    }

    bool update(const float yaw_rate_dps, const float dt_s)
    {
        if (!std::isfinite(yaw_rate_dps) || !std::isfinite(dt_s) ||
            !(dt_s > 0.0f))
        {
            return oscillating_;
        }
        const float rate_abs = std::fabs(yaw_rate_dps);
        if (rate_abs > params_.rate_threshold_dps)
        {
            high_accum_s_ += dt_s;
            quiet_accum_s_ = 0.0f;
            if (high_accum_s_ >= params_.trigger_s)
            {
                oscillating_ = true;
            }
        }
        else
        {
            high_accum_s_ = 0.0f;
            if (rate_abs < params_.quiet_threshold_dps)
            {
                quiet_accum_s_ += dt_s;
                if (quiet_accum_s_ >= params_.recover_s)
                {
                    oscillating_ = false;
                    quiet_accum_s_ = 0.0f;
                }
            }
            else
            {
                quiet_accum_s_ = 0.0f;
            }
        }
        return oscillating_;
    }

    bool oscillating() const { return oscillating_; }
    float highAccumS() const { return high_accum_s_; }

private:
    YawOscillationParams params_{};
    float high_accum_s_ = 0.0f;
    float quiet_accum_s_ = 0.0f;
    bool oscillating_ = false;
};

} // namespace bfs

#endif // TANDEMVEC_YAW_OSCILLATION_DETECTOR_H
