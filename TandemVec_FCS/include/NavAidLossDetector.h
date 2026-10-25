/**
 * @file    NavAidLossDetector.h
 * @module  固件/导航（GNSS+光流组合失效检测）
 * @brief   检测“GNSS 陈旧且光流不可用”的持续组合失效，输出降级/受控下降请求
 * @details 输入：GNSS 是否陈旧、光流是否无效、导航帧 dt；处理：连续丢失计时、
 *          连续恢复计时、滞回；输出：degraded（停位置保持）与 descent_requested
 *          （受控下降），以及 recovered 一次性事件。
 * @theory  组合失效检测采用“连续时间 + 滞回”避免单帧毛刺；GNSS 与光流是
 *          两条独立水平速度/位置参照，单独光流丢失由 GNSS 兜底（SIL 50 seeds
 *          0/50），只有两者同时失效才进入失效保护。
 * @frame  N/A（纯逻辑，不解释坐标系）
 * @units  dt_s / 门限单位 s
 * @note   本层只做判定，不直接写飞控模式；上层负责降级与受控下降动作。
 * @see     src/navigation_task.cpp、docs/05-控制与分配/CTL-004
 */
#ifndef TANDEMVEC_NAV_AID_LOSS_DETECTOR_H
#define TANDEMVEC_NAV_AID_LOSS_DETECTOR_H

#include <cmath>

namespace bfs
{

class NavAidLossDetector
{
public:
    struct Output
    {
        bool degraded = false;
        bool descent_requested = false;
        bool recovered = false;
    };

    Output update(const bool gnss_stale, const bool flow_invalid,
                  const float dt_s)
    {
        Output out;
        if (!std::isfinite(dt_s) || dt_s <= 0.0f)
        {
            out.degraded = degraded_;
            out.descent_requested = descent_;
            return out;
        }

        const bool lost = gnss_stale && flow_invalid;
        if (lost)
        {
            lost_s_ += dt_s;
            recover_s_ = 0.0f;
            if (lost_s_ >= kDegradeS)
            {
                degraded_ = true;
            }
            if (lost_s_ >= kDescentS)
            {
                descent_ = true;
            }
        }
        else
        {
            lost_s_ = 0.0f;
            if (degraded_ || descent_)
            {
                recover_s_ += dt_s;
                if (recover_s_ >= kRecoverS)
                {
                    degraded_ = false;
                    descent_ = false;
                    recover_s_ = 0.0f;
                    out.recovered = true;
                }
            }
        }

        out.degraded = degraded_;
        out.descent_requested = descent_;
        return out;
    }

    bool degraded() const { return degraded_; }
    bool descentRequested() const { return descent_; }

private:
    static constexpr float kDegradeS = 0.5f;
    static constexpr float kDescentS = 2.0f;
    static constexpr float kRecoverS = 1.0f;
    float lost_s_ = 0.0f;
    float recover_s_ = 0.0f;
    bool degraded_ = false;
    bool descent_ = false;
};

} // namespace bfs

#endif // TANDEMVEC_NAV_AID_LOSS_DETECTOR_H
