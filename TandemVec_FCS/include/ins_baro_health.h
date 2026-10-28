/**
 * @file    ins_baro_health.h
 * @module  固件/导航（气压健康监测）
 * @brief   用 GNSS 高度作参照检测气压慢漂/台阶/尖峰/卡死，输出可疑标志与
 *          漂移速率；只做诊断，不改变气压融合或垂直仲裁
 * @details 输入：气压相对高度、GNSS 绝对高（有效时）、EKF 垂直速度、dt；
 *          处理：去初始安装偏差后的残差低通 + 残差斜率低通；斜率超门限
 *          连续确认判 Drift，残差超限判 Drift，单帧变化率超限判 Spike，
 *          长时间不变且垂速非零判 Stuck；Drift 采用迟滞锁存，只有速率/残差
 *          回到释放门限并连续保持才撤销（避免噪声下占空比抖动）；
 *          输出：`BaroHealthStatus`（state/flags/drift_rate/residual）。
 *          不变量：GNSS 无效时不做漂移判定（无参照），只保留尖峰/卡死；
 *          本模块不写 EKF/控制。
 * @theory  双源垂直一致性监测：GNSS 高度短期可信时，气压慢漂表现为
 *          残差斜率；用低通抑制 GNSS 噪声，再用持续时间确认避免单帧误报。
 *          @ref REF-121 §7.3.1 p.255–259（系统级故障检测与隔离）
 * @frame   N/A（标量高度/垂速）
 * @units   m、m/s、s
 * @note    0.5 m/s 漂移在 8 s 内可识别（2 m 残差门限抑制 GNSS 有色噪声假阳性）；
 *          0.02 m/s 级慢漂不在本模块承诺范围（与 GNSS 慢漂不可检测同源）。
 *          默认只观测；仲裁/降级需上层决定。
 * @test    test_host/test_ins_baro_health.cpp
 * @see     src/navigation_task.cpp、include/ins_gnss_consistency.h
 */

#ifndef TANDEMVEC_INS_BARO_HEALTH_H
#define TANDEMVEC_INS_BARO_HEALTH_H

#include <cmath>
#include <cstdint>

namespace bfs
{

enum class BaroHealthState : uint8_t
{
    Ok = 0,
    Suspect = 1,
};

enum class BaroHealthFlag : uint16_t
{
    None = 0,
    Drift = 1U << 0,
    Spike = 1U << 1,
    Stuck = 1U << 2,
};

inline uint16_t baroHealthFlagBit(const BaroHealthFlag flag)
{
    return static_cast<uint16_t>(flag);
}

struct BaroHealthConfig
{
    float drift_rate_threshold_mps = 0.30f;
    // 速率判据必须同时有至少 2 m 的残差，否则 GNSS 高度有色噪声的短时
    // 斜率会单独触发 Drift（SIL 标称 seed4 实测 0.35 m/s 假阳性）。
    float drift_min_residual_m = 2.0f;
    float drift_confirm_s = 2.0f;
    // 已确认漂移的释放门限：速率回落到 clear_rate 以下、残差回到 clear_m
    // 以内并连续保持 clear_s，才撤销 Drift。否则残差斜率在噪声下会
    // 反复穿越 0.3 m/s，导致门控占空比抖动（SIL 漂移臂实测 583–989/1250）。
    float drift_clear_rate_mps = 0.15f;
    float drift_clear_residual_m = 20.0f;
    float drift_clear_s = 3.0f;
    float residual_limit_m = 40.0f;
    float spike_rate_mps = 20.0f;
    float stuck_epsilon_m = 0.01f;
    float stuck_confirm_s = 3.0f;
    float stuck_min_abs_vz_mps = 1.0f;
    float residual_lpf_tau_s = 1.0f;
    float rate_lpf_tau_s = 0.5f;
};

struct BaroHealthStatus
{
    BaroHealthState state = BaroHealthState::Ok;
    uint16_t flags = 0;
    float drift_rate_mps = 0.0f;
    float residual_m = 0.0f;
    float drift_s = 0.0f;
    float stuck_s = 0.0f;
};

class BaroHealthMonitor
{
  public:
    BaroHealthMonitor() = default;
    explicit BaroHealthMonitor(const BaroHealthConfig &config)
        : config_(config)
    {
    }

    void reset()
    {
        status_ = BaroHealthStatus{};
        have_ref_ = false;
        have_last_ = false;
        r0_r_lp_ = 0.0f;
        r_lp_prev_ = 0.0f;
        rate_lp_ = 0.0f;
        last_baro_ = 0.0f;
        drift_latched_ = false;
        drift_clear_elapsed_s_ = 0.0f;
    }

    BaroHealthStatus update(const float baro_alt_m, const float gnss_alt_m,
                            const float ins_vz_mps, const bool gnss_valid,
                            const float dt_s)
    {
        if (!(dt_s > 0.0f) || !std::isfinite(dt_s) ||
            !std::isfinite(baro_alt_m) || !std::isfinite(ins_vz_mps))
        {
            return status_;
        }
        const float dt = std::fmin(dt_s, 0.05f);
        status_.flags = 0;

        if (gnss_valid && std::isfinite(gnss_alt_m))
        {
            if (!have_ref_)
            {
                r0_r_lp_ = baro_alt_m - gnss_alt_m;
                r_lp_prev_ = 0.0f;
                have_ref_ = true;
            }
            const float residual = (baro_alt_m - gnss_alt_m) - r0_r_lp_;
            const float a_r = dt / (config_.residual_lpf_tau_s + dt);
            const float r_lp_old = r_lp_prev_;
            r_lp_prev_ = r_lp_prev_ + a_r * (residual - r_lp_prev_);
            status_.residual_m = r_lp_prev_;

            // 低通残差的斜率（不是单帧残差差，避免 GNSS 噪声放大）。
            const float raw_rate = (r_lp_prev_ - r_lp_old) / dt;
            const float a_d = dt / (config_.rate_lpf_tau_s + dt);
            rate_lp_ = rate_lp_ + a_d * (raw_rate - rate_lp_);
            status_.drift_rate_mps = rate_lp_;

            if ((std::fabs(rate_lp_) > config_.drift_rate_threshold_mps &&
                 std::fabs(residual) > config_.drift_min_residual_m) ||
                std::fabs(residual) > config_.residual_limit_m)
            {
                status_.drift_s += dt;
            }
            else
            {
                status_.drift_s = 0.0f;
            }
            if (status_.drift_s >= config_.drift_confirm_s)
            {
                drift_latched_ = true;
                drift_clear_elapsed_s_ = 0.0f;
            }
            if (drift_latched_)
            {
                const bool quiet =
                    std::fabs(rate_lp_) < config_.drift_clear_rate_mps &&
                    std::fabs(residual) < config_.drift_clear_residual_m;
                if (quiet)
                {
                    drift_clear_elapsed_s_ += dt;
                    if (drift_clear_elapsed_s_ >= config_.drift_clear_s)
                    {
                        drift_latched_ = false;
                        drift_clear_elapsed_s_ = 0.0f;
                    }
                }
                else
                {
                    drift_clear_elapsed_s_ = 0.0f;
                }
                if (drift_latched_)
                {
                    status_.flags |= baroHealthFlagBit(BaroHealthFlag::Drift);
                }
            }
        }
        else
        {
            // 无 GNSS 参照：漂移判定暂停，避免把正常爬升/下降当漂移。
            status_.drift_s = 0.0f;
            status_.drift_rate_mps = 0.0f;
        }

        if (have_last_)
        {
            const float step = std::fabs(baro_alt_m - last_baro_);
            if (step / dt > config_.spike_rate_mps)
            {
                status_.flags |= baroHealthFlagBit(BaroHealthFlag::Spike);
            }
            if (step < config_.stuck_epsilon_m &&
                std::fabs(ins_vz_mps) > config_.stuck_min_abs_vz_mps)
            {
                status_.stuck_s += dt;
            }
            else
            {
                status_.stuck_s = 0.0f;
            }
            if (status_.stuck_s >= config_.stuck_confirm_s)
            {
                status_.flags |= baroHealthFlagBit(BaroHealthFlag::Stuck);
            }
        }
        last_baro_ = baro_alt_m;
        have_last_ = true;

        status_.state = (status_.flags != 0U) ? BaroHealthState::Suspect
                                              : BaroHealthState::Ok;
        return status_;
    }

    const BaroHealthStatus &status() const { return status_; }
    bool suspect() const { return status_.state == BaroHealthState::Suspect; }
    const BaroHealthConfig &config() const { return config_; }
    void set_config(const BaroHealthConfig &config) { config_ = config; }

  private:
    BaroHealthConfig config_{};
    BaroHealthStatus status_{};
    bool have_ref_ = false;
    bool have_last_ = false;
    float r0_r_lp_ = 0.0f;
    float r_lp_prev_ = 0.0f;
    float rate_lp_ = 0.0f;
    float last_baro_ = 0.0f;
    bool drift_latched_ = false;
    float drift_clear_elapsed_s_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_INS_BARO_HEALTH_H
