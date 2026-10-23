/**
 * @file    ins_gnss_consistency.h
 * @module  固件/导航（GNSS 慢漂移/欺骗一致性监测）
 * @brief   用 IMU 速度增量、位置自洽、光流/气压交叉检查 GNSS 慢变异常，
 *          输出 Ok/Warning/Suspect；本层不写 EKF，只给导航层拒绝依据
 * @details 输入：每个 GNSS epoch 的 dt、GNSS 速度/位置、该区间 IMU 速度增量、
 *          光流/气压交叉量、reset 标志；处理：1 s 窗口累加残差，检查加速度
 *          一致性、位置自洽、光流水平速度差、气压高度漂移；输出：状态、
 *          最近窗口残差、Suspect 锁存计时与计数。
 *          不变量：Suspect 锁存 5 s；reset（失联/重锚）清空窗口；本层
 *          不直接改 AidSource/EKF，由 navigation_task 决定拒绝。
 * @theory  系统级 FDI：用独立于 GNSS 的 IMU/光流/气压信息做 χ²/门限检验，
 *          检测与隔离分离 @ref REF-121 §7.3.1 p.255–259；短时比力积分
 *          @ref REF-120 §7.4 (7.4.6) p.189
 * @frame   NED；光流由调用方转到 NED 后传入
 * @units   m/s、m/s²、m、s
 * @note    ★ 0.02 m/s 级慢漂移在 1 s 窗口内低于 IMU 偏差/噪声，本层不承诺
 *          检测；只能依赖 RTK/光流/视觉/已知点等外部基准。恒定位置偏置+
 *          一致速度的欺骗也不可区分。
 * @test    test_host/test_gnss_consistency.cpp
 * @see     src/navigation_task.cpp、lib/navigation-main/src/ekf_15_state.h
 */

#ifndef TANDEMVEC_INS_GNSS_CONSISTENCY_H
#define TANDEMVEC_INS_GNSS_CONSISTENCY_H

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>

namespace bfs
{

enum class GnssConsistencyState : uint8_t
{
    Ok = 0,
    Warning = 1,
    Suspect = 2,
};

enum class GnssConsistencyReason : uint8_t
{
    None = 0,
    Accel = 1,
    Position = 2,
    Flow = 3,
    Baro = 4,
};

struct GnssConsistencyConfig
{
    float window_s = 1.0f;
    float accel_threshold_mps2 = 0.35f;
    float accel_warn_ratio = 0.60f;
    // 加速度残差单独判 Suspect 需要更高的门限：姿态误差/位置保持动态会
    // 在 1 s 窗口产生 ~1 m/s² 的假残差；0.5 m/s² 慢 ramp 由光流/位置交叉
    // 证据判定，不能只凭加速度。
    float accel_suspect_threshold_mps2 = 1.5f;
    float pos_threshold_m = 5.0f;
    float flow_threshold_mps = 1.2f;
    float baro_span_threshold_m = 4.0f;
    float suspect_hold_s = 5.0f;
    float max_dt_s = 0.35f;
};

struct GnssConsistencyInput
{
    float dt_s = 0.1f;
    Eigen::Vector3f gnss_vel_ned = Eigen::Vector3f::Zero();
    Eigen::Vector3f imu_dv_ned = Eigen::Vector3f::Zero();
    bool pos_valid = false;
    Eigen::Vector3f gnss_pos_ned = Eigen::Vector3f::Zero();
    bool flow_valid = false;
    Eigen::Vector3f flow_vel_ned = Eigen::Vector3f::Zero();
    bool baro_valid = false;
    float gnss_alt_up_m = 0.0f;
    float baro_alt_up_m = 0.0f;
    bool reset = false;
};

struct GnssConsistencyStatus
{
    GnssConsistencyState state = GnssConsistencyState::Ok;
    GnssConsistencyReason last_reason = GnssConsistencyReason::None;
    float accel_res_mps2 = 0.0f;
    float pos_res_m = 0.0f;
    float flow_res_mps = 0.0f;
    float baro_span_m = 0.0f;
    float suspect_hold_s = 0.0f;
    uint32_t windows = 0;
    uint32_t suspect_count = 0;
};

class InsGnssConsistencyMonitor
{
  public:
    InsGnssConsistencyMonitor() = default;
    explicit InsGnssConsistencyMonitor(const GnssConsistencyConfig &config)
        : config_(config)
    {
    }

    void reset()
    {
        status_ = GnssConsistencyStatus{};
        have_prev_ = false;
        prev_vel_.setZero();
        prev_pos_.setZero();
        window_s_ = 0.0f;
        dv_sum_.setZero();
        pos_max_ = 0.0f;
        have_prev_pos_ = false;
        flow_sum_ = 0.0f;
        flow_count_ = 0U;
        baro_min_ = 0.0f;
        baro_max_ = 0.0f;
        baro_count_ = 0U;
    }

    const GnssConsistencyStatus &status() const { return status_; }
    bool suspect() const
    {
        return status_.state == GnssConsistencyState::Suspect;
    }

    /**
     * @brief 处理一个 GNSS epoch
     * @return 当前状态；input.reset 时清空历史并返回 Ok
     * @note `imu_dv_ned` 必须覆盖同一 dt 区间；dt>0.35 s 视为失联/重锚并自动清窗。
     */
    GnssConsistencyStatus update(const GnssConsistencyInput &in)
    {
        if (in.reset || !(in.dt_s > 0.0f) || !std::isfinite(in.dt_s) ||
            in.dt_s > config_.max_dt_s ||
            !in.gnss_vel_ned.allFinite() || !in.imu_dv_ned.allFinite())
        {
            reset();
            return status_;
        }
        const float dt = in.dt_s;
        if (!have_prev_)
        {
            prev_vel_ = in.gnss_vel_ned;
            prev_pos_ = in.gnss_pos_ned;
            have_prev_ = true;
            have_prev_pos_ = in.pos_valid && in.gnss_pos_ned.allFinite();
            return status_;
        }

        const Eigen::Vector3f dv_res =
            (in.gnss_vel_ned - prev_vel_) - in.imu_dv_ned;
        dv_sum_ += dv_res;
        if (in.pos_valid && in.gnss_pos_ned.allFinite())
        {
            if (have_prev_pos_)
            {
                const Eigen::Vector3f pos_res =
                    in.gnss_pos_ned -
                    (prev_pos_ + 0.5f * (prev_vel_ + in.gnss_vel_ned) * dt);
                pos_max_ = std::fmax(pos_max_, pos_res.norm());
            }
            prev_pos_ = in.gnss_pos_ned;
            have_prev_pos_ = true;
        }
        else
        {
            have_prev_pos_ = false;
        }
        if (in.flow_valid && in.flow_vel_ned.allFinite())
        {
            const float flow_res =
                (in.gnss_vel_ned - in.flow_vel_ned).head<2>().norm();
            flow_sum_ += flow_res;
            ++flow_count_;
        }
        if (in.baro_valid && std::isfinite(in.gnss_alt_up_m) &&
            std::isfinite(in.baro_alt_up_m))
        {
            const float baro_res = in.gnss_alt_up_m - in.baro_alt_up_m;
            if (baro_count_ == 0U)
            {
                baro_min_ = baro_max_ = baro_res;
            }
            else
            {
                baro_min_ = std::fmin(baro_min_, baro_res);
                baro_max_ = std::fmax(baro_max_, baro_res);
            }
            ++baro_count_;
        }

        window_s_ += dt;
        prev_vel_ = in.gnss_vel_ned;
        if (window_s_ < config_.window_s)
        {
            return status_;
        }

        const float accel_res = dv_sum_.norm() / window_s_;
        const float pos_res = pos_max_;
        const float flow_res =
            flow_count_ > 0U ? flow_sum_ / static_cast<float>(flow_count_)
                             : -1.0f;
        const float baro_span =
            baro_count_ > 1U ? (baro_max_ - baro_min_) : 0.0f;
        const bool bad_accel =
            accel_res > config_.accel_suspect_threshold_mps2;
        const bool bad_pos = pos_res > config_.pos_threshold_m;
        const bool bad_flow = flow_res > config_.flow_threshold_mps;
        const bool bad_baro = baro_span > config_.baro_span_threshold_m;
        ++status_.windows;
        status_.accel_res_mps2 = accel_res;
        status_.pos_res_m = pos_res;
        status_.flow_res_mps = flow_res;
        status_.baro_span_m = baro_span;
        if (bad_accel || bad_pos || bad_flow || bad_baro)
        {
            status_.last_reason = bad_accel   ? GnssConsistencyReason::Accel
                                  : bad_pos   ? GnssConsistencyReason::Position
                                  : bad_flow  ? GnssConsistencyReason::Flow
                                              : GnssConsistencyReason::Baro;
            status_.suspect_hold_s = config_.suspect_hold_s;
            ++status_.suspect_count;
        }
        else
        {
            status_.suspect_hold_s =
                std::fmax(0.0f, status_.suspect_hold_s - window_s_);
            if (status_.suspect_hold_s <= 0.0f)
            {
                // 锁存期内保持 Suspect；锁存结束后按本窗口是否接近门限决定
                // Warning/Ok，不能永远停在 Warning。
                status_.last_reason =
                    (accel_res > config_.accel_warn_ratio *
                                     config_.accel_threshold_mps2)
                        ? GnssConsistencyReason::Accel
                        : GnssConsistencyReason::None;
            }
        }
        status_.state = status_.suspect_hold_s > 0.0f
                            ? GnssConsistencyState::Suspect
                            : (status_.last_reason == GnssConsistencyReason::None
                                   ? GnssConsistencyState::Ok
                                   : GnssConsistencyState::Warning);

        window_s_ = 0.0f;
        dv_sum_.setZero();
        pos_max_ = 0.0f;
        flow_sum_ = 0.0f;
        flow_count_ = 0U;
        baro_min_ = baro_max_ = 0.0f;
        baro_count_ = 0U;
        return status_;
    }

  private:
    GnssConsistencyConfig config_{};
    GnssConsistencyStatus status_{};
    bool have_prev_ = false;
    bool have_prev_pos_ = false;
    Eigen::Vector3f prev_vel_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f prev_pos_ = Eigen::Vector3f::Zero();
    float window_s_ = 0.0f;
    Eigen::Vector3f dv_sum_ = Eigen::Vector3f::Zero();
    float pos_max_ = 0.0f;
    float flow_sum_ = 0.0f;
    uint32_t flow_count_ = 0U;
    float baro_min_ = 0.0f;
    float baro_max_ = 0.0f;
    uint32_t baro_count_ = 0U;
};

} // namespace bfs

#endif // TANDEMVEC_INS_GNSS_CONSISTENCY_H
