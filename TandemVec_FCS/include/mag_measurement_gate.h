/**
 * @file    mag_measurement_gate.h
 * @module  固件/导航（磁测量质量门控）
 * @brief   在磁矢量进入 EKF 前做新鲜度、标定、模长、倾角、残差、航迹与
 *          油门干扰检查，并给出自适应量测噪声
 * @details 输入：MagSensor 的机体 FRD 样本（µT，已完成椭球标定/轴对齐）、
 *          样本序号/时间戳、参考场、EKF 当前 C^n_b、估计航向、GNSS 航迹、
 *          速度、姿态不确定度、油门；处理：逐项门控 + 自适应噪声；
 *          输出：pass/拒绝原因 + 诊断量。**本层不写 EKF、不写执行器**。
 *          不变量：只有 `pass=true` 才允许调用磁矢量量测更新；门控失败时
 *          EKF 状态与协方差完全不变。
 * @theory  磁航向 = 地磁水平分量在机体系中的方位；倾角补偿后与参考场比较。
 *          模长/倾角偏差用于检测硬/软铁残差与局部磁干扰，NIS/χ² 作为 EKF
 *          内部第二层；GNSS 航迹只在有速度时作为弱交叉证据（含风/侧滑）。
 *          @ref REF-120 §9.9.1 p.323–324（磁矢量定方位）、
 *          REF-121 §7.3.1 p.255–259（NIS/FDI）、
 *          notes/04-航向与磁辅助（磁偏角、倾角与航向可观测性）
 * @frame   NED 参考场（x 北/y 东/z 下）+ 机体 FRD 量测；C^n_b 为 NED→body
 * @units   磁场 µT，角度 rad，时间 µs，速度 m/s，油门 %
 * @note    门限是仿真/台架保守值；真实硬铁/软铁残差、电机电流干扰和安装
 *          方向必须在实机重新标定后再调。`heading_error_rad` 是“参考场方位
 *          与量测方位之差”，不是 EKF 最终航向误差。
 * @test    test_host/test_mag_gate.cpp
 * @see     mag_reference.h、lib/MagSensor/MagSensor.h、
 *          src/navigation_task.cpp、lib/navigation-main/src/ekf_15_state.h
 */

#ifndef TANDEMVEC_MAG_MEASUREMENT_GATE_H
#define TANDEMVEC_MAG_MEASUREMENT_GATE_H

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <limits>

#include "mag_reference.h"

namespace bfs
{

inline float magWrapPi(float angle_rad);

/**
 * @brief 由机体系磁矢量与 roll/pitch 计算倾斜补偿绝对航向 [rad]（NED）
 * @param[in] mag_body_ut 机体系磁矢量（FRD，已标定）
 * @param[in] roll_rad    EKF 横滚 φ
 * @param[in] pitch_rad   EKF 俯仰 θ
 * @param[in] mag_ned_ref NED 参考磁矢量（含磁偏角/倾角）
 * @return 航向 ψ（真北，[-π,π]）；输入非有限时返回 NaN
 * @formula C^n_b = Rx(φ)Ry(θ)Rz(ψ)，m_b = C^T m_n；
 *          n = (Rx(φ)Ry(θ))^T m_n，ψ = atan2(n_y,n_x) − atan2(m_by,m_bx)
 * @ref REF-120 §9.8.2 (9.8.13) p.317–322（姿态与磁观测）；
 *      notes/06（航向可观测性）
 * @note 只用 EKF 的 roll/pitch，不依赖 EKF 当前 yaw；roll/pitch 误差会直接
 *       引入航向偏差，只适合短时 GNSS 失联的标量 yaw 回退，不能替代 3D 磁更新。
 */
inline float magTiltCompensatedHeadingRad(
    const Eigen::Vector3f &mag_body_ut,
    const float roll_rad,
    const float pitch_rad,
    const Eigen::Vector3f &mag_ned_ref)
{
    if (!mag_body_ut.allFinite() || !mag_ned_ref.allFinite() ||
        !std::isfinite(roll_rad) || !std::isfinite(pitch_rad))
    {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const float cr = std::cos(roll_rad);
    const float sr = std::sin(roll_rad);
    const float cp = std::cos(pitch_rad);
    const float sp = std::sin(pitch_rad);
    // n = (Rx·Ry)^T · m_ned，展开避免引入 transforms 依赖
    const float rx = mag_ned_ref.y() * cr + mag_ned_ref.z() * sr;
    const float rz = -mag_ned_ref.y() * sr + mag_ned_ref.z() * cr;
    const float nx = cp * mag_ned_ref.x() - sp * rz;
    const float ny = rx;
    return magWrapPi(std::atan2(ny, nx) -
                     std::atan2(mag_body_ut.y(), mag_body_ut.x()));
}

enum class MagGateReason : uint8_t
{
    None = 0,
    RefInvalid,
    CalInvalid,
    NonFinite,
    NoNewSample,
    Stale,
    NormLow,
    NormHigh,
    NormDeviation,
    DipDeviation,
    Residual,
    TrackMismatch,
    TiltUncertain,
    ThrottleHigh,
    EkfRejected, // 通过上层门控但被 EKF NIS/输入检查拒绝
    Count
};

const char *magGateReasonName(const MagGateReason reason);

struct MagGateConfig
{
    uint32_t max_age_us = 250000U;
    float min_norm_ut = 15.0f;
    float max_norm_ut = 100.0f;
    float max_norm_dev_ut = 8.0f;
    float max_dip_dev_rad = 10.0f * bfs::kMagReferenceDegToRad;
    float max_residual_rad = 45.0f * bfs::kMagReferenceDegToRad;
    float max_track_dev_rad = 20.0f * bfs::kMagReferenceDegToRad;
    float max_tilt_sigma_rad = 8.0f * bfs::kMagReferenceDegToRad;
    float max_throttle_pct = 85.0f;
    float base_noise_ut = 8.0f;
    float max_noise_ut = 20.0f;
    float k_norm_noise = 0.25f;
    float k_dip_noise = 10.0f;
    float k_track_noise = 6.0f;
    float min_speed_for_track_mps = 3.0f;
};

struct MagGateInput
{
    Eigen::Vector3f mag_body_ut = Eigen::Vector3f::Zero();
    uint32_t sample_timestamp_us = 0;
    uint32_t now_us = 0;
    uint32_t sample_seq = 0;
    uint32_t last_consumed_seq = 0;
    bool cal_valid = false;
    const MagReferenceNed *ref = nullptr;
    Eigen::Matrix3f c_n_b = Eigen::Matrix3f::Identity(); // NED -> body
    bool gnss_course_valid = false;
    float gnss_course_rad = 0.0f;
    float estimated_yaw_rad = 0.0f;
    float speed_mps = 0.0f;
    float tilt_sigma_rad = 0.0f;
    float throttle_pct = 0.0f;
};

struct MagGateResult
{
    bool pass = false;
    MagGateReason reason = MagGateReason::None;
    float norm_ut = 0.0f;
    float norm_dev_ut = 0.0f;
    float dip_rad = 0.0f;
    float dip_dev_rad = 0.0f;
    float heading_error_rad = 0.0f;
    float track_dev_rad = 0.0f;
    float noise_ut = 0.0f;
};

inline float magWrapPi(float angle_rad)
{
    constexpr float kTwoPi = 6.28318530717958647692f;
    if (!std::isfinite(angle_rad))
    {
        return angle_rad;
    }
    while (angle_rad > bfs::kMagReferencePi)
    {
        angle_rad -= kTwoPi;
    }
    while (angle_rad < -bfs::kMagReferencePi)
    {
        angle_rad += kTwoPi;
    }
    return angle_rad;
}

class MagMeasurementGate
{
  public:
    MagMeasurementGate() = default;
    explicit MagMeasurementGate(const MagGateConfig &config) : config_(config) {}

    const MagGateConfig &config() const { return config_; }
    void set_config(const MagGateConfig &config) { config_ = config; }

    MagGateResult Evaluate(const MagGateInput &in) const
    {
        MagGateResult out;
        if (in.ref == nullptr || !in.ref->valid)
        {
            out.reason = MagGateReason::RefInvalid;
            return out;
        }
        if (!in.cal_valid)
        {
            out.reason = MagGateReason::CalInvalid;
            return out;
        }
        if (!in.mag_body_ut.allFinite() || !in.c_n_b.allFinite() ||
            !std::isfinite(in.estimated_yaw_rad) ||
            !std::isfinite(in.gnss_course_rad) ||
            !std::isfinite(in.speed_mps) ||
            !std::isfinite(in.tilt_sigma_rad) ||
            !std::isfinite(in.throttle_pct) ||
            !std::isfinite(in.ref->total_ut) || !std::isfinite(in.ref->dip_rad) ||
            !std::isfinite(in.ref->declination_rad))
        {
            out.reason = MagGateReason::NonFinite;
            return out;
        }
        if (in.sample_seq == 0U || in.sample_seq == in.last_consumed_seq)
        {
            out.reason = MagGateReason::NoNewSample;
            return out;
        }
        // micros() 32 位回绕在 71 分钟；用无符号差，短龄期判断仍正确。
        const uint32_t age_us = in.now_us - in.sample_timestamp_us;
        if (in.sample_timestamp_us == 0U || age_us > config_.max_age_us)
        {
            out.reason = MagGateReason::Stale;
            return out;
        }
        if (in.throttle_pct > config_.max_throttle_pct)
        {
            out.reason = MagGateReason::ThrottleHigh;
            return out;
        }

        const float norm = in.mag_body_ut.norm();
        out.norm_ut = norm;
        out.norm_dev_ut = std::fabs(norm - in.ref->total_ut);
        if (norm < config_.min_norm_ut)
        {
            out.reason = MagGateReason::NormLow;
            return out;
        }
        if (norm > config_.max_norm_ut)
        {
            out.reason = MagGateReason::NormHigh;
            return out;
        }
        if (out.norm_dev_ut > config_.max_norm_dev_ut)
        {
            out.reason = MagGateReason::NormDeviation;
            return out;
        }

        // C^n_b 为 NED→body；转置得到 body→NED，取水平方位与倾角。
        const Eigen::Vector3f mag_ned = in.c_n_b.transpose() * in.mag_body_ut;
        const float horizontal = std::hypot(mag_ned.x(), mag_ned.y());
        out.dip_rad = std::atan2(mag_ned.z(), horizontal);
        out.dip_dev_rad = std::fabs(out.dip_rad - in.ref->dip_rad);
        if (out.dip_dev_rad > config_.max_dip_dev_rad)
        {
            out.reason = MagGateReason::DipDeviation;
            return out;
        }

        const float measured_azimuth = std::atan2(mag_ned.y(), mag_ned.x());
        out.heading_error_rad =
            magWrapPi(in.ref->declination_rad - measured_azimuth);
        if (std::fabs(out.heading_error_rad) > config_.max_residual_rad)
        {
            out.reason = MagGateReason::Residual;
            return out;
        }

        if (in.gnss_course_valid &&
            in.speed_mps > config_.min_speed_for_track_mps)
        {
            const float course_error =
                magWrapPi(in.gnss_course_rad - in.estimated_yaw_rad);
            out.track_dev_rad =
                std::fabs(magWrapPi(out.heading_error_rad + course_error));
            if (out.track_dev_rad > config_.max_track_dev_rad)
            {
                out.reason = MagGateReason::TrackMismatch;
                return out;
            }
        }

        if (in.tilt_sigma_rad > config_.max_tilt_sigma_rad)
        {
            out.reason = MagGateReason::TiltUncertain;
            return out;
        }

        float noise_ut = config_.base_noise_ut +
                         config_.k_norm_noise * out.norm_dev_ut +
                         config_.k_dip_noise * out.dip_dev_rad +
                         config_.k_track_noise * out.track_dev_rad;
        if (!std::isfinite(noise_ut))
        {
            out.reason = MagGateReason::NonFinite;
            return out;
        }
        if (noise_ut < config_.base_noise_ut)
        {
            noise_ut = config_.base_noise_ut;
        }
        if (noise_ut > config_.max_noise_ut)
        {
            noise_ut = config_.max_noise_ut;
        }
        out.noise_ut = noise_ut;
        out.pass = true;
        out.reason = MagGateReason::None;
        return out;
    }

  private:
    MagGateConfig config_{};
};

inline const char *magGateReasonName(const MagGateReason reason)
{
    switch (reason)
    {
    case MagGateReason::None:
        return "none";
    case MagGateReason::RefInvalid:
        return "ref";
    case MagGateReason::CalInvalid:
        return "cal";
    case MagGateReason::NonFinite:
        return "nonfinite";
    case MagGateReason::NoNewSample:
        return "no_new";
    case MagGateReason::Stale:
        return "stale";
    case MagGateReason::NormLow:
        return "norm_low";
    case MagGateReason::NormHigh:
        return "norm_high";
    case MagGateReason::NormDeviation:
        return "norm_dev";
    case MagGateReason::DipDeviation:
        return "dip_dev";
    case MagGateReason::Residual:
        return "residual";
    case MagGateReason::TrackMismatch:
        return "track";
    case MagGateReason::TiltUncertain:
        return "tilt";
    case MagGateReason::ThrottleHigh:
        return "throttle";
    case MagGateReason::EkfRejected:
        return "ekf_rej";
    case MagGateReason::Count:
    default:
        return "count";
    }
}

} // namespace bfs

#endif // TANDEMVEC_MAG_MEASUREMENT_GATE_H
