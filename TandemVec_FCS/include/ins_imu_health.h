/**
 * @file    ins_imu_health.h
 * @module  固件/导航（IMU 健康监测）
 * @brief   检测 IMU 非有限值、无样本/低样本、精确冻结、饱和与高振动，输出
 *          Ok/Warning/Failed 与故障标志；只做诊断/隔离，不写控制输出
 * @details 输入：导航帧平均比力/角速度、本帧 IMU 样本数、dt、振动估计、
 *          解锁/油门状态、已有零偏跳变标志；处理：逐项连续时间计数并聚合
 *          状态；输出：`ImuHealthStatus`（状态、标志、计时器）与
 *          `staticAidsAllowed()`。
 *          不变量：非有限/确认冻结/无样本为 Failed；饱和/低样本/高振动/
 *          零偏跳变为 Warning；Failed 时调用方必须禁止 ZUPT/Gravity/StaticGyro。
 * @theory  系统级 FDI：残差/状态门限 + 持续时间确认，检测与隔离分离
 *          @ref REF-121 §7.3.1 p.255–259（NIS/χ² 与系统级故障检测）；
 *          AGENTS.md 踩坑记录（IMU NaN/冻结、静止检测误判）
 * @frame   N/A（三轴矢量模长/逐轴差分，不解释姿态）
 * @units   m/s²、rad/s、s、Hz、g
 * @note    ★ 本监测只覆盖“精确重复/无样本/非有限/饱和”这类可观测模式；
 *          “卡在缓慢变化值”或攻击者伪造带噪声数据不在此列。单 IMU 无冗余，
 *          检测到 Failed 也不能保证安全降落；本模块不自动切油门。
 * @test    test_host/test_imu_health_monitor.cpp
 * @see     src/navigation_task.cpp、include/ins_static_detector.h
 */

#ifndef TANDEMVEC_INS_IMU_HEALTH_H
#define TANDEMVEC_INS_IMU_HEALTH_H

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>

namespace bfs
{

enum class ImuHealthState : uint8_t
{
    Ok = 0,
    Warning = 1,
    Failed = 2,
};

enum class ImuHealthFlag : uint16_t
{
    None = 0,
    NonFinite = 1U << 0,
    NoSamples = 1U << 1,
    LowSampleRate = 1U << 2,
    Frozen = 1U << 3,
    Saturated = 1U << 4,
    HighVibration = 1U << 5,
    BiasJump = 1U << 6,
};

inline uint16_t imuHealthFlagBit(const ImuHealthFlag flag)
{
    return static_cast<uint16_t>(flag);
}

struct ImuHealthConfig
{
    float expected_sample_rate_hz = 2000.0f;
    float min_sample_fraction = 0.5f;
    float no_sample_confirm_s = 0.2f;
    float frozen_epsilon = 1.0e-6f;
    float frozen_confirm_s = 0.5f;
    float accel_range_mps2 = 78.45f;  // ICM42688 ±8 g
    float gyro_range_radps = 34.9066f; // ICM42688 ±2000 dps
    float saturation_fraction = 0.95f;
    float saturation_confirm_s = 0.2f;
    float high_vibration_g = 3.0f;
};

struct ImuHealthStatus
{
    ImuHealthState state = ImuHealthState::Ok;
    uint16_t flags = 0;
    float no_sample_s = 0.0f;
    float frozen_s = 0.0f;
    float saturation_s = 0.0f;
    float sample_rate_hz = 0.0f;
};

/**
 * @brief 平台无关的 IMU 健康状态机
 * @note 采用“连续超阈值计时 + 状态聚合”，不依赖 Arduino/Eigen 之外的平台层。
 */
class ImuHealthMonitor
{
  public:
    ImuHealthMonitor() = default;
    explicit ImuHealthMonitor(const ImuHealthConfig &config) : config_(config) {}

    void reset()
    {
        status_ = ImuHealthStatus{};
        have_last_ = false;
        last_accel_.setZero();
        last_gyro_.setZero();
    }

    ImuHealthStatus update(const Eigen::Vector3f &accel,
                           const Eigen::Vector3f &gyro,
                           const uint32_t sample_count,
                           const float dt_s,
                           const float vib_g,
                           const bool unlocked,
                           const float throttle_pct,
                           const bool bias_jump = false)
    {
        if (!(dt_s > 0.0f) || !std::isfinite(dt_s))
        {
            return status_;
        }
        const float dt = std::fmin(dt_s, 0.05f);
        const float safe_vib =
            (std::isfinite(vib_g) && vib_g > 0.0f) ? vib_g : 0.0f;
        status_.flags = 0;

        if (!accel.allFinite() || !gyro.allFinite())
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::NonFinite);
            status_.state = ImuHealthState::Failed;
            status_.frozen_s = 0.0f;
            status_.saturation_s = 0.0f;
            have_last_ = false;
            return status_;
        }

        const float expected_samples =
            config_.expected_sample_rate_hz * dt;
        status_.sample_rate_hz = static_cast<float>(sample_count) / dt;
        if (sample_count == 0U)
        {
            status_.no_sample_s += dt;
        }
        else
        {
            status_.no_sample_s = 0.0f;
            if (expected_samples > 1.0f &&
                static_cast<float>(sample_count) <
                    config_.min_sample_fraction * expected_samples)
            {
                status_.flags |= imuHealthFlagBit(ImuHealthFlag::LowSampleRate);
            }
        }
        if (status_.no_sample_s >= config_.no_sample_confirm_s)
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::NoSamples);
        }

        // 冻结检测：只在上锁/有油门时启用，避免地面静止/无噪声测试误判。
        const bool frozen_armed = unlocked && throttle_pct >= 3.0f;
        if (sample_count > 0U && frozen_armed && have_last_)
        {
            const float accel_delta =
                (accel - last_accel_).cwiseAbs().maxCoeff();
            const float gyro_delta =
                (gyro - last_gyro_).cwiseAbs().maxCoeff();
            // 精确重复本身已是强证据；振动估计可能因冻结样本而滞后，
            // 不能用它来屏蔽“同一帧样本反复出现”。真实振动/噪声下不会
            // 连续 0.5 s 完全一致，因此不会误报。
            const bool flat = accel_delta < config_.frozen_epsilon &&
                              gyro_delta < config_.frozen_epsilon;
            status_.frozen_s = flat ? (status_.frozen_s + dt) : 0.0f;
        }
        else
        {
            status_.frozen_s = 0.0f;
        }
        if (status_.frozen_s >= config_.frozen_confirm_s)
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::Frozen);
        }

        const float accel_limit =
            config_.accel_range_mps2 * config_.saturation_fraction;
        const float gyro_limit =
            config_.gyro_range_radps * config_.saturation_fraction;
        const bool saturated =
            accel.cwiseAbs().maxCoeff() > accel_limit ||
            gyro.cwiseAbs().maxCoeff() > gyro_limit;
        status_.saturation_s =
            saturated ? (status_.saturation_s + dt) : 0.0f;
        if (status_.saturation_s >= config_.saturation_confirm_s)
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::Saturated);
        }

        if (safe_vib > config_.high_vibration_g)
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::HighVibration);
        }
        if (bias_jump)
        {
            status_.flags |= imuHealthFlagBit(ImuHealthFlag::BiasJump);
        }

        const bool failed =
            (status_.flags &
             (imuHealthFlagBit(ImuHealthFlag::NonFinite) |
              imuHealthFlagBit(ImuHealthFlag::NoSamples) |
              imuHealthFlagBit(ImuHealthFlag::Frozen))) != 0U;
        if (failed)
        {
            status_.state = ImuHealthState::Failed;
        }
        else if (status_.flags != 0U)
        {
            status_.state = ImuHealthState::Warning;
        }
        else
        {
            status_.state = ImuHealthState::Ok;
        }

        if (sample_count > 0U)
        {
            last_accel_ = accel;
            last_gyro_ = gyro;
            have_last_ = true;
        }
        return status_;
    }

    const ImuHealthStatus &status() const { return status_; }
    bool staticAidsAllowed() const
    {
        // 冻结候选/无样本期间即使尚未确认 Failed，也不能让 ZUPT/Gravity/StaticGyro
        // 把“传感器没更新”当成“机体静止”。
        return status_.state != ImuHealthState::Failed &&
               status_.frozen_s <= 0.0f && status_.no_sample_s <= 0.0f;
    }
    bool failed() const
    {
        return status_.state == ImuHealthState::Failed;
    }
    const ImuHealthConfig &config() const { return config_; }
    void set_config(const ImuHealthConfig &config) { config_ = config; }

  private:
    ImuHealthConfig config_{};
    ImuHealthStatus status_{};
    Eigen::Vector3f last_accel_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_gyro_ = Eigen::Vector3f::Zero();
    bool have_last_ = false;
};

} // namespace bfs

#endif // TANDEMVEC_INS_IMU_HEALTH_H
