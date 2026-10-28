/**
 * @file    ins_imu_saturation.h
 * @module  固件/导航（IMU 饱和锁存）
 * @brief   把 IMU 健康监测的“本帧饱和”脉冲锁存成持续窗口，供导航层在
 *          加速度削顶期间膨胀过程噪声；带进入/释放双计时，避免削顶边界抖动
 * @details 输入：每帧“是否饱和”（由 `ImuHealthMonitor` 的 saturation_s>0 给出）
 *          与 dt；处理：饱和累计到 enter_s 后锁存，无饱和累计到 release_s 后释放；
 *          输出：锁存状态与加计噪声标准差建议值（未锁存时原样返回标称值）。
 *          不变量：dt 非有限/非正时状态不变；未锁存时绝不改变调用方传入的标称值。
 * @theory  传感器削顶期间比力幅值不可信、但方向仍可用的退化观测模型：
 *          被削顶的比力误差上界约为量程外真实比力与量程之差，滤波应把这段
 *          误差计入过程噪声而不是量测噪声，才能让 GNSS 主导速度/位置。
 *          @ref REF-121 §7.3.1 p.255–259（系统级故障检测与隔离）；
 *          AGENTS.md 踩坑记录（生产 ICM ±8 g 量程、10.1 g 包线失效）
 * @frame  N/A（只处理标量幅值判定，不解释姿态）
 * @units  s、m/s²
 * @note   本模块只给“建议噪声”，不直接写 EKF；是否启用由导航层 opt-in 宏控制。
 *          默认关闭时行为与既有一致（饱和仍只作为健康 Warning）。
 * @test    test_host/test_ins_imu_saturation.cpp
 * @see     src/navigation_task.cpp、include/ins_imu_health.h
 */

#ifndef TANDEMVEC_INS_IMU_SATURATION_H
#define TANDEMVEC_INS_IMU_SATURATION_H

#include <cmath>

namespace bfs
{

struct ImuSaturationQConfig
{
    float enter_s = 0.05f;
    float release_s = 1.0f;
    float inflate_accel_std_mps2 = 25.0f;
};

/**
 * @brief 饱和锁存 + 加计噪声建议
 * @note  enter_s 是“连续饱和”累计门限；release_s 是“连续不饱和”释放门限。
 *        两者都按导航帧 dt 累加，不依赖 millis()，宿主可逐帧复现。
 */
class ImuSaturationQMonitor
{
  public:
    ImuSaturationQMonitor() = default;
    explicit ImuSaturationQMonitor(const ImuSaturationQConfig &config)
        : config_(config)
    {
    }

    void reset()
    {
        latched_ = false;
        saturated_s_ = 0.0f;
        clean_s_ = 0.0f;
    }

    bool update(const bool saturated, const float dt_s)
    {
        if (!(dt_s > 0.0f) || !std::isfinite(dt_s))
        {
            return latched_;
        }
        if (saturated)
        {
            clean_s_ = 0.0f;
            saturated_s_ += dt_s;
            if (saturated_s_ + 1.0e-6f >= config_.enter_s)
            {
                latched_ = true;
            }
        }
        else
        {
            saturated_s_ = 0.0f;
            if (latched_)
            {
                clean_s_ += dt_s;
                if (clean_s_ + 1.0e-6f >= config_.release_s)
                {
                    latched_ = false;
                    clean_s_ = 0.0f;
                }
            }
        }
        return latched_;
    }

    bool active() const { return latched_; }

    float accelStdMps2(const float nominal) const
    {
        if (!latched_)
        {
            return nominal;
        }
        const bool nominal_valid = std::isfinite(nominal) && nominal > 0.0f;
        return nominal_valid && nominal > config_.inflate_accel_std_mps2
                   ? nominal
                   : config_.inflate_accel_std_mps2;
    }

    float saturatedDurationS() const { return saturated_s_; }
    float cleanDurationS() const { return clean_s_; }
    const ImuSaturationQConfig &config() const { return config_; }
    void set_config(const ImuSaturationQConfig &config) { config_ = config; }

  private:
    ImuSaturationQConfig config_{};
    bool latched_ = false;
    float saturated_s_ = 0.0f;
    float clean_s_ = 0.0f;
};

} // namespace bfs

#endif // TANDEMVEC_INS_IMU_SATURATION_H
