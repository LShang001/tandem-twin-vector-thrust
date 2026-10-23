/**
 * @file    aid_source_status.h
 * @module  固件/导航（辅助观测源生命周期与诊断）
 * @brief   记录各类辅助观测源的可用性、融合尝试、门控结果与龄期
 * @details 输入：每帧的观测源事件（是否可用/质量是否通过/是否尝试/是否融合/
 *          是否拒绝/是否超时/是否触发重置）；处理：BeginFrame() 清瞬时标志并
 *          推进 age_s，RecordAttempt() 累计计数，RecordTimeout()/RecordReset()
 *          记异常；输出：AidSourceStatus 快照与 AgeForOutput()，供遥测/VOFA
 *          诊断消费。与 Arduino、Eigen、串口协议完全解耦，可宿主机回归。
 *          不变量：瞬时标志仅本帧有效；累计计数与 age_s 跨帧保留。
 * @theory  NIS/χ² 量测有效性检验与系统级故障检测 @ref REF-121 §7.3.1 p.255–259；
 *          间接法中“阻尼必须利用外来信息”@ref REF-121 §8.2.3 p.291
 *          （解释为何 GNSS/ZUPT 的 rejected 计数必须与外部参照一起解读）
 * @frame   N/A（纯诊断记账层，不解释任何坐标系）
 * @units   age_s、dt_s 单位 s；innovation_norm 与 test_ratio 无量纲/随量测维度
 * @note    本层只记账、不做门控决策：fused/rejected 必须来自 EKF 的
 *          MeasurementUpdateResult，禁止上层“调用成功即标记 fused”。
 * @see     ekf_15_state.h::MeasurementUpdateResult、src/navigation_task.cpp
 */

#ifndef NAVIGATION_SRC_AID_SOURCE_STATUS_H_ // NOLINT
#define NAVIGATION_SRC_AID_SOURCE_STATUS_H_

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace bfs
{

/*
 * 辅助观测源枚举。
 *
 * 每个值对应一类可选的 EKF 辅助量测。Count 是枚举末尾的哨兵，
 * 用于数组大小和循环遍历。
 */
enum class AidSource : uint8_t
{
  StaticZupt = 0, // 静止零速量测 (ZUPT)，主要约束速度漂移和加速度零偏
  Gravity = 1,    // 重力方向量测，主要约束 roll/pitch 和横向零偏
  Attitude = 2,   // 外部 AHRS/Madgwick roll/pitch 姿态辅助
  Gnss = 3,       // GNSS 位置 + 速度 6 维组合量测
  YawGsf = 4,     // GNSS 速度辅助航向估计 (EKF-GSF)，后续可接双天线航向
  StaticGyro = 5, // 静止角速度量测，只约束陀螺零偏，不直接注入姿态
  Baro = 6,       // 气压高度标量量测，观测 NED D 分量 (位置误差状态索引 2)
  Count = 7
};

/*
 * 单个观测源的状态快照。
 *
 * 每个导航帧开始时，BeginFrame() 会清除本帧瞬时标志（available、
 * quality_passed、attempted、fused、rejected、timed_out、
 * innovation_norm、test_ratio_max），保留累计计数和 age_s。
 * 随后由上层在各量测更新路径中逐步填充。
 */
struct AidSourceStatus
{
  bool available = false;          // 当前帧该观测源是否有原始数据或满足触发条件
  bool quality_passed = false;     // 原始质量/静止置信度/运动条件是否允许尝试融合
  bool attempted = false;          // 当前帧是否实际调用了 EKF 量测接口
  bool fused = false;              // 当前帧量测是否被 EKF 接受并完成状态更新
  bool rejected = false;           // 当前帧量测是否被质量检查或 EKF 门控拒绝
  bool timed_out = false;          // 当前观测源是否超时（主要给 GNSS/外部传感器使用）
  float age_s = FLT_MAX;           // 距离上一次成功融合的时间，未融合过保持 FLT_MAX
  float innovation_norm = 0.0f;    // 新息模长，用于观察残差是否接近门限
  float test_ratio_max = 0.0f;     // 最大门控比或等效 NIS 比值，<1 通常表示门控通过
  uint32_t attempt_count = 0;      // 累计尝试融合次数
  uint32_t fuse_count = 0;         // 累计融合成功次数
  uint32_t reject_count = 0;       // 累计 EKF 更新拒绝次数
  uint32_t timeout_count = 0;      // 累计超时次数
  uint32_t reset_count = 0;        // 累计由该观测源触发的强制重置次数
};

/*
 * 量测更新的结构化返回结果。
 *
 * 由 Ekf15State 的各 Detailed 量测接口返回，区分"输入无效"、
 * "进入 EKF 但被门控拒绝"和"实际融合成功"三种情况。
 * 上层 AidSource 诊断不应使用简单 bool 返回值，必须使用本结构。
 */
struct MeasurementUpdateResult
{
  bool input_valid = false;         // 输入量测和噪声参数是否有限且物理有效
  bool attempted = false;           // 是否实际进入了 EKF 更新流程
  bool fused = false;               // 是否完成了状态/协方差更新
  bool subset_position = false;     // P0-1：仅位置子集更新是否融合
  bool subset_velocity = false;     // P0-1：仅速度子集更新是否融合
  float nis_pos3 = 0.0f;            // P0-1：3 维位置子集 NIS
  float nis_vel3 = 0.0f;            // P0-1：3 维速度子集 NIS
  float innovation_norm = 0.0f;     // 新息模长，便于上层记录
  float test_ratio = 0.0f;          // NIS/门限归一化比值，不可得时为 0
};

/*
 * 观测源状态追踪器。
 *
 * 设计意图：
 * - 不与任何特定传感器或输出协议耦合；不包含 Arduino、IMU、
 *   串口、VOFA 等平台相关依赖。
 * - 状态按帧推进：BeginFrame() 清瞬时标志 → 各辅助路径填充 →
 *   VOFA/AidSource 诊断消费。
 * - 所有累计计数和 age_s 跨帧保留，支持长时间运行后观察
 *   各类辅助融合的稳定性和拒绝率。
 */
class AidSourceStatusTracker
{
public:
  /* 获取指定观测源的可写状态引用。 */
  AidSourceStatus &status(const AidSource source)
  {
    return status_[Index(source)];
  }

  /* 获取指定观测源的只读状态。 */
  const AidSourceStatus &status(const AidSource source) const
  {
    return status_[Index(source)];
  }

  /*
   * 每个导航帧开始时调用。
   *
   * 清除本帧的瞬时标志（available、quality_passed、attempted 等），
   * 但保留累计计数器（attempt_count、fuse_count、reject_count 等）
   * 和距上次融合时间 age_s。
   *
   * @param dt_s  本次导航帧的时间步长。异常时（非有限或非正）不推进 age_s，
   *              避免坏时间戳污染诊断通道。
   */
  void BeginFrame(const float dt_s)
  {
    const float safe_dt_s = (std::isfinite(dt_s) && dt_s > 0.0f) ? dt_s : 0.0f;
    for (size_t i = 0; i < static_cast<size_t>(AidSource::Count); ++i)
    {
      status_[i].available = false;
      status_[i].quality_passed = false;
      status_[i].attempted = false;
      status_[i].fused = false;
      status_[i].rejected = false;
      status_[i].timed_out = false;
      status_[i].innovation_norm = 0.0f;
      status_[i].test_ratio_max = 0.0f;
      if (status_[i].age_s < FLT_MAX * 0.5f)
      {
        status_[i].age_s += safe_dt_s;
      }
    }
  }

  /*
   * 标记观测源不可用。
   *
   * 适用于本帧根本没有收到观测数据的场景：
   * - GNSS epoch 队列为空。
   * - 静止检测返回 false（非静止 → ZUPT/Gravity/Gyro 均不可用）。
   */
  void MarkUnavailable(const AidSource source)
  {
    AidSourceStatus &entry = status(source);
    entry.available = false;
    entry.quality_passed = false;
  }

  /**
   * @brief 标记观测源可用并设置本层质量检查结果
   * @param[in] source 观测源
   * @param[in] quality_passed 原始数据是否通过本层质量筛选（如 GNSS fix>=3、
   *            SV>=9、hAcc<6 m）
   * @note quality_passed 为 false 时只置 rejected 标志、不累加 reject_count：
   *       “本帧有观测但暂不允许进 EKF”不属于 EKF 门控拒绝，二者混计会让
   *       拒绝率失去诊断意义。
   */
  void MarkAvailable(const AidSource source, const bool quality_passed)
  {
    AidSourceStatus &entry = status(source);
    entry.available = true;
    entry.quality_passed = quality_passed;
    if (!quality_passed)
    {
      // 质量不通过表示"本帧有观测但暂不允许进 EKF"，不是 EKF 门控拒绝。
      entry.rejected = true;
    }
  }

  /*
   * 记录一次 EKF 量测更新尝试的结果。
   *
   * 适用场景：上层已经调用了 EKF 量测接口并获得详细返回结果。
   * 内部更新 attempt_count、fuse_count 或 reject_count，以及 age_s。
   */
  void RecordAttempt(const AidSource source,
                     const bool fused,
                     const float innovation_norm = 0.0f,
                     const float test_ratio_max = 0.0f)
  {
    AidSourceStatus &entry = status(source);
    entry.available = true;
    entry.quality_passed = true;
    entry.attempted = true;
    entry.fused = fused;
    entry.rejected = !fused;
    entry.innovation_norm = SanitizeNonNegative(innovation_norm);
    entry.test_ratio_max = SanitizeNonNegative(test_ratio_max);
    entry.attempt_count++;
    if (fused)
    {
      entry.fuse_count++;
      entry.age_s = 0.0f;
    }
    else
    {
      entry.reject_count++;
    }
  }

  /*
   * 从 MeasurementUpdateResult 结构体批量记录一次尝试的结果。
   *
   * 若 result.input_valid 为 false 或 result.attempted 为 false，
   * 说明该观测未进入 EKF 更新流程（可能因为输入质量不达标），
   * 只标记为 quality_passed=false，不累加 reject_count。
   */
  void RecordAttempt(const AidSource source,
                     const MeasurementUpdateResult &result)
  {
    if (!result.input_valid || !result.attempted)
    {
      // 输入无效或未进入 EKF：只能判为"质量不允许融合"，不是 EKF 门控拒绝。
      MarkAvailable(source, false);
      return;
    }
    RecordAttempt(source,
                  result.fused,
                  result.innovation_norm,
                  result.test_ratio);
  }

  /*
   * 记录该观测源超时一次。
   *
   * 适用场景：GNSS 失联超过阈值、双天线航向长时间未更新等。
   */
  void RecordTimeout(const AidSource source)
  {
    AidSourceStatus &entry = status(source);
    entry.timed_out = true;
    entry.timeout_count++;
  }

  /*
   * 记录该观测源触发了一次强制状态重置。
   *
   * 适用场景：首次获得可信 GNSS 解时用 ResetPositionVelocityToGnss
   * 重锚定位置和速度，清除累计漂移。
   */
  void RecordReset(const AidSource source)
  {
    AidSourceStatus &entry = status(source);
    entry.reset_count++;
    entry.age_s = 0.0f;
  }

  /*
   * 获取该观测源的安全输出龄期（单位 s）。
   *
   * 返回值：
   * -曾融合过：距上次成功融合的秒数。
   * -从未融合：-1.0f（便于 VOFA 面板区分"勿噪"和"刚失联"）。
   */
  float AgeForOutput(const AidSource source) const
  {
    const float age_s = status(source).age_s;
    return (age_s < FLT_MAX * 0.5f) ? age_s : -1.0f;
  }

private:
  /* 将枚举值映射到数组下标，越界保护兜底为索引 0。 */
  static size_t Index(const AidSource source)
  {
    const size_t idx = static_cast<size_t>(source);
    return (idx < static_cast<size_t>(AidSource::Count)) ? idx : 0U;
  }

  /* 将非正或非有限值规整为 0.0f，避免诊断通道被 NaN/负值污染。 */
  static float SanitizeNonNegative(const float value)
  {
    return (std::isfinite(value) && value > 0.0f) ? value : 0.0f;
  }

  /* 所有观测源的状态数组，下标由 Index() 解析。 */
  AidSourceStatus status_[static_cast<size_t>(AidSource::Count)] = {};
};

} // namespace bfs

#endif // NAVIGATION_SRC_AID_SOURCE_STATUS_H_ NOLINT
