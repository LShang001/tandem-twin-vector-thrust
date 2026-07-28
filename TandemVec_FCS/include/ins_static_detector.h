#pragma once

#include "Eigen/Dense"
#include <cmath>
#include <cstdint>

/*
 * 静止检测器 —— 状态机 + 置信度评分
 * ===================================
 *
 * 把 isStaticDeskSample() 从主程序抽离为独立可测模块：
 * 1. 阈值检查：加速度模长是否接近重力、角速度模长是否接近零、
 *    相邻帧加速度/角速度变化是否足够小。
 * 2. 迟滞状态机：需连续满足 enter_min_frames 帧才"确认静止"；
 *    连续不满足 exit_min_frames 帧才"退出静止"。
 * 3. 置信度评分：考虑加速度偏差、角速度模长、相邻帧变化量、驻留时长，
 *    输出 [0,1] 置信度供上层 Icm45686SelectStaticAidProfile 做辅助强度调度。
 *
 * 坐标系：机体系 FRD，比力单位 m/s²，角速度单位 rad/s。
 * 无 Arduino / ESP-IDF 依赖，可用 g++ 直接编译主机回归测试。
 */

struct Icm45686StaticDetector {
  // ---- 常量配置（setup 阶段一次性写入）----

  // 静止候选阈值（运行期使用预计算的平方值，见 ConfigureThresholds）
  float accel_norm_lower_sq_mps4 = 0.0f;
  float accel_norm_upper_sq_mps4 = 0.0f;
  float gyro_norm_thresh_sq = 0.0f;
  float accel_delta_thresh_sq = 0.0f;
  float gyro_delta_thresh_sq = 0.0f;

  // 置信度评分所用的一阶阈值（运行期不再补平方）
  float accel_norm_tolerance_mps2 = 0.45f;
  float gyro_norm_threshold_radps = 0.035f;
  float accel_delta_threshold_mps2 = 0.18f;
  float gyro_delta_threshold_radps = 0.012f;

  // 迟滞帧数
  uint32_t enter_min_frames = 10;
  uint32_t exit_min_frames = 10;

  // 本地参考重力（用于加速度模长偏差计算）
  float gravity_mps2 = 9.80665f;

  // ---- 运行期可变状态（Update 每帧修改）----

  Eigen::Vector3f last_accel_body_mps2 = Eigen::Vector3f::Zero();
  Eigen::Vector3f last_gyro_body_radps = Eigen::Vector3f::Zero();

  // 最近一帧的观测指标（上层可读取用于诊断/VOFA）
  float accel_norm_error_mps2 = 0.0f;
  float gyro_norm_radps = 0.0f;
  float accel_delta_mps2 = 0.0f;
  float gyro_delta_radps = 0.0f;

  // 输出
  float confidence = 0.0f;
  bool confirmed_static = false;
  uint32_t confirmed_static_frames = 0;

  bool initialized = false;

  // 内部迟滞计数器（公开以便测试验证，上层不应直接修改）
  uint32_t enter_candidate_frames = 0;
  uint32_t exit_candidate_frames = 0;
};

/*
 * 一次性配置阈值（平方值），避免每帧重复计算 sqrt。
 * 需在 Update() 第一次调用前或在 setup() 阶段执行。
 */
inline void Icm45686StaticDetectorConfigureThresholds(
    Icm45686StaticDetector &detector,
    const float accel_norm_lower_mps2,
    const float accel_norm_upper_mps2,
    const float gyro_norm_threshold_radps,
    const float accel_delta_threshold_mps2,
    const float gyro_delta_threshold_radps)
{
  detector.accel_norm_lower_sq_mps4 =
      accel_norm_lower_mps2 * accel_norm_lower_mps2;
  detector.accel_norm_upper_sq_mps4 =
      accel_norm_upper_mps2 * accel_norm_upper_mps2;
  detector.gyro_norm_thresh_sq =
      gyro_norm_threshold_radps * gyro_norm_threshold_radps;
  detector.accel_delta_thresh_sq =
      accel_delta_threshold_mps2 * accel_delta_threshold_mps2;
  detector.gyro_delta_thresh_sq =
      gyro_delta_threshold_radps * gyro_delta_threshold_radps;

  detector.accel_norm_tolerance_mps2 = accel_norm_upper_mps2 - detector.gravity_mps2;
  detector.gyro_norm_threshold_radps = gyro_norm_threshold_radps;
  detector.accel_delta_threshold_mps2 = accel_delta_threshold_mps2;
  detector.gyro_delta_threshold_radps = gyro_delta_threshold_radps;
}

/*
 * 解除静止状态（GNSS 重锚定等需要丢弃旧静止历史时调用）。
 */
inline void Icm45686StaticDetectorReset(Icm45686StaticDetector &detector)
{
  detector.confirmed_static = false;
  detector.confirmed_static_frames = 0;
  detector.enter_candidate_frames = 0;
  detector.exit_candidate_frames = 0;
  detector.confidence = 0.0f;
}

/*
 * 每帧调用一次：阈值检查 → 迟滞状态机 → 置信度评分。
 *
 * @return 当前是否处于"确认静止"状态（等同于原 isStaticDeskSample 返回值）。
 */
inline bool Icm45686StaticDetectorUpdate(
    const Eigen::Vector3f &accel_body_mps2,
    const Eigen::Vector3f &gyro_body_radps,
    Icm45686StaticDetector &detector)
{
  // --------------- 阈值检查 ---------------
  const float accel_norm_sq = accel_body_mps2.squaredNorm();
  const float gyro_norm_sq = gyro_body_radps.squaredNorm();
  const float accel_norm = std::sqrt(accel_norm_sq);
  const float gyro_norm = std::sqrt(gyro_norm_sq);

  const bool accel_norm_ok =
      (accel_norm_sq >= detector.accel_norm_lower_sq_mps4) &&
      (accel_norm_sq <= detector.accel_norm_upper_sq_mps4);
  const bool gyro_norm_ok =
      gyro_norm_sq < detector.gyro_norm_thresh_sq;

  bool accel_delta_ok = true;
  bool gyro_delta_ok = true;
  float accel_delta_mps2 = 0.0f;
  float gyro_delta_radps = 0.0f;
  if (detector.initialized)
  {
    accel_delta_mps2 =
        (accel_body_mps2 - detector.last_accel_body_mps2).norm();
    gyro_delta_radps =
        (gyro_body_radps - detector.last_gyro_body_radps).norm();
    accel_delta_ok =
        (accel_delta_mps2 * accel_delta_mps2) <
        detector.accel_delta_thresh_sq;
    gyro_delta_ok =
        (gyro_delta_radps * gyro_delta_radps) <
        detector.gyro_delta_thresh_sq;
  }

  // 保存最近帧诊断信息
  detector.accel_norm_error_mps2 =
      std::fabs(accel_norm - detector.gravity_mps2);
  detector.gyro_norm_radps = gyro_norm;
  detector.accel_delta_mps2 = accel_delta_mps2;
  detector.gyro_delta_radps = gyro_delta_radps;

  detector.last_accel_body_mps2 = accel_body_mps2;
  detector.last_gyro_body_radps = gyro_body_radps;
  detector.initialized = true;

  // --------------- 迟滞状态机 ---------------
  const bool instant_static_candidate =
      accel_norm_ok && gyro_norm_ok && accel_delta_ok && gyro_delta_ok;

  if (instant_static_candidate)
  {
    detector.exit_candidate_frames = 0;
    if (!detector.confirmed_static)
    {
      if (detector.enter_candidate_frames < UINT32_MAX)
      {
        detector.enter_candidate_frames++;
      }
      if (detector.enter_candidate_frames >= detector.enter_min_frames)
      {
        detector.confirmed_static = true;
        detector.confirmed_static_frames =
            static_cast<uint32_t>(detector.enter_candidate_frames);
      }
    }
    else if (detector.confirmed_static_frames < UINT32_MAX)
    {
      detector.confirmed_static_frames++;
    }
  }
  else
  {
    detector.enter_candidate_frames = 0;
    if (detector.confirmed_static)
    {
      if (detector.exit_candidate_frames < UINT32_MAX)
      {
        detector.exit_candidate_frames++;
      }
      if (detector.exit_candidate_frames >= detector.exit_min_frames)
      {
        detector.confirmed_static = false;
        detector.confirmed_static_frames = 0;
        detector.exit_candidate_frames = 0;
      }
    }
    else
    {
      detector.confirmed_static_frames = 0;
      detector.exit_candidate_frames = 0;
    }
  }

  // --------------- 置信度评分 ---------------
  const float accel_score =
      (detector.accel_norm_error_mps2 >= detector.accel_norm_tolerance_mps2)
          ? 0.0f
          : (1.0f - detector.accel_norm_error_mps2 /
                         detector.accel_norm_tolerance_mps2);
  const float gyro_score =
      (detector.gyro_norm_radps >= detector.gyro_norm_threshold_radps)
          ? 0.0f
          : (1.0f - detector.gyro_norm_radps /
                         detector.gyro_norm_threshold_radps);
  const float accel_delta_score =
      detector.initialized
          ? ((detector.accel_delta_mps2 >= detector.accel_delta_threshold_mps2)
                 ? 0.0f
                 : (1.0f - detector.accel_delta_mps2 /
                                detector.accel_delta_threshold_mps2))
          : 1.0f;
  const float gyro_delta_score =
      detector.initialized
          ? ((detector.gyro_delta_radps >= detector.gyro_delta_threshold_radps)
                 ? 0.0f
                 : (1.0f - detector.gyro_delta_radps /
                                detector.gyro_delta_threshold_radps))
          : 1.0f;

  const float raw_confidence = [&]() {
    float r = accel_score;
    if (gyro_score < r) r = gyro_score;
    if (accel_delta_score < r) r = accel_delta_score;
    if (gyro_delta_score < r) r = gyro_delta_score;
    return r;
  }();

  // 原 120 帧(600ms)爬升改为 60 帧(300ms)：让置信度更快满，
  // 从而档位 2→3 过渡更早，速度残差更快被收紧约束。
  const float dwell_score =
      detector.confirmed_static
          ? ((static_cast<float>(detector.confirmed_static_frames) >= 60.0f)
                 ? 1.0f
                 : static_cast<float>(detector.confirmed_static_frames) /
                       60.0f)
          : 0.0f;

  detector.confidence = detector.confirmed_static
                            ? (raw_confidence < dwell_score ? raw_confidence
                                                             : dwell_score)
                            : 0.0f;

  return detector.confirmed_static;
}
