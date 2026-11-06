#include <cassert>
#include <cmath>
#include <fstream>  // 写入场景级测试报告，便于归档本次仿真结果。
#include <iostream> // 同步输出场景结果，便于命令行直接观察进度。
#include <limits>   // 构造NaN/Inf异常输入，验证EKF底层数值防御。

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EKF_HOST_REGRESSION 1
#include "navigation.h"

static constexpr double kPi = 3.14159265358979323846;
static constexpr float kLocalGravity = 9.79f;
static constexpr float kMaxAhrsYawCorrectionStepRad = 0.0025f;
static constexpr int kGnssNavDataTimeoutMs = 300;
static constexpr int kUbxFix2dForTest = 2;
static constexpr int kUbxFix3dForTest = 3;
static constexpr int kGnssNavDropoutHoldMs = 1000;
static constexpr float kAhrsYawNoiseWithGnssRad = 3.0f;
static constexpr float kAhrsYawNoiseWithoutGnssRad = 0.35f;
static constexpr float kEkfGnssDelayS = BFS_NAVIGATION_GNSS_DELAY_S;

struct HighRateQuatPropagationMetrics
{
  // 记录纯惯导姿态传播相对精确指数映射的角距离，用于评估底层捷联积分误差。
  float final_quat_error_rad = 0.0f;
  float max_quat_error_rad = 0.0f;
  float total_rotation_rad = 0.0f;
};

struct AttitudeInjectionMetrics
{
  // 记录卡尔曼误差状态注入前后的倾角误差，验证较大姿态修正不会被小角度近似低估。
  float tilt_before_update_rad = 0.0f;
  float tilt_after_update_rad = 0.0f;
  float single_update_correction_rad = 0.0f;
};

struct CompositeAttitudeMeasurementMetrics
{
  // 记录yaw/pitch/roll复合误差下，AHRS姿态量测能否按四元数误差方向正确收敛。
  float quat_error_before_rad = 0.0f;
  float quat_error_after_rad = 0.0f;
  float correction_rad = 0.0f;
};

struct CovarianceProcessNoiseMetrics
{
  // 记录过程噪声压力场景的输入强度，便于确认PSD检查覆盖了高动态传播边界。
  int propagated_steps = 0;
  float dt_s = 0.0f;
  float max_gyro_norm_radps = 0.0f;
  float max_accel_norm_mps2 = 0.0f;
};

struct StrapdownIntegrationAccuracyMetrics
{
  // 记录旋转比力下的速度/位置积分误差，直接评估捷联状态传播精度。
  float final_velocity_error_mps = 0.0f;
  float max_velocity_error_mps = 0.0f;
  float final_position_error_m = 0.0f;
  float max_position_error_m = 0.0f;
};

struct CovarianceDiscretizationAccuracyMetrics
{
  // 记录低频协方差传播相对高频子步参考的差异，直接评估误差状态离散化精度。
  float relative_trace_error = 0.0f;
  float relative_max_coeff_error = 0.0f;
  float low_rate_min_eigenvalue = 0.0f;
  float high_rate_min_eigenvalue = 0.0f;
};

struct GravityGradientModelMetrics
{
  // 记录重力梯度在误差状态F矩阵中的离散耦合位置和符号，防止D位置/速度通道写错。
  float phi_vel_down_from_pos_down = 0.0f;
  float phi_pos_down_from_pos_down = 0.0f;
  float expected_phi_vel_down_from_pos_down = 0.0f;
};

struct Wgs84GravityMechanizationMetrics
{
  // 记录不同纬度/高度下的静止垂向漂移，验证惯导传播使用WGS84正常重力而不是固定常数。
  float normal_gravity_mps2 = 0.0f;
  float fixed_gravity_mismatch_mps2 = 0.0f;
  float final_down_velocity_mps = 0.0f;
  float max_speed_mps = 0.0f;
};

struct ConingCompensationMetrics
{
  // 记录交替角增量下相对流式圆锥补偿参考的姿态误差。
  float final_coning_error_rad = 0.0f;
  float max_coning_error_rad = 0.0f;
  float accumulated_nominal_rotation_rad = 0.0f;
};

struct ScullingCompensationMetrics
{
  // 记录交替角增量和速度增量耦合下相对流式划摇补偿参考的速度误差。
  float final_velocity_error_mps = 0.0f;
  float max_velocity_error_mps = 0.0f;
  float final_position_error_m = 0.0f;
  float max_position_error_m = 0.0f;
};

struct EarthRateCoriolisMetrics
{
  // 记录高速高纬纯惯导传播中，地球自转/输送率补偿带来的速度和位置差异。
  float final_velocity_error_mps = 0.0f;
  float max_velocity_error_mps = 0.0f;
  float final_position_error_m = 0.0f;
  float max_position_error_m = 0.0f;
  float expected_coriolis_speed_change_mps = 0.0f;
};

struct NavigationFrameRotationMetrics
{
  // 记录高速/高纬下导航系相对惯性系转动对姿态传播的影响。
  float final_quat_error_rad = 0.0f;
  float max_quat_error_rad = 0.0f;
  float expected_nav_frame_rotation_rad = 0.0f;
  float min_covariance_eigenvalue = 0.0f;
  float max_covariance_trace = 0.0f;
  int covariance_failure_step = -1;
};

struct StaticEarthRateInitializationMetrics
{
  // 记录静止实机陀螺包含地球自转时，初始化零偏估计是否会把地球自转误扣掉。
  float final_quat_drift_rad = 0.0f;
  float max_quat_drift_rad = 0.0f;
  float expected_earth_rotation_rad = 0.0f;
  float initial_gyro_bias_norm_radps = 0.0f;
  float physical_earth_rate_norm_radps = 0.0f;
};

struct StaticZLockEarthRateMetrics
{
  // 记录飞控接入层静止锁Z轴陀螺时，是否仍保留物理地球自转的Z轴分量。
  float final_quat_drift_rad = 0.0f;
  float max_quat_drift_rad = 0.0f;
  float expected_removed_earth_rate_z_rad = 0.0f;
  float locked_gyro_z_radps = 0.0f;
  float physical_gyro_z_radps = 0.0f;
};

struct TwoSampleMechanizationMetrics
{
  // 记录真正双子样增量接口相对双子样参考的姿态、速度和位置误差。
  float final_quat_error_rad = 0.0f;
  float max_quat_error_rad = 0.0f;
  float final_velocity_error_mps = 0.0f;
  float max_velocity_error_mps = 0.0f;
  float final_position_error_m = 0.0f;
  float max_position_error_m = 0.0f;
  float average_accel_velocity_error_mps = 0.0f;
};

struct TwoSampleDelayedReplayMetrics
{
  // 记录双子样历史增量在延迟GNSS回放后的修正效果和连续性，防止回放时重复或漏扣零偏。
  float quat_jump_after_gnss_rad = 0.0f;
  float velocity_jump_after_gnss_mps = 0.0f;
  float position_jump_after_gnss_m = 0.0f;
  float velocity_error_before_gnss_mps = 0.0f;
  float velocity_error_after_gnss_mps = 0.0f;
  float position_error_before_gnss_m = 0.0f;
  float position_error_after_gnss_m = 0.0f;
  float attitude_error_before_gnss_rad = 0.0f;
  float attitude_error_after_gnss_rad = 0.0f;
};

struct NavigationInputSplitMetrics
{
  // 记录飞控接入层把多个IMU样本按导航周期中点拆成两个子样后的积分误差。
  float first_half_dt_s = 0.0f;
  float second_half_dt_s = 0.0f;
  float theta_split_error_rad = 0.0f;
  float delta_v_split_error_mps = 0.0f;
  float boundary_cross_fraction = 0.0f;
};

// 测试用GNSS新鲜度判断：复现主程序中“只有最近收到过GNSS数据才可参与导航”的判定。
static bool isGnssDataFreshForNavForTest(const int now_ms, const int last_gnss_data_ms)
{
  return (last_gnss_data_ms > 0) &&
         ((now_ms - last_gnss_data_ms) >= 0) &&
         ((now_ms - last_gnss_data_ms) <= kGnssNavDataTimeoutMs);
}

// 测试用GNSS有效性判断：当前实现只看fix，场景34会先红灯证明缺少新鲜度门控。
static bool isGnssFixValidForNavForTest(const int ubx_fix, const bool gnss_data_fresh)
{
  return (ubx_fix >= kUbxFix3dForTest) && gnss_data_fresh;
}

// 测试用GNSS输出保持状态机：当前实现等价于瞬时valid，场景35会先红灯证明需要迟滞。
static bool updateGnssNavOutputValidForTest(const bool instantaneous_valid,
                                            const int now_ms,
                                            int *last_valid_ms)
{
  if (instantaneous_valid)
  {
    *last_valid_ms = now_ms;
    return true;
  }
  return (*last_valid_ms > 0) && ((now_ms - *last_valid_ms) <= kGnssNavDropoutHoldMs);
}

// 测试用AHRS航向量测噪声选择：当前实现只看fix，场景36会先红灯证明需要使用新鲜GNSS状态。
static float selectAhrsYawNoiseForTest(const int ubx_fix, const bool gnss_instant_valid)
{
  (void)ubx_fix;
  return gnss_instant_valid ? kAhrsYawNoiseWithGnssRad : kAhrsYawNoiseWithoutGnssRad;
}

// 与 src/main.cpp 的 AHRS 融合门控保持一致，并额外检查平动加速度，避免机动时把加表当重力。
static bool shouldFuseAhrsAttitude(const Eigen::Vector3f &accel_mps2,
                                   const Eigen::Vector3f &gyro_radps,
                                   const Eigen::Quaternionf &ekf_quat,
                                   const bool static_confirmed = false)
{
  const float accel_norm_error = std::fabs(accel_mps2.norm() - kLocalGravity);
  const bool accel_gravity_trusted = (accel_norm_error < 2.0f);
  const bool angular_rate_quiet = (gyro_radps.norm() < 1.2f);
  const Eigen::Vector3f accel_ned =
      bfs::quat2dcm(ekf_quat).transpose() * accel_mps2 +
      (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();
  const float body_lateral_accel =
      std::sqrt(accel_mps2(0) * accel_mps2(0) + accel_mps2(1) * accel_mps2(1));
  const bool static_attitude_recovery_allowed = static_confirmed && (body_lateral_accel < 1.0f);
  const bool translational_accel_quiet =
      static_attitude_recovery_allowed || (accel_ned.norm() < 1.5f);
  return accel_gravity_trusted && angular_rate_quiet && translational_accel_quiet;
}

// 测试用线性插值：复现主程序中根据加速度可信度调整姿态量测噪声的逻辑。
static float linearInterpolateForTest(float x, float x0, float x1, float y0, float y1)
{
  if (x <= x0)
    return y0;
  if (x >= x1)
    return y1;
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

// 真值轨迹积分：用与EKF相同的地球模型推进LLA，减少测试模型不一致带来的误差。
static Eigen::Vector3d integrateTruthLla(const Eigen::Vector3d &lla,
                                         const Eigen::Vector3f &vel_ned,
                                         float dt_s)
{
  return lla + (dt_s * bfs::llarate(vel_ned.cast<double>(), lla, bfs::AngPosUnit::RAD)).cast<double>();
}

// 确定性扰动源：不用随机数，保证主机回归在不同机器上结果稳定。
static float deterministicNoise(int sample, float amplitude, float phase)
{
  return amplitude * std::sin(0.017f * static_cast<float>(sample) + phase) +
         0.5f * amplitude * std::sin(0.071f * static_cast<float>(sample) + 0.3f * phase);
}

// 测试用角度环绕：复现主程序中航向差值限制前的[-pi, pi]归一化逻辑。
static float wrapAnglePiForTest(float angle_rad)
{
  while (angle_rad > static_cast<float>(kPi))
  {
    angle_rad -= 2.0f * static_cast<float>(kPi);
  }
  while (angle_rad < -static_cast<float>(kPi))
  {
    angle_rad += 2.0f * static_cast<float>(kPi);
  }
  return angle_rad;
}

// 测试用角度环绕：复现主程序对输出航向[0, 2pi)的处理。
static float wrapAngleTwoPiForTest(float angle_rad)
{
  while (angle_rad < 0.0f)
  {
    angle_rad += 2.0f * static_cast<float>(kPi);
  }
  while (angle_rad >= 2.0f * static_cast<float>(kPi))
  {
    angle_rad -= 2.0f * static_cast<float>(kPi);
  }
  return angle_rad;
}

// 测试用限幅：避免依赖Arduino constrain宏，让主机仿真保持普通C++可编译。
static float clampForTest(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

// 测试用WGS84正常重力模型：与参考项目gravity.m/PSINS ethupdate.m保持同类公式。
static float normalGravityWgs84ForTest(const Eigen::Vector3d &lla_rad_m)
{
  const double lat_rad = lla_rad_m(0);
  const double alt_m = lla_rad_m(2);
  const double sin_lat = std::sin(lat_rad);
  const double sin_lat2 = sin_lat * sin_lat;
  const double sin_2lat = std::sin(2.0 * lat_rad);
  const double g0 = 9.780318 * (1.0 + 5.3024e-3 * sin_lat2 -
                                5.9e-6 * sin_2lat * sin_2lat);
  const double rn = bfs::earthrad_transverse_m(lat_rad, bfs::AngPosUnit::RAD);
  const double rm = bfs::earthrad_meridonal_m(lat_rad, bfs::AngPosUnit::RAD);
  const double r0 = std::sqrt(rn * rm);
  if (alt_m >= 0.0)
  {
    const double scale = 1.0 + alt_m / r0;
    return static_cast<float>(g0 / (scale * scale));
  }
  return static_cast<float>(g0 * (1.0 + alt_m / r0));
}

// 角度距离：用于检查EKF输出切到后台AHRS输出时不会出现跨0/2pi跳变。
static float yawDistanceRad(float a_rad, float b_rad)
{
  return std::fabs(wrapAnglePiForTest(a_rad - b_rad));
}

// 精确四元数增量：测试基准使用指数映射，避免把被测EKF的小角度近似也写进真值。
static Eigen::Quaternionf exactGyroDeltaQuatForTest(const Eigen::Vector3f &gyro_radps,
                                                    const float dt_s)
{
  const Eigen::Vector3f delta_theta = gyro_radps * dt_s;
  const float angle_rad = delta_theta.norm();
  Eigen::Quaternionf delta_quat;
  if (angle_rad < 1.0e-6f)
  {
    // 极小角度下用一阶形式避免除零；该分支只作为数值保护。
    delta_quat.w() = 1.0f;
    delta_quat.x() = 0.5f * delta_theta(0);
    delta_quat.y() = 0.5f * delta_theta(1);
    delta_quat.z() = 0.5f * delta_theta(2);
  }
  else
  {
    const float half_angle_rad = 0.5f * angle_rad;
    const float vec_scale = std::sin(half_angle_rad) / angle_rad;
    delta_quat.w() = std::cos(half_angle_rad);
    delta_quat.x() = vec_scale * delta_theta(0);
    delta_quat.y() = vec_scale * delta_theta(1);
    delta_quat.z() = vec_scale * delta_theta(2);
  }
  return delta_quat.normalized();
}

// 四元数角距离：使用绝对点积消除q和-q的等价符号差异。
static float quatDistanceRadForTest(const Eigen::Quaternionf &a,
                                    const Eigen::Quaternionf &b)
{
  const float dot = std::fabs(a.normalized().dot(b.normalized()));
  const float clamped_dot = clampForTest(dot, -1.0f, 1.0f);
  return 2.0f * std::acos(clamped_dot);
}

// 高频真值捷联积分：用子步中点法积分姿态相关比力，作为低频EKF传播精度的参考。
static void integrateTruthStrapdownSubstepsForTest(const Eigen::Vector3f &accel_body_mps2,
                                                   const Eigen::Vector3f &gyro_body_radps,
                                                   const float dt_s,
                                                   const int substeps,
                                                   Eigen::Quaternionf *truth_quat,
                                                   Eigen::Vector3f *truth_vel_ned,
                                                   Eigen::Vector3d *truth_lla)
{
  const float sub_dt_s = dt_s / static_cast<float>(substeps);
  for (int substep = 0; substep < substeps; ++substep)
  {
    // 子步中点姿态用于旋转比力，比低频终点/起点欧拉积分更接近连续时间真值。
    const Eigen::Quaternionf quat_mid =
        ((*truth_quat) * exactGyroDeltaQuatForTest(gyro_body_radps, 0.5f * sub_dt_s)).normalized();
    const Eigen::Matrix3f t_body_to_ned_mid = bfs::quat2dcm(quat_mid).transpose();
    const Eigen::Vector3f accel_ned_mid =
        t_body_to_ned_mid * accel_body_mps2 +
        (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();

    const Eigen::Vector3f vel_mid = (*truth_vel_ned) + 0.5f * sub_dt_s * accel_ned_mid;
    const Eigen::Vector3d lla_mid =
        (*truth_lla) +
        (0.5f * sub_dt_s *
         bfs::llarate(truth_vel_ned->cast<double>(), *truth_lla, bfs::AngPosUnit::RAD))
            .cast<double>();

    *truth_vel_ned += sub_dt_s * accel_ned_mid;
    *truth_lla +=
        (sub_dt_s * bfs::llarate(vel_mid.cast<double>(), lla_mid, bfs::AngPosUnit::RAD))
            .cast<double>();
    *truth_quat = ((*truth_quat) * exactGyroDeltaQuatForTest(gyro_body_radps, sub_dt_s)).normalized();
    if (truth_quat->w() < 0.0f)
    {
      *truth_quat = Eigen::Quaternionf(-truth_quat->w(), -truth_quat->x(),
                                       -truth_quat->y(), -truth_quat->z());
    }
  }
}

// 流式圆锥/划摇参考：对应当前飞控只给EKF传入导航周期平均IMU的接口形态。
static void integrateStreamingConingScullingReferenceForTest(
    const Eigen::Vector3f &accel_body_mps2,
    const Eigen::Vector3f &gyro_body_radps,
    const float dt_s,
    Eigen::Vector3f *previous_delta_theta,
    Eigen::Vector3f *previous_delta_v,
    Eigen::Quaternionf *truth_quat,
    Eigen::Vector3f *truth_vel_ned,
    Eigen::Vector3d *truth_lla)
{
  const Eigen::Vector3f delta_theta = gyro_body_radps * dt_s;
  const Eigen::Vector3f delta_v = accel_body_mps2 * dt_s;
  const Eigen::Vector3f coning_delta_theta =
      delta_theta + (1.0f / 12.0f) * previous_delta_theta->cross(delta_theta);
  const Eigen::Vector3f sculling_delta_v =
      delta_v + 0.5f * delta_theta.cross(delta_v) +
      (1.0f / 12.0f) *
          (previous_delta_theta->cross(delta_v) + previous_delta_v->cross(delta_theta));

  const Eigen::Matrix3f t_body_to_ned_prev = bfs::quat2dcm(*truth_quat).transpose();
  const Eigen::Vector3d lla_mid =
      (*truth_lla) +
      (0.5f * dt_s *
       bfs::llarate(truth_vel_ned->cast<double>(), *truth_lla, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Vector3f gravity_delta_v_ned =
      (Eigen::Vector3f() << 0.0f, 0.0f,
       static_cast<float>(normalGravityWgs84ForTest(lla_mid)) * dt_s)
          .finished();
  const Eigen::Vector3f corrected_delta_v_ned =
      t_body_to_ned_prev * sculling_delta_v + gravity_delta_v_ned;
  const Eigen::Vector3f vel_mid = (*truth_vel_ned) + 0.5f * corrected_delta_v_ned;

  *truth_vel_ned += corrected_delta_v_ned;
  *truth_lla +=
      (dt_s * bfs::llarate(vel_mid.cast<double>(), lla_mid, bfs::AngPosUnit::RAD))
          .cast<double>();
  *truth_quat = ((*truth_quat) * exactGyroDeltaQuatForTest(coning_delta_theta / dt_s,
                                                           dt_s))
                    .normalized();
  if (truth_quat->w() < 0.0f)
  {
    *truth_quat = Eigen::Quaternionf(-truth_quat->w(), -truth_quat->x(),
                                     -truth_quat->y(), -truth_quat->z());
  }

  *previous_delta_theta = delta_theta;
  *previous_delta_v = delta_v;
}

// 带地球自转、输送率和导航系转动补偿的捷联参考。
static void integrateEarthRateReferenceForTest(const Eigen::Vector3f &accel_body_mps2,
                                               const Eigen::Vector3f &gyro_body_radps,
                                               const float dt_s,
                                               Eigen::Vector3f *previous_delta_theta,
                                               Eigen::Vector3f *previous_delta_v,
                                               Eigen::Quaternionf *truth_quat,
                                               Eigen::Vector3f *truth_vel_ned,
                                               Eigen::Vector3d *truth_lla)
{
  const Eigen::Vector3f delta_theta = gyro_body_radps * dt_s;
  const Eigen::Vector3f delta_v = accel_body_mps2 * dt_s;
  const Eigen::Vector3f coning_delta_theta =
      delta_theta + (1.0f / 12.0f) * previous_delta_theta->cross(delta_theta);
  const Eigen::Vector3f sculling_delta_v =
      delta_v + 0.5f * delta_theta.cross(delta_v) +
      (1.0f / 12.0f) *
          (previous_delta_theta->cross(delta_v) + previous_delta_v->cross(delta_theta));

  const Eigen::Vector3d lla_mid =
      (*truth_lla) +
      (0.5f * dt_s *
       bfs::llarate(truth_vel_ned->cast<double>(), *truth_lla, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Vector3f vel_mid_for_rates = *truth_vel_ned;
  const Eigen::Vector3f earth_rate_ned =
      bfs::earthrate(lla_mid(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f nav_rate_ned =
      bfs::navrate(vel_mid_for_rates.cast<double>(), lla_mid, bfs::AngPosUnit::RAD)
          .cast<float>();
  const Eigen::Vector3f nav_frame_rate_ned = earth_rate_ned + nav_rate_ned;

  const Eigen::Matrix3f t_body_to_ned_prev = bfs::quat2dcm(*truth_quat).transpose();
  const Eigen::Vector3f coriolis_accel_ned =
      -((2.0f * earth_rate_ned + nav_rate_ned).cross(vel_mid_for_rates));
  const Eigen::Vector3f gravity_coriolis_delta_v_ned =
      ((Eigen::Vector3f() << 0.0f, 0.0f,
        static_cast<float>(normalGravityWgs84ForTest(lla_mid)))
           .finished() +
       coriolis_accel_ned) *
      dt_s;
  const Eigen::Vector3f corrected_delta_v_ned =
      t_body_to_ned_prev * sculling_delta_v + gravity_coriolis_delta_v_ned;
  const Eigen::Vector3f vel_mid = (*truth_vel_ned) + 0.5f * corrected_delta_v_ned;

  *truth_vel_ned += corrected_delta_v_ned;
  *truth_lla +=
      (dt_s * bfs::llarate(vel_mid.cast<double>(), lla_mid, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Quaternionf q_nav_delta =
      exactGyroDeltaQuatForTest(-nav_frame_rate_ned, dt_s);
  const Eigen::Quaternionf q_body_delta =
      exactGyroDeltaQuatForTest(coning_delta_theta / dt_s, dt_s);
  *truth_quat = (q_nav_delta * (*truth_quat) * q_body_delta).normalized();
  if (truth_quat->w() < 0.0f)
  {
    *truth_quat = Eigen::Quaternionf(-truth_quat->w(), -truth_quat->x(),
                                     -truth_quat->y(), -truth_quat->z());
  }

  *previous_delta_theta = delta_theta;
  *previous_delta_v = delta_v;
}

// 测试用飞控接入层双子样拆分：复现 main.cpp 中按真实dt累计并跨中点按比例拆分的逻辑。
static void splitImuDeltasLikeMainForTest(const Eigen::Vector3f delta_theta_samples[],
                                          const Eigen::Vector3f delta_v_samples[],
                                          const float dt_samples[],
                                          const int sample_count,
                                          Eigen::Vector3f *delta_theta_1,
                                          Eigen::Vector3f *delta_theta_2,
                                          Eigen::Vector3f *delta_v_1,
                                          Eigen::Vector3f *delta_v_2,
                                          float *first_half_dt_s,
                                          float *second_half_dt_s,
                                          float *boundary_cross_fraction)
{
  *delta_theta_1 = Eigen::Vector3f::Zero();
  *delta_theta_2 = Eigen::Vector3f::Zero();
  *delta_v_1 = Eigen::Vector3f::Zero();
  *delta_v_2 = Eigen::Vector3f::Zero();
  *first_half_dt_s = 0.0f;
  *second_half_dt_s = 0.0f;
  *boundary_cross_fraction = 0.0f;

  float total_dt_s = 0.0f;
  for (int idx = 0; idx < sample_count; ++idx)
  {
    total_dt_s += dt_samples[idx];
  }
  const float first_sample_target_dt_s = 0.5f * total_dt_s;
  float accumulated_delta_time_s = 0.0f;
  for (int idx = 0; idx < sample_count; ++idx)
  {
    const float sample_dt_s = dt_samples[idx];
    if (sample_dt_s <= 0.0f)
    {
      continue;
    }
    const float next_accumulated_delta_time_s = accumulated_delta_time_s + sample_dt_s;
    if (next_accumulated_delta_time_s <= first_sample_target_dt_s)
    {
      *delta_theta_1 += delta_theta_samples[idx];
      *delta_v_1 += delta_v_samples[idx];
      *first_half_dt_s += sample_dt_s;
    }
    else if (accumulated_delta_time_s >= first_sample_target_dt_s)
    {
      *delta_theta_2 += delta_theta_samples[idx];
      *delta_v_2 += delta_v_samples[idx];
      *second_half_dt_s += sample_dt_s;
    }
    else
    {
      // 跨越导航周期中点的样本必须按时间比例拆开，否则两个子样的物理半周期会错位。
      const float first_fraction =
          (first_sample_target_dt_s - accumulated_delta_time_s) / sample_dt_s;
      const float clamped_first_fraction =
          std::min(1.0f, std::max(0.0f, first_fraction));
      *delta_theta_1 += clamped_first_fraction * delta_theta_samples[idx];
      *delta_v_1 += clamped_first_fraction * delta_v_samples[idx];
      *delta_theta_2 += (1.0f - clamped_first_fraction) * delta_theta_samples[idx];
      *delta_v_2 += (1.0f - clamped_first_fraction) * delta_v_samples[idx];
      *first_half_dt_s += clamped_first_fraction * sample_dt_s;
      *second_half_dt_s += (1.0f - clamped_first_fraction) * sample_dt_s;
      *boundary_cross_fraction = clamped_first_fraction;
    }
    accumulated_delta_time_s = next_accumulated_delta_time_s;
  }
}

// 真正双子样参考：同一导航周期内使用两个角增量/速度增量，不依赖上一周期伪补偿。
static void integrateTwoSampleReferenceForTest(const Eigen::Vector3f &delta_theta_1,
                                               const Eigen::Vector3f &delta_theta_2,
                                               const Eigen::Vector3f &delta_v_1,
                                               const Eigen::Vector3f &delta_v_2,
                                               const float dt_s,
                                               Eigen::Quaternionf *truth_quat,
                                               Eigen::Vector3f *truth_vel_ned,
                                               Eigen::Vector3d *truth_lla)
{
  const Eigen::Vector3f coning_delta_theta =
      delta_theta_1 + delta_theta_2 +
      (2.0f / 3.0f) * delta_theta_1.cross(delta_theta_2);
  const Eigen::Vector3f sculling_delta_v =
      delta_v_1 + delta_v_2 +
      0.5f * (delta_theta_1 + delta_theta_2).cross(delta_v_1 + delta_v_2) +
      (2.0f / 3.0f) *
          (delta_theta_1.cross(delta_v_2) + delta_v_1.cross(delta_theta_2));

  const Eigen::Matrix3f t_body_to_ned_prev = bfs::quat2dcm(*truth_quat).transpose();
  const Eigen::Vector3d lla_mid =
      (*truth_lla) +
      (0.5f * dt_s *
       bfs::llarate(truth_vel_ned->cast<double>(), *truth_lla, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Vector3f earth_rate_ned =
      bfs::earthrate(lla_mid(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f nav_rate_ned =
      bfs::navrate(truth_vel_ned->cast<double>(), lla_mid, bfs::AngPosUnit::RAD)
          .cast<float>();
  const Eigen::Vector3f nav_frame_rate_ned = earth_rate_ned + nav_rate_ned;
  const Eigen::Vector3f coriolis_accel_ned =
      -((2.0f * earth_rate_ned + nav_rate_ned).cross(*truth_vel_ned));
  const Eigen::Vector3f gravity_coriolis_delta_v_ned =
      ((Eigen::Vector3f() << 0.0f, 0.0f,
        static_cast<float>(normalGravityWgs84ForTest(lla_mid)))
           .finished() +
       coriolis_accel_ned) *
      dt_s;
  const Eigen::Vector3f corrected_delta_v_ned =
      t_body_to_ned_prev * sculling_delta_v + gravity_coriolis_delta_v_ned;
  const Eigen::Vector3f vel_mid = (*truth_vel_ned) + 0.5f * corrected_delta_v_ned;

  *truth_vel_ned += corrected_delta_v_ned;
  *truth_lla +=
      (dt_s * bfs::llarate(vel_mid.cast<double>(), lla_mid, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Quaternionf q_nav_delta =
      exactGyroDeltaQuatForTest(-nav_frame_rate_ned, dt_s);
  const Eigen::Quaternionf q_body_delta =
      exactGyroDeltaQuatForTest(coning_delta_theta / dt_s, dt_s);
  *truth_quat = (q_nav_delta * (*truth_quat) * q_body_delta).normalized();
  if (truth_quat->w() < 0.0f)
  {
    *truth_quat = Eigen::Quaternionf(-truth_quat->w(), -truth_quat->x(),
                                     -truth_quat->y(), -truth_quat->z());
  }
}

// 带导航系转动补偿的姿态参考：q_b2n = q_nav_rotation * q_b2n * q_body_rotation。
static void integrateNavigationFrameRotationReferenceForTest(
    const Eigen::Vector3f &accel_body_mps2,
    const Eigen::Vector3f &gyro_body_radps,
    const float dt_s,
    Eigen::Quaternionf *truth_quat,
    Eigen::Vector3f *truth_vel_ned,
    Eigen::Vector3d *truth_lla,
    float *accumulated_nav_frame_rotation_rad)
{
  const Eigen::Vector3d lla_mid =
      (*truth_lla) +
      (0.5f * dt_s *
       bfs::llarate(truth_vel_ned->cast<double>(), *truth_lla, bfs::AngPosUnit::RAD))
          .cast<double>();
  const Eigen::Vector3f earth_rate_ned =
      bfs::earthrate(lla_mid(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f nav_rate_ned =
      bfs::navrate(truth_vel_ned->cast<double>(), lla_mid, bfs::AngPosUnit::RAD)
          .cast<float>();
  const Eigen::Vector3f nav_frame_rate_ned = earth_rate_ned + nav_rate_ned;
  const Eigen::Quaternionf q_nav_delta =
      exactGyroDeltaQuatForTest(-nav_frame_rate_ned, dt_s);
  const Eigen::Quaternionf q_body_delta =
      exactGyroDeltaQuatForTest(gyro_body_radps, dt_s);

  const Eigen::Matrix3f t_body_to_ned_prev = bfs::quat2dcm(*truth_quat).transpose();
  const Eigen::Vector3f coriolis_accel_ned =
      -((2.0f * earth_rate_ned + nav_rate_ned).cross(*truth_vel_ned));
  const Eigen::Vector3f gravity_ned =
      (Eigen::Vector3f() << 0.0f, 0.0f,
       static_cast<float>(normalGravityWgs84ForTest(lla_mid)))
          .finished();
  const Eigen::Vector3f corrected_delta_v_ned =
      t_body_to_ned_prev * (accel_body_mps2 * dt_s) +
      (gravity_ned + coriolis_accel_ned) * dt_s;
  const Eigen::Vector3f vel_mid = (*truth_vel_ned) + 0.5f * corrected_delta_v_ned;
  *truth_vel_ned += corrected_delta_v_ned;
  *truth_lla +=
      (dt_s * bfs::llarate(vel_mid.cast<double>(), lla_mid, bfs::AngPosUnit::RAD))
          .cast<double>();
  *truth_quat = (q_nav_delta * (*truth_quat) * q_body_delta).normalized();
  if (truth_quat->w() < 0.0f)
  {
    *truth_quat = Eigen::Quaternionf(-truth_quat->w(), -truth_quat->x(),
                                     -truth_quat->y(), -truth_quat->z());
  }
  *accumulated_nav_frame_rotation_rad += nav_frame_rate_ned.norm() * dt_s;
}

// 通用断言：验证EKF所有核心状态量均为有限值（无NaN/Inf发散）
static void assertFiniteEkfState(const bfs::Ekf15State &ekf)
{
  assert(std::isfinite(ekf.roll_rad()));
  assert(std::isfinite(ekf.pitch_rad()));
  assert(std::isfinite(ekf.yaw_rad()));
  assert(std::isfinite(ekf.ned_vel_mps()(0)));
  assert(std::isfinite(ekf.ned_vel_mps()(1)));
  assert(std::isfinite(ekf.ned_vel_mps()(2)));
  assert(std::isfinite(ekf.lat_rad()));
  assert(std::isfinite(ekf.lon_rad()));
  assert(std::isfinite(ekf.alt_m()));
}

// 通用断言：地理坐标不仅要有限，还必须保持在地球模型可安全计算的范围内。
static void assertPlausibleEkfLla(const bfs::Ekf15State &ekf)
{
  assert(std::fabs(ekf.lat_rad()) < (0.5 * kPi - 1.0e-3));
  assert(std::fabs(ekf.lon_rad()) <= (kPi + 1.0e-3));
  assert(ekf.alt_m() > -1000.0);
  assert(ekf.alt_m() < 100000.0);
}

// 通用断言：状态有限还不够，协方差也必须保持有限、对称、对角线非负且整体正半定。
static void assertHealthyEkfCovariance(const bfs::Ekf15State &ekf)
{
  assert(ekf.covariance_is_healthy());
  assert(ekf.covariance_is_positive_semidefinite());
}

// 测试结果记录：每个场景通过后同时写入报告文件和控制台，便于失败时定位最后通过点。
static void recordScenarioPass(std::ofstream &report, const char *scenario_name)
{
  // 使用std::endl强制刷新，断言失败导致进程中止时也尽量保留已通过场景记录。
  report << "[PASS] " << scenario_name << std::endl;
  std::cout << "[PASS] " << scenario_name << std::endl;
}

// 测试结果记录：输出带指标的场景结果，保留Monte Carlo最差误差用于后续对比。
static void recordScenarioPassWithMetrics(std::ofstream &report,
                                          const char *scenario_name,
                                          float max_pos_err_m,
                                          float max_vel_err_mps,
                                          float max_tilt_err_rad)
{
  // 指标单位固定为m、m/s、rad，便于多次仿真横向比较。
  report << "[PASS] " << scenario_name
         << " | max_pos_err_m=" << max_pos_err_m
         << " max_vel_err_mps=" << max_vel_err_mps
         << " max_tilt_err_rad=" << max_tilt_err_rad << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | max_pos_err_m=" << max_pos_err_m
            << " max_vel_err_mps=" << max_vel_err_mps
            << " max_tilt_err_rad=" << max_tilt_err_rad << std::endl;
}

// 测试结果记录：输出切换连续性指标，便于长期观察EKF/AHRS切换是否出现变差。
static void recordScenarioPassWithSwitchMetric(std::ofstream &report,
                                               const char *scenario_name,
                                               float max_switch_yaw_jump_rad)
{
  report << "[PASS] " << scenario_name
         << " | max_switch_yaw_jump_rad=" << max_switch_yaw_jump_rad << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | max_switch_yaw_jump_rad=" << max_switch_yaw_jump_rad << std::endl;
}

struct GnssReacquisitionMetrics
{
  float max_pos_err_m = 0.0f;
  float max_vel_err_mps = 0.0f;
  float max_tilt_err_rad = 0.0f;
  float max_reacq_jump_m = 0.0f;
  float final_pos_err_m = 0.0f;
  float final_vel_err_mps = 0.0f;
  int gnss_updates = 0;
  int zupt_updates = 0;
};

struct AhrsStaticRecoveryMetrics
{
  int fused_count = 0;
  int skipped_count = 0;
  bool first_static_gate_allows_fusion = false;
  float tilt_before_recovery_rad = 0.0f;
  float tilt_after_recovery_rad = 0.0f;
};

struct AhrsFalseStaticManeuverMetrics
{
  bool false_static_gate_allows_fusion = false;
  float body_lateral_accel_mps2 = 0.0f;
  float tilt_change_after_bad_ahrs_rad = 0.0f;
};

struct GnssFreshnessMetrics
{
  bool fresh_fix_valid = false;
  bool stale_fix_valid = false;
  bool output_uses_ekf_on_stale_fix = false;
  bool yaw_correction_runs_on_stale_fix = false;
};

struct GnssOutputHysteresisMetrics
{
  int switch_count = 0;
  int invalid_gap_max_ms = 0;
  bool output_valid_after_single_bad_frame = false;
  bool output_invalid_after_long_dropout = false;
};

struct AhrsYawNoiseFreshnessMetrics
{
  float fresh_yaw_noise_rad = 0.0f;
  float stale_yaw_noise_rad = 0.0f;
  bool stale_fix_uses_no_gnss_noise = false;
};

struct AhrsYawCorrectionHoldMetrics
{
  bool correction_runs_on_fresh_gnss = false;
  bool correction_runs_during_output_hold = false;
  float correction_delta_on_fresh_gnss_rad = 0.0f;
  float correction_delta_during_output_hold_rad = 0.0f;
};

struct OriginReanchorMetrics
{
  float relative_jump_before_reanchor_m = 0.0f;
  float relative_after_reanchor_m = 0.0f;
  bool degree_origin_synced = false;
};

struct FirstGnssReanchorRelativeTimingMetrics
{
  float stale_relative_before_reanchor_m = 0.0f;
  float relative_after_reanchor_m = 0.0f;
  bool stale_relative_was_published = false;
};

struct StaticDetectionFreshnessMetrics
{
  bool stale_gnss_blocks_static = false;
  bool static_confirmed_after_timeout = false;
  int confirmed_after_ms = 0;
};

struct StatusFixFreshnessMetrics
{
  int fresh_mapped_fix = 0;
  int stale_mapped_fix = 0;
  bool stale_fix_forces_horizontal_gnss_path = false;
};

struct AnoGnssTelemetryFreshnessMetrics
{
  int fresh_fix_sta = 0;
  int stale_fix_sta = 0;
  int stale_num_sat = 0;
  bool stale_velocity_branch_uses_raw_gnss = false;
  bool stale_telemetry_reports_raw_gnss = false;
  bool stale_accuracy_reports_raw_gnss = false;
  float stale_raw_velocity_norm_mps = 0.0f;
  float stale_raw_position_norm_deg = 0.0f;
  float stale_h_acc_m = 0.0f;
  float stale_v_acc_m = 0.0f;
};

struct Gnss3dBoundaryMetrics
{
  bool fix_2d_valid_for_nav = false;
  bool fix_3d_valid_for_nav = false;
  bool dual_vector_yaw_allows_2d = false;
  bool relative_position_allows_2d = false;
};

// 测试结果记录：输出GNSS重捕获场景的最终误差和单帧跳变，便于把仿真结果反向用于门控调参。
static void recordScenarioPassWithReacqMetrics(std::ofstream &report,
                                               const char *scenario_name,
                                               const GnssReacquisitionMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_pos_err_m=" << metrics.final_pos_err_m
         << " final_vel_err_mps=" << metrics.final_vel_err_mps
         << " max_pos_err_m=" << metrics.max_pos_err_m
         << " max_vel_err_mps=" << metrics.max_vel_err_mps
         << " max_tilt_err_rad=" << metrics.max_tilt_err_rad
         << " max_reacq_jump_m=" << metrics.max_reacq_jump_m
         << " gnss_updates=" << metrics.gnss_updates
         << " zupt_updates=" << metrics.zupt_updates << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_pos_err_m=" << metrics.final_pos_err_m
            << " final_vel_err_mps=" << metrics.final_vel_err_mps
            << " max_pos_err_m=" << metrics.max_pos_err_m
            << " max_vel_err_mps=" << metrics.max_vel_err_mps
            << " max_tilt_err_rad=" << metrics.max_tilt_err_rad
            << " max_reacq_jump_m=" << metrics.max_reacq_jump_m
            << " gnss_updates=" << metrics.gnss_updates
            << " zupt_updates=" << metrics.zupt_updates << std::endl;
}

// 测试结果记录：输出静止恢复姿态融合门控指标，便于验证无GNSS时AHRS能重新接管倾角修正。
static void recordScenarioPassWithAhrsStaticRecoveryMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AhrsStaticRecoveryMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | first_static_gate_allows_fusion="
         << (metrics.first_static_gate_allows_fusion ? 1 : 0)
         << " fused_count=" << metrics.fused_count
         << " skipped_count=" << metrics.skipped_count
         << " tilt_before_recovery_rad=" << metrics.tilt_before_recovery_rad
         << " tilt_after_recovery_rad=" << metrics.tilt_after_recovery_rad << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | first_static_gate_allows_fusion="
            << (metrics.first_static_gate_allows_fusion ? 1 : 0)
            << " fused_count=" << metrics.fused_count
            << " skipped_count=" << metrics.skipped_count
            << " tilt_before_recovery_rad=" << metrics.tilt_before_recovery_rad
            << " tilt_after_recovery_rad=" << metrics.tilt_after_recovery_rad << std::endl;
}

// 测试结果记录：输出静止假阳性机动保护指标，防止为恢复姿态而放松机动门控。
static void recordScenarioPassWithAhrsFalseStaticManeuverMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AhrsFalseStaticManeuverMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | false_static_gate_allows_fusion="
         << (metrics.false_static_gate_allows_fusion ? 1 : 0)
         << " body_lateral_accel_mps2=" << metrics.body_lateral_accel_mps2
         << " tilt_change_after_bad_ahrs_rad=" << metrics.tilt_change_after_bad_ahrs_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | false_static_gate_allows_fusion="
            << (metrics.false_static_gate_allows_fusion ? 1 : 0)
            << " body_lateral_accel_mps2=" << metrics.body_lateral_accel_mps2
            << " tilt_change_after_bad_ahrs_rad=" << metrics.tilt_change_after_bad_ahrs_rad
            << std::endl;
}

// 测试结果记录：输出GNSS新鲜度门控指标，避免串口断流后仍把旧fix当成有效卫导。
static void recordScenarioPassWithGnssFreshnessMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const GnssFreshnessMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | fresh_fix_valid=" << (metrics.fresh_fix_valid ? 1 : 0)
         << " stale_fix_valid=" << (metrics.stale_fix_valid ? 1 : 0)
         << " output_uses_ekf_on_stale_fix="
         << (metrics.output_uses_ekf_on_stale_fix ? 1 : 0)
         << " yaw_correction_runs_on_stale_fix="
         << (metrics.yaw_correction_runs_on_stale_fix ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | fresh_fix_valid=" << (metrics.fresh_fix_valid ? 1 : 0)
            << " stale_fix_valid=" << (metrics.stale_fix_valid ? 1 : 0)
            << " output_uses_ekf_on_stale_fix="
            << (metrics.output_uses_ekf_on_stale_fix ? 1 : 0)
            << " yaw_correction_runs_on_stale_fix="
            << (metrics.yaw_correction_runs_on_stale_fix ? 1 : 0) << std::endl;
}

// 测试结果记录：输出GNSS有效性迟滞指标，检查fix边界抖动不会造成EKF/AHRS来回切换。
static void recordScenarioPassWithGnssOutputHysteresisMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const GnssOutputHysteresisMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | switch_count=" << metrics.switch_count
         << " invalid_gap_max_ms=" << metrics.invalid_gap_max_ms
         << " output_valid_after_single_bad_frame="
         << (metrics.output_valid_after_single_bad_frame ? 1 : 0)
         << " output_invalid_after_long_dropout="
         << (metrics.output_invalid_after_long_dropout ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | switch_count=" << metrics.switch_count
            << " invalid_gap_max_ms=" << metrics.invalid_gap_max_ms
            << " output_valid_after_single_bad_frame="
            << (metrics.output_valid_after_single_bad_frame ? 1 : 0)
            << " output_invalid_after_long_dropout="
            << (metrics.output_invalid_after_long_dropout ? 1 : 0) << std::endl;
}

// 测试结果记录：输出AHRS航向噪声是否跟随GNSS新鲜度，防止旧fix影响无GNSS姿态闭环。
static void recordScenarioPassWithAhrsYawNoiseFreshnessMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AhrsYawNoiseFreshnessMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | fresh_yaw_noise_rad=" << metrics.fresh_yaw_noise_rad
         << " stale_yaw_noise_rad=" << metrics.stale_yaw_noise_rad
         << " stale_fix_uses_no_gnss_noise="
         << (metrics.stale_fix_uses_no_gnss_noise ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | fresh_yaw_noise_rad=" << metrics.fresh_yaw_noise_rad
            << " stale_yaw_noise_rad=" << metrics.stale_yaw_noise_rad
            << " stale_fix_uses_no_gnss_noise="
            << (metrics.stale_fix_uses_no_gnss_noise ? 1 : 0) << std::endl;
}

// 测试结果记录：输出备用AHRS航向偏置校正门控指标，避免GNSS保持期继续消耗旧EKF航向。
static void recordScenarioPassWithAhrsYawCorrectionHoldMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AhrsYawCorrectionHoldMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | correction_runs_on_fresh_gnss="
         << (metrics.correction_runs_on_fresh_gnss ? 1 : 0)
         << " correction_runs_during_output_hold="
         << (metrics.correction_runs_during_output_hold ? 1 : 0)
         << " correction_delta_on_fresh_gnss_rad="
         << metrics.correction_delta_on_fresh_gnss_rad
         << " correction_delta_during_output_hold_rad="
         << metrics.correction_delta_during_output_hold_rad << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | correction_runs_on_fresh_gnss="
            << (metrics.correction_runs_on_fresh_gnss ? 1 : 0)
            << " correction_runs_during_output_hold="
            << (metrics.correction_runs_during_output_hold ? 1 : 0)
            << " correction_delta_on_fresh_gnss_rad="
            << metrics.correction_delta_on_fresh_gnss_rad
            << " correction_delta_during_output_hold_rad="
            << metrics.correction_delta_during_output_hold_rad << std::endl;
}

// 测试结果记录：输出假原点重锚定后度制/弧度制原点同步指标，避免原始UBX相对位置跳变。
static void recordScenarioPassWithOriginReanchorMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const OriginReanchorMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | relative_jump_before_reanchor_m="
         << metrics.relative_jump_before_reanchor_m
         << " relative_after_reanchor_m="
         << metrics.relative_after_reanchor_m
         << " degree_origin_synced="
         << (metrics.degree_origin_synced ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | relative_jump_before_reanchor_m="
            << metrics.relative_jump_before_reanchor_m
            << " relative_after_reanchor_m="
            << metrics.relative_after_reanchor_m
            << " degree_origin_synced="
            << (metrics.degree_origin_synced ? 1 : 0) << std::endl;
}

// 测试结果记录：输出首次GNSS重锚定首帧UBX相对位置更新时序，避免发布旧假原点结果。
static void recordScenarioPassWithFirstGnssRelativeTimingMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const FirstGnssReanchorRelativeTimingMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | stale_relative_before_reanchor_m="
         << metrics.stale_relative_before_reanchor_m
         << " relative_after_reanchor_m="
         << metrics.relative_after_reanchor_m
         << " stale_relative_was_published="
         << (metrics.stale_relative_was_published ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | stale_relative_before_reanchor_m="
            << metrics.stale_relative_before_reanchor_m
            << " relative_after_reanchor_m="
            << metrics.relative_after_reanchor_m
            << " stale_relative_was_published="
            << (metrics.stale_relative_was_published ? 1 : 0) << std::endl;
}

// 测试结果记录：输出静止检测对GNSS新鲜度的依赖，避免旧GNSS速度阻塞ZUPT和无GNSS初始化。
static void recordScenarioPassWithStaticDetectionFreshnessMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const StaticDetectionFreshnessMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | stale_gnss_blocks_static="
         << (metrics.stale_gnss_blocks_static ? 1 : 0)
         << " static_confirmed_after_timeout="
         << (metrics.static_confirmed_after_timeout ? 1 : 0)
         << " confirmed_after_ms=" << metrics.confirmed_after_ms << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | stale_gnss_blocks_static="
            << (metrics.stale_gnss_blocks_static ? 1 : 0)
            << " static_confirmed_after_timeout="
            << (metrics.static_confirmed_after_timeout ? 1 : 0)
            << " confirmed_after_ms=" << metrics.confirmed_after_ms << std::endl;
}

// 测试结果记录：输出状态包fix新鲜度门控，避免旧fix阻塞水平KF/光流回退。
static void recordScenarioPassWithStatusFixFreshnessMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const StatusFixFreshnessMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | fresh_mapped_fix=" << metrics.fresh_mapped_fix
         << " stale_mapped_fix=" << metrics.stale_mapped_fix
         << " stale_fix_forces_horizontal_gnss_path="
         << (metrics.stale_fix_forces_horizontal_gnss_path ? 1 : 0) << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | fresh_mapped_fix=" << metrics.fresh_mapped_fix
            << " stale_mapped_fix=" << metrics.stale_mapped_fix
            << " stale_fix_forces_horizontal_gnss_path="
            << (metrics.stale_fix_forces_horizontal_gnss_path ? 1 : 0) << std::endl;
}

// 测试结果记录：输出AnoCom地面站GNSS遥测的新鲜度门控，避免断流后继续显示旧卫导。
static void recordScenarioPassWithAnoGnssTelemetryFreshnessMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AnoGnssTelemetryFreshnessMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | fresh_fix_sta=" << metrics.fresh_fix_sta
         << " stale_fix_sta=" << metrics.stale_fix_sta
         << " stale_num_sat=" << metrics.stale_num_sat
         << " stale_velocity_branch_uses_raw_gnss="
         << (metrics.stale_velocity_branch_uses_raw_gnss ? 1 : 0)
         << " stale_telemetry_reports_raw_gnss="
         << (metrics.stale_telemetry_reports_raw_gnss ? 1 : 0)
         << " stale_accuracy_reports_raw_gnss="
         << (metrics.stale_accuracy_reports_raw_gnss ? 1 : 0)
         << " stale_raw_velocity_norm_mps=" << metrics.stale_raw_velocity_norm_mps
         << " stale_raw_position_norm_deg=" << metrics.stale_raw_position_norm_deg
         << " stale_h_acc_m=" << metrics.stale_h_acc_m
         << " stale_v_acc_m=" << metrics.stale_v_acc_m
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | fresh_fix_sta=" << metrics.fresh_fix_sta
            << " stale_fix_sta=" << metrics.stale_fix_sta
            << " stale_num_sat=" << metrics.stale_num_sat
            << " stale_velocity_branch_uses_raw_gnss="
            << (metrics.stale_velocity_branch_uses_raw_gnss ? 1 : 0)
            << " stale_telemetry_reports_raw_gnss="
            << (metrics.stale_telemetry_reports_raw_gnss ? 1 : 0)
            << " stale_accuracy_reports_raw_gnss="
            << (metrics.stale_accuracy_reports_raw_gnss ? 1 : 0)
            << " stale_raw_velocity_norm_mps=" << metrics.stale_raw_velocity_norm_mps
            << " stale_raw_position_norm_deg=" << metrics.stale_raw_position_norm_deg
            << " stale_h_acc_m=" << metrics.stale_h_acc_m
            << " stale_v_acc_m=" << metrics.stale_v_acc_m
            << std::endl;
}

// 测试结果记录：输出2D/3D fix边界门控，防止把2D定位误当成可用于组合导航的3D卫导。
static void recordScenarioPassWithGnss3dBoundaryMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const Gnss3dBoundaryMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | fix_2d_valid_for_nav=" << (metrics.fix_2d_valid_for_nav ? 1 : 0)
         << " fix_3d_valid_for_nav=" << (metrics.fix_3d_valid_for_nav ? 1 : 0)
         << " dual_vector_yaw_allows_2d=" << (metrics.dual_vector_yaw_allows_2d ? 1 : 0)
         << " relative_position_allows_2d=" << (metrics.relative_position_allows_2d ? 1 : 0)
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | fix_2d_valid_for_nav=" << (metrics.fix_2d_valid_for_nav ? 1 : 0)
            << " fix_3d_valid_for_nav=" << (metrics.fix_3d_valid_for_nav ? 1 : 0)
            << " dual_vector_yaw_allows_2d=" << (metrics.dual_vector_yaw_allows_2d ? 1 : 0)
            << " relative_position_allows_2d=" << (metrics.relative_position_allows_2d ? 1 : 0)
            << std::endl;
}

// 测试结果记录：输出纯惯导高角速度传播误差，直接反馈EKF底层姿态积分精度。
static void recordScenarioPassWithHighRateQuatPropagationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const HighRateQuatPropagationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_quat_error_rad=" << metrics.final_quat_error_rad
         << " max_quat_error_rad=" << metrics.max_quat_error_rad
         << " total_rotation_rad=" << metrics.total_rotation_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_quat_error_rad=" << metrics.final_quat_error_rad
            << " max_quat_error_rad=" << metrics.max_quat_error_rad
            << " total_rotation_rad=" << metrics.total_rotation_rad
            << std::endl;
}

// 测试结果记录：输出误差状态注入后的剩余倾角，便于跟踪EKF底层反馈修正精度。
static void recordScenarioPassWithAttitudeInjectionMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const AttitudeInjectionMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | tilt_before_update_rad=" << metrics.tilt_before_update_rad
         << " tilt_after_update_rad=" << metrics.tilt_after_update_rad
         << " single_update_correction_rad=" << metrics.single_update_correction_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | tilt_before_update_rad=" << metrics.tilt_before_update_rad
            << " tilt_after_update_rad=" << metrics.tilt_after_update_rad
            << " single_update_correction_rad=" << metrics.single_update_correction_rad
            << std::endl;
}

// 测试结果记录：输出复合姿态量测后的四元数误差，检查欧拉角量测到误差状态的映射。
static void recordScenarioPassWithCompositeAttitudeMeasurementMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const CompositeAttitudeMeasurementMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | quat_error_before_rad=" << metrics.quat_error_before_rad
         << " quat_error_after_rad=" << metrics.quat_error_after_rad
         << " correction_rad=" << metrics.correction_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | quat_error_before_rad=" << metrics.quat_error_before_rad
            << " quat_error_after_rad=" << metrics.quat_error_after_rad
            << " correction_rad=" << metrics.correction_rad
            << std::endl;
}

// 测试结果记录：输出过程噪声压力场景输入范围，验证协方差PSD问题没有被低动态场景掩盖。
static void recordScenarioPassWithCovarianceProcessNoiseMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const CovarianceProcessNoiseMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | propagated_steps=" << metrics.propagated_steps
         << " dt_s=" << metrics.dt_s
         << " max_gyro_norm_radps=" << metrics.max_gyro_norm_radps
         << " max_accel_norm_mps2=" << metrics.max_accel_norm_mps2
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | propagated_steps=" << metrics.propagated_steps
            << " dt_s=" << metrics.dt_s
            << " max_gyro_norm_radps=" << metrics.max_gyro_norm_radps
            << " max_accel_norm_mps2=" << metrics.max_accel_norm_mps2
            << std::endl;
}

// 测试结果记录：输出高精度捷联传播误差，便于比较状态积分算法升级前后的精度。
static void recordScenarioPassWithStrapdownIntegrationAccuracyMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const StrapdownIntegrationAccuracyMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
         << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
         << " final_position_error_m=" << metrics.final_position_error_m
         << " max_position_error_m=" << metrics.max_position_error_m
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
            << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
            << " final_position_error_m=" << metrics.final_position_error_m
            << " max_position_error_m=" << metrics.max_position_error_m
            << std::endl;
}

// 测试结果记录：输出协方差离散化精度指标，便于确认高动态低频传播没有明显偏离子步参考。
static void recordScenarioPassWithCovarianceDiscretizationAccuracyMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const CovarianceDiscretizationAccuracyMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | relative_trace_error=" << metrics.relative_trace_error
         << " relative_max_coeff_error=" << metrics.relative_max_coeff_error
         << " low_rate_min_eigenvalue=" << metrics.low_rate_min_eigenvalue
         << " high_rate_min_eigenvalue=" << metrics.high_rate_min_eigenvalue
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | relative_trace_error=" << metrics.relative_trace_error
            << " relative_max_coeff_error=" << metrics.relative_max_coeff_error
            << " low_rate_min_eigenvalue=" << metrics.low_rate_min_eigenvalue
            << " high_rate_min_eigenvalue=" << metrics.high_rate_min_eigenvalue
            << std::endl;
}

// 测试结果记录：输出重力梯度耦合项，确认它进入D向速度误差行而不是位置误差行。
static void recordScenarioPassWithGravityGradientModelMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const GravityGradientModelMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | phi_vel_down_from_pos_down=" << metrics.phi_vel_down_from_pos_down
         << " phi_pos_down_from_pos_down=" << metrics.phi_pos_down_from_pos_down
         << " expected_phi_vel_down_from_pos_down="
         << metrics.expected_phi_vel_down_from_pos_down
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | phi_vel_down_from_pos_down=" << metrics.phi_vel_down_from_pos_down
            << " phi_pos_down_from_pos_down=" << metrics.phi_pos_down_from_pos_down
            << " expected_phi_vel_down_from_pos_down="
            << metrics.expected_phi_vel_down_from_pos_down
            << std::endl;
}

// 测试结果记录：输出WGS84正常重力静止传播指标，便于比较固定重力模型的垂向漂移。
static void recordScenarioPassWithWgs84GravityMechanizationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const Wgs84GravityMechanizationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | normal_gravity_mps2=" << metrics.normal_gravity_mps2
         << " fixed_gravity_mismatch_mps2=" << metrics.fixed_gravity_mismatch_mps2
         << " final_down_velocity_mps=" << metrics.final_down_velocity_mps
         << " max_speed_mps=" << metrics.max_speed_mps
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | normal_gravity_mps2=" << metrics.normal_gravity_mps2
            << " fixed_gravity_mismatch_mps2=" << metrics.fixed_gravity_mismatch_mps2
            << " final_down_velocity_mps=" << metrics.final_down_velocity_mps
            << " max_speed_mps=" << metrics.max_speed_mps
            << std::endl;
}

// 测试结果记录：输出流式圆锥补偿场景的姿态误差，直接反馈姿态机械编排精度。
static void recordScenarioPassWithConingCompensationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const ConingCompensationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_coning_error_rad=" << metrics.final_coning_error_rad
         << " max_coning_error_rad=" << metrics.max_coning_error_rad
         << " accumulated_nominal_rotation_rad=" << metrics.accumulated_nominal_rotation_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_coning_error_rad=" << metrics.final_coning_error_rad
            << " max_coning_error_rad=" << metrics.max_coning_error_rad
            << " accumulated_nominal_rotation_rad=" << metrics.accumulated_nominal_rotation_rad
            << std::endl;
}

// 测试结果记录：输出流式划摇补偿场景的速度/位置误差，评估纯惯导速度递推精度。
static void recordScenarioPassWithScullingCompensationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const ScullingCompensationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
         << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
         << " final_position_error_m=" << metrics.final_position_error_m
         << " max_position_error_m=" << metrics.max_position_error_m
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
            << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
            << " final_position_error_m=" << metrics.final_position_error_m
            << " max_position_error_m=" << metrics.max_position_error_m
            << std::endl;
}

// 测试结果记录：输出地球自转/科里奥利补偿误差，跟踪长时间高纬高速传播精度。
static void recordScenarioPassWithEarthRateCoriolisMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const EarthRateCoriolisMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
         << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
         << " final_position_error_m=" << metrics.final_position_error_m
         << " max_position_error_m=" << metrics.max_position_error_m
         << " expected_coriolis_speed_change_mps="
         << metrics.expected_coriolis_speed_change_mps
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_velocity_error_mps=" << metrics.final_velocity_error_mps
            << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
            << " final_position_error_m=" << metrics.final_position_error_m
            << " max_position_error_m=" << metrics.max_position_error_m
            << " expected_coriolis_speed_change_mps="
            << metrics.expected_coriolis_speed_change_mps
            << std::endl;
}

// 测试结果记录：输出导航系转动导致的姿态传播误差，验证姿态机械编排是否包含地球/输送率。
static void recordScenarioPassWithNavigationFrameRotationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const NavigationFrameRotationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_quat_error_rad=" << metrics.final_quat_error_rad
         << " max_quat_error_rad=" << metrics.max_quat_error_rad
         << " expected_nav_frame_rotation_rad="
         << metrics.expected_nav_frame_rotation_rad
         << " min_covariance_eigenvalue=" << metrics.min_covariance_eigenvalue
         << " max_covariance_trace=" << metrics.max_covariance_trace
         << " covariance_failure_step=" << metrics.covariance_failure_step
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_quat_error_rad=" << metrics.final_quat_error_rad
            << " max_quat_error_rad=" << metrics.max_quat_error_rad
            << " expected_nav_frame_rotation_rad="
            << metrics.expected_nav_frame_rotation_rad
            << " min_covariance_eigenvalue=" << metrics.min_covariance_eigenvalue
            << " max_covariance_trace=" << metrics.max_covariance_trace
            << " covariance_failure_step=" << metrics.covariance_failure_step
            << std::endl;
}

// 测试结果记录：输出静止初始化时地球自转是否被误估为陀螺零偏。
static void recordScenarioPassWithStaticEarthRateInitializationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const StaticEarthRateInitializationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_quat_drift_rad=" << metrics.final_quat_drift_rad
         << " max_quat_drift_rad=" << metrics.max_quat_drift_rad
         << " expected_earth_rotation_rad=" << metrics.expected_earth_rotation_rad
         << " initial_gyro_bias_norm_radps=" << metrics.initial_gyro_bias_norm_radps
         << " physical_earth_rate_norm_radps=" << metrics.physical_earth_rate_norm_radps
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_quat_drift_rad=" << metrics.final_quat_drift_rad
            << " max_quat_drift_rad=" << metrics.max_quat_drift_rad
            << " expected_earth_rotation_rad=" << metrics.expected_earth_rotation_rad
            << " initial_gyro_bias_norm_radps=" << metrics.initial_gyro_bias_norm_radps
            << " physical_earth_rate_norm_radps=" << metrics.physical_earth_rate_norm_radps
            << std::endl;
}

// 测试结果记录：输出静止锁Z轴陀螺时的地球自转保留情况。
static void recordScenarioPassWithStaticZLockEarthRateMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const StaticZLockEarthRateMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_quat_drift_rad=" << metrics.final_quat_drift_rad
         << " max_quat_drift_rad=" << metrics.max_quat_drift_rad
         << " expected_removed_earth_rate_z_rad="
         << metrics.expected_removed_earth_rate_z_rad
         << " locked_gyro_z_radps=" << metrics.locked_gyro_z_radps
         << " physical_gyro_z_radps=" << metrics.physical_gyro_z_radps
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_quat_drift_rad=" << metrics.final_quat_drift_rad
            << " max_quat_drift_rad=" << metrics.max_quat_drift_rad
            << " expected_removed_earth_rate_z_rad="
            << metrics.expected_removed_earth_rate_z_rad
            << " locked_gyro_z_radps=" << metrics.locked_gyro_z_radps
            << " physical_gyro_z_radps=" << metrics.physical_gyro_z_radps
            << std::endl;
}

// 测试结果记录：输出真正双子样接口相对参考算法的误差。
static void recordScenarioPassWithTwoSampleMechanizationMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const TwoSampleMechanizationMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | final_quat_error_rad=" << metrics.final_quat_error_rad
         << " max_quat_error_rad=" << metrics.max_quat_error_rad
         << " final_velocity_error_mps=" << metrics.final_velocity_error_mps
         << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
         << " final_position_error_m=" << metrics.final_position_error_m
         << " max_position_error_m=" << metrics.max_position_error_m
         << " average_accel_velocity_error_mps="
         << metrics.average_accel_velocity_error_mps
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | final_quat_error_rad=" << metrics.final_quat_error_rad
            << " max_quat_error_rad=" << metrics.max_quat_error_rad
            << " final_velocity_error_mps=" << metrics.final_velocity_error_mps
            << " max_velocity_error_mps=" << metrics.max_velocity_error_mps
            << " final_position_error_m=" << metrics.final_position_error_m
            << " max_position_error_m=" << metrics.max_position_error_m
            << " average_accel_velocity_error_mps="
            << metrics.average_accel_velocity_error_mps
            << std::endl;
}

// 测试结果记录：输出双子样延迟GNSS回放前后的状态跳变量，验证历史增量重放连续性。
static void recordScenarioPassWithTwoSampleDelayedReplayMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const TwoSampleDelayedReplayMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | quat_jump_after_gnss_rad=" << metrics.quat_jump_after_gnss_rad
         << " velocity_jump_after_gnss_mps=" << metrics.velocity_jump_after_gnss_mps
         << " position_jump_after_gnss_m=" << metrics.position_jump_after_gnss_m
         << " velocity_error_before_gnss_mps=" << metrics.velocity_error_before_gnss_mps
         << " velocity_error_after_gnss_mps=" << metrics.velocity_error_after_gnss_mps
         << " position_error_before_gnss_m=" << metrics.position_error_before_gnss_m
         << " position_error_after_gnss_m=" << metrics.position_error_after_gnss_m
         << " attitude_error_before_gnss_rad=" << metrics.attitude_error_before_gnss_rad
         << " attitude_error_after_gnss_rad=" << metrics.attitude_error_after_gnss_rad
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | quat_jump_after_gnss_rad=" << metrics.quat_jump_after_gnss_rad
            << " velocity_jump_after_gnss_mps=" << metrics.velocity_jump_after_gnss_mps
            << " position_jump_after_gnss_m=" << metrics.position_jump_after_gnss_m
            << " velocity_error_before_gnss_mps=" << metrics.velocity_error_before_gnss_mps
            << " velocity_error_after_gnss_mps=" << metrics.velocity_error_after_gnss_mps
            << " position_error_before_gnss_m=" << metrics.position_error_before_gnss_m
            << " position_error_after_gnss_m=" << metrics.position_error_after_gnss_m
            << " attitude_error_before_gnss_rad=" << metrics.attitude_error_before_gnss_rad
            << " attitude_error_after_gnss_rad=" << metrics.attitude_error_after_gnss_rad
            << std::endl;
}

// 测试结果记录：输出飞控接入层IMU多样本拆成双子样后的时间和积分误差。
static void recordScenarioPassWithNavigationInputSplitMetrics(
    std::ofstream &report,
    const char *scenario_name,
    const NavigationInputSplitMetrics &metrics)
{
  report << "[PASS] " << scenario_name
         << " | first_half_dt_s=" << metrics.first_half_dt_s
         << " second_half_dt_s=" << metrics.second_half_dt_s
         << " theta_split_error_rad=" << metrics.theta_split_error_rad
         << " delta_v_split_error_mps=" << metrics.delta_v_split_error_mps
         << " boundary_cross_fraction=" << metrics.boundary_cross_fraction
         << std::endl;
  std::cout << "[PASS] " << scenario_name
            << " | first_half_dt_s=" << metrics.first_half_dt_s
            << " second_half_dt_s=" << metrics.second_half_dt_s
            << " theta_split_error_rad=" << metrics.theta_split_error_rad
            << " delta_v_split_error_mps=" << metrics.delta_v_split_error_mps
            << " boundary_cross_fraction=" << metrics.boundary_cross_fraction
            << std::endl;
}

// 测试用状态包fix映射：GNSS超时后必须发布NoFix，避免旧fix阻塞水平KF/光流回退。
static int mapStatusFixForTest(const int ubx_fix, const bool gnss_data_fresh)
{
  if (!gnss_data_fresh)
  {
    return 1;
  }
  switch (ubx_fix)
  {
  case 1:
    return 1; // NoFix
  case 2:
    return 2; // 2D
  case 3:
    return 3; // 3D
  case 4:
    return 4; // DGPS
  case 5:
    return 5; // RTK Float
  case 6:
    return 6; // RTK Fixed
  default:
    return 1;
  }
}

// 测试用静止检测GNSS速度门控：只有新鲜GNSS速度才参与静止判断。
static bool isGpsStaticForStaticDetectionForTest(const int ubx_fix,
                                                 const float gnd_spd_mps,
                                                 const bool gnss_data_fresh)
{
  if ((ubx_fix >= kUbxFix3dForTest) && gnss_data_fresh)
  {
    return gnd_spd_mps < 0.2f;
  }
  return true;
}

// 测试用原点重锚定辅助：复现主程序首次真实GNSS接入后同步rad/deg两套原点。
static void syncOriginToGnssForTest(const Eigen::Vector3d &gnss_lla_rad_m,
                                    double *origin_lat_rad,
                                    double *origin_lon_rad,
                                    double *origin_lat_deg,
                                    double *origin_lon_deg,
                                    double *origin_alt_m)
{
  *origin_lat_rad = gnss_lla_rad_m(0);
  *origin_lon_rad = gnss_lla_rad_m(1);
  *origin_lat_deg = gnss_lla_rad_m(0) * 180.0 / kPi;
  *origin_lon_deg = gnss_lla_rad_m(1) * 180.0 / kPi;
  *origin_alt_m = gnss_lla_rad_m(2);
}

// 测试用UBX相对位置计算：主程序该路径使用度制原点，必须与EKF真实原点同步。
static float computeUbxRelativeDistanceFromDegreeOriginForTest(const Eigen::Vector3d &ubx_lla_rad_m,
                                                               const double origin_lat_deg,
                                                               const double origin_lon_deg,
                                                               const double origin_alt_m)
{
  const Eigen::Vector3d current_lla_deg(
      ubx_lla_rad_m(0) * 180.0 / kPi,
      ubx_lla_rad_m(1) * 180.0 / kPi,
      ubx_lla_rad_m(2));
  const Eigen::Vector3d origin_lla_deg(origin_lat_deg, origin_lon_deg, origin_alt_m);
  return static_cast<float>(bfs::lla2ned(current_lla_deg, origin_lla_deg, bfs::AngPosUnit::DEG).norm());
}

// 测试用UBX相对位置更新时序：假原点首次真实GNSS帧必须等待重锚定完成后再更新。
static bool shouldUpdateUbxRelativeBeforePotentialReanchorForTest(const bool gnss_data_ready,
                                                                  const bool initialized_with_fake_origin,
                                                                  const bool has_real_gnss_anchor)
{
  return gnss_data_ready && (!initialized_with_fake_origin || has_real_gnss_anchor);
}

// 测试用AHRS航向偏置校正门控：只有新鲜GNSS参与融合时，才允许用EKF航向校正备用AHRS。
static bool shouldCorrectBackupAhrsYawForTest(const bool gnss_instant_valid,
                                              const bool gnss_output_valid)
{
  (void)gnss_output_valid;
  return gnss_instant_valid;
}

// 测试用双矢量航向内层门控：函数内部必须拒绝旧GNSS和2D fix，不能只依赖外层调用点。
static bool shouldRunDualVectorYawFusionForTest(const int ubx_fix,
                                                const bool gnss_data_fresh,
                                                const bool flow_valid)
{
  return (ubx_fix >= kUbxFix3dForTest) && gnss_data_fresh && flow_valid;
}

// 测试用UBX相对位置内层门控：函数内部必须拒绝旧GNSS和2D fix，防止未来被其它任务误调用。
static bool shouldUpdateUbxRelativePositionForTest(const bool origin_set,
                                                   const bool nav_initialized,
                                                   const int ubx_fix,
                                                   const bool gnss_data_fresh)
{
  return origin_set && nav_initialized && (ubx_fix >= kUbxFix3dForTest) && gnss_data_fresh;
}

// 测试用AHRS航向偏置步进：复现主程序中“慢速把后台AHRS航向对齐EKF”的限幅积分。
static void applyBackupAhrsYawCorrectionForTest(const float ekf_yaw_rad,
                                                const float backup_ahrs_yaw_rad,
                                                float *ahrs_yaw_correction_rad)
{
  float yaw_error = wrapAnglePiForTest(ekf_yaw_rad - backup_ahrs_yaw_rad);
  yaw_error = clampForTest(yaw_error,
                           -kMaxAhrsYawCorrectionStepRad,
                           kMaxAhrsYawCorrectionStepRad);
  *ahrs_yaw_correction_rad = wrapAnglePiForTest(*ahrs_yaw_correction_rad + yaw_error);
}

// 场景27模块：长时间无GNSS后先用低质量GNSS恢复，再切到较好GNSS，验证重捕获逻辑能收敛且不跳变。
static GnssReacquisitionMetrics runLongGnssLossPoorReacquisitionScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla,
    const Eigen::Vector3f &level_ypr)
{
  bfs::Ekf15State degraded_reacq_ekf;
  degraded_reacq_ekf.gyro_std_radps(0.0015f);
  degraded_reacq_ekf.gyro_markov_bias_std_radps(0.00001f);
  degraded_reacq_ekf.gyro_tau_s(50.0f);
  degraded_reacq_ekf.accel_std_mps2(0.25f);
  degraded_reacq_ekf.accel_markov_bias_std_mps2(0.05f);
  degraded_reacq_ekf.accel_tau_s(100.0f);
  degraded_reacq_ekf.init_heading_err_std_rad(0.1f);
  degraded_reacq_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  constexpr int degraded_steps = 3600;           // 18秒，覆盖长时间无GNSS和后续恢复。
  constexpr int degraded_gnss_period_steps = 20; // 10Hz GNSS恢复。
  constexpr int degraded_delay_steps = 24;       // 与120ms延迟对应。
  Eigen::Vector3d degraded_truth_lla = lla;
  Eigen::Vector3f degraded_truth_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3d degraded_lla_hist[degraded_steps + 1];
  Eigen::Vector3f degraded_vel_hist[degraded_steps + 1];
  degraded_lla_hist[0] = degraded_truth_lla;
  degraded_vel_hist[0] = degraded_truth_vel;

  GnssReacquisitionMetrics metrics;

  for (int step = 1; step <= degraded_steps; ++step)
  {
    const float t_s = (step - 1) * 0.005f;
    const bool static_phase = (t_s < 2.0f) || (t_s >= 15.0f);
    const bool gnss_available = (t_s >= 12.0f);
    const bool poor_gnss_phase = (t_s >= 12.0f && t_s < 14.2f);
    Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
    if (t_s >= 2.0f && t_s < 4.5f)
    {
      truth_accel_ned << 0.50f, 0.08f, 0.0f;
    }
    else if (t_s >= 4.5f && t_s < 7.5f)
    {
      truth_accel_ned << 1.4f * std::sin(1.3f * t_s),
          0.8f * std::cos(1.1f * t_s),
          -0.45f + 0.30f * std::sin(1.7f * t_s);
    }
    else if (t_s >= 8.0f && t_s < 11.0f)
    {
      truth_accel_ned << -0.35f, -0.05f, 0.0f;
    }
    else if (static_phase)
    {
      degraded_truth_vel.setZero();
    }

    Eigen::Vector3f degraded_imu_accel =
        truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();
    degraded_imu_accel(0) += deterministicNoise(step, 0.050f, 0.8f);
    degraded_imu_accel(1) += deterministicNoise(step, 0.045f, 1.8f);
    degraded_imu_accel(2) += deterministicNoise(step, 0.075f, 2.8f);
    Eigen::Vector3f degraded_imu_gyro;
    degraded_imu_gyro << deterministicNoise(step, 0.0010f, 0.7f),
        deterministicNoise(step, 0.0010f, 1.7f),
        deterministicNoise(step, 0.0012f, 2.7f);

    degraded_reacq_ekf.TimeUpdate(degraded_imu_accel, degraded_imu_gyro, 0.005f);
    if (shouldFuseAhrsAttitude(degraded_imu_accel, degraded_imu_gyro, degraded_reacq_ekf.quat()))
    {
      const float accel_norm_error = std::fabs(degraded_imu_accel.norm() - kLocalGravity);
      const float roll_pitch_noise_rad = linearInterpolateForTest(accel_norm_error,
                                                                  0.3f, 2.0f,
                                                                  0.05f, 0.35f);
      degraded_reacq_ekf.MeasurementUpdateAttitude(level_ypr, roll_pitch_noise_rad, 0.50f);
    }

    if (!gnss_available && static_phase && (step % 20) == 0)
    {
      degraded_reacq_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
      ++metrics.zupt_updates;
    }

    degraded_truth_vel += truth_accel_ned * 0.005f;
    if (static_phase)
    {
      degraded_truth_vel.setZero();
    }
    degraded_truth_lla = integrateTruthLla(degraded_truth_lla, degraded_truth_vel, 0.005f);
    degraded_lla_hist[step] = degraded_truth_lla;
    degraded_vel_hist[step] = degraded_truth_vel;

    if (gnss_available && step > degraded_delay_steps && ((step % degraded_gnss_period_steps) == 0))
    {
      const Eigen::Vector3d lla_before_gnss = degraded_reacq_ekf.lla_rad_m();
      const int meas_idx = step - degraded_delay_steps;
      Eigen::Vector3d meas_lla = degraded_lla_hist[meas_idx];
      Eigen::Vector3f meas_vel = degraded_vel_hist[meas_idx];
      const float pos_noise_m = poor_gnss_phase ? 2.5f : 0.35f;
      const float vel_noise_mps = poor_gnss_phase ? 0.45f : 0.08f;
      const Eigen::Vector3d pos_noise_ned =
          (Eigen::Vector3d() << deterministicNoise(step, poor_gnss_phase ? 1.4f : 0.18f, 0.9f),
           deterministicNoise(step, poor_gnss_phase ? 1.2f : 0.16f, 1.9f),
           deterministicNoise(step, poor_gnss_phase ? 1.8f : 0.25f, 2.9f))
              .finished();
      meas_lla += bfs::ned2lla(pos_noise_ned, lla, bfs::AngPosUnit::RAD) - lla;
      meas_vel(0) += deterministicNoise(step, poor_gnss_phase ? 0.35f : 0.050f, 1.1f);
      meas_vel(1) += deterministicNoise(step, poor_gnss_phase ? 0.30f : 0.045f, 2.1f);
      meas_vel(2) += deterministicNoise(step, poor_gnss_phase ? 0.40f : 0.060f, 3.1f);

      degraded_reacq_ekf.gnss_pos_ne_std_m(pos_noise_m);
      degraded_reacq_ekf.gnss_pos_d_std_m(poor_gnss_phase ? 3.5f : 0.50f);
      degraded_reacq_ekf.gnss_vel_ne_std_mps(vel_noise_mps);
      degraded_reacq_ekf.gnss_vel_d_std_mps(poor_gnss_phase ? 0.60f : 0.10f);
      const bfs::MeasurementUpdateResult update_result =
          degraded_reacq_ekf.MeasurementUpdateDetailed(meas_vel, meas_lla, 0.005f);
      ++metrics.gnss_updates;

      const float reacq_jump_m = static_cast<float>(
          bfs::lla2ned(degraded_reacq_ekf.lla_rad_m(), lla_before_gnss, bfs::AngPosUnit::RAD).norm());
      metrics.max_reacq_jump_m = std::max(metrics.max_reacq_jump_m, reacq_jump_m);
      (void)update_result;
    }

    const Eigen::Vector3d degraded_err_ned =
        bfs::lla2ned(degraded_reacq_ekf.lla_rad_m(), degraded_truth_lla, bfs::AngPosUnit::RAD);
    const float degraded_pos_err_m = static_cast<float>(degraded_err_ned.head<2>().norm());
    const float degraded_vel_err_mps = (degraded_reacq_ekf.ned_vel_mps() - degraded_truth_vel).norm();
    const float degraded_tilt_err_rad =
        std::sqrt(degraded_reacq_ekf.roll_rad() * degraded_reacq_ekf.roll_rad() +
                  degraded_reacq_ekf.pitch_rad() * degraded_reacq_ekf.pitch_rad());
    metrics.max_pos_err_m = std::max(metrics.max_pos_err_m, degraded_pos_err_m);
    metrics.max_vel_err_mps = std::max(metrics.max_vel_err_mps, degraded_vel_err_mps);
    metrics.max_tilt_err_rad = std::max(metrics.max_tilt_err_rad, degraded_tilt_err_rad);
    assertFiniteEkfState(degraded_reacq_ekf);
    assertHealthyEkfCovariance(degraded_reacq_ekf);
  }

  const Eigen::Vector3d degraded_final_err_ned =
      bfs::lla2ned(degraded_reacq_ekf.lla_rad_m(), degraded_truth_lla, bfs::AngPosUnit::RAD);
  metrics.final_pos_err_m = static_cast<float>(degraded_final_err_ned.head<2>().norm());
  metrics.final_vel_err_mps = (degraded_reacq_ekf.ned_vel_mps() - degraded_truth_vel).norm();
  return metrics;
}

// 场景32模块：无GNSS静止恢复时，如果EKF倾角已经偏大，静止确认应重新允许AHRS姿态量测拉回姿态。
static AhrsStaticRecoveryMetrics runStaticAhrsRecoveryAfterEkfTiltErrorScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla,
    const Eigen::Vector3f &level_ypr)
{
  bfs::Ekf15State static_recovery_ekf;
  static_recovery_ekf.gyro_std_radps(0.0015f);
  static_recovery_ekf.gyro_markov_bias_std_radps(0.00001f);
  static_recovery_ekf.gyro_tau_s(50.0f);
  static_recovery_ekf.accel_std_mps2(0.25f);
  static_recovery_ekf.accel_markov_bias_std_mps2(0.05f);
  static_recovery_ekf.accel_tau_s(100.0f);
  static_recovery_ekf.init_heading_err_std_rad(0.1f);
  static_recovery_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Vector3f pitch_rate;
  pitch_rate << 0.0f, 0.30f, 0.0f;
  for (int i = 0; i < 260; ++i)
  {
    // 先模拟一段无GNSS、无AHRS辅助的姿态积分偏差，复现“姿态已经偏了以后门控不再放行”的风险。
    static_recovery_ekf.TimeUpdate(accel, pitch_rate, 0.005f);
  }

  AhrsStaticRecoveryMetrics metrics;
  metrics.tilt_before_recovery_rad =
      std::sqrt(static_recovery_ekf.roll_rad() * static_recovery_ekf.roll_rad() +
                static_recovery_ekf.pitch_rad() * static_recovery_ekf.pitch_rad());
  metrics.first_static_gate_allows_fusion =
      shouldFuseAhrsAttitude(accel, Eigen::Vector3f::Zero(), static_recovery_ekf.quat(), true);

  for (int step = 1; step <= 300; ++step)
  {
    // 静止确认后，陀螺和加速度都平稳，AHRS应能作为低频姿态观测帮助EKF回到水平。
    static_recovery_ekf.TimeUpdate(accel, Eigen::Vector3f::Zero(), 0.005f);
    if (shouldFuseAhrsAttitude(accel, Eigen::Vector3f::Zero(), static_recovery_ekf.quat(), true))
    {
      static_recovery_ekf.MeasurementUpdateAttitude(level_ypr, 0.04f, 0.50f);
      ++metrics.fused_count;
    }
    else
    {
      ++metrics.skipped_count;
    }
    if ((step % 20) == 0)
    {
      static_recovery_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
    }
    assertFiniteEkfState(static_recovery_ekf);
    assertHealthyEkfCovariance(static_recovery_ekf);
  }

  metrics.tilt_after_recovery_rad =
      std::sqrt(static_recovery_ekf.roll_rad() * static_recovery_ekf.roll_rad() +
                static_recovery_ekf.pitch_rad() * static_recovery_ekf.pitch_rad());
  return metrics;
}

// 场景33模块：静止检测假阳性时，明显机体系横向加速度不能因为static_confirmed而绕过AHRS机动门控。
static AhrsFalseStaticManeuverMetrics runFalseStaticManeuverAhrsGateScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State false_static_ekf;
  false_static_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Vector3f maneuver_accel;
  maneuver_accel << 2.4f, 0.0f, -std::sqrt(kLocalGravity * kLocalGravity - 2.4f * 2.4f);

  AhrsFalseStaticManeuverMetrics metrics;
  metrics.body_lateral_accel_mps2 =
      std::sqrt(maneuver_accel(0) * maneuver_accel(0) + maneuver_accel(1) * maneuver_accel(1));
  metrics.false_static_gate_allows_fusion =
      shouldFuseAhrsAttitude(maneuver_accel, Eigen::Vector3f::Zero(), false_static_ekf.quat(), true);

  const float tilt_before_bad_ahrs =
      std::sqrt(false_static_ekf.roll_rad() * false_static_ekf.roll_rad() +
                false_static_ekf.pitch_rad() * false_static_ekf.pitch_rad());
  if (metrics.false_static_gate_allows_fusion)
  {
    // 用错误AHRS倾角模拟机动加速度被当成重力后的输出，门控正确时不应执行到这里。
    Eigen::Vector3f bad_ahrs_ypr;
    bad_ahrs_ypr << 0.0f, std::atan2(maneuver_accel(0), -maneuver_accel(2)), 0.0f;
    false_static_ekf.MeasurementUpdateAttitude(bad_ahrs_ypr, 0.04f, 0.50f);
  }
  const float tilt_after_bad_ahrs =
      std::sqrt(false_static_ekf.roll_rad() * false_static_ekf.roll_rad() +
                false_static_ekf.pitch_rad() * false_static_ekf.pitch_rad());
  metrics.tilt_change_after_bad_ahrs_rad = std::fabs(tilt_after_bad_ahrs - tilt_before_bad_ahrs);
  return metrics;
}

// 场景34模块：UBX fix值保持3D但GNSS数据已超时时，导航输出和AHRS航向校正都必须按无GNSS处理。
static GnssFreshnessMetrics runStaleGnssFixShouldNotDriveNavigationScenario()
{
  GnssFreshnessMetrics metrics;
  const int fresh_now_ms = 1000;
  const int fresh_last_gnss_ms = 940;
  const int stale_now_ms = 2000;
  const int stale_last_gnss_ms = 1200;

  const bool fresh_data = isGnssDataFreshForNavForTest(fresh_now_ms, fresh_last_gnss_ms);
  const bool stale_data = isGnssDataFreshForNavForTest(stale_now_ms, stale_last_gnss_ms);

  metrics.fresh_fix_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, fresh_data);
  metrics.stale_fix_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, stale_data);

  // 主程序里这两个分支都应由同一个“新鲜GNSS有效”布尔量驱动。
  metrics.output_uses_ekf_on_stale_fix = metrics.stale_fix_valid;
  metrics.yaw_correction_runs_on_stale_fix = metrics.stale_fix_valid;
  return metrics;
}

// 场景35模块：GNSS fix偶发掉到非3D一两帧时，输出不应在EKF/AHRS之间快速来回切换。
static GnssOutputHysteresisMetrics runGnssFixFlickerShouldNotToggleOutputScenario()
{
  GnssOutputHysteresisMetrics metrics;
  bool previous_output_valid = false;
  int last_valid_ms = 0;
  int current_invalid_gap_ms = 0;

  for (int sample = 0; sample <= 80; ++sample)
  {
    const int now_ms = 1000 + sample * 50;
    const bool forced_bad_frame = (sample == 10) || (sample == 11) || (sample == 38);
    const int fix = forced_bad_frame ? 0 : kUbxFix3dForTest;
    const bool fresh = isGnssDataFreshForNavForTest(now_ms, now_ms);
    const bool instant_valid = isGnssFixValidForNavForTest(fix, fresh);
    const bool output_valid = updateGnssNavOutputValidForTest(instant_valid, now_ms, &last_valid_ms);

    if (sample == 10)
    {
      metrics.output_valid_after_single_bad_frame = output_valid;
    }
    if (sample > 0 && output_valid != previous_output_valid)
    {
      ++metrics.switch_count;
    }
    if (!instant_valid)
    {
      current_invalid_gap_ms += 50;
      metrics.invalid_gap_max_ms = std::max(metrics.invalid_gap_max_ms, current_invalid_gap_ms);
    }
    else
    {
      current_invalid_gap_ms = 0;
    }
    previous_output_valid = output_valid;
  }

  for (int sample = 1; sample <= 28; ++sample)
  {
    const int now_ms = 5100 + sample * 50;
    const bool fresh = isGnssDataFreshForNavForTest(now_ms, now_ms);
    const bool instant_valid = isGnssFixValidForNavForTest(0, fresh);
    const bool output_valid = updateGnssNavOutputValidForTest(instant_valid, now_ms, &last_valid_ms);
    if (sample > 1 && output_valid != previous_output_valid)
    {
      ++metrics.switch_count;
    }
    if (!instant_valid)
    {
      current_invalid_gap_ms += 50;
      metrics.invalid_gap_max_ms = std::max(metrics.invalid_gap_max_ms, current_invalid_gap_ms);
    }
    previous_output_valid = output_valid;
    if (sample == 28)
    {
      metrics.output_invalid_after_long_dropout = !output_valid;
    }
  }

  return metrics;
}

// 场景36模块：AHRS yaw噪声必须跟随GNSS新鲜度，旧fix不能让无GNSS姿态闭环继续降权航向。
static AhrsYawNoiseFreshnessMetrics runAhrsYawNoiseMustUseFreshGnssScenario()
{
  AhrsYawNoiseFreshnessMetrics metrics;
  const int fresh_now_ms = 1000;
  const int fresh_last_gnss_ms = 950;
  const bool fresh_data = isGnssDataFreshForNavForTest(fresh_now_ms, fresh_last_gnss_ms);
  const bool fresh_instant_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, fresh_data);
  metrics.fresh_yaw_noise_rad = selectAhrsYawNoiseForTest(kUbxFix3dForTest, fresh_instant_valid);

  const int stale_now_ms = 2000;
  const int stale_last_gnss_ms = 1200;
  const bool stale_data = isGnssDataFreshForNavForTest(stale_now_ms, stale_last_gnss_ms);
  const bool stale_instant_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, stale_data);
  metrics.stale_yaw_noise_rad = selectAhrsYawNoiseForTest(kUbxFix3dForTest, stale_instant_valid);
  metrics.stale_fix_uses_no_gnss_noise =
      std::fabs(metrics.stale_yaw_noise_rad - kAhrsYawNoiseWithoutGnssRad) < 1.0e-6f;
  return metrics;
}

// 场景37模块：GNSS输出保持期只允许维持EKF输出，不允许继续用旧EKF航向修正备用AHRS。
static AhrsYawCorrectionHoldMetrics runAhrsYawCorrectionMustUseFreshGnssScenario()
{
  AhrsYawCorrectionHoldMetrics metrics;
  int last_valid_ms = 0;
  float ahrs_yaw_correction_rad = 0.0f;

  const int fresh_now_ms = 1000;
  const bool fresh_data = isGnssDataFreshForNavForTest(fresh_now_ms, fresh_now_ms);
  const bool fresh_instant_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, fresh_data);
  const bool fresh_output_valid =
      updateGnssNavOutputValidForTest(fresh_instant_valid, fresh_now_ms, &last_valid_ms);

  const float fresh_before = ahrs_yaw_correction_rad;
  if (shouldCorrectBackupAhrsYawForTest(fresh_instant_valid, fresh_output_valid))
  {
    // 新鲜GNSS存在时，允许用EKF航向慢速对齐备用AHRS。
    applyBackupAhrsYawCorrectionForTest(0.42f, 0.20f, &ahrs_yaw_correction_rad);
  }
  metrics.correction_delta_on_fresh_gnss_rad =
      std::fabs(wrapAnglePiForTest(ahrs_yaw_correction_rad - fresh_before));
  metrics.correction_runs_on_fresh_gnss =
      metrics.correction_delta_on_fresh_gnss_rad > 1.0e-6f;

  const float hold_before = ahrs_yaw_correction_rad;
  const int hold_now_ms = fresh_now_ms + 500;
  const bool hold_data_fresh = isGnssDataFreshForNavForTest(hold_now_ms, fresh_now_ms);
  const bool hold_instant_valid = isGnssFixValidForNavForTest(kUbxFix3dForTest, hold_data_fresh);
  const bool hold_output_valid =
      updateGnssNavOutputValidForTest(hold_instant_valid, hold_now_ms, &last_valid_ms);
  assert(!hold_instant_valid);
  assert(hold_output_valid);

  if (shouldCorrectBackupAhrsYawForTest(hold_instant_valid, hold_output_valid))
  {
    // 输出保持期内没有新鲜GNSS量测；如果这里继续积分，备用AHRS会被旧EKF yaw 牵引。
    applyBackupAhrsYawCorrectionForTest(0.82f, 0.20f, &ahrs_yaw_correction_rad);
  }
  metrics.correction_delta_during_output_hold_rad =
      std::fabs(wrapAnglePiForTest(ahrs_yaw_correction_rad - hold_before));
  metrics.correction_runs_during_output_hold =
      metrics.correction_delta_during_output_hold_rad > 1.0e-6f;
  return metrics;
}

// 场景38模块：无GNSS假原点启动后，首次真实GNSS重锚定必须同步UBX相对位置使用的度制原点。
static OriginReanchorMetrics runOriginReanchorMustSyncDegreeOriginScenario()
{
  OriginReanchorMetrics metrics;
  Eigen::Vector3d fake_origin_lla;
  fake_origin_lla << 28.2 * kPi / 180.0, 112.9 * kPi / 180.0, 50.0;
  Eigen::Vector3d real_gnss_lla;
  real_gnss_lla << 31.2 * kPi / 180.0, 121.5 * kPi / 180.0, 18.0;

  double origin_lat_rad = fake_origin_lla(0);
  double origin_lon_rad = fake_origin_lla(1);
  double origin_lat_deg = fake_origin_lla(0) * 180.0 / kPi;
  double origin_lon_deg = fake_origin_lla(1) * 180.0 / kPi;
  double origin_alt_m = fake_origin_lla(2);

  metrics.relative_jump_before_reanchor_m =
      computeUbxRelativeDistanceFromDegreeOriginForTest(real_gnss_lla,
                                                        origin_lat_deg,
                                                        origin_lon_deg,
                                                        origin_alt_m);

  syncOriginToGnssForTest(real_gnss_lla,
                          &origin_lat_rad,
                          &origin_lon_rad,
                          &origin_lat_deg,
                          &origin_lon_deg,
                          &origin_alt_m);

  metrics.relative_after_reanchor_m =
      computeUbxRelativeDistanceFromDegreeOriginForTest(real_gnss_lla,
                                                        origin_lat_deg,
                                                        origin_lon_deg,
                                                        origin_alt_m);
  metrics.degree_origin_synced =
      (std::fabs(origin_lat_deg - real_gnss_lla(0) * 180.0 / kPi) < 1.0e-9) &&
      (std::fabs(origin_lon_deg - real_gnss_lla(1) * 180.0 / kPi) < 1.0e-9) &&
      (std::fabs(origin_alt_m - real_gnss_lla(2)) < 1.0e-6);
  assert(std::fabs(origin_lat_rad - real_gnss_lla(0)) < 1.0e-12);
  assert(std::fabs(origin_lon_rad - real_gnss_lla(1)) < 1.0e-12);
  return metrics;
}

// 场景39模块：首次真实GNSS触发重锚定时，UBX相对位置必须在重锚定之后计算。
static FirstGnssReanchorRelativeTimingMetrics runFirstGnssRelativeUpdateMustWaitForReanchorScenario()
{
  FirstGnssReanchorRelativeTimingMetrics metrics;
  Eigen::Vector3d fake_origin_lla;
  fake_origin_lla << 28.2 * kPi / 180.0, 112.9 * kPi / 180.0, 50.0;
  Eigen::Vector3d real_gnss_lla;
  real_gnss_lla << 31.2 * kPi / 180.0, 121.5 * kPi / 180.0, 18.0;

  double origin_lat_rad = fake_origin_lla(0);
  double origin_lon_rad = fake_origin_lla(1);
  double origin_lat_deg = fake_origin_lla(0) * 180.0 / kPi;
  double origin_lon_deg = fake_origin_lla(1) * 180.0 / kPi;
  double origin_alt_m = fake_origin_lla(2);

  const bool gnss_data_ready = true;
  const bool initialized_with_fake_origin = true;
  const bool has_real_gnss_anchor = false;
  if (shouldUpdateUbxRelativeBeforePotentialReanchorForTest(gnss_data_ready,
                                                            initialized_with_fake_origin,
                                                            has_real_gnss_anchor))
  {
    // 复现旧时序：首帧GNSS进来后，先用假原点算相对位置，随后才重锚定。
    metrics.stale_relative_before_reanchor_m =
        computeUbxRelativeDistanceFromDegreeOriginForTest(real_gnss_lla,
                                                          origin_lat_deg,
                                                          origin_lon_deg,
                                                          origin_alt_m);
    metrics.stale_relative_was_published = true;
  }

  syncOriginToGnssForTest(real_gnss_lla,
                          &origin_lat_rad,
                          &origin_lon_rad,
                          &origin_lat_deg,
                          &origin_lon_deg,
                          &origin_alt_m);
  metrics.relative_after_reanchor_m =
      computeUbxRelativeDistanceFromDegreeOriginForTest(real_gnss_lla,
                                                        origin_lat_deg,
                                                        origin_lon_deg,
                                                        origin_alt_m);
  assert(std::fabs(origin_lat_rad - real_gnss_lla(0)) < 1.0e-12);
  assert(std::fabs(origin_lon_rad - real_gnss_lla(1)) < 1.0e-12);
  return metrics;
}

// 场景40模块：GNSS数据超时后，旧的3D fix和旧速度不能阻塞IMU静止确认。
static StaticDetectionFreshnessMetrics runStaleGnssSpeedMustNotBlockStaticDetectionScenario()
{
  StaticDetectionFreshnessMetrics metrics;
  int static_start_time_ms = 0;
  bool is_static_confirmed = false;

  for (int sample = 0; sample <= 16; ++sample)
  {
    const int now_ms = 1000 + sample * 50;
    const int last_gnss_data_ms = 200;
    const bool gnss_fresh = isGnssDataFreshForNavForTest(now_ms, last_gnss_data_ms);
    const bool acc_static = true;
    const bool gyro_static = true;
    const bool gps_static =
        isGpsStaticForStaticDetectionForTest(kUbxFix3dForTest, 3.5f, gnss_fresh);

    if (sample == 0)
    {
      metrics.stale_gnss_blocks_static = !gps_static;
    }

    if (acc_static && gyro_static && gps_static)
    {
      if (static_start_time_ms == 0)
      {
        static_start_time_ms = now_ms;
      }
      else if (now_ms - static_start_time_ms > 500)
      {
        is_static_confirmed = true;
        if (metrics.confirmed_after_ms == 0)
        {
          metrics.confirmed_after_ms = now_ms - static_start_time_ms;
        }
      }
    }
    else
    {
      static_start_time_ms = 0;
      is_static_confirmed = false;
    }
  }

  metrics.static_confirmed_after_timeout = is_static_confirmed;
  return metrics;
}

// 场景41模块：GNSS数据超时后，状态包fix必须降级，避免水平估计继续走GNSS路径。
static StatusFixFreshnessMetrics runStatusFixMustUseFreshGnssScenario()
{
  StatusFixFreshnessMetrics metrics;
  const int fresh_now_ms = 1000;
  const bool fresh = isGnssDataFreshForNavForTest(fresh_now_ms, fresh_now_ms);
  metrics.fresh_mapped_fix = mapStatusFixForTest(3, fresh);

  const int stale_now_ms = 2000;
  const int stale_last_gnss_ms = 1200;
  const bool stale = isGnssDataFreshForNavForTest(stale_now_ms, stale_last_gnss_ms);
  metrics.stale_mapped_fix = mapStatusFixForTest(3, stale);
  metrics.stale_fix_forces_horizontal_gnss_path = metrics.stale_mapped_fix >= 3;
  return metrics;
}

// 场景42模块：AnoCom地面站GNSS遥测也必须使用新鲜度门控，不能继续发布旧UBX原始值。
static AnoGnssTelemetryFreshnessMetrics runAnoComGnssTelemetryMustUseFreshnessScenario()
{
  AnoGnssTelemetryFreshnessMetrics metrics;
  const int fresh_now_ms = 1000;
  const bool fresh = isGnssDataFreshForNavForTest(fresh_now_ms, fresh_now_ms);
  metrics.fresh_fix_sta = mapStatusFixForTest(3, fresh);

  const int stale_now_ms = 2000;
  const int stale_last_gnss_ms = 1200;
  const bool stale = isGnssDataFreshForNavForTest(stale_now_ms, stale_last_gnss_ms);
  metrics.stale_fix_sta = mapStatusFixForTest(3, stale);

  const float raw_lon_deg = 112.900123f;
  const float raw_lat_deg = 28.200456f;
  const float raw_north_vel_mps = 2.4f;
  const float raw_east_vel_mps = -0.7f;
  const int raw_num_sat = 18;
  const float raw_h_acc_m = 0.42f;
  const float raw_v_acc_m = 0.75f;

  // 速度分支必须同时检查GNSS新鲜度，串口断流后只允许使用KF/光流融合速度。
  metrics.stale_velocity_branch_uses_raw_gnss =
      (raw_lon_deg != 0.0f) && (raw_lat_deg != 0.0f) && (3 >= 3) && stale;
  if (metrics.stale_velocity_branch_uses_raw_gnss)
  {
    metrics.stale_raw_velocity_norm_mps =
        std::sqrt(raw_north_vel_mps * raw_north_vel_mps +
                  raw_east_vel_mps * raw_east_vel_mps);
  }

  // Status包降级为NoFix后，AnoCom也必须清零原始GNSS字段，避免地面站显示缓存旧值。
  if (stale)
  {
    metrics.stale_num_sat = raw_num_sat;
    metrics.stale_raw_position_norm_deg =
        std::sqrt(raw_lon_deg * raw_lon_deg + raw_lat_deg * raw_lat_deg);
  }
  else
  {
    metrics.stale_num_sat = 0;
    metrics.stale_raw_position_norm_deg = 0.0f;
  }
  // GNSS精度字段也来自接收机缓存；超时后必须清零，否则地面站会显示“无fix但仍有旧精度”。
  if (stale)
  {
    metrics.stale_h_acc_m = raw_h_acc_m;
    metrics.stale_v_acc_m = raw_v_acc_m;
  }
  else
  {
    metrics.stale_h_acc_m = 0.0f;
    metrics.stale_v_acc_m = 0.0f;
  }
  metrics.stale_telemetry_reports_raw_gnss =
      (metrics.stale_num_sat != 0) ||
      (metrics.stale_raw_position_norm_deg > 1.0e-6f) ||
      (metrics.stale_raw_velocity_norm_mps > 1.0e-6f);
  metrics.stale_accuracy_reports_raw_gnss =
      (metrics.stale_h_acc_m > 1.0e-6f) || (metrics.stale_v_acc_m > 1.0e-6f);
  return metrics;
}

// 场景43模块：组合导航相关GNSS路径必须严格使用3D fix，2D fix只能作为遥测状态显示。
static Gnss3dBoundaryMetrics runGnss3dFixBoundaryMustBeStrictScenario()
{
  Gnss3dBoundaryMetrics metrics;
  const int now_ms = 1000;
  const bool fresh = isGnssDataFreshForNavForTest(now_ms, now_ms);

  metrics.fix_2d_valid_for_nav = isGnssFixValidForNavForTest(kUbxFix2dForTest, fresh);
  metrics.fix_3d_valid_for_nav = isGnssFixValidForNavForTest(kUbxFix3dForTest, fresh);
  metrics.dual_vector_yaw_allows_2d =
      shouldRunDualVectorYawFusionForTest(kUbxFix2dForTest, fresh, true);
  metrics.relative_position_allows_2d =
      shouldUpdateUbxRelativePositionForTest(true, true, kUbxFix2dForTest, fresh);
  return metrics;
}

// 场景44模块：底层纯惯导姿态传播必须接近指数映射真值，避免高角速度时小角度近似累计拉偏。
static HighRateQuatPropagationMetrics runHighRateGyroPropagationMustUseExactDeltaScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State high_rate_ekf;
  high_rate_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Quaternionf truth_quat = high_rate_ekf.quat();
  HighRateQuatPropagationMetrics metrics;
  constexpr float dt_s = 0.005f; // 与主飞控导航任务200Hz一致。
  constexpr int steps = 300;     // 1.5秒，覆盖短时高角速度姿态传播累计误差。

  for (int step = 0; step < steps; ++step)
  {
    // 角速度保持在ICM42688 ±2000dps量程附近但不越界，直接检验EKF底层传播精度。
    Eigen::Vector3f gyro_profile_radps;
    gyro_profile_radps << 18.0f + 2.0f * std::sin(0.07f * step),
        -12.0f + 1.5f * std::cos(0.05f * step),
        24.0f + 2.5f * std::sin(0.04f * step + 0.6f);

    high_rate_ekf.TimeUpdate(accel, gyro_profile_radps, dt_s);
    truth_quat = (truth_quat * exactGyroDeltaQuatForTest(gyro_profile_radps, dt_s)).normalized();
    if (truth_quat.w() < 0.0f)
    {
      // 与EKF内部四元数符号规范保持一致，便于报告长期对比。
      truth_quat = Eigen::Quaternionf(-truth_quat.w(), -truth_quat.x(), -truth_quat.y(), -truth_quat.z());
    }

    const float quat_error_rad = quatDistanceRadForTest(high_rate_ekf.quat(), truth_quat);
    metrics.max_quat_error_rad = std::max(metrics.max_quat_error_rad, quat_error_rad);
    metrics.total_rotation_rad += gyro_profile_radps.norm() * dt_s;
    assertFiniteEkfState(high_rate_ekf);
    assertHealthyEkfCovariance(high_rate_ekf);
  }

  metrics.final_quat_error_rad = quatDistanceRadForTest(high_rate_ekf.quat(), truth_quat);
  return metrics;
}

// 场景45模块：误差状态姿态反馈也必须用指数映射，较大但合法的姿态残差不能被一阶注入低估。
static AttitudeInjectionMetrics runAttitudeErrorInjectionMustUseExactDeltaScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State injection_ekf;
  injection_ekf.init_att_err_std_rad(1.0f);
  injection_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Vector3f pitch_rate;
  pitch_rate << 0.0f, 0.5f, 0.0f;
  for (int i = 0; i < 200; ++i)
  {
    // 先形成0.5rad级别的合法倾角误差，模拟无GNSS阶段姿态积分后由AHRS强量测拉回。
    injection_ekf.TimeUpdate(accel, pitch_rate, 0.005f);
  }

  AttitudeInjectionMetrics metrics;
  metrics.tilt_before_update_rad =
      std::sqrt(injection_ekf.roll_rad() * injection_ekf.roll_rad() +
                injection_ekf.pitch_rad() * injection_ekf.pitch_rad());

  Eigen::Vector3f level_ypr;
  level_ypr << 0.0f, 0.0f, 0.0f;
  injection_ekf.MeasurementUpdateAttitude(level_ypr, 0.02f, 0.50f);

  metrics.tilt_after_update_rad =
      std::sqrt(injection_ekf.roll_rad() * injection_ekf.roll_rad() +
                injection_ekf.pitch_rad() * injection_ekf.pitch_rad());
  metrics.single_update_correction_rad =
      metrics.tilt_before_update_rad - metrics.tilt_after_update_rad;
  assertFiniteEkfState(injection_ekf);
  assertHealthyEkfCovariance(injection_ekf);
  return metrics;
}

// 场景59模块：复合yaw/pitch/roll误差不能用简单欧拉残差错误注入，应按等效四元数误差收敛。
static CompositeAttitudeMeasurementMetrics runCompositeAttitudeMeasurementScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State attitude_ekf;
  attitude_ekf.init_att_err_std_rad(1.2f);
  attitude_ekf.init_heading_err_std_rad(1.2f);
  attitude_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  const Eigen::Quaternionf truth_quat = attitude_ekf.quat();
  const Eigen::Vector3f composite_error =
      (Eigen::Vector3f() << 0.32f, -0.24f, 0.27f).finished();

  for (int i = 0; i < 200; ++i)
  {
    // 用真实陀螺传播制造复合姿态误差，避免先用高置信量测压缩协方差后再测试相反量测。
    attitude_ekf.TimeUpdate(accel, composite_error, 0.005f);
  }
  const Eigen::Quaternionf biased_state_quat = attitude_ekf.quat();

  CompositeAttitudeMeasurementMetrics metrics;
  metrics.quat_error_before_rad = quatDistanceRadForTest(biased_state_quat, truth_quat);

  const Eigen::Vector3f truth_ypr = bfs::quat2angle(truth_quat);
  attitude_ekf.MeasurementUpdateAttitude(truth_ypr, 0.01f, 0.01f);
  metrics.quat_error_after_rad = quatDistanceRadForTest(attitude_ekf.quat(), truth_quat);
  metrics.correction_rad = metrics.quat_error_before_rad - metrics.quat_error_after_rad;

  assertFiniteEkfState(attitude_ekf);
  assertHealthyEkfCovariance(attitude_ekf);
  return metrics;
}

// 场景46模块：过程噪声离散化必须保持PSD，高动态和较长dt也不能把协方差传播成非法矩阵。
static CovarianceProcessNoiseMetrics runProcessNoiseCovarianceMustRemainPsdScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State covariance_stress_ekf;
  covariance_stress_ekf.accel_std_mps2(0.25f);
  covariance_stress_ekf.gyro_std_radps(0.01f);
  covariance_stress_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  CovarianceProcessNoiseMetrics metrics;
  metrics.dt_s = 0.020f; // 覆盖任务调度抖动或低频回放时的较长但合法时间步。

  for (int step = 0; step < 200; ++step)
  {
    Eigen::Vector3f stress_accel;
    stress_accel << 20.0f * std::sin(0.04f * step),
        20.0f * std::cos(0.03f * step),
        -9.80665f + 0.2f * std::sin(0.05f * step);
    Eigen::Vector3f stress_gyro;
    stress_gyro << 15.0f * std::sin(0.03f * step),
        15.0f * std::cos(0.02f * step),
        7.5f * std::sin(0.04f * step);

    metrics.max_accel_norm_mps2 = std::max(metrics.max_accel_norm_mps2, stress_accel.norm());
    metrics.max_gyro_norm_radps = std::max(metrics.max_gyro_norm_radps, stress_gyro.norm());
    covariance_stress_ekf.TimeUpdate(stress_accel, stress_gyro, metrics.dt_s);
    ++metrics.propagated_steps;
    assertFiniteEkfState(covariance_stress_ekf);
    assertHealthyEkfCovariance(covariance_stress_ekf);
  }

  return metrics;
}

// 场景47模块：旋转机体系比力下，EKF低频传播应接近高频子步捷联真值。
static StrapdownIntegrationAccuracyMetrics runRotatingSpecificForceIntegrationAccuracyScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State strapdown_accuracy_ekf;
  strapdown_accuracy_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Quaternionf truth_quat = strapdown_accuracy_ekf.quat();
  Eigen::Vector3f truth_vel_ned = ned_vel;
  Eigen::Vector3d truth_lla = lla;

  StrapdownIntegrationAccuracyMetrics metrics;
  constexpr float dt_s = 0.020f; // 覆盖调度抖动/低频回放时更容易暴露积分误差的合法步长。
  constexpr int steps = 180;
  constexpr int truth_substeps = 20;

  for (int step = 0; step < steps; ++step)
  {
    const float t_s = step * dt_s;
    Eigen::Vector3f rotating_accel_body;
    rotating_accel_body << 2.2f + 0.5f * std::sin(1.7f * t_s),
        -1.4f + 0.4f * std::cos(1.1f * t_s),
        -9.80665f + 0.3f * std::sin(0.9f * t_s);
    Eigen::Vector3f rotating_gyro_body;
    rotating_gyro_body << 0.65f + 0.18f * std::sin(1.3f * t_s),
        -0.42f + 0.15f * std::cos(1.6f * t_s),
        0.88f + 0.12f * std::sin(1.9f * t_s);

    strapdown_accuracy_ekf.TimeUpdate(rotating_accel_body, rotating_gyro_body, dt_s);
    integrateTruthStrapdownSubstepsForTest(rotating_accel_body,
                                           rotating_gyro_body,
                                           dt_s,
                                           truth_substeps,
                                           &truth_quat,
                                           &truth_vel_ned,
                                           &truth_lla);

    const float velocity_error_mps =
        (strapdown_accuracy_ekf.ned_vel_mps() - truth_vel_ned).norm();
    const float position_error_m = static_cast<float>(
        bfs::lla2ned(strapdown_accuracy_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
    metrics.max_velocity_error_mps = std::max(metrics.max_velocity_error_mps,
                                              velocity_error_mps);
    metrics.max_position_error_m = std::max(metrics.max_position_error_m,
                                            position_error_m);
    assertFiniteEkfState(strapdown_accuracy_ekf);
    assertHealthyEkfCovariance(strapdown_accuracy_ekf);
  }

  metrics.final_velocity_error_mps =
      (strapdown_accuracy_ekf.ned_vel_mps() - truth_vel_ned).norm();
  metrics.final_position_error_m = static_cast<float>(
      bfs::lla2ned(strapdown_accuracy_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
  return metrics;
}

// 场景48模块：协方差低频传播应接近高频子步参考，避免一阶Phi在高动态时明显高估或低估不确定度。
static CovarianceDiscretizationAccuracyMetrics runCovarianceDiscretizationShouldMatchSubstepsScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State low_rate_ekf;
  bfs::Ekf15State high_rate_reference_ekf;
  low_rate_ekf.accel_std_mps2(0.35f);
  low_rate_ekf.gyro_std_radps(0.03f);
  low_rate_ekf.init_att_err_std_rad(0.40f);
  high_rate_reference_ekf.accel_std_mps2(0.35f);
  high_rate_reference_ekf.gyro_std_radps(0.03f);
  high_rate_reference_ekf.init_att_err_std_rad(0.40f);
  low_rate_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  high_rate_reference_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  constexpr float low_rate_dt_s = 0.020f;
  constexpr int substeps = 4;
  constexpr int steps = 80;

  for (int step = 0; step < steps; ++step)
  {
    const float t_s = step * low_rate_dt_s;
    Eigen::Vector3f dynamic_accel;
    dynamic_accel << 2.0f + 0.4f * std::sin(4.2f * t_s),
        -1.0f + 0.3f * std::cos(3.1f * t_s),
        -9.6f + 0.2f * std::sin(5.7f * t_s);
    Eigen::Vector3f dynamic_gyro;
    dynamic_gyro << 1.2f + 0.2f * std::cos(4.7f * t_s),
        -0.8f + 0.15f * std::sin(3.9f * t_s),
        1.5f + 0.25f * std::cos(2.5f * t_s);

    low_rate_ekf.TimeUpdate(dynamic_accel, dynamic_gyro, low_rate_dt_s);
    for (int substep = 0; substep < substeps; ++substep)
    {
      high_rate_reference_ekf.TimeUpdate(dynamic_accel,
                                         dynamic_gyro,
                                         low_rate_dt_s / static_cast<float>(substeps));
    }

    assertFiniteEkfState(low_rate_ekf);
    assertFiniteEkfState(high_rate_reference_ekf);
    assertHealthyEkfCovariance(low_rate_ekf);
    assertHealthyEkfCovariance(high_rate_reference_ekf);
  }

  CovarianceDiscretizationAccuracyMetrics metrics;
  const float high_rate_trace = high_rate_reference_ekf.covariance_trace();
  metrics.relative_trace_error =
      std::fabs(low_rate_ekf.covariance_trace() - high_rate_trace) /
      std::max(1.0f, high_rate_trace);
  const float high_rate_max_coeff = high_rate_reference_ekf.covariance_max_abs_coeff();
  metrics.relative_max_coeff_error =
      std::fabs(low_rate_ekf.covariance_max_abs_coeff() -
                high_rate_max_coeff) /
      std::max(1.0f, high_rate_max_coeff);
  metrics.low_rate_min_eigenvalue = low_rate_ekf.covariance_min_eigenvalue();
  metrics.high_rate_min_eigenvalue = high_rate_reference_ekf.covariance_min_eigenvalue();
  return metrics;
}

// 场景49模块：NED下D位置误差应通过重力梯度耦合到D速度误差，且位置自身不应承载重力梯度项。
static GravityGradientModelMetrics runGravityGradientMustBeInDownVelocityChannelScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State gravity_gradient_ekf;
  gravity_gradient_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  constexpr float dt_s = 0.020f;
  gravity_gradient_ekf.TimeUpdate(accel, gyro, dt_s);

  GravityGradientModelMetrics metrics;
  metrics.phi_vel_down_from_pos_down =
      gravity_gradient_ekf.state_transition_coeff_for_test(5, 2);
  metrics.phi_pos_down_from_pos_down =
      gravity_gradient_ekf.state_transition_coeff_for_test(2, 2);
  // NED中D向下为正，高度h=-D；g随高度升高减小，因此δD为正时下向重力加速度增大。
  const float normal_gravity_mps2 = normalGravityWgs84ForTest(lla);
  const double mean_radius_m =
      bfs::earthrad_mean_m(lla(0), bfs::AngPosUnit::RAD);
  metrics.expected_phi_vel_down_from_pos_down =
      static_cast<float>((2.0 * normal_gravity_mps2 / mean_radius_m) * dt_s);
  assertFiniteEkfState(gravity_gradient_ekf);
  assertHealthyEkfCovariance(gravity_gradient_ekf);
  return metrics;
}

// 场景50模块：静止比力应与WGS84正常重力匹配，固定9.79m/s²会在高纬/高空形成可观垂向漂移。
static Wgs84GravityMechanizationMetrics runWgs84NormalGravityMustPreventStaticVerticalDriftScenario(
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag)
{
  Eigen::Vector3d polar_high_lla;
  polar_high_lla << 70.0 * kPi / 180.0, 15.0 * kPi / 180.0, 1800.0;
  const float normal_gravity_mps2 = normalGravityWgs84ForTest(polar_high_lla);
  Eigen::Vector3f static_accel;
  static_accel << 0.0f, 0.0f, -normal_gravity_mps2;

  bfs::Ekf15State gravity_model_ekf;
  gravity_model_ekf.Initialize(static_accel, gyro, mag, Eigen::Vector3f::Zero(), polar_high_lla);

  Wgs84GravityMechanizationMetrics metrics;
  metrics.normal_gravity_mps2 = normal_gravity_mps2;
  metrics.fixed_gravity_mismatch_mps2 = std::fabs(normal_gravity_mps2 - kLocalGravity);

  constexpr float dt_s = 0.005f;
  constexpr int steps = 2000; // 10秒静止传播，固定重力模型会累计明显垂向速度。
  for (int step = 0; step < steps; ++step)
  {
    gravity_model_ekf.TimeUpdate(static_accel, gyro, dt_s);
    metrics.max_speed_mps = std::max(metrics.max_speed_mps,
                                     gravity_model_ekf.ned_vel_mps().norm());
    assertFiniteEkfState(gravity_model_ekf);
    assertHealthyEkfCovariance(gravity_model_ekf);
  }

  metrics.final_down_velocity_mps = gravity_model_ekf.ned_vel_mps()(2);
  return metrics;
}

// 场景51模块：交替小角增量会产生圆锥误差，EKF应使用上一样本补偿姿态更新。
static ConingCompensationMetrics runStreamingConingCompensationScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State coning_ekf;
  coning_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Quaternionf truth_quat = coning_ekf.quat();
  Eigen::Vector3f truth_vel_ned = ned_vel;
  Eigen::Vector3d truth_lla = lla;
  Eigen::Vector3f previous_delta_theta = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_delta_v = Eigen::Vector3f::Zero();

  ConingCompensationMetrics metrics;
  constexpr float dt_s = 0.005f;
  constexpr int steps = 800;
  for (int step = 0; step < steps; ++step)
  {
    Eigen::Vector3f coning_gyro;
    if ((step & 1) == 0)
    {
      coning_gyro << 24.0f, 0.0f, 3.0f;
    }
    else
    {
      coning_gyro << 0.0f, 24.0f, -3.0f;
    }
    const Eigen::Vector3f quiet_accel =
        (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(lla)).finished();

    coning_ekf.TimeUpdate(quiet_accel, coning_gyro, dt_s);
    integrateStreamingConingScullingReferenceForTest(quiet_accel,
                                                     coning_gyro,
                                                     dt_s,
                                                     &previous_delta_theta,
                                                     &previous_delta_v,
                                                     &truth_quat,
                                                     &truth_vel_ned,
                                                     &truth_lla);

    const float quat_error_rad = quatDistanceRadForTest(coning_ekf.quat(), truth_quat);
    metrics.max_coning_error_rad = std::max(metrics.max_coning_error_rad, quat_error_rad);
    metrics.accumulated_nominal_rotation_rad += coning_gyro.norm() * dt_s;
    assertFiniteEkfState(coning_ekf);
    assertHealthyEkfCovariance(coning_ekf);
  }
  metrics.final_coning_error_rad = quatDistanceRadForTest(coning_ekf.quat(), truth_quat);
  return metrics;
}

// 场景52模块：角增量与速度增量交替耦合时，EKF应补偿划摇项降低速度和位置误差。
static ScullingCompensationMetrics runStreamingScullingCompensationScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State sculling_ekf;
  sculling_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Quaternionf truth_quat = sculling_ekf.quat();
  Eigen::Vector3f truth_vel_ned = ned_vel;
  Eigen::Vector3d truth_lla = lla;
  Eigen::Vector3f previous_delta_theta = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_delta_v = Eigen::Vector3f::Zero();

  ScullingCompensationMetrics metrics;
  constexpr float dt_s = 0.005f;
  constexpr int steps = 900;
  for (int step = 0; step < steps; ++step)
  {
    Eigen::Vector3f sculling_gyro;
    Eigen::Vector3f sculling_accel;
    if ((step & 1) == 0)
    {
      sculling_gyro << 16.0f, -2.0f, 4.0f;
      sculling_accel << 6.0f, 0.5f, -normalGravityWgs84ForTest(lla) + 1.0f;
    }
    else
    {
      sculling_gyro << -1.0f, 15.0f, -3.5f;
      sculling_accel << -0.5f, 6.5f, -normalGravityWgs84ForTest(lla) - 1.0f;
    }

    sculling_ekf.TimeUpdate(sculling_accel, sculling_gyro, dt_s);
    integrateStreamingConingScullingReferenceForTest(sculling_accel,
                                                     sculling_gyro,
                                                     dt_s,
                                                     &previous_delta_theta,
                                                     &previous_delta_v,
                                                     &truth_quat,
                                                     &truth_vel_ned,
                                                     &truth_lla);

    const float velocity_error_mps = (sculling_ekf.ned_vel_mps() - truth_vel_ned).norm();
    const float position_error_m = static_cast<float>(
        bfs::lla2ned(sculling_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
    metrics.max_velocity_error_mps = std::max(metrics.max_velocity_error_mps,
                                              velocity_error_mps);
    metrics.max_position_error_m = std::max(metrics.max_position_error_m,
                                            position_error_m);
    assertFiniteEkfState(sculling_ekf);
    assertHealthyEkfCovariance(sculling_ekf);
  }

  metrics.final_velocity_error_mps = (sculling_ekf.ned_vel_mps() - truth_vel_ned).norm();
  metrics.final_position_error_m = static_cast<float>(
      bfs::lla2ned(sculling_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
  return metrics;
}

// 场景53模块：高纬高速下，地球自转和输送率补偿必须限制科里奥利速度误差。
static EarthRateCoriolisMetrics runEarthRateCoriolisCompensationScenario(
    const Eigen::Vector3f &mag)
{
  Eigen::Vector3d high_lat_lla;
  high_lat_lla << 68.0 * kPi / 180.0, 25.0 * kPi / 180.0, 1500.0;
  const Eigen::Vector3f initial_vel_ned =
      (Eigen::Vector3f() << 120.0f, 80.0f, -4.0f).finished();
  const Eigen::Vector3f initial_accel =
      (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(high_lat_lla)).finished();
  const Eigen::Vector3f initial_earth_rate_ned =
      bfs::earthrate(high_lat_lla(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f initial_nav_rate_ned =
      bfs::navrate(initial_vel_ned.cast<double>(), high_lat_lla, bfs::AngPosUnit::RAD)
          .cast<float>();
  const Eigen::Vector3f gyro = initial_earth_rate_ned + initial_nav_rate_ned;

  bfs::Ekf15State coriolis_ekf;
  coriolis_ekf.Initialize(initial_accel, gyro, mag, initial_vel_ned, high_lat_lla);

  Eigen::Quaternionf truth_quat = coriolis_ekf.quat();
  Eigen::Vector3f truth_vel_ned = initial_vel_ned;
  Eigen::Vector3d truth_lla = high_lat_lla;
  Eigen::Vector3f previous_delta_theta = Eigen::Vector3f::Zero();
  Eigen::Vector3f previous_delta_v = Eigen::Vector3f::Zero();

  EarthRateCoriolisMetrics metrics;
  constexpr float dt_s = 0.02f;
  constexpr int steps = 900; // 18秒高纬高速传播，足够暴露未补偿的科里奥利速度误差。
  const Eigen::Vector3f earth_rate_ned =
      bfs::earthrate(high_lat_lla(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f nav_rate_ned =
      bfs::navrate(initial_vel_ned.cast<double>(), high_lat_lla, bfs::AngPosUnit::RAD)
          .cast<float>();
  metrics.expected_coriolis_speed_change_mps =
      ((2.0f * earth_rate_ned + nav_rate_ned).cross(initial_vel_ned)).norm() *
      dt_s * static_cast<float>(steps);

  for (int step = 0; step < steps; ++step)
  {
    const Eigen::Vector3f accel =
        (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(truth_lla)).finished();
    const Eigen::Vector3f true_gyro =
        bfs::earthrate(truth_lla(0), bfs::AngPosUnit::RAD).cast<float>() +
        bfs::navrate(truth_vel_ned.cast<double>(), truth_lla, bfs::AngPosUnit::RAD)
            .cast<float>();
    coriolis_ekf.TimeUpdate(accel, true_gyro, dt_s);
    integrateEarthRateReferenceForTest(accel,
                                       true_gyro,
                                       dt_s,
                                       &previous_delta_theta,
                                       &previous_delta_v,
                                       &truth_quat,
                                       &truth_vel_ned,
                                       &truth_lla);

    const float velocity_error_mps = (coriolis_ekf.ned_vel_mps() - truth_vel_ned).norm();
    const float position_error_m = static_cast<float>(
        bfs::lla2ned(coriolis_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
    metrics.max_velocity_error_mps = std::max(metrics.max_velocity_error_mps,
                                              velocity_error_mps);
    metrics.max_position_error_m = std::max(metrics.max_position_error_m,
                                            position_error_m);
    assertFiniteEkfState(coriolis_ekf);
    assertHealthyEkfCovariance(coriolis_ekf);
  }

  metrics.final_velocity_error_mps = (coriolis_ekf.ned_vel_mps() - truth_vel_ned).norm();
  metrics.final_position_error_m = static_cast<float>(
      bfs::lla2ned(coriolis_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
  return metrics;
}

// 场景56模块：姿态传播应补偿导航系相对惯性系转动，避免高速/高纬长时航向漂移。
static NavigationFrameRotationMetrics runNavigationFrameRotationCompensationScenario(
    const Eigen::Vector3f &mag)
{
  bfs::Ekf15State nav_rotation_ekf;
  Eigen::Vector3d high_lat_lla;
  high_lat_lla << 70.0 * kPi / 180.0, 20.0 * kPi / 180.0, 1000.0;
  Eigen::Vector3f high_speed_vel;
  high_speed_vel << 0.0f, 280.0f, 0.0f;
  const Eigen::Vector3f accel =
      (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(high_lat_lla))
          .finished();
  const Eigen::Vector3f gyro =
      bfs::earthrate(high_lat_lla(0), bfs::AngPosUnit::RAD).cast<float>() +
      bfs::navrate(high_speed_vel.cast<double>(), high_lat_lla, bfs::AngPosUnit::RAD)
          .cast<float>();
  nav_rotation_ekf.Initialize(accel, gyro, mag, high_speed_vel, high_lat_lla);

  Eigen::Quaternionf truth_quat = nav_rotation_ekf.quat();
  Eigen::Vector3f truth_vel_ned = high_speed_vel;
  Eigen::Vector3d truth_lla = high_lat_lla;
  NavigationFrameRotationMetrics metrics;
  metrics.min_covariance_eigenvalue = std::numeric_limits<float>::infinity();
  constexpr float dt_s = 0.02f;
  constexpr int steps = 4000; // 80秒，放大导航系转动对姿态的可观测误差。
  for (int step = 0; step < steps; ++step)
  {
    const Eigen::Vector3f static_accel =
        (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(truth_lla))
            .finished();
    const Eigen::Vector3f true_gyro =
        bfs::earthrate(truth_lla(0), bfs::AngPosUnit::RAD).cast<float>() +
        bfs::navrate(truth_vel_ned.cast<double>(), truth_lla, bfs::AngPosUnit::RAD)
            .cast<float>();
    nav_rotation_ekf.TimeUpdate(static_accel, true_gyro, dt_s);
    integrateNavigationFrameRotationReferenceForTest(static_accel,
                                                     true_gyro,
                                                     dt_s,
                                                     &truth_quat,
                                                     &truth_vel_ned,
                                                     &truth_lla,
                                                     &metrics.expected_nav_frame_rotation_rad);
    const float quat_error_rad =
        quatDistanceRadForTest(nav_rotation_ekf.quat(), truth_quat);
    metrics.max_quat_error_rad = std::max(metrics.max_quat_error_rad, quat_error_rad);
    metrics.min_covariance_eigenvalue =
        std::min(metrics.min_covariance_eigenvalue,
                 nav_rotation_ekf.covariance_min_eigenvalue());
    metrics.max_covariance_trace =
        std::max(metrics.max_covariance_trace, nav_rotation_ekf.covariance_trace());
    if ((metrics.covariance_failure_step < 0) &&
        !nav_rotation_ekf.covariance_is_positive_semidefinite())
    {
      metrics.covariance_failure_step = step;
      break;
    }
    assertFiniteEkfState(nav_rotation_ekf);
    assertHealthyEkfCovariance(nav_rotation_ekf);
  }

  metrics.final_quat_error_rad =
      quatDistanceRadForTest(nav_rotation_ekf.quat(), truth_quat);
  return metrics;
}

// 场景57模块：静止初始化时陀螺读数包含地球自转，EKF不应把物理地球自转误当成零偏扣除。
static StaticEarthRateInitializationMetrics runStaticEarthRateInitializationScenario(
    const Eigen::Vector3f &mag)
{
  bfs::Ekf15State static_ekf;
  Eigen::Vector3d high_lat_lla;
  high_lat_lla << 65.0 * kPi / 180.0, 112.0 * kPi / 180.0, 120.0;
  const Eigen::Vector3f zero_vel = Eigen::Vector3f::Zero();
  const Eigen::Vector3f static_accel =
      (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(high_lat_lla))
          .finished();
  const Eigen::Vector3f physical_earth_rate_ned =
      bfs::earthrate(high_lat_lla(0), bfs::AngPosUnit::RAD).cast<float>();
  const Eigen::Vector3f static_gyro_body = physical_earth_rate_ned;

  static_ekf.Initialize(static_accel, static_gyro_body, mag, zero_vel, high_lat_lla);
  const Eigen::Quaternionf initial_quat = static_ekf.quat();

  StaticEarthRateInitializationMetrics metrics;
  metrics.initial_gyro_bias_norm_radps = static_ekf.gyro_bias_radps().norm();
  metrics.physical_earth_rate_norm_radps = physical_earth_rate_ned.norm();

  constexpr float dt_s = 0.02f;
  constexpr int steps = 12000; // 240秒，用静止长时场景放大地球自转误扣带来的姿态漂移。
  for (int step = 0; step < steps; ++step)
  {
    static_ekf.TimeUpdate(static_accel, static_gyro_body, dt_s);
    const float drift_rad = quatDistanceRadForTest(static_ekf.quat(), initial_quat);
    metrics.max_quat_drift_rad = std::max(metrics.max_quat_drift_rad, drift_rad);
    metrics.expected_earth_rotation_rad += physical_earth_rate_ned.norm() * dt_s;
    assertFiniteEkfState(static_ekf);
    assertHealthyEkfCovariance(static_ekf);
  }

  metrics.final_quat_drift_rad = quatDistanceRadForTest(static_ekf.quat(), initial_quat);
  return metrics;
}

// 场景58模块：复现飞控接入层静止锁Z轴逻辑，验证不能把地球自转Z分量从EKF输入中抹掉。
static StaticZLockEarthRateMetrics runStaticZLockMustPreserveEarthRateScenario(
    const Eigen::Vector3f &mag)
{
  bfs::Ekf15State z_lock_ekf;
  Eigen::Vector3d lla;
  lla << 65.0 * kPi / 180.0, 112.0 * kPi / 180.0, 120.0;
  const Eigen::Vector3f zero_vel = Eigen::Vector3f::Zero();
  const Eigen::Vector3f static_accel =
      (Eigen::Vector3f() << 0.0f, 0.0f, -normalGravityWgs84ForTest(lla)).finished();
  const Eigen::Vector3f physical_gyro =
      bfs::earthrate(lla(0), bfs::AngPosUnit::RAD).cast<float>();

  z_lock_ekf.Initialize(static_accel, physical_gyro, mag, zero_vel, lla);
  const Eigen::Quaternionf initial_quat = z_lock_ekf.quat();

  StaticZLockEarthRateMetrics metrics;
  constexpr float dt_s = 0.02f;
  constexpr int steps = 12000; // 240秒，用静止锁轴路径放大被抹掉的地球自转Z分量。

  for (int step = 0; step < steps; ++step)
  {
    Eigen::Vector3f locked_gyro = physical_gyro;
    // 复现当前main.cpp逻辑：静止时不再强行覆盖Z轴陀螺，保留物理地球自转输入。
    metrics.locked_gyro_z_radps = locked_gyro(2);
    metrics.physical_gyro_z_radps = physical_gyro(2);
    metrics.expected_removed_earth_rate_z_rad +=
        std::fabs(physical_gyro(2) - locked_gyro(2)) * dt_s;

    z_lock_ekf.TimeUpdate(static_accel, locked_gyro, dt_s);
    const float drift_rad = quatDistanceRadForTest(z_lock_ekf.quat(), initial_quat);
    metrics.max_quat_drift_rad = std::max(metrics.max_quat_drift_rad, drift_rad);
    assertFiniteEkfState(z_lock_ekf);
    assertHealthyEkfCovariance(z_lock_ekf);
  }

  metrics.final_quat_drift_rad = quatDistanceRadForTest(z_lock_ekf.quat(), initial_quat);
  return metrics;
}

// 场景54模块：同一导航周期内两个IMU子样必须走真正双子样圆锥/划摇补偿。
static TwoSampleMechanizationMetrics runTrueTwoSampleMechanizationScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State two_sample_ekf;
  bfs::Ekf15State average_reference_ekf;
  two_sample_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  average_reference_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  Eigen::Quaternionf truth_quat = two_sample_ekf.quat();
  Eigen::Vector3f truth_vel_ned = ned_vel;
  Eigen::Vector3d truth_lla = lla;

  TwoSampleMechanizationMetrics metrics;
  constexpr float dt_s = 0.010f;
  constexpr int steps = 500;
  for (int step = 0; step < steps; ++step)
  {
    const float t_s = step * dt_s;
    const Eigen::Vector3f gyro_1 =
        (Eigen::Vector3f() << 18.0f + 1.2f * std::sin(1.1f * t_s),
         -1.0f + 0.6f * std::cos(0.7f * t_s),
         2.0f)
            .finished();
    const Eigen::Vector3f gyro_2 =
        (Eigen::Vector3f() << -0.5f,
         17.0f + 1.0f * std::cos(0.9f * t_s),
         -2.5f)
            .finished();
    const Eigen::Vector3f accel_1 =
        (Eigen::Vector3f() << 5.5f + 0.3f * std::sin(0.8f * t_s),
         -0.8f,
         -normalGravityWgs84ForTest(truth_lla) + 0.5f)
            .finished();
    const Eigen::Vector3f accel_2 =
        (Eigen::Vector3f() << -0.6f,
         5.8f + 0.4f * std::cos(0.6f * t_s),
         -normalGravityWgs84ForTest(truth_lla) - 0.4f)
            .finished();

    const float half_dt_s = 0.5f * dt_s;
    const Eigen::Vector3f delta_theta_1 = gyro_1 * half_dt_s;
    const Eigen::Vector3f delta_theta_2 = gyro_2 * half_dt_s;
    const Eigen::Vector3f delta_v_1 = accel_1 * half_dt_s;
    const Eigen::Vector3f delta_v_2 = accel_2 * half_dt_s;
    const Eigen::Vector3f average_gyro = 0.5f * (gyro_1 + gyro_2);
    const Eigen::Vector3f average_accel = 0.5f * (accel_1 + accel_2);

    // 主路径使用两个子样原始增量，验证EKF内部同周期圆锥/划摇补偿。
    two_sample_ekf.TimeUpdateTwoSample(delta_theta_1,
                                       delta_theta_2,
                                       delta_v_1,
                                       delta_v_2,
                                       dt_s);
    average_reference_ekf.TimeUpdate(average_accel, average_gyro, dt_s);
    integrateTwoSampleReferenceForTest(delta_theta_1,
                                       delta_theta_2,
                                       delta_v_1,
                                       delta_v_2,
                                       dt_s,
                                       &truth_quat,
                                       &truth_vel_ned,
                                       &truth_lla);

    const float quat_error_rad = quatDistanceRadForTest(two_sample_ekf.quat(), truth_quat);
    const float velocity_error_mps = (two_sample_ekf.ned_vel_mps() - truth_vel_ned).norm();
    const float position_error_m = static_cast<float>(
        bfs::lla2ned(two_sample_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
    metrics.max_quat_error_rad = std::max(metrics.max_quat_error_rad, quat_error_rad);
    metrics.max_velocity_error_mps = std::max(metrics.max_velocity_error_mps,
                                              velocity_error_mps);
    metrics.max_position_error_m = std::max(metrics.max_position_error_m,
                                            position_error_m);
    assertFiniteEkfState(two_sample_ekf);
    assertHealthyEkfCovariance(two_sample_ekf);
  }

  metrics.final_quat_error_rad = quatDistanceRadForTest(two_sample_ekf.quat(), truth_quat);
  metrics.final_velocity_error_mps = (two_sample_ekf.ned_vel_mps() - truth_vel_ned).norm();
  metrics.final_position_error_m = static_cast<float>(
      bfs::lla2ned(two_sample_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
  metrics.average_accel_velocity_error_mps =
      (average_reference_ekf.ned_vel_mps() - truth_vel_ned).norm();
  return metrics;
}

// 场景55模块：延迟GNSS更新后必须按原始双子样增量重放，不能破坏当前状态连续性。
static TwoSampleDelayedReplayMetrics runTwoSampleDelayedReplayScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla)
{
  bfs::Ekf15State replay_ekf;
  replay_ekf.gnss_pos_ne_std_m(1000.0f);
  replay_ekf.gnss_pos_d_std_m(1000.0f);
  replay_ekf.gnss_vel_ne_std_mps(0.02f);
  replay_ekf.gnss_vel_d_std_mps(0.02f);
  replay_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  Eigen::Quaternionf truth_quat = replay_ekf.quat();
  Eigen::Vector3f truth_vel_ned = ned_vel;
  Eigen::Vector3d truth_lla = lla;

  constexpr float dt_s = 0.010f;
  constexpr int steps = 80;
  const int gnss_delay_steps =
      std::max(1, static_cast<int>(std::round(kEkfGnssDelayS / dt_s)));
  Eigen::Vector3f vel_hist[steps];
  Eigen::Vector3d lla_hist[steps];
  Eigen::Vector3f truth_vel_hist[steps];
  Eigen::Vector3d truth_lla_hist[steps];

  for (int step = 0; step < steps; ++step)
  {
    const float t_s = step * dt_s;
    const Eigen::Vector3f gyro_bias =
        (Eigen::Vector3f() << 0.004f, -0.003f, 0.002f).finished();
    const Eigen::Vector3f true_gyro_1 =
        (Eigen::Vector3f() << 1.2f + 0.1f * std::sin(0.5f * t_s),
         -0.4f,
         0.8f)
            .finished();
    const Eigen::Vector3f true_gyro_2 =
        (Eigen::Vector3f() << -0.6f,
         1.1f + 0.1f * std::cos(0.4f * t_s),
         -0.7f)
            .finished();
    const Eigen::Vector3f accel_1 =
        (Eigen::Vector3f() << 0.8f,
         -0.3f,
         -normalGravityWgs84ForTest(replay_ekf.lla_rad_m()) + 0.2f)
            .finished();
    const Eigen::Vector3f accel_2 =
        (Eigen::Vector3f() << -0.2f,
         0.6f,
         -normalGravityWgs84ForTest(replay_ekf.lla_rad_m()) - 0.1f)
            .finished();
    const float half_dt_s = 0.5f * dt_s;
    const Eigen::Vector3f true_delta_theta_1 = true_gyro_1 * half_dt_s;
    const Eigen::Vector3f true_delta_theta_2 = true_gyro_2 * half_dt_s;
    const Eigen::Vector3f true_delta_v_1 = accel_1 * half_dt_s;
    const Eigen::Vector3f true_delta_v_2 = accel_2 * half_dt_s;
    integrateTwoSampleReferenceForTest(true_delta_theta_1,
                                       true_delta_theta_2,
                                       true_delta_v_1,
                                       true_delta_v_2,
                                       dt_s,
                                       &truth_quat,
                                       &truth_vel_ned,
                                       &truth_lla);
    replay_ekf.TimeUpdateTwoSample((true_gyro_1 + gyro_bias) * half_dt_s,
                                   (true_gyro_2 + gyro_bias) * half_dt_s,
                                   (accel_1 + Eigen::Vector3f(0.06f, -0.04f, 0.02f)) * half_dt_s,
                                   (accel_2 + Eigen::Vector3f(0.06f, -0.04f, 0.02f)) * half_dt_s,
                                   dt_s);
    vel_hist[step] = replay_ekf.ned_vel_mps();
    lla_hist[step] = replay_ekf.lla_rad_m();
    truth_vel_hist[step] = truth_vel_ned;
    truth_lla_hist[step] = truth_lla;
  }

  const Eigen::Quaternionf quat_before = replay_ekf.quat();
  const Eigen::Vector3f vel_before = replay_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before = replay_ekf.lla_rad_m();
  const int meas_idx = steps - 1 - gnss_delay_steps;
  replay_ekf.MeasurementUpdate(truth_vel_hist[meas_idx], truth_lla_hist[meas_idx], dt_s);

  TwoSampleDelayedReplayMetrics metrics;
  metrics.velocity_error_before_gnss_mps = (vel_before - truth_vel_ned).norm();
  metrics.position_error_before_gnss_m = static_cast<float>(
      bfs::lla2ned(lla_before, truth_lla, bfs::AngPosUnit::RAD).norm());
  metrics.attitude_error_before_gnss_rad = quatDistanceRadForTest(quat_before, truth_quat);
  metrics.velocity_error_after_gnss_mps = (replay_ekf.ned_vel_mps() - truth_vel_ned).norm();
  metrics.position_error_after_gnss_m = static_cast<float>(
      bfs::lla2ned(replay_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD).norm());
  metrics.attitude_error_after_gnss_rad = quatDistanceRadForTest(replay_ekf.quat(), truth_quat);
  metrics.quat_jump_after_gnss_rad =
      quatDistanceRadForTest(replay_ekf.quat(), quat_before);
  metrics.velocity_jump_after_gnss_mps =
      (replay_ekf.ned_vel_mps() - vel_before).norm();
  metrics.position_jump_after_gnss_m = static_cast<float>(
      bfs::lla2ned(replay_ekf.lla_rad_m(), lla_before, bfs::AngPosUnit::RAD).norm());
  assertFiniteEkfState(replay_ekf);
  assertHealthyEkfCovariance(replay_ekf);
  return metrics;
}

// 场景60模块：复现飞控接入层多IMU样本按真实dt拆成两个子样，防止跨中点样本整包落错半周期。
static NavigationInputSplitMetrics runNavigationInputSplitScenario()
{
  constexpr int sample_count = 7;
  const float dt_samples[sample_count] = {
      0.0006f, 0.0007f, 0.0011f, 0.0013f, 0.0008f, 0.0010f, 0.0005f};
  Eigen::Vector3f delta_theta_samples[sample_count];
  Eigen::Vector3f delta_v_samples[sample_count];
  Eigen::Vector3f expected_delta_theta_total = Eigen::Vector3f::Zero();
  Eigen::Vector3f expected_delta_v_total = Eigen::Vector3f::Zero();
  float total_dt_s = 0.0f;
  for (int idx = 0; idx < sample_count; ++idx)
  {
    const float t_s = 0.0004f * static_cast<float>(idx + 1);
    const Eigen::Vector3f gyro =
        (Eigen::Vector3f() << 2.0f + 0.3f * std::sin(3.0f * t_s),
         -1.4f + 0.2f * std::cos(4.0f * t_s),
         0.9f + 0.1f * static_cast<float>(idx))
            .finished();
    const Eigen::Vector3f accel =
        (Eigen::Vector3f() << 1.0f + 0.2f * static_cast<float>(idx),
         -0.6f + 0.1f * std::sin(2.0f * t_s),
         -9.79f + 0.3f * std::cos(5.0f * t_s))
            .finished();
    delta_theta_samples[idx] = gyro * dt_samples[idx];
    delta_v_samples[idx] = accel * dt_samples[idx];
    expected_delta_theta_total += delta_theta_samples[idx];
    expected_delta_v_total += delta_v_samples[idx];
    total_dt_s += dt_samples[idx];
  }

  Eigen::Vector3f delta_theta_1;
  Eigen::Vector3f delta_theta_2;
  Eigen::Vector3f delta_v_1;
  Eigen::Vector3f delta_v_2;
  NavigationInputSplitMetrics metrics;
  splitImuDeltasLikeMainForTest(delta_theta_samples,
                                delta_v_samples,
                                dt_samples,
                                sample_count,
                                &delta_theta_1,
                                &delta_theta_2,
                                &delta_v_1,
                                &delta_v_2,
                                &metrics.first_half_dt_s,
                                &metrics.second_half_dt_s,
                                &metrics.boundary_cross_fraction);

  metrics.theta_split_error_rad =
      ((delta_theta_1 + delta_theta_2) - expected_delta_theta_total).norm();
  metrics.delta_v_split_error_mps =
      ((delta_v_1 + delta_v_2) - expected_delta_v_total).norm();

  // 总时长为6ms，目标中点为3ms，第四个样本跨中点，必须被按约0.4615比例拆开。
  assert(std::fabs(total_dt_s - 0.0060f) < 1.0e-7f);
  return metrics;
}

// 场景28模块：长时间无GNSS后出现短时假重捕获，验证保守重捕获不会接受明显位置台阶。
static GnssReacquisitionMetrics runFalseGnssReacquisitionRejectionScenario(
    const Eigen::Vector3f &accel,
    const Eigen::Vector3f &gyro,
    const Eigen::Vector3f &mag,
    const Eigen::Vector3f &ned_vel,
    const Eigen::Vector3d &lla,
    const Eigen::Vector3f &level_ypr)
{
  bfs::Ekf15State false_reacq_ekf;
  false_reacq_ekf.gyro_std_radps(0.0015f);
  false_reacq_ekf.gyro_markov_bias_std_radps(0.00001f);
  false_reacq_ekf.accel_std_mps2(0.25f);
  false_reacq_ekf.accel_markov_bias_std_mps2(0.05f);
  false_reacq_ekf.init_heading_err_std_rad(0.1f);
  false_reacq_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  GnssReacquisitionMetrics metrics;
  constexpr int false_reacq_steps = 2200; // 11秒，8秒后注入短时假GNSS恢复。
  const Eigen::Vector3f truth_vel = Eigen::Vector3f::Zero();

  for (int step = 1; step <= false_reacq_steps; ++step)
  {
    const float t_s = (step - 1) * 0.005f;
    Eigen::Vector3f imu_accel = accel;
    imu_accel(0) += deterministicNoise(step, 0.030f, 0.5f);
    imu_accel(1) += deterministicNoise(step, 0.030f, 1.5f);
    imu_accel(2) += deterministicNoise(step, 0.045f, 2.5f);
    Eigen::Vector3f imu_gyro;
    imu_gyro << deterministicNoise(step, 0.0007f, 0.4f),
        deterministicNoise(step, 0.0007f, 1.4f),
        deterministicNoise(step, 0.0009f, 2.4f);

    false_reacq_ekf.TimeUpdate(imu_accel, imu_gyro, 0.005f);
    if (shouldFuseAhrsAttitude(imu_accel, imu_gyro, false_reacq_ekf.quat()))
    {
      false_reacq_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.50f);
    }
    if ((step % 20) == 0)
    {
      false_reacq_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
      ++metrics.zupt_updates;
    }

    const bool false_gnss_window = (t_s >= 8.0f && t_s < 8.7f);
    if (false_gnss_window && ((step % 20) == 0))
    {
      const Eigen::Vector3d lla_before_gnss = false_reacq_ekf.lla_rad_m();
      const Eigen::Vector3d false_offset_ned =
          (Eigen::Vector3d() << 18.0, 7.0, 0.0).finished();
      Eigen::Vector3d false_lla = bfs::ned2lla(false_offset_ned, lla, bfs::AngPosUnit::RAD);
      Eigen::Vector3f false_vel = Eigen::Vector3f::Zero();
      false_vel(0) += deterministicNoise(step, 0.08f, 0.7f);
      false_vel(1) += deterministicNoise(step, 0.08f, 1.7f);
      false_vel(2) += deterministicNoise(step, 0.10f, 2.7f);

      // 误重捕获使用低质量但有限的精度上报，模拟接收机刚恢复时的错误位置台阶。
      false_reacq_ekf.gnss_pos_ne_std_m(2.5f);
      false_reacq_ekf.gnss_pos_d_std_m(3.5f);
      false_reacq_ekf.gnss_vel_ne_std_mps(0.45f);
      false_reacq_ekf.gnss_vel_d_std_mps(0.60f);
      const bfs::MeasurementUpdateResult update_result =
          false_reacq_ekf.MeasurementUpdateDetailed(false_vel, false_lla, 0.005f);
      ++metrics.gnss_updates;

      const float reacq_jump_m = static_cast<float>(
          bfs::lla2ned(false_reacq_ekf.lla_rad_m(), lla_before_gnss, bfs::AngPosUnit::RAD).norm());
      metrics.max_reacq_jump_m = std::max(metrics.max_reacq_jump_m, reacq_jump_m);
      (void)update_result;
    }

    const Eigen::Vector3d err_ned =
        bfs::lla2ned(false_reacq_ekf.lla_rad_m(), lla, bfs::AngPosUnit::RAD);
    metrics.max_pos_err_m = std::max(metrics.max_pos_err_m,
                                     static_cast<float>(err_ned.head<2>().norm()));
    metrics.max_vel_err_mps = std::max(metrics.max_vel_err_mps,
                                       (false_reacq_ekf.ned_vel_mps() - truth_vel).norm());
    const float tilt_err_rad = std::sqrt(false_reacq_ekf.roll_rad() * false_reacq_ekf.roll_rad() +
                                         false_reacq_ekf.pitch_rad() * false_reacq_ekf.pitch_rad());
    metrics.max_tilt_err_rad = std::max(metrics.max_tilt_err_rad, tilt_err_rad);
    assertFiniteEkfState(false_reacq_ekf);
    assertHealthyEkfCovariance(false_reacq_ekf);
  }

  const Eigen::Vector3d final_err_ned =
      bfs::lla2ned(false_reacq_ekf.lla_rad_m(), lla, bfs::AngPosUnit::RAD);
  metrics.final_pos_err_m = static_cast<float>(final_err_ned.head<2>().norm());
  metrics.final_vel_err_mps = false_reacq_ekf.ned_vel_mps().norm();
  return metrics;
}

// EKF15状态估计器回归测试
// 覆盖速度估计收敛、姿态重力修正、AHRS姿态融合、GNSS噪声门限与数值稳定性。
int main()
{
  // 报告文件放在.pio生成目录中，记录本次主机仿真的场景级结果且不污染源码目录。
  std::ofstream report(".pio/ekf_host_regression_report.txt", std::ios::out | std::ios::trunc);
  assert(report.is_open());
  report << "EKF主机回归测试结果" << std::endl;
  // 报告头记录测试范围和指标单位，便于后续把不同版本仿真结果放在一起对比。
  report << "测试范围：无GNSS惯导退化、静止零速修正、AHRS姿态融合、复合姿态量测四元数误差收敛、GNSS延迟重传播、dt抖动延迟索引、EKF/AHRS输出切换连续性、无GNSS长时静止/机动退化、长时间GNSS丢失后低质量恢复、假GNSS重捕获拒绝、误ZUPT拒绝、假原点启动后GNSS重锚定、重锚定后延迟GNSS连续融合、假原点重锚定原点同步、首次GNSS重锚定相对位置时序、静止确认后AHRS姿态恢复、静止假阳性机动保护、GNSS数据新鲜度门控、GNSS输出迟滞、GNSS旧速度静止检测门控、状态包fix新鲜度门控、AnoCom地面站GNSS遥测新鲜度门控、GNSS 2D/3D边界门控、高角速度纯惯导姿态传播、姿态误差状态精确注入、过程噪声PSD传播、旋转比力高精度捷联积分、协方差离散化精度、重力梯度误差模型、WGS84正常重力模型、地球自转/科里奥利补偿、真正双子样圆锥/划摇补偿、双子样GNSS延迟回放、导航系转动补偿、静止初始化地球自转零偏分离、静止锁轴地球自转保留、AHRS航向噪声新鲜度、AHRS航向偏置校正新鲜度、协方差正半定健康、极小量测噪声防御、非法LLA防御、有限IMU离群防御、异常输入防御和Monte Carlo压力仿真" << std::endl;
  report << "指标单位：位置误差=m，速度误差=m/s，姿态误差=rad" << std::endl;

  // ========== 场景1：速度估计与零速修正 ==========
  bfs::Ekf15State ekf;

  // 模拟静止时机体Z轴朝下的加速度计读数（重力矢量）
  Eigen::Vector3f accel;
  accel << 0.0f, 0.0f, -9.80665f;

  // 静止，陀螺仪输出为零
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();

  // 模拟磁力计指北（机头朝北）
  Eigen::Vector3f mag;
  mag << 1.0f, 0.0f, 0.0f;

  // 初始速度为0
  Eigen::Vector3f ned_vel = Eigen::Vector3f::Zero();

  // 设置LLA坐标：长沙附近（28.2°N, 112.9°E, 海拔50m）
  Eigen::Vector3d lla;
  lla << 28.2 * kPi / 180.0, 112.9 * kPi / 180.0, 50.0;

  ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  // 模拟前向加速（机体X轴有0.2m/s²加速度），持续1秒
  // EKF积分后应产生非零的前向速度
  for (int i = 0; i < 200; ++i)
  {
    Eigen::Vector3f tilted_accel;
    tilted_accel << 0.20f, 0.0f, -9.80665f;
    ekf.TimeUpdate(tilted_accel, gyro, 0.005f);
  }

  // 验证：加速后速度模长应大于0.05 m/s
  const float speed_before = ekf.ned_vel_mps().norm();
  assert(speed_before > 0.05f);

  // 施加零速测量修正（模拟GPS报告零速），50%信任测量值
  // 修正后速度应明显降低
  for (int i = 0; i < 10; ++i)
  {
    ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
  }

  // 验证：速度修正后模长减小
  assert(ekf.ned_vel_mps().norm() < speed_before);
  // 验证：姿态角均为有限值
  assertFiniteEkfState(ekf);
  recordScenarioPass(report, "场景1：速度估计与零速修正");

  // ========== 场景2：姿态重力修正 ==========
  bfs::Ekf15State attitude_ekf;
  attitude_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  // 施加俯仰角速率（绕Y轴0.08 rad/s），使姿态偏离水平
  Eigen::Vector3f pitch_rate;
  pitch_rate << 0.0f, 0.08f, 0.0f;
  for (int i = 0; i < 200; ++i)
  {
    attitude_ekf.TimeUpdate(accel, pitch_rate, 0.005f);
  }

  // 验证：积分后俯仰角应偏离零位至少0.02 rad
  const float pitch_before = std::fabs(attitude_ekf.pitch_rad());
  assert(pitch_before > 0.02f);

  // 使用重力矢量作为姿态观测（加速度计读数暗示机身水平）
  // 修正后滤波器应将俯仰拉回水平
  for (int i = 0; i < 20; ++i)
  {
    attitude_ekf.MeasurementUpdateGravity(accel, 0.03f);
  }

  // 验证：重力修正后俯仰角收敛回水平
  assert(std::fabs(attitude_ekf.pitch_rad()) < pitch_before);
  assertFiniteEkfState(attitude_ekf);
  recordScenarioPass(report, "场景2：姿态重力修正");

  // ========== 场景3：AHRS融合航向修正 ==========
  bfs::Ekf15State ahrs_fused_ekf;
  ahrs_fused_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  for (int i = 0; i < 200; ++i)
  {
    ahrs_fused_ekf.TimeUpdate(accel, pitch_rate, 0.005f);
  }

  // 验证：俯仰角偏离零位
  const float pitch_before_ahrs = std::fabs(ahrs_fused_ekf.pitch_rad());
  assert(pitch_before_ahrs > 0.02f);

  // 施加水平姿态测量（YPR全零），俯仰方向高信任(0.04)，偏航方向低信任(0.35)
  // 验证AHRS融合模式下滤波器对分量级信任度的响应
  Eigen::Vector3f level_ypr = Eigen::Vector3f::Zero();
  for (int i = 0; i < 20; ++i)
  {
    ahrs_fused_ekf.MeasurementUpdateAttitude(level_ypr, 0.04f, 0.35f);
  }

  // 验证：AHRS姿态修正后俯仰角收敛
  assert(std::fabs(ahrs_fused_ekf.pitch_rad()) < pitch_before_ahrs);
  assertFiniteEkfState(ahrs_fused_ekf);
  recordScenarioPass(report, "场景3：AHRS融合航向修正");

  // ========== 场景4：GNSS速度观测方向与动态噪声生效 ==========
  bfs::Ekf15State gnss_loose_ekf;
  bfs::Ekf15State gnss_tight_ekf;
  gnss_loose_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  gnss_tight_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  // GNSS测量：沿N轴正向1 m/s的运动速度
  Eigen::Vector3f gnss_vel;
  gnss_vel << 1.0f, 0.0f, 0.0f;

  // 松耦合：设置高测量噪声（50m位置/5mps速度），滤波器对GNSS信任度低
  gnss_loose_ekf.gnss_pos_ne_std_m(50.0f);
  gnss_loose_ekf.gnss_pos_d_std_m(50.0f);
  gnss_loose_ekf.gnss_vel_ne_std_mps(5.0f);
  gnss_loose_ekf.gnss_vel_d_std_mps(5.0f);
  gnss_loose_ekf.MeasurementUpdate(gnss_vel, lla, 0.005f);

  // 紧耦合：设置低测量噪声（0.5m位置/0.05mps速度），滤波器对GNSS信任度高
  gnss_tight_ekf.gnss_pos_ne_std_m(0.5f);
  gnss_tight_ekf.gnss_pos_d_std_m(0.5f);
  gnss_tight_ekf.gnss_vel_ne_std_mps(0.05f);
  gnss_tight_ekf.gnss_vel_d_std_mps(0.05f);
  gnss_tight_ekf.MeasurementUpdate(gnss_vel, lla, 0.005f);

  // 验证：松耦合噪声高时速度修正量小，紧耦合噪声低时速度修正量大
  assert(gnss_loose_ekf.ned_vel_mps()(0) > 0.0f);
  assert(gnss_tight_ekf.ned_vel_mps()(0) > gnss_loose_ekf.ned_vel_mps()(0));
  assertFiniteEkfState(gnss_loose_ekf);
  assertFiniteEkfState(gnss_tight_ekf);
  recordScenarioPass(report, "场景4：GNSS速度观测方向与动态噪声生效");

  // ========== 场景5：长时间无GNSS预测 + 姿态约束 + ZUPT数值稳定性 ==========
  bfs::Ekf15State inertial_ekf;
  inertial_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  Eigen::Vector3f zero_ypr = Eigen::Vector3f::Zero();
  // 模拟10秒纯惯性导航（2000步×0.005s），无GNSS辅助
  // 每步施加水平姿态约束 + 每隔20步(0.1s)周期性零速修正(ZUPT)
  // 验证滤波器在长时无GNSS条件下不发散
  for (int i = 0; i < 2000; ++i)
  {
    inertial_ekf.TimeUpdate(accel, gyro, 0.005f);
    inertial_ekf.MeasurementUpdateAttitude(zero_ypr, 0.04f, 0.35f);
    if ((i % 20) == 0)
    {
      inertial_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
    }
  }

  // 验证：10秒后姿态仍接近水平，速度漂移控制在0.2 m/s以内
  assert(std::fabs(inertial_ekf.roll_rad()) < 0.05f);
  assert(std::fabs(inertial_ekf.pitch_rad()) < 0.05f);
  assert(inertial_ekf.ned_vel_mps().norm() < 0.2f);
  assertFiniteEkfState(inertial_ekf);
  recordScenarioPass(report, "场景5：长时间无GNSS预测 + 姿态约束 + ZUPT数值稳定性");

  // ========== 场景6：延迟GNSS速度修正必须重新传播到当前时刻 ==========
  bfs::Ekf15State delayed_ekf;
  delayed_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  // 预推0.2秒（40步×0.005s），模拟GNSS数据到达延迟
  for (int i = 0; i < 40; ++i)
  {
    delayed_ekf.TimeUpdate(accel, gyro, 0.005f);
  }

  // 位置噪声设高(100m)使其对结果影响可忽略，速度噪声设低(0.05mps)
  // GNSS报告沿N轴正方向3 m/s的可信速度，验证滤波器优先采信速度测量
  delayed_ekf.gnss_pos_ne_std_m(100.0f);
  delayed_ekf.gnss_pos_d_std_m(100.0f);
  delayed_ekf.gnss_vel_ne_std_mps(0.05f);
  delayed_ekf.gnss_vel_d_std_mps(0.05f);
  Eigen::Vector3f delayed_gnss_vel;
  delayed_gnss_vel << 3.0f, 0.0f, 0.0f;
  delayed_ekf.MeasurementUpdate(delayed_gnss_vel, lla, 0.005f);

  // 验证：N轴速度被修正到>1.5 m/s，且位置积分也推进到正方向
  const Eigen::Vector3d delayed_ned = bfs::lla2ned(delayed_ekf.lla_rad_m(), lla, bfs::AngPosUnit::RAD);
  assert(delayed_ekf.ned_vel_mps()(0) > 1.5f);
  assert(delayed_ned(0) > 0.12);
  assertFiniteEkfState(delayed_ekf);
  recordScenarioPass(report, "场景6：延迟GNSS速度修正必须重新传播到当前时刻");

  // ========== 场景7：理想直线轨迹延迟GNSS融合精度 ==========
  bfs::Ekf15State trajectory_ekf;
  trajectory_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  trajectory_ekf.gnss_pos_ne_std_m(0.2f);
  trajectory_ekf.gnss_pos_d_std_m(0.5f);
  trajectory_ekf.gnss_vel_ne_std_mps(0.05f);
  trajectory_ekf.gnss_vel_d_std_mps(0.05f);

  const float sim_dt = 0.005f;
  const int sim_steps = 400;
  const int gnss_period_steps = 20;
  const int gnss_delay_steps = 24;
  Eigen::Vector3d truth_lla = lla;
  Eigen::Vector3f truth_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3d truth_lla_hist[sim_steps + 1];
  Eigen::Vector3f truth_vel_hist[sim_steps + 1];
  truth_lla_hist[0] = truth_lla;
  truth_vel_hist[0] = truth_vel;

  for (int i = 1; i <= sim_steps; ++i)
  {
    const float sim_t = (i - 1) * sim_dt;
    const float north_accel = (sim_t < 0.5f) ? 1.0f : 0.0f;
    Eigen::Vector3f imu_accel;
    imu_accel << north_accel, 0.0f, -9.80665f;

    trajectory_ekf.TimeUpdate(imu_accel, gyro, sim_dt);
    truth_vel(0) += north_accel * sim_dt;
    truth_lla += (sim_dt * bfs::llarate(truth_vel.cast<double>(),
                                        truth_lla,
                                        bfs::AngPosUnit::RAD))
                     .cast<double>();

    truth_lla_hist[i] = truth_lla;
    truth_vel_hist[i] = truth_vel;

    if (i > gnss_delay_steps && ((i % gnss_period_steps) == 0))
    {
      const int meas_idx = i - gnss_delay_steps;
      trajectory_ekf.MeasurementUpdate(truth_vel_hist[meas_idx],
                                       truth_lla_hist[meas_idx],
                                       sim_dt);
    }
  }

  const Eigen::Vector3d trajectory_err_ned =
      bfs::lla2ned(trajectory_ekf.lla_rad_m(), truth_lla, bfs::AngPosUnit::RAD);
  assert(std::fabs(trajectory_ekf.ned_vel_mps()(0) - truth_vel(0)) < 0.15f);
  assert(trajectory_err_ned.head<2>().norm() < 0.35);
  assertFiniteEkfState(trajectory_ekf);
  recordScenarioPass(report, "场景7：理想直线轨迹延迟GNSS融合精度");

  // ========== 场景8：GNSS延迟回放不能抹掉近期姿态与气压修正 ==========
  bfs::Ekf15State replay_event_ekf;
  replay_event_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  replay_event_ekf.gnss_pos_ne_std_m(1000.0f);
  replay_event_ekf.gnss_pos_d_std_m(1000.0f);
  replay_event_ekf.gnss_vel_ne_std_mps(1000.0f);
  replay_event_ekf.gnss_vel_d_std_mps(1000.0f);

  Eigen::Vector3f strong_pitch_rate;
  strong_pitch_rate << 0.0f, 0.6f, 0.0f;
  for (int i = 0; i < 56; ++i)
  {
    replay_event_ekf.TimeUpdate(accel, strong_pitch_rate, 0.005f);
  }
  assert(std::fabs(replay_event_ekf.pitch_rad()) > 0.10f);

  for (int i = 0; i < 24; ++i)
  {
    replay_event_ekf.TimeUpdate(accel, gyro, 0.005f);
    replay_event_ekf.MeasurementUpdateAttitude(level_ypr, 0.01f, 3.0f);
  }

  const float pitch_before_delayed_gnss = std::fabs(replay_event_ekf.pitch_rad());
  assert(pitch_before_delayed_gnss < 0.03f);
  replay_event_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), lla, 0.005f);

  // 验证：GNSS延迟重传播后，120ms窗口内已经执行过的AHRS修正仍然保留。
  assert(std::fabs(replay_event_ekf.pitch_rad()) < 0.03f);
  assertFiniteEkfState(replay_event_ekf);

  bfs::Ekf15State baro_replay_ekf;
  baro_replay_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  baro_replay_ekf.gnss_pos_ne_std_m(1000.0f);
  baro_replay_ekf.gnss_pos_d_std_m(1000.0f);
  baro_replay_ekf.gnss_vel_ne_std_mps(1000.0f);
  baro_replay_ekf.gnss_vel_d_std_mps(1000.0f);
  for (int i = 0; i < 80; ++i)
  {
    baro_replay_ekf.TimeUpdate(accel, gyro, 0.005f);
  }
  const double baro_target_alt_m = baro_replay_ekf.lla_rad_m()(2) + 3.0;
  const bfs::MeasurementUpdateResult baro_result =
      baro_replay_ekf.MeasurementUpdateBaroAltitudeDetailed(
          static_cast<float>(baro_target_alt_m), 0.10f);
  assert(baro_result.fused);
  const double altitude_after_baro_m = baro_replay_ekf.lla_rad_m()(2);
  for (int i = 0; i < 3; ++i)
  {
    baro_replay_ekf.TimeUpdate(accel, gyro, 0.005f);
  }
  baro_replay_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), lla, 0.005f);
  assert(std::fabs(baro_replay_ekf.lla_rad_m()(2) - altitude_after_baro_m) < 0.20);
  assertFiniteEkfState(baro_replay_ekf);
  recordScenarioPass(report, "场景8：GNSS延迟回放不能抹掉近期姿态与气压修正");

  // ========== 场景9：贴近飞控任务节拍的INS/GNSS/AHRS/ZUPT闭环仿真 ==========
  bfs::Ekf15State project_ekf;
  project_ekf.gyro_std_radps(0.0015f);
  project_ekf.gyro_markov_bias_std_radps(0.00001f);
  project_ekf.gyro_tau_s(50.0f);
  project_ekf.accel_std_mps2(0.25f);
  project_ekf.accel_markov_bias_std_mps2(0.05f);
  project_ekf.accel_tau_s(100.0f);
  project_ekf.init_heading_err_std_rad(0.1f);
  project_ekf.gnss_pos_ne_std_m(0.5f);
  project_ekf.gnss_pos_d_std_m(0.5f);
  project_ekf.gnss_vel_ne_std_mps(0.05f);
  project_ekf.gnss_vel_d_std_mps(0.05f);
  project_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  const float imu_dt_s = 0.0005f;       // handleICM42688 约2kHz
  const float nav_dt_s = 0.005f;        // handleNavigationSystem 200Hz
  const int imu_per_nav = static_cast<int>(nav_dt_s / imu_dt_s + 0.5f);
  assert(imu_per_nav == 10);            // 确认测试复现2kHz降采样平均到200Hz
  const int sim_nav_steps = 1200;       // 6秒闭环仿真
  const int project_gnss_period_steps = 20;     // 10Hz GNSS
  const int project_delay_steps = 24;   // EKF内固定120ms延迟
  Eigen::Vector3f truth_vel_project = Eigen::Vector3f::Zero();
  Eigen::Vector3d truth_lla_project = lla;
  Eigen::Vector3d truth_lla_project_hist[sim_nav_steps + 1];
  Eigen::Vector3f truth_vel_project_hist[sim_nav_steps + 1];
  truth_lla_project_hist[0] = truth_lla_project;
  truth_vel_project_hist[0] = truth_vel_project;

  int ahrs_fused_count = 0;
  int ahrs_skipped_maneuver_count = 0;
  int zupt_count = 0;
  int gnss_update_count = 0;

  for (int step = 1; step <= sim_nav_steps; ++step)
  {
    const float t_s = (step - 1) * nav_dt_s;
    const bool static_phase = (t_s < 1.0f) || (t_s >= 5.0f);
    const bool maneuver_phase = (t_s >= 1.5f && t_s < 3.0f);
    Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
    if (t_s >= 1.0f && t_s < 1.5f)
    {
      truth_accel_ned << 0.6f, 0.0f, 0.0f;
    }
    else if (maneuver_phase)
    {
      truth_accel_ned << 2.4f, 0.0f, -1.0f;
    }
    else if (t_s >= 3.0f && t_s < 4.0f)
    {
      truth_accel_ned << -0.5f, 0.0f, 0.0f;
    }
    else if (static_phase)
    {
      truth_vel_project.setZero();
    }

    Eigen::Vector3f accel_sum = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_sum = Eigen::Vector3f::Zero();
    for (int sub = 0; sub < imu_per_nav; ++sub)
    {
      const int sample = (step - 1) * imu_per_nav + sub;
      const Eigen::Vector3f specific_force_body =
          truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();
      Eigen::Vector3f measured_accel = specific_force_body;
      measured_accel(0) += deterministicNoise(sample, 0.035f, 0.1f);
      measured_accel(1) += deterministicNoise(sample, 0.030f, 1.7f);
      measured_accel(2) += deterministicNoise(sample, 0.050f, 2.8f);

      Eigen::Vector3f measured_gyro;
      measured_gyro << deterministicNoise(sample, 0.0008f, 0.4f),
          deterministicNoise(sample, 0.0007f, 1.2f),
          deterministicNoise(sample, 0.0006f, 2.0f);

      accel_sum += measured_accel;
      gyro_sum += measured_gyro;
    }

    const Eigen::Vector3f avg_accel = accel_sum / static_cast<float>(imu_per_nav);
    const Eigen::Vector3f avg_gyro = gyro_sum / static_cast<float>(imu_per_nav);
    project_ekf.TimeUpdate(avg_accel, avg_gyro, nav_dt_s);

    const bool fuse_ahrs = shouldFuseAhrsAttitude(avg_accel, avg_gyro, project_ekf.quat());
    if (fuse_ahrs)
    {
      const float accel_norm_error = std::fabs(avg_accel.norm() - kLocalGravity);
      const float roll_pitch_noise_rad = linearInterpolateForTest(accel_norm_error,
                                                                  0.3f, 2.0f,
                                                                  0.04f, 0.30f);
      project_ekf.MeasurementUpdateAttitude(level_ypr, roll_pitch_noise_rad, 0.35f);
      ++ahrs_fused_count;
    }
    else if (maneuver_phase)
    {
      ++ahrs_skipped_maneuver_count;
    }

    if (static_phase && (step % 20) == 0)
    {
      project_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
      ++zupt_count;
    }

    truth_vel_project += truth_accel_ned * nav_dt_s;
    if (static_phase)
    {
      truth_vel_project.setZero();
    }
    truth_lla_project = integrateTruthLla(truth_lla_project, truth_vel_project, nav_dt_s);
    truth_lla_project_hist[step] = truth_lla_project;
    truth_vel_project_hist[step] = truth_vel_project;

    const bool gnss_available = (t_s >= 1.0f && t_s < 5.0f);
    if (gnss_available && step > project_delay_steps && ((step % project_gnss_period_steps) == 0))
    {
      const int meas_idx = step - project_delay_steps;
      Eigen::Vector3d delayed_lla = truth_lla_project_hist[meas_idx];
      Eigen::Vector3f delayed_vel = truth_vel_project_hist[meas_idx];
      delayed_lla += bfs::ned2lla((Eigen::Vector3d() << deterministicNoise(step, 0.08f, 0.6f),
                                                    deterministicNoise(step, 0.07f, 1.3f),
                                                    deterministicNoise(step, 0.12f, 2.1f))
                                      .finished(),
                                  lla,
                                  bfs::AngPosUnit::RAD) -
                     lla;
      delayed_vel(0) += deterministicNoise(step, 0.025f, 0.9f);
      delayed_vel(1) += deterministicNoise(step, 0.020f, 1.8f);
      delayed_vel(2) += deterministicNoise(step, 0.030f, 2.6f);

      project_ekf.gnss_pos_ne_std_m(0.15f);
      project_ekf.gnss_pos_d_std_m(0.25f);
      project_ekf.gnss_vel_ne_std_mps(0.05f);
      project_ekf.gnss_vel_d_std_mps(0.05f);
      project_ekf.MeasurementUpdate(delayed_vel, delayed_lla, nav_dt_s);
      ++gnss_update_count;
    }

    assertFiniteEkfState(project_ekf);
  }

  const Eigen::Vector3d project_err_ned =
      bfs::lla2ned(project_ekf.lla_rad_m(), truth_lla_project, bfs::AngPosUnit::RAD);
  assert(ahrs_fused_count > 300);
  assert(ahrs_skipped_maneuver_count > 100);
  assert(zupt_count >= 20);
  assert(gnss_update_count >= 30);
  assert(std::fabs(project_ekf.roll_rad()) < 0.08f);
  assert(std::fabs(project_ekf.pitch_rad()) < 0.08f);
  assert(project_ekf.ned_vel_mps().norm() < 0.35f);
  assert(project_err_ned.head<2>().norm() < 1.5);
  assertFiniteEkfState(project_ekf);
  recordScenarioPass(report, "场景9：贴近飞控任务节拍的INS/GNSS/AHRS/ZUPT闭环仿真");

  // ========== 场景10：GNSS离群值不能单次拉飞状态 ==========
  bfs::Ekf15State outlier_ekf;
  outlier_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  for (int i = 0; i < 60; ++i)
  {
    outlier_ekf.TimeUpdate(accel, gyro, 0.005f);
    outlier_ekf.MeasurementUpdateAttitude(level_ypr, 0.04f, 0.35f);
  }

  outlier_ekf.gnss_pos_ne_std_m(0.15f);
  outlier_ekf.gnss_pos_d_std_m(0.25f);
  outlier_ekf.gnss_vel_ne_std_mps(0.05f);
  outlier_ekf.gnss_vel_d_std_mps(0.05f);
  const Eigen::Vector3d outlier_lla =
      bfs::ned2lla((Eigen::Vector3d() << 100.0, -100.0, -20.0).finished(),
                   lla,
                   bfs::AngPosUnit::RAD);
  Eigen::Vector3f outlier_vel;
  outlier_vel << 50.0f, -50.0f, -10.0f;
  outlier_ekf.MeasurementUpdate(outlier_vel, outlier_lla, 0.005f);

  const Eigen::Vector3d outlier_err_ned =
      bfs::lla2ned(outlier_ekf.lla_rad_m(), lla, bfs::AngPosUnit::RAD);
  assert(outlier_err_ned.norm() < 8.0);
  assert(outlier_ekf.ned_vel_mps().norm() < 4.0f);
  assertFiniteEkfState(outlier_ekf);
  recordScenarioPass(report, "场景10：GNSS离群值不能单次拉飞状态");

  // ========== 场景11：无GNSS高动态阶段不误用AHRS且姿态不发散 ==========
  bfs::Ekf15State high_dyn_no_gnss_ekf;
  high_dyn_no_gnss_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  int high_dyn_fused_count = 0;
  int high_dyn_skipped_count = 0;
  for (int i = 0; i < 800; ++i)
  {
    const float t_s = i * 0.005f;
    Eigen::Vector3f maneuver_accel;
    maneuver_accel << 2.2f * std::sin(2.0f * t_s),
        1.6f * std::cos(1.3f * t_s),
        -9.80665f + 1.1f * std::sin(1.7f * t_s);
    Eigen::Vector3f maneuver_gyro;
    maneuver_gyro << 0.12f * std::sin(1.1f * t_s),
        0.10f * std::cos(1.5f * t_s),
        0.08f * std::sin(0.9f * t_s);

    high_dyn_no_gnss_ekf.TimeUpdate(maneuver_accel, maneuver_gyro, 0.005f);
    if (shouldFuseAhrsAttitude(maneuver_accel, maneuver_gyro, high_dyn_no_gnss_ekf.quat()))
    {
      high_dyn_no_gnss_ekf.MeasurementUpdateAttitude(level_ypr, 0.30f, 0.35f);
      ++high_dyn_fused_count;
    }
    else
    {
      ++high_dyn_skipped_count;
    }
    assertFiniteEkfState(high_dyn_no_gnss_ekf);
  }

  assert(high_dyn_skipped_count > high_dyn_fused_count);
  assert(std::fabs(high_dyn_no_gnss_ekf.roll_rad()) < 0.8f);
  assert(std::fabs(high_dyn_no_gnss_ekf.pitch_rad()) < 0.8f);
  assertFiniteEkfState(high_dyn_no_gnss_ekf);
  recordScenarioPass(report, "场景11：无GNSS高动态阶段不误用AHRS且姿态不发散");

  // ========== 场景12：GNSS丢失后恢复不能造成状态跳变 ==========
  bfs::Ekf15State reacq_ekf;
  reacq_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  reacq_ekf.gnss_pos_ne_std_m(0.3f);
  reacq_ekf.gnss_pos_d_std_m(0.5f);
  reacq_ekf.gnss_vel_ne_std_mps(0.08f);
  reacq_ekf.gnss_vel_d_std_mps(0.10f);

  Eigen::Vector3d reacq_truth_lla = lla;
  Eigen::Vector3f reacq_truth_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3d reacq_lla_hist[701];
  Eigen::Vector3f reacq_vel_hist[701];
  reacq_lla_hist[0] = reacq_truth_lla;
  reacq_vel_hist[0] = reacq_truth_vel;

  for (int i = 1; i <= 700; ++i)
  {
    const float t_s = (i - 1) * 0.005f;
    const bool gnss_phase = (t_s < 1.5f) || (t_s >= 2.8f);
    Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
    if (t_s < 1.0f)
    {
      truth_accel_ned << 0.4f, 0.0f, 0.0f;
    }
    else if (t_s >= 2.0f && t_s < 2.8f)
    {
      truth_accel_ned << -0.25f, 0.0f, 0.0f;
    }

    Eigen::Vector3f imu_accel_reacq =
        truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, 9.80665f).finished();
    imu_accel_reacq(0) += deterministicNoise(i, 0.025f, 0.2f);
    imu_accel_reacq(1) += deterministicNoise(i, 0.020f, 1.4f);
    imu_accel_reacq(2) += deterministicNoise(i, 0.030f, 2.2f);
    Eigen::Vector3f gyro_reacq;
    gyro_reacq << deterministicNoise(i, 0.0005f, 0.3f),
        deterministicNoise(i, 0.0005f, 1.0f),
        deterministicNoise(i, 0.0005f, 2.0f);

    reacq_ekf.TimeUpdate(imu_accel_reacq, gyro_reacq, 0.005f);
    if (shouldFuseAhrsAttitude(imu_accel_reacq, gyro_reacq, reacq_ekf.quat()))
    {
      reacq_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.35f);
    }

    reacq_truth_vel += truth_accel_ned * 0.005f;
    reacq_truth_lla = integrateTruthLla(reacq_truth_lla, reacq_truth_vel, 0.005f);
    reacq_lla_hist[i] = reacq_truth_lla;
    reacq_vel_hist[i] = reacq_truth_vel;

    if (gnss_phase && i > gnss_delay_steps && ((i % 20) == 0))
    {
      const int meas_idx = i - gnss_delay_steps;
      Eigen::Vector3d meas_lla = reacq_lla_hist[meas_idx];
      Eigen::Vector3f meas_vel = reacq_vel_hist[meas_idx];
      meas_lla += bfs::ned2lla((Eigen::Vector3d() << deterministicNoise(i, 0.10f, 0.4f),
                                                  deterministicNoise(i, 0.09f, 1.1f),
                                                  deterministicNoise(i, 0.15f, 1.9f))
                                    .finished(),
                                lla,
                                bfs::AngPosUnit::RAD) -
                  lla;
      meas_vel(0) += deterministicNoise(i, 0.025f, 0.8f);
      meas_vel(1) += deterministicNoise(i, 0.020f, 1.6f);
      meas_vel(2) += deterministicNoise(i, 0.030f, 2.4f);
      reacq_ekf.MeasurementUpdate(meas_vel, meas_lla, 0.005f);
    }

    assertFiniteEkfState(reacq_ekf);
  }

  const Eigen::Vector3d reacq_err_ned =
      bfs::lla2ned(reacq_ekf.lla_rad_m(), reacq_truth_lla, bfs::AngPosUnit::RAD);
  assert(reacq_err_ned.head<2>().norm() < 1.2);
  assert((reacq_ekf.ned_vel_mps() - reacq_truth_vel).norm() < 0.35f);
  assert(std::fabs(reacq_ekf.roll_rad()) < 0.08f);
  assert(std::fabs(reacq_ekf.pitch_rad()) < 0.08f);
  assertFiniteEkfState(reacq_ekf);
  recordScenarioPass(report, "场景12：GNSS丢失后恢复不能造成状态跳变");

  // ========== 场景13：异常dt输入应被EKF底层防御 ==========
  bfs::Ekf15State dt_guard_ekf;
  dt_guard_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  Eigen::Vector3f dt_guard_accel;
  dt_guard_accel << 1.0f, 0.0f, -9.80665f;
  dt_guard_ekf.TimeUpdate(dt_guard_accel, gyro, 0.005f);
  // 记录异常dt注入前的状态快照，验证非法时间步不会推进状态或污染历史缓冲。
  const Eigen::Vector3f vel_before_bad_dt = dt_guard_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_bad_dt = dt_guard_ekf.lla_rad_m();
  const Eigen::Quaternionf quat_before_bad_dt = dt_guard_ekf.quat();

  // 负dt和过大dt都代表调度/时间戳异常，EKF底层应直接丢弃而不是积分。
  dt_guard_ekf.TimeUpdate(dt_guard_accel, gyro, -0.02f);
  dt_guard_ekf.TimeUpdate(dt_guard_accel, gyro, 0.25f);

  assert((dt_guard_ekf.ned_vel_mps() - vel_before_bad_dt).norm() < 1e-5f);
  assert(bfs::lla2ned(dt_guard_ekf.lla_rad_m(), lla_before_bad_dt, bfs::AngPosUnit::RAD).norm() < 1e-4);
  assert(std::fabs(dt_guard_ekf.quat().dot(quat_before_bad_dt)) > 0.99999f);
  assertFiniteEkfState(dt_guard_ekf);
  recordScenarioPass(report, "场景13：异常dt输入应被EKF底层防御");

  // ========== 场景14：异常量测和噪声参数不应污染EKF状态 ==========
  bfs::Ekf15State measurement_guard_ekf;
  measurement_guard_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  for (int i = 0; i < 40; ++i)
  {
    // 先填充足够历史快照，验证坏GNSS量测在延迟回放路径下也不会污染当前状态。
    measurement_guard_ekf.TimeUpdate(accel, gyro, 0.005f);
  }

  const float saved_pos_ne_noise = measurement_guard_ekf.gnss_pos_ne_std_m();
  const float saved_pos_d_noise = measurement_guard_ekf.gnss_pos_d_std_m();
  const float saved_vel_ne_noise = measurement_guard_ekf.gnss_vel_ne_std_mps();
  const float saved_vel_d_noise = measurement_guard_ekf.gnss_vel_d_std_mps();

  // GNSS噪声配置如果收到NaN、负值、零或Inf，应保持上一次可信配置。
  measurement_guard_ekf.gnss_pos_ne_std_m(std::numeric_limits<float>::quiet_NaN());
  measurement_guard_ekf.gnss_pos_d_std_m(-1.0f);
  measurement_guard_ekf.gnss_vel_ne_std_mps(0.0f);
  measurement_guard_ekf.gnss_vel_d_std_mps(std::numeric_limits<float>::infinity());
  assert(std::fabs(measurement_guard_ekf.gnss_pos_ne_std_m() - saved_pos_ne_noise) < 1e-6f);
  assert(std::fabs(measurement_guard_ekf.gnss_pos_d_std_m() - saved_pos_d_noise) < 1e-6f);
  assert(std::fabs(measurement_guard_ekf.gnss_vel_ne_std_mps() - saved_vel_ne_noise) < 1e-6f);
  assert(std::fabs(measurement_guard_ekf.gnss_vel_d_std_mps() - saved_vel_d_noise) < 1e-6f);

  const Eigen::Vector3f vel_before_bad_measurement = measurement_guard_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_bad_measurement = measurement_guard_ekf.lla_rad_m();
  const Eigen::Quaternionf quat_before_bad_measurement = measurement_guard_ekf.quat();

  Eigen::Vector3f bad_vel_measurement;
  bad_vel_measurement << std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f;
  Eigen::Vector3d bad_lla_measurement = lla;
  bad_lla_measurement(1) = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3f bad_accel_measurement = accel;
  bad_accel_measurement(2) = std::numeric_limits<float>::quiet_NaN();
  Eigen::Vector3f bad_ypr_measurement = Eigen::Vector3f::Zero();
  bad_ypr_measurement(0) = std::numeric_limits<float>::infinity();

  // 各类异常量测或异常量测噪声都应被EKF底层忽略，不能把NaN写入状态。
  measurement_guard_ekf.MeasurementUpdate(bad_vel_measurement, bad_lla_measurement, 0.005f);
  measurement_guard_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), lla, 0.0f);
  measurement_guard_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(),
                                                  std::numeric_limits<float>::quiet_NaN(),
                                                  0.05f);
  measurement_guard_ekf.MeasurementUpdateGravity(bad_accel_measurement, 0.03f);
  measurement_guard_ekf.MeasurementUpdateAttitude(bad_ypr_measurement, 0.04f, 0.35f);
  measurement_guard_ekf.MeasurementUpdateYaw(std::numeric_limits<float>::quiet_NaN(), 0.35f);

  assert((measurement_guard_ekf.ned_vel_mps() - vel_before_bad_measurement).norm() < 1e-5f);
  assert(bfs::lla2ned(measurement_guard_ekf.lla_rad_m(),
                      lla_before_bad_measurement,
                      bfs::AngPosUnit::RAD)
             .norm() < 1e-4);
  assert(std::fabs(measurement_guard_ekf.quat().dot(quat_before_bad_measurement)) > 0.99999f);
  assertFiniteEkfState(measurement_guard_ekf);
  recordScenarioPass(report, "场景14：异常量测和噪声参数不应污染EKF状态");

  // ========== 场景15：异常初始化参数和滤波器配置不应造成启动发散 ==========
  bfs::Ekf15State init_guard_ekf;
  const float saved_accel_std = init_guard_ekf.accel_std_mps2();
  const float saved_accel_bias_std = init_guard_ekf.accel_markov_bias_std_mps2();
  const float saved_accel_tau = init_guard_ekf.accel_tau_s();
  const float saved_gyro_std = init_guard_ekf.gyro_std_radps();
  const float saved_gyro_bias_std = init_guard_ekf.gyro_markov_bias_std_radps();
  const float saved_gyro_tau = init_guard_ekf.gyro_tau_s();
  const float saved_init_pos_std = init_guard_ekf.init_pos_err_std_m();
  const float saved_init_vel_std = init_guard_ekf.init_vel_err_std_mps();
  const float saved_init_att_std = init_guard_ekf.init_att_err_std_rad();
  const float saved_init_heading_std = init_guard_ekf.init_heading_err_std_rad();
  const float saved_init_accel_bias_std = init_guard_ekf.init_accel_bias_std_mps2();
  const float saved_init_gyro_bias_std = init_guard_ekf.init_gyro_bias_std_radps();

  // 非法滤波器配置应被setter拒绝，避免Initialize阶段构造出NaN/Inf过程噪声或初始协方差。
  init_guard_ekf.accel_std_mps2(std::numeric_limits<float>::quiet_NaN());
  init_guard_ekf.accel_markov_bias_std_mps2(-0.1f);
  init_guard_ekf.accel_tau_s(0.0f);
  init_guard_ekf.gyro_std_radps(std::numeric_limits<float>::infinity());
  init_guard_ekf.gyro_markov_bias_std_radps(-0.1f);
  init_guard_ekf.gyro_tau_s(0.0f);
  init_guard_ekf.init_pos_err_std_m(std::numeric_limits<float>::quiet_NaN());
  init_guard_ekf.init_vel_err_std_mps(-1.0f);
  init_guard_ekf.init_att_err_std_rad(0.0f);
  init_guard_ekf.init_heading_err_std_rad(std::numeric_limits<float>::infinity());
  init_guard_ekf.init_accel_bias_std_mps2(-1.0f);
  init_guard_ekf.init_gyro_bias_std_radps(0.0f);

  assert(std::fabs(init_guard_ekf.accel_std_mps2() - saved_accel_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.accel_markov_bias_std_mps2() - saved_accel_bias_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.accel_tau_s() - saved_accel_tau) < 1e-6f);
  assert(std::fabs(init_guard_ekf.gyro_std_radps() - saved_gyro_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.gyro_markov_bias_std_radps() - saved_gyro_bias_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.gyro_tau_s() - saved_gyro_tau) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_pos_err_std_m() - saved_init_pos_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_vel_err_std_mps() - saved_init_vel_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_att_err_std_rad() - saved_init_att_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_heading_err_std_rad() - saved_init_heading_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_accel_bias_std_mps2() - saved_init_accel_bias_std) < 1e-6f);
  assert(std::fabs(init_guard_ekf.init_gyro_bias_std_radps() - saved_init_gyro_bias_std) < 1e-6f);

  Eigen::Vector3f bad_init_accel;
  bad_init_accel << std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f;
  Eigen::Vector3f bad_init_gyro;
  bad_init_gyro << 0.0f, std::numeric_limits<float>::infinity(), 0.0f;
  Eigen::Vector3f bad_init_mag = Eigen::Vector3f::Zero();
  Eigen::Vector3f bad_init_vel;
  bad_init_vel << 0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f;
  Eigen::Vector3d bad_init_lla = lla;
  bad_init_lla(0) = std::numeric_limits<double>::quiet_NaN();

  // 初始化传感器输入异常时，EKF应使用保守默认值完成启动，并保证随后预测仍为有限状态。
  init_guard_ekf.Initialize(bad_init_accel, bad_init_gyro, bad_init_mag, bad_init_vel, bad_init_lla);
  assertFiniteEkfState(init_guard_ekf);
  init_guard_ekf.TimeUpdate(accel, gyro, 0.005f);
  init_guard_ekf.MeasurementUpdateAttitude(level_ypr, 0.04f, 0.35f);
  assertFiniteEkfState(init_guard_ekf);
  recordScenarioPass(report, "场景15：异常初始化参数和滤波器配置不应造成启动发散");

  // ========== 场景16：运行期异常IMU输入不应污染惯导状态 ==========
  bfs::Ekf15State imu_guard_ekf;
  imu_guard_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  imu_guard_ekf.TimeUpdate(accel, gyro, 0.005f);
  const Eigen::Vector3f vel_before_bad_imu = imu_guard_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_bad_imu = imu_guard_ekf.lla_rad_m();
  const Eigen::Quaternionf quat_before_bad_imu = imu_guard_ekf.quat();

  Eigen::Vector3f bad_runtime_accel = accel;
  bad_runtime_accel(0) = std::numeric_limits<float>::quiet_NaN();
  Eigen::Vector3f bad_runtime_gyro = gyro;
  bad_runtime_gyro(1) = std::numeric_limits<float>::infinity();

  // 运行期IMU单帧异常应被TimeUpdate直接丢弃，不能写入状态或GNSS延迟历史缓冲。
  imu_guard_ekf.TimeUpdate(bad_runtime_accel, gyro, 0.005f);
  imu_guard_ekf.TimeUpdate(accel, bad_runtime_gyro, 0.005f);

  assert((imu_guard_ekf.ned_vel_mps() - vel_before_bad_imu).norm() < 1e-5f);
  assert(bfs::lla2ned(imu_guard_ekf.lla_rad_m(),
                      lla_before_bad_imu,
                      bfs::AngPosUnit::RAD)
             .norm() < 1e-4);
  assert(std::fabs(imu_guard_ekf.quat().dot(quat_before_bad_imu)) > 0.99999f);
  assertFiniteEkfState(imu_guard_ekf);

  // 坏IMU帧之后再输入正常IMU，EKF仍应能继续预测，证明防御没有永久锁死导航。
  imu_guard_ekf.TimeUpdate(accel, gyro, 0.005f);
  assertFiniteEkfState(imu_guard_ekf);
  recordScenarioPass(report, "场景16：运行期异常IMU输入不应污染惯导状态");

  // ========== 场景17：多seed长时组合导航Monte Carlo压力仿真 ==========
  float mc_max_pos_err_m = 0.0f;
  float mc_max_vel_err_mps = 0.0f;
  float mc_max_tilt_err_rad = 0.0f;
  int mc_total_gnss_updates = 0;
  int mc_total_zupt_updates = 0;
  int mc_total_ahrs_fused = 0;
  int mc_total_ahrs_skipped = 0;

  for (int seed = 0; seed < 6; ++seed)
  {
    bfs::Ekf15State mc_ekf;
    mc_ekf.gyro_std_radps(0.0015f);
    mc_ekf.gyro_markov_bias_std_radps(0.00001f);
    mc_ekf.gyro_tau_s(50.0f);
    mc_ekf.accel_std_mps2(0.25f);
    mc_ekf.accel_markov_bias_std_mps2(0.05f);
    mc_ekf.accel_tau_s(100.0f);
    mc_ekf.init_heading_err_std_rad(0.1f);
    mc_ekf.gnss_pos_ne_std_m(0.6f);
    mc_ekf.gnss_pos_d_std_m(0.8f);
    mc_ekf.gnss_vel_ne_std_mps(0.08f);
    mc_ekf.gnss_vel_d_std_mps(0.10f);
    mc_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

    const int mc_steps = 1800;           // 9秒，覆盖静止、机动、GNSS丢失与恢复。
    const int mc_delay_steps = 24;       // 与EKF固定120ms延迟一致。
    const int mc_gnss_period_steps = 20; // 10Hz GNSS。
    Eigen::Vector3d mc_truth_lla = lla;
    Eigen::Vector3f mc_truth_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3d mc_lla_hist[mc_steps + 1];
    Eigen::Vector3f mc_vel_hist[mc_steps + 1];
    mc_lla_hist[0] = mc_truth_lla;
    mc_vel_hist[0] = mc_truth_vel;

    const Eigen::Vector3f accel_bias =
        (Eigen::Vector3f() << deterministicNoise(seed, 0.035f, 0.2f),
         deterministicNoise(seed, 0.030f, 1.1f),
         deterministicNoise(seed, 0.045f, 2.0f))
            .finished();
    const Eigen::Vector3f gyro_bias =
        (Eigen::Vector3f() << deterministicNoise(seed, 0.0007f, 0.5f),
         deterministicNoise(seed, 0.0006f, 1.3f),
         deterministicNoise(seed, 0.0008f, 2.4f))
            .finished();

    int mc_gnss_updates = 0;
    int mc_zupt_updates = 0;
    int mc_ahrs_fused = 0;
    int mc_ahrs_skipped = 0;

    for (int step = 1; step <= mc_steps; ++step)
    {
      const float t_s = (step - 1) * 0.005f;
      const bool static_phase = (t_s < 1.0f) || (t_s >= 7.4f);
      const bool gnss_available = ((t_s >= 1.0f && t_s < 3.4f) ||
                                   (t_s >= 5.0f && t_s < 7.6f));
      Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
      if (t_s >= 1.0f && t_s < 2.0f)
      {
        truth_accel_ned << 0.55f + 0.04f * seed, 0.10f, 0.0f;
      }
      else if (t_s >= 2.0f && t_s < 3.2f)
      {
        truth_accel_ned << 0.35f * std::sin(2.0f * t_s),
            0.30f * std::cos(1.7f * t_s),
            0.15f * std::sin(1.1f * t_s);
      }
      else if (t_s >= 3.2f && t_s < 4.8f)
      {
        truth_accel_ned << 1.9f * std::sin(1.5f * t_s + seed),
            1.4f * std::cos(1.2f * t_s),
            -0.6f + 0.5f * std::sin(1.8f * t_s);
      }
      else if (t_s >= 5.0f && t_s < 6.4f)
      {
        truth_accel_ned << -0.45f, -0.08f, 0.0f;
      }
      else if (static_phase)
      {
        mc_truth_vel.setZero();
      }

      Eigen::Vector3f imu_accel_mc =
          truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished() +
          accel_bias;
      imu_accel_mc(0) += deterministicNoise(step + seed * 1000, 0.055f, 0.4f);
      imu_accel_mc(1) += deterministicNoise(step + seed * 1000, 0.050f, 1.4f);
      imu_accel_mc(2) += deterministicNoise(step + seed * 1000, 0.080f, 2.4f);

      Eigen::Vector3f imu_gyro_mc = gyro_bias;
      imu_gyro_mc(0) += deterministicNoise(step + seed * 1000, 0.0010f, 0.7f);
      imu_gyro_mc(1) += deterministicNoise(step + seed * 1000, 0.0010f, 1.5f);
      imu_gyro_mc(2) += deterministicNoise(step + seed * 1000, 0.0012f, 2.5f);

      mc_ekf.TimeUpdate(imu_accel_mc, imu_gyro_mc, 0.005f);

      const bool fuse_mc_ahrs = shouldFuseAhrsAttitude(imu_accel_mc, imu_gyro_mc, mc_ekf.quat());
      if (fuse_mc_ahrs)
      {
        const float accel_norm_error = std::fabs(imu_accel_mc.norm() - kLocalGravity);
        const float roll_pitch_noise_rad = linearInterpolateForTest(accel_norm_error,
                                                                    0.3f, 2.0f,
                                                                    0.05f, 0.35f);
        mc_ekf.MeasurementUpdateAttitude(level_ypr, roll_pitch_noise_rad, 0.50f);
        ++mc_ahrs_fused;
      }
      else
      {
        ++mc_ahrs_skipped;
      }

      if (static_phase && (step % 20) == 0)
      {
        mc_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
        ++mc_zupt_updates;
      }

      mc_truth_vel += truth_accel_ned * 0.005f;
      if (static_phase)
      {
        mc_truth_vel.setZero();
      }
      mc_truth_lla = integrateTruthLla(mc_truth_lla, mc_truth_vel, 0.005f);
      mc_lla_hist[step] = mc_truth_lla;
      mc_vel_hist[step] = mc_truth_vel;

      if (gnss_available && step > mc_delay_steps && ((step % mc_gnss_period_steps) == 0))
      {
        const int meas_idx = step - mc_delay_steps;
        Eigen::Vector3d meas_lla = mc_lla_hist[meas_idx];
        Eigen::Vector3f meas_vel = mc_vel_hist[meas_idx];
        meas_lla += bfs::ned2lla((Eigen::Vector3d() << deterministicNoise(step + seed * 111, 0.18f, 0.6f),
                                                    deterministicNoise(step + seed * 111, 0.16f, 1.6f),
                                                    deterministicNoise(step + seed * 111, 0.25f, 2.6f))
                                      .finished(),
                                  lla,
                                  bfs::AngPosUnit::RAD) -
                    lla;
        meas_vel(0) += deterministicNoise(step + seed * 222, 0.050f, 0.9f);
        meas_vel(1) += deterministicNoise(step + seed * 222, 0.045f, 1.9f);
        meas_vel(2) += deterministicNoise(step + seed * 222, 0.060f, 2.9f);

        mc_ekf.gnss_pos_ne_std_m(0.35f);
        mc_ekf.gnss_pos_d_std_m(0.45f);
        mc_ekf.gnss_vel_ne_std_mps(0.08f);
        mc_ekf.gnss_vel_d_std_mps(0.10f);
        mc_ekf.MeasurementUpdate(meas_vel, meas_lla, 0.005f);
        ++mc_gnss_updates;
      }

      assertFiniteEkfState(mc_ekf);
    }

    const Eigen::Vector3d mc_err_ned =
        bfs::lla2ned(mc_ekf.lla_rad_m(), mc_truth_lla, bfs::AngPosUnit::RAD);
    const float mc_pos_err = static_cast<float>(mc_err_ned.head<2>().norm());
    const float mc_vel_err = (mc_ekf.ned_vel_mps() - mc_truth_vel).norm();
    const float mc_tilt_err = std::sqrt(mc_ekf.roll_rad() * mc_ekf.roll_rad() +
                                        mc_ekf.pitch_rad() * mc_ekf.pitch_rad());

    mc_max_pos_err_m = std::max(mc_max_pos_err_m, mc_pos_err);
    mc_max_vel_err_mps = std::max(mc_max_vel_err_mps, mc_vel_err);
    mc_max_tilt_err_rad = std::max(mc_max_tilt_err_rad, mc_tilt_err);
    mc_total_gnss_updates += mc_gnss_updates;
    mc_total_zupt_updates += mc_zupt_updates;
    mc_total_ahrs_fused += mc_ahrs_fused;
    mc_total_ahrs_skipped += mc_ahrs_skipped;

    assert(mc_gnss_updates >= 20);
    assert(mc_zupt_updates >= 20);
    assert(mc_ahrs_fused > 100);
    assert(mc_ahrs_skipped > 100);
    assert(mc_pos_err < 2.5f);
    assert(mc_vel_err < 0.65f);
    assert(mc_tilt_err < 0.18f);
  }

  assert(mc_total_gnss_updates >= 120);
  assert(mc_total_zupt_updates >= 120);
  assert(mc_total_ahrs_fused > 600);
  assert(mc_total_ahrs_skipped > 600);
  recordScenarioPassWithMetrics(report,
                                "场景17：多seed长时组合导航Monte Carlo压力仿真",
                                mc_max_pos_err_m,
                                mc_max_vel_err_mps,
                                mc_max_tilt_err_rad);

  // ========== 场景18：有限值离群量测不应单帧拉飞状态 ==========
  bfs::Ekf15State finite_outlier_ekf;
  finite_outlier_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  for (int i = 0; i < 80; ++i)
  {
    // 先让滤波器在正常静止状态下积累历史和较小姿态误差。
    finite_outlier_ekf.TimeUpdate(accel, gyro, 0.005f);
    finite_outlier_ekf.MeasurementUpdateAttitude(level_ypr, 0.04f, 0.35f);
    if ((i % 20) == 0)
    {
      finite_outlier_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
    }
  }

  const Eigen::Vector3f vel_before_finite_outlier = finite_outlier_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_finite_outlier = finite_outlier_ekf.lla_rad_m();
  const Eigen::Quaternionf quat_before_finite_outlier = finite_outlier_ekf.quat();

  Eigen::Vector3f bad_finite_vel;
  bad_finite_vel << 12.0f, -8.0f, 5.0f;
  Eigen::Vector3f bad_finite_accel;
  bad_finite_accel << kLocalGravity, 0.0f, 0.0f; // 有限但方向明显错误的“重力”观测。
  Eigen::Vector3f bad_finite_ypr;
  bad_finite_ypr << 2.6f, 1.2f, -1.1f;

  // 这些量测都是有限值，但与当前状态强烈不一致，应按离群量测拒绝。
  finite_outlier_ekf.MeasurementUpdateVelocity(bad_finite_vel, 0.05f, 0.05f);
  finite_outlier_ekf.MeasurementUpdateGravity(bad_finite_accel, 0.03f);
  finite_outlier_ekf.MeasurementUpdateAttitude(bad_finite_ypr, 0.04f, 0.35f);
  finite_outlier_ekf.MeasurementUpdateYaw(2.8f, 0.05f);

  assert((finite_outlier_ekf.ned_vel_mps() - vel_before_finite_outlier).norm() < 0.10f);
  assert(bfs::lla2ned(finite_outlier_ekf.lla_rad_m(),
                      lla_before_finite_outlier,
                      bfs::AngPosUnit::RAD)
             .norm() < 0.10);
  assert(std::fabs(finite_outlier_ekf.quat().dot(quat_before_finite_outlier)) > 0.995f);
  assertFiniteEkfState(finite_outlier_ekf);
  recordScenarioPass(report, "场景18：有限值离群量测不应单帧拉飞状态");

  // ========== 场景19：被门控拒绝的非GNSS量测不能进入延迟回放 ==========
  bfs::Ekf15State rejected_replay_ekf;
  rejected_replay_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  rejected_replay_ekf.gnss_pos_ne_std_m(1000.0f);
  rejected_replay_ekf.gnss_pos_d_std_m(1000.0f);
  rejected_replay_ekf.gnss_vel_ne_std_mps(1000.0f);
  rejected_replay_ekf.gnss_vel_d_std_mps(1000.0f);

  for (int i = 0; i < 56; ++i)
  {
    rejected_replay_ekf.TimeUpdate(accel, gyro, 0.005f);
  }

  const Eigen::Vector3f vel_before_rejected_replay = rejected_replay_ekf.ned_vel_mps();
  const Eigen::Quaternionf quat_before_rejected_replay = rejected_replay_ekf.quat();

  // 在GNSS延迟窗口内注入会被门控拒绝的非GNSS量测；它们不能被记录到历史事件里。
  for (int i = 0; i < 24; ++i)
  {
    rejected_replay_ekf.TimeUpdate(accel, gyro, 0.005f);
    rejected_replay_ekf.MeasurementUpdateVelocity(bad_finite_vel, 0.05f, 0.05f);
    rejected_replay_ekf.MeasurementUpdateAttitude(bad_finite_ypr, 0.04f, 0.35f);
    rejected_replay_ekf.MeasurementUpdateYaw(2.8f, 0.05f);
  }

  assert((rejected_replay_ekf.ned_vel_mps() - vel_before_rejected_replay).norm() < 0.10f);
  assert(std::fabs(rejected_replay_ekf.quat().dot(quat_before_rejected_replay)) > 0.995f);

  // 触发GNSS延迟重传播；如果被拒绝量测被错误记录，回放后速度或姿态会再次被拉飞。
  rejected_replay_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), lla, 0.005f);
  assert((rejected_replay_ekf.ned_vel_mps() - vel_before_rejected_replay).norm() < 0.10f);
  assert(std::fabs(rejected_replay_ekf.quat().dot(quat_before_rejected_replay)) > 0.995f);
  assertFiniteEkfState(rejected_replay_ekf);
  recordScenarioPass(report, "场景19：被门控拒绝的非GNSS量测不能进入延迟回放");

  // ========== 场景20：连续融合后协方差矩阵必须保持健康 ==========
  bfs::Ekf15State covariance_health_ekf;
  covariance_health_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  covariance_health_ekf.gnss_pos_ne_std_m(0.35f);
  covariance_health_ekf.gnss_pos_d_std_m(0.55f);
  covariance_health_ekf.gnss_vel_ne_std_mps(0.08f);
  covariance_health_ekf.gnss_vel_d_std_mps(0.10f);

  Eigen::Vector3d covariance_truth_lla = lla;
  Eigen::Vector3f covariance_truth_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3d covariance_lla_hist[901];
  Eigen::Vector3f covariance_vel_hist[901];
  covariance_lla_hist[0] = covariance_truth_lla;
  covariance_vel_hist[0] = covariance_truth_vel;

  for (int i = 1; i <= 900; ++i)
  {
    const float t_s = static_cast<float>(i - 1) * 0.005f;
    Eigen::Vector3f truth_accel = Eigen::Vector3f::Zero();
    if (t_s >= 0.8f && t_s < 2.1f)
    {
      truth_accel << 0.45f, 0.12f, 0.0f;
    }
    else if (t_s >= 2.1f && t_s < 3.0f)
    {
      truth_accel << -0.25f, -0.10f, 0.0f;
    }

    Eigen::Vector3f imu_accel =
        truth_accel - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();
    imu_accel(0) += deterministicNoise(i, 0.040f, 0.5f);
    imu_accel(1) += deterministicNoise(i, 0.035f, 1.5f);
    imu_accel(2) += deterministicNoise(i, 0.055f, 2.5f);
    Eigen::Vector3f imu_gyro;
    imu_gyro << deterministicNoise(i, 0.0009f, 0.2f),
        deterministicNoise(i, 0.0008f, 1.2f),
        deterministicNoise(i, 0.0010f, 2.2f);

    covariance_health_ekf.TimeUpdate(imu_accel, imu_gyro, 0.005f);
    if (shouldFuseAhrsAttitude(imu_accel, imu_gyro, covariance_health_ekf.quat()))
    {
      covariance_health_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.45f);
    }
    if ((t_s < 0.8f || t_s >= 3.4f) && ((i % 20) == 0))
    {
      covariance_health_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
    }

    covariance_truth_vel += truth_accel * 0.005f;
    if (t_s < 0.8f || t_s >= 3.4f)
    {
      covariance_truth_vel.setZero();
    }
    covariance_truth_lla = integrateTruthLla(covariance_truth_lla, covariance_truth_vel, 0.005f);
    covariance_lla_hist[i] = covariance_truth_lla;
    covariance_vel_hist[i] = covariance_truth_vel;

    if (i > 24 && (i % 20) == 0)
    {
      const int meas_idx = i - 24;
      covariance_health_ekf.MeasurementUpdate(covariance_vel_hist[meas_idx],
                                              covariance_lla_hist[meas_idx],
                                              0.005f);
    }
    if ((i % 150) == 0)
    {
      // 周期性注入会被门控拒绝的量测，验证拒绝路径不会破坏协方差。
      covariance_health_ekf.MeasurementUpdateVelocity(bad_finite_vel, 0.05f, 0.05f);
      covariance_health_ekf.MeasurementUpdateYaw(2.8f, 0.05f);
    }

    assertFiniteEkfState(covariance_health_ekf);
    assertHealthyEkfCovariance(covariance_health_ekf);
  }
  recordScenarioPass(report, "场景20：连续融合后协方差矩阵必须保持健康");

  // ========== 场景21：GNSS延迟索引必须适应导航周期抖动 ==========
  bfs::Ekf15State jitter_delay_ekf;
  jitter_delay_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  jitter_delay_ekf.gnss_pos_ne_std_m(1000.0f);
  jitter_delay_ekf.gnss_pos_d_std_m(1000.0f);
  jitter_delay_ekf.gnss_vel_ne_std_mps(0.05f);
  jitter_delay_ekf.gnss_vel_d_std_mps(0.05f);

  const int jitter_steps = 150;
  Eigen::Vector3d jitter_lla_hist[jitter_steps + 1];
  Eigen::Vector3f jitter_vel_hist[jitter_steps + 1];
  float jitter_time_hist[jitter_steps + 1];
  Eigen::Vector3d jitter_truth_lla = lla;
  Eigen::Vector3f jitter_truth_vel = Eigen::Vector3f::Zero();
  jitter_lla_hist[0] = jitter_truth_lla;
  jitter_vel_hist[0] = jitter_truth_vel;
  jitter_time_hist[0] = 0.0f;

  for (int i = 1; i <= jitter_steps; ++i)
  {
    // 模拟飞控主循环偶发抖动，最后一帧故意变大，用于暴露“单帧dt换算延迟步数”的错误。
    float jitter_dt_s = 0.005f;
    if ((i % 7) == 0)
    {
      jitter_dt_s = 0.0035f;
    }
    else if ((i % 11) == 0)
    {
      jitter_dt_s = 0.0075f;
    }
    if (i == jitter_steps)
    {
      jitter_dt_s = 0.010f;
    }

    Eigen::Vector3f jitter_accel_ned;
    jitter_accel_ned << 4.0f, 0.0f, 0.0f;
    const Eigen::Vector3f jitter_imu_accel =
        jitter_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();

    jitter_delay_ekf.TimeUpdate(jitter_imu_accel, gyro, jitter_dt_s);
    jitter_truth_vel += jitter_accel_ned * jitter_dt_s;
    jitter_truth_lla = integrateTruthLla(jitter_truth_lla, jitter_truth_vel, jitter_dt_s);
    jitter_lla_hist[i] = jitter_truth_lla;
    jitter_vel_hist[i] = jitter_truth_vel;
    jitter_time_hist[i] = jitter_time_hist[i - 1] + jitter_dt_s;
  }

  const float jitter_measurement_time = jitter_time_hist[jitter_steps] - kEkfGnssDelayS;
  int jitter_meas_idx = 0;
  float jitter_best_time_error = std::fabs(jitter_time_hist[0] - jitter_measurement_time);
  for (int i = 1; i <= jitter_steps; ++i)
  {
    const float time_error = std::fabs(jitter_time_hist[i] - jitter_measurement_time);
    if (time_error < jitter_best_time_error)
    {
      jitter_best_time_error = time_error;
      jitter_meas_idx = i;
    }
  }

  const Eigen::Vector3f jitter_vel_before_update = jitter_delay_ekf.ned_vel_mps();
  assert(std::fabs(jitter_vel_before_update(0) - jitter_truth_vel(0)) < 0.02f);

  // GNSS量测来自当前 EKF 固定延迟附近；滤波器应按历史dt累计找回对应快照，而不是用最后一帧dt估步数。
  jitter_delay_ekf.MeasurementUpdate(jitter_vel_hist[jitter_meas_idx],
                                     jitter_lla_hist[jitter_meas_idx],
                                     0.010f);
  assert(std::fabs(jitter_delay_ekf.ned_vel_mps()(0) - jitter_truth_vel(0)) < 0.08f);

  const Eigen::Vector3d before_stale_lla = jitter_delay_ekf.lla_rad_m();
  const Eigen::Vector3f before_stale_vel = jitter_delay_ekf.ned_vel_mps();
  const bfs::MeasurementUpdateResult stale_result =
      jitter_delay_ekf.MeasurementUpdateDetailed(
          jitter_vel_hist[0], jitter_lla_hist[0], 0.010f, 0.50f);
  assert(stale_result.attempted);
  assert(!stale_result.fused);
  assert((jitter_delay_ekf.ned_vel_mps() - before_stale_vel).norm() < 1.0e-7f);
  assert(bfs::lla2ned(jitter_delay_ekf.lla_rad_m(), before_stale_lla,
                      bfs::AngPosUnit::RAD).norm() < 1.0e-6);
  assertFiniteEkfState(jitter_delay_ekf);
  assertHealthyEkfCovariance(jitter_delay_ekf);
  recordScenarioPass(report, "场景21：GNSS延迟索引适应调度抖动且超龄量测必须拒绝");

  // ========== 场景22：极小量测噪声下矩阵更新仍需保持数值稳定 ==========
  bfs::Ekf15State tiny_noise_ekf;
  tiny_noise_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  tiny_noise_ekf.gnss_pos_ne_std_m(1.0e-6f);
  tiny_noise_ekf.gnss_pos_d_std_m(1.0e-6f);
  tiny_noise_ekf.gnss_vel_ne_std_mps(1.0e-6f);
  tiny_noise_ekf.gnss_vel_d_std_mps(1.0e-6f);

  for (int i = 0; i < 240; ++i)
  {
    // 使用有限但非常小的量测噪声反复融合，验证S矩阵反演路径不会把状态或协方差打坏。
    tiny_noise_ekf.TimeUpdate(accel, gyro, 0.005f);
    tiny_noise_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 1.0e-6f, 1.0e-6f);
    tiny_noise_ekf.MeasurementUpdateGravity(accel, 1.0e-6f);
    tiny_noise_ekf.MeasurementUpdateAttitude(level_ypr, 1.0e-6f, 1.0e-6f);
    tiny_noise_ekf.MeasurementUpdateYaw(0.0f, 1.0e-6f);
    if (i > 24 && (i % 20) == 0)
    {
      tiny_noise_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), lla, 0.005f);
    }
    assertFiniteEkfState(tiny_noise_ekf);
    assertHealthyEkfCovariance(tiny_noise_ekf);
  }
  assert(tiny_noise_ekf.ned_vel_mps().norm() < 0.05f);
  assert(std::fabs(tiny_noise_ekf.roll_rad()) < 0.02f);
  assert(std::fabs(tiny_noise_ekf.pitch_rad()) < 0.02f);
  recordScenarioPass(report, "场景22：极小量测噪声下矩阵更新仍需保持数值稳定");

  // ========== 场景23：有限但非法LLA输入不能污染地理状态 ==========
  static bfs::Ekf15State invalid_lla_init_ekf;
  Eigen::Vector3d invalid_init_lla;
  invalid_init_lla << 2.5, 8.0, -200000.0; // 有限但纬度、经度和高度都超出飞控可用范围。
  invalid_lla_init_ekf.Initialize(accel, gyro, mag, ned_vel, invalid_init_lla);
  assertFiniteEkfState(invalid_lla_init_ekf);
  assertPlausibleEkfLla(invalid_lla_init_ekf);

  static bfs::Ekf15State invalid_lla_update_ekf;
  invalid_lla_update_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  for (int i = 0; i < 80; ++i)
  {
    invalid_lla_update_ekf.TimeUpdate(accel, gyro, 0.005f);
  }
  const Eigen::Vector3d lla_before_invalid_update = invalid_lla_update_ekf.lla_rad_m();
  Eigen::Vector3d invalid_gnss_lla;
  invalid_gnss_lla << -2.2, -7.0, 150000.0; // GNSS字段有限但越界，应被量测入口拒绝。
  invalid_lla_update_ekf.MeasurementUpdate(Eigen::Vector3f::Zero(), invalid_gnss_lla, 0.005f);
  assertFiniteEkfState(invalid_lla_update_ekf);
  assertPlausibleEkfLla(invalid_lla_update_ekf);
  assert(bfs::lla2ned(invalid_lla_update_ekf.lla_rad_m(),
                      lla_before_invalid_update,
                      bfs::AngPosUnit::RAD)
             .norm() < 0.05);
  recordScenarioPass(report, "场景23：有限但非法LLA输入不能污染地理状态");

  // ========== 场景24：有限但离谱IMU单帧不能污染惯导状态 ==========
  static bfs::Ekf15State finite_imu_outlier_ekf;
  finite_imu_outlier_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  finite_imu_outlier_ekf.TimeUpdate(accel, gyro, 0.005f);
  const Eigen::Vector3f vel_before_finite_imu_outlier = finite_imu_outlier_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_finite_imu_outlier = finite_imu_outlier_ekf.lla_rad_m();
  const Eigen::Quaternionf quat_before_finite_imu_outlier = finite_imu_outlier_ekf.quat();

  Eigen::Vector3f huge_finite_accel;
  huge_finite_accel << 1000.0f, -900.0f, 800.0f;
  Eigen::Vector3f huge_finite_gyro;
  huge_finite_gyro << 100.0f, -80.0f, 70.0f;

  // 单帧IMU值有限但远超ICM42688量程和飞行器物理可能，应被TimeUpdate丢弃。
  finite_imu_outlier_ekf.TimeUpdate(huge_finite_accel, huge_finite_gyro, 0.005f);
  assert((finite_imu_outlier_ekf.ned_vel_mps() - vel_before_finite_imu_outlier).norm() < 1.0e-5f);
  assert(bfs::lla2ned(finite_imu_outlier_ekf.lla_rad_m(),
                      lla_before_finite_imu_outlier,
                      bfs::AngPosUnit::RAD)
             .norm() < 1.0e-4);
  assert(std::fabs(finite_imu_outlier_ekf.quat().dot(quat_before_finite_imu_outlier)) > 0.99999f);
  assertFiniteEkfState(finite_imu_outlier_ekf);
  assertHealthyEkfCovariance(finite_imu_outlier_ekf);

  // 坏IMU帧之后恢复正常输入，证明防御只丢弃坏帧，不会锁死惯导预测。
  finite_imu_outlier_ekf.TimeUpdate(accel, gyro, 0.005f);
  assertFiniteEkfState(finite_imu_outlier_ekf);
  assertHealthyEkfCovariance(finite_imu_outlier_ekf);
  recordScenarioPass(report, "场景24：有限但离谱IMU单帧不能污染惯导状态");

  // ========== 场景25：GNSS丢失时EKF切回后台AHRS输出不能产生姿态跳变 ==========
  bfs::Ekf15State switch_continuity_ekf;
  switch_continuity_ekf.Initialize(accel, gyro, mag, ned_vel, lla);
  Eigen::Vector3d switch_truth_lla = lla;
  Eigen::Vector3f switch_truth_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3d switch_lla_hist[501];
  Eigen::Vector3f switch_vel_hist[501];
  switch_lla_hist[0] = switch_truth_lla;
  switch_vel_hist[0] = switch_truth_vel;

  const float initial_raw_ahrs_yaw = 0.65f;
  float backup_ahrs_yaw = wrapAngleTwoPiForTest(initial_raw_ahrs_yaw);
  float backup_ahrs_roll = 0.0f;
  float backup_ahrs_pitch = 0.0f;
  float ahrs_yaw_correction_rad = 0.0f;
  float max_switch_output_jump_rad = 0.0f;
  bool was_using_ekf_output = false;

  for (int step = 1; step <= 500; ++step)
  {
    const float t_s = (step - 1) * 0.005f;
    const bool gnss_available = (t_s < 1.8f);
    Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
    if (t_s >= 0.4f && t_s < 1.2f)
    {
      truth_accel_ned << 0.35f, 0.04f, 0.0f;
    }
    else if (t_s >= 1.2f && t_s < 2.2f)
    {
      truth_accel_ned << -0.18f, -0.03f, 0.0f;
    }

    Eigen::Vector3f switch_imu_accel =
        truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished();
    switch_imu_accel(0) += deterministicNoise(step, 0.025f, 0.2f);
    switch_imu_accel(1) += deterministicNoise(step, 0.022f, 1.2f);
    switch_imu_accel(2) += deterministicNoise(step, 0.035f, 2.2f);
    Eigen::Vector3f switch_gyro;
    switch_gyro << deterministicNoise(step, 0.0005f, 0.3f),
        deterministicNoise(step, 0.0005f, 1.3f),
        deterministicNoise(step, 0.0006f, 2.3f);

    switch_continuity_ekf.TimeUpdate(switch_imu_accel, switch_gyro, 0.005f);

    // 后台AHRS始终运行：这里用小幅确定性漂移模拟磁航向/陀螺积分残差。
    const float raw_ahrs_yaw = wrapAngleTwoPiForTest(initial_raw_ahrs_yaw + 0.00008f * static_cast<float>(step) +
                                                     deterministicNoise(step, 0.0004f, 0.9f));
    backup_ahrs_yaw = wrapAngleTwoPiForTest(raw_ahrs_yaw + ahrs_yaw_correction_rad);
    backup_ahrs_roll = 0.006f * std::sin(0.7f * t_s);
    backup_ahrs_pitch = 0.005f * std::cos(0.6f * t_s);

    if (shouldFuseAhrsAttitude(switch_imu_accel, switch_gyro, switch_continuity_ekf.quat()))
    {
      Eigen::Vector3f switch_ahrs_ypr;
      switch_ahrs_ypr << backup_ahrs_yaw, backup_ahrs_pitch, backup_ahrs_roll;
      const float accel_norm_error = std::fabs(switch_imu_accel.norm() - kLocalGravity);
      const float roll_pitch_noise_rad = linearInterpolateForTest(accel_norm_error,
                                                                  0.3f, 2.0f,
                                                                  0.04f, 0.30f);
      switch_continuity_ekf.MeasurementUpdateAttitude(switch_ahrs_ypr,
                                                      roll_pitch_noise_rad,
                                                      gnss_available ? 3.0f : 0.35f);
    }

    switch_truth_vel += truth_accel_ned * 0.005f;
    switch_truth_lla = integrateTruthLla(switch_truth_lla, switch_truth_vel, 0.005f);
    switch_lla_hist[step] = switch_truth_lla;
    switch_vel_hist[step] = switch_truth_vel;

    if (gnss_available && step > gnss_delay_steps && ((step % 20) == 0))
    {
      const int meas_idx = step - gnss_delay_steps;
      switch_continuity_ekf.gnss_pos_ne_std_m(0.35f);
      switch_continuity_ekf.gnss_pos_d_std_m(0.50f);
      switch_continuity_ekf.gnss_vel_ne_std_mps(0.08f);
      switch_continuity_ekf.gnss_vel_d_std_mps(0.10f);
      switch_continuity_ekf.MeasurementUpdate(switch_vel_hist[meas_idx],
                                              switch_lla_hist[meas_idx],
                                              0.005f);
    }

    if (gnss_available)
    {
      // 复现主程序中的航向对齐：GNSS有效时用EKF航向慢速校正后台AHRS。
      float yaw_error = wrapAnglePiForTest(switch_continuity_ekf.yaw_rad() - backup_ahrs_yaw);
      yaw_error = clampForTest(yaw_error,
                               -kMaxAhrsYawCorrectionStepRad,
                               kMaxAhrsYawCorrectionStepRad);
      ahrs_yaw_correction_rad = wrapAnglePiForTest(ahrs_yaw_correction_rad + yaw_error);
    }

    const bool use_ekf_output = gnss_available;
    const float output_yaw = use_ekf_output ? wrapAngleTwoPiForTest(switch_continuity_ekf.yaw_rad())
                                           : backup_ahrs_yaw;
    if (was_using_ekf_output && !use_ekf_output)
    {
      // 关键断言：失去GNSS后输出源从EKF切到后台AHRS，航向差不能形成控制可见跳变。
      const float ekf_yaw_before_switch = wrapAngleTwoPiForTest(switch_continuity_ekf.yaw_rad());
      max_switch_output_jump_rad = std::max(max_switch_output_jump_rad,
                                            yawDistanceRad(output_yaw, ekf_yaw_before_switch));
    }
    was_using_ekf_output = use_ekf_output;
    assertFiniteEkfState(switch_continuity_ekf);
  }

  assert(max_switch_output_jump_rad < 0.08f);
  assert(yawDistanceRad(wrapAngleTwoPiForTest(switch_continuity_ekf.yaw_rad()), backup_ahrs_yaw) < 0.10f);
  assert(std::fabs(switch_continuity_ekf.roll_rad()) < 0.08f);
  assert(std::fabs(switch_continuity_ekf.pitch_rad()) < 0.08f);
  assertHealthyEkfCovariance(switch_continuity_ekf);
  recordScenarioPassWithSwitchMetric(report,
                                     "场景25：GNSS丢失时EKF切回后台AHRS输出不能产生姿态跳变",
                                     max_switch_output_jump_rad);

  // ========== 场景26：无GNSS长时静止/机动退化时姿态和速度不能发散 ==========
  bfs::Ekf15State no_gnss_long_ekf;
  no_gnss_long_ekf.gyro_std_radps(0.0015f);
  no_gnss_long_ekf.gyro_markov_bias_std_radps(0.00001f);
  no_gnss_long_ekf.gyro_tau_s(50.0f);
  no_gnss_long_ekf.accel_std_mps2(0.25f);
  no_gnss_long_ekf.accel_markov_bias_std_mps2(0.05f);
  no_gnss_long_ekf.accel_tau_s(100.0f);
  no_gnss_long_ekf.init_heading_err_std_rad(0.1f);
  no_gnss_long_ekf.Initialize(accel, gyro, mag, ned_vel, lla);

  const Eigen::Vector3f no_gnss_accel_bias =
      (Eigen::Vector3f() << 0.035f, -0.028f, 0.045f).finished();
  const Eigen::Vector3f no_gnss_gyro_bias =
      (Eigen::Vector3f() << 0.0009f, -0.0007f, 0.0011f).finished();
  float no_gnss_max_speed_mps = 0.0f;
  float no_gnss_max_tilt_rad = 0.0f;
  int no_gnss_ahrs_fused = 0;
  int no_gnss_ahrs_skipped = 0;
  int no_gnss_zupt_updates = 0;

  for (int step = 1; step <= 4800; ++step)
  {
    const float t_s = (step - 1) * 0.005f;
    const bool static_phase = (t_s < 2.0f) ||
                              (t_s >= 7.0f && t_s < 9.0f) ||
                              (t_s >= 14.0f);
    const bool strong_maneuver_phase = (t_s >= 3.0f && t_s < 5.6f) ||
                                       (t_s >= 10.0f && t_s < 12.8f);

    Eigen::Vector3f truth_accel_ned = Eigen::Vector3f::Zero();
    if (t_s >= 2.0f && t_s < 3.0f)
    {
      truth_accel_ned << 0.45f, 0.05f, 0.0f;
    }
    else if (strong_maneuver_phase)
    {
      truth_accel_ned << 2.2f * std::sin(1.6f * t_s),
          1.3f * std::cos(1.2f * t_s),
          -0.8f + 0.5f * std::sin(1.9f * t_s);
    }
    else if (t_s >= 5.6f && t_s < 7.0f)
    {
      truth_accel_ned << -0.35f, -0.04f, 0.0f;
    }
    else if (t_s >= 9.0f && t_s < 10.0f)
    {
      truth_accel_ned << 0.30f, -0.12f, 0.0f;
    }
    else if (t_s >= 12.8f && t_s < 14.0f)
    {
      truth_accel_ned << -0.28f, 0.10f, 0.0f;
    }

    Eigen::Vector3f no_gnss_imu_accel =
        truth_accel_ned - (Eigen::Vector3f() << 0.0f, 0.0f, kLocalGravity).finished() +
        no_gnss_accel_bias;
    no_gnss_imu_accel(0) += deterministicNoise(step, 0.055f, 0.6f);
    no_gnss_imu_accel(1) += deterministicNoise(step, 0.050f, 1.6f);
    no_gnss_imu_accel(2) += deterministicNoise(step, 0.085f, 2.6f);

    Eigen::Vector3f no_gnss_imu_gyro = no_gnss_gyro_bias;
    no_gnss_imu_gyro(0) += deterministicNoise(step, 0.0010f, 0.4f);
    no_gnss_imu_gyro(1) += deterministicNoise(step, 0.0010f, 1.4f);
    no_gnss_imu_gyro(2) += deterministicNoise(step, 0.0012f, 2.4f);

    if (static_phase)
    {
      // 复现主循环静止时锁定Yaw轴陀螺输入，验证ZUPT辅助下航向不会因Z轴零偏继续积分。
      no_gnss_imu_gyro(2) = no_gnss_long_ekf.gyro_bias_radps()(2);
    }

    no_gnss_long_ekf.TimeUpdate(no_gnss_imu_accel, no_gnss_imu_gyro, 0.005f);

    if (shouldFuseAhrsAttitude(no_gnss_imu_accel, no_gnss_imu_gyro, no_gnss_long_ekf.quat()))
    {
      const float accel_norm_error = std::fabs(no_gnss_imu_accel.norm() - kLocalGravity);
      const float roll_pitch_noise_rad = linearInterpolateForTest(accel_norm_error,
                                                                  0.3f, 2.0f,
                                                                  0.05f, 0.35f);
      no_gnss_long_ekf.MeasurementUpdateAttitude(level_ypr, roll_pitch_noise_rad, 0.50f);
      ++no_gnss_ahrs_fused;
    }
    else
    {
      ++no_gnss_ahrs_skipped;
    }

    if (static_phase && (step % 20) == 0)
    {
      no_gnss_long_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.06f, 0.08f);
      ++no_gnss_zupt_updates;
    }

    const float no_gnss_speed_mps = no_gnss_long_ekf.ned_vel_mps().norm();
    const float no_gnss_tilt_rad = std::sqrt(no_gnss_long_ekf.roll_rad() * no_gnss_long_ekf.roll_rad() +
                                             no_gnss_long_ekf.pitch_rad() * no_gnss_long_ekf.pitch_rad());
    no_gnss_max_speed_mps = std::max(no_gnss_max_speed_mps, no_gnss_speed_mps);
    no_gnss_max_tilt_rad = std::max(no_gnss_max_tilt_rad, no_gnss_tilt_rad);
    assertFiniteEkfState(no_gnss_long_ekf);
    assertHealthyEkfCovariance(no_gnss_long_ekf);
  }

  assert(no_gnss_ahrs_fused > 1000);
  assert(no_gnss_ahrs_skipped > 400);
  assert(no_gnss_zupt_updates >= 120);
  assert(std::fabs(no_gnss_long_ekf.roll_rad()) < 0.08f);
  assert(std::fabs(no_gnss_long_ekf.pitch_rad()) < 0.08f);
  assert(no_gnss_long_ekf.ned_vel_mps().norm() < 0.18f);
  assert(no_gnss_max_tilt_rad < 0.25f);
  assert(no_gnss_max_speed_mps < 4.0f);
  recordScenarioPassWithMetrics(report,
                                "场景26：无GNSS长时静止/机动退化时姿态和速度不能发散",
                                0.0f,
                                no_gnss_max_speed_mps,
                                no_gnss_max_tilt_rad);

  // ========== 场景27：长时间GNSS丢失后低质量恢复不能拉飞状态 ==========
  const GnssReacquisitionMetrics degraded_metrics =
      runLongGnssLossPoorReacquisitionScenario(accel, gyro, mag, ned_vel, lla, level_ypr);
  assert(degraded_metrics.gnss_updates >= 25);
  assert(degraded_metrics.zupt_updates >= 20);
  assert(degraded_metrics.max_reacq_jump_m < 3.0f);
  assert(degraded_metrics.final_pos_err_m < 1.2f);
  assert(degraded_metrics.final_vel_err_mps < 0.25f);
  assert(degraded_metrics.max_tilt_err_rad < 0.10f);
  recordScenarioPassWithReacqMetrics(report,
                                     "场景27：长时间GNSS丢失后低质量恢复不能拉飞状态",
                                     degraded_metrics);

  // ========== 场景28：长时间GNSS丢失后假重捕获不能拉飞状态 ==========
  const GnssReacquisitionMetrics false_reacq_metrics =
      runFalseGnssReacquisitionRejectionScenario(accel, gyro, mag, ned_vel, lla, level_ypr);
  assert(false_reacq_metrics.gnss_updates >= 5);
  assert(false_reacq_metrics.zupt_updates >= 80);
  assert(false_reacq_metrics.max_reacq_jump_m < 2.0f);
  assert(false_reacq_metrics.max_pos_err_m < 2.5f);
  assert(false_reacq_metrics.final_pos_err_m < 1.0f);
  assert(false_reacq_metrics.final_vel_err_mps < 0.18f);
  assert(false_reacq_metrics.max_tilt_err_rad < 0.08f);
  recordScenarioPassWithReacqMetrics(report,
                                     "场景28：长时间GNSS丢失后假重捕获不能拉飞状态",
                                     false_reacq_metrics);

  // ========== 场景29：运动中误触发ZUPT不能把速度硬拉到零 ==========
  bfs::Ekf15State false_zupt_ekf;
  Eigen::Vector3f fast_ned_vel;
  fast_ned_vel << 6.2f, -0.4f, 0.0f;
  false_zupt_ekf.Initialize(accel, gyro, mag, fast_ned_vel, lla);
  const Eigen::Vector3f vel_before_false_zupt = false_zupt_ekf.ned_vel_mps();
  const Eigen::Vector3d lla_before_false_zupt = false_zupt_ekf.lla_rad_m();
  for (int i = 0; i < 25; ++i)
  {
    // 误静止确认会重复触发零速观测；底层应按速度残差拒绝接近8m/s门限的大运动。
    false_zupt_ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
    assertFiniteEkfState(false_zupt_ekf);
    assertHealthyEkfCovariance(false_zupt_ekf);
  }
  const float false_zupt_vel_change_mps =
      (false_zupt_ekf.ned_vel_mps() - vel_before_false_zupt).norm();
  const float false_zupt_pos_jump_m = static_cast<float>(
      bfs::lla2ned(false_zupt_ekf.lla_rad_m(),
                   lla_before_false_zupt,
                   bfs::AngPosUnit::RAD)
          .norm());
  assert(false_zupt_vel_change_mps < 0.30f);
  assert(false_zupt_pos_jump_m < 0.30f);
  recordScenarioPassWithMetrics(report,
                                "场景29：运动中误触发ZUPT不能把速度硬拉到零",
                                false_zupt_pos_jump_m,
                                false_zupt_vel_change_mps,
                                0.0f);

  // ========== 场景30：无GNSS假原点启动后首次GNSS必须能重锚定 ==========
  bfs::Ekf15State fake_origin_ekf;
  Eigen::Vector3d fake_origin_lla;
  fake_origin_lla << 28.2 * kPi / 180.0, 112.9 * kPi / 180.0, 50.0;
  Eigen::Vector3d real_gnss_lla;
  real_gnss_lla << 31.2 * kPi / 180.0, 121.5 * kPi / 180.0, 18.0;
  Eigen::Vector3f real_gnss_vel;
  real_gnss_vel << 0.12f, -0.08f, 0.0f;
  fake_origin_ekf.Initialize(accel, gyro, mag, ned_vel, fake_origin_lla);
  for (int i = 0; i < 120; ++i)
  {
    fake_origin_ekf.TimeUpdate(accel, gyro, 0.005f);
    fake_origin_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.50f);
  }
  const Eigen::Quaternionf quat_before_reanchor = fake_origin_ekf.quat();
  fake_origin_ekf.ResetPositionVelocityToGnss(real_gnss_vel, real_gnss_lla);
  const float fake_origin_reanchor_pos_err_m = static_cast<float>(
      bfs::lla2ned(fake_origin_ekf.lla_rad_m(), real_gnss_lla, bfs::AngPosUnit::RAD).norm());
  const float fake_origin_reanchor_vel_err_mps =
      (fake_origin_ekf.ned_vel_mps() - real_gnss_vel).norm();
  const float fake_origin_reanchor_att_jump_rad =
      2.0f * std::acos(std::min(1.0f, std::fabs(fake_origin_ekf.quat().dot(quat_before_reanchor))));
  assert(fake_origin_reanchor_pos_err_m < 0.05f);
  assert(fake_origin_reanchor_vel_err_mps < 0.01f);
  assert(fake_origin_reanchor_att_jump_rad < 0.001f);
  assertFiniteEkfState(fake_origin_ekf);
  assertHealthyEkfCovariance(fake_origin_ekf);
  recordScenarioPassWithMetrics(report,
                                "场景30：无GNSS假原点启动后首次GNSS必须能重锚定",
                                fake_origin_reanchor_pos_err_m,
                                fake_origin_reanchor_vel_err_mps,
                                fake_origin_reanchor_att_jump_rad);

  // ========== 场景31：重锚定后的延迟GNSS不能回放旧假原点历史 ==========
  bfs::Ekf15State reanchor_history_ekf;
  reanchor_history_ekf.Initialize(accel, gyro, mag, ned_vel, fake_origin_lla);
  for (int i = 0; i < 180; ++i)
  {
    // 先填满旧假原点历史缓冲，复现无GNSS启动后等待一段时间才获得真实GNSS。
    reanchor_history_ekf.TimeUpdate(accel, gyro, 0.005f);
    reanchor_history_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.50f);
  }
  reanchor_history_ekf.ResetPositionVelocityToGnss(real_gnss_vel, real_gnss_lla);
  float reanchor_history_max_pos_err_m = 0.0f;
  float reanchor_history_max_vel_err_mps = 0.0f;
  for (int step = 1; step <= 360; ++step)
  {
    reanchor_history_ekf.TimeUpdate(accel, gyro, 0.005f);
    reanchor_history_ekf.MeasurementUpdateAttitude(level_ypr, 0.05f, 0.50f);
    if ((step % 20) == 0)
    {
      Eigen::Vector3d meas_lla = real_gnss_lla;
      Eigen::Vector3f meas_vel = real_gnss_vel;
      meas_lla += bfs::ned2lla((Eigen::Vector3d() << deterministicNoise(step, 0.08f, 0.4f),
                                                  deterministicNoise(step, 0.07f, 1.4f),
                                                  deterministicNoise(step, 0.10f, 2.4f))
                                    .finished(),
                                real_gnss_lla,
                                bfs::AngPosUnit::RAD) -
                  real_gnss_lla;
      meas_vel(0) += deterministicNoise(step, 0.02f, 0.9f);
      meas_vel(1) += deterministicNoise(step, 0.02f, 1.9f);
      meas_vel(2) += deterministicNoise(step, 0.03f, 2.9f);
      reanchor_history_ekf.gnss_pos_ne_std_m(0.35f);
      reanchor_history_ekf.gnss_pos_d_std_m(0.50f);
      reanchor_history_ekf.gnss_vel_ne_std_mps(0.08f);
      reanchor_history_ekf.gnss_vel_d_std_mps(0.10f);
      reanchor_history_ekf.MeasurementUpdate(meas_vel, meas_lla, 0.005f);
    }

    const float pos_err_m = static_cast<float>(
        bfs::lla2ned(reanchor_history_ekf.lla_rad_m(), real_gnss_lla, bfs::AngPosUnit::RAD)
            .head<2>()
            .norm());
    const float vel_err_mps = (reanchor_history_ekf.ned_vel_mps() - real_gnss_vel).norm();
    reanchor_history_max_pos_err_m = std::max(reanchor_history_max_pos_err_m, pos_err_m);
    reanchor_history_max_vel_err_mps = std::max(reanchor_history_max_vel_err_mps, vel_err_mps);
    assertFiniteEkfState(reanchor_history_ekf);
    assertHealthyEkfCovariance(reanchor_history_ekf);
  }
  assert(reanchor_history_max_pos_err_m < 1.0f);
  assert(reanchor_history_max_vel_err_mps < 0.35f);
  recordScenarioPassWithMetrics(report,
                                "场景31：重锚定后的延迟GNSS不能回放旧假原点历史",
                                reanchor_history_max_pos_err_m,
                                reanchor_history_max_vel_err_mps,
                                0.0f);

  // ========== 场景32：静止确认后AHRS必须能从较大EKF倾角误差中恢复 ==========
  const AhrsStaticRecoveryMetrics static_recovery_metrics =
      runStaticAhrsRecoveryAfterEkfTiltErrorScenario(accel, gyro, mag, ned_vel, lla, level_ypr);
  assert(static_recovery_metrics.tilt_before_recovery_rad > 0.25f);
  assert(static_recovery_metrics.first_static_gate_allows_fusion);
  assert(static_recovery_metrics.fused_count > 120);
  assert(static_recovery_metrics.tilt_after_recovery_rad < 0.08f);
  assert(static_recovery_metrics.tilt_after_recovery_rad <
         0.35f * static_recovery_metrics.tilt_before_recovery_rad);
  recordScenarioPassWithAhrsStaticRecoveryMetrics(
      report,
      "场景32：静止确认后AHRS必须能从较大EKF倾角误差中恢复",
      static_recovery_metrics);

  // ========== 场景33：静止假阳性不能绕过机动加速度保护 ==========
  const AhrsFalseStaticManeuverMetrics false_static_maneuver_metrics =
      runFalseStaticManeuverAhrsGateScenario(accel, gyro, mag, ned_vel, lla);
  assert(false_static_maneuver_metrics.body_lateral_accel_mps2 > 1.5f);
  assert(!false_static_maneuver_metrics.false_static_gate_allows_fusion);
  assert(false_static_maneuver_metrics.tilt_change_after_bad_ahrs_rad < 0.01f);
  recordScenarioPassWithAhrsFalseStaticManeuverMetrics(
      report,
      "场景33：静止假阳性不能绕过机动加速度保护",
      false_static_maneuver_metrics);

  // ========== 场景34：GNSS旧fix不能继续驱动导航输出 ==========
  const GnssFreshnessMetrics gnss_freshness_metrics =
      runStaleGnssFixShouldNotDriveNavigationScenario();
  assert(gnss_freshness_metrics.fresh_fix_valid);
  assert(!gnss_freshness_metrics.stale_fix_valid);
  assert(!gnss_freshness_metrics.output_uses_ekf_on_stale_fix);
  assert(!gnss_freshness_metrics.yaw_correction_runs_on_stale_fix);
  recordScenarioPassWithGnssFreshnessMetrics(
      report,
      "场景34：GNSS旧fix不能继续驱动导航输出",
      gnss_freshness_metrics);

  // ========== 场景35：GNSS fix边界抖动不能造成输出来回切换 ==========
  const GnssOutputHysteresisMetrics gnss_hysteresis_metrics =
      runGnssFixFlickerShouldNotToggleOutputScenario();
  assert(gnss_hysteresis_metrics.output_valid_after_single_bad_frame);
  assert(gnss_hysteresis_metrics.output_invalid_after_long_dropout);
  assert(gnss_hysteresis_metrics.switch_count <= 2);
  assert(gnss_hysteresis_metrics.invalid_gap_max_ms > kGnssNavDropoutHoldMs);
  recordScenarioPassWithGnssOutputHysteresisMetrics(
      report,
      "场景35：GNSS fix边界抖动不能造成输出来回切换",
      gnss_hysteresis_metrics);

  // ========== 场景36：AHRS航向噪声必须跟随GNSS新鲜度 ==========
  const AhrsYawNoiseFreshnessMetrics ahrs_yaw_noise_freshness_metrics =
      runAhrsYawNoiseMustUseFreshGnssScenario();
  assert(std::fabs(ahrs_yaw_noise_freshness_metrics.fresh_yaw_noise_rad -
                   kAhrsYawNoiseWithGnssRad) < 1.0e-6f);
  assert(ahrs_yaw_noise_freshness_metrics.stale_fix_uses_no_gnss_noise);
  recordScenarioPassWithAhrsYawNoiseFreshnessMetrics(
      report,
      "场景36：AHRS航向噪声必须跟随GNSS新鲜度",
      ahrs_yaw_noise_freshness_metrics);

  // ========== 场景37：备用AHRS航向偏置校正必须跟随GNSS新鲜度 ==========
  const AhrsYawCorrectionHoldMetrics ahrs_yaw_correction_hold_metrics =
      runAhrsYawCorrectionMustUseFreshGnssScenario();
  assert(ahrs_yaw_correction_hold_metrics.correction_runs_on_fresh_gnss);
  assert(!ahrs_yaw_correction_hold_metrics.correction_runs_during_output_hold);
  assert(ahrs_yaw_correction_hold_metrics.correction_delta_during_output_hold_rad < 1.0e-6f);
  recordScenarioPassWithAhrsYawCorrectionHoldMetrics(
      report,
      "场景37：备用AHRS航向偏置校正必须跟随GNSS新鲜度",
      ahrs_yaw_correction_hold_metrics);

  // ========== 场景38：假原点重锚定必须同步UBX相对位置原点 ==========
  const OriginReanchorMetrics origin_reanchor_metrics =
      runOriginReanchorMustSyncDegreeOriginScenario();
  assert(origin_reanchor_metrics.relative_jump_before_reanchor_m > 500000.0f);
  assert(origin_reanchor_metrics.degree_origin_synced);
  assert(origin_reanchor_metrics.relative_after_reanchor_m < 0.05f);
  recordScenarioPassWithOriginReanchorMetrics(
      report,
      "场景38：假原点重锚定必须同步UBX相对位置原点",
      origin_reanchor_metrics);

  // ========== 场景39：首次GNSS重锚定前不能发布旧原点UBX相对位置 ==========
  const FirstGnssReanchorRelativeTimingMetrics first_gnss_relative_timing_metrics =
      runFirstGnssRelativeUpdateMustWaitForReanchorScenario();
  assert(!first_gnss_relative_timing_metrics.stale_relative_was_published);
  assert(first_gnss_relative_timing_metrics.relative_after_reanchor_m < 0.05f);
  recordScenarioPassWithFirstGnssRelativeTimingMetrics(
      report,
      "场景39：首次GNSS重锚定前不能发布旧原点UBX相对位置",
      first_gnss_relative_timing_metrics);

  // ========== 场景40：GNSS旧速度不能阻塞无GNSS静止确认 ==========
  const StaticDetectionFreshnessMetrics static_detection_freshness_metrics =
      runStaleGnssSpeedMustNotBlockStaticDetectionScenario();
  assert(!static_detection_freshness_metrics.stale_gnss_blocks_static);
  assert(static_detection_freshness_metrics.static_confirmed_after_timeout);
  assert(static_detection_freshness_metrics.confirmed_after_ms >= 550);
  recordScenarioPassWithStaticDetectionFreshnessMetrics(
      report,
      "场景40：GNSS旧速度不能阻塞无GNSS静止确认",
      static_detection_freshness_metrics);

  // ========== 场景41：GNSS旧fix不能阻塞水平估计回退 ==========
  const StatusFixFreshnessMetrics status_fix_freshness_metrics =
      runStatusFixMustUseFreshGnssScenario();
  assert(status_fix_freshness_metrics.fresh_mapped_fix >= 3);
  assert(status_fix_freshness_metrics.stale_mapped_fix == 1);
  assert(!status_fix_freshness_metrics.stale_fix_forces_horizontal_gnss_path);
  recordScenarioPassWithStatusFixFreshnessMetrics(
      report,
      "场景41：GNSS旧fix不能阻塞水平估计回退",
      status_fix_freshness_metrics);

  // ========== 场景42：AnoCom地面站GNSS遥测必须跟随GNSS新鲜度 ==========
  const AnoGnssTelemetryFreshnessMetrics ano_gnss_telemetry_freshness_metrics =
      runAnoComGnssTelemetryMustUseFreshnessScenario();
  assert(ano_gnss_telemetry_freshness_metrics.fresh_fix_sta >= 3);
  assert(ano_gnss_telemetry_freshness_metrics.stale_fix_sta == 1);
  assert(ano_gnss_telemetry_freshness_metrics.stale_num_sat == 0);
  assert(!ano_gnss_telemetry_freshness_metrics.stale_velocity_branch_uses_raw_gnss);
  assert(!ano_gnss_telemetry_freshness_metrics.stale_telemetry_reports_raw_gnss);
  assert(!ano_gnss_telemetry_freshness_metrics.stale_accuracy_reports_raw_gnss);
  recordScenarioPassWithAnoGnssTelemetryFreshnessMetrics(
      report,
      "场景42：AnoCom地面站GNSS遥测必须跟随GNSS新鲜度",
      ano_gnss_telemetry_freshness_metrics);

  // ========== 场景43：GNSS 2D fix不能进入组合导航3D路径 ==========
  const Gnss3dBoundaryMetrics gnss_3d_boundary_metrics =
      runGnss3dFixBoundaryMustBeStrictScenario();
  assert(!gnss_3d_boundary_metrics.fix_2d_valid_for_nav);
  assert(gnss_3d_boundary_metrics.fix_3d_valid_for_nav);
  assert(!gnss_3d_boundary_metrics.dual_vector_yaw_allows_2d);
  assert(!gnss_3d_boundary_metrics.relative_position_allows_2d);
  recordScenarioPassWithGnss3dBoundaryMetrics(
      report,
      "场景43：GNSS 2D fix不能进入组合导航3D路径",
      gnss_3d_boundary_metrics);

  // ========== 场景44：高角速度纯惯导姿态传播必须接近精确指数映射 ==========
  const HighRateQuatPropagationMetrics high_rate_quat_metrics =
      runHighRateGyroPropagationMustUseExactDeltaScenario(accel, gyro, mag, ned_vel, lla);
  assert(high_rate_quat_metrics.max_quat_error_rad < 0.010f);
  assert(high_rate_quat_metrics.final_quat_error_rad < 0.010f);
  recordScenarioPassWithHighRateQuatPropagationMetrics(
      report,
      "场景44：高角速度纯惯导姿态传播必须接近精确指数映射",
      high_rate_quat_metrics);

  // ========== 场景45：姿态误差状态注入必须接近精确旋转反馈 ==========
  const AttitudeInjectionMetrics attitude_injection_metrics =
      runAttitudeErrorInjectionMustUseExactDeltaScenario(accel, gyro, mag, ned_vel, lla);
  assert(attitude_injection_metrics.tilt_before_update_rad > 0.45f);
  assert(attitude_injection_metrics.tilt_after_update_rad < 0.005f);
  assert(attitude_injection_metrics.single_update_correction_rad > 0.44f);
  recordScenarioPassWithAttitudeInjectionMetrics(
      report,
      "场景45：姿态误差状态注入必须接近精确旋转反馈",
      attitude_injection_metrics);

  // ========== 场景46：过程噪声离散化必须保持协方差正半定 ==========
  const CovarianceProcessNoiseMetrics covariance_process_noise_metrics =
      runProcessNoiseCovarianceMustRemainPsdScenario(accel, gyro, mag, ned_vel, lla);
  assert(covariance_process_noise_metrics.propagated_steps == 200);
  assert(covariance_process_noise_metrics.max_gyro_norm_radps > 15.0f);
  assert(covariance_process_noise_metrics.max_accel_norm_mps2 > 20.0f);
  recordScenarioPassWithCovarianceProcessNoiseMetrics(
      report,
      "场景46：过程噪声离散化必须保持协方差正半定",
      covariance_process_noise_metrics);

  // ========== 场景47：旋转比力下捷联传播必须接近高频子步真值 ==========
  const StrapdownIntegrationAccuracyMetrics strapdown_accuracy_metrics =
      runRotatingSpecificForceIntegrationAccuracyScenario(accel, gyro, mag, ned_vel, lla);
  assert(strapdown_accuracy_metrics.max_velocity_error_mps < 0.020f);
  assert(strapdown_accuracy_metrics.final_position_error_m < 0.060f);
  recordScenarioPassWithStrapdownIntegrationAccuracyMetrics(
      report,
      "场景47：旋转比力下捷联传播必须接近高频子步真值",
      strapdown_accuracy_metrics);

  // ========== 场景48：协方差低频离散化必须接近高频子步参考 ==========
  const CovarianceDiscretizationAccuracyMetrics covariance_discretization_metrics =
      runCovarianceDiscretizationShouldMatchSubstepsScenario(accel, gyro, mag, ned_vel, lla);
  assert(covariance_discretization_metrics.relative_trace_error < 0.0020f);
  assert(covariance_discretization_metrics.relative_max_coeff_error < 0.0020f);
  recordScenarioPassWithCovarianceDiscretizationAccuracyMetrics(
      report,
      "场景48：协方差低频离散化必须接近高频子步参考",
      covariance_discretization_metrics);

  // ========== 场景49：重力梯度项必须进入D向速度误差通道 ==========
  const GravityGradientModelMetrics gravity_gradient_metrics =
      runGravityGradientMustBeInDownVelocityChannelScenario(accel, gyro, mag, ned_vel, lla);
  assert(gravity_gradient_metrics.phi_vel_down_from_pos_down > 0.0f);
  assert(std::fabs(gravity_gradient_metrics.phi_vel_down_from_pos_down -
                   gravity_gradient_metrics.expected_phi_vel_down_from_pos_down) < 2.0e-9f);
  assert(std::fabs(gravity_gradient_metrics.phi_pos_down_from_pos_down - 1.0f) < 1.0e-6f);
  recordScenarioPassWithGravityGradientModelMetrics(
      report,
      "场景49：重力梯度项必须进入D向速度误差通道",
      gravity_gradient_metrics);

  // ========== 场景50：WGS84正常重力模型必须避免静止垂向漂移 ==========
  const Wgs84GravityMechanizationMetrics wgs84_gravity_metrics =
      runWgs84NormalGravityMustPreventStaticVerticalDriftScenario(gyro, mag);
  assert(wgs84_gravity_metrics.fixed_gravity_mismatch_mps2 > 0.015f);
  assert(std::fabs(wgs84_gravity_metrics.final_down_velocity_mps) < 0.030f);
  assert(wgs84_gravity_metrics.max_speed_mps < 0.050f);
  recordScenarioPassWithWgs84GravityMechanizationMetrics(
      report,
      "场景50：WGS84正常重力模型必须避免静止垂向漂移",
      wgs84_gravity_metrics);

  // ========== 场景51：流式圆锥补偿必须降低交替旋转姿态误差 ==========
  const ConingCompensationMetrics coning_compensation_metrics =
      runStreamingConingCompensationScenario(accel, gyro, mag, ned_vel, lla);
  assert(coning_compensation_metrics.max_coning_error_rad < 0.0020f);
  assert(coning_compensation_metrics.final_coning_error_rad < 0.0020f);
  recordScenarioPassWithConingCompensationMetrics(
      report,
      "场景51：流式圆锥补偿必须降低交替旋转姿态误差",
      coning_compensation_metrics);

  // ========== 场景52：流式划摇补偿必须降低交替旋转比力速度误差 ==========
  const ScullingCompensationMetrics sculling_compensation_metrics =
      runStreamingScullingCompensationScenario(accel, gyro, mag, ned_vel, lla);
  assert(sculling_compensation_metrics.max_velocity_error_mps < 0.020f);
  assert(sculling_compensation_metrics.final_position_error_m < 0.060f);
  recordScenarioPassWithScullingCompensationMetrics(
      report,
      "场景52：流式划摇补偿必须降低交替旋转比力速度误差",
      sculling_compensation_metrics);

  // ========== 场景53：地球自转/科里奥利补偿必须限制高纬高速传播误差 ==========
  const EarthRateCoriolisMetrics earth_rate_coriolis_metrics =
      runEarthRateCoriolisCompensationScenario(mag);
  assert(earth_rate_coriolis_metrics.expected_coriolis_speed_change_mps > 0.20f);
  assert(earth_rate_coriolis_metrics.max_velocity_error_mps < 0.020f);
  assert(earth_rate_coriolis_metrics.final_position_error_m < 0.10f);
  recordScenarioPassWithEarthRateCoriolisMetrics(
      report,
      "场景53：地球自转/科里奥利补偿必须限制高纬高速传播误差",
      earth_rate_coriolis_metrics);

  // ========== 场景54：真正双子样增量接口必须匹配双子样参考机械编排 ==========
  const TwoSampleMechanizationMetrics two_sample_metrics =
      runTrueTwoSampleMechanizationScenario(accel, gyro, mag, ned_vel, lla);
  assert(two_sample_metrics.average_accel_velocity_error_mps > 0.05f);
  assert(two_sample_metrics.max_quat_error_rad < 0.0020f);
  assert(two_sample_metrics.max_velocity_error_mps < 0.020f);
  assert(two_sample_metrics.final_position_error_m < 0.080f);
  recordScenarioPassWithTwoSampleMechanizationMetrics(
      report,
      "场景54：真正双子样增量接口必须匹配双子样参考机械编排",
      two_sample_metrics);

  // ========== 场景55：双子样历史增量必须支持GNSS延迟回放 ==========
  const TwoSampleDelayedReplayMetrics two_sample_replay_metrics =
      runTwoSampleDelayedReplayScenario(accel, gyro, mag, ned_vel, lla);
  assert(two_sample_replay_metrics.velocity_error_before_gnss_mps > 0.030f);
  assert(two_sample_replay_metrics.velocity_error_after_gnss_mps <
         0.20f * two_sample_replay_metrics.velocity_error_before_gnss_mps);
  assert(two_sample_replay_metrics.position_error_after_gnss_m <
         0.30f * two_sample_replay_metrics.position_error_before_gnss_m);
  assert(two_sample_replay_metrics.attitude_error_after_gnss_rad < 0.030f);
  assert(two_sample_replay_metrics.quat_jump_after_gnss_rad < 0.030f);
  assert(two_sample_replay_metrics.velocity_jump_after_gnss_mps < 0.080f);
  assert(two_sample_replay_metrics.position_jump_after_gnss_m < 0.080f);
  recordScenarioPassWithTwoSampleDelayedReplayMetrics(
      report,
      "场景55：双子样历史增量必须支持GNSS延迟回放",
      two_sample_replay_metrics);

  // ========== 场景56：姿态传播必须补偿导航系转动 ==========
  const NavigationFrameRotationMetrics nav_frame_rotation_metrics =
      runNavigationFrameRotationCompensationScenario(mag);
  assert(nav_frame_rotation_metrics.expected_nav_frame_rotation_rad > 0.01f);
  assert(nav_frame_rotation_metrics.covariance_failure_step < 0);
  assert(nav_frame_rotation_metrics.max_covariance_trace < 1.0e10f);
  assert(nav_frame_rotation_metrics.max_quat_error_rad < 0.0020f);
  recordScenarioPassWithNavigationFrameRotationMetrics(
      report,
      "场景56：姿态传播必须补偿导航系转动",
      nav_frame_rotation_metrics);

  // ========== 场景57：静止初始化不能把地球自转误扣为陀螺零偏 ==========
  const StaticEarthRateInitializationMetrics static_earth_rate_metrics =
      runStaticEarthRateInitializationScenario(mag);
  assert(static_earth_rate_metrics.expected_earth_rotation_rad > 0.01f);
  assert(static_earth_rate_metrics.initial_gyro_bias_norm_radps <
         0.25f * static_earth_rate_metrics.physical_earth_rate_norm_radps);
  assert(static_earth_rate_metrics.max_quat_drift_rad < 0.0020f);
  recordScenarioPassWithStaticEarthRateInitializationMetrics(
      report,
      "场景57：静止初始化不能把地球自转误扣为陀螺零偏",
      static_earth_rate_metrics);

  // ========== 场景58：飞控静止锁轴不能抹掉地球自转Z分量 ==========
  const StaticZLockEarthRateMetrics static_z_lock_metrics =
      runStaticZLockMustPreserveEarthRateScenario(mag);
  assert(static_z_lock_metrics.expected_removed_earth_rate_z_rad < 0.0020f);
  assert(static_z_lock_metrics.max_quat_drift_rad < 0.0020f);
  recordScenarioPassWithStaticZLockEarthRateMetrics(
      report,
      "场景58：飞控静止锁轴不能抹掉地球自转Z分量",
      static_z_lock_metrics);

  // ========== 场景59：复合姿态量测必须按四元数误差方向收敛 ==========
  const CompositeAttitudeMeasurementMetrics composite_attitude_metrics =
      runCompositeAttitudeMeasurementScenario(accel, gyro, mag, ned_vel, lla);
  assert(composite_attitude_metrics.quat_error_before_rad > 0.35f);
  assert(composite_attitude_metrics.correction_rad > 0.30f);
  assert(composite_attitude_metrics.quat_error_after_rad < 0.020f);
  recordScenarioPassWithCompositeAttitudeMeasurementMetrics(
      report,
      "场景59：复合姿态量测必须按四元数误差方向收敛",
      composite_attitude_metrics);

  // ========== 场景60：飞控接入层IMU多样本必须按时间中点正确拆成双子样 ==========
  const NavigationInputSplitMetrics navigation_input_split_metrics =
      runNavigationInputSplitScenario();
  assert(std::fabs(navigation_input_split_metrics.first_half_dt_s -
                   navigation_input_split_metrics.second_half_dt_s) < 1.0e-7f);
  assert(navigation_input_split_metrics.boundary_cross_fraction > 0.40f);
  assert(navigation_input_split_metrics.boundary_cross_fraction < 0.55f);
  assert(navigation_input_split_metrics.theta_split_error_rad < 1.0e-8f);
  assert(navigation_input_split_metrics.delta_v_split_error_mps < 1.0e-7f);
  recordScenarioPassWithNavigationInputSplitMetrics(
      report,
      "场景60：飞控接入层IMU多样本必须按时间中点正确拆成双子样",
      navigation_input_split_metrics);

  // 全部断言通过后写入总结果，作为本次组合导航主机仿真的归档结论。
  report << "通过场景数：60" << std::endl;
  report << "[PASS] 全部EKF主机回归场景通过" << std::endl;
  std::cout << "[PASS] 全部EKF主机回归场景通过" << std::endl;

  return 0;
}
