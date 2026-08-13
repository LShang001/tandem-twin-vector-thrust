#ifdef BFS_NAV_PROFILE
#include <stdint.h>
void recordNavProfileStage(uint8_t stage, uint32_t elapsed_us);
#define BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED 1
#define BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US() micros()
#define BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(stage, elapsed_us) \
  recordNavProfileStage(static_cast<uint8_t>(stage), static_cast<uint32_t>(elapsed_us))
#endif

#include "navigation_task.h"
#include "math_utils.h"
#include "QuaternionMath.h"

#include "ins_gnss_dynamic_weight.h"
#include "ins_gnss_epoch_timing.h"
#include "ins_altitude_reference.h"
#include "ins_static_detector.h"
#include "ins_static_aid_profile.h"

#include <cmath>

#ifdef BFS_NAV_PROFILE
namespace
{
struct NavProfileBucket
{
  uint32_t sum_us = 0;
  uint32_t max_us = 0;
  uint32_t count = 0;
};

constexpr uint8_t NAV_PROFILE_STAGE_COUNT = 12;
NavProfileBucket nav_profile_buckets[NAV_PROFILE_STAGE_COUNT];
uint32_t nav_profile_zupt_fuse_count = 0;
uint32_t nav_profile_zupt_reject_count = 0;
uint32_t nav_profile_zupt_skip_count = 0;
uint32_t nav_profile_zupt_scale_sum_x100 = 0;
uint32_t nav_profile_zupt_scale_count = 0;
float nav_profile_zupt_residual_max_mps = 0.0f;

void recordNavProfileZupt(const bool fused,
                          const bool skipped,
                          const float residual_mps,
                          const float noise_scale)
{
  if (skipped)
  {
    nav_profile_zupt_skip_count++;
  }
  else if (fused)
  {
    nav_profile_zupt_fuse_count++;
  }
  else
  {
    nav_profile_zupt_reject_count++;
  }
  if (std::isfinite(residual_mps) &&
      residual_mps > nav_profile_zupt_residual_max_mps)
  {
    nav_profile_zupt_residual_max_mps = residual_mps;
  }
  if (std::isfinite(noise_scale) && noise_scale > 0.0f)
  {
    nav_profile_zupt_scale_sum_x100 +=
        static_cast<uint32_t>(noise_scale * 100.0f + 0.5f);
    nav_profile_zupt_scale_count++;
  }
}

void printNavProfileBucket(const char *name, const NavProfileBucket &bucket)
{
  const uint32_t avg_us = bucket.count > 0 ? bucket.sum_us / bucket.count : 0;
  Serial8.print(" ");
  Serial8.print(name);
  Serial8.print("_avg=");
  Serial8.print(avg_us);
  Serial8.print(" ");
  Serial8.print(name);
  Serial8.print("_max=");
  Serial8.print(bucket.max_us);
  Serial8.print(" ");
  Serial8.print(name);
  Serial8.print("_cnt=");
  Serial8.print(bucket.count);
}

void clearNavProfileBuckets()
{
  for (uint8_t idx = 0; idx < NAV_PROFILE_STAGE_COUNT; ++idx)
  {
    nav_profile_buckets[idx] = NavProfileBucket{};
  }
}

void printNavProfileIfDue()
{
  static uint32_t last_print_ms = 0;
  const uint32_t now_ms = millis();
  if (now_ms - last_print_ms < 2000)
  {
    return;
  }
  last_print_ms = now_ms;

  Serial8.print("[NAVP]");
  printNavProfileBucket("geo", nav_profile_buckets[1]);
  printNavProfileBucket("mech", nav_profile_buckets[2]);
  printNavProfileBucket("lin", nav_profile_buckets[3]);
  printNavProfileBucket("cov", nav_profile_buckets[4]);
  printNavProfileBucket("stab", nav_profile_buckets[5]);
  printNavProfileBucket("time", nav_profile_buckets[6]);
  printNavProfileBucket("aid", nav_profile_buckets[7]);
  printNavProfileBucket("out", nav_profile_buckets[8]);
  printNavProfileBucket("total", nav_profile_buckets[9]);
#ifdef BFS_EKF_Z_BIAS_TUNING
  Serial8.print(" z_bias=");
  Serial8.print(ekf_accel_bias_z_mps2, 4);
#endif
#ifdef BFS_EKF_BARO_ALTITUDE_UPDATE
  Serial8.print(" b_innov=");
  Serial8.print(baro_ekf_innovation_m, 3);
  Serial8.print(" b_fused=");
  Serial8.print(baro_ekf_fused ? 1 : 0);
  Serial8.print(" b_nis_r=");
  Serial8.print(baro_ekf_test_ratio, 2);
#endif
  Serial8.print(" zf=");
  Serial8.print(nav_profile_zupt_fuse_count);
  Serial8.print(" zr=");
  Serial8.print(nav_profile_zupt_reject_count);
  Serial8.print(" zs=");
  Serial8.print(nav_profile_zupt_skip_count);
  Serial8.print(" zres_max=");
  Serial8.print(nav_profile_zupt_residual_max_mps, 3);
  Serial8.print(" zscale_avg=");
  const float zupt_scale_avg =
      nav_profile_zupt_scale_count > 0
          ? static_cast<float>(nav_profile_zupt_scale_sum_x100) /
                (100.0f * static_cast<float>(nav_profile_zupt_scale_count))
          : 0.0f;
  Serial8.print(zupt_scale_avg, 2);
  Serial8.println();

  clearNavProfileBuckets();
  nav_profile_zupt_fuse_count = 0;
  nav_profile_zupt_reject_count = 0;
  nav_profile_zupt_skip_count = 0;
  nav_profile_zupt_scale_sum_x100 = 0;
  nav_profile_zupt_scale_count = 0;
  nav_profile_zupt_residual_max_mps = 0.0f;
}
} // namespace
void recordNavProfileStage(uint8_t stage, uint32_t elapsed_us)
{
  if (stage >= NAV_PROFILE_STAGE_COUNT)
  {
    return;
  }
  NavProfileBucket &bucket = nav_profile_buckets[stage];
  bucket.sum_us += elapsed_us;
  if (elapsed_us > bucket.max_us)
  {
    bucket.max_us = elapsed_us;
  }
  bucket.count++;
}
#endif
// ================================================================================================
// GNSS 动态权重配置（替代旧硬门限）
static constexpr Icm45686GnssDynamicWeightConfig kGnssDwCfg{
    .floor_pos_ne_m   = 2.0f,
    .floor_pos_d_m    = 3.0f,
    .floor_vel_ne_mps = 0.15f,
    .floor_vel_d_mps  = 0.25f,
    .cap_pos_ne_m     = 30.0f,
    .cap_pos_d_m      = 50.0f,
    .cap_vel_mps      = 3.0f,
    .pdop_ref         = 2.0f,
    .min_sv           = 3,
};
// 统一静止检测器（ins_static_detector.h，平台无关纯算法）
static Icm45686StaticDetector static_det;
static constexpr float GNSS_FIRST_REANCHOR_MAX_AGE_S = 0.15f;

static bool gnssEpochPassesFullQuality(const bfs::UbxEpoch &epoch)
{
  // 动态权重：最小门限仅需 fix>=3D + sv>=3 + h_acc>0，接收机精度直接作为 R
  const auto dw = Icm45686ComputeGnssDynamicWeights(
      static_cast<int8_t>(epoch.fix), epoch.num_sv,
      epoch.horz_acc_m, epoch.vert_acc_m, epoch.spd_acc_mps,
      epoch.pvt_pdop, kGnssDwCfg);

  // ★ 2026-08-10 全面审查修复：gnss_status 快照已移至本函数全部质量检查
  //   通过后（return true 前）——原实现在检查前写 has_obs/last_obs_*，
  //   被拒 epoch（tow 不匹配/重复 iTOW/h_acc 不达标）也留下 3D fix 快照，
  //   地面站显示 fix 而 EKF 实际未融合该帧

  if (epoch.pvt_tow_ms != epoch.eoe_tow_ms)
  {
    gnss_status.tow_mismatch_count++;
    aid_tracker.MarkAvailable(bfs::AidSource::Gnss, false);
    return false;
  }

  if (gnss_status.has_last_tow && epoch.pvt_tow_ms == gnss_status.last_tow_ms)
  {
    gnss_status.duplicate_itow_count++;
    aid_tracker.MarkAvailable(bfs::AidSource::Gnss, false);
    return false;
  }

  if (!dw.passed_minimum)
  {
    gnss_status.quality_fail_count++;
    return false;
  }

  gnss_status.fix3d_count++;
  // ★ 2026-08-10 全面审查修复：快照只反映通过全部质量检查的 epoch
  gnss_status.has_obs = true;
  gnss_status.last_obs_fix_type = static_cast<float>(epoch.fix);
  gnss_status.last_obs_num_sv = static_cast<float>(epoch.num_sv);
  gnss_status.last_obs_h_acc_m = epoch.horz_acc_m;
  gnss_status.last_obs_v_acc_m = epoch.vert_acc_m;
  gnss_status.last_obs_s_acc_mps = epoch.spd_acc_mps;
  gnss_status.last_obs_p_dop = epoch.pvt_pdop;
  gnss_status.last_obs_vel_n_mps = epoch.north_vel_mps;
  gnss_status.last_obs_vel_e_mps = epoch.east_vel_mps;
  gnss_status.last_obs_vel_d_mps = epoch.down_vel_mps;
  gnss_status.last_obs_lla_rad_m = Eigen::Vector3d(
      epoch.lat_rad, epoch.lon_rad, static_cast<double>(epoch.alt_wgs84_m));
  return true;
}

static bool gnssEpochQualifiesForDowngrade(const bfs::UbxEpoch &epoch)
{
  if (epoch.pvt_tow_ms != epoch.eoe_tow_ms)
  {
    return false;
  }
  if (gnss_status.has_last_tow && epoch.pvt_tow_ms == gnss_status.last_tow_ms)
  {
    return false;
  }

  const auto fix = static_cast<bfs::Ubx::Fix>(epoch.fix);
  return (fix >= bfs::Ubx::FIX_3D) &&
         (epoch.num_sv >= GNSS_MIN_SV) &&
         (epoch.spd_acc_mps > 0.0f && epoch.spd_acc_mps < GNSS_DOWNGRADE_MAX_SPD_ACC_MPS) &&
         (epoch.pvt_pdop <= 0.0f || epoch.pvt_pdop < GNSS_MAX_PDOP);
}

static void reanchorNavigationToGnss(const Eigen::Vector3f &gnss_vel_ned,
                                     const Eigen::Vector3d &gnss_lla,
                                     const float measurement_age_s)
{
  const Eigen::Vector3d current_gnss_lla = bfs::ned2lla(
      gnss_vel_ned.cast<double>() * static_cast<double>(measurement_age_s),
      gnss_lla, bfs::AngPosUnit::RAD);
  Eigen::Vector3d takeoff_to_current_ned = Eigen::Vector3d::Zero();
  const bool preserve_takeoff_origin = g_is_unlocked && is_origin_lla_set;
  if (preserve_takeoff_origin)
  {
    const Eigen::Vector3d old_origin_lla(
        origin_lat_rad, origin_lon_rad, origin_alt_m);
    takeoff_to_current_ned = bfs::lla2ned(
        nav_ekf.lla_rad_m(), old_origin_lla, bfs::AngPosUnit::RAD);
  }

  nav_ekf.ResetPositionVelocityToGnss(gnss_vel_ned, current_gnss_lla);

  if (preserve_takeoff_origin)
  {
    // GNSS 给出当前绝对位置，反向平移已积累的相对位移即可恢复真实起飞点。
    Eigen::Vector3d takeoff_lla = bfs::ned2lla(
        -takeoff_to_current_ned, current_gnss_lla, bfs::AngPosUnit::RAD);
    takeoff_lla(2) = InsTakeoffOriginAltitudeFromCurrent(
        current_gnss_lla(2), takeoff_to_current_ned(2));
    setNavigationOriginFromLlaRad(
        takeoff_lla(0), takeoff_lla(1), takeoff_lla(2));
    relative_north = static_cast<float>(takeoff_to_current_ned(0));
    relative_east = static_cast<float>(takeoff_to_current_ned(1));
    relative_down = static_cast<float>(takeoff_to_current_ned(2));
  }
  else
  {
    setNavigationOriginFromLlaRad(current_gnss_lla(0), current_gnss_lla(1),
                                 current_gnss_lla(2));
    relative_north = 0.0f;
    relative_east = 0.0f;
    relative_down = 0.0f;
  }
  is_origin_lla_set = true;
}

static bool applyDowngradedGnssUpdate(const bfs::UbxEpoch &epoch,
                                      float dt_s,
                                      float measurement_age_s)
{
  const Eigen::Vector3f gnss_vel_ned(epoch.north_vel_mps,
                                     epoch.east_vel_mps,
                                     epoch.down_vel_mps);
  const float inflated_pos_std_m = 1.0e4f; // 等价于关闭位置融合，避免弱 GNSS 位置拉偏原点。
  nav_ekf.gnss_pos_ne_std_m(inflated_pos_std_m);
  nav_ekf.gnss_pos_d_std_m(inflated_pos_std_m);
  nav_ekf.gnss_vel_ne_std_mps(epoch.spd_acc_mps);
  nav_ekf.gnss_vel_d_std_mps(epoch.spd_acc_mps);

  // ★ 2026-08-12 审查修复：原实现把 nav_ekf.lla_rad_m()（EKF 自身当前估计）
  // 当作"GNSS 位置量测"，位置新息变成"EKF 当前 vs EKF 重放快照"的自差≈0，
  // 是语义错误（R=1e4 下当前影响小，但未来调低 R 会形成假位置融合）。
  // 改传 epoch 的真实 LLA，与全质量路径（本文件 1116 行）一致；
  // 位置融合关闭仍由 R=1e4 保证（防止弱位置拉偏原点），语义不再依赖"自差=0"。
  const Eigen::Vector3d gnss_lla_rad(
      epoch.lat_rad, epoch.lon_rad, static_cast<double>(epoch.alt_wgs84_m));

  const bfs::MeasurementUpdateResult result =
      nav_ekf.MeasurementUpdateDetailed(gnss_vel_ned, gnss_lla_rad,
                                        dt_s, measurement_age_s);
  gnss_status.last_test_ratio = result.test_ratio;
  gnss_status.fix3d_count++;  // 降级帧也是有效 3D fix
  aid_tracker.RecordAttempt(bfs::AidSource::Gnss, result);

  // 降级融合后也执行 AHRS yaw 慢对齐
  {
    // ★ 2026-08-12 审查修复：backup_ahrs_yaw 在数据源处已叠加校正量
    // （sensor_imu.cpp:406 backup_ahrs_yaw = wrap(yaw + ahrs_yaw_correction_rad)），
    // 原式再减 ahrs_yaw_correction_rad 会双重抵消，使 yaw_err = EKF - raw 与校正量无关，
    // 闭环退化为开环积分器，长降级下校正量持续缠绕。此处与全质量路径（本文件 1183 行）统一。
    float yaw_err = wrapAnglePi(nav_ekf.yaw_rad() - backup_ahrs_yaw);
    yaw_err = constrain(yaw_err, -0.0025f, 0.0025f /* ~28.6 deg/s @200Hz */);
    ahrs_yaw_correction_rad = wrapAnglePi(ahrs_yaw_correction_rad + yaw_err);
  }
  return result.fused;
}


bool isGnssDataFreshForNav()
{
  if (last_gnss_data_ms == 0)
    return false;
  return (millis() - last_gnss_data_ms) <= GNSS_NAV_DATA_TIMEOUT_MS;
}

bool isGnssInstantValidForNav()
{
  return (ubx.fix() >= bfs::Ubx::FIX_3D) && isGnssDataFreshForNav();
}

bool isGnssOutputValidForNav()
{
  // 导航输出允许短时间保持最近一次有效GNSS状态，但不能把EKF假原点LLA误判为真实GNSS。
  if (isGnssInstantValidForNav())
  {
    return true;
  }
  if (last_gnss_nav_valid_ms == 0)
  {
    return false;
  }
  return (millis() - last_gnss_nav_valid_ms) <= GNSS_NAV_DROPOUT_HOLD_MS;
}

GPSFixType mapUbxFixToDeta100FixType(bfs::Ubx::Fix ubx_fix)
{
  switch (ubx_fix)
  {
  case bfs::Ubx::FIX_NONE:
    return DETA100_GPS_FIX_TYPE_NO_FIX;
  case bfs::Ubx::FIX_2D:
    return DETA100_GPS_FIX_TYPE_2D_FIX;
  case bfs::Ubx::FIX_3D:
    return DETA100_GPS_FIX_TYPE_3D_FIX;
  case bfs::Ubx::FIX_DGNSS:
    return DETA100_GPS_FIX_TYPE_DGPS;
  case bfs::Ubx::FIX_RTK_FLOAT:
    return DETA100_GPS_FIX_TYPE_RTK_FLOAT;
  case bfs::Ubx::FIX_RTK_FIXED:
    return DETA100_GPS_FIX_TYPE_RTK_FIXED;
  default:
    return DETA100_GPS_FIX_TYPE_NO_FIX;
  }
}
void setNavigationOriginFromLlaRad(const double lat_rad, const double lon_rad, const double alt_m)
{
  origin_lat_rad = lat_rad;
  origin_lon_rad = lon_rad;
  origin_lat_deg = lat_rad * RAD_TO_DEG;
  origin_lon_deg = lon_rad * RAD_TO_DEG;
  origin_alt_m = alt_m;
}

void handleDualVectorYawFusion()
{
  // ========================= 参数配置 (Tuning) =========================

  // 1. 速度门限: 速度越快，航向几何解算越准
  // 共轴机震动大，建议设高一点，比如 1.5m/s
  const float MIN_SPEED_MPS = 1.0f;

  // 2. 角速度门限: 仅在转弯平缓时信任解算 (单位: rad/s)
  // 0.35 rad/s ≈ 20 deg/s。超过这个转速认为是在剧烈机动，暂停修正。
  const float MAX_TURN_RATE_RADPS = 0.35f;

  // 3. 模长一致性容忍度: GPS速度与光流速度的比例差异
  // 0.3 表示允许 30% 的误差 (0.7 ~ 1.3)。如果光流高度算错，这个比例会严重偏离。
  const float SCALE_CHECK_TOLERANCE = 0.35f;

  // 4. 观测噪声模型
  const float BASE_NOISE_RAD = 0.08f; // 基础噪声 (~4.5度)
  const float MIN_NOISE_RAD = 0.03f;  // 最小噪声 (~1.7度)，防止过拟合

  // ========================= 1. 状态预检查 =========================

  // ★ 2026-08-12 审查修复：静止时 GNSS 速度纯噪声（多径可达 0.5-1 m/s），
  // 若仍用原始 GNSS 速度解算航迹角，会把噪声航向当真实航向融合（静止+噪声下
  // 约 9% 帧穿过三道门控、航向噪声 σ≈145°）。静止时位置更新已强制速度=0
  // （本文件 1118-1123 行），航向融合应同步跳过，避免与"静止即零速"语义矛盾。
  if (static_det.confirmed_static)
  {
    return;
  }

  // 检查 GPS 状态：必须是新鲜的 3D Fix，不能使用UBX缓存中的旧速度矢量修正航向。
  if (!isGnssInstantValidForNav())
    return;

  // 检查 光流 状态 (必须有效)
  if (!flow_data.is_flow_valid)
    return;

  // ========================= 2. 数据获取与预处理 =========================

  // A. 获取 GNSS 速度 (NED系) - 使用本帧已通过质量检查的 epoch 快照，
  //    而非 UBX 最新缓存，避免 epoch 队列堆积时速度矢量与融合位置属不同历元。
  float v_n_gps = gnss_status.last_obs_vel_n_mps;
  float v_e_gps = gnss_status.last_obs_vel_e_mps;
  // 计算 GPS 地速模长
  float speed_gps = sqrtf(v_n_gps * v_n_gps + v_e_gps * v_e_gps);

  // B. 获取 光流 速度 (机体系, 已做旋转补偿)
  float v_x_flow = flow_data.velocity_x_mps;
  float v_y_flow = flow_data.velocity_y_mps;
  // 计算 光流 速度模长
  float speed_flow = sqrtf(v_x_flow * v_x_flow + v_y_flow * v_y_flow);

  // C. 获取 当前角速度 (用于判断是否飞直线)
  // 使用低通滤波后的角速度 (单位: rad/s)
  // 注意：current_omega_dps_body_filtered 是 deg/s，需要转 rad/s
  float gyro_z_radps = fabsf(current_omega_dps_body_filtered.z * DEG_TO_RAD);

  // ========================= 3. 核心防御逻辑 (Gatekeepers) =========================

  // [防御 1] 低速保护
  // 速度太低时，atan2 极其敏感，稍微一点噪声就会导致角度乱跳
  if (speed_gps < MIN_SPEED_MPS || speed_flow < MIN_SPEED_MPS)
  {
    return;
  }

  // [防御 2] 大机动保护 (新增)
  // 如果飞机正在快速转弯，光流和GPS的时间同步误差、安装臂效应会放大
  // 此时解算出的航向不可信，暂停修正，依靠陀螺仪短时积分即可
  if (gyro_z_radps > MAX_TURN_RATE_RADPS)
  {
    return;
  }

  // [防御 3] 尺度一致性检查 (Scale Consistency)
  // 物理上 V_gps 和 V_flow 应该是相等的。
  // 如果差异过大，说明光流的高度数据(distance_m)可能有误（例如飞过障碍物上方）
  float ratio = speed_flow / speed_gps;
  if (fabsf(ratio - 1.0f) > SCALE_CHECK_TOLERANCE)
  {
    return;
  }

  // ========================= 4. 几何解算 =========================

  // A. 计算 航迹角 (Track Angle) - 飞机相对于地面的运动方向
  // atan2(Y, X) -> atan2(East, North) -> 0度朝北, 90度朝东
  float track_angle_rad = atan2f(v_e_gps, v_n_gps);

  // B. 计算 侧滑角 (Sideslip Angle) - 飞机运动方向相对于机头的夹角
  // atan2(Y, X) -> atan2(Right, Front)
  float sideslip_angle_rad = atan2f(v_y_flow, v_x_flow);

  // C. 推算 真实航向 (True Heading)
  // 关系式: 航迹角 = 航向角 + 侧滑角
  // 所以:   航向角 = 航迹角 - 侧滑角
  float estimated_yaw = track_angle_rad - sideslip_angle_rad;

  // D. 角度归一化 (-PI ~ +PI)
  estimated_yaw = wrapAnglePi(estimated_yaw);

  // ========================= 5. 动态观测噪声模型 =========================

  // 策略：飞得越快，几何解算越准，噪声 R 越小（EKF 越信任）
  // 模型：Noise = Base / (Speed_Excess + 1)
  // 当 Speed = 2.0 (门限) 时, Noise = Base / 1 = 0.08 (约4.5度)
  // 当 Speed = 10.0 时,       Noise = Base / 9 ≈ 0.01 (极小)
  float speed_excess = speed_gps - MIN_SPEED_MPS;
  float dynamic_noise = BASE_NOISE_RAD / (speed_excess + 1.0f);

  // 限制下限，防止过度自信 (Over-confidence)
  if (dynamic_noise < MIN_NOISE_RAD)
    dynamic_noise = MIN_NOISE_RAD;

  // ========================= 6. 执行 EKF 更新 =========================

  // 调用 Ekf15State 中新增的标量更新接口
  // 这一步会将计算出的 Yaw 融合进滤波器，并修正陀螺仪零偏
  const bfs::MeasurementUpdateResult result = nav_ekf.MeasurementUpdateYawDetailed(estimated_yaw, dynamic_noise);
  aid_tracker.RecordAttempt(bfs::AidSource::YawGsf, result);

  // (可选) 调试输出，用于在地面站观察融合效果
  // Serial8.print("Fusion: Yaw_Est=");
  // Serial8.print(estimated_yaw * RAD_TO_DEG);
  // Serial8.print(" EKF_Yaw=");
  // Serial8.print(nav_ekf.yaw_rad() * RAD_TO_DEG);
  // Serial8.print(" Noise=");
  // Serial8.println(dynamic_noise);
}

void updateUbxRelativePosition()
{
  // 1. 基础检查：原点已设置、导航已初始化，且GNSS必须是新鲜3D定位。
  if (!is_origin_lla_set || !isGnssInstantValidForNav() || !nav_system_initialized)
  {
    // GNSS无效时不能用UBX缓存覆盖Home Point，只清零原始相对位移，保留已确认的导航原点。
    ubx_relative_north = 0.0f;
    ubx_relative_east = 0.0f;
    ubx_relative_down = 0.0f;
    return;
  }

  // 2. 使用本帧已通过质量检查的 GNSS 观测快照 (来自 gnssEpochPassesFullQuality)，
  //    而非 UBX 最新缓存，确保相对位置与 EKF 融合的 GNSS 量测属同一历元。
  Eigen::Vector3d current_lla_rad = gnss_status.last_obs_lla_rad_m;
  Eigen::Vector3d origin_lla_rad;
  origin_lla_rad << origin_lat_rad, origin_lon_rad, origin_alt_m;

  // 3. 执行坐标转换 (LLA -> NED)，显式指定输入为 RAD
  Eigen::Vector3d ned_pos = bfs::lla2ned(current_lla_rad, origin_lla_rad, bfs::AngPosUnit::RAD);

  // 4. 更新全局变量 (转为 float 供控制/回传使用)
  ubx_relative_north = (float)ned_pos(0);
  ubx_relative_east = (float)ned_pos(1);
  ubx_relative_down = (float)ned_pos(2);
}

void handleNavigationSystem()
{
  // EKF 总耗时测量 + 零偏诊断
  const uint32_t _ekf_t0 = micros();
#ifdef BFS_NAV_PROFILE
  const uint32_t _nav_profile_total_start = _ekf_t0;
  uint32_t _nav_profile_stage_start = _ekf_t0;
#endif
  static uint32_t _ekf_time_sum = 0, _ekf_time_cnt = 0, _ekf_last = 0;
  static uint32_t _sg_attempt = 0, _sg_fused = 0;

  // 1. 获取当前时间与 dt
  unsigned long now_us = micros();
  float dt_s = (now_us - last_ekf_update_us) / 1e6f;
  last_ekf_update_us = now_us;

  // 保护：防止 dt 异常导致 EKF 发散
  if (dt_s <= 0.0001f || dt_s > 0.1f)
    dt_s = 0.005f;

  // ====================================================================
  // 2. 准备传感器数据 (IMU & Mag)
  // ====================================================================
  // 从全局变量获取 IMU 数据 (已由 readIMUData 处理过，单位: rad/s, m/s^2)
  // 注意：BFS EKF 期望 IMU 数据为 FRD (前右下) 坐标系
  // 代码中 icm_accel_x 已经是机体系(FRD)下的 m/s^2
  // icm_gyro_x 已经是机体系(FRD)下的 rad/s (在 readIMUData 中被转换过)

  Eigen::Vector3f imu_accel_mps2;
  Eigen::Vector3f imu_gyro_radps;
  Eigen::Vector3f mag_ut;
  Eigen::Vector3f delta_theta_1_rad = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_theta_2_rad = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_v_1_mps = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_v_2_mps = Eigen::Vector3f::Zero();
  float local_acc_sum_x = 0.0f;
  float local_acc_sum_y = 0.0f;
  float local_acc_sum_z = 0.0f;
  float local_gyro_sum_x = 0.0f;
  float local_gyro_sum_y = 0.0f;
  float local_gyro_sum_z = 0.0f;
  float local_delta_theta_x[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_theta_y[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_theta_z[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_v_x[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_v_y[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_v_z[NAV_IMU_DELTA_BUFFER_SIZE];
  float local_delta_dt_s[NAV_IMU_DELTA_BUFFER_SIZE];

  noInterrupts();
  const int local_imu_sample_count = imu_sample_count;
  if (local_imu_sample_count == 0)
  {
    interrupts();
#ifdef BFS_NAV_PROFILE
    recordNavProfileStage(9U, micros() - _nav_profile_total_start);
    printNavProfileIfDue();
#endif
    return; // 防止除零
  }
  local_acc_sum_x = acc_sum_x;
  local_acc_sum_y = acc_sum_y;
  local_acc_sum_z = acc_sum_z;
  local_gyro_sum_x = gyro_sum_x;
  local_gyro_sum_y = gyro_sum_y;
  local_gyro_sum_z = gyro_sum_z;
  const int local_delta_sample_count = imu_delta_sample_count;
  const bool local_delta_overflow = imu_delta_overflow;
  const int local_delta_count =
      (local_delta_sample_count < NAV_IMU_DELTA_BUFFER_SIZE)
          ? local_delta_sample_count
          : NAV_IMU_DELTA_BUFFER_SIZE;
  const float local_delta_time_sum_s = imu_delta_time_sum_s;
  for (int idx = 0; idx < local_delta_count; ++idx)
  {
    local_delta_theta_x[idx] = imu_delta_theta_x[idx];
    local_delta_theta_y[idx] = imu_delta_theta_y[idx];
    local_delta_theta_z[idx] = imu_delta_theta_z[idx];
    local_delta_v_x[idx] = imu_delta_v_x[idx];
    local_delta_v_y[idx] = imu_delta_v_y[idx];
    local_delta_v_z[idx] = imu_delta_v_z[idx];
    local_delta_dt_s[idx] = imu_delta_dt_s[idx];
  }
  acc_sum_x = 0;
  acc_sum_y = 0;
  acc_sum_z = 0;
  gyro_sum_x = 0;
  gyro_sum_y = 0;
  gyro_sum_z = 0;
  imu_sample_count = 0;
  imu_delta_time_sum_s = 0.0f;
  imu_delta_sample_count = 0;
  imu_delta_overflow = false;
  interrupts();

  const float scale = 1.0f / local_imu_sample_count;

  // 使用 IMU 数据降采样后的平均值
  imu_accel_mps2 << local_acc_sum_x * scale, local_acc_sum_y * scale, local_acc_sum_z * scale;
  imu_gyro_radps << local_gyro_sum_x * scale, local_gyro_sum_y * scale, local_gyro_sum_z * scale;
  const float first_sample_target_dt_s = 0.5f * local_delta_time_sum_s;
  float accumulated_delta_time_s = 0.0f;
  for (int idx = 0; idx < local_delta_count; ++idx)
  {
    Eigen::Vector3f sample_delta_theta;
    Eigen::Vector3f sample_delta_v;
    const float sample_dt_s = local_delta_dt_s[idx];
    sample_delta_theta << local_delta_theta_x[idx],
        local_delta_theta_y[idx],
        local_delta_theta_z[idx];
    sample_delta_v << local_delta_v_x[idx],
        local_delta_v_y[idx],
        local_delta_v_z[idx];
    if (sample_dt_s <= 0.0f)
    {
      continue;
    }
    const float next_accumulated_delta_time_s = accumulated_delta_time_s + sample_dt_s;
    if (next_accumulated_delta_time_s <= first_sample_target_dt_s)
    {
      delta_theta_1_rad += sample_delta_theta;
      delta_v_1_mps += sample_delta_v;
    }
    else if (accumulated_delta_time_s >= first_sample_target_dt_s)
    {
      delta_theta_2_rad += sample_delta_theta;
      delta_v_2_mps += sample_delta_v;
    }
    else
    {
      // 单个IMU样本跨越导航周期中点时按时间比例拆分，避免采样抖动破坏双子样半周期定义。
      const float first_fraction =
          (first_sample_target_dt_s - accumulated_delta_time_s) / sample_dt_s;
      const float clamped_first_fraction = fminf(fmaxf(first_fraction, 0.0f), 1.0f);
      delta_theta_1_rad += clamped_first_fraction * sample_delta_theta;
      delta_v_1_mps += clamped_first_fraction * sample_delta_v;
      delta_theta_2_rad += (1.0f - clamped_first_fraction) * sample_delta_theta;
      delta_v_2_mps += (1.0f - clamped_first_fraction) * sample_delta_v;
    }
    accumulated_delta_time_s = next_accumulated_delta_time_s;
  }
  const bool use_two_sample_imu =
      (!local_delta_overflow) && (local_delta_count >= 2) &&
      (local_delta_time_sum_s > 0.0001f) && (local_delta_time_sum_s <= 0.1f) &&
      ((delta_theta_1_rad + delta_theta_2_rad).allFinite()) &&
      ((delta_v_1_mps + delta_v_2_mps).allFinite());
  // EKF 传播和 GNSS 延迟回放统一使用本帧实际消费的 IMU 时间跨度，避免调度抖动带来比例误差。
  // ★ 2026-08-12 审查修复：缓冲溢出时（任务卡顿 >32ms）imu_delta_time_sum_s 只含前 64 样本，
  // 直接使用会低估传播 dt（卡顿 64-100ms 时低估 20-36%），使速度/姿态积分欠量。
  // 改为以"本帧消费的真实时间跨度"为准：双子样路径用缓冲内时间（增量与时间同源自洽），
  // 溢出/降级路径用 micros 差分（覆盖全部样本时间）。
  const float nav_update_dt_s =
      (use_two_sample_imu)
          ? local_delta_time_sum_s
          : dt_s;  // 单样本/溢出路径：micros 差分覆盖全部样本，已含卡顿时长
  aid_tracker.BeginFrame(nav_update_dt_s);

  // 获取磁力计数据 (用于初始化航向)
  // IST8310_Vector mag_raw = compass.get_data();
  // 坐标系对齐：将 IST8310 原始坐标系旋转到机体 FRD
  // 假设安装方向与之前 Madgwick 代码一致：
  // mx_frd = -my_raw, my_frd = -mx_raw, mz_frd = mz_raw
  // mag_ut << -mag_raw.y, -mag_raw.x, mag_raw.z;

  // 强制磁场矢量指向正北 (X轴)，Z轴分量是为了模拟地磁倾角(Dip Angle)，防止奇异值
  // 这里的 {50, 0, 0} 代表磁力线完全水平指向前方
  // 如果在长沙，地磁倾角约 45度，写成 {30, 0, 40} 更符合物理真实，利于 TiltCompass 解算
  mag_ut << 30.0f, 0.0f, 40.0f;

  // ====================================================================
  // 3. 统一静止检测 (Unified Static Detection)
  // ====================================================================
  //
  // 检测条件：
  // 1. 加速度模长在 9.5 ~ 10.1 m/s² 范围内（接近重力）
  // 2. 角速度模长足够小 (< 0.01 rad/s)
  // 3. 相邻帧的加速度和角速度变化都很小（防止单帧噪声误触发）
  // 4. 连续 50 帧满足条件才认为真正静止（约 250ms @200Hz）
  //
  // 注意：不依赖 GNSS 速度条件，避免 GNSS 噪声导致漏触发
  //

  // A. 统一静止检测（Icm45686StaticDetector，含迟滞+置信度）
  static bool static_det_configured = false;
  if (!static_det_configured) {
    Icm45686StaticDetectorConfigureThresholds(
        static_det,
        9.5f, 10.1f, 0.01f, 0.5f, 0.05f);
    // 减少"运动→静止"死区：原 50 帧(250ms) 改为 20 帧(100ms)。
    // 减少后 ZUPT 更早启动，运动停止后速度积分误差更小；代价是误判概率略增，
    // 但迟滞机制（exit_min_frames=5）足以过滤单帧抖动。
    static_det.enter_min_frames = 20;
    static_det.exit_min_frames = 5;
    static_det_configured = true;
  }
  Icm45686StaticDetectorUpdate(imu_accel_mps2, imu_gyro_radps, static_det);
  bool is_static_confirmed = static_det.confirmed_static;
  const float static_confidence = static_det.confidence;

  // GNSS 数据新鲜度由 UBX epoch 队列驱动；每次成功出队就是唯一消费凭证。
  gnss_status.epoch_count = ubx.queued_epoch_count();
  gnss_status.pending_epoch_count = ubx.pending_epoch_count();
  gnss_status.parser_queue_overflow_count = ubx.queue_overflow_count();
  gnss_status.parser_eoe_without_pvt_count = ubx.eoe_without_pvt_count();
  gnss_status.parser_nav_pvt_count = ubx.nav_pvt_count();
  gnss_status.parser_nav_eoe_count = ubx.nav_eoe_count();
  gnss_status.tow_mismatch_count = ubx.tow_mismatch_count();

  bfs::UbxEpoch gnss_epoch;
  const bool has_gnss_epoch = ubx.PopEpoch(&gnss_epoch);
  // 注：NMEA 兜底已内聚进库（2026-08-09）——kAuto 模式下 UBX 失效时库内自动
  // 合成 NMEA epoch 入同一队列，导航层无感（见 lib/ublox-main Ubx::TryPushNmeaEpoch）。
  const bool perform_measurement_update =
      InsGnssShouldConsumeEpoch(has_gnss_epoch, false);
  float gnss_measurement_age_s = BFS_NAVIGATION_GNSS_DELAY_S;
  bool gnss_time_mapping_valid = false;
  if (has_gnss_epoch)
  {
    gnss_status.pending_epoch_count = ubx.pending_epoch_count();
    gnss_status.has_measurement = true;
    gnss_status.has_last_epoch_tow = true;
    gnss_status.last_epoch_tow_ms = gnss_epoch.pvt_tow_ms;
    const InsGnssEpochTimeMapping time_mapping = InsGnssMapEpochToLocalTime(
        gnss_epoch.pvt_tow_ms,
        gnss_epoch.receive_time_us,
        ubx.latest_epoch_tow_ms(),
        ubx.latest_epoch_receive_time_us(),
        static_cast<uint32_t>(BFS_NAVIGATION_GNSS_DELAY_S * 1000000.0f));
    if (time_mapping.valid)
    {
      gnss_time_mapping_valid = true;
      gnss_measurement_age_s = InsGnssEpochAgeSeconds(time_mapping, micros());
      gnss_status.last_measurement_age_s = gnss_measurement_age_s;
    }
    else
    {
      gnss_status.time_mapping_invalid_count++;
    }
  }

  const bool gnss_data_fresh_for_nav = isGnssDataFreshForNav();
  const bool gnss_full_quality_for_nav = has_gnss_epoch && gnssEpochPassesFullQuality(gnss_epoch);
  const bool gnss_downgrade_quality_for_nav = has_gnss_epoch && !gnss_full_quality_for_nav && gnssEpochQualifiesForDowngrade(gnss_epoch);
  const bool gnss_instant_valid_for_nav = gnss_full_quality_for_nav;
  if (gnss_full_quality_for_nav || gnss_downgrade_quality_for_nav)
  {
    gnss_status.has_last_tow = true;
    gnss_status.last_tow_ms = gnss_epoch.pvt_tow_ms;
    aid_tracker.MarkAvailable(bfs::AidSource::Gnss, true);
  }
  else if (has_gnss_epoch)
  {
    aid_tracker.MarkAvailable(bfs::AidSource::Gnss, false);
  }

  if (gnss_instant_valid_for_nav)
  {
    last_gnss_nav_valid_ms = millis();
  }

  // ====================================================================
  // 4. EKF 状态机管理
  // ====================================================================

  if (!nav_system_initialized)
  {
    // --- 初始化阶段 ---
    // 只要确认静止，即使没有 GPS 也允许初始化
    if (is_static_confirmed)
    {
      Serial8.println("[Nav] Static Detected. Attempting EKF Init...");

      // 如果 EKF 未初始化，用原始GNSS经纬高覆盖 Geodetic_Pos_Packet (大地坐标系位置)
      if (gnss_instant_valid_for_nav)
      {
        Geodetic_Pos_Packet.latitude = gnss_epoch.lat_rad;
        Geodetic_Pos_Packet.longitude = gnss_epoch.lon_rad;
        Geodetic_Pos_Packet.height = gnss_epoch.alt_wgs84_m;
      }

      // 同时也把位置填进去 (注意 INS 包通常是 NED 相对位置，或者也是 LLA)
      // 如果 INS_GNSS_Packet.location_north 是绝对位置 (如 UTM)，这里很难填
      // 如果是相对位置，设为 0
      INS_GNSS_Packet.location_north = 0.0f;
      INS_GNSS_Packet.location_east = 0.0f;
      INS_GNSS_Packet.location_down = 0.0f;

      // 准备初始状态向量
      Eigen::Vector3f init_vel_ned = Eigen::Vector3f::Zero(); // 初始速度设为0
      Eigen::Vector3d init_lla;

      // 检查 GPS 质量
      bool gps_available = gnss_instant_valid_for_nav;

      if (gps_available)
      {
        // [情况A] 有 GPS: 使用真实坐标
        init_lla << gnss_epoch.lat_rad, gnss_epoch.lon_rad,
            static_cast<double>(gnss_epoch.alt_wgs84_m);
        Serial8.println("[Nav] Init with VALID GPS Fix.");
      }
      else
      {
        // [情况B] 无 GPS: 使用默认坐标 (伪造原点)
        // 默认坐标设为长沙 (28.2N, 112.9E) 或任意合理值
        // 这对于局部姿态解算没有影响，但能让 EKF 建立重力模型
        double def_lat = 28.2 * DEG_TO_RAD;
        double def_lon = 112.9 * DEG_TO_RAD;
        double def_alt = 50.0;
        init_lla << def_lat, def_lon, def_alt;
        Serial8.println("[Nav] Init with FAKE Origin (No GPS).");
      }

      // ------------------------------------------------------------
      // EKF 噪声参数配置 (针对 ICM-42688 调优)
      // ------------------------------------------------------------

      // 1. 陀螺仪参数 (ICM-42688 精度极高，可以信任)
      // 降低测量噪声，提高姿态响应速度
      nav_ekf.gyro_std_radps(0.0015f);              // 减小 gyro_std 告诉 EKF："陀螺仪很准，别轻易改我的航向"
      // 零偏一阶 Gauss-Markov 驱动噪声: 设为 0.3 dps = 0.00524 rad/s。
      // 之前 0.05dps 过小导致 P 矩阵坍缩, Kalman 增益趋于零, Z 轴零偏无法修正。
      // 提高到 0.3dps 确保 EKF 持续信任新量测, 不会在长静止后锁死旧零偏估计。
      nav_ekf.gyro_markov_bias_std_radps(0.00524f); // 0.3 dps → rad/s
      nav_ekf.gyro_tau_s(180.0f);                    // 零偏时间常数 180s (参考 esp32-ins-gnss-fusion)

      // 2. 加速度计参数 (受共轴双桨震动影响大，需保守)
      // 适当增大测量噪声，防止震动导致状态发散
      nav_ekf.accel_std_mps2(0.25f);             // 设为 0.15~0.2 以抵抗震动

      // 3. 初始状态不确定度 (参考 esp32-ins-gnss-fusion)
      //    init_heading_err_std 之前设 0.1 (~5.7度) 过小, 告诉 EKF "初始航向很准",
      //    导致 P 矩阵初始协方差远小于实际不确定性, 辅助量测的修正被压制。
      //    无磁力计时航向完全不可观, 应设为 pi。
      //    同时补充缺失的 init_att/accel_bias/gyro_bias 初始不确定度。
      nav_ekf.init_pos_err_std_m(2.0f);            // 初始位置不确定度 2m
      nav_ekf.init_vel_err_std_mps(0.08f);         // 初始速度不确定度 0.08 m/s
      nav_ekf.init_att_err_std_rad(0.08f);         // 初始姿态 (roll/pitch) 不确定度 ~4.6度
      nav_ekf.init_heading_err_std_rad(3.14159f);  // 初始航向不确定度 pi (无磁力计, 航向完全未知)
      nav_ekf.init_accel_bias_std_mps2(0.196f);    // 初始加速度零偏不确定度 0.02g
      nav_ekf.init_gyro_bias_std_radps(0.00698f);  // 初始陀螺零偏不确定度 0.4度/s

      // 协方差稳定化分频: 每8帧做一次完整稳定化 (200Hz/8=25Hz), 节省 CPU。
      // 2026-06-23 H743 实测该配置把 Navigation 平均耗时压到约0.58ms；
      // 主机全精度/FAST A-B 60场景回归通过后才作为默认值。
      nav_ekf.propagation_stabilize_divider(8U);
      nav_ekf.static_aid_stabilize_divider(8U);

      // 4. GNSS 参数 (使用 UBX 报告的精度)
      // 根据是否有 GPS 调整初始位置/速度的不确定度
      if (gps_available)
      {
        // 如果 GPS 信号很好，UBX 可能会报 0.1m，这会让 EKF 过于信任 GPS 而忽略 IMU
        // 建议设置一个下限 (Floor)，防止过度修正导致“抽搐”
        float h_acc = fmaxf(gnss_epoch.horz_acc_m, 0.15f);    // 水平定位精度，限制最小 0.15m
        float v_acc = fmaxf(gnss_epoch.vert_acc_m, 0.25f);    // 垂直定位精度，限制最小 0.25m
        float spd_acc = fmaxf(gnss_epoch.spd_acc_mps, 0.05f); // 速度定位精度，限制最小 0.05m/s

        nav_ekf.gnss_pos_ne_std_m(h_acc);
        nav_ekf.gnss_pos_d_std_m(v_acc);
        nav_ekf.gnss_vel_ne_std_mps(spd_acc);
        nav_ekf.gnss_vel_d_std_mps(spd_acc);
      }
      // 无 GPS 时 GNSS R 不会被消费 (perform_measurement_update 为 false),
      // 不需要设伪值. 删除了原来的 else { nav_ekf.gnss_pos_ne_std_m(0.5); ... } 死代码.

      // 执行初始化
      nav_ekf.Initialize(imu_accel_mps2, imu_gyro_radps, mag_ut, init_vel_ned, init_lla);

      nav_system_initialized = true;

      // 设置系统原点 (Home Point)
      if (gps_available)
      {
        // Home 必须与本次 Initialize 使用同一个出队 epoch，不能回读可能已前移的 UBX 最新缓存。
        setNavigationOriginFromLlaRad(init_lla(0), init_lla(1), init_lla(2));
        nav_initialized_with_fake_origin = false;
        nav_has_real_gnss_anchor = true;
      }
      else
      {
        setNavigationOriginFromLlaRad(init_lla(0), init_lla(1), init_lla(2));
        nav_initialized_with_fake_origin = true;
        nav_has_real_gnss_anchor = false;
      }
      is_origin_lla_set = true;

      Serial8.println("[Nav] System Active (Dead Reckoning Yaw Mode)!");
    }
  }

  else
  {
    // --- 运行阶段 ---

    // [A] 陀螺仪输入准备
    // EKF底层会在初始化和传播中显式处理地球自转/导航系转动；静止时不能再把陀螺强行改成bias，
    // 否则会把真实物理角速度也抹掉，长时间无GNSS静置会重新引入慢姿态漂移。
    Eigen::Vector3f ekf_gyro_input = imu_gyro_radps;

    // [B] 时间更新 (预测) - 始终执行
    // 这一步负责 EKF 内部短时惯导外推；无 GNSS 时还会用 AHRS 姿态量测约束漂移。
    if (use_two_sample_imu)
    {
      nav_ekf.TimeUpdateTwoSample(delta_theta_1_rad,
                                  delta_theta_2_rad,
                                  delta_v_1_mps,
                                  delta_v_2_mps,
                                  nav_update_dt_s);
    }
    else
    {
      nav_ekf.TimeUpdate(imu_accel_mps2, ekf_gyro_input, nav_update_dt_s);
    }
#ifdef BFS_NAV_PROFILE
    {
      const uint32_t now_stage_us = micros();
      recordNavProfileStage(6U, now_stage_us - _nav_profile_stage_start);
      _nav_profile_stage_start = now_stage_us;
    }
#endif

    // [B2] 姿态量测更新 - 统一的静止辅助 + 条件性 AHRS 辅助
    //
    // 设计思路：
    //   有 GNSS 时：EKF 通过 GNSS 位置/速度更新 + 零偏估计，姿态估计已经足够好
    //              静止时只需要重力方向辅助 + ZUPT 约束，不需要 AHRS 辅助
    //   无 GNSS 时：纯惯导会漂移，需要额外的姿态约束
    //              静止时用重力方向辅助 + ZUPT，运动时用 AHRS 姿态量测
    const float accel_norm = imu_accel_mps2.norm();
    //
    const float accel_norm_error = fabsf(accel_norm - G_ACCEL_CONST);
    const bool accel_gravity_trusted = (accel_norm_error < 2.0f);
    // ★ 2026-08-13 升级修复：原 1.2 rad/s(≈69°/s) 门限近乎恒真——剧烈机动时
    //   Madgwick roll/pitch 本身不准，作为 EKF 姿态量测等于周期灌入低质量姿态。
    //   收紧到 0.5 rad/s(≈28°/s)，与双矢量航向 20°/s 门限一致量级，仅在平缓
    //   运动下用 AHRS 约束姿态漂移。
    const bool angular_rate_quiet = (imu_gyro_radps.norm() < 0.5f); // ≈28 deg/s

    // ========================================================================
    // 静止辅助：分频调度 + 置信度自适应（ins_static_aid_profile.h）
    // 参考 esp32-ins-gnss-fusion: ZUPT 每 2 帧(100Hz), Gravity 每 4 帧(50Hz),
    // 独立 divider + 错开 phase, 同帧优先 ZUPT。
    // ========================================================================
    if (is_static_confirmed) {
      static uint32_t static_aid_frame_counter = 0;
      ++static_aid_frame_counter;

      // 静止重力均值累加器: 累加静止期间的加速度, 用于 Gravity 辅助而非单帧值,
      // 参考 esp32-ins-gnss-fusion Icm45686StaticGravityAccumulator.
      static Eigen::Vector3f static_gravity_acc = Eigen::Vector3f::Zero();
      static uint32_t static_gravity_count = 0;
      static uint32_t static_gravity_last_frame = 0;
      // 如果刚进入静止 (confirmed_static_frames 回绕), 或非连续静止帧, 则重置累加器
      const uint32_t frame_seq = static_det.confirmed_static_frames;
      if (frame_seq < static_gravity_last_frame || frame_seq == 1U)
      {
        static_gravity_acc = Eigen::Vector3f::Zero();
        static_gravity_count = 0;
      }
      static_gravity_last_frame = frame_seq;
      static_gravity_acc += imu_accel_mps2;
      static_gravity_count++;

      const auto profile = Icm45686SelectStaticAidProfile(
          static_confidence, static_det.confirmed_static_frames,
          nav_ekf.covariance_is_healthy());

      // 三层自适应降频: 静止越久、收敛越充分, 辅助量测频率越低以节约 CPU。
      // 参考 esp32-ins-gnss-fusion: 长静止下 ZUPT 可降到 10-25Hz 无损零偏收敛。
      const bool relaxed  = (static_confidence > 0.85f && static_det.confirmed_static_frames > 1000U);
      const bool deep_rlx = (static_confidence > 0.90f && static_det.confirmed_static_frames > 4000U);
      uint32_t zupt_div, gravity_div, gyro_div;
      if (deep_rlx) {
        zupt_div = 16U; gravity_div = 32U; gyro_div = 32U;   // 12.5 / 6.25 / 6.25 Hz
      } else if (relaxed) {
        zupt_div = 8U;  gravity_div = 16U; gyro_div = 16U;   // 25 / 12.5 / 12.5 Hz
      } else {
        zupt_div = 4U;  gravity_div = 8U;  gyro_div = 8U;    // 50 / 25 / 25 Hz (过渡期)
      }

      const bool zupt_due = (static_aid_frame_counter % zupt_div) == 0U;
      const bool gravity_due = (static_aid_frame_counter % gravity_div) == 1U;
      const bool static_gyro_due = (static_aid_frame_counter % gyro_div) == 3U;

      const Icm45686StaticAidAction static_aid_action =
          Icm45686SelectStaticAidAction(
              zupt_due, gravity_due, static_gyro_due,
              profile.prefer_gravity_first);

      if (static_aid_action == Icm45686StaticAidAction::Zupt) {
        const float zupt_residual_mps = nav_ekf.ned_vel_mps().norm();
        const auto zupt_noise = Icm45686AdaptZuptNoiseForResidual(
            profile, zupt_residual_mps);
        if (zupt_noise.allow_fusion) {
          const bfs::MeasurementUpdateResult result =
              nav_ekf.MeasurementUpdateVelocityDetailed(
                  Eigen::Vector3f::Zero(),
                  zupt_noise.vel_ne_std_mps,
                  zupt_noise.vel_d_std_mps);
          aid_tracker.RecordAttempt(bfs::AidSource::StaticZupt, result);
#ifdef BFS_NAV_PROFILE
          recordNavProfileZupt(result.fused,
                               false /*skipped*/,
                               zupt_residual_mps,
                               zupt_noise.noise_scale);
#endif
        } else {
          aid_tracker.MarkAvailable(bfs::AidSource::StaticZupt, false);
#ifdef BFS_NAV_PROFILE
          recordNavProfileZupt(false /*fused*/,
                               true /*skipped*/,
                               zupt_residual_mps,
                               zupt_noise.noise_scale);
#endif
        }
      } else if (static_aid_action == Icm45686StaticAidAction::Gravity) {
        // 使用静止加速度均值 (降噪), 而非单帧 imu_accel_mps2
        const Eigen::Vector3f gravity_mean =
            (static_gravity_count > 0)
                ? (static_gravity_acc / static_cast<float>(static_gravity_count))
                : imu_accel_mps2;
        static_gravity_acc = Eigen::Vector3f::Zero();
        static_gravity_count = 0;
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateGravityDetailed(gravity_mean, profile.gravity_noise_rad);
        aid_tracker.RecordAttempt(bfs::AidSource::Gravity, result);
      } else if (static_aid_action == Icm45686StaticAidAction::StaticGyro) {
        // 静止陀螺量测: 直接将当前 gyro 读数作为观测, 噪声 ~0.01 rad/s (0.5°/s).
        // 这直接约束三轴陀螺零偏, 尤其对 Z 轴 (yaw) 零偏——ZUPT/Gravity 对其弱可观.
        // 参考 esp32-ins-gnss-fusion applyStaticGyroUpdate.
        static constexpr float kStaticGyroNoiseRadps = 0.0001f; // 0.006dps, 极高信任
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateStaticGyroDetailed(
                imu_gyro_radps, kStaticGyroNoiseRadps);
        aid_tracker.RecordAttempt(bfs::AidSource::StaticGyro, result);
        _sg_attempt++; if (result.fused) _sg_fused++;
      }
    }
    // 退出静止后重力累加器在下一次进入静止时由 static_gravity_acc=0,count=0 自然重置
    // ========================================================================
    // 无 GNSS + 运动时：AHRS 姿态量测（利用重力融合约束姿态）
    // ========================================================================
    else if (!gnss_instant_valid_for_nav && accel_gravity_trusted && angular_rate_quiet) {
      // 无 GNSS 时，运动中需要 AHRS 约束姿态漂移
      // Madgwick 的 roll/pitch 来自”陀螺积分 + 加速度重力修正”，已经融合了重力信息
      // 把它作为 EKF 姿态量测，可以在运动中持续约束姿态
      Eigen::Vector3f ahrs_ypr_rad;
      ahrs_ypr_rad << backup_ahrs_yaw, backup_ahrs_pitch, backup_ahrs_roll;

      // 动态噪声：加速度误差越大，对 AHRS 的信任越低
      const float roll_pitch_noise_rad = linearInterpolate(accel_norm_error,
                                                            0.3f, 2.0f,
                                                            0.04f, 0.30f);
      // yaw 噪声设为极大值 (≈π), 等效告诉 EKF "忽略 AHRS 的 yaw 量测, 只使用 roll/pitch"。
      // Madgwick 无磁力计时 yaw 纯陀螺积分会漂移, 强行喂入 EKF 会反向污染 EKF 零偏估计。
      // 参考 esp32-ins-gnss-fusion: yaw 残差恒设为 0 (nav.yaw_rad() 作为测量), 
      // 这里用极高测量噪声达到同样效果。
      const float yaw_noise_rad = 3.14159f;  // π rad, 等效忽略 yaw 量测
      const bfs::MeasurementUpdateResult result = nav_ekf.MeasurementUpdateAttitudeDetailed(ahrs_ypr_rad,
                                                                                              roll_pitch_noise_rad,
                                                                                              yaw_noise_rad);
      aid_tracker.RecordAttempt(bfs::AidSource::Attitude, result);
    }
    // 有 GNSS + 运动时：不需要 AHRS 辅助
    // EKF 通过 GNSS 位置/速度更新 + 零偏估计，姿态估计已经足够好
#ifdef BFS_NAV_PROFILE
    {
      const uint32_t now_stage_us = micros();
      recordNavProfileStage(7U, now_stage_us - _nav_profile_stage_start);
      _nav_profile_stage_start = now_stage_us;
    }
#endif

    // [C] 测量更新 (修正) - GNSS 位置/速度更新
    if (perform_measurement_update && gnss_instant_valid_for_nav)
    {
      // --- GNSS 可用时的位置/速度更新 ---

      // ============================================================
      // 1. 双矢量航向融合 (Dual-Vector Yaw Fusion)
      //    在进行位置/速度更新前，先尝试修正航向。
      //    这能让后续的速度更新使用更准确的旋转矩阵。
      // ============================================================
      handleDualVectorYawFusion();

      // ============================================================
      // 2. 标准 GNSS 位置/速度更新
      // ============================================================
      Eigen::Vector3f meas_vel_ned;
      Eigen::Vector3d meas_lla;

      // 使用本导航帧弹出的 epoch 快照，避免 UBX 解析缓存被后续串口字节覆盖。
      meas_lla << gnss_epoch.lat_rad, gnss_epoch.lon_rad, static_cast<double>(gnss_epoch.alt_wgs84_m);

      if (is_static_confirmed)
      {
        // 如果静止，强制告诉 EKF 速度为 0。
        // 这防止了 GNSS 的随机漂移速度去”拉动”航向。
        meas_vel_ned = Eigen::Vector3f::Zero();
      }
      else
      {
        // 运动中，使用真实 GNSS NED 速度。
        meas_vel_ned << gnss_epoch.north_vel_mps, gnss_epoch.east_vel_mps, gnss_epoch.down_vel_mps;
      }

      // 动态权重：接收机精度经 clamp→pDOP 缩放后直接作为 EKF R 矩阵噪声
      const auto meas_dw = Icm45686ComputeGnssDynamicWeights(
          static_cast<int8_t>(gnss_epoch.fix), gnss_epoch.num_sv,
          gnss_epoch.horz_acc_m, gnss_epoch.vert_acc_m, gnss_epoch.spd_acc_mps,
          gnss_epoch.pvt_pdop, kGnssDwCfg);
      nav_ekf.gnss_pos_ne_std_m(meas_dw.eff_pos_ne_std_m);
      nav_ekf.gnss_pos_d_std_m(meas_dw.eff_pos_d_std_m);
      nav_ekf.gnss_vel_ne_std_mps(meas_dw.eff_vel_ne_std_mps);
      nav_ekf.gnss_vel_d_std_mps(meas_dw.eff_vel_d_std_mps);

      if (InsGnssShouldReanchor(has_gnss_epoch,
                                gnss_full_quality_for_nav,
                                gnss_time_mapping_valid,
                                gnss_measurement_age_s,
                                GNSS_FIRST_REANCHOR_MAX_AGE_S,
                                nav_initialized_with_fake_origin,
                                nav_has_real_gnss_anchor))
      {
        // 无 GNSS 时使用假原点启动后，首次真实 GNSS 与假原点可能相差很远。
        // 此时不能走普通残差门控，应直接把位置/速度重锚定到真实 GNSS，姿态和 IMU 零偏保持连续。
        // 将最多 150 ms 的位置按 GNSS NED 速度外推到当前时刻，再执行硬重锚。
        reanchorNavigationToGnss(meas_vel_ned, meas_lla,
                                 gnss_measurement_age_s);
        nav_initialized_with_fake_origin = false;
        nav_has_real_gnss_anchor = true;
        aid_tracker.RecordReset(bfs::AidSource::Gnss);
        aid_tracker.RecordAttempt(bfs::AidSource::Gnss, true, 0.0f, 0.0f);
        Serial8.println("[Nav] Re-anchored EKF to first valid GNSS fix.");
      }
      else
      {
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateDetailed(meas_vel_ned, meas_lla,
                                              nav_update_dt_s,
                                              gnss_measurement_age_s);
        gnss_status.last_test_ratio = result.test_ratio;
        aid_tracker.RecordAttempt(bfs::AidSource::Gnss, result);
      }
    }
    else if (perform_measurement_update && gnss_downgrade_quality_for_nav)
    {
      // 边缘 GNSS 解只融合速度，位置噪声放大为近似关闭，避免弱位置观测拉偏原点。
      gnss_status.downgrade_count++;
      applyDowngradedGnssUpdate(gnss_epoch, nav_update_dt_s,
                                gnss_measurement_age_s);
    }

    if (gnss_full_quality_for_nav)
    {
      // 只有新鲜GNSS有效时，才按导航周期慢速校正后台 AHRS 航向偏置。
      // 输出保持期只用于避免EKF/AHRS输出抖动，不能继续用旧EKF航向牵引备用AHRS。
      // 这样 GNSS 丢失后 AHRS 接管时航向基准已对齐，避免10Hz量测分支过慢造成硬切换跳变。
      const float MAX_AHRS_YAW_CORRECTION_STEP_RAD = 0.0025f; // 200Hz 下约 28.6 deg/s
      float yaw_error = wrapAnglePi(nav_ekf.yaw_rad() - backup_ahrs_yaw);
      yaw_error = constrain(yaw_error,
                            -MAX_AHRS_YAW_CORRECTION_STEP_RAD,
                            0.0025f /* ~28.6 deg/s @200Hz */);
      ahrs_yaw_correction_rad = wrapAnglePi(ahrs_yaw_correction_rad + yaw_error);
    }

    // --- DETA100 在线时，用 DETA100 航向慢速校正 backup_ahrs 航向偏置 ---
    // 确保从 DETA100 直出回退到内置 EKF/AHRS 时航向基准对齐，实现丝滑切换。
    if (nav_data_source == NavDataSource::DETA100 && deta100_online)
    {
      const float MAX_AHRS_YAW_CORRECTION_STEP_RAD = 0.0025f;
      float yaw_error = wrapAnglePi(AHRS_Packet.Heading - backup_ahrs_yaw);
      yaw_error = constrain(yaw_error,
                            -MAX_AHRS_YAW_CORRECTION_STEP_RAD,
                            MAX_AHRS_YAW_CORRECTION_STEP_RAD);
      ahrs_yaw_correction_rad = wrapAnglePi(ahrs_yaw_correction_rad + yaw_error);
    }
  }

#ifdef BFS_EKF_BARO_ALTITUDE_UPDATE
  // ====================================================================
  // 气压高度标量量测更新 (对标 GNSS 位置 D 分量观测)
  // ====================================================================
  // 对标 GNSS MeasurementUpdateDetailed 中位置部分:
  //   创新项 = lla2ned(baro_lla, pred_lla).z()  (NED-D 方向)
  //   H = [0,0,1,0,...] (仅观测位置误差状态 D 分量)
  // 其余状态 (速度/姿态/零偏) 通过 P 矩阵耦合获得间接修正。
  //
  // DPS310 对外值是起飞点向上为正的相对高度。进入绝对 LLA EKF 前，
  // 使用同一个 Home 高度恢复为绝对观测，确保与 GNSS 使用一致的 WGS84 基准。
  //
  // 降采样: 25Hz (每 8 帧一次), DPS310 输出约 71Hz, 避免过密更新。
  {
    static int baro_ekf_update_counter = 0;
    if (++baro_ekf_update_counter >= 8 &&
        nav_system_initialized && is_origin_lla_set)
    {
      baro_ekf_update_counter = 0;
      double baro_absolute_altitude_m = 0.0;
      if (InsComputeAbsoluteBaroAltitude(
              origin_alt_m, baro_altitude, &baro_absolute_altitude_m))
      {
        baro_ekf_innovation_m = static_cast<float>(
            baro_absolute_altitude_m - nav_ekf.alt_m());
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateBaroAltitudeDetailed(
                static_cast<float>(baro_absolute_altitude_m), BARO_NOISE_STD);
        aid_tracker.RecordAttempt(bfs::AidSource::Baro, result);
        baro_ekf_test_ratio = result.test_ratio;
        baro_ekf_fused = result.fused;
      }
      else
      {
        baro_ekf_fused = false;
      }
    }
  }
#endif

  if (perform_measurement_update && gnss_full_quality_for_nav)
  {
    // UBX原始相对位置必须在EKF初始化/假原点重锚定之后更新，避免首帧真实GNSS用旧原点发布跳变。
    updateUbxRelativePosition();
  }

  // ====================================================================
  // 5. 将 EKF 结果注入全局状态 (Bridge to Global State)
  // ====================================================================
  // 核心策略：DETA100 在线时直接透传其数据给飞控，EKF 输出桥跳过写入；
  //           DETA100 离线时 EKF 始终作为唯一姿态输出源（参考 esp32-ins-gnss-fusion 架构），
  //           无 GNSS 时靠静止辅助(ZUPT+Gravity)和 AHRS 姿态量测持续闭环修正，
  //           不再回退到 backup_ahrs(Madgwick)。EKF 未初始化的启动阶段仍用 backup_ahrs 填充。

  const bool use_deta100_output = (nav_data_source == NavDataSource::DETA100) && deta100_online;

  // --- A. 注入 EKF 解算结果 (仅当 EKF 初始化后 且 非 DETA100 直出模式) ---
  if (nav_system_initialized)
  {
    if (use_deta100_output)
    {
      // --- DETA100 直出模式：EKF 结果不写入全局状态，仅同步 icm_ 辅助变量 ---
      // DETA100 的 DataUnpacking() 已直接写入 AHRS_Packet / INS_GNSS_Packet /
      // Geodetic_Pos_Packet，飞控直接消费这些全局变量，无需 EKF 覆盖。
      // EKF 仍在后台预测运行，作为 DETA100 离线时的无缝备份。
      icm_Roll = AHRS_Packet.Roll;
      icm_Pitch = AHRS_Packet.Pitch;
      icm_Yaw = AHRS_Packet.Heading;
      use_ekf_attitude_output = false;

      // 同步相对位置：DETA100 输出绝对 LLA (弧度, WGS84 椭球高, 见 deta100_types.h)，
      // 需要从 Geodetic_Pos_Packet 计算相对于导航原点的 NED 位移。
      // DETA100 模式下内置 UBX 无 fix，EKF 只能用假原点初始化，
      // 因此当 DETA100 首次获得 3D 定位时，用其 LLA 重设导航原点，
      // 使 relative_* 基准为真实起飞点而非假原点。
      const bool deta100_has_absolute_position =
          Status_Packet.filter_status.gnss_fix_status >= DETA100_GPS_FIX_TYPE_3D_FIX &&
          std::isfinite(Geodetic_Pos_Packet.latitude) &&
          std::isfinite(Geodetic_Pos_Packet.longitude) &&
          std::isfinite(Geodetic_Pos_Packet.height) &&
          Geodetic_Pos_Packet.latitude != 0.0 &&
          Geodetic_Pos_Packet.longitude != 0.0;
      if (nav_initialized_with_fake_origin && !nav_has_real_gnss_anchor &&
          deta100_has_absolute_position)
      {
        Eigen::Vector3d deta100_lla(
            Geodetic_Pos_Packet.latitude,
            Geodetic_Pos_Packet.longitude,
            Geodetic_Pos_Packet.height);
        if (g_is_unlocked && is_origin_lla_set)
        {
          const Eigen::Vector3d old_origin_lla(
              origin_lat_rad, origin_lon_rad, origin_alt_m);
          const Eigen::Vector3d takeoff_to_current_ned = bfs::lla2ned(
              nav_ekf.lla_rad_m(), old_origin_lla, bfs::AngPosUnit::RAD);
          Eigen::Vector3d takeoff_lla = bfs::ned2lla(
              -takeoff_to_current_ned, deta100_lla, bfs::AngPosUnit::RAD);
          takeoff_lla(2) = InsTakeoffOriginAltitudeFromCurrent(
              deta100_lla(2), takeoff_to_current_ned(2));
          setNavigationOriginFromLlaRad(
              takeoff_lla(0), takeoff_lla(1), takeoff_lla(2));
        }
        else
        {
          setNavigationOriginFromLlaRad(
              deta100_lla(0), deta100_lla(1), deta100_lla(2));
        }
        is_origin_lla_set = true;
        nav_initialized_with_fake_origin = false;
        nav_has_real_gnss_anchor = true;
        Serial8.println("[Nav] DETA100 3D fix: re-anchored origin to DETA100 LLA.");
      }

      if (is_origin_lla_set && deta100_has_absolute_position)
      {
        Eigen::Vector3d current_lla_rad;
        current_lla_rad << Geodetic_Pos_Packet.latitude, Geodetic_Pos_Packet.longitude, Geodetic_Pos_Packet.height;
        Eigen::Vector3d origin_lla_rad;
        origin_lla_rad << origin_lat_rad, origin_lon_rad, origin_alt_m;
        Eigen::Vector3d relative_ned = bfs::lla2ned(current_lla_rad, origin_lla_rad, bfs::AngPosUnit::RAD);
        relative_north = relative_ned(0);
        relative_east = relative_ned(1);
        relative_down = relative_ned(2);
      }
    }
    else
    {
      // --- 内置 EKF 输出模式：EKF 始终作为唯一姿态输出源 ---
      // 参考 esp32-ins-gnss-fusion 架构：EKF 始终输出姿态，不再按 GNSS 状态回退到 backup_ahrs。
      // 无 GNSS 时 EKF 靠静止辅助(ZUPT+Gravity)和无 GNSS 运动时 AHRS 姿态量测持续闭环修正，
      // 姿态连续性和响应优于 Madgwick 回退方案。backup_ahrs(Madgwick) 仅作后台对照，
      // 不再直接写入 AHRS_Packet（EKF 未初始化的启动阶段除外）。
      use_ekf_attitude_output = true;
      {
        Eigen::Quaternionf q_ekf = nav_ekf.quat();

        AHRS_Packet.Qw = q_ekf.w();
        AHRS_Packet.Qx = q_ekf.x();
        AHRS_Packet.Qy = q_ekf.y();
        AHRS_Packet.Qz = q_ekf.z();

        AHRS_Packet.Roll = nav_ekf.roll_rad();
        AHRS_Packet.Pitch = nav_ekf.pitch_rad();
        AHRS_Packet.Heading = wrapAngleTwoPi(nav_ekf.yaw_rad()); // 注意：这是真北航向

        icm_Roll = AHRS_Packet.Roll;
        icm_Pitch = AHRS_Packet.Pitch;
        icm_Yaw = AHRS_Packet.Heading;

        // 导出 WGS-84 当地重力（Somigliana, 随经纬高）供控制律消费；
        // 仅在 EKF 初始化分支刷新，未初始化时保持回退值 G_ACCEL_CONST。
        //
        // 防护（防突变/异常定位数据/噪声）：
        //   1. 合理性范围检查：WGS-84 正常重力全局范围 9.78~9.83 m/s²，
        //      加高度/异常余量取 [9.0, 10.5]，范围外（GNSS 野值/重锚定
        //      导致的 LLA 跳变）整拍丢弃，保持上一有效值；
        //   2. 一阶低通：重力随位置变化极慢（纬度敏感度 ~5e-3 m/s²/度），
        //      tau≈1s 低通滤除 LLA 噪声与瞬态跳变，初始化后平滑收敛；
        //   3. 与回退值偏差限幅 ±0.5 m/s²：防御性兜底，杜绝任何路径
        //      把异常值灌入控制律。
        {
          static const float kGravityAlpha = 0.005f;   // 一阶低通系数（tau≈1s @200Hz）
          static const float kGravityMin = 9.0f;       // 合理性范围下界
          static const float kGravityMax = 10.5f;      // 合理性范围上界
          static const float kGravityDevMax = 0.5f;    // 与回退值最大偏差
          static float filtered_gravity = G_ACCEL_CONST;
          const float raw_gravity = static_cast<float>(nav_ekf.gravity_mps2());
          if (raw_gravity >= kGravityMin && raw_gravity <= kGravityMax)
          {
            filtered_gravity += kGravityAlpha * (raw_gravity - filtered_gravity);
            filtered_gravity = constrain(filtered_gravity,
                                         G_ACCEL_CONST - kGravityDevMax,
                                         G_ACCEL_CONST + kGravityDevMax);
            ekf_gravity_mps2 = filtered_gravity;
          }
          // 范围外（异常）丢弃本拍，ekf_gravity_mps2 保持上一有效值
        }
      }

      // --- 2. 注入位置与速度 (INS_GNSS_Packet) ---
      INS_GNSS_Packet.velocity_north = nav_ekf.ned_vel_mps()(0);
      INS_GNSS_Packet.velocity_east = nav_ekf.ned_vel_mps()(1);
      INS_GNSS_Packet.velocity_down = nav_ekf.ned_vel_mps()(2);
      ekf_accel_bias_z_mps2 = nav_ekf.accel_bias_mps2()(2);  // 机体系FRD Z轴 (下) 零偏, m/s^2

      // 注入绝对位置 (用于原点设置逻辑)
      Geodetic_Pos_Packet.latitude = nav_ekf.lat_rad();
      Geodetic_Pos_Packet.longitude = nav_ekf.lon_rad();
      Geodetic_Pos_Packet.height = nav_ekf.alt_m();

      // --- 3. 注入融合相对位置 (每4帧计算一次, 桌面静止时位置几乎不变)
      if (is_origin_lla_set)
      {
        // 垂直控制需要 200Hz 更新；局部飞行范围内高度差与 NED Down 严格反号。
        relative_down = static_cast<float>(origin_alt_m - nav_ekf.alt_m());
        static uint32_t lla2ned_skip = 0;
        if (++lla2ned_skip >= 4U) {
          lla2ned_skip = 0;
          Eigen::Vector3d current_lla_rad = nav_ekf.lla_rad_m();
          Eigen::Vector3d origin_lla_rad;
          origin_lla_rad << origin_lat_rad, origin_lon_rad, origin_alt_m;
          Eigen::Vector3d relative_ned = bfs::lla2ned(current_lla_rad, origin_lla_rad, bfs::AngPosUnit::RAD);
          relative_north = relative_ned(0);
          relative_east = relative_ned(1);
          relative_down = relative_ned(2);
        }
      }
    }
  }
  else
  {
    // --- EKF 未初始化时，用后台 AHRS 填充 AHRS_Packet，避免飞控读到空值 ---
    // ★ 2026-08-13 升级修复：DETA100 直出模式下 DETA100 的 DataUnpacking() 已写入
    //   AHRS_Packet，此分支原不检查 use_deta100_output，会逐帧用 Madgwick 值覆盖
    //   DETA100 输出，使直出通道在 EKF 尚未初始化（静止检测未通过）期间失效。
    //   加 !use_deta100_output 隔离，仅内置 EKF 模式用备份填充。
    if (!use_deta100_output)
    {
      AHRS_Packet.Qw = backup_ahrs_Qw;
      AHRS_Packet.Qx = backup_ahrs_Qx;
      AHRS_Packet.Qy = backup_ahrs_Qy;
      AHRS_Packet.Qz = backup_ahrs_Qz;
      AHRS_Packet.Roll = backup_ahrs_roll;
      AHRS_Packet.Pitch = backup_ahrs_pitch;
      AHRS_Packet.Heading = wrapAngleTwoPi(backup_ahrs_yaw);
    }
  }

  if (nav_system_initialized && is_origin_lla_set)
  {
    INS_GNSS_Packet.location_north = relative_north;
    INS_GNSS_Packet.location_east = relative_east;
    INS_GNSS_Packet.location_down = relative_down;
    estimated_height = InsRelativeHeightUpFromNedDown(relative_down);
    estimated_velocity = InsVerticalVelocityUpFromNedDown(
        INS_GNSS_Packet.velocity_down);
  }

  // --- 注入 GNSS 原始状态 (覆盖 DETA100 的 Geodetic_Pos & Status) ---
  // 无论 EKF 是否初始化，只要有 GPS 数据就应该更新这些包，以便地面站看到卫星状态
  // 精度因子来自GNSS接收机缓存；GNSS超时后必须清零，避免地面站显示“无fix但仍有旧精度”。
  // ★ 2026-08-13 升级修复：hAcc/vAcc 与下方 fix 类型一致加 gnss_data_fresh_for_nav
  //   门控——原实现 has_obs 超时后从不清零，GNSS 断电后地面站仍显示最后一次通过
  //   质量帧的旧精度（与注释"必须清零"矛盾）。
  if (gnss_status.has_obs && gnss_data_fresh_for_nav)
  {
    Geodetic_Pos_Packet.hAcc = gnss_status.last_obs_h_acc_m; // 最近一帧 GNSS 水平精度
    Geodetic_Pos_Packet.vAcc = gnss_status.last_obs_v_acc_m; // 最近一帧 GNSS 垂直精度
  }
  else
  {
    Geodetic_Pos_Packet.hAcc = 0.0f;
    Geodetic_Pos_Packet.vAcc = 0.0f;
  }

  // [A] 映射 GPS 定位状态 (Fix Type)
  // BFS UBX Fix: 0=None, 1=2D, 2=3D, 3=DGNSS, 4=RTK-Float, 5=RTK-Fixed
  // DETA100 Enum: 1=NoFix, 2=2D, 3=3D, 4=DGPS, 5=RTK_Float, 6=RTK_Fixed
  // 注意：DETA100 的枚举值比 BFS 的通常大 1 (因为 0 是 No Module)

  bfs::Ubx::Fix ubx_fix = (gnss_data_fresh_for_nav && gnss_status.has_obs)
                             ? static_cast<bfs::Ubx::Fix>(static_cast<int>(gnss_status.last_obs_fix_type))
                             : bfs::Ubx::FIX_NONE;
  GPSFixType mapped_fix_type = mapUbxFixToDeta100FixType(ubx_fix);
  Status_Packet.filter_status.gnss_fix_status = mapped_fix_type;
#ifdef BFS_NAV_PROFILE
  {
    const uint32_t now_stage_us = micros();
    recordNavProfileStage(8U, now_stage_us - _nav_profile_stage_start);
    recordNavProfileStage(9U, now_stage_us - _nav_profile_total_start);
    printNavProfileIfDue();
  }
#endif

  // EKF 总耗时统计
  _ekf_time_sum += micros() - _ekf_t0;
  _ekf_time_cnt++;
  if (millis() - _ekf_last >= 60000) { // 60秒一次, 基本静默
    _ekf_last = millis();
    Serial8.print("[EKF] avg=");
    // ★ 2026-08-13 升级修复：原整型除法把 60s 平均耗时截断为整数微秒（诊断精度损失）。
    Serial8.print(static_cast<float>(_ekf_time_sum) / static_cast<float>(_ekf_time_cnt));
    Serial8.print("us cnt=");
    Serial8.print(_ekf_time_cnt);
    Serial8.print(" SG=");
    Serial8.print(_sg_fused);
    Serial8.print("/");
    Serial8.print(_sg_attempt);
    const Eigen::Vector3f gb = nav_ekf.gyro_bias_radps();
    Serial8.print(" gbias=[");
    Serial8.print(gb(0)*57.2958f, 4); Serial8.print(",");
    Serial8.print(gb(1)*57.2958f, 4); Serial8.print(",");
    Serial8.print(gb(2)*57.2958f, 4);
    Serial8.print("]dps");
    Serial8.println();
    _ekf_time_sum = _ekf_time_cnt = 0;
    _sg_attempt = _sg_fused = 0;
  }
}

void updateEstimatedVerticalVelocity()
{
  // --- 1. 静态状态变量 ---
  // 这些变量必须是静态的，以便在函数调用之间保持它们的值，用于差分计算。
  static float last_flow_height_for_vel = 0.0f;
  static float last_baro_height_for_vel = 0.0f;
  static unsigned long last_vel_calc_time_micros = 0;
  // 状态标志，用于处理首次计算，避免使用无效的旧数据。
  static bool is_first_run_flow_vel = true;
  static bool is_first_run_baro_vel = true;

  // --- 2. 数据源选择与速度计算 ---
  // 条件1: 检查是否有可用GNSS导航输出。
  // 注意：Geodetic_Pos_Packet 会被EKF假原点/惯导结果写回，不能再用经纬度非零判断GNSS有效。
  if (isGnssOutputValidForNav())
  {
    // --- 优先级1: 使用最高质量的INS/GNSS融合速度 ---
    // 这是最理想的情况，直接获取高精度速度，无需估算。
    // INS_GNSS_Packet.velocity_down 是NED坐标系，向下为正，所以需要取反得到向上为正的速度。
    estimated_velocity = -INS_GNSS_Packet.velocity_down;

    // 当GNSS有效时，重置差分估算的首次运行标志，以便在GNSS丢失时能平滑启动估算。
    is_first_run_flow_vel = true;
    is_first_run_baro_vel = true;

    // 同时，用高质量的GNSS速度来“预热”或重置估算速度滤波器，确保切换时无跳变。
    estimatedVerticalVelocityFilter.initialize(estimated_velocity);
  }
  else
  {
    // --- 无GNSS数据，需要通过高度差分来估算垂直速度 ---

    // 获取当前时间戳和时间差 (dt)
    unsigned long current_time_micros = micros();
    float dt = (last_vel_calc_time_micros == 0) ? 0.005f : (current_time_micros - last_vel_calc_time_micros) / 1000000.0f;
    last_vel_calc_time_micros = current_time_micros;

    // 防御性编程：保证dt有效，防止除零
    if (dt < 0.001f)
      dt = 0.001f;

    float raw_estimated_velocity = 0.0f;

    // 条件2: 检查激光测距是否有效且在合理范围内
    if (flow_data.is_range_valid && flow_data.distance_m > 0.05f)
    {
      // --- 优先级2: 使用激光高度进行差分 ---
      if (is_first_run_flow_vel)
      {
        // 首次运行时，速度为0，只记录当前高度作为下一次计算的基准
        last_flow_height_for_vel = flow_data.distance_m;
        is_first_run_flow_vel = false;
        raw_estimated_velocity = 0.0f;
      }
      else
      {
        // 计算原始速度 = (当前高度 - 上次高度) / 时间差
        raw_estimated_velocity = (flow_data.distance_m - last_flow_height_for_vel) / dt;
        // 更新上次高度，为下一次计算做准备
        last_flow_height_for_vel = flow_data.distance_m;
      }
      // 当切换到气压计时，需要重置气压计的首次运行标志
      is_first_run_baro_vel = true;
    }
    else
    {
      // --- 优先级3: 光流无效，使用气压计高度进行差分 ---
      if (is_first_run_baro_vel)
      {
        // 首次运行，速度为0，记录当前高度
        last_baro_height_for_vel = baro_altitude;
        is_first_run_baro_vel = false;
        raw_estimated_velocity = 0.0f;
      }
      else
      {
        // 计算原始速度
        raw_estimated_velocity = (baro_altitude - last_baro_height_for_vel) / dt;
        // 更新上次高度
        last_baro_height_for_vel = baro_altitude;
      }
      // 当切换到光流时，需要重置光流的首次运行标志
      is_first_run_flow_vel = true;
    }

    // *** 关键步骤：对估算出的原始速度进行强力低通滤波 ***
    // 无论来源是光流还是气压计，都需要滤波来平滑噪声。
    estimated_velocity = estimatedVerticalVelocityFilter.filter(raw_estimated_velocity);
  }
}

void handleVerticalEstimation()
{
  // ========================================================================
  // 步骤 1: 计算时间差 (dt)
  // ========================================================================
  // 程序说明：
  // 使用静态变量来记录上次调用的时间，以计算精确的时间间隔 dt。
  // dt 是卡尔曼滤波运动学模型中的关键参数。
  static unsigned long last_time_micros = 0;
  unsigned long current_time_micros = micros();
  // 计算时间差（秒）。首次运行时，使用一个默认的较小值。
  float dt = (last_time_micros == 0) ? 0.01f : (current_time_micros - last_time_micros) / 1000000.0f;
  last_time_micros = current_time_micros; // 更新时间戳为下一次计算做准备
  // 安全检查：防止dt为零或负数，这会导致计算错误。
  if (dt <= 0)
    dt = 0.001f;

  // ========================================================================
  // 步骤 2: 获取并转换IMU加速度数据
  // ========================================================================
  // 程序说明：
  // 我们的卡尔曼滤波器是在世界坐标系（NED）下工作的，但IMU测量的是机体坐标系（FRD）下的加速度。
  // 因此，必须先将机体系的加速度向量旋转到世界系。

  // 1. 从传感器数据包中获取机体坐标系下的加速度向量。
  Vector3 accel_body = {IMU_Packet.accelerometer_x, IMU_Packet.accelerometer_y, IMU_Packet.accelerometer_z};

  // 2. 获取当前的姿态四元数，它表示从机体系(B)到导航系(N)的旋转。
  Quaternion q_N_from_B = {AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz};

  // 3. 使用库中的 rotateVector 函数执行坐标变换。
  //    将机体系下的加速度向量 accel_body，通过 q_N_from_B 旋转，得到其在导航系下的表示 accel_ned。
  Vector3 accel_ned = rotateVector(q_N_from_B, accel_body);

  // 4. 从世界系下的总加速度中减去重力，得到纯粹的线性加速度。
  //    在NED坐标系中，重力沿着+Z轴（地向），大小为 G_ACCEL_CONST，加表测量值为 -G_ACCEL_CONST。。
  float linear_accel_z_ned = accel_ned.z + G_ACCEL_CONST;

  // 5. 适配卡尔曼滤波器的坐标系。
  //    我们的KF模型假设Z轴向上为正，而NED系的Z轴向下为正，因此需要取反。
  float linear_accel_z_up_is_positive = -linear_accel_z_ned;

  // ========================================================================
  // 步骤 3: 执行卡尔曼滤波的预测步骤
  // ========================================================================
  // 程序说明：
  // 调用我们之前创建的 vertical_estimator 对象的 predict 方法，
  // 将处理好的垂直线性加速度和时间差 dt 作为输入。
  vertical_estimator_2state.predict(linear_accel_z_up_is_positive, dt);

  // ========================================================================
  // 步骤 4: 根据可用的传感器数据，执行更新步骤
  // ========================================================================
  // 程序说明：
  // 这一步是数据融合的核心。我们检查哪个高度传感器的数据是可用的，并用它来修正预测结果。
  // 传感器的优先级和噪声参数在这里起关键作用。

  // 1. 优先使用精度更高的激光高度计。
  //    检查激光测距数据是否有效，并且高度在合理范围内（例如大于5cm，避免地面噪声）。
  // 为了降低计算负担，给激光高度计也加上降采样更新（例如，每2个预测周期更新一次）。
  static int laser_update_counter = 0;
  // if (flow_data.is_range_valid && flow_data.distance_m > 0.05f)
  if (0) // 暂时禁用激光数据
  {
    if (++laser_update_counter >= 1)
    {                           // 假设主循环200Hz，则激光高度计更新频率为 200/2 = 100Hz
      laser_update_counter = 0; // 重置计数器
      // 使用激光高度计的测量值和其对应的噪声标准差来更新滤波器。
      vertical_estimator_2state.update(flow_data.distance_m, LASER_NOISE_STD);
    }
  }
  // 2. 如果激光高度计不可用，则使用气压计。
  else
  {
    // 气压计读取任务约 71Hz，但垂向 KF 仍按 50Hz 更新，避免把气压噪声过密注入估计器。
    static int baro_update_counter = 0;
    if (++baro_update_counter >= 4)
    {                          // 假设主循环200Hz，则气压计更新频率为 200/4 = 50Hz
      baro_update_counter = 0; // 重置计数器
      // 使用气压计的高度测量值和其较大的噪声标准差来更新滤波器。
      vertical_estimator_2state.update(baro_altitude, BARO_NOISE_STD);
    }
  }

  // ========================================================================
  // 步骤 5: 更新独立输出变量（与 EKF 并行运行，不覆盖 estimated_height/velocity）
  // ========================================================================
  // 输出到 vfk_* 专用变量，供地面站遥测对比 EKF 与独立 VKF 的一致性。
  // 控制律仍消费 EKF 输出的 estimated_height / estimated_velocity。
  vfk_height = vertical_estimator_2state.getHeight();
  vfk_velocity = vertical_estimator_2state.getVelocity();
}

void handleHorizontalEstimation()
{
  // --- 1. 计算时间差 dt ---
  static unsigned long last_hkf_time_micros = 0;
  unsigned long current_time_micros = micros();

  // 首次运行保护
  if (last_hkf_time_micros == 0)
  {
    last_hkf_time_micros = current_time_micros;
    return;
  }

  float dt = (float)(current_time_micros - last_hkf_time_micros) / 1000000.0f;
  last_hkf_time_micros = current_time_micros;

  // 异常时间间隔保护
  if (dt <= 0.0001f || dt > 0.1f)
    return;

  // --- 2. 准备预测输入: 水平加速度 (NED系) ---

  // 2.1 获取机体坐标系加速度 (m/s^2)
  // 注意: 这里使用去除了重力成分还是原始值取决于 rotateVector 的定义。
  // 通常 IMU_Packet 输出的是包含重力的比力 (Specific Force)。
  // 当我们将比力旋转到 NED 系后，Z轴分量约为 -9.8 (或+9.8)，X/Y 轴分量即为运动加速度。
  Vector3 accel_body = {IMU_Packet.accelerometer_x, IMU_Packet.accelerometer_y, IMU_Packet.accelerometer_z};

  // 2.2 获取当前姿态四元数 (Body -> NED)
  Quaternion q_body_to_ned = {AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz};

  // 2.3 旋转向量到 NED 系
  Vector3 accel_ned = rotateVector(q_body_to_ned, accel_body);

  // 2.4 提取水平分量
  // 在 NED 系下，水平加速度即为 X (North) 和 Y (East) 分量。
  // 不需要显式减去重力，因为重力矢量在 NED 系下垂直于水平面 (仅在 Z 轴)。
  float accel_north_input = accel_ned.x;
  float accel_east_input = accel_ned.y;

  // --- 3. 执行预测 (Predict) ---
  kf_north.predict(accel_north_input, dt);
  kf_east.predict(accel_east_input, dt);

  // --- 4. 准备更新输入: 光流速度 (NED系) ---

  // 检查光流数据有效性 (使用去旋转后的纯平移速度 flow_data.velocity_x/y_mps)
  // 并且高度不能太低 (例如 > 5cm)，否则光流数据不可靠
  if (flow_data.is_flow_valid && estimated_height > 0.05f)
  {
    // 立即清除标志位，防止同一个数据在下个循环被重复使用
    flow_data.is_flow_valid = false;

    // 4.1 获取机体坐标系下的光流速度
    float flow_vel_body_x = flow_data.velocity_x_mps;
    float flow_vel_body_y = flow_data.velocity_y_mps;

    // 4.2 旋转到 NED 坐标系
    float vel_meas_north, vel_meas_east;
    float yaw_rad = AHRS_Packet.Heading;
    bodyToNed(flow_vel_body_x, flow_vel_body_y, yaw_rad, vel_meas_north, vel_meas_east);

    // 4.3 确定观测噪声 (R)
    // 使用全局基础噪声变量，并结合运行状态动态调整
    float flow_noise_std = kf_h_r_vel_base;

    // 动态增加噪声：速度越快，光流通常越抖
    float total_vel = sqrtf(flow_data.velocity_x_mps * flow_data.velocity_x_mps +
                            flow_data.velocity_y_mps * flow_data.velocity_y_mps);
    flow_noise_std += 0.1f * total_vel;

    // --- 5. 执行更新 (Update) ---
    kf_north.update(vel_meas_north, flow_noise_std);
    kf_east.update(vel_meas_east, flow_noise_std);
  }

  // --- 6. 获取结果并注入系统 ---
  fused_north_pos = kf_north.getPosition();
  fused_north_vel = kf_north.getVelocity();
  fused_east_pos = kf_east.getPosition();
  fused_east_vel = kf_east.getVelocity();

  // EKF 输出桥已写入 relative_north/east (lla→NED 转换) 和 INS_GNSS_Packet.velocity_*。
  // 不再用 kf_north/kf_east 覆盖——水平 KF 纯加速度积分无零速更新, 会持续漂移。
  // kf_north/kf_east 保留为光流融合后备 (AUTO_POSITION 可切换使用 fused_north_pos/vel),
  // 但不再无条件替换 EKF 的 relative 和 INS 速度。
  // if (!isGnssOutputValidForNav())
  // {
  //   relative_north = fused_north_pos;
  //   relative_east = fused_east_pos;
  //   INS_GNSS_Packet.velocity_north = fused_north_vel;
  //   INS_GNSS_Packet.velocity_east = fused_east_vel;
  // }
}
