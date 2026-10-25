/**
 * @file    navigation_task.cpp
 * @module  固件/导航（EKF 组合导航任务实现）
 * @brief   实现 200 Hz 导航状态机：IMU 双子样预测、静止/ZUPT/重力/AHRS 辅助、
 *          GNSS 位置速度更新、双矢量航向、气压高度、光流速度与输出桥
 * @details 输入：IMU 平均比力与角速度、双子样增量、UBX epoch 队列、DPS310 相对高度、
 *          光流速度/测距、MagSensor FRD 磁样本（无/无效时回退伪磁初始化）、DETA100（可选直出）；
 *          处理：取数（关中断快照）→ 静止检测 → EKF 初始化/预测 →
 *          静止辅助或 AHRS 姿态 → GNSS 全质量/降级更新或首次重锚 →
 *          光流速度 → 气压高度 → 输出桥（EKF→全局状态）；
 *          输出：`AHRS_Packet`/`INS_GNSS_Packet`/`Geodetic_Pos_Packet`/
 *          `relative_*`/`estimated_height`/`estimated_velocity` 与 `gnc_tel` 诊断。
 *          不变量：GNSS 只用本帧出队 epoch 快照；静止确认由外部速度可推翻；
 *          EKF 是唯一姿态输出源（DETA100 直出与未初始化两情形除外）。
 * @theory  姿态四元数右乘更新 q⊗Exp(ωΔt) @ref REF-120 §9.2 (9.2.14) p.268；
 *          比力方程 V̇ⁿ=Cⁿ_b f^b−(2ω_ie+ω_en)×Vⁿ+gⁿ @ref REF-120 §7.4 (7.4.6) p.189；
 *          双子样速度增量（旋转/划摇补偿）@ref REF-120 §9.6 (9.6.9)(9.6.17)(9.6.19) p.298–299；
 *          位置速率 L̇=V_N/(R_m+h) 等 @ref REF-120 §7.2/§8.1；
 *          KF 五方程与 Joseph 形式 @ref REF-121 §2.2.1 p.33–34、§2.2.4 p.42–43；
 *          NIS 门控/系统级 FDI @ref REF-121 §7.3.1 p.255–259；
 *          组合导航阻尼须用外来信息 @ref REF-121 §8.2.3 p.291；
 *          初始对准与航向可观性 @ref REF-120 §9.9 p.322–338；
 *          3-2-1 欧拉角 theta=-asin(R13) @ref MOD-002 §3
 * @frame   NED（北东地）/ FRD（前右下）；LLA 为 WGS84 弧度；机体系加速度为比力
 * @units   rad、rad/s、m、m/s、m/s²、s/ms/µs（时间）、°C、uT
 * @redline 四元数/双子样姿态更新、theta=-asin(R13)、ZUPT 与静止退出门限、
 *          GNSS R 地板、光流速度/测距双新鲜度通道、static gyro 的 throttle≥3% 跳过
 *          （逐条见行为保持红线清单与 TandemVec_FCS/AGENTS.md 踩坑记录）
 * @note    本文件是导航算法的主要“事实源”，但 EKF 内部方程在
 *          `lib/navigation-main/src/ekf_15_state.h`（fw-ekf 区域）。
 *          修改门限/参数前先读 AGENTS.md 的 EKF 踩坑记录与核查报告。
 * @see     `navigation_task.h`、`FW-NAV-代码理论核查.md`、`docs/04-数学建模/代码理论核查/`
 */

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
#include "ins_gnss_delay_auto.h"
#include "ins_vibration_adaptive.h"
#include "ins_inertial_reference.h"
#include "ins_bias_jump_monitor.h"
#include "ins_bias_temp_observer.h"
#include "ins_bias_temp_table.h"
#include "ins_altitude_reference.h"
#include "ins_static_detector.h"
#include "ins_static_aid_profile.h"
#include "mag_reference.h"
#include "mag_measurement_gate.h"
#include "HeadingAidingGate.h"
#include "ins_imu_health.h"
#include "ins_gnss_consistency.h"

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

/**
 * @brief 判断一个 UBX epoch 是否通过全部质量门限，并只在此刻刷新 GNSS 快照
 * @details 输入：出队 epoch；处理：pvt/eoe iTOW 一致性 → 重复 iTOW 判重 →
 *          动态权重最小门限 → 写 `gnss_status.last_obs_*`；输出：是否可用于
 *          全质量融合。不变量：快照只反映通过全部检查的 epoch，避免地面站
 *          显示 fix 而 EKF 实际拒收（2026-08-10 审查修复）。
 * @param[in] epoch 本帧出队的 UBX 历元（iTOW、fix、精度、LLA、NED 速度）
 * @return true 通过全部门限；false 任一检查失败（同时登记对应计数器）
 * @note 最小门限是 fix≥3D、卫星数≥3、h_acc>0，精度本身不丢弃、只作为 R 权重。
 * @see `Icm45686ComputeGnssDynamicWeights()`、`applyDowngradedGnssUpdate()`
 */
// EKF 初始化的 GNSS 质量门：初始位置一旦被单帧野值污染，后续好帧会被
// 反向拒绝。要求接收机自报精度足够好；差分解/RTK 更严格，普通 3D 解
// 的 h_acc 通常 1–3 m。不合格时先用伪原点初始化，等首个可信 GNSS 再重锚。
static constexpr float GNSS_INIT_MAX_H_ACC_M = 5.0f;
static constexpr float GNSS_INIT_MAX_V_ACC_M = 10.0f;
// NMEA 兜底 epoch 的 vAcc=0、sAcc=3.0（ubx.cpp），部分 UBX 接收机也不上报
// 精度；0 视为未知而不是“不合格”，sAcc 门放到 5 m/s 仍能挡住野值。
static constexpr float GNSS_INIT_MAX_S_ACC_MPS = 5.0f;

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

/**
 * @brief 判断 epoch 是否满足“只融速度、位置降权”的降级门限
 * @details 输入：epoch；处理：iTOW 一致性与重复判重 → fix≥3D、卫星数、
 *          速度精度与 pDOP 门限；输出：是否可走降级路径。
 * @param[in] epoch 本帧 UBX 历元
 * @return true 可降级融合；false 完全不融合
 * @note 降级门限比全质量门限宽松（位置精度不参与），但速度精度必须可信。
 * @see `gnssEpochPassesFullQuality()`、`applyDowngradedGnssUpdate()`
 */
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

/**
 * @brief 用首次真实 GNSS 把 EKF 位置/速度硬重锚到真值，并恢复真实起飞点
 * @details 输入：GNSS 速度、GNSS LLA、量测年龄；处理：按速度把 LLA 外推到当前
 *          时刻 → `ResetPositionVelocityToGnss()` → 若解锁过则用重锚前后相对
 *          位移反推真实起飞点并重设原点 → 否则把原点设为当前点；
 *          输出：EKF 位置/速度、`relative_*`、导航原点。
 *          不变量：只改位置/速度与原点，姿态与 IMU 零偏保持连续。
 * @param[in] gnss_vel_ned      本帧 GNSS 速度 [m/s]（NED）
 * @param[in] gnss_lla          本帧 GNSS 位置（WGS84 弧度 + 椭球高 m）
 * @param[in] measurement_age_s 量测年龄 [s]，用于把位置外推到当前时刻
 * @note 无 GNSS 假原点启动后的“首次真实锚定”专用；普通帧走残差门控更新。
 * @see `InsGnssShouldReanchor()`、`setNavigationOriginFromLlaRad()`
 */
static void reanchorNavigationToGnss(const Eigen::Vector3f &gnss_vel_ned,
                                     const Eigen::Vector3d &gnss_lla,
                                     const float measurement_age_s)
{
  const Eigen::Vector3d current_gnss_lla = bfs::ned2lla(
      gnss_vel_ned.cast<double>() * static_cast<double>(measurement_age_s),
      gnss_lla, bfs::AngPosUnit::RAD);
  Eigen::Vector3d takeoff_to_current_ned = Eigen::Vector3d::Zero();
  // 假原点初始化时 EKF 的“历史位移”没有真实起飞点语义；若此时 reacquire
  // 已先重置过位置，再用 EKF 位置反推会把假原点当成起飞点。直接以当前
  // GNSS 为原点，避免污染 origin。
  const bool preserve_takeoff_origin =
      g_is_unlocked && is_origin_lla_set && !nav_initialized_with_fake_origin;
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

/**
 * @brief 边缘 GNSS 解（仅速度可靠、位置不可信）的降级融合
 * @details 输入：epoch、步长、量测年龄；处理：把位置 R 放大到 1e4（等价关闭
 *          位置融合，防止弱位置把原点拉偏）→ 速度 R 用接收机 spd_acc →
 *          `MeasurementUpdateDetailed()` → 慢速校正后台 AHRS 航向偏置；
 *          输出：EKF 速度更新结果与 `gnss_status.last_test_ratio`。
 *          不变量：位置量测使用 epoch 的真实 LLA，不能用 EKF 自身估计当量测。
 * @formula R_pos = 1e4，R_vel = spd_acc；K→0 仅对位置通道生效
 * @ref REF-121 §2.2.1 p.33（量测噪声 R 的物理含义与降权处理）；
 *          航向弱可观时不要用速度新息“造”航向观测 @ref REF-121 §8.2.3 p.291
 * @frame NED / WGS84
 * @units m、m/s、s
 * @note 降级帧也计为有效 3D fix（`fix3d_count++`），但不刷新全质量快照。
 * @see `gnssEpochQualifiesForDowngrade()`
 */
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
  // 改传 epoch 的真实 LLA，与全质量路径（`MeasurementUpdateDetailed()` 调用点）一致；
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
    // 闭环退化为开环积分器，长降级下校正量持续缠绕。此处与全质量路径的
    // AHRS 慢校正段统一。
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

/**
 * @brief 双矢量航向观测：由 GNSS 航迹角与光流侧滑角反解机头航向并送入 EKF
 * @details 输入：本帧 GNSS 速度快照（NED）、光流机体速度（FRD）、滤波后角速度；
 *          处理：静止/GNSS 不新鲜/光流不新鲜直接跳过 → 三道门控（低速、大机动、
 *          尺度一致性）→ 航向 = 航迹角 − 侧滑角 → 动态 R → 标量航向更新；
 *          输出：EKF 航向量测（无返回值，失败静默跳过）。
 *          不变量：只用“同一历元”的 GNSS 速度快照与新鲜光流；航向是相对真北的
 *          绝对观测，但仅在速度方向与机头存在确定几何关系时成立。
 *
 * @formula ψ = atan2(v_E,v_N) − atan2(v_y^b,v_x^b)，σ_ψ = max(σ_min, σ_0/(v−v_min+1))
 * @ref REF-120 §9.8.14 p.320–322（航迹角/侧滑角几何与航向弱可观性：失准角
 *          通过比力水平分量进入速度观测）；双矢量定姿思想 @ref REF-120 §9.9.1 p.323
 * @frame v^{n} 为 NED，v^{b} 为机体 FRD，ψ 为绕 NED z 轴的航向角
 * @units rad、m/s、rad/s
 * @approx 忽略风（真空速≠地速时侧滑角含风致偏差）与光流安装臂效应；
 *          偏航角速度门限 0.35 rad/s 用于压制时间不同步误差。
 * @deviation 本观测把“GNSS 航迹 − 光流侧滑”当绝对航向：悬停/匀速直线时航迹角
 *          退化、侧滑角≈0，此时观测退化为“航向≈航迹角”，只在有速度时成立；
 *          这不是磁航向，也不是秦书意义的罗经对准。
 * @test 仿真 SC4 过渡段 kick、test_host 航向相关回归（见核查记录）
 */
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
  // （`is_static_confirmed` 分支的 meas_vel_ned 置零段），航向融合应同步跳过，
  // 避免与"静止即零速"语义矛盾。
  if (static_det.confirmed_static)
  {
    return;
  }

  // 检查 GPS 状态：必须是新鲜的 3D Fix，不能使用UBX缓存中的旧速度矢量修正航向。
  if (!isGnssInstantValidForNav())
    return;

  // 检查 光流 状态 (必须有效且新鲜)
  if (!flow_data.is_flow_valid)
    return;

  // 光流数据陈旧保护：超过 kFlowDataStaleTimeoutMs 未收到新帧时，
  // 不再用旧速度参与航向融合，避免传感器停流后继续污染 EKF。
  if ((millis() - flow_data.last_update_ms) > kFlowDataStaleTimeoutMs)
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

  // 调试建议：需要观察融合效果时用 DBG vars / 0xF2 上报 estimated_yaw 与
  // dynamic_noise，不要在 200 Hz 路径里加串口打印（会破坏导航时序）。
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

// ----------------------------------------------------------------------
// 室内无 GNSS 光流速度融合（NED）
// 仅在：导航已初始化、GNSS 无效、非静止确认、光流数据有效且新鲜时执行。
// 光流采样约 50Hz，而导航调度 200Hz；用 last_update_ms 门控，保证每个
// 新光流样本只融合一次，避免重复注入同一观测。
//
// ★ 2026-08-21 失锁重锚：底层速度更新走 MeasurementUpdateAidingVelocityDetailed
//   （豁免 ZUPT 零速守卫——悬停时近零测量是合法观测，漂移超 5.5m/s 后旧接口
//   会永久硬拒）。若光流量测连续 50 次被拒（≈1s @50Hz，EKF 速度漂移超门或
//   P 坍缩），把水平速度重锚到光流测量值，速度协方差抬回 0.5m/s 量级。
// ----------------------------------------------------------------------
/**
 * @brief 室内无 GNSS 时的光流速度融合（含失锁重锚）
 * @details 输入：光流机体速度 FRD、EKF 姿态四元数、当前垂直速度；
 *          处理：新鲜度判重（每个光流样本只融合一次）→ 用 C_b2n 旋转到 NED →
 *          水平两轴动态 R、垂直通道 R=1e3 近似关闭 → 走豁免 ZUPT 守卫的
 *          辅助速度接口 → 连续 50 次被拒时把水平速度重锚到光流值并抬 P；
 *          输出：EKF 速度更新（失败计数/重锚计数进 `g_flow_fusion_*`）。
 *          不变量：垂直通道不可观，不参与修正；本函数不进入静止确认分支。
 * @formula v^n = C_b^n·[v_x^b, v_y^b, 0]^T，C_b^n = (C_n^b)^T = quat2dcm(q)^T
 * @ref REF-120 §7.4 (7.4.6) p.189（速度量测的坐标变换与 R 建模）；
 *          EKF 量测更新/NIS 门控 @ref REF-121 §2.2.1 p.33–34、§7.3.1 p.255
 * @frame 光流 FRD → NED；水平可视、垂向不可观
 * @units m/s、m/s²、rad
 * @approx 光流 z 分量取 0，误差 ≈ v_z·sin(tilt)，室内小垂速下可忽略。
 * @deviation 光流速度失锁重锚是 2026-08-21 加的恢复路径（豁免 ZUPT 零速守卫），
 *          与 ZUPT 路径行为不同；重锚会改变速度状态，故用连续拒绝计数而非单帧触发。
 * @test test_host/test_ekf_aiding_velocity.cpp（16 断言，fw-tests 区域）
 */
static void handleIndoorOpticalFlowVelocityFusion()
{
  static uint32_t s_last_fused_flow_update_ms = 0;
  static uint32_t s_consecutive_rejects = 0;

  if (!nav_system_initialized)
  {
    return;
  }
  if (isGnssInstantValidForNav())
  {
    return;
  }
  if (static_det.confirmed_static)
  {
    return;
  }
  if (!flow_data.is_flow_valid)
  {
    return;
  }
  if ((millis() - flow_data.last_update_ms) > kFlowDataStaleTimeoutMs)
  {
    return;
  }
  if (flow_data.last_update_ms == s_last_fused_flow_update_ms)
  {
    return; // 本光流样本已经融合过，等待下一个新样本
  }
  s_last_fused_flow_update_ms = flow_data.last_update_ms;

  // 光流速度在机体 FRD 下，经 EKF 全姿态四元数旋转到 NED（C_b2n = quat2dcm(q)^T，
  // 与库内 t_b2ned 同一约定）。机体系 Z 分量光流不可观，置 0 引入的误差
  // ≈ v_z·sin(tilt)，室内小垂直速度下可忽略。
  // 室内无磁/无 GNSS 时航向由 EKF 陀螺积分维持，长时间航向漂移仍会使
  // NED 分量产生旋转误差。
  const Eigen::Matrix3f c_b2ned = bfs::quat2dcm(nav_ekf.quat()).transpose();
  // volatile 先拷到本地：单点读取，也避免 volatile 无法绑定 Eigen 常量引用。
  const float flow_vx_mps = flow_data.velocity_x_mps;
  const float flow_vy_mps = flow_data.velocity_y_mps;
  const Eigen::Vector3f vel_body(flow_vx_mps, flow_vy_mps, 0.0f);
  const Eigen::Vector3f vel_ned = c_b2ned * vel_body;
  const float vel_north = vel_ned(0);
  const float vel_east = vel_ned(1);

  // 动态噪声：速度越大光流通常越抖；质量低时进一步降信任。
  const float speed_mps = sqrtf(vel_north * vel_north + vel_east * vel_east);
  float noise_mps = kf_h_r_vel_base + 0.10f * speed_mps;
  if (flow_data.flow_quality < 50)
  {
    noise_mps += 0.20f;
  }
  noise_mps = constrain(noise_mps, 0.10f, 2.0f);

  // 光流不观测垂直通道：D 分量用 EKF 当前值 + 极大噪声（1e3），该通道 K≈0，
  // 状态与协方差都不受无信息压缩，数值上等价于纯 N/E 二维更新。
  const Eigen::Vector3f measured_vel_ned(
      vel_north, vel_east, nav_ekf.ned_vel_mps()(2));
  const float down_noise_mps = 1000.0f;

  const bfs::MeasurementUpdateResult result =
      nav_ekf.MeasurementUpdateAidingVelocityDetailed(measured_vel_ned,
                                                      noise_mps,
                                                      down_noise_mps);
  g_flow_fusion_attempts++;
  if (result.fused)
  {
    g_flow_fusion_fused++;
    s_consecutive_rejects = 0;
  }
  else if (result.attempted)
  {
    g_flow_fusion_rejected++;
    s_consecutive_rejects++;
    if (s_consecutive_rejects >= 50U)
    {
      // 失锁恢复：水平速度重置到光流测量；垂直保持 EKF 当前值（光流不可观）。
      nav_ekf.ResetVelocityToExternal(measured_vel_ned, 0.5f);
      s_consecutive_rejects = 0;
      g_flow_fusion_reanchors++;
    }
  }
}

#if BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE
static InsGnssDelayAutoEstimator s_gnss_delay_auto;
static uint32_t s_gnss_delay_auto_last_ms = 0;
static InsInertialReference s_ins_ref;
static InsBiasJumpMonitor s_bias_jump_monitor;
static uint32_t s_bias_jump_hold_until_ms = 0;
static InsBiasTempObserver s_bias_temp_observer;
static InsBiasTempTable s_bias_temp_table;
static uint32_t s_bias_temp_table_last_ms = 0;
#endif
static InsVibrationEstimator s_vib_estimator;
static uint32_t s_vib_gnss_bad_until_ms = 0;
static bfs::MagMeasurementGate s_mag_gate;
static bfs::HeadingAidingGate s_heading_aiding_gate;
static uint32_t s_mag_last_consumed_seq = 0;
static uint32_t s_mag_last_attempt_us = 0xFFFFFFFFu; // 首帧立即允许

// IMU 无样本/非有限/Failed 时导航本帧不更新，但位置保持门控必须立即失效；
// 否则上一帧的“磁航向有效”会继续放行位置闭环（控制仍用最后有效姿态）。
static void markHeadingAidingInvalid()
{
  bfs::HeadingAidingInput heading_in; // 年龄默认 FLT_MAX = 无源
  (void)s_heading_aiding_gate.update(heading_in);
  g_heading_aiding_valid = false;
  g_heading_aiding_age_s = -1.0f;
  g_pos_hold_degraded = true;
}
static bfs::ImuHealthMonitor s_imu_health;
static bfs::InsGnssConsistencyMonitor s_gnss_consistency;
static Eigen::Vector3f s_gnss_imu_dv_accum = Eigen::Vector3f::Zero();
static uint32_t s_gnss_last_epoch_us = 0;

static void publishImuHealth(const bfs::ImuHealthStatus &status)
{
  g_imu_health_state = static_cast<uint8_t>(status.state);
  g_imu_health_flags = status.flags;
  g_imu_health_frozen_s = status.frozen_s;
  g_imu_health_no_sample_s = status.no_sample_s;
  g_imu_health_sample_rate_hz = status.sample_rate_hz;
#ifdef FW_SIL_REAL_NAV
  g_sil_nav_debug_imu_failed =
      (status.state == bfs::ImuHealthState::Failed);
#endif
}

static bfs::ImuHealthStatus updateImuHealth(
    const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
    const uint32_t sample_count, const float dt_s)
{
  const float vib_g = s_vib_estimator.last().accel_rms_g;
  // 复用既有零偏跳变监测的保持标志（上一帧写入）作为健康 Warning 输入，
  // 不重复实现跳变检测；一帧延迟对 5 s 保持窗口无影响。
  const bool bias_jump_hint = gnc_tel.bias_jump_hold;
  const bfs::ImuHealthStatus status = s_imu_health.update(
      accel, gyro, sample_count, dt_s, vib_g, g_is_unlocked, throttlePercent,
      bias_jump_hint);
  publishImuHealth(status);
  return status;
}

static void publishGnssConsistency(const bfs::GnssConsistencyStatus &status)
{
  gnss_status.consistency_suspect =
      (status.state == bfs::GnssConsistencyState::Suspect);
  gnss_status.consistency_accel_res = status.accel_res_mps2;
  gnss_status.consistency_pos_res = status.pos_res_m;
  gnss_status.consistency_flow_res = status.flow_res_mps;
  gnss_status.consistency_baro_span = status.baro_span_m;
  gnss_status.consistency_reason = static_cast<uint8_t>(status.last_reason);
  gnss_status.consistency_suspect_count = status.suspect_count;
}

// 参考场按 home 站点固定；origin 尚未建立时按 home 处理（GNSS 首帧前）。
static bool magReferenceValidForCurrentOrigin()
{
  if (!bfs::kDefaultMagReference.valid)
  {
    return false;
  }
  if (!is_origin_lla_set)
  {
    return true;
  }
  const float distance_km = bfs::magref::siteDistanceKm(
      static_cast<float>(origin_lat_deg), static_cast<float>(origin_lon_deg),
      bfs::kDefaultMagReference.site_lat_deg,
      bfs::kDefaultMagReference.site_lon_deg);
  return distance_km <= 50.0f;
}

/**
 * @brief 导航主任务（200 Hz）：取 IMU → 静止检测 → EKF 初始化/预测 →
 *        静止/AHRS 辅助 → GNSS/光流/气压量测 → 输出桥
 * @details 完整时序见文件头 `@details`。本函数在 200 Hz 调度 tick 内调用，
 *          禁止阻塞（不可加 `delay()`/长串口打印）；所有共享量在取数时关中断
 *          快照，量测更新使用本帧出队 epoch，避免跨历元混用。
 *          不变量：
 *          1) 时间更新每帧必做；量测更新按质量门限条件执行；
 *          2) 静止确认可被 GNSS/光流速度推翻（只出不进，非对称）；
 *          3) GNSS 联合更新失败时，子集更新与 `consider_heading` 只在本次生效；
 *          4) 输出桥的 `relative_*`/`estimated_*` 与控制接口单位一致（m、m/s）。
 * @note 关键数值（静止进 20 帧/出 5 帧、GNSS 退出门限 0.8 m/s、气压 25 Hz、
 *       振动 Q 回退 0.25）改动前必须走行为红线流程并重采回归基线。
 * @see `handleVerticalEstimation()`、`handleHorizontalEstimation()`、
 *      `handleDualVectorYawFusion()`、`handleIndoorOpticalFlowVelocityFusion()`
 */
#ifdef FW_SIL_REAL_NAV
int g_sil_nav_debug_imu_count = 0;
int g_sil_nav_debug_imu_invalid = 0;
bool g_sil_nav_debug_imu_failed = false;
float g_sil_nav_debug_accel[3] = {0.0f, 0.0f, 0.0f};
float g_sil_nav_debug_gyro[3] = {0.0f, 0.0f, 0.0f};
bool g_sil_nav_debug_static = false;
float g_sil_nav_debug_static_confidence = 0.0f;
uint32_t g_sil_nav_debug_static_frames = 0;
bool g_sil_nav_debug_gnss_epoch = false;
bool g_sil_nav_debug_initialized = false;
#endif

#ifdef BFS_AHRS_ATT_GNSS_OUTAGE_GATE
// Madgwick 的 roll/pitch 来自“陀螺积分 + 加速度重力修正”。GNSS 失联且
// 持续推力时比力≈mg，AHRS 会把倾斜航迹逐渐拉回水平；以 2° 噪声把它
// 灌进 EKF 会覆盖陀螺，形成姿态锁平。仅在 GNSS 曾成功融合且当前已失联
// 时禁用该量测；首次定位前仍允许 AHRS 辅助启动。
static bool ahrsAttitudeAidAllowed(const float roll_rad)
{
  (void)roll_rad;
  const bfs::AidSourceStatus &gnss = aid_tracker.status(bfs::AidSource::Gnss);
  const bool gnss_was_used = gnss.fuse_count > 0U;
  const bool gnss_recent =
      !std::isfinite(gnss.age_s) || gnss.age_s <= 0.5f;
#ifdef BFS_AHRS_ATT_GNSS_OUTAGE_BOUNDED
  // 长失联不再禁用 AHRS：20 s 50 seeds 出现 roll 179.7° 尾部反噬。
  const bool beyond_gate_window =
      std::isfinite(gnss.age_s) && gnss.age_s > 10.0f;
#else
  const bool beyond_gate_window = false;
#endif
#if defined(BFS_AHRS_ATT_ROLL_GUARD_45) || defined(BFS_AHRS_ATT_ROLL_GUARD_60)
  // 大滚转时临时恢复 AHRS，避免失联尾巴滚到 180°。
#ifdef BFS_AHRS_ATT_ROLL_GUARD_45
  constexpr float kGuardRad = 45.0f * 0.017453292f;
#else
  constexpr float kGuardRad = 60.0f * 0.017453292f;
#endif
  if (gnss_was_used && !gnss_recent &&
      std::isfinite(roll_rad) && std::fabs(roll_rad) > kGuardRad)
  {
    return true;
  }
#endif
  return !gnss_was_used || gnss_recent || beyond_gate_window;
}
#endif

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
  MagSample mag_sample{};
  bool mag_real_for_init = false;
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
#ifdef FW_SIL_REAL_NAV
  g_sil_nav_debug_imu_count = local_imu_sample_count;
#endif
  if (local_imu_sample_count == 0)
  {
    interrupts();
    // 无样本也要推进健康计时，否则冻结/丢样会被静默跳过。
    updateImuHealth(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), 0U, dt_s);
#ifdef BFS_NAV_PROFILE
    recordNavProfileStage(9U, micros() - _nav_profile_total_start);
    printNavProfileIfDue();
#endif
    markHeadingAidingInvalid();
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
  // 健康监测先于 NaN 门和静止检测：非有限/冻结/丢样要留下状态，
  // 不能只在控制侧“保持上一帧”而让导航继续积分。
  const bfs::ImuHealthStatus imu_health_status =
      updateImuHealth(imu_accel_mps2, imu_gyro_radps,
                      static_cast<uint32_t>(local_imu_sample_count), dt_s);
  // 传感器 NaN/Inf 安全门：跳过本帧传播，保持上一有效导航输出；控制侧另有
  // 陀螺有限值守卫。不能把 NaN 灌进 EKF/姿态环（SIL 故障注入已复现崩溃）。
  if (!imu_accel_mps2.allFinite() || !imu_gyro_radps.allFinite())
  {
#ifdef FW_SIL_REAL_NAV
    ++g_sil_nav_debug_imu_invalid;
#endif
    markHeadingAidingInvalid();
    return;
  }
  // 单 IMU 失效没有独立姿态源：确认 Failed 后保持最后有效导航输出，
  // 不继续用冻结样本积分，也不伪称“受控下降”。控制侧仍按最后有效陀螺值保持。
  if (imu_health_status.state == bfs::ImuHealthState::Failed)
  {
    markHeadingAidingInvalid();
    return;
  }
#ifdef FW_SIL_REAL_NAV
  g_sil_nav_debug_accel[0] = imu_accel_mps2(0);
  g_sil_nav_debug_accel[1] = imu_accel_mps2(1);
  g_sil_nav_debug_accel[2] = imu_accel_mps2(2);
  g_sil_nav_debug_gyro[0] = imu_gyro_radps(0);
  g_sil_nav_debug_gyro[1] = imu_gyro_radps(1);
  g_sil_nav_debug_gyro[2] = imu_gyro_radps(2);
#endif

  {
    const InsVibrationEstimate vib =
        s_vib_estimator.Update(imu_accel_mps2, imu_gyro_radps, dt_s);
    gnc_tel.vib_accel_rms_g = vib.accel_rms_g;
    gnc_tel.vib_gyro_rms_dps = vib.gyro_rms_dps;
    gnc_tel.vib_accel_std_suggest = vib.accel_std_suggest;
    gnc_tel.vib_high = vib.high_vibration;
  }
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

  // ★ 2026-10 P0-3：ZUPT 外部参照门控。
  //   理论依据：秦永元、张洪钺、汪叔华《卡尔曼滤波与组合导航原理（第3版）》，
  //   西北工业大学出版社 2015，§8.2.3 p.291：“在间接法滤波中，要实现对某一
  //   导航系统的阻尼，必须利用来自其他导航系统的外来信息，而不能利用自身
  //   信息作为阻尼”；§7.3.1 p.255 系统级 FDI 用 χ² 检验。
  //   失联/无光流时，静止检测仍可能误确认；同时 P_vv 被膨胀后 NIS 门限失效，
  //   ZUPT 会把慢速巡航（尤其 1.0–1.5 m/s）速度拉零。这里按"是否存在外部速度
  //   参照"做结构门控：无外部参照且 |v|>0.75 m/s 立即拒绝 ZUPT；无参照 3 s 后
  //   噪声封顶 0.30 m/s；无参照 30 s 后停用 ZUPT，仅保留 Gravity/StaticGyro。
  //   详见 ins_static_aid_profile.h。
  static Icm45686ZuptExtRefGuard s_zupt_ext_ref_guard;
  const bool has_external_velocity_reference =
      isGnssDataFreshForNav() ||
      (flow_data.is_flow_valid &&
       (millis() - flow_data.last_update_ms) <= kFlowDataStaleTimeoutMs);
  s_zupt_ext_ref_guard = Icm45686UpdateZuptExtRefGuard(
      s_zupt_ext_ref_guard, has_external_velocity_reference, nav_update_dt_s);

  // 获取磁力计数据（真实路径优先，失败回退伪磁）
  // MagSensor 已完成芯片系→FRD 对齐、椭球标定与标定有效性校验；
  // 这里只做初始化前的粗量程检查，完整门控在 EKF 运行阶段执行。
  mag_sample = magSensor.sampleFRD();
  const uint32_t mag_now_us = micros();
  const bool mag_present = magSensor.isPresent();
  const bool mag_cal_valid = magSensor.calValid();
  const bool mag_ref_valid = magReferenceValidForCurrentOrigin();
  const float mag_norm_ut = std::sqrt(
      mag_sample.x * mag_sample.x + mag_sample.y * mag_sample.y +
      mag_sample.z * mag_sample.z);
  mag_status.present = mag_present;
  mag_status.fresh = magSensor.fresh();
  mag_status.cal_valid = mag_cal_valid;
  mag_status.ref_valid = mag_ref_valid;
  mag_status.seq = mag_sample.seq;
  mag_status.norm_ut = mag_norm_ut;
  mag_status.norm_dev_ut =
      std::fabs(mag_norm_ut - bfs::kDefaultMagReference.total_ut);
  mag_status.age_ms = (mag_sample.seq == 0U || mag_sample.timestamp_us == 0U)
                          ? 0xFFFFFFFFu
                          : (mag_now_us - mag_sample.timestamp_us) / 1000U;
  mag_status.last_gate_reason =
      static_cast<uint8_t>(bfs::MagGateReason::None);
  mag_real_for_init =
      g_heading_aid_mode != 2U &&
      mag_present && mag_sample.seq != 0U && mag_cal_valid && mag_ref_valid &&
      std::isfinite(mag_sample.x) && std::isfinite(mag_sample.y) &&
      std::isfinite(mag_sample.z) &&
      mag_norm_ut >= 15.0f && mag_norm_ut <= 100.0f;
  if (mag_real_for_init)
  {
    mag_ut << mag_sample.x, mag_sample.y, mag_sample.z;
  }
  else
  {
    // 伪磁只保证确定性初始化，不保证真北；真实磁不可用时保留历史语义。
    mag_ut << 30.0f, 0.0f, 40.0f;
    bfs::MagGateReason init_reason = bfs::MagGateReason::None;
    if (!mag_present || mag_sample.seq == 0U)
    {
      init_reason = bfs::MagGateReason::Stale;
    }
    else if (!mag_cal_valid)
    {
      init_reason = bfs::MagGateReason::CalInvalid;
    }
    else if (!mag_ref_valid)
    {
      init_reason = bfs::MagGateReason::RefInvalid;
    }
    else if (!std::isfinite(mag_sample.x) || !std::isfinite(mag_sample.y) ||
             !std::isfinite(mag_sample.z))
    {
      init_reason = bfs::MagGateReason::NonFinite;
    }
    else if (mag_norm_ut < 15.0f)
    {
      init_reason = bfs::MagGateReason::NormLow;
    }
    else
    {
      init_reason = bfs::MagGateReason::NormHigh;
    }
    mag_status.last_gate_reason = static_cast<uint8_t>(init_reason);
  }

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
  // 有油门出力且 EKF 已初始化时，禁止把“静止”用于 ZUPT/重力/静止陀螺辅助：
  // 螺旋桨振动/机体微动可能在个别帧同时满足模长与角速率门限，误判静止会把
  // 真实垂直速度拉零。用 g_is_unlocked 门控，避免地面未解锁时摇杆微动
  // （throttlePercent>3%）阻断 EKF 初始化；初始化后重置检测器，落地收油门后
  // 重新积累。检测器本身继续更新，不改变初始化语义。
  if (g_is_unlocked && nav_system_initialized && throttlePercent >= 3.0f)
  {
    Icm45686StaticDetectorReset(static_det);
  }
  // ★ 2026-08-21 光流静止出口：检测器原有唯一出口是 GNSS 速度>1.5m/s（见下方
  //   GNSS 分支），室内无 GNSS 时恒速平飞会误确认静止 → ZUPT 钉零速度 → 光流
  //   融合又被 confirmed_static 门控自我禁用（唯一能证伪静止的传感器被静止假设
  //   关掉）。这里用新鲜有效的光流速度做第二出口：模长>1m/s 期间持续按住复位，
  //   检测器无法积累确认帧（与 GNSS 出口同构：只出不进，非对称；光流静止噪声
  //   ≪1m/s，不误挡真静止确认）。
  if (flow_data.is_flow_valid &&
      (millis() - flow_data.last_update_ms) <= kFlowDataStaleTimeoutMs)
  {
    const float flow_speed_mps =
        sqrtf(flow_data.velocity_x_mps * flow_data.velocity_x_mps +
              flow_data.velocity_y_mps * flow_data.velocity_y_mps);
    if (flow_speed_mps > 1.0f)
    {
      Icm45686StaticDetectorReset(static_det);
    }
  }
  bool is_static_confirmed =
      static_det.confirmed_static && s_imu_health.staticAidsAllowed();
  // 地面静止校准：EKF 已初始化、油门未上量且检测器确认静止才累计。
  // 默认 30 s 后才允许无磁位置保持；陀螺零偏/水平初值需要长静置收敛，
  // 2 s 级静置不够（用户 2026-10-25 明确要求几十秒量级）。
  if (!g_imu_static_calibrated && nav_system_initialized &&
      throttlePercent < 3.0f && is_static_confirmed &&
      std::isfinite(nav_update_dt_s) && nav_update_dt_s > 0.0f)
  {
    g_static_calib_s += nav_update_dt_s;
    if (g_static_calib_s >= g_static_calib_required_s)
    {
      g_imu_static_calibrated = true;
    }
  }
  const float static_confidence = static_det.confidence;
#ifdef FW_SIL_REAL_NAV
  g_sil_nav_debug_static = is_static_confirmed;
  g_sil_nav_debug_static_confidence = static_confidence;
  g_sil_nav_debug_static_frames = static_det.confirmed_static_frames;
  g_sil_nav_debug_initialized = nav_system_initialized;
#endif

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
#ifdef FW_SIL_REAL_NAV
  g_sil_nav_debug_gnss_epoch = has_gnss_epoch;
#endif
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
#if BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE && BFS_NAVIGATION_GNSS_DELAY_AUTO_APPLY
      // 仅在估计有效、曲线尖锐、与先验偏差合理时替换固定 15 ms。
      const uint32_t now_auto_apply_ms = millis();
      if (gnss_status.auto_delay_valid &&
          !gnc_tel.vib_high &&
          (now_auto_apply_ms - gnss_status.auto_delay_last_ms) <= 5000U &&
          gnss_status.auto_delay_sharpness >= 0.003f &&
          std::fabs(gnss_status.auto_delay_tau_s -
                    BFS_NAVIGATION_GNSS_DELAY_S) <= 0.10f)
      {
        gnss_measurement_age_s = gnss_status.auto_delay_tau_s;
      }
#endif
      gnss_status.last_measurement_age_s = gnss_measurement_age_s;
    }
    else
    {
      gnss_status.time_mapping_invalid_count++;
    }
  }

  // GNSS 一致性监视所需的机体系比力→NED 速度增量（每帧积累，epoch 消费后清零）。
  if (nav_system_initialized)
  {
    const Eigen::Matrix3f c_b2n = bfs::quat2dcm(nav_ekf.quat()).transpose();
    const Eigen::Vector3f specific_force_ned =
        c_b2n * (imu_accel_mps2 - nav_ekf.accel_bias_mps2());
    Eigen::Vector3f gravity_ned;
    gravity_ned << 0.0f, 0.0f,
        static_cast<float>(nav_ekf.gravity_mps2());
    const Eigen::Vector3f accel_ned = specific_force_ned + gravity_ned;
    s_gnss_imu_dv_accum += accel_ned * nav_update_dt_s;
  }

  const bool gnss_data_fresh_for_nav = isGnssDataFreshForNav();
  const bool gnss_full_quality_for_nav = has_gnss_epoch && gnssEpochPassesFullQuality(gnss_epoch);
  // 首次真实重锚必须使用高精度解；低精度/野值帧不能把 EKF 从假原点锚到错误位置。
  const bool gnss_quality_for_reanchor =
      gnss_full_quality_for_nav &&
      gnss_epoch.horz_acc_m > 0.0f &&
      gnss_epoch.horz_acc_m <= GNSS_INIT_MAX_H_ACC_M &&
      (gnss_epoch.vert_acc_m <= 0.0f ||
       gnss_epoch.vert_acc_m <= GNSS_INIT_MAX_V_ACC_M) &&
      (gnss_epoch.spd_acc_mps <= 0.0f ||
       gnss_epoch.spd_acc_mps <= GNSS_INIT_MAX_S_ACC_MPS);
  const bool gnss_downgrade_quality_for_nav = has_gnss_epoch && !gnss_full_quality_for_nav && gnssEpochQualifiesForDowngrade(gnss_epoch);
  bool gnss_instant_valid_for_nav = gnss_full_quality_for_nav;

  // 慢漂移/欺骗一致性：只在完整质量 epoch 上做窗口检验；Suspect 时把
  // gnss_instant_valid_for_nav 置 false，让后续 GNSS 更新跳过，并让 AHRS/光流
  // 辅助接管。它不是瞬时野值门（后者仍在 EKF 内保留）。
  if (nav_system_initialized && gnss_full_quality_for_nav)
  {
    const uint32_t now_us = micros();
    const float epoch_dt_s =
        (s_gnss_last_epoch_us == 0U)
            ? 0.0f
            : static_cast<float>(static_cast<uint32_t>(now_us -
                                                        s_gnss_last_epoch_us)) *
                  1.0e-6f;
    Eigen::Vector3d gnss_pos_ned = Eigen::Vector3d::Zero();
    bool pos_valid = false;
    if (is_origin_lla_set)
    {
      const Eigen::Vector3d gnss_lla(gnss_epoch.lat_rad, gnss_epoch.lon_rad,
                                     static_cast<double>(gnss_epoch.alt_wgs84_m));
      const Eigen::Vector3d origin_lla(origin_lat_rad, origin_lon_rad,
                                       origin_alt_m);
      if (gnss_lla.allFinite() && origin_lla.allFinite())
      {
        gnss_pos_ned = bfs::lla2ned(gnss_lla, origin_lla, bfs::AngPosUnit::RAD);
        pos_valid = gnss_pos_ned.allFinite();
      }
    }
    bool flow_valid = false;
    Eigen::Vector3f flow_ned = Eigen::Vector3f::Zero();
    if (flow_data.is_flow_valid && flow_data.flow_quality >= 50 &&
        (millis() - flow_data.last_update_ms) <= kFlowDataStaleTimeoutMs &&
        std::isfinite(flow_data.velocity_x_mps) &&
        std::isfinite(flow_data.velocity_y_mps))
    {
      const Eigen::Matrix3f c_b2n = bfs::quat2dcm(nav_ekf.quat()).transpose();
      Eigen::Vector3f flow_body;
      flow_body << static_cast<float>(flow_data.velocity_x_mps),
          static_cast<float>(flow_data.velocity_y_mps), 0.0f;
      flow_ned = c_b2n * flow_body;
      flow_valid = flow_ned.allFinite();
    }
    bfs::GnssConsistencyInput consistency_input;
    consistency_input.dt_s = (epoch_dt_s > 0.0f && epoch_dt_s <= 0.35f)
                                 ? epoch_dt_s
                                 : 0.1f;
    consistency_input.gnss_vel_ned
        << gnss_epoch.north_vel_mps, gnss_epoch.east_vel_mps,
        gnss_epoch.down_vel_mps;
    consistency_input.imu_dv_ned = s_gnss_imu_dv_accum;
    consistency_input.pos_valid = pos_valid;
    consistency_input.gnss_pos_ned = gnss_pos_ned.cast<float>();
    consistency_input.flow_valid = flow_valid;
    consistency_input.flow_vel_ned = flow_ned;
    consistency_input.baro_valid = pos_valid && std::isfinite(baro_altitude);
    consistency_input.gnss_alt_up_m =
        pos_valid ? -static_cast<float>(gnss_pos_ned.z()) : 0.0f;
    consistency_input.baro_alt_up_m = baro_altitude;
    consistency_input.reset =
        (s_gnss_last_epoch_us == 0U || !(epoch_dt_s > 0.0f) ||
         epoch_dt_s > 0.35f);
    const bfs::GnssConsistencyStatus consistency_status =
        s_gnss_consistency.update(consistency_input);
    publishGnssConsistency(consistency_status);
    if (consistency_status.state == bfs::GnssConsistencyState::Suspect &&
        g_gnss_consistency_reject_enabled)
    {
      ++gnss_status.consistency_reject_count;
      gnss_instant_valid_for_nav = false;
    }
    s_gnss_last_epoch_us = now_us;
    s_gnss_imu_dv_accum.setZero();
  }
  else if (!gnss_data_fresh_for_nav)
  {
    s_gnss_consistency.reset();
    publishGnssConsistency(s_gnss_consistency.status());
    s_gnss_last_epoch_us = 0U;
    s_gnss_imu_dv_accum.setZero();
  }

  if (gnss_instant_valid_for_nav || gnss_downgrade_quality_for_nav)
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
    //
    // ★ 对准语义（必须读懂，避免误称为“罗经对准”）：
    //   本机**没有**秦书 §9.9 的静基座罗经/自对准：MEMS 陀螺零偏（配置的初始
    //   不确定度 0.4 dps）远大于地球自转水平分量（长沙 28.2°N 时 ω_ie·cosL≈13.25°/h），
    //   无法用 ω_ie 定向；§9.9 的粗对准双矢量是 g^b 与 ω_ie^b，这里第二把
    //   “尺子”换成了**伪磁常量** (30,0,40)（见下方 mag_ut）经 TiltCompass，
    //   故航向初值只保证“数值确定”，不保证物理指向真北。
    //   航向的物理修正只来自：GNSS 航迹−光流侧滑的双矢量观测（有速度时），
    //   或未来接入的磁矢量。静止时航向不可观（无绝对航向源）。
    //   参考：@ref REF-120 §9.9 p.322–338（对准与可观测性）、
    //        notes/06 §1（φ_U≈ε_E/(ω_ie·cosL)）；P0(8,8)=0.2 rad 只是把
    //        “伪磁确定性初始化”的 1σ 写成 0.2（≈11.5°），不是真实航向精度。
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
      const bool gps_available =
          gnss_instant_valid_for_nav &&
          gnss_epoch.horz_acc_m > 0.0f &&
          gnss_epoch.horz_acc_m <= GNSS_INIT_MAX_H_ACC_M &&
          (gnss_epoch.vert_acc_m <= 0.0f ||
           gnss_epoch.vert_acc_m <= GNSS_INIT_MAX_V_ACC_M) &&
          (gnss_epoch.spd_acc_mps <= 0.0f ||
           gnss_epoch.spd_acc_mps <= GNSS_INIT_MAX_S_ACC_MPS);

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
      //    ★ 2026-08-13 优化研究修正（原 π，仿真实证"航向弱可观踢动"机制）：
      //    ① 本机 TiltCompass 用伪磁场 (30,0,40) 做确定性航向初始化（见上文
      //       mag_ut 赋值段），
      //      初始航向实际是"伪磁系下的确定航向"，π 的语义与之一致性矛盾（原 LOW-7）；
      //    ② 仿真证明 P0(8,8)=π 时，大比力水平分量（VTOL 过渡/转弯）会把 GNSS
      //      速度噪声放大为 ±10-25°/epoch 的航向踢动（K[8,v] ∝ f_x·P(8,8)/Q_a），
      //      继而 GNSS 联合更新被 NIS 拒绝、位置/速度永久失锁（闭环悬停 5 场景
      //      全崩，位置误差 2.5-6.7km）；P0=0.2 rad 后闭环姿态误差 55-170°→5-17°。
      //    init_heading_err_std 0.2 rad（≈11.5°）与确定性伪磁初始化语义一致。
      nav_ekf.init_pos_err_std_m(2.0f);            // 初始位置不确定度 2m
      nav_ekf.init_vel_err_std_mps(0.08f);         // 初始速度不确定度 0.08 m/s
      nav_ekf.init_att_err_std_rad(0.08f);         // 初始姿态 (roll/pitch) 不确定度 ~4.6度
      // 无磁手动对准：假定机头指向已知（±3°）；真实磁初始化保留 0.2 rad。
      nav_ekf.init_heading_err_std_rad(
          (g_heading_aid_mode >= 1U)
              ? (mag_real_for_init ? 0.1f : 0.05f)
              : 0.2f);
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

      // 执行初始化。真实磁样本给的是磁北航向，需加站点磁偏角转到真北；
      // 伪磁/不可用磁保持 offset=0，避免改变历史确定性行为。
      const float mag_init_yaw_offset =
          mag_real_for_init
              ? bfs::kDefaultMagReference.declination_rad
              : ((g_heading_aid_mode >= 1U)
                     ? g_heading_align_deg * 0.0174532925f
                     : 0.0f);
      // 重新初始化会重置零偏估计，静止校准必须重新累积。
      g_imu_static_calibrated = false;
      g_static_calib_s = 0.0f;
      nav_ekf.set_q_accel_scale(g_ekf_q_accel_scale);
      nav_ekf.Initialize(imu_accel_mps2, imu_gyro_radps, mag_ut, init_vel_ned,
                         init_lla, mag_init_yaw_offset);
      mag_status.init_source = mag_real_for_init ? 1U : 0U;

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
#if BFS_NAVIGATION_VIBRATION_ADAPTIVE
    // 仅显式打开时改变 EKF 噪声；默认保持 0.25/0.0015。
    // GNSS 失效时不能把加速度 Q 降到保守值以下。
    // 安全策略：振动自适应只允许把 Q 降到固定值以下；
    // 延时/失锁/恢复/低精度时一律回到保守 0.25。
    float vib_q = std::clamp(s_vib_estimator.last().accel_std_suggest, 0.15f, 0.25f);
    if (!gnss_instant_valid_for_nav) {
      s_vib_gnss_bad_until_ms = millis() + 10000U;
    }
    if (static_cast<int32_t>(millis() - s_vib_gnss_bad_until_ms) < 0 ||
        gnss_measurement_age_s > 0.05f ||
        gnss_status.last_obs_h_acc_m > 2.0f ||
        gnss_status.last_obs_h_acc_m < 0.0f) {
      vib_q = 0.25f;
    }
    nav_ekf.accel_std_mps2(vib_q);
    nav_ekf.gyro_std_radps(s_vib_estimator.last().gyro_std_suggest);
#endif
    const bool bias_jump_hold =
        static_cast<int32_t>(millis() - s_bias_jump_hold_until_ms) < 0;
    // GNSS 失锁 >2 s 时冻结加计零偏更新：无外部速度/位置基准，
    // 继续让零偏跟踪会把姿态/振动误差吸收进零偏。
    const bool gnss_bias_freeze =
        !gnss_instant_valid_for_nav && gnss_measurement_age_s > 2.0f;
    // 2026-10-23 组合故障 A/B 发现：把“非激励”加计零偏 Q 压到 0.001
    // 会在阶跃+温漂场景产生 30 km 级位置发散；撤销该宽门控。
    // 只在跳变保持/失锁冻结时使用 0.0001，其余恢复正常 0.010。
    nav_ekf.accel_markov_bias_std_mps2(
        (bias_jump_hold || gnss_bias_freeze) ? 0.0001f : 0.010f);

    Eigen::Vector3f ekf_gyro_input = imu_gyro_radps;

    // [B] 时间更新 (预测) - 始终执行
    // 这一步负责 EKF 内部短时惯导外推；无 GNSS 时还会用 AHRS 姿态量测约束漂移。
    // ── 双子样时间更新（速度增量在 EKF 内做旋转/划摇补偿）──────────────
    // 公式：V^n_m = V^n_{m−1} + C^n_b(m−1)·ΔV_sf,m + ΔV_g/cor,m
    //       ΔV_sf,m = ΔV_m + ΔV_rot,m + ΔV_scull,m
    //       ΔV_rot,m = ½Δθ_m×ΔV_m；ΔV_scull,m = (2/3)(Δθ₁×Δv₂ + Δv₁×Δθ₂)
    // 符号：Δθ 角增量 [rad]、Δv 速度增量 [m/s]、C^n_b 机体系→导航系、下标 1/2 为
    //       导航周期内的前后两个子样（本函数按时间中点拆分样本）
    // 来源：@ref REF-120 §9.6 (9.6.9)(9.6.17)(9.6.19) p.298–299、
    //       (9.6.25) p.301；姿态阵取区间起始时刻，旋转项正是把它补到区间中点
    // 实现：`TimeUpdateTwoSample()`（EKF 内），本段只负责把 IMU 缓冲拆成两子样
    // 等价变换：秦书 ENU/RFU ↔ 本项目 NED/FRD，矢量方程整体协变，分量次序见
    //       @ref MOD-002 §2；旋转效应只允许计一次（2026-10 P0-0 修复）
    // 数值证据：test_host/test_ekf_15state.cpp T7（速度增量残差 1.25e-4）
    // ────────────────────────────────────────────────────────────────
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

      auto profile = Icm45686SelectStaticAidProfile(
          static_confidence, static_det.confirmed_static_frames,
          nav_ekf.covariance_is_healthy());
      Icm45686ApplyZuptExtRefNoiseFloor(profile, s_zupt_ext_ref_guard);

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

      const bool zupt_due_raw = (static_aid_frame_counter % zupt_div) == 0U;
      const bool zupt_due =
          zupt_due_raw &&
          Icm45686ZuptExternalRefAllowsFusion(
              s_zupt_ext_ref_guard, nav_ekf.ned_vel_mps().norm());
      const bool gravity_due = (static_aid_frame_counter % gravity_div) == 1U;
      const bool static_gyro_due = (static_aid_frame_counter % gyro_div) == 3U;
      // 静止陀螺修正是高信任量测（1e-4 rad/s），必须同时满足：
      // 速度残差小、重力模长可信、角速率低、静止已持续足够久。
      // 仅靠 is_static_confirmed 不够：低速漂移/地面滑行/外力推动可能误判。
      const bool static_gyro_consistent =
          (nav_ekf.ned_vel_mps().norm() < 0.30f) &&
          (accel_norm_error < 0.50f) &&
          (imu_gyro_radps.norm() < 0.15f) &&
          (static_det.confirmed_static_frames >= 200U);

      const Icm45686StaticAidAction static_aid_action =
          Icm45686SelectStaticAidAction(
              zupt_due, gravity_due, static_gyro_due,
              profile.prefer_gravity_first);

      // ── 零速修正（ZUPT）：静止时把速度真值 0 作为三轴速度量测 ──────────
      // 公式：z = 0 − v̂，S = H P Hᵀ + R，K = P Hᵀ S⁻¹，x ← x + K·z
      // 符号：v̂ 为 EKF 当前 NED 速度估计 [m/s]；R 由静止档位给出（水平/垂直分设）
      // 来源：@ref REF-121 §8.2.3 p.291（间接法阻尼须用外来信息，ZUPT 的“0”来自
      //       静止假设而非滤波器自身）；静基座观测条件 @ref REF-120 §9.9.3 p.336
      // 实现：`MeasurementUpdateVelocityDetailed()`（EKF 内做 NIS 门控/子集更新）
      // 等价变换：无；三轴速度的观测矩阵 H=[0₃ I₃ 0₃…]
      // 边界：残差 >1.5 m/s 跳过（已非“刚停稳残速”）；无外部参照时噪声封底
      //       0.30 m/s、30 s 后停用（见 ins_static_aid_profile.h）
      // 证据：ZUPT NIS 门限 1000→χ²(3)≈16.3 修正（AGENTS.md EKF 踩坑记录）
      // ────────────────────────────────────────────────────────────────
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
      } else if (static_aid_action == Icm45686StaticAidAction::StaticGyro &&
                 throttlePercent < 3.0f && static_gyro_consistent &&
                 !bias_jump_hold) {
        // 静止陀螺量测: 直接将当前 gyro 读数作为观测, 噪声 ~0.01 rad/s (0.5°/s).
        // 这直接约束三轴陀螺零偏, 尤其对 Z 轴 (yaw) 零偏——ZUPT/Gravity 对其弱可观.
        // ★2026-10 守卫: 油门 >3% 说明电机已出力/已离地，即使 IMU 判“静止”
        // 也不能把机体缓慢漂移当成零角速率去估零偏；否则会污染 bg 并反过来
        // 让姿态估计漂移（hifi 闭环复现：static gyro 在不该融合时把 bg 拉偏）。
        // 参考 esp32-ins-gnss-fusion applyStaticGyroUpdate.
        // 分轴低信任：Z 轴（航向/偏航零偏）用较高信任，X/Y 轴主要由重力/GNSS
        // 约束；给 X/Y 极大噪声，避免静止误判时把滚转/俯仰零偏一起拉坏。
        static constexpr float kStaticGyroZNoiseRadps = 0.0005f;
        static constexpr float kStaticGyroXYNoiseRadps = 1.0f;
        const Eigen::Vector3f static_gyro_noise(
            kStaticGyroXYNoiseRadps, kStaticGyroXYNoiseRadps,
            kStaticGyroZNoiseRadps);
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateStaticGyroDetailed(
                imu_gyro_radps, static_gyro_noise);
        aid_tracker.RecordAttempt(bfs::AidSource::StaticGyro, result);
        _sg_attempt++; if (result.fused) _sg_fused++;
      }
    }
    // 退出静止后重力累加器在下一次进入静止时由 static_gravity_acc=0,count=0 自然重置
    // ========================================================================
    // 无 GNSS + 运动时：AHRS 姿态量测（利用重力融合约束姿态）
    // ========================================================================
    else if (!gnss_instant_valid_for_nav && accel_gravity_trusted && angular_rate_quiet
#ifdef BFS_AHRS_ATT_GNSS_OUTAGE_GATE
             && ahrsAttitudeAidAllowed(nav_ekf.roll_rad())
#endif
            ) {
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

    // [B3] 磁矢量量测（20 Hz）：真实磁传感器路径，门控通过才进 EKF。
    // 不改变位置/速度；只约束绝对航向。磁默认不存在时 MarkUnavailable，
    // 行为与历史伪磁路径一致（伪磁只用于初始化）。
    {
      const uint32_t mag_now_us = micros();
      const bool mag_rate_due =
          static_cast<uint32_t>(mag_now_us - s_mag_last_attempt_us) >= 100000U;
      if (g_heading_aid_mode == 0U && mag_present && mag_sample.seq != 0U &&
          mag_rate_due)
      {
        bool gnss_course_valid = false;
        float gnss_course_rad = 0.0f;
        float speed_mps = 0.0f;
        if (gnss_instant_valid_for_nav && has_gnss_epoch)
        {
          speed_mps = std::hypot(gnss_epoch.north_vel_mps,
                                 gnss_epoch.east_vel_mps);
          if (std::isfinite(speed_mps) && speed_mps > 1.0f)
          {
            gnss_course_rad =
                std::atan2(gnss_epoch.east_vel_mps, gnss_epoch.north_vel_mps);
            gnss_course_valid = std::isfinite(gnss_course_rad);
          }
        }
        const Eigen::Vector3f mag_body(mag_sample.x, mag_sample.y,
                                       mag_sample.z);
        bfs::MagGateInput gate_input;
        gate_input.mag_body_ut = mag_body;
        gate_input.sample_timestamp_us = mag_sample.timestamp_us;
        gate_input.now_us = mag_now_us;
        gate_input.sample_seq = mag_sample.seq;
        gate_input.last_consumed_seq = s_mag_last_consumed_seq;
        gate_input.cal_valid = mag_cal_valid;
        gate_input.ref = &bfs::kDefaultMagReference;
        gate_input.c_n_b = bfs::quat2dcm(nav_ekf.quat());
        gate_input.gnss_course_valid = gnss_course_valid;
        gate_input.gnss_course_rad = gnss_course_rad;
        gate_input.estimated_yaw_rad = nav_ekf.yaw_rad();
        gate_input.speed_mps = speed_mps;
        gate_input.tilt_sigma_rad = nav_ekf.tilt_sigma_rad();
        gate_input.throttle_pct = throttlePercent;
        const bfs::MagGateResult gate = s_mag_gate.Evaluate(gate_input);
        s_mag_last_attempt_us = mag_now_us;
        mag_status.norm_ut = gate.norm_ut;
        mag_status.norm_dev_ut = gate.norm_dev_ut;
        mag_status.dip_deg = bfs::magref::radToDeg(gate.dip_rad);
        mag_status.dip_dev_deg = bfs::magref::radToDeg(gate.dip_dev_rad);
        mag_status.heading_error_deg =
            bfs::magref::radToDeg(gate.heading_error_rad);
        mag_status.track_dev_deg =
            bfs::magref::radToDeg(gate.track_dev_rad);
        if (!gate.pass)
        {
          mag_status.last_gate_reason = static_cast<uint8_t>(gate.reason);
          ++mag_status.gate_rejected;
          aid_tracker.MarkAvailable(bfs::AidSource::Mag, false);
        }
        else
        {
          // 三轴磁矢量只观测 2 个姿态自由度（绕磁场方向的滚转不可观）；
          // 仅当加速度方向仍可信时先做重力方向辅助，形成“重力+磁”双矢量
          // 姿态观测，避免磁更新沿不可观方向把 roll/pitch 慢慢拉走。
          // 大水平加速度/大角速率时跳过，防止把线加速度误当重力方向。
          if (!is_static_confirmed)
          {
            const Eigen::Vector3f gravity_dir_body =
                bfs::quat2dcm(nav_ekf.quat()) *
                (-Eigen::Vector3f::UnitZ());
            const float accel_norm_safe =
                std::fmax(imu_accel_mps2.norm(), 1.0e-3f);
            const float dot = std::fmax(
                -1.0f, std::fmin(1.0f, imu_accel_mps2.dot(gravity_dir_body) /
                                          accel_norm_safe));
            const float gravity_dir_error_rad = std::acos(dot);
            if (accel_norm_error < 0.5f && angular_rate_quiet &&
                gravity_dir_error_rad < 15.0f * 0.017453292f)
            {
              const bfs::MeasurementUpdateResult grav_result =
                  nav_ekf.MeasurementUpdateGravityDetailed(imu_accel_mps2,
                                                           0.05f);
              aid_tracker.RecordAttempt(bfs::AidSource::Gravity, grav_result);
            }
          }

          // 无论 EKF 是否融合，本样本只尝试一次，避免 NIS 拒绝后逐帧重试。
          s_mag_last_consumed_seq = mag_sample.seq;
          const Eigen::Vector3f mag_ref_field(
              bfs::kDefaultMagReference.field_ut[0],
              bfs::kDefaultMagReference.field_ut[1],
              bfs::kDefaultMagReference.field_ut[2]);
          const float yaw_before_rad = nav_ekf.yaw_rad();
          const float roll_before_rad = nav_ekf.roll_rad();
          const float pitch_before_rad = nav_ekf.pitch_rad();
          bool mag_update_allowed = true;
#ifdef BFS_MAG_YAW_GYRO_GATE
          // 物理约束：两次磁更新之间真实航向变化不可能远大于陀螺积分。
          // 磁扰造成的 yaw 翻转通常伴随很小的陀螺 z；用 2° 地板 + 3 倍
          // 陀螺积分作为单次修正上限，既挡住翻转、又允许正常转弯收敛。
          {
            constexpr float kMagNominalDtS = 0.10f;
            const float gyro_yaw_step_rad =
                std::fabs(imu_gyro_radps.z()) * kMagNominalDtS;
            nav_ekf.mag_yaw_jump_limit_rad(
                0.035f + 3.0f * gyro_yaw_step_rad);
          }
#endif
#ifdef BFS_MAG_YAW_MULTIFRAME_GATE
          // 多帧一致性：单帧限幅只允许 2°，但记录“期望修正是否持续超限”。
          // 连续 5 帧（约 0.5 s）都要求大修正 → 磁矢量受扰，暂停磁融合 3 s；
          // 正常收敛的期望修正会随航向接近而减小，不会持续超限。
          static int s_mag_clamp_streak = 0;
          static uint32_t s_mag_untrusted_until_ms = 0;
          const uint32_t now_ms = millis();
          if (s_mag_untrusted_until_ms != 0 &&
              static_cast<int32_t>(now_ms - s_mag_untrusted_until_ms) < 0)
          {
            mag_update_allowed = false;
          }
          else
          {
            s_mag_untrusted_until_ms = 0;
            nav_ekf.mag_yaw_jump_limit_rad(0.035f);
          }
#endif
          bfs::MeasurementUpdateResult result;
          if (mag_update_allowed)
          {
            result = nav_ekf.MeasurementUpdateMagVectorDetailed(
                mag_body, mag_ref_field, gate.noise_ut);
          }
          else
          {
            result.input_valid = true;
            result.attempted = true;
          }
#ifdef BFS_MAG_YAW_MULTIFRAME_GATE
          if (result.fused && result.yaw_correction_clamped)
          {
            if (++s_mag_clamp_streak >= 5)
            {
              s_mag_untrusted_until_ms = now_ms + 3000u;
              s_mag_clamp_streak = 0;
            }
          }
          else if (result.fused)
          {
            s_mag_clamp_streak = 0;
          }
#endif
          mag_status.last_kick_deg = bfs::magref::radToDeg(bfs::magWrapPi(
              nav_ekf.yaw_rad() - yaw_before_rad));
          mag_status.last_kick_rp_deg = bfs::magref::radToDeg(std::fmax(
              std::fabs(bfs::magWrapPi(nav_ekf.roll_rad() - roll_before_rad)),
              std::fabs(bfs::magWrapPi(nav_ekf.pitch_rad() - pitch_before_rad))));
          aid_tracker.RecordAttempt(bfs::AidSource::Mag, result);
          if (result.fused)
          {
            ++mag_status.fused;
            mag_status.last_gate_reason =
                static_cast<uint8_t>(bfs::MagGateReason::None);
          }
          else
          {
            ++mag_status.ekf_rejected;
            mag_status.last_gate_reason =
                static_cast<uint8_t>(bfs::MagGateReason::EkfRejected);
          }
        }
      }
      else if (!mag_present || mag_sample.seq == 0U)
      {
        mag_status.last_gate_reason =
            static_cast<uint8_t>(bfs::MagGateReason::Stale);
        aid_tracker.MarkUnavailable(bfs::AidSource::Mag);
      }
    }

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

      // ★ 2026-08-13 静态门控（仿真实证 F1）：水平巡航恒速时 |a|≈g、|ω|≈0、
      //   Δa≈0，静止检测器会误确认静止 → 下面把 GNSS 速度强制为 0 → 6 维联合
      //   更新 NIS 爆炸 → 位置一并被拒 → 位置纯惯导漂移（仿真 120s 漂 330m）。
      //   修复：GNSS 速度模长超过门限时强制退出静止（仅退出不进入——非对称，
      //   避免 GNSS 噪声妨碍真静止确认）。
      //   理论依据：秦永元、张洪钺、汪叔华《卡尔曼滤波与组合导航原理（第3版）》，
      //   西北工业大学出版社 2015，§8.2.3 p.291：间接法滤波的阻尼必须利用来自
      //   其他导航系统的外来信息，不能用自身信息；§7.3.1 p.255：系统级 FDI 用
      //   χ² 检验，检测应与故障源独立。这里的 GNSS 速度就是推翻“静止”假设的
      //   外来信息。
      //   2026-10 P0-3 仿真 SC9b：1.5 m/s 门在 1.0 m/s 慢速巡航时不触发，静止
      //   确认把 GNSS 速度强制 0，失联 40 s 位置误差 51.6 m；门限降到 0.8 m/s
      //   + 外部参照门控后 SC9b 位置 RMSE 25.4→4.19 m（5 种子，
      //   sim-data/opt/upgrade_P0_guard.json）。0.8 约为典型 GNSS 速度噪声的
      //   3–5 倍；真静止偶发越门只会短暂退出，100 ms 后可重新确认。
      if (is_static_confirmed)
      {
        const float gnss_speed_mps =
            std::sqrt(gnss_epoch.north_vel_mps * gnss_epoch.north_vel_mps +
                      gnss_epoch.east_vel_mps * gnss_epoch.east_vel_mps);
        if (gnss_speed_mps > 0.8f)
        {
          Icm45686StaticDetectorReset(static_det);
          is_static_confirmed = false;
        }
      }

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
#if BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE
        {
          Eigen::Vector3d origin_lla_auto;
          origin_lla_auto << origin_lat_rad, origin_lon_rad, origin_alt_m;
          const Eigen::Vector3d meas_ned_auto =
              bfs::lla2ned(meas_lla, origin_lla_auto, bfs::AngPosUnit::RAD);
          // 批处理时间戳用“投递时刻”，量测值仍是该 age 对应的旧值；
          // 这样 Fit 的 p_ref(t-tau) 模型才成立。
          const double t_deliver_auto =
              static_cast<double>(millis()) * 1.0e-3;
          s_gnss_delay_auto.PushGnss(t_deliver_auto, meas_ned_auto,
                                     meas_vel_ned.cast<double>());
        }
#endif
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

      // ★ 2026-10 P1-5：按飞行相位启用“航向 consider”。
      //   过渡/大倾斜（|pitch| 或 |roll| > 0.5 rad ≈ 28.6°）且水平速度 >3 m/s 时，
      //   F(v,β) 的比力方向变化大，GNSS 速度新息经 P(8,v) 会把航向拉偏；
      //   此时把 GNSS 更新的航向行清零（速度/位置/水平姿态仍修正）。
      //   悬停（pitch≈90°）速度<3 m/s 不触发；巡航小坡度不触发。
      //   理论依据：notes/06 §6；秦永元《卡尔曼滤波与组合导航原理（第3版）》
      //   §8.2.3 p.291：间接法阻尼必须利用外来信息，航向弱可观时不能靠速度新息“造”观测。
      const bool phase_consider_heading =
          (fabsf(nav_ekf.pitch_rad()) > 0.5f ||
           fabsf(nav_ekf.roll_rad()) > 0.5f) &&
          (nav_ekf.ned_vel_mps().norm() > 3.0f) &&
          !is_static_confirmed;
      nav_ekf.set_gnss_consider_heading(phase_consider_heading);

      if (InsGnssShouldReanchor(has_gnss_epoch,
                                gnss_quality_for_reanchor,
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
        s_gnss_consistency.reset();
        publishGnssConsistency(s_gnss_consistency.status());
        s_gnss_last_epoch_us = 0U;
        s_gnss_imu_dv_accum.setZero();
        Serial8.println("[Nav] Re-anchored EKF to first valid GNSS fix.");
      }
      else
      {
        // ── GNSS 位置/速度联合更新（6 维量测）─────────────────────────
        // 公式：z = [p_gnss−p̂; v_gnss−v̂]（NED/LLA 混合残差），
        //       rᵀS⁻¹r ~ χ²(6) 门控，拒绝时退化为 3D 位置/3D 速度子集 χ² 检验
        // 符号：p̂/v̂ 为 EKF 预测，z 为同历元 GNSS 观测；R 由动态权重给出
        // 来源：@ref REF-121 §2.2.1 p.33–34（五方程）、§7.3.1 p.255–259（NIS/FDI），
        //       §8.2.3 p.291（外来信息阻尼）
        // 实现：`MeasurementUpdateDetailed()`（EKF 内：年龄补偿、门控、子集更新）
        // 等价变换：位置残差在 LLA→NED 后与速度同系比较 @ref MOD-002 §2
        // 边界：静止确认时 meas_vel 强制 0（由 GNSS 速度>0.8 先推翻静止）；
        //       过渡/大倾斜+速度>3 m/s 时启用 consider_heading（P1-5，按相位）
        // 数值证据：test_host/test_ekf_15state.cpp T9–T11（子集 FDI）
        // ──────────────────────────────────────────────────────────────
        const bfs::MeasurementUpdateResult result =
            nav_ekf.MeasurementUpdateDetailed(meas_vel_ned, meas_lla,
                                              nav_update_dt_s,
                                              gnss_measurement_age_s);
        gnss_status.last_test_ratio = result.test_ratio;
        aid_tracker.RecordAttempt(bfs::AidSource::Gnss, result);
        if (result.reacquired)
        {
          // 长失联后连续不可信 GNSS 触发的位置/速度硬重锚，单独计入 reset_count，
          // 便于 DBG 区分“普通融合”和“恢复重锚”。
          aid_tracker.RecordReset(bfs::AidSource::Gnss);
          s_gnss_consistency.reset();
          publishGnssConsistency(s_gnss_consistency.status());
          s_gnss_last_epoch_us = 0U;
          s_gnss_imu_dv_accum.setZero();
        }
      }
    }
    else if (perform_measurement_update && gnss_downgrade_quality_for_nav &&
             !gnss_status.consistency_suspect)
    {
      // 边缘 GNSS 解只融合速度，位置噪声放大为近似关闭，避免弱位置观测拉偏原点。
      gnss_status.downgrade_count++;
      applyDowngradedGnssUpdate(gnss_epoch, nav_update_dt_s,
                                gnss_measurement_age_s);
    }

    // consider 只对本次 GNSS 更新生效；下一帧若相位不满足，恢复原行为。
    nav_ekf.set_gnss_consider_heading(false);

    // 无 GNSS 且非静止时，用光流速度约束 EKF 水平速度。
    // 这是室内无 GPS 悬停/定点的主要速度观测源；EKF 积分得到相对位置。
    if (!gnss_instant_valid_for_nav)
    {
      handleIndoorOpticalFlowVelocityFusion();
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

  if (perform_measurement_update && gnss_instant_valid_for_nav)
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

#if BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE
  {
    const uint32_t now_ms_auto = millis();
    const Eigen::Vector3d p_nav(relative_north, relative_east, relative_down);
    const Eigen::Vector3d v_nav(fused_north_vel, fused_east_vel,
                                INS_GNSS_Packet.velocity_down);
    if (!s_ins_ref.valid()) {
      s_ins_ref.Reset(p_nav, v_nav);
    }
    s_ins_ref.Update(imu_accel_mps2, nav_ekf.quat(),
                     nav_ekf.accel_bias_mps2(), dt_s);
    s_gnss_delay_auto.PushReference(
        static_cast<double>(now_ms_auto) * 1.0e-3,
        s_ins_ref.pos(), s_ins_ref.vel());
    if ((now_ms_auto - s_gnss_delay_auto_last_ms) >= 1000U)
    {
      s_gnss_delay_auto_last_ms = now_ms_auto;
      if (s_gnss_delay_auto.TryEstimate())
      {
        const auto &r = s_gnss_delay_auto.last_result();
        gnss_status.auto_delay_last_ms = now_ms_auto;
        gnss_status.auto_delay_valid = r.valid;
        gnss_status.auto_delay_tau_s = r.tau_s;
        gnss_status.auto_delay_residual_mps = r.residual_mps;
        gnss_status.auto_delay_sharpness = r.sharpness;
      }
    }
  }
#endif

  // 温度-零偏诊断：只在静止 + GNSS 健康时采样，不参与控制。
  {
    const auto tr = s_bias_temp_observer.Update(
        imu_die_temp_c, nav_ekf.gyro_bias_radps(),
        is_static_confirmed, gnss_instant_valid_for_nav, nav_update_dt_s);
    gnc_tel.bias_temp_slope_dps_c = tr.gyro_slope_radps_per_c(2) * 57.2958f;
    gnc_tel.bias_temp_span_c = tr.temp_span_c;
    gnc_tel.bias_temp_confidence = tr.confidence;
    // 温度-零偏表只采点，不参与补偿；温度点重复时覆盖旧点。
    if (is_static_confirmed && gnss_instant_valid_for_nav &&
        tr.confidence > 0.2f &&
        static_cast<int32_t>(millis() - s_bias_temp_table_last_ms) >= 5000)
    {
      s_bias_temp_table_last_ms = millis();
      s_bias_temp_table.AddPoint(imu_die_temp_c,
                                 nav_ekf.gyro_bias_radps(),
                                 nav_ekf.accel_bias_mps2());
    }
  }

  // 零偏跳变监控：只告警，不直接回滚，避免把一次坏更新当成真实零偏。
  if (!s_bias_jump_monitor.initialized()) {
    s_bias_jump_monitor.Reset(nav_ekf.accel_bias_mps2(),
                              nav_ekf.gyro_bias_radps());
  }
  {
    const auto bj = s_bias_jump_monitor.Update(
        nav_ekf.accel_bias_mps2(), nav_ekf.gyro_bias_radps(),
        nav_update_dt_s);
    gnc_tel.bias_jump_accel_delta = bj.accel_delta;
    gnc_tel.bias_jump_gyro_delta = bj.gyro_delta;
    gnc_tel.bias_jump_count = bj.jump_count;
    gnc_tel.bias_slow_drift = bj.accel_slow_drift || bj.gyro_slow_drift;
    gnc_tel.bias_drift_accel_rate = bj.accel_drift_rate;
    gnc_tel.bias_drift_gyro_rate = bj.gyro_drift_rate;
    if (bj.accel_jump || bj.gyro_jump) {
      s_bias_jump_hold_until_ms = millis() + 5000U;
    }
    gnc_tel.bias_jump_hold =
        static_cast<int32_t>(millis() - s_bias_jump_hold_until_ms) < 0;
  }

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

  // GNSS 失联 → 重捕获：请求位置保持把当前点重设为目标，避免失联期间
  // 累积的位置误差在重捕获后变成大姿态指令（seed7 的 runaway 触发条件）。
  {
    static bool s_gnss_was_out = false;
    const float gnss_age_s = aid_tracker.status(bfs::AidSource::Gnss).age_s;
    g_gnss_outage_age_s = (std::isfinite(gnss_age_s) && gnss_age_s < 1.0e5f) ? gnss_age_s : 0.0f;
    if (std::isfinite(gnss_age_s) && gnss_age_s > 0.5f)
    {
      s_gnss_was_out = true;
    }
    else if (s_gnss_was_out && gnss_age_s >= 0.0f && gnss_age_s < 0.2f)
    {
      g_pos_hold_reanchor = true;
      s_gnss_was_out = false;
    }
  }

  // 绝对航向辅助可用性 → 位置保持门控（2026-10）：
  // 磁矢量只有 EKF **成功融合**才把 age 清零；门控拒绝/无样本都会让年龄增长。
  // YawGsf 是“GNSS 航迹 − 光流侧滑”的航向观测，只在有速度时成立，不能替代
  // 悬停磁航向。失效时上层 AUTO_POSITION/GUIDED 必须降级为定高，否则位置
  // 保持会追逐不可观航向估计，形成姿态-位置耦合发散（AGENTS.md EKF-in-loop）。
  {
    bfs::HeadingAidingInput heading_in;
    heading_in.mag_age_s = aid_tracker.status(bfs::AidSource::Mag).age_s;
    heading_in.yaw_gsf_age_s =
        aid_tracker.status(bfs::AidSource::YawGsf).age_s;
    const Eigen::Vector3f vel_ned = nav_ekf.ned_vel_mps();
    heading_in.horizontal_speed_mps =
        std::hypot(vel_ned(0), vel_ned(1));
    heading_in.heading_mode = g_heading_aid_mode;
    // 离地判据：解锁且油门已出力。SIL 地面会滑跑/自动油门，长地面等待
    // 场景不代表实机；窗口语义由门控单元测试 T9b/T9c 锁定。
    heading_in.airborne = g_is_unlocked && throttlePercent >= 3.0f;
    heading_in.calibrated = g_imu_static_calibrated;

    heading_in.yaw_sigma_deg = nav_ekf.yaw_sigma_rad() * 57.2957795f;
    {
      // 无磁一致性：EKF 航向变化率应有陀螺 z 支撑；纯平移时估计器自漂
      // 会表现为“航向在变、陀螺没动”。仅用于无磁模式门控。
      static float s_prev_drift_heading_rad = 0.0f;
      static bool s_prev_drift_heading_valid = false;
      const float heading_now_rad = AHRS_Packet.Heading;
      float heading_rate_dps = 0.0f;
      if (s_prev_drift_heading_valid && nav_update_dt_s > 1.0e-4f)
      {
        float d = heading_now_rad - s_prev_drift_heading_rad;
        while (d > 3.14159265f) d -= 6.2831853f;
        while (d < -3.14159265f) d += 6.2831853f;
        heading_rate_dps = d * 57.2957795f / nav_update_dt_s;
      }
      s_prev_drift_heading_rad = heading_now_rad;
      s_prev_drift_heading_valid = true;
      heading_in.yaw_drift_dps =
          heading_rate_dps - current_omega_dps_body_filtered.z;
    }
    if (gnss_instant_valid_for_nav && has_gnss_epoch)
    {
      const float gnss_speed_mps =
          std::hypot(gnss_epoch.north_vel_mps, gnss_epoch.east_vel_mps);
      heading_in.gnss_motion_valid =
          std::isfinite(gnss_speed_mps) && gnss_speed_mps > 2.0f;
    }
    // 纯平移（风推、无倾斜）不能观测机头航向；只有“速度+倾斜”运动激励
    // 才允许无磁位置保持。倾斜门限 5° 高于配平抖动、低于正常前飞姿态。
    {
      const float tilt_deg =
          std::fmax(std::fabs(nav_ekf.roll_rad()),
                    std::fabs(nav_ekf.pitch_rad())) *
          57.2957795f;
      heading_in.motion_excitation_valid =
          heading_in.gnss_motion_valid && tilt_deg > 5.0f;
    }
#ifdef BFS_YAW_SIGMA_GATE
    // 航向协方差是比“事后速率”更早的不可观信号：P(8,8) 持续大时，
    // 位置保持不应继续依赖该航向估计。
    heading_in.monitor_yaw_sigma = true;
    heading_in.yaw_sigma_deg = nav_ekf.yaw_sigma_rad() * 57.2957795f;
#endif
#ifdef BFS_YAW_OSC_GATE
    // 监测航向估计自身的跳变速率（不是陀螺 z）：seed7 的 yaw 翻转来自
    // 磁融合修正，实际机体角速度很小；只监测陀螺会漏掉这种失效。
    static float s_prev_heading_rad = 0.0f;
    static bool s_prev_heading_valid = false;
    const float heading_now_rad = AHRS_Packet.Heading;
    float heading_rate_dps = 0.0f;
    if (s_prev_heading_valid && nav_update_dt_s > 1.0e-4f)
    {
      float d = heading_now_rad - s_prev_heading_rad;
      while (d > 3.14159265f) d -= 6.2831853f;
      while (d < -3.14159265f) d += 6.2831853f;
      heading_rate_dps = d * 57.2957795f / nav_update_dt_s;
    }
    s_prev_heading_rad = heading_now_rad;
    s_prev_heading_valid = true;
    heading_in.monitor_yaw_oscillation = true;
    heading_in.yaw_rate_dps = heading_rate_dps;
#endif
    heading_in.dt_s = nav_update_dt_s;
    const bfs::HeadingAidingOutput heading_out =
        s_heading_aiding_gate.update(heading_in);
    g_heading_aiding_valid = heading_out.source_fresh;
    g_heading_aiding_age_s =
        (heading_out.best_age_s < 1.0e6f) ? heading_out.best_age_s : -1.0f;
    g_heading_hold_allowed = heading_out.heading_hold_allowed;
    g_pos_hold_degraded = heading_out.position_hold_degraded;
    g_nomag_hold_left_s = (g_heading_aid_mode != 0U)
                              ? std::fmax(0.0f, heading_out.valid_for_s)
                              : -1.0f;
    // 无磁离地：限制零偏 z。GNSS 位置/速度更新会通过姿态-速度耦合把机体
    // 真实偏航率误估成陀螺零偏，导致 EKF 航向冻结而真值在转（SIL 风扰
    // 消融：GNSS 正常时 bias_z 40 s 吸收到 0.41 rad/s，断 GNSS 时保持 0.008）。
    // 航向门控降级时同样钳制：磁模式机动 + GNSS 失联后，GNSS 更新经
    // 姿态-速度耦合把真实偏航率吸收成 bias_z；SIL 200 Hz 实测降级后
    // bias_z 达 -37 rad/s，EKF 航向与真值反向并进入 179° 滚转尾部。
    // 航向源陈旧 0.5 s 即提前钳制，不等 position_hold_degraded 的 2 s。
    const bool heading_source_stale = g_heading_aiding_age_s > 0.5f;
    const bool heading_degraded = (g_heading_aid_mode != 0U ||
                                   g_pos_hold_degraded ||
                                   heading_source_stale);
    if (heading_in.airborne)
    {
      // 正常磁模式也硬限 bias_z：风扰下真值 yaw 可先旋转、航向仍新鲜，
      // 真实偏航率被吸到 ~96 deg/s（wind_S_10_s10 trace）；0.05 rad/s
      // 保留正常 ~0.039 零偏，降级时仍用无磁验证的 0.01。
      nav_ekf.ClampGyroBiasZ(heading_degraded ? g_nomag_bias_z_max_radps
                                              : 0.05f);
    }
  }
}

/**
 * @brief 估算“向上为正”的垂直速度：优先 EKF 融合值，否则由高度差分校准
 * @details 输入：`INS_GNSS_Packet.velocity_down`（GNSS 输出有效时）或激光/气压
 *          高度及其新鲜度；处理：GNSS 有效直接用 −v_D 并复位差分基准，否则按
 *          “激光优先（测距通道新鲜度）、气压兜底”做一阶差分 + 低通；
 *          输出：`estimated_velocity` [m/s]（向上为正）。
 *          不变量：切换数据源时清对应首次标志，避免跨源差分跳变。
 * @formula v_up = −v_D；无 GNSS 时 v_up ≈ Δh/Δt（h 向上为正），再经一阶低通
 * @ref REF-120 §7.5 p.190–191（高度通道需外部高度反馈才稳定）；
 *          NED z 向下与“向上为正”的符号关系 @ref MOD-002 §5.1
 * @frame NED（v_D 向下为正）→ 向上为正
 * @units m、m/s、µs
 * @note **测距用 `flow_data.last_range_update_ms`，不是 `last_update_ms`**：
 *       光流速度与测距是两条独立新鲜度通道，测距帧不得刷新速度通道时间戳
 *       （2026-08-21 踩坑，见 TandemVec_FCS/AGENTS.md）。
 * @see `handleNavigationSystem()` 气压更新、`flow_data.last_range_update_ms`
 */
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

    // 条件2: 检查激光测距是否有效、在合理范围内且数据新鲜
    // ★ 2026-09-01 修复：测距消费用测距通道新鲜度 last_range_update_ms
    //   （原误用速度通道 last_update_ms——流活距死时陈旧距离会冒充新鲜）
    if (flow_data.is_range_valid && flow_data.distance_m > 0.08f &&
        (millis() - flow_data.last_range_update_ms) <= kFlowDataStaleTimeoutMs)
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

/**
 * @brief 独立二状态垂直 KF 任务：由 IMU 天向加速度预测、气压高度更新
 * @details 输入：`IMU_Packet` 比力、`AHRS_Packet` 四元数、`baro_altitude`；
 *          处理：比力旋转到 NED → 减重力并翻转为“向上为正” → `predict()` →
 *          气压 50 Hz 降采样 `update()`；输出：`vfk_height`/`vfk_velocity`。
 *          该滤波器与 EKF 并行独立运行，只作遥测对照，不写控制量。
 * @formula a_up = −(C_n^b f^b + g_n)_z，其中 g_n=(0,0,+g)（NED 向下为正）
 * @ref REF-120 §7.4 (7.4.6) p.189（比力方程）；KF 五方程与 Joseph
 *          形式 @ref REF-121 §2.2.1 p.33–34、§2.2.4 p.42–43
 * @frame 机体系 FRD 比力 → NED → 向上为正的标量
 * @units m/s²、m、m/s、s
 * @note 激光分支已被 `if (0)` 暂时禁用，实际只用气压；`LASER_NOISE_STD`
 *       仍在编译期保留。不要用本滤波器输出替换 EKF 的 estimated_height。
 * @see `VerticalKF_2State.h`、`updateEstimatedVerticalVelocity()`
 */
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

/**
 * @brief 独立水平双轴 KF 任务：IMU 水平加速度预测、光流速度更新
 * @details 输入：`IMU_Packet` 比力、`AHRS_Packet` 四元数与航向、光流机体速度；
 *          处理：比力旋转到 NED 取北/东分量 → 两轴 `predict()` → 每个新光流
 *          样本用航向做平面旋转后 `update()`；输出：`fused_north/east_pos/vel`。
 *          与 EKF 并行独立，只作对照/后备，不覆盖 EKF 的 relative_*。
 * @formula v^n = R(ψ)·v^b（仅航向平面旋转）；观测 H=[0,1]（速度量测）
 * @ref REF-121 §8.2.3 p.291（光流速度作为外部速度阻尼）；
 *          平面旋转的 DCM 语义 @ref MOD-002 §2
 * @frame 机体系 FRD（水平两轴）→ NED
 * @units m、m/s、m/s²、rad
 * @deviation 本函数只用航向 ψ 做二维旋转（`bodyToNed`），忽略俯仰/滚转；
 *          机体倾斜时水平投影会含 cos(tilt) 尺度差，EKF 路径用全姿态四元数
 *          旋转（`handleIndoorOpticalFlowVelocityFusion()`）。两者结果不严格一致，
 *          属已知差异（仅影响遥测对照/后备，不影响控制）。
 * @note 判重条件用 `last_update_ms` 且不清 `is_flow_valid`：该标志被多个消费者
 *       共享，清标志会让同帧其它消费者丢数据（2026-08-21 修复）。
 * @see `HorizontalKF.h`、`handleIndoorOpticalFlowVelocityFusion()`
 */
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
  // ★ 2026-08-21 判重改用 last_update_ms 时间戳，不再清 is_flow_valid——
  //   该标志由传感器任务写入、多个消费者共享（EKF 光流融合/双矢量航向），
  //   清标志会让同帧其他消费者丢数据。
  static uint32_t s_last_consumed_flow_update_ms = 0;
  if (flow_data.is_flow_valid && estimated_height > 0.05f &&
      (millis() - flow_data.last_update_ms) <= kFlowDataStaleTimeoutMs &&
      flow_data.last_update_ms != s_last_consumed_flow_update_ms)
  {
    s_last_consumed_flow_update_ms = flow_data.last_update_ms;

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
  // 但不再无条件替换 EKF 的 relative 和 INS 速度。历史实现曾按
  // “无 GNSS 输出有效”分支整体覆盖，已因漂移问题移除，勿恢复。
}
