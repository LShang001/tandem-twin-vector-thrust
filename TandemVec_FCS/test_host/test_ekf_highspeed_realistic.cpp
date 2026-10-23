/**
 * @file    test_ekf_highspeed_realistic.cpp
 * @module  固件/宿主研究（fw-tests：标称传感器精度下的高速导航仿真台）
 * @brief   用 ICM-42688/UBX PVT 标称噪声跑 EKF 闭环仿真，输出延时/振动/故障组合指标
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h`，以及
 *          `include/ins_vibration_adaptive.h`、`include/ins_gnss_delay_auto.h`
 *          （`--auto_delay` / 振动自适应开关）。
 *          传感器模型（按型号标称量级，非实机标定）：ICM-42688 陀螺噪声密度
 *          0.0035 dps/√Hz、零偏 0.5 dps；加计噪声密度 70 µg/√Hz、零偏 20 mg；
 *          UBX PVT 水平 1.5 m (1σ)、速度 0.05 m/s (1σ)、10 Hz；链路延迟可配置。
 *          输入：argv 为速度/延时/各噪声/种子/机动/振动 g/模式/频率/失锁窗口/
 *          杆臂/纬度抖动/auto_delay/尺度误差/安装失准/轴零偏/野值比例/
 *          加计门控/阶跃与缓漂参数等（默认值见 main 开头）。
 *          处理：60 s 闭环仿真，比较补偿前后的位置/速度 RMSE；
 *          输出：`age-comp: … pos3d=… vel=… improve=…` 等机器可读行。
 *          运行：见 run_all.sh 的 hsreal 目标（-DEKF_HOST_REGRESSION + convang 源）。
 * @theory  延迟量测回放 + 振动自适应 Q；判据为 3D 位置误差与速度 RMSE。
 *          @ref REF-121 §2.2.1 p.33、@ref REF-121 §7.3.1 p.255（NIS）
 * @frame   NED（南/东/地位置与速度）
 * @units   速度 m/s，延时 s，位置 m，噪声 m 或 m/s，振动 g，角速率 dps，杆臂 m
 * @note    ★ 历史坑：本文件的命令行参数是 1-based 顺序映射，
 *          argv[21] 是 auto_delay、argv[22] 是 accel_scale；曾因错位一位导致整批
 *          MC 结果（40 km 发散、硬重锚等）被撤回，见 AGENTS.md（EKF 在线零偏可靠性）。
 *          改参数顺序会静默改变所有历史 CSV 的语义。
 * @test    S1 速度/延时扫描；S2 振动矩阵；S3 失锁与恢复；S4 杆臂补偿；
 *          S5 阶跃/缓漂零偏；S6 野值注入；S7 auto_delay 闭环；S8 参数扫频
 * @see     TandemVec-EKF-Report/卫惯组合导航-延时补偿-高速精度-2026-10-22.md、
 *          TandemVec-EKF-Report/EKF在线零偏估计可靠性-2026-10-23.md
 */
// ============================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "ins_vibration_adaptive.h"
#include "ins_gnss_delay_auto.h"
#include "ins_vibration_adaptive.h"
#include "ins_gnss_delay_auto.h"
#include "transforms.h"

using bfs::Ekf15State;

namespace {
constexpr float kG = 9.79f;
constexpr double kPi = 3.14159265358979323846;
constexpr float kDt = 0.005f;
constexpr float kDurationS = 60.0f;
const Eigen::Vector3d kHome(31.2304 * kPi / 180.0,
                            121.4737 * kPi / 180.0, 50.0);

// ICM-42688 标称量级（连续时间噪声密度 -> 200 Hz 样本标准差）
constexpr float kGyroNoiseDensityDpsPerSqrtHz = 0.0035f;
constexpr float kAccelNoiseDensityUgPerSqrtHz = 70.0f;
constexpr float kGyroBiasDps = 0.5f;
constexpr float kAccelBiasMg = 20.0f;

struct Sample { double t; double deliver_t; Eigen::Vector3f vel; Eigen::Vector3d lla; };
struct Result {
  double pos_base, pos_comp, vel_base, vel_comp;
  double pos3d_base, pos3d_comp;
  double est_delay_s;
  int fused, rejected;
  Eigen::Vector3f accel_bias;
  Eigen::Vector3f gyro_bias;
};

double horizError(const Ekf15State &ekf, const Eigen::Vector3d &truth) {
  const Eigen::Vector3d lla(ekf.lat_rad(), ekf.lon_rad(), ekf.alt_m());
  const Eigen::Vector3d p = bfs::lla2ned(lla, kHome);
  return std::hypot(p(0) - truth(0), p(1) - truth(1));
}

double pos3dError(const Ekf15State &ekf, const Eigen::Vector3d &truth) {
  const Eigen::Vector3d lla(ekf.lat_rad(), ekf.lon_rad(), ekf.alt_m());
  const Eigen::Vector3d p = bfs::lla2ned(lla, kHome);
  return (p - truth).norm();
}

Eigen::Vector3d makeImuLla(const Eigen::Vector3d &gnss_lla,
                            const Eigen::Quaternionf &q,
                            float lever_m, bool compensate) {
  if (!compensate || std::fabs(lever_m) < 1e-6f) return gnss_lla;
  Eigen::Vector3d p = bfs::lla2ned(gnss_lla, kHome);
  const Eigen::Vector3f lever_ned = q * Eigen::Vector3f(lever_m, 0.0f, 0.0f);
  p -= lever_ned.cast<double>();
  return bfs::ned2lla(p, kHome);
}

Result run(float speed_mps, float latency_s, unsigned seed, int maneuver,
           float ekf_accel_std, float ekf_gyro_std,
           float ekf_gyro_markov, float ekf_gnss_pos, float ekf_gnss_vel,
           float ekf_accel_markov, float vib_g, bool adaptive_q,
           int vib_mode, float gyro_vib_dps, float vib_hz,
           float outage_start_s, float outage_end_s, float lever_m,
           bool lever_comp, float latency_jitter_s, bool auto_delay,
           float accel_scale, float gyro_scale, float misalign,
           float accel_bias_y, float accel_bias_z, float gnss_outlier_frac,
           bool accel_bias_gate, float bias_step_s, float bias_step_accel,
           float bias_step_gyro_dps, float bias_drift_accel,
           float bias_drift_gyro_dps, bool gnss_bias_freeze, bool trace,
           bool post_outage_boost, bool bias_reanchor,
           bool reset_on_recovery, bool soft_velocity) {
  std::mt19937 rng(seed);
  // 200 Hz 样本噪声：密度 × sqrt(带宽 100 Hz)
  const float gyro_sample_sigma =
      kGyroNoiseDensityDpsPerSqrtHz * (float)M_PI / 180.0f * std::sqrt(100.0f);
  const float accel_sample_sigma =
      kAccelNoiseDensityUgPerSqrtHz * 1e-6f * kG * std::sqrt(100.0f);
  const float gyro_bias0 = kGyroBiasDps * (float)M_PI / 180.0f;
  const float accel_bias0 = kAccelBiasMg * 0.001f * kG;
  std::normal_distribution<float> n_gyro(0.0f, gyro_sample_sigma);
  std::normal_distribution<float> n_accel(0.0f, accel_sample_sigma);
  std::normal_distribution<float> n_pos(0.0f, 1.5f);
  std::normal_distribution<float> n_vel(0.0f, 0.05f);

  Ekf15State base, comp;
  for (Ekf15State *e : {&base, &comp}) {
    e->accel_std_mps2(ekf_accel_std);
    e->accel_markov_bias_std_mps2(ekf_accel_markov);
    e->gyro_std_radps(ekf_gyro_std);
    e->gyro_markov_bias_std_radps(ekf_gyro_markov);
    e->gnss_pos_ne_std_m(ekf_gnss_pos);
    e->gnss_pos_d_std_m(ekf_gnss_pos * 2.0f);
    e->gnss_vel_ne_std_mps(ekf_gnss_vel);
    e->gnss_vel_d_std_mps(ekf_gnss_vel * 2.0f);
    e->init_pos_err_std_m(2.0f);
    e->init_vel_err_std_mps(0.08f);
    e->init_att_err_std_rad(0.08f);
    e->init_heading_err_std_rad(0.2f);
  }
  const Eigen::Vector3f init_accel(0.0f, 0.0f, -kG);
  const Eigen::Vector3f init_gyro = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_mag = Eigen::Vector3f::Zero();
  const Eigen::Vector3f init_vel(speed_mps, 0.0f, 0.0f);
  base.Initialize(init_accel, init_gyro, init_mag, init_vel, kHome);
  comp.Initialize(init_accel, init_gyro, init_mag, init_vel, kHome);

  std::deque<Sample> pending;
  const int steps = static_cast<int>(kDurationS / kDt);
  const int gnss_every = static_cast<int>(1.0f / (10.0f * kDt)); // 10 Hz
  Eigen::Vector3d truth(0.0, 0.0, 0.0);
  Eigen::Vector3f truth_vel(speed_mps, 0.0f, 0.0f);
  std::vector<Eigen::Vector3d> truth_hist(steps + 1);
  std::vector<Eigen::Vector3f> vel_hist(steps + 1);
  float yaw = 0.0f;
  float yaw_rate = 0.0f;

  float a_bias_x = accel_bias0, a_bias_y = 0.0f, a_bias_z = 0.0f;
  float g_bias_x = gyro_bias0, g_bias_y = 0.0f, g_bias_z = 0.0f;

  Result out{0, 0, 0, 0, 0, 0, 0.0, 0, 0};
  double sum_pb = 0, sum_pc = 0, sum_vb = 0, sum_vc = 0;
  double sum_p3b = 0, sum_p3c = 0;
  int count = 0;
  // 自适应 Q：使用固件同款在线振动估计器（高通 RMS + 连续映射）
  InsVibrationEstimator vib_est;
  InsGnssDelayAutoEstimator batch_delay;
  bool reanchor_done = false;
  bool reset_recovery_done = false;
  bool soft_velocity_done = false;
  bool batch_delay_done = false;
  bool batch_delay_ok = false;
  float tau_est = 0.0f;
  a_bias_y = accel_bias_y;
  a_bias_z = accel_bias_z;
  Eigen::Matrix3f M_sensor = Eigen::Matrix3f::Identity();
  M_sensor(0, 1) = misalign;  M_sensor(0, 2) = -misalign;
  M_sensor(1, 0) = -misalign; M_sensor(1, 2) = misalign;
  M_sensor(2, 0) = misalign;  M_sensor(2, 1) = -misalign;
  std::uniform_real_distribution<float> outlier_unif(0.0f, 1.0f);

  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    const bool gnss_outage = (t >= outage_start_s && t < outage_end_s);
    const bool gnss_recovery = (t >= outage_end_s && t < outage_end_s + 10.0);
    // 机动剖面：
    // 0 = 匀速；1 = 纵向加减速；2 = 纵向 + 侧向 + 爬升
    Eigen::Vector3f a_cmd(0.0f, 0.0f, 0.0f);
    yaw_rate = 0.0f;
    if (maneuver == 3) {
      // 90° 航向转弯：10–15 s，无倾斜（只验证航向/杆臂，不冒充协调转弯）
      if (t >= 10.0 && t < 15.0) yaw_rate = 0.30f; // ~17°/s
      yaw += yaw_rate * kDt;
      const float c = std::cos(yaw), sn = std::sin(yaw);
      truth_vel = Eigen::Vector3f(speed_mps * c, speed_mps * sn, 0.0f);
      a_cmd = Eigen::Vector3f(-speed_mps * yaw_rate * sn,
                               speed_mps * yaw_rate * c, 0.0f);
    } else if (maneuver == 1) {
      if (t < 5.0) a_cmd(0) = 2.0f;
      else if (t >= 15.0 && t < 25.0) a_cmd(0) = -2.0f;
    } else if (maneuver == 2) {
      if (t < 5.0) a_cmd(0) = 2.0f;             // 加速
      else if (t >= 10.0 && t < 15.0) a_cmd(1) = 2.0f; // 侧向机动
      else if (t >= 20.0 && t < 25.0) a_cmd(2) = -1.0f; // 爬升
      else if (t >= 25.0 && t < 30.0) a_cmd(0) = -2.0f; // 减速
    }
    if (maneuver != 3) {
      truth_vel += a_cmd * kDt;
    }
    truth += truth_vel.cast<double>() * kDt;
    truth_hist[k] = truth;
    vel_hist[k] = truth_vel;
    batch_delay.PushReference(t, truth, truth_vel.cast<double>());

    // 真实姿态：mode3 仅航向旋转（无倾斜）
    const Eigen::Quaternionf q_true =
        Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    // ICM-42688 白噪声 + 常值零偏
    // 共轴双桨振动：mode0=20 Hz 单频；mode1=40/80/120 Hz 多频谐波
    float vib = 0.0f;
    if (vib_mode == 1) {
      const float w = 2.0f * 3.14159265f * (float)t;
      vib = vib_g * kG * (0.70f * std::sin(w * 40.0f) +
                          0.45f * std::sin(w * 80.0f + 1.1f) +
                          0.25f * std::sin(w * 120.0f + 2.2f));
    } else {
      vib = vib_g * kG * std::sin(2.0f * 3.14159265f * vib_hz * (float)t);
    }
    const Eigen::Vector3f f_ned = a_cmd - Eigen::Vector3f(0.0f, 0.0f, kG);
    Eigen::Vector3f accel = q_true.conjugate() * f_ned;
    accel(0) += vib;
    accel(2) += 0.5f * vib;
    const bool bias_stepped = (bias_step_s > 0.0f && t >= bias_step_s);
    const float drift_t = static_cast<float>(std::max(0.0, t - 20.0));
    const float true_abx = a_bias_x +
        (bias_stepped ? bias_step_accel : 0.0f) +
        drift_t * bias_drift_accel;
    accel(0) += true_abx + n_accel(rng);
    accel(1) += a_bias_y + n_accel(rng);
    accel(2) += a_bias_z + n_accel(rng);
    accel = accel_scale * (M_sensor * accel);
    // 陀螺振动耦合（dps 幅值，转换为 rad/s；随振动级别缩放）
    const float gyro_vib = gyro_vib_dps * (float)M_PI / 180.0f * vib_g *
                           std::sin(2.0f * 3.14159265f * 80.0f * (float)t);
    const Eigen::Vector3f w_ned(0.0f, 0.0f, yaw_rate);
    const Eigen::Vector3f w_body = q_true.conjugate() * w_ned;
    const float true_gbx = g_bias_x +
        (bias_stepped ? bias_step_gyro_dps * (float)M_PI / 180.0f : 0.0f) +
        drift_t * bias_drift_gyro_dps * (float)M_PI / 180.0f;
    Eigen::Vector3f gyro(true_gbx + n_gyro(rng) + 0.3f * gyro_vib + w_body(0),
                         g_bias_y + n_gyro(rng) + 0.2f * gyro_vib + w_body(1),
                         g_bias_z + n_gyro(rng) + gyro_vib + w_body(2));
    gyro = gyro_scale * (M_sensor * gyro);

    if (adaptive_q) {
      const auto ve = vib_est.Update(accel, gyro, kDt);
      // 安全版：只在 [0.15,0.25] 内自适应，绝不高于固定值。
      float q = std::clamp(ve.accel_std_suggest, 0.15f, 0.25f);
      if (gnss_outage || gnss_recovery || latency_s > 0.05f) q = 0.25f;
      if (ekf_gnss_pos > 2.0f) q = 0.25f;
      comp.accel_std_mps2(q);
      base.accel_std_mps2(q);
    }
    if (accel_bias_gate) {
      const bool excited = (std::fabs(accel.norm() - kG) > 0.5f) ||
                           (gyro.norm() > 0.30f);
      const float abq = excited ? 0.010f : 0.001f;
      base.accel_markov_bias_std_mps2(abq);
      comp.accel_markov_bias_std_mps2(abq);
    }
    if (gnss_bias_freeze && gnss_outage) {
      base.accel_markov_bias_std_mps2(0.0001f);
      comp.accel_markov_bias_std_mps2(0.0001f);
    }
    if (post_outage_boost && gnss_recovery) {
      base.accel_markov_bias_std_mps2(0.050f);
      comp.accel_markov_bias_std_mps2(0.050f);
    }
    if (bias_reanchor && gnss_recovery && !reanchor_done) {
      base.ReanchorBiasCovariance(0.10f, 0.010f);
      comp.ReanchorBiasCovariance(0.10f, 0.010f);
      reanchor_done = true;
    }
    if (soft_velocity && gnss_recovery && !soft_velocity_done) {
      comp.ReanchorVelocityCovariance(2.0f);
      soft_velocity_done = true;
    }
    if (reset_on_recovery && gnss_recovery && !reset_recovery_done) {
      comp.ResetPositionVelocityToGnss(truth_vel,
                                      bfs::ned2lla(truth, kHome));
      reset_recovery_done = true;
    }
    base.TimeUpdate(accel, gyro, kDt);
    comp.TimeUpdate(accel, gyro, kDt);
    if (trace && (k % 200 == 0)) {
      const auto ab = comp.accel_bias_mps2();
      const auto gb = comp.gyro_bias_radps();
      const Eigen::Vector3d ep = bfs::lla2ned(comp.lla_rad_m(), kHome) - truth;
      const Eigen::Vector3f ev = comp.ned_vel_mps() - truth_vel;
      std::printf("TRACE t=%.0f ab=%.4f gb_dps=%.4f pos=%.1f vel=%.1f yawerr=%.2f outage=%d\n",
                  t, ab(0), gb(0) * 57.2958f, ep.norm(), ev.norm(),
                  (comp.yaw_rad() - yaw) * 57.2958f, gnss_outage ? 1 : 0);
    }

    if (!gnss_outage && k % gnss_every == 0) {
      Sample s;
      // ★ 真实延迟模型：量测值取 t-tau 时刻的真实轨迹，而不是当前时刻。
      const double t_meas = std::max(0.0, t - latency_s);
      const int j = std::min(steps, static_cast<int>(std::lround(t_meas / kDt)));
      const Eigen::Vector3d p_meas = truth_hist[j];
      const Eigen::Vector3f v_meas = vel_hist[j];
      s.t = t_meas;
      s.deliver_t = t; // 当前时刻投递
      s.vel = Eigen::Vector3f(v_meas(0) + n_vel(rng),
                              v_meas(1) + n_vel(rng),
                              v_meas(2) + n_vel(rng));
      // GNSS 天线杆臂：机体 x 前方 lever_m，未在 EKF 中补偿
      const Eigen::Vector3f lever_ned = q_true * Eigen::Vector3f(lever_m, 0.0f, 0.0f);
      s.lla = bfs::ned2lla(Eigen::Vector3d(p_meas(0) + lever_ned(0) + n_pos(rng),
                                           p_meas(1) + lever_ned(1) + n_pos(rng),
                                           p_meas(2) + lever_ned(2) + n_pos(rng)),
                           kHome);
      if (outlier_unif(rng) < gnss_outlier_frac) {
        s.vel += Eigen::Vector3f(50.0f, 50.0f, 50.0f);
        s.lla = bfs::ned2lla(
            bfs::lla2ned(s.lla, kHome) + Eigen::Vector3d(100.0, 100.0, 100.0),
            kHome);
      }
      // 批处理使用投递时刻作为时间戳；量测值来自 t-tau 的历史快照。
      batch_delay.PushGnss(t, bfs::lla2ned(s.lla, kHome),
                           s.vel.cast<double>());
      pending.push_back(s);
    }

    while (!pending.empty() && t >= pending.front().deliver_t) {
      const Sample s = pending.front();
      pending.pop_front();
      const float age = static_cast<float>(t - s.t);
      float age_use = age;
      if (auto_delay) {
        if (!batch_delay_done && t >= 30.0f) {
          const bool ok = batch_delay.TryEstimate(1000, 50);
          tau_est = batch_delay.last_result().tau_s;
          batch_delay_ok = ok && batch_delay.last_result().valid &&
                           batch_delay.last_result().sharpness >= 0.003f &&
                           std::fabs(tau_est - 0.015f) <= 0.10f &&
                           vib_g <= 0.15f;
          std::printf("DBG_BATCH ok=%d tau=%.1f valid=%d ref=%zu gnss=%zu c0=%.3f c50=%.3f c100=%.3f\n",
                      ok ? 1 : 0, tau_est * 1000.0f,
                      batch_delay.last_result().valid ? 1 : 0,
                      batch_delay.DebugRefCount(), batch_delay.DebugGnssCount(),
                      batch_delay.DebugCost(0.0), batch_delay.DebugCost(0.05),
                      batch_delay.DebugCost(0.10));
          batch_delay_done = true;
        }
        if (batch_delay_done) {
          // 与固件一致：估计不可用/门控不过 -> 回退 15 ms。
          age_use = batch_delay_ok ? tau_est : 0.015f;
        }
      }
      if (false) {
        // 速度新息估计：v_gnss(t-tau) ≈ v_true(t) - a*tau
        // 当前 EKF 速度为 v_true，GNSS 速度为 v_gnss；a 为机体系加速度转 NED。
        // 匀速时 a≈0，时延不可观测；只有机动/加速段才更新估计。
        const Eigen::Vector3f dv = s.vel - comp.ned_vel_mps();
        const float a2 = a_cmd.squaredNorm();
        if (a2 > 0.25f) {
          const float raw = -dv.dot(a_cmd) / a2;
          if (std::isfinite(raw) && std::fabs(raw) < 0.2f) {
            tau_est += 0.05f * (raw - tau_est);
            tau_est = std::fmax(0.0f, std::fmin(0.2f, tau_est));
          }
        }
        age_use = tau_est;
      }
      const auto rc = comp.MeasurementUpdateDetailed(
          s.vel, makeImuLla(s.lla, comp.quat(), lever_m, lever_comp),
          kDt, age_use, true);
      if (rc.fused) ++out.fused; else ++out.rejected;
      base.MeasurementUpdateDetailed(
          s.vel, makeImuLla(s.lla, base.quat(), lever_m, lever_comp),
          kDt, 0.0f, false);
    }

    if (t >= 40.0) {
      const double pb = horizError(base, truth), pc = horizError(comp, truth);
      const double p3b = pos3dError(base, truth), p3c = pos3dError(comp, truth);
      const double vb = (base.ned_vel_mps() - truth_vel).norm();
      const double vc = (comp.ned_vel_mps() - truth_vel).norm();
      sum_pb += pb * pb; sum_pc += pc * pc;
      sum_p3b += p3b * p3b; sum_p3c += p3c * p3c;
      sum_vb += vb * vb; sum_vc += vc * vc;
      ++count;
    }
  }
  out.pos_base = std::sqrt(sum_pb / count);
  out.pos_comp = std::sqrt(sum_pc / count);
  out.pos3d_base = std::sqrt(sum_p3b / count);
  out.pos3d_comp = std::sqrt(sum_p3c / count);
  out.est_delay_s = tau_est;
  out.vel_base = std::sqrt(sum_vb / count);
  out.vel_comp = std::sqrt(sum_vc / count);
  out.accel_bias = comp.accel_bias_mps2();
  out.gyro_bias = comp.gyro_bias_radps();
  return out;
}
} // namespace

int main(int argc, char **argv) {
  float speed = 30.0f, latency = 0.050f;
  float ekf_accel_std = 0.25f, ekf_gyro_std = 0.0015f;
  float ekf_gyro_markov = 0.00524f;
  float ekf_gnss_pos = 1.5f, ekf_gnss_vel = 0.05f;
  float ekf_accel_markov = 0.01f;
  float vib_g = 0.0f;
  bool adaptive_q = false;
  int vib_mode = 0;
  float gyro_vib_dps = 0.5f;
  float vib_hz = 20.0f;
  float outage_start_s = 1.0e9f;
  float outage_end_s = 1.0e9f;
  float lever_m = 0.0f;
  bool lever_comp = false;
  float latency_jitter_s = 0.0f;
  bool auto_delay = false;
  bool accel_bias_gate = false;
  float bias_step_s = 0.0f, bias_step_accel = 0.0f, bias_step_gyro_dps = 0.0f;
  float bias_drift_accel = 0.0f, bias_drift_gyro_dps = 0.0f;
  bool gnss_bias_freeze = false;
  bool trace = false;
  bool post_outage_boost = false;
  bool bias_reanchor = false;
  bool reset_on_recovery = false;
  bool soft_velocity = false;
  float accel_scale = 1.0f, gyro_scale = 1.0f, misalign = 0.0f;
  float accel_bias_y = 0.0f, accel_bias_z = 0.0f;
  float gnss_outlier_frac = 0.0f;
  unsigned seed = 20261022u;
  int maneuver = 0;
  if (argc > 1) speed = std::atof(argv[1]);
  if (argc > 2) latency = std::atof(argv[2]);
  if (argc > 3) ekf_accel_std = std::atof(argv[3]);
  if (argc > 4) ekf_gyro_std = std::atof(argv[4]);
  if (argc > 5) ekf_gyro_markov = std::atof(argv[5]);
  if (argc > 6) ekf_gnss_pos = std::atof(argv[6]);
  if (argc > 7) ekf_gnss_vel = std::atof(argv[7]);
  if (argc > 8) seed = static_cast<unsigned>(std::atoi(argv[8]));
  if (argc > 9) maneuver = std::atoi(argv[9]);
  if (argc > 10) ekf_accel_markov = std::atof(argv[10]);
  if (argc > 11) vib_g = std::atof(argv[11]);
  if (argc > 12) adaptive_q = std::atoi(argv[12]) != 0;
  if (argc > 13) vib_mode = std::atoi(argv[13]);
  if (argc > 14) gyro_vib_dps = std::atof(argv[14]);
  if (argc > 15) vib_hz = std::atof(argv[15]);
  if (argc > 16) outage_start_s = std::atof(argv[16]);
  if (argc > 17) outage_end_s = std::atof(argv[17]);
  if (argc > 18) lever_m = std::atof(argv[18]);
  if (argc > 19) lever_comp = std::atoi(argv[19]) != 0;
  if (argc > 20) latency_jitter_s = std::atof(argv[20]);
  if (argc > 21) auto_delay = std::atoi(argv[21]) != 0;
  if (argc > 22) accel_scale = std::atof(argv[22]);
  if (argc > 23) gyro_scale = std::atof(argv[23]);
  if (argc > 24) misalign = std::atof(argv[24]);
  if (argc > 25) accel_bias_y = std::atof(argv[25]);
  if (argc > 26) accel_bias_z = std::atof(argv[26]);
  if (argc > 27) gnss_outlier_frac = std::atof(argv[27]);
  if (argc > 28) accel_bias_gate = std::atoi(argv[28]) != 0;
  if (argc > 29) bias_step_s = std::atof(argv[29]);
  if (argc > 30) bias_step_accel = std::atof(argv[30]);
  if (argc > 31) bias_step_gyro_dps = std::atof(argv[31]);
  if (argc > 32) bias_drift_accel = std::atof(argv[32]);
  if (argc > 33) bias_drift_gyro_dps = std::atof(argv[33]);
  if (argc > 34) gnss_bias_freeze = std::atoi(argv[34]) != 0;
  if (argc > 35) trace = std::atoi(argv[35]) != 0;
  if (argc > 36) post_outage_boost = std::atoi(argv[36]) != 0;
  if (argc > 37) bias_reanchor = std::atoi(argv[37]) != 0;
  if (argc > 38) reset_on_recovery = std::atoi(argv[38]) != 0;
  if (argc > 39) soft_velocity = std::atoi(argv[39]) != 0;

  std::printf("ARG trace=%d argc=%d\n", trace ? 1 : 0, argc);
  const Result r = run(speed, latency, seed, maneuver, ekf_accel_std,
                       ekf_gyro_std, ekf_gyro_markov, ekf_gnss_pos,
                       ekf_gnss_vel, ekf_accel_markov, vib_g, adaptive_q,
                       vib_mode, gyro_vib_dps, vib_hz,
                       outage_start_s, outage_end_s, lever_m, lever_comp,
                       latency_jitter_s, auto_delay,
                       accel_scale, gyro_scale, misalign,
                       accel_bias_y, accel_bias_z, gnss_outlier_frac,
                       accel_bias_gate, bias_step_s, bias_step_accel,
                       bias_step_gyro_dps, bias_drift_accel,
                       bias_drift_gyro_dps, gnss_bias_freeze, trace,
                       post_outage_boost, bias_reanchor,
                       reset_on_recovery, soft_velocity);
  const double gain = (r.pos_base > 1e-9) ? (1.0 - r.pos_comp / r.pos_base) * 100.0 : 0.0;
  std::printf("ICM-42688 + UBX PVT | v=%.1f m/s tau=%.0f ms seed=%u maneuver=%d vib=%.2fg adaptiveQ=%d\n",
              speed, latency * 1000.0f, seed, maneuver, vib_g,
              adaptive_q ? 1 : 0);
  std::printf("EKF: accel_std=%.3f accel_markov=%.4f gyro_std=%.5f gyro_markov=%.5f gnss_pos=%.2f gnss_vel=%.3f\n",
              ekf_accel_std, ekf_accel_markov, ekf_gyro_std, ekf_gyro_markov,
              ekf_gnss_pos, ekf_gnss_vel);
  std::printf("no-comp: pos=%.3f m pos3d=%.3f m vel=%.3f m/s\n",
              r.pos_base, r.pos3d_base, r.vel_base);
  std::printf("bias: ab=[%.4f,%.4f,%.4f] gb_dps=[%.4f,%.4f,%.4f]\n",
              r.accel_bias(0), r.accel_bias(1), r.accel_bias(2),
              r.gyro_bias(0)*57.2958f, r.gyro_bias(1)*57.2958f,
              r.gyro_bias(2)*57.2958f);
  std::printf("age-comp: pos=%.3f m pos3d=%.3f m vel=%.3f m/s | fused=%d rejected=%d | improve=%.1f%% | est_tau=%.1f ms\n",
              r.pos_comp, r.pos3d_comp, r.vel_comp, r.fused, r.rejected, gain,
              r.est_delay_s * 1000.0);
  return (r.pos_comp < r.pos_base * 1.05) ? 0 : 1;
}
