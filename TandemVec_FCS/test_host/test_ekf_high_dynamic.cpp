/**
 * @file    test_ekf_high_dynamic.cpp
 * @module  固件/宿主研究（fw-tests：大过载/大机动卫惯组合导航压力台架）
 * @brief   用解析高动态轨迹驱动真实 `Ekf15State`，量化大过载/高角速率下的
 *          位置、速度、姿态精度与协方差一致性
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h`（与固件同源）。
 *          真值：2 kHz 积分 q/v/LLA，比力方程含 WGS-84 重力、地球自转与运输率；
 *          IMU 400 Hz（双子样 200 Hz，可切单子样）；GNSS 10 Hz，支持延迟/失锁/
 *          位置速度噪声/杆臂。IMU 误差含白噪声、常值零偏、尺度因子、安装失准、
 *          振动；轨迹含协调转弯、S 转弯、拉杆/推杆、连续滚转、圆锥运动。
 *          输出：`HIGH_DYN_JSON {...}`（RMSE/最大误差/NEES/融合计数）。
 *          运行示例：
 *            highdyn --profile turn --load 4 --speed 80 --latency-ms 20 \
 *                    --seed 1 --duration 60 --json
 * @theory  比力方程 a^n = C_b^n f^b + g^n − (2ω_ie+ω_en)×v^n；
 *          姿态 q̇ = ½ q⊗ω_nb^b；双子样圆锥/划桨补偿见 REF-120 §9.3/§9.6。
 *          @ref REF-120 §7.4 (7.4.6) p.189；§9.6.1 p.298；§9.6.3 p.303
 * @frame   NED（位置/速度/重力），FRD（IMU）
 * @units   m、m/s、rad、s、g（load=比力模长/g）
 * @note    本台架是导航算法压力测试，不代表 TandemVec 的气动/推力能力；
 *          过载剖面为解析/可行轨迹，用于暴露 IMU/GNSS/滤波器误差机制。
 * @test    H1 协调转弯 g 扫描；H2 S 转弯滚转反转；H3 拉杆/推杆；H4 连续滚转；
 *          H5 圆锥运动；H6 GNSS 延迟/失锁；H7 尺度/失准/杆臂；H8 大规模 seeds
 * @see     TandemVec-EKF-Report/卫惯组合导航-延时补偿-高速精度-2026-10-22.md
 */
// ============================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "earth_model.h"
#include "transforms.h"

using bfs::Ekf15State;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr float kG0 = 9.80665f;
constexpr double kHomeLat = 31.2304 * kPi / 180.0;
constexpr double kHomeLon = 121.4737 * kPi / 180.0;
constexpr double kHomeAlt = 50.0;
const Eigen::Vector3d kHome(kHomeLat, kHomeLon, kHomeAlt);
constexpr double kTruthDt = 0.0005;   // 2 kHz truth
constexpr double kNavDt = 0.005;      // 200 Hz navigation
constexpr float kSettleS = 10.0f;

struct Cfg {
  std::string profile = "turn";
  float load = 4.0f;
  float speed = 80.0f;
  float rate_dps = 0.0f;      // roll profile: roll rate; coning: base rate
  float duration = 60.0f;
  float lead_in = 5.0f;
  float latency_ms = 20.0f;
  float gnss_pos_std = 1.5f;
  float gnss_vel_std = 0.05f;
  float outage_a = -1.0f;
  float outage_b = -1.0f;
  float accel_scale = 1.0f;
  float gyro_scale = 1.0f;
  float misalign_deg = 0.0f;
  float accel_bias_mg = 20.0f;
  float gyro_bias_dps = 0.5f;
  float lever_m = 0.0f;
  int lever_comp = 0;
  int vel_subset = 0;
  float vib_g = 0.0f;
  float vib_hz = 80.0f;
  float accel_fs_g = 8.0f;
  float gyro_fs_dps = 2000.0f;
  int imu_rate_hz = 400;
  bool two_sample = true;
  int seed = 1;
  bool json = false;
  bool trace = false;
  float assert_rmse = 0.0f;
};

float smoothstep(const float x) {
  const float u = std::clamp(x, 0.0f, 1.0f);
  return u * u * (3.0f - 2.0f * u);
}

Eigen::Quaternionf EulerToQuat(const float roll, const float pitch,
                               const float yaw) {
  return (Eigen::Quaternionf(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())) *
          Eigen::Quaternionf(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())) *
          Eigen::Quaternionf(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())))
      .normalized();
}

// 解析轨迹的机体系角速率与比力（FRD；f_b 的 z 向下为负）。
struct BodyMotion {
  Eigen::Vector3f omega_b = Eigen::Vector3f::Zero();
  Eigen::Vector3f f_b = Eigen::Vector3f(0.0f, 0.0f, -kG0);
};

class Profile {
 public:
  explicit Profile(const Cfg& c) : cfg_(c) {
    if (cfg_.profile == "turn") {
      load_ = std::max(1.0f, cfg_.load);
      bank_ = std::acos(1.0f / load_);
      yaw_rate_ = kG0 * std::tan(bank_) / std::max(1.0f, cfg_.speed);
    } else if (cfg_.profile == "sturn") {
      load_ = std::max(1.0f, cfg_.load);
      bank_ = std::acos(1.0f / load_);
    } else if (cfg_.profile == "pull" || cfg_.profile == "negg") {
      load_ = std::max(0.2f, cfg_.load);
      pull_rate_hz_ = 0.5f;
    } else if (cfg_.profile == "roll") {
      load_ = std::max(0.2f, cfg_.load);
      roll_rate_ = cfg_.rate_dps * static_cast<float>(kPi) / 180.0f;
    } else if (cfg_.profile == "coning") {
      coning_amp_ = std::max(0.05f, cfg_.rate_dps) *
                    static_cast<float>(kPi) / 180.0f;
    }
  }

  // 由时间解析/状态积分得到机体系角速度与比力；state q 由调用方推进。
  BodyMotion motion(const double t, const Eigen::Quaternionf& q) {
    BodyMotion m;
    const float tl = static_cast<float>(t);
    const float ramp = smoothstep((tl - 1.0f) / std::max(0.5f, cfg_.lead_in));
    if (cfg_.profile == "turn") {
      const float phi = bank_ * ramp;
      const float psi_dot = yaw_rate_ * ramp;
      m.omega_b = Eigen::Vector3f(0.0f, psi_dot * std::sin(phi),
                                  psi_dot * std::cos(phi));
      m.f_b = Eigen::Vector3f(0.0f, 0.0f, -(1.0f + (load_ - 1.0f) * ramp) * kG0);
    } else if (cfg_.profile == "sturn") {
      // 8 s 周期的平滑 S 转弯：bank 从 +φmax 反转到 −φmax。
      const float s = std::sin(2.0f * static_cast<float>(kPi) * tl / 8.0f);
      const float shape = std::tanh(2.5f * s) / std::tanh(2.5f);
      const float phi = bank_ * ramp * shape;
      const float dshape = 2.5f * (1.0f - std::tanh(2.5f * s) * std::tanh(2.5f * s)) /
                           std::tanh(2.5f) * std::cos(2.0f * static_cast<float>(kPi) *
                                                       tl / 8.0f) *
                           (2.0f * static_cast<float>(kPi) / 8.0f);
      const float phi_dot = bank_ * ramp * dshape;
      const float psi_dot = kG0 * std::tan(phi) / std::max(1.0f, cfg_.speed);
      m.omega_b = Eigen::Vector3f(phi_dot, psi_dot * std::sin(phi),
                                  psi_dot * std::cos(phi));
      m.f_b = Eigen::Vector3f(0.0f, 0.0f,
                              -kG0 / std::max(0.2f, std::cos(phi)));
    } else if (cfg_.profile == "pull") {
      // 纯正过载拉杆：n 从 1 g 平滑升到 load，再回 1 g，不进入负 g。
      const float w = 2.0f * static_cast<float>(kPi) * pull_rate_hz_;
      const float n = 1.0f + (load_ - 1.0f) * ramp *
                              0.5f * (1.0f - std::cos(w * tl));
      const float theta_dot = (n - 1.0f) * kG0 / std::max(1.0f, cfg_.speed);
      m.omega_b = Eigen::Vector3f(0.0f, theta_dot, 0.0f);
      m.f_b = Eigen::Vector3f(0.0f, 0.0f, -n * kG0);
    } else if (cfg_.profile == "negg") {
      // 专项边界：正负 g 交替，最小 n=1-(load-1)（负 g 时姿态不可观）。
      const float w = 2.0f * static_cast<float>(kPi) * pull_rate_hz_;
      const float n = 1.0f + (load_ - 1.0f) * ramp * std::sin(w * tl);
      const float theta_dot = (n - 1.0f) * kG0 / std::max(1.0f, cfg_.speed);
      m.omega_b = Eigen::Vector3f(0.0f, theta_dot, 0.0f);
      m.f_b = Eigen::Vector3f(0.0f, 0.0f, -n * kG0);
    } else if (cfg_.profile == "roll") {
      const float p = roll_rate_ * ramp;
      m.omega_b = Eigen::Vector3f(p, 0.0f, 0.0f);
      m.f_b = Eigen::Vector3f(0.0f, 0.0f, -load_ * kG0);
    } else {  // coning
      const float w = 2.0f * static_cast<float>(kPi) * 1.0f;
      const float amp = coning_amp_ * ramp;
      m.omega_b = Eigen::Vector3f(amp * std::sin(w * tl),
                                  amp * std::cos(w * tl), 0.2f * amp);
      m.f_b = Eigen::Vector3f(0.0f, 0.0f, -kG0);
    }
    (void)q;
    return m;
  }

 private:
  Cfg cfg_;
  float load_ = 1.0f, bank_ = 0.0f, yaw_rate_ = 0.0f;
  float pull_amp_ = 0.0f, pull_rate_hz_ = 0.5f;
  float roll_rate_ = 0.0f, coning_amp_ = 0.0f;
};

double GravityD(const Eigen::Vector3d& lla) {
  return bfs::normalgravity_mps2(lla);
}

void ParseArgs(int argc, char** argv, Cfg* c) {
  auto next = [&](int& i) -> std::string {
    return (i + 1 < argc) ? argv[++i] : std::string();
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--profile") c->profile = next(i);
    else if (a == "--load") c->load = std::atof(next(i).c_str());
    else if (a == "--speed") c->speed = std::atof(next(i).c_str());
    else if (a == "--rate-dps") c->rate_dps = std::atof(next(i).c_str());
    else if (a == "--duration") c->duration = std::atof(next(i).c_str());
    else if (a == "--lead-in") c->lead_in = std::atof(next(i).c_str());
    else if (a == "--latency-ms") c->latency_ms = std::atof(next(i).c_str());
    else if (a == "--gnss-pos-std") c->gnss_pos_std = std::atof(next(i).c_str());
    else if (a == "--gnss-vel-std") c->gnss_vel_std = std::atof(next(i).c_str());
    else if (a == "--outage") std::sscanf(next(i).c_str(), "%f:%f", &c->outage_a, &c->outage_b);
    else if (a == "--accel-scale") c->accel_scale = std::atof(next(i).c_str());
    else if (a == "--gyro-scale") c->gyro_scale = std::atof(next(i).c_str());
    else if (a == "--misalign-deg") c->misalign_deg = std::atof(next(i).c_str());
    else if (a == "--accel-bias-mg") c->accel_bias_mg = std::atof(next(i).c_str());
    else if (a == "--gyro-bias-dps") c->gyro_bias_dps = std::atof(next(i).c_str());
    else if (a == "--lever-m") c->lever_m = std::atof(next(i).c_str());
    else if (a == "--lever-comp") c->lever_comp = std::atoi(next(i).c_str());
    else if (a == "--vel-subset") c->vel_subset = std::atoi(next(i).c_str());
    else if (a == "--vib-g") c->vib_g = std::atof(next(i).c_str());
    else if (a == "--vib-hz") c->vib_hz = std::atof(next(i).c_str());
    else if (a == "--accel-fs-g") c->accel_fs_g = std::atof(next(i).c_str());
    else if (a == "--gyro-fs-dps") c->gyro_fs_dps = std::atof(next(i).c_str());
    else if (a == "--imu-rate") c->imu_rate_hz = std::atoi(next(i).c_str());
    else if (a == "--two-sample") c->two_sample = std::atoi(next(i).c_str()) != 0;
    else if (a == "--seed") c->seed = std::atoi(next(i).c_str());
    else if (a == "--json") c->json = true;
    else if (a == "--trace") c->trace = true;
    else if (a == "--assert-rmse") c->assert_rmse = std::atof(next(i).c_str());
  }
}

struct Metrics {
  double sum_pos2 = 0, sum_h2 = 0, sum_vel2 = 0, sum_att2 = 0;
  double max_pos = 0, max_att_deg = 0;
  int n = 0, fused = 0, rejected = 0, fail = 0;
  std::vector<float> nees_p, nees_v;
};

Eigen::Vector3f QuatToRpy(const Eigen::Quaternionf& q) {
  return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

}  // namespace

int main(int argc, char** argv) {
  Cfg cfg;
  ParseArgs(argc, argv, &cfg);
  const double speed = std::max(1.0f, cfg.speed);
  const int steps = static_cast<int>(cfg.duration / kNavDt);
  const int truth_steps = static_cast<int>(cfg.duration / kTruthDt);
  const int gnss_div = std::max(1, static_cast<int>(std::lround(
                                     200.0 / std::max(1.0f, 10.0f))));
  const int imu_per_nav = std::max(1, cfg.imu_rate_hz / 200);
  const float sub_dt = static_cast<float>(kNavDt / imu_per_nav);

  std::mt19937 rng(static_cast<unsigned>(cfg.seed));
  const float gyro_sigma = 0.0035f * static_cast<float>(kPi) / 180.0f *
                           std::sqrt(0.5f * cfg.imu_rate_hz);
  const float accel_sigma = 70.0e-6f * kG0 * std::sqrt(0.5f * cfg.imu_rate_hz);
  std::normal_distribution<float> ng(0.0f, gyro_sigma);
  std::normal_distribution<float> na(0.0f, accel_sigma);
  std::normal_distribution<float> np(0.0f, cfg.gnss_pos_std);
  std::normal_distribution<float> nv(0.0f, cfg.gnss_vel_std);
  const float mis = cfg.misalign_deg * static_cast<float>(kPi) / 180.0f;
  Eigen::Matrix3f M = Eigen::Matrix3f::Identity();
  M(0, 1) = mis;   M(0, 2) = -mis;
  M(1, 0) = -mis;  M(1, 2) = mis;
  M(2, 0) = mis;   M(2, 1) = -mis;

  std::vector<Eigen::Quaternionf> tq(truth_steps + 1);
  std::vector<Eigen::Vector3d> tlla(truth_steps + 1);
  std::vector<Eigen::Vector3f> tv(truth_steps + 1);
  std::vector<Eigen::Vector3f> tf(truth_steps + 1);
  std::vector<Eigen::Vector3f> tw(truth_steps + 1);

  Profile profile(cfg);
  Eigen::Quaternionf q_true = Eigen::Quaternionf::Identity();
  Eigen::Vector3f v_true(speed, 0.0f, 0.0f);
  Eigen::Vector3d lla = kHome;
  tq[0] = q_true;
  tlla[0] = lla;
  tv[0] = v_true;
  tf[0] = Eigen::Vector3f(0.0f, 0.0f, -kG0);
  tw[0] = Eigen::Vector3f::Zero();
  for (int k = 0; k < truth_steps; ++k) {
    const double t = k * kTruthDt;
    BodyMotion bm = profile.motion(t, q_true);
    const Eigen::Matrix3f C = q_true.toRotationMatrix();
    const double g = GravityD(lla);
    const Eigen::Vector3d wie = bfs::earthrate(lla(0));
    const Eigen::Vector3d wen = bfs::navrate(v_true.cast<double>(), lla);
    const Eigen::Vector3f coriolis =
        static_cast<float>(-1.0) *
        ((2.0 * wie + wen).cross(v_true.cast<double>())).cast<float>();
    const Eigen::Vector3f a_ned =
        C * bm.f_b + Eigen::Vector3f(0.0f, 0.0f, static_cast<float>(g)) + coriolis;
    const float wnorm = bm.omega_b.norm();
    if (wnorm > 1e-9f) {
      q_true = (q_true * Eigen::Quaternionf(Eigen::AngleAxisf(
                             wnorm * static_cast<float>(kTruthDt),
                             bm.omega_b / wnorm))).normalized();
    }
    v_true += a_ned * static_cast<float>(kTruthDt);
    lla += bfs::llarate(v_true.cast<double>(), lla) * kTruthDt;
    tq[k + 1] = q_true;
    tlla[k + 1] = lla;
    tv[k + 1] = v_true;
    tf[k + 1] = bm.f_b;
    tw[k + 1] = bm.omega_b;
  }

  Ekf15State ekf;
  ekf.accel_std_mps2(0.25f);
  ekf.accel_markov_bias_std_mps2(0.010f);
  ekf.gyro_std_radps(0.0015f);
  ekf.gyro_markov_bias_std_radps(0.001f);
  ekf.gnss_pos_ne_std_m(cfg.gnss_pos_std);
  ekf.gnss_pos_d_std_m(cfg.gnss_pos_std * 2.0f);
  ekf.gnss_vel_ne_std_mps(cfg.gnss_vel_std);
  ekf.gnss_vel_d_std_mps(cfg.gnss_vel_std * 2.0f);
  ekf.init_pos_err_std_m(2.0f);
  ekf.init_vel_err_std_mps(0.08f);
  ekf.init_att_err_std_rad(0.08f);
  ekf.init_heading_err_std_rad(0.2f);
  ekf.Initialize(tf[0], tw[0], Eigen::Vector3f::Zero(), tv[0], kHome);
  // 台架的合理值门限必须随配置量程缩放，模拟固件在对应 FS 档的配置；
  // 否则 >12.2 g 会被默认 8 g 门限整帧丢弃（32 g 试点定位的隐藏上限）。
  ekf.set_imu_plausibility_limits(
      cfg.accel_fs_g * static_cast<float>(kG0) * 1.6f,
      cfg.gyro_fs_dps * static_cast<float>(kPi) / 180.0f * 1.2f);
  if (cfg.lever_comp == 2) {
    ekf.gnss_lever_arm_body_m(Eigen::Vector3f(cfg.lever_m, 0.0f, 0.0f));
  }
  ekf.set_gnss_velocity_subset_recovery_enabled(cfg.vel_subset != 0);

  std::deque<Eigen::Vector3d> gnss_lla_hist;
  std::deque<Eigen::Vector3f> gnss_vel_hist;
  Metrics m;
  const double gnom = GravityD(kHome);
  const float abias = cfg.accel_bias_mg * 0.001f * gnom;
  const float gbias = cfg.gyro_bias_dps * static_cast<float>(kPi) / 180.0f;

  auto sample_imu = [&](const double ts, Eigen::Vector3f* f_out,
                        Eigen::Vector3f* w_out) {
    const int idx = std::clamp(static_cast<int>(std::lround(ts / kTruthDt)), 0,
                               truth_steps);
    BodyMotion bm = profile.motion(ts, tq[idx]);
    const float vib = cfg.vib_g * kG0 *
                      std::sin(2.0f * static_cast<float>(kPi) * cfg.vib_hz *
                               static_cast<float>(ts));
    Eigen::Vector3f fm = bm.f_b;
    fm(0) += vib;
    fm(2) += 0.5f * vib;
    fm = cfg.accel_scale * (M * fm);
    fm += Eigen::Vector3f(abias + na(rng), abias + na(rng), abias + na(rng));
    Eigen::Vector3f wm = cfg.gyro_scale * (M * bm.omega_b);
    wm += Eigen::Vector3f(gbias + ng(rng), gbias + ng(rng), gbias + ng(rng));
    // 传感器量程饱和（ICM42688 默认 ±8 g / ±2000 dps）。
    const float afs = cfg.accel_fs_g * kG0;
    const float gfs = cfg.gyro_fs_dps * static_cast<float>(kPi) / 180.0f;
    for (int a = 0; a < 3; ++a) {
      fm(a) = std::clamp(fm(a), -afs, afs);
      wm(a) = std::clamp(wm(a), -gfs, gfs);
    }
    *f_out = fm;
    *w_out = wm;
  };

  for (int k = 0; k < steps; ++k) {
    const double t = k * kNavDt;
    const int it = std::min(truth_steps, static_cast<int>(std::lround(t / kTruthDt)));

    const double t1 = t + 0.25 * kNavDt;
    const double t2 = t + 0.75 * kNavDt;
    Eigen::Vector3f f1, w1, f2, w2;
    sample_imu(t1, &f1, &w1);
    sample_imu(t2, &f2, &w2);
    if (cfg.two_sample) {
      const Eigen::Vector3f dt1 = f1 * sub_dt;
      const Eigen::Vector3f dt2 = f2 * sub_dt;
      const Eigen::Vector3f dw1 = w1 * sub_dt;
      const Eigen::Vector3f dw2 = w2 * sub_dt;
      ekf.TimeUpdateTwoSample(dw1, dw2, dt1, dt2, static_cast<float>(kNavDt));
    } else {
      const Eigen::Vector3f f_avg = 0.5f * (f1 + f2);
      const Eigen::Vector3f w_avg = 0.5f * (w1 + w2);
      ekf.TimeUpdate(f_avg, w_avg, static_cast<float>(kNavDt));
    }

    if (t >= 0.05) {
      gnss_lla_hist.push_back(tlla[it]);
      gnss_vel_hist.push_back(tv[it]);
      while (gnss_lla_hist.size() > 2000) {
        gnss_lla_hist.pop_front();
        gnss_vel_hist.pop_front();
      }
    }

    const bool outage = (cfg.outage_a >= 0.0f && t >= cfg.outage_a &&
                         t < cfg.outage_b);
    if (!outage && (k % gnss_div) == 0 && t >= 0.1) {
      const float age = cfg.latency_ms * 0.001f;
      const int back = std::min(static_cast<int>(gnss_lla_hist.size()) - 1,
                                static_cast<int>(std::lround(age / kNavDt)));
      if (back >= 0) {
        const Eigen::Vector3d p_meas = gnss_lla_hist[gnss_lla_hist.size() - 1 - back];
        const Eigen::Vector3f v_meas = gnss_vel_hist[gnss_vel_hist.size() - 1 - back];
        Eigen::Vector3d p_ned = bfs::lla2ned(p_meas, kHome);
        Eigen::Vector3f v_ant = v_meas;
        if (std::fabs(cfg.lever_m) > 1e-6f) {
          const int im = std::clamp(
              it - back * static_cast<int>(std::lround(kNavDt / kTruthDt)), 0,
              truth_steps);
          const Eigen::Vector3f lever_ned =
              tq[im] * Eigen::Vector3f(cfg.lever_m, 0.0f, 0.0f);
          p_ned += lever_ned.cast<double>();
          if (cfg.lever_comp == 1) {
            p_ned -= (ekf.quat() * Eigen::Vector3f(cfg.lever_m, 0.0f, 0.0f))
                         .cast<double>();
          }
        }
        if (std::fabs(cfg.lever_m) > 1e-6f) {
          // GNSS 天线速度 = IMU 速度 + C_b^n(ω×r)，用测量时刻真值。
          const int im = std::clamp(
              it - back * static_cast<int>(std::lround(kNavDt / kTruthDt)), 0,
              truth_steps);
          v_ant += tq[im] *
                   tw[im].cross(Eigen::Vector3f(cfg.lever_m, 0.0f, 0.0f));
        }
        p_ned(0) += np(rng);
        p_ned(1) += np(rng);
        p_ned(2) += np(rng) * 2.0;
        const Eigen::Vector3d lla_meas = bfs::ned2lla(p_ned, kHome);
        Eigen::Vector3f vel_meas = v_ant;
        vel_meas += Eigen::Vector3f(nv(rng), nv(rng), nv(rng));
        const auto r = ekf.MeasurementUpdateDetailed(
            vel_meas, lla_meas, static_cast<float>(kNavDt), age, true);
        if (r.fused) ++m.fused; else ++m.rejected;
      }
    }

    if (t >= kSettleS) {
      const Eigen::Vector3d p_est = bfs::lla2ned(ekf.lla_rad_m(), kHome);
      const Eigen::Vector3d p_true = bfs::lla2ned(tlla[it], kHome);
      const Eigen::Vector3d ep = p_est - p_true;
      const Eigen::Vector3f ev = ekf.ned_vel_mps() - tv[it];
      const float pos2 = static_cast<float>(ep.squaredNorm());
      const float h2 = static_cast<float>(ep(0) * ep(0) + ep(1) * ep(1));
      const float vel2 = ev.squaredNorm();
      const Eigen::Quaternionf qe = ekf.quat();
      const float dot = std::fabs(std::clamp(qe.dot(tq[it]), -1.0f, 1.0f));
      const float att = 2.0f * std::acos(dot);
      m.sum_pos2 += pos2;
      m.sum_h2 += h2;
      m.sum_vel2 += vel2;
      m.sum_att2 += att * att;
      m.max_pos = std::max(m.max_pos, static_cast<double>(std::sqrt(pos2)));
      m.max_att_deg = std::max(m.max_att_deg, static_cast<double>(att) * 57.29578);
      ++m.n;
      if (!std::isfinite(pos2) || m.max_pos > 1.0e4) m.fail = 1;
#ifdef EKF_HOST_REGRESSION
      const Eigen::Vector3f epf = ep.cast<float>();
      const Eigen::Vector3f evf = ev;
      Eigen::Matrix3f Pp, Pv;
      for (int rr = 0; rr < 3; ++rr) {
        for (int cc = 0; cc < 3; ++cc) {
          Pp(rr, cc) = ekf.covariance_coeff_for_test(rr, cc);
          Pv(rr, cc) = ekf.covariance_coeff_for_test(3 + rr, 3 + cc);
        }
      }
      const Eigen::Vector3f wp = Pp.ldlt().solve(epf);
      const Eigen::Vector3f wv = Pv.ldlt().solve(evf);
      if (std::isfinite(wp.norm())) m.nees_p.push_back(epf.dot(wp) / 3.0f);
      if (std::isfinite(wv.norm())) m.nees_v.push_back(evf.dot(wv) / 3.0f);
#endif
    }
    if (cfg.trace && (k % 200) == 0) {
      const Eigen::Vector3d p_est = bfs::lla2ned(ekf.lla_rad_m(), kHome);
      const Eigen::Vector3d p_true = bfs::lla2ned(tlla[it], kHome);
      const Eigen::Vector3f rpy_e =
          ekf.quat().toRotationMatrix().eulerAngles(2, 1, 0);
      const Eigen::Vector3f rpy_t =
          tq[it].toRotationMatrix().eulerAngles(2, 1, 0);
      std::printf("TRACE t=%.1f pos=%.2f attmax=%.2f re=(%.1f,%.1f,%.1f) rt=(%.1f,%.1f,%.1f) vel=%.2f fused=%d\n",
                  t, (p_est - p_true).norm(), m.max_att_deg,
                  rpy_e(0) * 57.3f, rpy_e(1) * 57.3f, rpy_e(2) * 57.3f,
                  rpy_t(0) * 57.3f, rpy_t(1) * 57.3f, rpy_t(2) * 57.3f,
                  (ekf.ned_vel_mps() - tv[it]).norm(), m.fused);
    }
  }

  auto median = [](std::vector<float> v) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  const double pos_rmse = m.n ? std::sqrt(m.sum_pos2 / m.n) : -1.0;
  const double h_rmse = m.n ? std::sqrt(m.sum_h2 / m.n) : -1.0;
  const double vel_rmse = m.n ? std::sqrt(m.sum_vel2 / m.n) : -1.0;
  const double att_rmse = m.n ? std::sqrt(m.sum_att2 / m.n) * 57.29578 : -1.0;
  if (cfg.json) {
    std::printf("HIGH_DYN_JSON {\"profile\":\"%s\",\"load\":%.3f,"
                "\"speed\":%.1f,\"rate_dps\":%.1f,\"seed\":%d,"
                "\"latency_ms\":%.1f,\"accel_scale\":%.4f,"
                "\"gyro_scale\":%.4f,\"misalign_deg\":%.3f,"
                "\"pos_rmse\":%.3f,\"h_rmse\":%.3f,\"pos_max\":%.3f,"
                "\"vel_rmse\":%.4f,\"att_rmse_deg\":%.3f,"
                "\"att_max_deg\":%.3f,\"nees_pos_med\":%.3f,"
                "\"nees_vel_med\":%.3f,\"nees_n\":%d,\"fused\":%d,"
                "\"rejected\":%d,\"fail\":%d}\n",
                cfg.profile.c_str(), cfg.load, cfg.speed, cfg.rate_dps, cfg.seed,
                cfg.latency_ms, cfg.accel_scale, cfg.gyro_scale,
                cfg.misalign_deg, pos_rmse, h_rmse, m.max_pos, vel_rmse,
                att_rmse, m.max_att_deg, median(m.nees_p), median(m.nees_v),
                static_cast<int>(m.nees_p.size()), m.fused, m.rejected, m.fail);
  } else {
    std::printf("[highdyn] %s load=%.1f speed=%.0f rate=%.1f seed=%d "
                "pos=%.2f h=%.2f max=%.2f vel=%.3f att=%.2f/%.2f "
                "NEESp=%.2f NEESv=%.2f fused=%d rej=%d fail=%d\n",
                cfg.profile.c_str(), cfg.load, cfg.speed, cfg.rate_dps,
                cfg.seed, pos_rmse, h_rmse, m.max_pos, vel_rmse, att_rmse,
                m.max_att_deg, median(m.nees_p), median(m.nees_v), m.fused,
                m.rejected, m.fail);
  }
  if (m.fail) return 2;
  if (cfg.assert_rmse > 0.0f && pos_rmse > cfg.assert_rmse) return 3;
  return 0;
}
