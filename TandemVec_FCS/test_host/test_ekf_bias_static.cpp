/**
 * @file    test_ekf_bias_static.cpp
 * @module  固件/宿主研究（fw-tests：EKF 在线零偏估计与静止矫正影响）
 * @brief   在静止/误检静止/油门/门控四种模式下观察陀螺与加计零偏的估计表现
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的零偏状态与
 *          static gyro / gravity / ZUPT 三条静止辅助路径的组合效应。
 *          输入：argv = seed、真实陀螺零偏 dps、真实加计零偏 mg、模式号；
 *          模式 1 开 static gyro、2 追加“误检静止 + 0.3 rad/s 假角速率”、
 *          3 追加 20% 油门、4 追加静止门控；
 *          处理：200 Hz 跑固定时长后看零偏估计；
 *          输出：一行汇总 + 逐组表格（真实值对照）。
 * @theory  零偏可观测性：静止时陀螺角速率应为零、加计应只剩重力，
 *          故静止窗口是零偏唯一的强观测段；一旦把机动误判为静止，零偏会被
 *          拉向错误值。@ref REF-121 §8.2.3 p.291（静止辅助）、@ref REF-121 §2.2.1 p.33
 * @frame   NED/FRD（比力与角速率均机体系）
 * @units   陀螺 dps，加计 mg（1 g = 9.80665 m/s²），步长 s
 * @note    脚本参数为 1-based 顺序映射，错位会静默改变场景（见 AGENTS.md
 *          “EKF 在线零偏可靠性”的数据更正）；mode=2 是人为构造的“误静止”压力工况，
 *          不对应任何正常飞行状态。
 * @test    M1 正常静止；M2 误检静止；M3 油门非零时跳过 static gyro；M4 静止门控
 * @see     TandemVec-EKF-Report/EKF在线零偏估计可靠性-2026-10-23.md
 */
// EKF 在线零偏估计 + 静止矫正影响 宿主回归
#include <cmath>
#include <cstdio>
#include <random>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "transforms.h"

using bfs::Ekf15State;
namespace {
constexpr float kDt = 0.005f;
constexpr float kG = 9.79f;
constexpr double kPi = 3.14159265358979323846;
const Eigen::Vector3d kHome(31.2304*kPi/180.0, 121.4737*kPi/180.0, 50.0);

struct BiasResult { Eigen::Vector3f accel; Eigen::Vector3f gyro; };

BiasResult run(bool use_static_gyro, bool use_gravity, bool use_zupt,
               float true_gyro_dps, float true_accel_mg,
               float false_rate_radps, float throttle_percent,
               bool misdetect_static, bool use_gnss, bool gate_static,
               unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<float> na(0.0f, 0.0069f);
  std::normal_distribution<float> ng(0.0f, 0.0005f);
  Ekf15State e;
  e.gyro_std_radps(0.0015f);
  e.gyro_markov_bias_std_radps(0.00524f);
  e.accel_std_mps2(0.25f);
  e.gnss_pos_ne_std_m(1.5f); e.gnss_pos_d_std_m(3.0f);
  e.gnss_vel_ne_std_mps(0.05f); e.gnss_vel_d_std_mps(0.10f);
  e.init_pos_err_std_m(2.0f); e.init_vel_err_std_mps(0.08f);
  e.init_att_err_std_rad(0.08f); e.init_heading_err_std_rad(0.2f);
  e.init_accel_bias_std_mps2(0.196f);
  e.init_gyro_bias_std_radps(0.00698f);
  e.Initialize(Eigen::Vector3f(0,0,-kG), Eigen::Vector3f::Zero(),
               Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), kHome);

  const float gyro_bias = true_gyro_dps * (float)kPi / 180.0f;
  const float accel_bias = true_accel_mg * 0.00980665f;
  const int steps = static_cast<int>(120.0f / kDt);
  for (int k = 0; k < steps; ++k)
  {
    // 静止但可能有误判/慢转；false_rate 用于“静止误判”实验。
    const float t = k * kDt;
    Eigen::Vector3f accel(accel_bias + na(rng), na(rng), -kG + na(rng));
    Eigen::Vector3f gyro(gyro_bias + ng(rng),
                         gyro_bias * 0.5f + ng(rng),
                         gyro_bias * 0.8f + ng(rng));
    if (false_rate_radps > 0.0f)
    {
      gyro(2) += false_rate_radps * std::sin(2.0f * 3.14159f * 0.05f * t);
    }
    e.TimeUpdate(accel, gyro, kDt);

    if (use_gnss && k % 20 == 0)
    {
      e.MeasurementUpdateDetailed(Eigen::Vector3f::Zero(), kHome, kDt, 0.0f, false);
    }

    // 静止辅助每 8 帧（25Hz），与固件量级一致。
    if (k % 8 == 0 && misdetect_static)
    {
      const bool static_ok = !gate_static ||
          (gyro.norm() < 0.15f && std::fabs(accel.norm() - kG) < 0.5f);
      if (use_static_gyro && throttle_percent < 3.0f && static_ok)
      {
        e.MeasurementUpdateStaticGyroDetailed(gyro, Eigen::Vector3f(1.0f, 1.0f, 0.0005f));
      }
      if (use_gravity)
      {
        e.MeasurementUpdateGravityDetailed(accel, 0.01f);
      }
      if (use_zupt)
      {
        e.MeasurementUpdateVelocityDetailed(Eigen::Vector3f::Zero(), 0.02f, 0.02f);
      }
    }
    else if (k % 8 == 0 && !misdetect_static)
    {
      // 真正静止：全部辅助
      const bool static_ok = !gate_static ||
          (gyro.norm() < 0.15f && std::fabs(accel.norm() - kG) < 0.5f);
      if (use_static_gyro && throttle_percent < 3.0f && static_ok)
        e.MeasurementUpdateStaticGyroDetailed(gyro, Eigen::Vector3f(1.0f, 1.0f, 0.0005f));
      if (use_gravity)
        e.MeasurementUpdateGravityDetailed(accel, 0.01f);
      if (use_zupt)
        e.MeasurementUpdateVelocityDetailed(Eigen::Vector3f::Zero(), 0.02f, 0.02f);
    }
  }
  return {e.accel_bias_mps2(), e.gyro_bias_radps()};
}

void row(const char *name, const BiasResult &r, float tg, float ta)
{
  const float gd = r.gyro.norm() * 57.2958f;
  const float ad = r.accel.norm() / 9.80665f * 1000.0f;
  std::printf("%-28s gyro=%6.2f dps (true %.1f)  accel=%6.1f mg (true %.1f)\n",
              name, gd, tg, ad, ta);
}
} // namespace

int main(int argc, char **argv)
{
  const unsigned seed = (argc > 1) ? static_cast<unsigned>(std::atoi(argv[1])) : 1;
  const float tg = (argc > 2) ? std::atof(argv[2]) : 0.5f;
  const float ta = (argc > 3) ? std::atof(argv[3]) : 20.0f;
  const int mode = (argc > 4) ? std::atoi(argv[4]) : 1;
  const bool use_gyro = (mode == 1 || mode == 2 || mode == 3);
  const bool misdetect = (mode == 2 || mode == 4);
  const float false_rate = misdetect ? 0.3f : 0.0f;
  const float throttle = (mode == 3) ? 20.0f : 0.0f;
  const bool gate = (mode == 4);
  const BiasResult r = run(use_gyro, true, true, tg, ta, false_rate,
                           throttle, misdetect, true, gate, seed);
  std::printf("seed=%u mode=%d true_gyro_dps=%.2f true_accel_mg=%.1f "
              "est_gyro_dps=%.3f est_accel_mg=%.2f\n",
              seed, mode, tg, ta, r.gyro.norm() * 57.2958f,
              r.accel.norm() / 9.80665f * 1000.0f);
  return 0;
}
