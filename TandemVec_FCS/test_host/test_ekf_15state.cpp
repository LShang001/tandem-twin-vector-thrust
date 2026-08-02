// ============================================================
//  Ekf15State 宿主回归测试（15 状态 INS/GNSS 融合 EKF）
//  覆盖：初始化、静态传播、GNSS 量测收敛、有效性门控、协方差
//  编译：g++ -std=c++17 -Iinclude -Ilib/navigation-main/src -Ilib/eigen/src \
//        test_host/test_ekf_15state.cpp -o test_host/bin/ekf15.exe
// ============================================================
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include "ekf_15_state.h"

using bfs::Ekf15State;

static int g_pass = 0, g_fail = 0;
#define CHECK(name, cond)                                        \
  do {                                                           \
    if (cond) { g_pass++; std::printf("[PASS] %s\n", name); }    \
    else { g_fail++; std::printf("[FAIL] %s\n", name); }         \
  } while (0)

// 上海附近测试坐标（rad, rad, m）
static const Eigen::Vector3d kLla(31.2304 * M_PI / 180.0, 121.4737 * M_PI / 180.0, 5.0);
static const Eigen::Vector3f kGravity(0.0f, 0.0f, -9.79f); // 静止水平时比力沿 -z_b；与 TandemVec_Config.h g=9.79 统一（项目唯一事实源）

static Ekf15State makeInit()
{
  Ekf15State ekf;
  ekf.Initialize(kGravity, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), kLla);
  return ekf;
}

int main()
{
  // ---- T1 初始化：姿态水平、速度零、位置正确 ----
  {
    Ekf15State ekf = makeInit();
    CHECK("T1 滚转 ≈ 0", std::fabs(ekf.roll_rad()) < 1e-3f);
    CHECK("T1 俯仰 ≈ 0", std::fabs(ekf.pitch_rad()) < 1e-3f);
    CHECK("T1 速度 = 0", ekf.ned_vel_mps().norm() < 1e-4f);
    CHECK("T1 纬度正确", std::fabs(ekf.lat_rad() - kLla(0)) < 1e-9);
    CHECK("T1 经度正确", std::fabs(ekf.lon_rad() - kLla(1)) < 1e-9);
    CHECK("T1 高度正确", std::fabs(ekf.alt_m() - kLla(2)) < 1e-3);
  }

  // ---- T2 静态传播 100 步（dt=5ms）：速度保持零、姿态保持、协方差有界对称 ----
  {
    Ekf15State ekf = makeInit();
    for (int i = 0; i < 100; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    CHECK("T2 静止速度保持 <0.02 m/s", ekf.ned_vel_mps().norm() < 0.02f);
    CHECK("T2 静止俯仰保持 <0.5°", std::fabs(ekf.pitch_rad()) < 0.5f * M_PI / 180.0f);
    // 协方差：对角线有限为正、对称性
    bool pos_diag = true, sym = true, finite = true;
    for (int r = 0; r < 15; ++r)
    {
      const float d = ekf.covariance_coeff_for_test(r, r);
      if (!(d > 0.0f) || !std::isfinite(d)) pos_diag = false;
      for (int c = 0; c < 15; ++c)
      {
        const float v = ekf.covariance_coeff_for_test(r, c);
        if (!std::isfinite(v)) finite = false;
        if (std::fabs(v - ekf.covariance_coeff_for_test(c, r)) > 1e-3f * std::fabs(v))
          sym = false;
      }
    }
    CHECK("T2 协方差对角线为正", pos_diag);
    CHECK("T2 协方差对称", sym);
    CHECK("T2 协方差有限", finite);
  }

  // ---- T3 静态传播 200 步：协方差增长有界（不爆炸） ----
  {
    Ekf15State ekf = makeInit();
    const float p0 = ekf.covariance_coeff_for_test(3, 3); // 速度北向方差初值
    for (int i = 0; i < 200; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const float p1 = ekf.covariance_coeff_for_test(3, 3);
    // 静止 1s 速度不确定性从 1.0 增长到约 13.7（13.7×，加速度计积分物理行为）；
    // 阈值取 50× 验证"有界不爆炸"（若 process noise 失配会指数发散远超此界）。
    CHECK("T3 静止协方差增长有界（<50×）", p1 < 50.0f * p0);
  }

  // ---- T4 GNSS 量测收敛：初始速度偏 5 m/s → 量测真值 → 收敛 ----
  {
    Ekf15State ekf = makeInit();
    // 先传播几步产生一点协方差
    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    // 注入 5 m/s 的初始速度偏差（直接 ResetPositionVelocityToGnss 模拟）
    ekf.ResetPositionVelocityToGnss(Eigen::Vector3f(5.0f, 0.0f, 0.0f), kLla);
    CHECK("T4 注入速度偏差 5 m/s", std::fabs(ekf.ned_vel_mps()(0) - 5.0f) < 0.01f);
    // 连续 GNSS 量测（真值 0 m/s）→ 收敛
    Eigen::Vector3f vel_true = Eigen::Vector3f::Zero();
    for (int i = 0; i < 50; ++i)
    {
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
      ekf.MeasurementUpdate(vel_true, kLla, 0.005f);
    }
    CHECK("T4 GNSS 量测收敛（速度误差 <0.2 m/s）", ekf.ned_vel_mps().norm() < 0.2f);
  }

  // ---- T5 有效性门控：NaN 输入被拒绝且不污染状态 ----
  {
    Ekf15State ekf = makeInit();
    const Eigen::Vector3f v_before = ekf.ned_vel_mps();
    const float nan_v = std::nanf("");
    ekf.TimeUpdate(kGravity, Eigen::Vector3f(nan_v, 0.0f, 0.0f), 0.005f);
    const Eigen::Vector3f v_after = ekf.ned_vel_mps();
    CHECK("T5 NaN 陀螺不污染速度状态", (v_after - v_before).norm() < 1e-6f);
    // NaN GNSS 量测 → input_valid=false
    auto res = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f(nan_v, 0.0f, 0.0f), kLla, 0.005f);
    CHECK("T5 NaN GNSS 量测被拒绝", !res.input_valid);
  }

  // ---- T6 静止辅助：速度零量测（ZUPT 路径）多次融合后保持零 ----
  {
    Ekf15State ekf = makeInit();
    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    ekf.MeasurementUpdateVelocity(Eigen::Vector3f::Zero(), 0.1f, 0.1f);
    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    CHECK("T6 ZUPT 后速度保持 <0.05 m/s", ekf.ned_vel_mps().norm() < 0.05f);
  }

  std::printf("\n=== 结果: %d 通过, %d 失败 ===\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}


