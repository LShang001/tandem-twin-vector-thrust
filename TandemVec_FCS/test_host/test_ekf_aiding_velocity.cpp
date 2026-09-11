// ============================================================
//  Ekf15State 测速源辅助接口回归测试
//  覆盖（2026-08-21 室内光流速度融合配套改动）：
//   - MeasurementUpdateAidingVelocityDetailed：豁免 ZUPT 零速守卫
//     （悬停近零测量 + 大漂移时 ZUPT 接口硬拒、aiding 接口按 NIS 裁决）
//   - AIDING_VELOCITY_RESIDUAL_REJECT_MPS 宽残差门仍挡荒谬跳点
//   - ResetVelocityToExternal：速度重锚 + 速度协方差块抬升 + NaN 防御
//   - D 通道 1e3 噪声下垂直协方差不被无信息压缩
//  编译：g++ -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
//        -Iinclude -Isrc -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src \
//        test_host/test_ekf_aiding_velocity.cpp lib/units/src/convang*.cpp -o bin/aidvel.exe
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

static const Eigen::Vector3d kLla(31.2304 * M_PI / 180.0, 121.4737 * M_PI / 180.0, 5.0);
static const Eigen::Vector3f kGravity(0.0f, 0.0f, -9.79f);

static Ekf15State makeInit()
{
  Ekf15State ekf;
  ekf.Initialize(kGravity, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                 Eigen::Vector3f::Zero(), kLla);
  return ekf;
}

int main()
{
  // ---- T1 旧行为保持：静止 + 零速测量，ZUPT 接口正常融合 ----
  {
    Ekf15State ekf = makeInit();
    const bfs::MeasurementUpdateResult r =
        ekf.MeasurementUpdateVelocityDetailed(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
    CHECK("T1 ZUPT 零速测量正常融合", r.fused);
  }

  // ---- T2 旧行为保持：漂移 6 m/s 后，ZUPT 接口的零速守卫仍硬拒 ----
  {
    Ekf15State ekf = makeInit();
    ekf.ResetVelocityToExternal(Eigen::Vector3f(6.0f, 0.0f, 0.0f), 0.5f);
    const bfs::MeasurementUpdateResult r =
        ekf.MeasurementUpdateVelocityDetailed(Eigen::Vector3f::Zero(), 0.05f, 0.05f);
    CHECK("T2 ZUPT 守卫拒绝（近零测量+6m/s残差）", r.attempted && !r.fused);
    CHECK("T2 拒绝后速度态未被拉动", std::fabs(ekf.ned_vel_mps()(0) - 6.0f) < 1e-4f);
  }

  // ---- T3 aiding 豁免零速守卫：同一近零测量，P 足够大时按 NIS 融合 ----
  {
    Ekf15State ekf = makeInit();
    // std=3.0 → P_v=9：NIS = 36/(9+0.0225) ≈ 4.0 < 16.3，应融合
    ekf.ResetVelocityToExternal(Eigen::Vector3f(6.0f, 0.0f, 0.0f), 3.0f);
    const Eigen::Vector3f meas_hover(0.0f, 0.0f, 6.0f); // D=当前估计（垂直不可观惯例）
    const bfs::MeasurementUpdateResult r =
        ekf.MeasurementUpdateAidingVelocityDetailed(meas_hover, 0.15f, 1000.0f);
    CHECK("T3 aiding 近零测量按 NIS 融合（零速守卫豁免）", r.fused);
    // K≈0.9975，融合后 v_n 应被大幅拉向测量 0
    CHECK("T3 融合后速度收敛向测量", ekf.ned_vel_mps()(0) < 0.2f);
  }

  // ---- T4 aiding 宽残差门仍挡荒谬跳点（>30 m/s 残差硬拒） ----
  {
    Ekf15State ekf = makeInit();
    const bfs::MeasurementUpdateResult r =
        ekf.MeasurementUpdateAidingVelocityDetailed(
            Eigen::Vector3f(40.0f, 0.0f, 0.0f), 0.15f, 1000.0f);
    CHECK("T4 aiding 40m/s 荒谬跳点硬拒", r.attempted && !r.fused);
    CHECK("T4 拒绝后速度保持 0", ekf.ned_vel_mps().norm() < 1e-4f);
  }

  // ---- T5 ResetVelocityToExternal：速度重置 + P 块抬升 + NaN 防御 ----
  {
    Ekf15State ekf = makeInit();
    ekf.ResetVelocityToExternal(Eigen::Vector3f(6.0f, 0.0f, 0.0f), 0.5f);
    CHECK("T5 重锚速度写入", std::fabs(ekf.ned_vel_mps()(0) - 6.0f) < 1e-5f);
    const float p_vn = ekf.covariance_coeff_for_test(3, 3);
    CHECK("T5 速度协方差块=std^2", std::fabs(p_vn - 0.25f) / 0.25f < 0.01f);
    const float nan_val = std::nanf("");
    ekf.ResetVelocityToExternal(Eigen::Vector3f(nan_val, 0.0f, 0.0f), 0.5f);
    CHECK("T5 NaN 输入不动状态", std::fabs(ekf.ned_vel_mps()(0) - 6.0f) < 1e-5f);
  }

  // ---- T6 失锁→重锚→恢复闭环（固件 50 连续拒绝触发重锚的底层语义） ----
  {
    Ekf15State ekf = makeInit();
    // 失锁状态：漂移 6m/s 且 P 小（0.25），近零测量 NIS≈132 拒绝
    ekf.ResetVelocityToExternal(Eigen::Vector3f(6.0f, 0.0f, 0.0f), 0.5f);
    const Eigen::Vector3f meas_hover(0.0f, 0.0f, 6.0f);
    const bfs::MeasurementUpdateResult r1 =
        ekf.MeasurementUpdateAidingVelocityDetailed(meas_hover, 0.15f, 1000.0f);
    CHECK("T6 失锁时 NIS 拒绝", r1.attempted && !r1.fused);
    // 上层重锚：速度重置到测量（D 保持当前估计）
    ekf.ResetVelocityToExternal(meas_hover, 0.5f);
    // 同一测量源后续帧恢复融合
    const bfs::MeasurementUpdateResult r2 =
        ekf.MeasurementUpdateAidingVelocityDetailed(
            Eigen::Vector3f(0.1f, 0.0f, 6.0f), 0.15f, 1000.0f);
    CHECK("T6 重锚后恢复融合", r2.fused);
    CHECK("T6 重锚后速度收敛到测量邻域",
          ekf.ned_vel_mps()(0) > 0.05f && ekf.ned_vel_mps()(0) < 0.15f);
  }

  // ---- T7 D 通道 1e3 噪声：垂直协方差不被无信息压缩 ----
  {
    Ekf15State ekf = makeInit();
    const float p_d_before = ekf.covariance_coeff_for_test(5, 5);
    const float p_n_before = ekf.covariance_coeff_for_test(3, 3);
    const Eigen::Vector3f meas(0.3f, 0.0f, ekf.ned_vel_mps()(2));
    const bfs::MeasurementUpdateResult r =
        ekf.MeasurementUpdateAidingVelocityDetailed(meas, 0.15f, 1000.0f);
    CHECK("T7 NE 量测融合", r.fused);
    const float p_d_after = ekf.covariance_coeff_for_test(5, 5);
    const float p_n_after = ekf.covariance_coeff_for_test(3, 3);
    CHECK("T7 P(Vd) 相对变化 <1%（垂直近似不参与）",
          std::fabs(p_d_after - p_d_before) / p_d_before < 0.01f);
    CHECK("T7 P(Vn) 明显下降（NE 通道确实融合）",
          p_n_after < p_n_before * 0.99f);
  }

  std::printf("---- aiding velocity: %d pass, %d fail ----\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
