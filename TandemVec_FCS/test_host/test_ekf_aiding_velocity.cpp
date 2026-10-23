/**
 * @file    test_ekf_aiding_velocity.cpp
 * @module  固件/宿主回归测试（fw-tests：外部测速源辅助的速度融合接口）
 * @brief   验证 aiding 速度更新豁免 ZUPT 零速守卫、宽残差门与失锁重锚语义
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h` 的
 *          `MeasurementUpdateAidingVelocityDetailed()`、`ResetVelocityToExternal()`
 *          与既有 `MeasurementUpdateVelocityDetailed()`（ZUPT 路径，行为必须不变）。
 *          输入：近零速度量测 + 大速度漂移、30 m/s 级荒谬残差、1e3 噪声的 D 通道；
 *          处理：逐项检查 fused/rejected 标志与协方差变化；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：与 test_ekf_15state.cpp 相同（-DEKF_HOST_REGRESSION + convang 源），
 *          见 run_all.sh 的 aidvel 目标。
 * @theory  测速源（光流/空速）辅助的速度更新必须绕过“零速守卫”——守卫假设
 *          机体静止，而室内光流悬停时速度近零但 P 很大，若走 ZUPT 路径会被硬拒。
 *          改用 NIS 裁决 + 30 m/s 宽残差门，并在失锁时抬升速度协方差实现软重锚。
 *          @ref REF-121 §7.3.1 p.255（NIS 门控）、@ref REF-121 §8.2.3 p.291（静止辅助）
 * @frame   NED（速度量测），残差门按 3D 范数
 * @units   速度 m/s，协方差 m²/s²，噪声 1e3 为极端压力值
 * @note    30 m/s 残差门是“物理上限”而不是调参量：大于此值的残差必然来自
 *          时间戳错位或光流失锁，不能融合。T7 用 1e3 噪声验证“无信息量测不得
 *          压缩垂直协方差”，对应实机光流停流场景。
 * @test    T1 ZUPT 零速量测正常融合；T2 漂移 6 m/s 后 ZUPT 仍硬拒；
 *          T3 aiding 豁免零速守卫；T4 30 m/s 残差门；T5 ResetVelocityToExternal
 *          含 NaN 防御；T6 失锁→重锚→恢复闭环；T7 D 通道大噪声不压垂直协方差
 * @see     TandemVec_FCS/test_host/test_ekf_15state.cpp、@ref AGENTS.md（2026-08-21 光流审查轮）
 */
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
