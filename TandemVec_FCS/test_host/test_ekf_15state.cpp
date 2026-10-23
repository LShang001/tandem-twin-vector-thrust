/**
 * @file    test_ekf_15state.cpp
 * @module  固件/宿主回归测试（fw-tests：15 状态 INS/GNSS 融合 EKF）
 * @brief   验证 EKF 初始化、静态传播、GNSS/ZUPT 更新、NIS 门控与子集回退
 * @details 被测：`lib/navigation-main/src/ekf_15_state.h`（`bfs::Ekf15State`）的
 *          `Initialize`/`TimeUpdate`/`MeasurementUpdate*Detailed`/协方差接口。
 *          输入：静止比力 (0,0,-g) 与零角速率；GNSS 速度/位置真值与故障样本
 *          （5 m/s 速度阶跃、30 m 位置跳变）；上海附近 LLA；
 *          处理：按 200 Hz 步长传播、按 10 Hz 融合量测；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译（header-only + bfs::convang 系列 + 测试用快速协方差块）：
 *          g++ -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES
 *          -Iinclude -Isrc -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src
 *          test_host/test_ekf_15state.cpp lib/units/src/convang*.cpp -o bin/ekf15
 * @theory  15 状态误差卡尔曼：位置/速度/失准角/加计零偏/陀螺零偏，误差状态按体轴右乘
 *          `q_true = q_est ⊗ Exp(δβ)`。
 *          预测 @ref REF-121 §2.2.1 (2.2.4a) p.33；
 *          更新 @ref REF-121 §2.2.1 (2.2.4c-e) p.33；Joseph 形式 @ref REF-121 §2.2.4 p.42；
 *          速度增量与旋转效应 @ref REF-120 §9.6.1 (9.6.9)(9.6.17) p.298；
 *          NIS 门控 @ref REF-121 §7.3.1 p.255。
 * @frame   NED（导航系速度/位置）+ FRD（机体系比力/角速率）；误差状态在体系
 * @units   位置 m、速度 m/s、角度 rad、加计零偏 m/s²、陀螺零偏 rad/s、步长 s
 * @note    重力输入必须与 `TandemVec_Config.h` 的 g=9.79 一致（项目唯一事实源），
 *          不要改成 9.81。`-DEKF_HOST_REGRESSION` 会打开仅供宿主的测试接口，
 *          缺这个宏会链接失败。T7 的期望值来自解析推导（旋转效应不得重复计入），
 *          详见 @ref AGENTS.md（P0-0 修复）。
 * @test    T1 初始化姿态/速度/位置；T2 静态传播 100 步；T3 静态传播 200 步协方差有界；
 *          T4 GNSS 速度量测收敛；T5 NaN 输入被拒且不污染状态；T6 ZUPT 保持零速；
 *          T7 速度旋转效应不重复计入（P0-0）；T8 航向 consider（P1-5）；
 *          T9 速度子集坏/位置健康；T10 位置子集坏/速度健康；T11 双坏软重锚（P0-1）；
 *          T12 sub-frame 延迟量测；T13 长时间失联后连续不可信 GNSS 的硬重锚；
 *          T14 正常跟踪期 50 m GNSS 跳点的显式野值门；T15 机体系磁矢量更新；
 *          T16 真实磁初始化磁偏角偏置与 tilt_sigma 可用性
 * @see     TandemVec-EKF-Report/导航算法升级评估-专业文献调研-2026-10.md §5.1/§9.6
 */
#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include "ekf_15_state.h"
#include "mag_reference.h"

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

  // ---- T7 速度旋转效应：中点姿态已含 0.5Δθ，不应再加 0.5Δθ×Δv（P0-0） ----
  // 理论依据：秦永元《惯性导航（第三版）》，科学出版社 2020，§9.6.1，
  //   式(9.6.9)(9.6.17)(9.6.19)，p.298–299：速度更新用区间起始姿态
  //   C_b^n(m-1)，旋转补偿 ΔV_rot=½Δθ×ΔV 与之配套；改用中点姿态后不能再
  //   重复加该补偿项。
  {
    Ekf15State ekf = makeInit();
    const float dt = 0.005f;
    const Eigen::Vector3f omega(0.0f, 0.0f, 2.0f);
    const Eigen::Vector3f f(5.0f, 0.0f, -9.79f);
    const Eigen::Vector3f dtheta = omega * (0.5f * dt);
    const Eigen::Vector3f dv = f * (0.5f * dt);
    ekf.TimeUpdateTwoSample(dtheta, dtheta, dv, dv, dt);

    // 解析参考：C_b^n(mid) · (f·dt) + g·dt；水平/yaw 小角下 v_E ≈ 0.5·ω·f_x·dt²
    const Eigen::Matrix3f c_mid =
        Eigen::AngleAxisf(0.5f * omega.norm() * dt, omega.normalized())
            .toRotationMatrix();
    const Eigen::Vector3f expected =
        c_mid * (f * dt) + Eigen::Vector3f(0.0f, 0.0f, 9.79f * dt);
    const Eigen::Vector3f v = ekf.ned_vel_mps();
    const float err_e = std::fabs(v(1) - expected(1));
    std::printf("[INFO] T7 v_E=%.9f expected=%.9f err=%.9f\n",
                v(1), expected(1), err_e);
    // 容差依据：v_E 的解析增量 ≈ ½·ω·f_x·dt² ≈ 1.25e-4 m/s；若旋转效应被重复计入，
    // 误差会翻倍到 2.5e-4 量级，故 3e-5 门限足以区分“正确/双计”，
    // 又远大于 float 在本量级的舍入噪声（~1e-8）。v_N 同理取 1e-4。
    CHECK("T7 速度旋转效应不双计（v_E 误差 <3e-5）", err_e < 3.0e-5f);
    CHECK("T7 速度旋转效应不双计（v_N 误差 <1e-4）",
          std::fabs(v(0) - expected(0)) < 1.0e-4f);
  }

  // ---- T8 P1-5 航向 consider：GNSS 速度新息不修正航向 ----
  // 构造 P(8,3) 非零 + 5° 航向误差 + 1 m/s 速度新息；consider=true 时 K(8,:)=0，
  // 航向应保持不变；consider=false 时航向应被速度新息拉动。
  {
    const float yaw_err = 5.0f * static_cast<float>(M_PI) / 180.0f;
    const float p_vv = 0.04f;
    const float p_hh = yaw_err * yaw_err;

    auto make_case = [&](bool consider) {
      Ekf15State e = makeInit();
      e.set_covariance_for_test(3, 3, p_vv);
      e.set_covariance_for_test(8, 8, p_hh);
      e.set_covariance_for_test(8, 3, 0.5f * std::sqrt(p_vv * p_hh));
      Eigen::Quaternionf q = e.quat();
      q = q * Eigen::AngleAxisf(yaw_err, Eigen::Vector3f::UnitZ());
      e.set_quaternion_for_test(q);
      e.set_gnss_consider_heading(consider);
      const float yaw_before = e.yaw_rad();
      const auto r = e.MeasurementUpdateDetailed(
          Eigen::Vector3f(1.0f, 0.0f, 0.0f), kLla, 0.005f, 0.0f, false);
      const float d = e.yaw_rad() - yaw_before;
      const float d_wrapped = std::atan2(std::sin(d), std::cos(d));
      std::printf("[INFO] T8 consider=%d fused=%d dyaw=%.6f°\n",
                  consider ? 1 : 0, r.fused ? 1 : 0,
                  d_wrapped * 180.0f / static_cast<float>(M_PI));
      return d_wrapped;
    };

    const float dy_no = make_case(false);
    const float dy_yes = make_case(true);
    CHECK("T8 consider 保持航向（|Δψ|<1e-4 rad）",
          std::fabs(dy_yes) < 1.0e-4f);
    CHECK("T8 非 consider 航向会被速度新息拉动",
          std::fabs(dy_no) > std::fabs(dy_yes) + 1.0e-6f);
  }

  // ---- T9/T10/T11 P0-1：GNSS 3+3 子集回退与软重锚 ----
  // 理论依据：秦永元等《卡尔曼滤波与组合导航原理（第3版）》§7.3.1 p.255
  //   （系统级 χ² FDI）；§3.3 p.84–86（序贯处理）。
  {
    auto make_case = []() {
      Ekf15State e = makeInit();
      for (int i = 0; i < 3; ++i)
      {
        e.set_covariance_for_test(i, i, 0.04f);       // 位置 0.2 m 1σ
        e.set_covariance_for_test(3 + i, 3 + i, 0.01f); // 速度 0.1 m/s 1σ
      }
      return e;
    };

    // T9：速度 5 m/s 故障、位置健康 → 联合门拒绝，但位置子集应融合、速度子集拒绝
    {
      Ekf15State e = make_case();
      const auto r = e.MeasurementUpdateDetailed(
          Eigen::Vector3f(5.0f, 0.0f, 0.0f), kLla, 0.005f, 0.0f, false);
      std::printf("[INFO] T9 fused=%d sub_p=%d sub_v=%d nis_p=%.2f nis_v=%.2f\n",
                  r.fused ? 1 : 0, r.subset_position ? 1 : 0,
                  r.subset_velocity ? 1 : 0, r.nis_pos3, r.nis_vel3);
      CHECK("T9 联合门拒绝后位置子集融合", r.subset_position);
      CHECK("T9 速度子集被拒（不拉速度）", !r.subset_velocity &&
                                          e.ned_vel_mps().norm() < 0.2f);
    }

    // T10：位置 30 m 故障、速度健康 → 速度子集融合、位置子集拒绝
    {
      Ekf15State e = make_case();
      Eigen::Vector3d bad_lla = kLla;
      bad_lla(0) += 30.0 / 6378137.0;
      const auto r = e.MeasurementUpdateDetailed(
          Eigen::Vector3f::Zero(), bad_lla, 0.005f, 0.0f, false);
      std::printf("[INFO] T10 fused=%d sub_p=%d sub_v=%d nis_p=%.2f nis_v=%.2f\n",
                  r.fused ? 1 : 0, r.subset_position ? 1 : 0,
                  r.subset_velocity ? 1 : 0, r.nis_pos3, r.nis_vel3);
      CHECK("T10 位置子集被拒", !r.subset_position);
      CHECK("T10 速度子集融合", r.subset_velocity);
    }

    // T11：位置+速度同时坏，连续 10 次后触发 P_vv 软重锚
    {
      Ekf15State e = make_case();
      Eigen::Vector3d bad_lla = kLla;
      bad_lla(0) += 30.0 / 6378137.0;
      for (int i = 0; i < 10; ++i)
      {
        const auto r = e.MeasurementUpdateDetailed(
            Eigen::Vector3f(5.0f, 0.0f, 0.0f), bad_lla, 0.005f, 0.0f, false);
        CHECK("T11 双坏帧不融合", !r.fused);
      }
      std::printf("[INFO] T11 reanchors=%u\n",
                  static_cast<unsigned>(e.gnss_velocity_reanchor_count()));
      CHECK("T11 连续拒绝触发软重锚", e.gnss_velocity_reanchor_count() > 0U);
    }
  }

  // ---- T12 sub-frame 延迟量测：age < 首帧 dt/2 仍应命中最新快照并融合 ----
  // 回归：GetDelayedSnapshotIndexByAge 曾在 delay < 0.5*dt 时 best_idx=-1
  // 返回 false，导致 require_delayed_snapshot=true 的 GNSS 更新全部被拒。
  {
    Ekf15State ekf = makeInit();
    for (int i = 0; i < 20; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto r1 = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f::Zero(), kLla, 0.005f, 0.002f, true);
    CHECK("T12 sub-frame age=2ms 延迟 GNSS 融合", r1.fused);
    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto r2 = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f::Zero(), kLla, 0.005f, 0.004f, true);
    CHECK("T12 sub-frame age=4ms 延迟 GNSS 融合", r2.fused);
  }

  // ---- T13 长时间失联 + 连续不可信 GNSS：第 3 帧应硬重锚位置/速度 ----
  // 场景：5 m/s 速度偏差纯惯导 7 s，位置漂出 ~35 m（超过 25 m 重捕获残差门），
  // 之后连续 3 帧 GNSS 都在真实位置。前两帧拒绝，第三帧应重锚并恢复融合。
  {
    Ekf15State ekf = makeInit();
    ekf.ResetPositionVelocityToGnss(Eigen::Vector3f(5.0f, 0.0f, 0.0f), kLla);
    for (int i = 0; i < 1400; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const double drift_m = bfs::lla2ned(ekf.lla_rad_m(), kLla,
                                        bfs::AngPosUnit::RAD).norm();
    CHECK("T13 失联漂移已超过重捕获残差门", drift_m > 25.0);

    Eigen::Vector3f vel_true = Eigen::Vector3f::Zero();
    const auto r1 = ekf.MeasurementUpdateDetailed(
        vel_true, kLla, 0.005f, 0.0f, true);
    for (int i = 0; i < 20; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto r2 = ekf.MeasurementUpdateDetailed(
        vel_true, kLla, 0.005f, 0.0f, true);
    CHECK("T13 第 1 帧不可信 GNSS 被拒", !r1.fused);
    CHECK("T13 第 2 帧不可信 GNSS 被拒", !r2.fused);
    CHECK("T13 前两帧未触发重锚",
          ekf.gnss_reacquire_reset_count() == 0U);

    for (int i = 0; i < 20; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto r3 = ekf.MeasurementUpdateDetailed(
        vel_true, kLla, 0.005f, 0.0f, true);
    CHECK("T13 第 3 帧触发硬重锚", r3.fused && r3.reacquired);
    const double pos_err_m = bfs::lla2ned(ekf.lla_rad_m(), kLla,
                                          bfs::AngPosUnit::RAD).norm();
    CHECK("T13 重锚后位置回到 GNSS（<1 m）", pos_err_m < 1.0);
    CHECK("T13 重锚后速度回到 GNSS（<0.01 m/s）",
          ekf.ned_vel_mps().norm() < 0.01f);
    CHECK("T13 重锚计数 = 1", ekf.gnss_reacquire_reset_count() == 1U);
    CHECK("T13 重锚抬升位置协方差",
          ekf.covariance_coeff_for_test(0, 0) > 30.0f);
    CHECK("T13 重锚抬升速度协方差",
          ekf.covariance_coeff_for_test(3, 3) > 0.5f);

    // 重锚后下一帧 GNSS 应回到普通融合路径，不再重复重锚。
    for (int i = 0; i < 20; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto r4 = ekf.MeasurementUpdateDetailed(
        vel_true, kLla, 0.005f, 0.0f, true);
    CHECK("T13 重锚后 GNSS 恢复正常融合", r4.fused && !r4.reacquired);
  }

  // ---- T13b 相邻帧速度互相矛盾（跳点串）不得触发硬重锚 ----
  {
    Ekf15State ekf = makeInit();
    ekf.ResetPositionVelocityToGnss(Eigen::Vector3f(5.0f, 0.0f, 0.0f), kLla);
    for (int i = 0; i < 1400; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);

    bool any_fused = false;
    for (int i = 0; i < 5; ++i)
    {
      const Eigen::Vector3f jumpy_vel =
          (i % 2 == 0) ? Eigen::Vector3f::Zero()
                       : Eigen::Vector3f(50.0f, 0.0f, 0.0f);
      const auto r = ekf.MeasurementUpdateDetailed(
          jumpy_vel, kLla, 0.005f, 0.0f, true);
      any_fused = any_fused || r.fused;
      for (int k = 0; k < 20; ++k)
        ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    }
    CHECK("T13b 矛盾跳点串始终不融合", !any_fused);
    CHECK("T13b 矛盾跳点串不触发重锚",
          ekf.gnss_reacquire_reset_count() == 0U);
  }

  // ---- T14 正常跟踪期 50 m GNSS 位置跳点：显式野值门必须拒绝 ----
  // 回归：高振动/高延迟下 P 膨胀会让 50 m 跳点通过 NIS，滤波锁定到错误
  // 位置后反过来拒绝好帧；单帧大跳点必须在进入 NIS 前被物理门拒绝。
  {
    Ekf15State ekf = makeInit();
    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto good = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f::Zero(), kLla, 0.005f, 0.0f, true);
    CHECK("T14 正常 GNSS 帧融合", good.fused);

    const Eigen::Vector3d jump_lla = bfs::ned2lla(
        Eigen::Vector3d(50.0, 0.0, 0.0), kLla, bfs::AngPosUnit::RAD);
    const auto bad = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f::Zero(), jump_lla, 0.005f, 0.0f, true);
    CHECK("T14 50 m 跳点被拒绝", !bad.fused);
    CHECK("T14 跳点标记为 outlier_rejected", bad.outlier_rejected);
    CHECK("T14 野值拒绝计数 = 1", ekf.gnss_outlier_reject_count() == 1U);
    const double pos_err_m = bfs::lla2ned(ekf.lla_rad_m(), kLla,
                                          bfs::AngPosUnit::RAD).norm();
    CHECK("T14 野值拒绝后位置未被拉动（<1 m）", pos_err_m < 1.0);

    const auto bad_vel = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f(20.0f, 0.0f, 0.0f), kLla, 0.005f, 0.0f, true);
    CHECK("T14 20 m/s 速度跳点被拒绝", !bad_vel.fused);
    CHECK("T14 速度跳点标记为 outlier_rejected", bad_vel.outlier_rejected);
    CHECK("T14 野值拒绝计数 = 2", ekf.gnss_outlier_reject_count() == 2U);

    for (int i = 0; i < 10; ++i)
      ekf.TimeUpdate(kGravity, Eigen::Vector3f::Zero(), 0.005f);
    const auto good2 = ekf.MeasurementUpdateDetailed(
        Eigen::Vector3f::Zero(), kLla, 0.005f, 0.0f, true);
    CHECK("T14 野值后好帧仍能融合", good2.fused && !good2.outlier_rejected);
  }

  // ---- T15 三轴磁矢量更新：yaw 收敛、roll/pitch 不被拉偏、野值拒绝 ----
  {
    Ekf15State ekf = makeInit();
    ekf.set_quaternion_for_test(Eigen::Quaternionf::Identity());
    const Eigen::Vector3f mag_ned =
        Eigen::Vector3f(30.0f, 0.0f, 40.0f).normalized();
    const float yaw_true = 0.15f;
    const Eigen::Quaternionf q_true(
        Eigen::AngleAxisf(yaw_true, Eigen::Vector3f::UnitZ()));
    const Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;

    bool all_fused = true;
    for (int i = 0; i < 100; ++i)
    {
      const auto r = ekf.MeasurementUpdateMagVectorDetailed(
          mag_body, mag_ned, 0.5f);
      all_fused = all_fused && r.fused;
    }
    std::printf("[INFO] T15 yaw=%.4f roll=%.4f pitch=%.4f\n",
                ekf.yaw_rad(), ekf.roll_rad(), ekf.pitch_rad());
    CHECK("T15 磁矢量更新全部融合", all_fused);
    CHECK("T15 yaw 收敛到真值（±0.05 rad）",
          std::fabs(ekf.yaw_rad() - yaw_true) < 0.05f);
    CHECK("T15 roll 未被拉偏（<0.02 rad）", std::fabs(ekf.roll_rad()) < 0.02f);
    CHECK("T15 pitch 未被拉偏（<0.02 rad）", std::fabs(ekf.pitch_rad()) < 0.02f);
    CHECK("T15 融合计数 = 100", ekf.mag_vector_fused_count() == 100U);

    const Eigen::Vector3f mag_bad = Eigen::AngleAxisf(
        1.2f, Eigen::Vector3f::UnitZ()).toRotationMatrix() * mag_body;
    const Eigen::Vector3d pos_before(ekf.lat_rad(), ekf.lon_rad(),
                                     ekf.alt_m());
    const auto bad = ekf.MeasurementUpdateMagVectorDetailed(
        mag_bad, mag_ned, 0.05f);
    CHECK("T15 1.2 rad 磁跳变被 NIS 拒绝", !bad.fused);
    CHECK("T15 磁拒绝计数 = 1", ekf.mag_vector_reject_count() == 1U);
    const Eigen::Vector3d pos_after(ekf.lat_rad(), ekf.lon_rad(),
                                    ekf.alt_m());
    CHECK("T15 磁更新不影响位置", (pos_after - pos_before).norm() < 1e-4f);
  }

  // ---- T16 真实磁初始化：磁偏角 offset 把磁北航向转到真北 ----
  {
    const Eigen::Vector3f mag_body(
        bfs::kDefaultMagReference.field_ut[0],
        bfs::kDefaultMagReference.field_ut[1],
        bfs::kDefaultMagReference.field_ut[2]);
    Ekf15State with_offset;
    with_offset.Initialize(kGravity, Eigen::Vector3f::Zero(), mag_body,
                           Eigen::Vector3f::Zero(), kLla,
                           bfs::kDefaultMagReference.declination_rad);
    Ekf15State without_offset;
    without_offset.Initialize(kGravity, Eigen::Vector3f::Zero(), mag_body,
                              Eigen::Vector3f::Zero(), kLla);
    std::printf("[INFO] T16 yaw with offset=%.5f no offset=%.5f rad\n",
                with_offset.yaw_rad(), without_offset.yaw_rad());
    CHECK("T16 加磁偏角后真北航向≈0",
          std::fabs(with_offset.yaw_rad()) < 1e-4f);
    CHECK("T16 不加 offset 时航向≈-dec（磁北）",
          std::fabs(without_offset.yaw_rad() +
                    bfs::kDefaultMagReference.declination_rad) < 1e-4f);
    CHECK("T16 tilt_sigma 有限且非负",
          std::isfinite(with_offset.tilt_sigma_rad()) &&
              with_offset.tilt_sigma_rad() >= 0.0f);
  }

  std::printf("\n=== 结果: %d 通过, %d 失败 ===\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}


