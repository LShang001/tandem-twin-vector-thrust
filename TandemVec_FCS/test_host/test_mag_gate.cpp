/**
 * @file    test_mag_gate.cpp
 * @module  固件/宿主回归测试（fw-tests：磁测量质量门控）
 * @brief   验证磁矢量进入 EKF 前的模长/倾角/残差/航迹/新鲜度/标定/油门门控
 * @details 被测：`include/mag_measurement_gate.h`。
 *          输入：站点参考场 + 合成的机体 FRD 磁样本（正常/模长异常/倾角异常/
 *          方位异常/航迹不一致/过期/未标定/高油门/NaN/序号重复）；
 *          处理：`MagMeasurementGate::Evaluate()`；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/eigen/src
 *                test_host/test_mag_gate.cpp -o bin/mg
 * @theory  模长门控检测硬/软铁残差与干扰；倾角门控检测垂直干扰；方位残差
 *          与 GNSS 航迹交叉作为航向一致性证据；NIS 是 EKF 内第二层。
 *          @ref REF-120 §9.9.1 p.323–324、REF-121 §7.3.1 p.255–259；
 *          notes/04-航向与磁辅助
 * @frame   参考场 NED（z 下），量测机体 FRD；水平时 C^n_b=I
 * @units   µT、rad、deg、µs、m/s、%
 * @note    本测试只锁门控逻辑，不代表真实磁干扰/标定质量。
 * @test    G1 正常样本；G2 参考/标定/非有限/序号/过期；G3 模长；
 *          G4 倾角；G5 方位残差；G6 GNSS 航迹；G7 姿态不确定度/油门；
 *          G8 micros 回绕；G9 拒绝原因名称
 * @see     TandemVec_FCS/include/mag_measurement_gate.h
 */
#include <cmath>
#include <cstdio>
#include <string>
#include <Eigen/Dense>

#include "mag_measurement_gate.h"

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

static Eigen::Vector3f refField()
{
  return Eigen::Vector3f(bfs::kDefaultMagReference.field_ut[0],
                         bfs::kDefaultMagReference.field_ut[1],
                         bfs::kDefaultMagReference.field_ut[2]);
}

static Eigen::Vector3f rotateZ(const Eigen::Vector3f &v, const float angle_rad)
{
  const float c = std::cos(angle_rad);
  const float s = std::sin(angle_rad);
  return Eigen::Vector3f(c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z());
}

static bfs::MagGateInput baseInput()
{
  bfs::MagGateInput in;
  in.mag_body_ut = refField();
  in.sample_timestamp_us = 990000U;
  in.now_us = 1000000U;
  in.sample_seq = 7U;
  in.last_consumed_seq = 6U;
  in.cal_valid = true;
  in.ref = &bfs::kDefaultMagReference;
  in.c_n_b = Eigen::Matrix3f::Identity();
  in.gnss_course_valid = false;
  in.gnss_course_rad = 0.0f;
  in.estimated_yaw_rad = 0.0f;
  in.speed_mps = 0.0f;
  in.tilt_sigma_rad = 0.0f;
  in.throttle_pct = 20.0f;
  return in;
}

int main()
{
  const bfs::MagMeasurementGate gate;

  // G1 正常样本：模长/倾角/方位与参考场一致。
  {
    const bfs::MagGateResult r = gate.Evaluate(baseInput());
    CHECK(r.pass, "G1 正常磁样本通过");
    CHECK(r.reason == bfs::MagGateReason::None, "G1 拒绝原因为 None");
    CHECK(r.noise_ut >= gate.config().base_noise_ut &&
              r.noise_ut <= gate.config().max_noise_ut,
          "G1 自适应噪声在范围内");
    CHECK(std::fabs(r.norm_ut - bfs::kDefaultMagReference.total_ut) < 0.1f,
          "G1 模长≈参考总场");
    CHECK(std::fabs(r.heading_error_rad) < 1e-4f, "G1 方位误差≈0");
  }

  // G2 参考场/标定/非有限/序号/过期。
  {
    bfs::MagGateInput in = baseInput();
    in.ref = nullptr;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::RefInvalid,
          "G2 无参考场拒绝");

    bfs::MagReferenceNed bad_ref = bfs::kDefaultMagReference;
    bad_ref.valid = false;
    in = baseInput();
    in.ref = &bad_ref;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::RefInvalid,
          "G2 参考场 invalid 拒绝");

    in = baseInput();
    in.cal_valid = false;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::CalInvalid,
          "G2 未标定拒绝");

    in = baseInput();
    in.mag_body_ut.x() = std::nanf("");
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::NonFinite,
          "G2 NaN 拒绝");

    in = baseInput();
    in.last_consumed_seq = in.sample_seq;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::NoNewSample,
          "G2 重复序号拒绝");

    in = baseInput();
    in.sample_timestamp_us = in.now_us - 300000U;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::Stale,
          "G2 过期样本拒绝");

    in = baseInput();
    in.sample_timestamp_us = 0U;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::Stale,
          "G2 无时间戳拒绝");
  }

  // G3 模长门控。
  {
    bfs::MagGateInput in = baseInput();
    in.mag_body_ut = refField() * 0.2f;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::NormLow,
          "G3 模长过低拒绝");

    in = baseInput();
    in.mag_body_ut = refField() * 2.5f;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::NormHigh,
          "G3 模长过高拒绝");

    in = baseInput();
    in.mag_body_ut = refField().normalized() *
                     (bfs::kDefaultMagReference.total_ut + 10.0f);
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::NormDeviation,
          "G3 模长偏离参考拒绝");
  }

  // G4 倾角门控：保持总场模长，去掉垂直分量。
  {
    bfs::MagGateInput in = baseInput();
    in.mag_body_ut = Eigen::Vector3f(bfs::kDefaultMagReference.total_ut, 0.0f,
                                     0.0f);
    const bfs::MagGateResult r = gate.Evaluate(in);
    CHECK(r.reason == bfs::MagGateReason::DipDeviation, "G4 倾角异常拒绝");
  }

  // G5 方位残差：同模长/同倾角，水平旋转 60°。
  {
    bfs::MagGateInput in = baseInput();
    in.mag_body_ut = rotateZ(refField(), 60.0f * bfs::kMagReferenceDegToRad);
    const bfs::MagGateResult r = gate.Evaluate(in);
    CHECK(r.reason == bfs::MagGateReason::Residual, "G5 60°方位跳变拒绝");
  }

  // G6 GNSS 航迹交叉：健康磁场但航迹差 90°。
  {
    bfs::MagGateInput in = baseInput();
    in.gnss_course_valid = true;
    in.gnss_course_rad = 90.0f * bfs::kMagReferenceDegToRad;
    in.estimated_yaw_rad = 0.0f;
    in.speed_mps = 5.0f;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::TrackMismatch,
          "G6 磁/航迹 90°不一致拒绝");

    in = baseInput();
    in.gnss_course_valid = true;
    in.gnss_course_rad = 5.0f * bfs::kMagReferenceDegToRad;
    in.estimated_yaw_rad = 0.0f;
    in.speed_mps = 5.0f;
    CHECK(gate.Evaluate(in).pass, "G6 磁/航迹 5°一致通过");
  }

  // G7 姿态不确定度与高油门。
  {
    bfs::MagGateInput in = baseInput();
    in.tilt_sigma_rad = 10.0f * bfs::kMagReferenceDegToRad;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::TiltUncertain,
          "G7 姿态 1σ>8°拒绝");

    in = baseInput();
    in.throttle_pct = 90.0f;
    CHECK(gate.Evaluate(in).reason == bfs::MagGateReason::ThrottleHigh,
          "G7 高油门跳过");
  }

  // G8 micros 回绕：样本在 0xFFFFFF00，当前 0x100，年龄 512us。
  {
    bfs::MagGateInput in = baseInput();
    in.sample_timestamp_us = 0xFFFFFF00U;
    in.now_us = 0x100U;
    CHECK(gate.Evaluate(in).pass, "G8 micros 回绕年龄计算通过");
  }

  // G9 原因名称非空且不重复。
  {
    CHECK(bfs::magGateReasonName(bfs::MagGateReason::None) != nullptr,
          "G9 原因名称非空");
    CHECK(std::string(bfs::magGateReasonName(bfs::MagGateReason::Residual)) ==
              "residual",
          "G9 residual 名称");
  }

  // G10 倾斜补偿航向：用已知 yaw/pitch/roll 合成机体磁矢量，反解应回原 yaw。
  {
    const Eigen::Vector3f m_ned(
        bfs::kDefaultMagReference.field_ut[0],
        bfs::kDefaultMagReference.field_ut[1],
        bfs::kDefaultMagReference.field_ut[2]);
    const float cases[][3] = {
        {30.0f, -20.0f, 10.0f},
        {0.0f, -20.0f, 10.0f},
        {-45.0f, 15.0f, 5.0f},
        {170.0f, 25.0f, -30.0f}};
    for (const auto &c : cases)
    {
      const float yaw = c[0] * 0.017453292f;
      const float pitch = c[1] * 0.017453292f;
      const float roll = c[2] * 0.017453292f;
      const float cr = std::cos(roll), sr = std::sin(roll);
      const float cp = std::cos(pitch), sp = std::sin(pitch);
      const float cy = std::cos(yaw), sy = std::sin(yaw);
      Eigen::Matrix3f rx;
      rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
      Eigen::Matrix3f ry;
      ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
      Eigen::Matrix3f rz;
      rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
      const Eigen::Vector3f mag_body = (rx * ry * rz).transpose() * m_ned;
      const float heading =
          bfs::magTiltCompensatedHeadingRad(mag_body, roll, pitch, m_ned);
      CHECK(std::fabs(bfs::magWrapPi(heading - yaw)) < 1e-3f,
            "G10 倾斜补偿航向回解");
    }
  }

  if (g_fail == 0)
  {
    std::printf("=== test_mag_gate: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_mag_gate: %d 项失败 ===\n", g_fail);
  return 1;
}
