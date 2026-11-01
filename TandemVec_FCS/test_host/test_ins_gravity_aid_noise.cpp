/**
 * @file    test_ins_gravity_aid_noise.cpp
 * @module  固件/宿主测试（fw-tests：无 GNSS 运动段重力方向量测噪声）
 * @brief   验证 ins_gravity_aid_noise.h：静止噪声底、线加速度几何放大、
 *          单调性、上下限、非法输入 fail-safe、方向门限常量与门控判据。
 * @details 被测：`InsGravityAidNoiseRad(accel_norm, vib_rms_g, g, sigma_a)`；
 *          参照：`InsAhrsGravityDirectionAllowed`（15° 运动段门限）。
 *          输入为解析量（无随机数），结果可复现。
 * @theory  方向观测噪声 σ_dir = sqrt(σ_a² + a_lin²)/|f|，其中
 *          a_lin = sqrt(max(0, |f|² − g²))；振动按 σ_a 线性并入。
 *          @ref REF-120 §9.9.3 p.336（加速度计作重力观测的适用条件）
 * @frame   N/A（标量模长与机体系方向矢量）
 * @units   rad、m/s²、g
 * @note    g 取项目唯一事实源 9.79（TandemVec_Config.h），本测试显式传入，
 *          避免测试偷偷用 9.81 与固件不一致。
 * @test    T1–T8
 * @see     TandemVec_FCS/include/ins_gravity_aid_noise.h
 */
#include "ins_gravity_aid_noise.h"
#include "ins_ahrs_gravity_gate.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s\n", msg);                                            \
      ++g_fail;                                                                 \
    }                                                                           \
    else                                                                        \
    {                                                                           \
      std::printf("PASS: %s\n", msg);                                            \
    }                                                                           \
  } while (0)

static bool Near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

int main()
{
  const float g = 9.79f;  // TandemVec_Config.h kDefaultTandemVecParams.g
  const float sigma_a = 0.25f;

  // T1 静止：传感器噪声底 σ_a/|f| ≈ 0.0255 rad（1.46°）
  {
    const float n = InsGravityAidNoiseRad(g, 0.0f, g, sigma_a);
    CHECK(Near(n, sigma_a / g, 1e-4f), "T1 静止噪声底 = σ_a/|f|");
    CHECK(n > BFS_GRAVITY_AID_NOISE_MIN_RAD, "T1b 静止噪声高于地板");
  }

  // T2 1 m/s² 水平线加速度：a_lin=1 → σ = sqrt(σ_a²+1)/|f| ≈ 0.1047 rad（6.0°）
  {
    const float f = std::sqrt(g * g + 1.0f);
    const float expect = std::sqrt(sigma_a * sigma_a + 1.0f) / f;
    const float n = InsGravityAidNoiseRad(f, 0.0f, g, sigma_a);
    CHECK(Near(n, expect, 1e-4f), "T2 1 m/s² 线加速度：按几何上界放大");
  }

  // T3 单调性：线加速度 0→5 m/s²，噪声单调不减
  {
    float prev = -1.0f;
    bool mono = true;
    for (int i = 0; i <= 10; ++i)
    {
      const float a_lin = static_cast<float>(i) * 0.5f;
      const float f = std::sqrt(g * g + a_lin * a_lin);
      const float n = InsGravityAidNoiseRad(f, 0.0f, g, sigma_a);
      if (n < prev - 1e-6f)
      {
        mono = false;
      }
      prev = n;
    }
    CHECK(mono, "T3 噪声随线加速度单调不减");
  }

  // T4 振动主导：1 g 振动 RMS 使噪声达到上限（等价放弃本拍重力观测）
  {
    const float n = InsGravityAidNoiseRad(g, 1.0f, g, sigma_a);
    CHECK(Near(n, BFS_GRAVITY_AID_NOISE_MAX_RAD, 1e-6f),
          "T4 1 g 振动：钳到噪声上限");
  }

  // T5 零噪声输入：钳到地板而不是 0（0 噪声会让 K→1）
  {
    const float n = InsGravityAidNoiseRad(g, 0.0f, g, 0.0f);
    CHECK(Near(n, BFS_GRAVITY_AID_NOISE_MIN_RAD, 1e-6f), "T5 零传感器噪声钳到地板");
  }

  // T6 非法输入 fail-safe：|f|→0 与 NaN 都给上限，不产生 NaN
  {
    const float n0 = InsGravityAidNoiseRad(0.0f, 0.0f, g, sigma_a);
    CHECK(std::isfinite(n0) && Near(n0, BFS_GRAVITY_AID_NOISE_MAX_RAD, 1e-6f),
          "T6a |f|=0：给上限且有限");
    const float nn = InsGravityAidNoiseRad(std::nanf(""), 0.0f, g, sigma_a);
    CHECK(std::isfinite(nn) && Near(nn, BFS_GRAVITY_AID_NOISE_MAX_RAD, 1e-6f),
          "T6b |f|=NaN：给上限且有限");
  }

  // T7 常量合理性：15° 门限、地板 < 上限
  {
    const float deg15 = 15.0f * 0.0174532925f;
    CHECK(Near(BFS_MOTION_GRAVITY_GATE_RAD, deg15, 1e-5f),
          "T7a 运动段方向门限 = 15°");
    CHECK(BFS_GRAVITY_AID_NOISE_MIN_RAD < BFS_GRAVITY_AID_NOISE_MAX_RAD,
          "T7b 噪声地板 < 上限");
  }

  // T8 与方向门联用：15° 不一致时拒绝（悬停/巡航持续推力场景）
  {
    const float limit = BFS_MOTION_GRAVITY_GATE_RAD;
    const Eigen::Vector3f expected(0.0f, 0.0f, -1.0f);
    const Eigen::Vector3f same(0.0f, 0.0f, -g);
    const Eigen::Vector3f tilted(std::sin(25.0f * 0.0174532925f) * g, 0.0f,
                                 -std::cos(25.0f * 0.0174532925f) * g);
    CHECK(InsAhrsGravityDirectionAllowed(expected, same, limit),
          "T8a 比力方向一致：放行");
    CHECK(!InsAhrsGravityDirectionAllowed(expected, tilted, limit),
          "T8b 25° 不一致：拒绝（防止把推力/升力当重力）");
  }

  std::printf("\n%s\n", (g_fail == 0) ? "ALL PASS" : "FAILED");
  return (g_fail == 0) ? 0 : 1;
}
