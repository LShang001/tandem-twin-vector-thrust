/**
 * @file    test_power_allocation.cpp
 * @module  宿主机回归（制导/位置环 → 推力方向的纯函数层）
 * @brief   验证 include/PowerAllocation.h 与拆分前的数学逐位一致，并覆盖
 *          圆形加速度限幅、比力归一化、倾角限幅、退化输入与饱和位。
 * @details 参考实现按 2026-10-31 之前的 flight_control.cpp 内联代码逐字复刻；
 *          本文件只比较、不修改被测实现。
 */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "PowerAllocation.h"

namespace
{
using namespace bfs::power;

constexpr float kMaxAccel = 2.6f;
const float kMaxComp = std::sin(15.0f * 0.017453292519943295f);

int g_pass = 0;
int g_fail = 0;

void expect(const bool ok, const char *name)
{
  if (ok)
  {
    ++g_pass;
    return;
  }
  ++g_fail;
  std::printf("[FAIL] %s\n", name);
}

bool samef(const float a, const float b)
{
  // 允许 1 ULP 级差异：归一化/开方的浮点契约不承诺跨函数位级一致，
  // 但要求与拆分前数学等价（相对/绝对 ≤2e-6）。SIL 验收另行逐场景比对。
  if (std::isnan(a) || std::isnan(b))
  {
    return std::isnan(a) && std::isnan(b);
  }
  const float diff = std::fabs(a - b);
  return diff <= 2.0e-6f || diff <= 2.0e-6f * std::fmax(std::fabs(a), std::fabs(b));
}

// ---- 拆分前参考实现（逐字复刻，不得改写） ----
void ref_accel_clamp(float &an, float &ae, const float max_accel,
                     bool &saturated)
{
  const float accel_mag_sq = an * an + ae * ae;
  const float max_accel_sq = max_accel * max_accel;
  saturated = false;
  if (accel_mag_sq > max_accel_sq)
  {
    const float scale = max_accel / std::sqrt(accel_mag_sq);
    an *= scale;
    ae *= scale;
    saturated = true;
  }
}

void ref_components(const float an, const float ae, const float az_up,
                    const float gravity, const float min_norm, float &cn,
                    float &ce, float &norm)
{
  const float total_z = gravity + az_up;
  norm = std::sqrt(an * an + ae * ae + total_z * total_z);
  if (norm < min_norm)
  {
    norm = min_norm;
  }
  cn = an / norm;
  ce = ae / norm;
}

bool ref_tilt_clamp(float &cn, float &ce, const float max_comp)
{
  const float mag_sq = cn * cn + ce * ce;
  if (mag_sq > max_comp * max_comp)
  {
    const float scale = max_comp / std::sqrt(mag_sq);
    cn *= scale;
    ce *= scale;
    return true;
  }
  return false;
}

void test_stateless_math()
{
  // 确定性网格：覆盖限幅内/外、正负号、天向需求、重力边界。
  const float accels[] = {-3.0f, -2.6f, -1.3f, -0.05f, 0.0f,
                          0.05f, 1.3f, 2.6f, 3.0f};
  const float ups[] = {-8.0f, -3.0f, 0.0f, 2.0f, 6.0f};
  const float gravs[] = {9.79f, 9.806f, 9.81f};
  for (const float an0 : accels)
  {
    for (const float ae0 : accels)
    {
      for (const float up : ups)
      {
        for (const float g : gravs)
        {
          float an = an0;
          float ae = ae0;
          bool sat_ref = false;
          ref_accel_clamp(an, ae, kMaxAccel, sat_ref);
          float cn_ref = 0.0f;
          float ce_ref = 0.0f;
          float norm_ref = 0.0f;
          ref_components(an, ae, up, g, 0.5f, cn_ref, ce_ref, norm_ref);
          float cn_ref_tilt = cn_ref;
          float ce_ref_tilt = ce_ref;
          const bool tilt_ref = ref_tilt_clamp(cn_ref_tilt, ce_ref_tilt, kMaxComp);

          float cn = an0;
          float ce = ae0;
          bool sat = false;
          clampAccelCircle(cn, ce, kMaxAccel, sat);
          expect(sat == sat_ref, "accel sat flag");
          expect(samef(cn, an) && samef(ce, ae), "accel clamp value");

          float cno = 0.0f;
          float ceo = 0.0f;
          float normo = 0.0f;
          bool degen = false;
          thrustComponentsFromAccel(cn, ce, up, g, 0.5f, cno, ceo, normo,
                                    degen);
          expect(!degen, "components not degenerate");
          expect(samef(cno, cn_ref) && samef(ceo, ce_ref),
                 "components value");
          expect(samef(normo, norm_ref), "force norm value");

          const bool tilt = clampTiltComponents(cno, ceo, kMaxComp);
          const float comp_mag =
              std::sqrt(cn_ref_tilt * cn_ref_tilt + ce_ref_tilt * ce_ref_tilt);
          if (std::fabs(comp_mag - kMaxComp) > 1.0e-5f)
          {
            expect(tilt == tilt_ref, "tilt clamp flag");
          }
          expect(samef(cno, cn_ref_tilt) && samef(ceo, ce_ref_tilt),
                 "tilt clamp value");
        }
      }
    }
  }
}

void test_combined_entry()
{
  for (int i = -30; i <= 30; ++i)
  {
    for (int j = -30; j <= 30; ++j)
    {
      for (int k = -6; k <= 6; ++k)
      {
        const float an = 0.1f * static_cast<float>(i);
        const float ae = 0.1f * static_cast<float>(j);
        const float up = 1.0f * static_cast<float>(k);
        const float g = 9.80665f;
        PowerAllocationOutput out = allocatePowerFromAccel(
            an, ae, up, g, kMaxAccel, kMaxComp, 0.5f);

        float nan = an;
        float nae = ae;
        bool sat = false;
        ref_accel_clamp(nan, nae, kMaxAccel, sat);
        float cn = 0.0f;
        float ce = 0.0f;
        float norm = 0.0f;
        ref_components(nan, nae, up, g, 0.5f, cn, ce, norm);
        float cn_tilt = cn;
        float ce_tilt = ce;
        const bool tilt = ref_tilt_clamp(cn_tilt, ce_tilt, kMaxComp);

        expect(out.valid, "combined valid");
        expect(samef(out.thrust_comp_n, cn_tilt) &&
                   samef(out.thrust_comp_e, ce_tilt),
               "combined components");
        expect(samef(out.specific_force_mps2, norm), "combined norm");
        expect(samef(out.accel_n_mps2, nan) &&
                   samef(out.accel_e_mps2, nae),
               "combined clamped accel");
        const float accel_mag = std::sqrt(an * an + ae * ae);
        if (std::fabs(accel_mag - kMaxAccel) > 1.0e-5f)
        {
          expect(((out.sat_mask & kSatAccelLimit) != 0u) == sat,
                 "combined accel sat bit");
        }
        const float comp_mag = std::sqrt(cn * cn + ce * ce);
        if (std::fabs(comp_mag - kMaxComp) > 1.0e-5f)
        {
          expect(((out.sat_mask & kSatTiltLimit) != 0u) == tilt,
                 "combined tilt sat bit");
        }
      }
    }
  }
}

void test_guided_path()
{
  // GUIDED：min_force_norm=0、无滤波、先限幅再归一化再倾角限幅。
  for (int i = -20; i <= 20; ++i)
  {
    for (int j = -20; j <= 20; ++j)
    {
      const float an = 0.2f * static_cast<float>(i);
      const float ae = 0.2f * static_cast<float>(j);
      PowerAllocationOutput out = allocatePowerFromAccel(
          an, ae, 0.0f, 9.80665f, kMaxAccel, kMaxComp, 0.0f);
      float nan = an;
      float nae = ae;
      bool sat = false;
      ref_accel_clamp(nan, nae, kMaxAccel, sat);
      float cn = 0.0f;
      float ce = 0.0f;
      float norm = 0.0f;
      ref_components(nan, nae, 0.0f, 9.80665f, 0.0f, cn, ce, norm);
      ref_tilt_clamp(cn, ce, kMaxComp);
      expect(out.valid && samef(out.thrust_comp_n, cn) &&
                 samef(out.thrust_comp_e, ce),
             "guided components");
    }
  }
}

void test_degenerate_and_nonfinite()
{
  // 合力为零：限幅层不除零，退化时输出零分量并给出饱和位。
  PowerAllocationOutput zero =
      allocatePowerFromAccel(0.0f, 0.0f, 0.0f, 0.0f, kMaxAccel, kMaxComp, 0.5f);
  expect(!zero.valid, "zero force invalid");
  expect(zero.thrust_comp_n == 0.0f && zero.thrust_comp_e == 0.0f,
         "zero force zero comps");
  expect((zero.sat_mask & kSatDegenerate) != 0u, "zero force degenerate bit");

  // NaN 输入：不产生 NaN 输出（置退化位）。
  const float nan = std::numeric_limits<float>::quiet_NaN();
  PowerAllocationOutput bad =
      allocatePowerFromAccel(nan, 0.0f, 0.0f, 9.8f, kMaxAccel, kMaxComp, 0.5f);
  expect(!bad.valid, "nan invalid");
  expect(std::isfinite(bad.thrust_comp_n) && std::isfinite(bad.thrust_comp_e),
         "nan comps finite");
  expect((bad.sat_mask & kSatDegenerate) != 0u, "nan degenerate bit");

  // 恰好触发圆形限幅与倾角限幅。
  PowerAllocationOutput sat =
      allocatePowerFromAccel(10.0f, 10.0f, 0.0f, 9.806f, kMaxAccel, kMaxComp, 0.5f);
  expect((sat.sat_mask & kSatAccelLimit) != 0u, "accel sat bit");
  // 倾角饱和：把倾角权限压到 5°，确保分量限幅一定触发。
  PowerAllocationOutput sat_tilt =
      allocatePowerFromAccel(3.0f, 0.0f, 0.0f, 9.806f, kMaxAccel, 0.05f, 0.5f);
  expect((sat_tilt.sat_mask & kSatTiltLimit) != 0u, "tilt sat bit");
  const float mag = std::sqrt(sat_tilt.thrust_comp_n * sat_tilt.thrust_comp_n +
                              sat_tilt.thrust_comp_e * sat_tilt.thrust_comp_e);
  expect(mag <= 0.05f + 1.0e-6f, "tilt clamp magnitude");
}

void test_filter_order_contract()
{
  // AUTO_POSITION 的契约：分量滤波位于"归一化"与"倾角限幅"之间。
  // 本测试用一阶滤波复现调用方的顺序，确认与参考路径逐位一致。
  const float alpha = 0.6f;
  float an = 1.2f;
  float ae = -0.7f;
  bool sat = false;
  clampAccelCircle(an, ae, kMaxAccel, sat);
  float cn = 0.0f;
  float ce = 0.0f;
  float norm = 0.0f;
  bool degen = false;
  thrustComponentsFromAccel(an, ae, 0.4f, 9.806f, 0.5f, cn, ce, norm, degen);
  float fn = alpha * cn + (1.0f - alpha) * 0.0f;
  float fe = alpha * ce + (1.0f - alpha) * 0.0f;
  clampTiltComponents(fn, fe, kMaxComp);

  float ran = 1.2f;
  float rae = -0.7f;
  bool rsat = false;
  ref_accel_clamp(ran, rae, kMaxAccel, rsat);
  float rcn = 0.0f;
  float rce = 0.0f;
  float rnorm = 0.0f;
  ref_components(ran, rae, 0.4f, 9.806f, 0.5f, rcn, rce, rnorm);
  float rfn = alpha * rcn + (1.0f - alpha) * 0.0f;
  float rfe = alpha * rce + (1.0f - alpha) * 0.0f;
  ref_tilt_clamp(rfn, rfe, kMaxComp);
  expect(samef(fn, rfn) && samef(fe, rfe), "filtered order value");
}

}  // namespace

int main()
{
  test_stateless_math();
  test_combined_entry();
  test_guided_path();
  test_degenerate_and_nonfinite();
  test_filter_order_contract();
  std::printf("=== test_power_allocation: %d passed, %d failed ===\n", g_pass,
              g_fail);
  return g_fail == 0 ? 0 : 1;
}
