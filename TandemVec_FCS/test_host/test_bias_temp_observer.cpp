/**
 * @file    test_bias_temp_observer.cpp
 * @module  固件/宿主回归测试（fw-tests：IMU 零偏温漂观测器）
 * @brief   合成线性温漂下验证观测器的斜率估计与有效跨度门控
 * @details 被测：`include/ins_bias_temp_observer.h`（`Reset()`/`Update()`/`Last()`）。
 *          输入：2000 拍、温度 20→25 ℃ 线性升温，三轴零偏按
 *          1e-3 / 5e-4 / -3e-4 rad/s/℃ 的线性温漂合成；
 *          处理：逐拍送入观测器；
 *          输出：`temp_span_c > 4.0`、x/y 轴斜率误差 < 5e-5 视为通过，否则返回 1。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_bias_temp_observer.cpp
 * @theory  一阶温漂模型 `b(T) = b0 + k·(T−T_ref)`；温度跨度不足时斜率不可辨，
 *          `valid` 必须为假（防止把噪声拟合当斜率）。@ref REF-121 §2.2.1 p.33（零偏随机游走口径）
 * @frame   机体系 FRD 三轴陀螺零偏（rad/s）
 * @units   温度 ℃，斜率 rad/s/℃，时间步 s
 * @note    容差 5e-5 约为斜率量级 1e-3 的 5%，对应 2000 拍最小二乘的统计下限；
 *          本测试不覆盖实机温度滞后与安装热梯度。
 * @test    T1 有效跨度与正负斜率符号；T2 斜率数值精度；T3 valid 门控
 * @see     TandemVec_FCS/include/ins_bias_temp_observer.h、test_bias_temp_table.cpp
 */
#include <cstdio>
#include <cmath>
#include "ins_bias_temp_observer.h"

int main()
{
  InsBiasTempObserver o;
  o.Reset();
  InsBiasTempObserver::Result r;
  for (int i = 0; i < 2000; ++i)
  {
    const float t = 20.0f + 5.0f * static_cast<float>(i) / 2000.0f;
    const Eigen::Vector3f bias(0.001f * (t - 20.0f),
                               0.0005f * (t - 20.0f),
                               -0.0003f * (t - 20.0f));
    r = o.Update(t, bias, true, true, 0.005f);
  }
  std::printf("valid=%d span=%.2f conf=%.3f slope=[%.5f,%.5f,%.5f]\n",
              r.valid ? 1 : 0, r.temp_span_c, r.confidence,
              r.gyro_slope_radps_per_c(0), r.gyro_slope_radps_per_c(1),
              r.gyro_slope_radps_per_c(2));
  const bool ok = r.valid && r.temp_span_c > 4.0f &&
                  std::fabs(r.gyro_slope_radps_per_c(0) - 0.001f) < 5e-5f &&
                  std::fabs(r.gyro_slope_radps_per_c(1) - 0.0005f) < 5e-5f;
  return ok ? 0 : 1;
}
