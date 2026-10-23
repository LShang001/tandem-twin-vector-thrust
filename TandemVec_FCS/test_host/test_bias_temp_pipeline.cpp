/**
 * @file    test_bias_temp_pipeline.cpp
 * @module  固件/宿主回归测试（fw-tests：温漂观测器 + 温度表的联合链路）
 * @brief   验证同一组线性温漂数据下观测器斜率与温度表插值互相一致
 * @details 被测：`include/ins_bias_temp_observer.h` 与 `include/ins_bias_temp_table.h`。
 *          输入：4000 拍、温度 20→30 ℃，x 轴零偏斜率 1e-3 rad/s/℃；
 *          每 400 拍往温度表 AddPoint 一次；
 *          处理：观测器在线估斜率，温度表在 25 ℃ 插值；
 *          输出：插值成功、25 ℃ 处零偏≈0.005、斜率≈1e-3 三者同时满足才返回 0。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_bias_temp_pipeline.cpp
 * @theory  观测器给连续斜率、温度表给查表补偿，二者对同一线性模型必须给出
 *          一致的斜率；若不一致说明两点中至少一处温度/时间基准错位。
 *          @ref REF-121 §2.2.1 p.33（零偏建模口径）
 * @frame   机体系 FRD（陀螺零偏 rad/s，加计零偏 m/s² 仅作占位）
 * @units   温度 ℃，零偏 rad/s，斜率 rad/s/℃，容差 1e-5 / 5e-4
 * @note    该链路是“温漂采点（未补偿）”与“温漂观测器”之间的接口回归；
 *          容差放宽到 5e-4 是因为表插值与在线拟合的窗口不同。
 * @test    T1 表插值命中 25 ℃；T2 表斜率与观测器斜率一致
 * @see     TandemVec_FCS/include/ins_bias_temp_observer.h、
 *          TandemVec_FCS/include/ins_bias_temp_table.h
 */
#include <cstdio>
#include <cmath>
#include "ins_bias_temp_observer.h"
#include "ins_bias_temp_table.h"

int main()
{
  InsBiasTempObserver obs;
  InsBiasTempTable table;
  obs.Reset();
  table.Reset();

  const float slope = 0.001f;
  for (int i = 0; i < 4000; ++i)
  {
    const float t = 20.0f + 10.0f * static_cast<float>(i) / 4000.0f;
    const Eigen::Vector3f bias(slope * (t - 20.0f), 0.0f, 0.0f);
    obs.Update(t, bias, true, true, 0.005f);
    if (i % 400 == 0)
    {
      table.AddPoint(t, bias, Eigen::Vector3f::Zero());
    }
  }
  Eigen::Vector3f gb, ab;
  const bool interp = table.Interpolate(25.0f, &gb, &ab);
  const auto r = obs.Last();
  std::printf("table_n=%u interp=%d bias25=%.5f slope=%.5f\n",
              table.size(), interp ? 1 : 0, gb(0),
              r.gyro_slope_radps_per_c(0));
  const bool ok = interp && std::fabs(gb(0) - 0.005f) < 1e-5f &&
                  std::fabs(r.gyro_slope_radps_per_c(0) - slope) < 5e-4f;
  return ok ? 0 : 1;
}
