/**
 * @file    test_bias_temp_table.cpp
 * @module  固件/宿主回归测试（fw-tests：温度-零偏表插值）
 * @brief   两点温度表在中间温度做线性插值的最小回归
 * @details 被测：`include/ins_bias_temp_table.h`（`Reset()`/`AddPoint()`/`Interpolate()`）。
 *          输入：20 ℃ 与 30 ℃ 两个采点，陀螺 x 零偏 0.001→0.003 rad/s，
 *          加计 x 零偏 0.01→0.02 m/s²；
 *          处理：请求 25 ℃ 插值；
 *          输出：插值成功且 gb≈0.002、ab≈0.015 才返回 0。
 *          运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_bias_temp_table.cpp
 * @theory  两点线性插值：中点值等于两端算术平均（本测试的解析期望值来源）；
 *          容差 1e-5 约为 float 在 1e-2 量级的有效位下限。
 * @frame   机体系 FRD（三轴零偏）
 * @units   温度 ℃，陀螺零偏 rad/s，加计零偏 m/s²
 * @note    仅覆盖两点插值；单调性/越界外推行为由固件侧调用点保证，本文件不断言。
 * @test    T1 中点插值数值；T2 表项计数
 * @see     TandemVec_FCS/include/ins_bias_temp_table.h、test_bias_temp_pipeline.cpp
 */
#include <cstdio>
#include <cmath>
#include "ins_bias_temp_table.h"

int main()
{
  InsBiasTempTable t;
  t.Reset();
  t.AddPoint(20.0f, Eigen::Vector3f(0.001f, 0.0f, -0.0005f),
             Eigen::Vector3f(0.01f, 0.0f, 0.0f));
  t.AddPoint(30.0f, Eigen::Vector3f(0.003f, 0.0f, -0.0005f),
             Eigen::Vector3f(0.02f, 0.0f, 0.0f));
  Eigen::Vector3f gb, ab;
  const bool ok = t.Interpolate(25.0f, &gb, &ab);
  std::printf("n=%u ok=%d gb=%.5f ab=%.4f\n", t.size(), ok ? 1 : 0, gb(0), ab(0));
  return (ok && std::fabs(gb(0) - 0.002f) < 1e-5f &&
          std::fabs(ab(0) - 0.015f) < 1e-5f) ? 0 : 1;
}
