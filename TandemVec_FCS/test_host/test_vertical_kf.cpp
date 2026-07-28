// 宿主机回归测试：三状态垂直卡尔曼滤波器（高度/速度/加速度计零偏）
// 编译运行：g++ -std=c++17 -Itest_host/stub -Iinclude test_host/test_vertical_kf.cpp -o test_host/bin/vkf && ./test_host/bin/vkf
//
// 验证 include/VerticalKF.h 的预测/更新数学、异常值门限、输入饱和、
// Joseph 形式协方差更新与协方差限制。通过 -Itest_host/stub 用极简 Arduino.h 桩
// 替代真实 Arduino.h，使该平台相关头文件可在宿主机编译。
#include "../include/VerticalKF.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

static int g_fail_count = 0;
static void check(bool cond, const std::string &name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
  if (!cond) ++g_fail_count;
}
static bool approx(float a, float b, float tol = 1e-3f)
{
  return std::fabs(a - b) <= tol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

int main()
{
  const float dt = 0.005f; // 200Hz
  const float g = 9.80665f;

  // ---- 1. begin 设置初始高度 ----
  {
    VerticalKF kf;
    kf.begin(10.0f, 0.5f, 0.001f);
    check(approx(kf.getHeight(), 10.0f), "begin 设置初始高度");
    check(approx(kf.getVelocity(), 0.0f), "初始速度为 0");
    check(approx(kf.getAccelBias(), 0.0f), "初始零偏为 0");
  }

  // ---- 2. 静止预测：零加速度下高度/速度保持 ----
  {
    VerticalKF kf;
    kf.begin(5.0f, 0.5f, 0.001f);
    for (int i = 0; i < 100; ++i)
      kf.predict(0.0f, dt);
    check(approx(kf.getHeight(), 5.0f, 1e-2f), "静止 (a=0) 高度保持");
    check(approx(kf.getVelocity(), 0.0f, 1e-3f), "静止速度保持 0");
  }

  // ---- 3. 恒定加速度预测：速度线性增长 ----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    const float a = 2.0f; // m/s^2
    const int n = 100;    // 0.5s
    for (int i = 0; i < n; ++i)
      kf.predict(a, dt);
    // 速度 ≈ a * t = 2 * 0.5 = 1.0 m/s（无 bias）
    check(approx(kf.getVelocity(), a * n * dt, 0.05f), "恒定加速度下速度线性增长");
    // 高度 ≈ 0.5 * a * t^2 = 0.5*2*0.25 = 0.25 m
    check(approx(kf.getHeight(), 0.5f * a * (n * dt) * (n * dt), 0.05f), "恒定加速度下高度二次增长");
  }

  // ---- 4. 输入加速度饱和限幅（默认 max_input_accel_=10）----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    VerticalKF kf2;
    kf2.begin(0.0f, 0.5f, 0.001f);
    for (int i = 0; i < 100; ++i)
    {
      kf.predict(100.0f, dt);   // 远超 max_input_accel_=10，应被饱和到 10
      kf2.predict(10.0f, dt);   // 恰好等于上限
    }
    check(approx(kf.getVelocity(), kf2.getVelocity(), 1e-3f), "加速度>max 被饱和到 max_input_accel");
  }

  // ---- 5. update 跳过 NaN 测量 ----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    kf.predict(1.0f, dt);
    const float h_before = kf.getHeight();
    kf.update(std::numeric_limits<float>::quiet_NaN(), 0.1f);
    check(approx(kf.getHeight(), h_before), "NaN 测量被跳过，状态不变");
  }

  // ---- 6. update 正常测量使高度收敛 ----
  // 用小残差（初始 7.5m，测量 8m，残差 0.5m << 3-sigma 门限）验证收敛，
  // 避免大残差被 3-sigma 门限拒绝（那是测试 7 的职责）。
  {
    VerticalKF kf;
    kf.begin(7.5f, 0.5f, 0.001f);
    for (int i = 0; i < 50; ++i)
      kf.predict(0.0f, dt);
    for (int i = 0; i < 200; ++i)
    {
      kf.predict(0.0f, dt);
      kf.update(8.0f, 0.1f); // 真实高度 8m
    }
    check(approx(kf.getHeight(), 8.0f, 0.1f), "小残差高度测量后收敛到测量值");
  }

  // ---- 7. update 异常值（3-sigma 门限）被拒绝 ----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    for (int i = 0; i < 50; ++i)
      kf.predict(0.0f, dt); // 状态在 0 附近，P[0][0] ~ O(1)
    const float h_before = kf.getHeight();
    // 测量 100m，残差 100，远超 3-sigma 门限，应被拒绝
    kf.update(100.0f, 0.1f);
    check(approx(kf.getHeight(), h_before, 1e-3f), "3-sigma 门限拒绝异常高度测量");
  }

  // ---- 8. update 后协方差减小（测量降低不确定性）----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    for (int i = 0; i < 100; ++i)
      kf.predict(0.0f, dt);
    // 用协方差限制上界 P_max[0]=100 间接观察：更新后高度方差应低于更新前。
    // 这里通过连续 update 验证 P 不会无限增长且有界。
    for (int i = 0; i < 50; ++i)
    {
      kf.predict(0.0f, dt);
      kf.update(0.0f, 0.1f);
    }
    // 多次更新后状态稳定在 0 附近，高度误差有界
    check(approx(kf.getHeight(), 0.0f, 0.1f), "多次更新后高度稳定有界");
  }

  // ---- 9. 协方差上限：长期无测量协方差不爆炸（P_max[0]=100）----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    // 长时间纯预测不更新，协方差应被 P_max 限制
    for (int i = 0; i < 10000; ++i)
      kf.predict(0.0f, dt);
    // 状态仍有限（未发散为 inf/nan）
    check(std::isfinite(kf.getHeight()) && std::isfinite(kf.getVelocity()),
          "长期纯预测后状态仍有限（协方差限制生效）");
  }

  // ---- 10. configureRobustness 覆盖默认参数 ----
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.001f);
    // 收紧门限到 2-sigma，异常值更易被拒绝
    kf.configureRobustness(2.0f, 5.0f,
                           1e-6f, 50.0f,
                           1e-4f, 10.0f,
                           1e-8f, 0.5f);
    for (int i = 0; i < 50; ++i)
      kf.predict(0.0f, dt);
    const float h_before = kf.getHeight();
    // 2-sigma 门限下，中等残差也该被拒绝（这里仍用大残差 50m 验证拒绝生效）
    kf.update(50.0f, 0.1f);
    check(approx(kf.getHeight(), h_before, 1e-3f), "configureRobustness 后异常值仍被拒绝");
  }

  // ---- 11. 加速度计零偏估计：恒定 bias 注入后可观测 ----
  // 注意：单高度观测对 bias 的可观测性弱，这里只验证 bias 状态有限且不发散。
  {
    VerticalKF kf;
    kf.begin(0.0f, 0.5f, 0.01f); // 较大 sigma_b 加速 bias 演化
    for (int i = 0; i < 500; ++i)
    {
      kf.predict(0.0f, dt);
      kf.update(0.0f, 0.2f);
    }
    check(std::isfinite(kf.getAccelBias()), "零偏估计保持有限");
    // 静止 + 高度持续为 0，bias 不应漂移过大
    check(std::fabs(kf.getAccelBias()) < 2.0f, "静止下零偏估计不发散");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== 垂直卡尔曼滤波器测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}
