// 宿主机回归测试：静止检测器（状态机 + 置信度评分）
// 编译运行：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_static_detector.cpp -o test_host/bin/sd && ./test_host/bin/sd
//
// 验证 include/ins_static_detector.h 的迟滞状态机、置信度评分与异常输入防御。
#include "../include/ins_static_detector.h"

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

// 配置一组合理的静止检测阈值，覆盖长沙地区重力。
// FRD 机体系，静止比力模长 ≈ 9.80665 m/s²。
static void setupDetector(Icm45686StaticDetector &d)
{
  // 重力默认 9.80665；上下容差各约 ±0.5 m/s²。
  Icm45686StaticDetectorConfigureThresholds(d,
                                            9.3f, 10.3f,   // accel_norm [lower, upper]
                                            0.05f,         // gyro_norm 阈值 rad/s
                                            0.30f,         // accel_delta 阈值 m/s²
                                            0.02f);        // gyro_delta 阈值 rad/s
  d.enter_min_frames = 10;
  d.exit_min_frames = 10;
}

int main()
{
  const Eigen::Vector3f static_accel(0.0f, 0.0f, -9.80665f);
  const Eigen::Vector3f static_gyro(0.0f, 0.0f, 0.0f);

  // ---- 1. 连续静止帧后确认静止 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    for (int i = 0; i < 9; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    check(!d.confirmed_static, "9 帧 (< enter_min_frames) 不确认静止");
    Icm45686StaticDetectorUpdate(static_accel, static_gyro, d); // 第 10 帧
    check(d.confirmed_static, "第 10 帧 (= enter_min_frames) 确认静止");
    check(d.confidence > 0.0f && d.confidence <= 1.0f, "确认后置信度 ∈ (0,1]");
  }

  // ---- 2. 静止确认后置信度随驻留帧爬升 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    for (int i = 0; i < 10; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    const float conf_early = d.confidence;
    for (int i = 0; i < 100; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    check(d.confidence >= conf_early, "长静止后置信度不低于刚确认时");
    check(d.confirmed_static_frames > 10U, "confirmed_static_frames 持续累积");
  }

  // ---- 3. 运动状态不确认静止 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    const Eigen::Vector3f move_accel(3.0f, 0.0f, -9.80665f); // 大横向加速度
    const Eigen::Vector3f move_gyro(0.0f, 0.0f, 1.0f);       // 大角速度
    for (int i = 0; i < 30; ++i)
      Icm45686StaticDetectorUpdate(move_accel, move_gyro, d);
    check(!d.confirmed_static, "持续大机动不确认静止");
    check(d.confidence == 0.0f, "未确认静止时置信度为 0");
  }

  // ---- 4. 迟滞退出：需连续 exit_min_frames 帧才退出 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    for (int i = 0; i < 12; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    check(d.confirmed_static, "先建立静止确认");
    // 喂 9 帧运动 (exit_min_frames-1)，应仍保持确认 (迟滞)
    const Eigen::Vector3f move_accel(3.0f, 0.0f, -9.80665f);
    const Eigen::Vector3f move_gyro(0.0f, 0.0f, 1.0f);
    for (int i = 0; i < 9; ++i)
      Icm45686StaticDetectorUpdate(move_accel, move_gyro, d);
    check(d.confirmed_static, "9 帧运动 (< exit_min_frames) 迟滞保持确认");
    // 第 10 帧运动应退出
    Icm45686StaticDetectorUpdate(move_accel, move_gyro, d);
    check(!d.confirmed_static, "第 10 帧运动 (= exit_min_frames) 退出静止");
  }

  // ---- 5. 运动中插入单帧静止不立即确认（需连续）----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    const Eigen::Vector3f move_accel(3.0f, 0.0f, -9.80665f);
    const Eigen::Vector3f move_gyro(0.0f, 0.0f, 1.0f);
    // 交替运动/静止，永远凑不齐连续 enter_min_frames
    for (int i = 0; i < 40; ++i)
    {
      Icm45686StaticDetectorUpdate(move_accel, move_gyro, d);
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    }
    check(!d.confirmed_static, "运动/静止交替不满足连续帧门槛，不确认");
  }

  // ---- 6. Reset 清空状态 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    for (int i = 0; i < 20; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    check(d.confirmed_static, "Reset 前已确认");
    Icm45686StaticDetectorReset(d);
    check(!d.confirmed_static, "Reset 后 confirmed_static=false");
    check(d.confidence == 0.0f, "Reset 后 confidence=0");
    check(d.confirmed_static_frames == 0U, "Reset 后 frames=0");
    check(d.enter_candidate_frames == 0U, "Reset 后 enter 候选清零");
  }

  // ---- 7. NaN 输入不崩溃且不误确认 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    const float nanv = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector3f nan_accel(nanv, 0.0f, -9.80665f);
    for (int i = 0; i < 30; ++i)
      Icm45686StaticDetectorUpdate(nan_accel, static_gyro, d);
    check(!d.confirmed_static, "NaN 加速度不误确认静止");
    // 状态量本身保持有限（不产生 NaN 传播崩溃）
    check(std::isfinite(d.gyro_norm_radps), "NaN 输入后 gyro_norm 仍有限");
    check(std::isfinite(d.confidence), "NaN 输入后 confidence 仍有限");
  }

  // ---- 8. NaN 输入后恢复正常输入仍能确认（状态机可恢复）----
  {
    Icm45686StaticDetector d;
    setupDetector(d);
    const float nanv = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector3f nan_accel(nanv, 0.0f, 0.0f);
    for (int i = 0; i < 10; ++i)
      Icm45686StaticDetectorUpdate(nan_accel, static_gyro, d);
    // 恢复正常静止输入，应能重新进入候选并确认
    for (int i = 0; i < 12; ++i)
      Icm45686StaticDetectorUpdate(static_accel, static_gyro, d);
    check(d.confirmed_static, "NaN 干扰后恢复正常输入仍能确认静止");
  }

  // ---- 9. 边界：角速度恰等于阈值 (norm < thresh 严格小于) ----
  {
    Icm45686StaticDetector d;
    setupDetector(d); // gyro_thresh=0.05
    // norm 恰好 0.05，应判为不满足 (严格小于)
    const Eigen::Vector3f edge_gyro(0.0f, 0.0f, 0.05f);
    for (int i = 0; i < 20; ++i)
      Icm45686StaticDetectorUpdate(static_accel, edge_gyro, d);
    check(!d.confirmed_static, "角速度=阈值 (非严格小于) 不确认静止");
  }

  // ---- 10. 加速度模长越界不确认 ----
  {
    Icm45686StaticDetector d;
    setupDetector(d); // [9.3, 10.3]
    const Eigen::Vector3f weak_accel(0.0f, 0.0f, -5.0f); // 模长 5 < lower 9.3
    for (int i = 0; i < 20; ++i)
      Icm45686StaticDetectorUpdate(weak_accel, static_gyro, d);
    check(!d.confirmed_static, "加速度模长低于下界不确认静止");
  }

  if (g_fail_count == 0)
  {
    std::printf("\n=== 静止检测器测试全部通过 ===\n");
    return 0;
  }
  std::printf("\n=== 失败 %d 项 ===\n", g_fail_count);
  return 1;
}
