/**
 * @file    test_static_detector.cpp
 * @module  固件/宿主回归测试（fw-tests：静止检测器状态机）
 * @brief   验证迟滞状态机、置信度爬升、异常输入防御与可恢复性
 * @details 被测：`include/ins_static_detector.h`
 *          （`Icm45686StaticDetectorUpdate()` 与配置函数）。
 *          输入：静止比力 (0,0,-g) 与零角速率、运动样本、NaN 输入；
 *          `setupDetector()` 配置一组覆盖长沙重力（9.80665）的阈值；
 *          处理：逐帧推进状态机（enter_min_frames/exit_min_frames 各 10 帧）；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude -Ilib/eigen/src test_host/test_static_detector.cpp
 * @theory  静止判据 = 比力模长贴近 g + 角速率近零 + 帧间增量小；进出各需连续
 *          N 帧构成迟滞，避免单帧噪声抖动导致 ZUPT 误开/误关。
 *          @ref REF-121 §8.2.3 p.291（静止检测/ZUPT 条件）
 * @frame   机体系 FRD（比力沿 -z_b，静止时模长 ≈ g）
 * @units   加速度 m/s²，角速率 rad/s，帧数无量纲
 * @note    阈值 9.3–10.3 m/s² 为 g±0.5、角速率 0.05 rad/s≈2.9 dps；
 *          NaN 用例验证的是“不崩溃且不误确认”，不是“NaN 被当静止”。
 *          该检测器是 ZUPT 的前置门，误判会直接污染 EKF 零偏估计。
 * @test    T1 连续静止帧确认；T2 置信度随驻留爬升；T3 运动不确认；
 *          T4 迟滞退出；T5 单帧静止不确认；T6 Reset 清状态；
 *          T7 NaN 不误确认；T8 NaN 后可恢复；T9 角速度恰等阈值的严格小于语义；
 *          T10 加速度模长越界不确认
 * @see     TandemVec_FCS/include/ins_static_detector.h、test_static_aid_profile.cpp
 */
// 宿主机回归测试：静止检测器（状态机 + 置信度评分）
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
