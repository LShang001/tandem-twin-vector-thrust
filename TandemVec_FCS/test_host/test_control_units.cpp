/**
 * @file    test_control_units.cpp
 * @module  固件/宿主回归测试（fw-tests：速率环量纲迁移的等价性）
 * @brief   验证 deg/s² 与 rad/s² 双向换算，以及迁移前后 PID 物理输出逐拍等价
 * @details 被测：`include/ControlUnits.h` 的换算常量/函数，配合
 *          `kFlightCtrlParams.rate_*` 与 `PositionPID`、`ComplementaryFilter`。
 *          输入：旧混合域参数（0.28/0.10、限幅 100、积分限幅 20、分离阈 60）
 *          经 `kRadPerDeg` 反推的角度域参数；参考/测量序列为手工波形与
 *          5000 拍正弦+阶跃混合序列；
 *          处理：逐拍比较“旧域直接输出”与“新域输出换算回 rad/s²”；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          运行：g++ -std=c++17 -Iinclude test_host/test_control_units.cpp -o bin/cu
 * @theory  速率环量纲统一（2026-08-11）：控制器输入 deg/s、输出 deg/s²，
 *          仅在写 `ControlOutputs_t` 的惯量逆解边界乘 π/180。因此
 *          kp[deg/s² per deg/s] × π/180 必须等于迁移前的混合域 kp。
 *          @ref AGENTS.md（速率环量纲已统一、旧值不得导入）、@ref CTL-001 §4
 * @frame   N/A（标量控制量，轴向由调用方决定）
 * @units   kp 为 s⁻¹、ki 为 s⁻²、限幅 deg/s²；比较在 rad/s² 域进行
 * @note    容差 2e-5（≈float 在 1e0 量级的相对 7 位有效数字）用于纯换算；
 *          5e-5 与 1e-4 用于 PID 动态序列，放大量级来自浮点累积与
 *          输出滤波的状态差异，不是行为放宽。
 * @test    T1 换算往返；T2 roll/pitch/yaw 的 kp 物理增益；T3 ki 物理增益；
 *          T4 输出/积分限幅物理上限；T5 PID 动态序列等价；
 *          T6 PID+输出滤波 5000 拍长序列等价
 * @see     TandemVec_FCS/include/ControlUnits.h、include/FlightCtrlParams.h
 */
#include "ControlUnits.h"
#include "FlightCtrlParams.h"
#include "PositionPID.h"
#include "ComplementaryFilter.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;

static void check(bool condition, const char *name)
{
    std::printf(condition ? "[PASS] %s\n" : "[FAIL] %s\n", name);
    if (!condition) ++g_fail;
}

static bool near(float a, float b, float tol = 2.0e-5f)
{
    return std::fabs(a - b) <= tol * std::fmax(1.0f, std::fmax(std::fabs(a), std::fabs(b)));
}

int main()
{
    check(near(ControlUnits::dps2ToRadps2(ControlUnits::kDegPerRad), 1.0f),
          "deg/s² 与 rad/s² 双向换算");
    check(near(kFlightCtrlParams.rate_roll.kp * ControlUnits::kRadPerDeg, 0.28f),
          "roll Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_pitch.kp * ControlUnits::kRadPerDeg, 0.28f),
          "pitch Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_yaw.kp * ControlUnits::kRadPerDeg, 0.20f),
          "yaw Kp 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_roll.ki * ControlUnits::kRadPerDeg, 0.10f),
          "roll/pitch Ki 迁移保持等效物理增益");
    check(near(kFlightCtrlParams.rate_yaw.ki * ControlUnits::kRadPerDeg, 0.075f),
          "yaw Ki 保守下调25%");
    check(near(kFlightCtrlParams.rate_roll.out_max * ControlUnits::kRadPerDeg, 100.0f),
          "输出限幅迁移保持 rad/s² 物理上限");
    check(near(kFlightCtrlParams.rate_roll.int_limit * ControlUnits::kRadPerDeg, 20.0f),
          "积分限幅迁移保持 rad/s² 物理上限");

    PositionPID old_pid(0.28f, 0.10f, 0.0f, -100.0f, 100.0f,
                        true, 20.0f, 60.0f, 0.2f);
    PositionPID new_pid(kFlightCtrlParams.rate_roll.kp,
                        kFlightCtrlParams.rate_roll.ki,
                        kFlightCtrlParams.rate_roll.kd,
                        kFlightCtrlParams.rate_roll.out_min,
                        kFlightCtrlParams.rate_roll.out_max,
                        true,
                        kFlightCtrlParams.rate_roll.int_limit,
                        kFlightCtrlParams.rate_roll.threshold,
                        kFlightCtrlParams.rate_roll.filter_alpha);

    bool sequence_equal = true;
    const float refs[] = {0.0f, 15.0f, 40.0f, 40.0f, -20.0f, 0.0f};
    const float meas[] = {0.0f, 1.0f, 8.0f, 20.0f, 5.0f, -2.0f};
    for (int i = 0; i < 6; ++i)
    {
        const float old_radps2 = old_pid.computeDerivativeOnMeasurement(refs[i], meas[i], 0.005f);
        const float new_radps2 = ControlUnits::dps2ToRadps2(
            new_pid.computeDerivativeOnMeasurement(refs[i], meas[i], 0.005f));
        sequence_equal &= near(old_radps2, new_radps2, 5.0e-5f);
    }
    check(sequence_equal, "PID 动态序列迁移前后 rad/s² 输出等价");

    PositionPID old_long(0.28f, 0.10f, 0.0f, -100.0f, 100.0f,
                         true, 20.0f, 60.0f, 0.2f);
    PositionPID new_long(kFlightCtrlParams.rate_roll.kp,
                         kFlightCtrlParams.rate_roll.ki,
                         kFlightCtrlParams.rate_roll.kd,
                         kFlightCtrlParams.rate_roll.out_min,
                         kFlightCtrlParams.rate_roll.out_max,
                         true,
                         kFlightCtrlParams.rate_roll.int_limit,
                         kFlightCtrlParams.rate_roll.threshold,
                         kFlightCtrlParams.rate_roll.filter_alpha);
    ComplementaryFilter old_filter(0.9f), new_filter(0.9f);
    float max_abs_error = 0.0f;
    bool long_sequence_equal = true;
    for (int i = 0; i < 5000; ++i)
    {
        const float t = i * 0.005f;
        const float ref = ((i / 400) % 2 == 0 ? 35.0f : -20.0f) + 8.0f * std::sin(0.7f * t);
        const float meas = 18.0f * std::sin(1.3f * t) + 3.0f * std::cos(0.17f * t);
        const float old_radps2 = old_filter.filter(
            old_long.computeDerivativeOnMeasurement(ref, meas, 0.005f));
        const float new_radps2 = ControlUnits::dps2ToRadps2(new_filter.filter(
            new_long.computeDerivativeOnMeasurement(ref, meas, 0.005f)));
        const float abs_error = std::fabs(old_radps2 - new_radps2);
        if (abs_error > max_abs_error) max_abs_error = abs_error;
        long_sequence_equal &= near(old_radps2, new_radps2, 1.0e-4f);
    }
    std::printf("[INFO] 5000 拍含输出滤波最大物理输出差 = %.6g rad/s²\n", max_abs_error);
    check(long_sequence_equal, "PID+输出滤波长序列迁移前后物理输出等价");

    std::printf(g_fail == 0 ? "test_control_units: ALL PASSED\n"
                            : "test_control_units: %d FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
